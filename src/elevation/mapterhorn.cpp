#include "providers.h"
#include "../net.h"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <curl/curl.h>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <mutex>
#include <numbers>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#ifdef FM_HAVE_WEBP
#include <webp/decode.h>
#endif

namespace arnis::elevation::providers
{
namespace
{
constexpr unsigned TILE_PIXELS = 512;
constexpr unsigned MAX_TILE_DOWNLOADS = 8;
constexpr unsigned RETRIES = 3;
constexpr unsigned MAX_ZOOM = 17;
constexpr unsigned MIN_ZOOM = 6;
constexpr std::uint64_t MISSING_MARKER_AGE_DAYS = 30;
constexpr std::size_t DOWNLOAD_CHUNK_SIZE = 64;
constexpr std::size_t PROBE_MIN_LEVEL_TILES = 64;
constexpr std::size_t PROBE_TILE_COUNT = 16;
constexpr std::size_t OUTAGE_FAILURE_THRESHOLD = 128;

struct NetworkResponse
{
	long status = 0;
	std::vector<std::uint8_t> bytes;
	CURLcode result = CURLE_FAILED_INIT;
};

enum class FetchStatus
{
	Hit,
	Missing,
	Failed
};

struct FetchResult
{
	XyzTileKey key;
	FetchStatus status = FetchStatus::Failed;
	bool new_missing = false;
	bool network_success = false;
};

std::uint64_t pack_key(const XyzTileKey &key)
{
	return (std::uint64_t(key.zoom) << 58) | (std::uint64_t(key.x) << 29) |
		   std::uint64_t(key.y);
}

std::filesystem::path tile_path(const std::filesystem::path &root, const XyzTileKey &key)
{
	return root / ("z" + std::to_string(key.zoom) + "_x" + std::to_string(key.x) + "_y" +
						  std::to_string(key.y) + ".webp");
}

std::filesystem::path missing_path(
		const std::filesystem::path &root, const XyzTileKey &key)
{
	return root / ("z" + std::to_string(key.zoom) + "_x" + std::to_string(key.x) + "_y" +
						  std::to_string(key.y) + ".missing");
}

XyzTileKey parent_key(const XyzTileKey &key)
{
	return {key.zoom - 1, key.x / 2, key.y / 2};
}

XyzTileKey tile_key_at(unsigned zoom, double norm_x, double norm_y)
{
	const auto count = std::int64_t{1} << zoom;
	const auto x = std::clamp<std::int64_t>(
			static_cast<std::int64_t>(std::floor(norm_x * count)), 0, count - 1);
	const auto y = std::clamp<std::int64_t>(
			static_cast<std::int64_t>(std::floor(norm_y * count)), 0, count - 1);
	return {zoom, static_cast<unsigned>(x), static_cast<unsigned>(y)};
}

double norm_y_for_lat(double latitude)
{
	return (1.0 - std::asinh(std::tan(latitude * std::numbers::pi / 180.0)) /
						   std::numbers::pi) *
		   0.5;
}

std::size_t receive_bytes(char *data, std::size_t size, std::size_t count, void *opaque)
{
	const auto bytes = size * count;
	auto &buffer = *static_cast<std::vector<std::uint8_t> *>(opaque);
	constexpr std::size_t MAX_RESPONSE_BYTES = 16 * 1024 * 1024;
	if (bytes > MAX_RESPONSE_BYTES - std::min(MAX_RESPONSE_BYTES, buffer.size()))
		return 0;
	buffer.insert(buffer.end(), reinterpret_cast<std::uint8_t *>(data),
			reinterpret_cast<std::uint8_t *>(data) + bytes);
	return bytes;
}

NetworkResponse http_get(const std::string &url)
{
	static std::once_flag curl_init;
	std::call_once(curl_init, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
	NetworkResponse response;
	CURL *curl = curl_easy_init();
	if (!curl)
		return response;
	curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
	curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 20L);
	curl_easy_setopt(curl, CURLOPT_TIMEOUT, 60L);
	curl_easy_setopt(curl, CURLOPT_USERAGENT, "Arnis-Cpp (+https://arnismc.com)");
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, receive_bytes);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response.bytes);
	response.result = curl_easy_perform(curl);
	curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response.status);
	curl_easy_cleanup(curl);
	return response;
}

bool decode_webp(const std::uint8_t *bytes, std::size_t size, RgbRaster &raster)
{
#ifdef FM_HAVE_WEBP
	int width = 0, height = 0;
	std::uint8_t *rgb = WebPDecodeRGB(bytes, size, &width, &height);
	if (!rgb || width <= 0 || height <= 0) {
		if (rgb)
			WebPFree(rgb);
		return false;
	}
	raster.width = static_cast<std::size_t>(width);
	raster.height = static_cast<std::size_t>(height);
	raster.pixels.resize(raster.width * raster.height);
	for (std::size_t i = 0; i < raster.pixels.size(); ++i)
		raster.pixels[i] = {rgb[i * 3], rgb[i * 3 + 1], rgb[i * 3 + 2]};
	WebPFree(rgb);
	return true;
#else
	(void)bytes;
	(void)size;
	(void)raster;
	return false;
#endif
}

bool read_webp(const std::filesystem::path &path, RgbRaster &raster)
{
	std::ifstream input(path, std::ios::binary);
	if (!input)
		return false;
	std::vector<std::uint8_t> bytes(
			(std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
	return !bytes.empty() && decode_webp(bytes.data(), bytes.size(), raster);
}

bool marker_is_fresh(const std::filesystem::path &path)
{
	std::error_code ec;
	const auto modified = std::filesystem::last_write_time(path, ec);
	if (ec)
		return false;
	const auto now = std::filesystem::file_time_type::clock::now();
	return now >= modified &&
		   now - modified < std::chrono::hours(24 * MISSING_MARKER_AGE_DAYS);
}

bool write_cache_file(
		const std::filesystem::path &path, const std::vector<std::uint8_t> &bytes)
{
	static std::atomic_uint64_t counter{0};
	std::error_code ec;
	std::filesystem::create_directories(path.parent_path(), ec);
	if (ec)
		return false;
	auto temporary = path;
	temporary +=
			".tmp-" +
			std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())) +
			"-" + std::to_string(counter.fetch_add(1));
	{
		std::ofstream output(temporary, std::ios::binary);
		if (!output)
			return false;
		output.write(reinterpret_cast<const char *>(bytes.data()),
				static_cast<std::streamsize>(bytes.size()));
		if (!output) {
			std::filesystem::remove(temporary, ec);
			return false;
		}
	}
	std::filesystem::rename(temporary, path, ec);
	if (!ec)
		return true;
	std::filesystem::remove(temporary, ec);
	return std::filesystem::is_regular_file(path, ec);
}

FetchResult fetch_tile(const std::filesystem::path &root, const XyzTileKey &key)
{
	FetchResult out{key};
	const auto marker = missing_path(root, key);
	std::error_code ec;
	if (std::filesystem::is_regular_file(marker, ec)) {
		if (marker_is_fresh(marker)) {
			out.status = FetchStatus::Missing;
			return out;
		}
		std::filesystem::remove(marker, ec);
	}
	const auto file = tile_path(root, key);
	if (std::filesystem::is_regular_file(file, ec)) {
		RgbRaster cached;
		if (read_webp(file, cached)) {
			out.status = FetchStatus::Hit;
			return out;
		}
		std::filesystem::remove(file, ec);
	}
	for (unsigned attempt = 0; attempt < RETRIES; ++attempt) {
		if (attempt)
			std::this_thread::sleep_for(std::chrono::milliseconds(500u << (attempt - 1)));
		[[maybe_unused]] auto permit = arnis::net::request_permit();
		const auto response = http_get(mapterhorn_tile_url(key));
		if (response.result != CURLE_OK) {
			continue;
		}
		if (response.status == 404) {
			out.status = FetchStatus::Missing;
			out.new_missing = true;
			return out;
		}
		if (response.status == 403 || response.status == 429)
			continue;
		if (response.status >= 400 && response.status < 500)
			return out;
		if (response.status < 200 || response.status >= 300)
			continue;
		RgbRaster decoded;
		if (!decode_webp(response.bytes.data(), response.bytes.size(), decoded))
			continue;
		out.network_success = true;
		out.status = write_cache_file(file, response.bytes) ? FetchStatus::Hit
															: FetchStatus::Failed;
		return out;
	}
	return out;
}

struct FetchState
{
	std::unordered_set<std::uint64_t> available;
	std::unordered_set<std::uint64_t> confirmed_missing;
	std::unordered_set<std::uint64_t> unreadable;
	std::vector<XyzTileKey> pending_markers;
	std::size_t failed = 0;
	std::size_t consecutive_failures = 0;
	bool saw_network_success = false;
};

bool fetch_level(const std::filesystem::path &root, const std::vector<XyzTileKey> &keys,
		FetchState &state, std::vector<XyzTileKey> &absent)
{
	for (std::size_t begin = 0; begin < keys.size(); begin += DOWNLOAD_CHUNK_SIZE) {
		const auto end = std::min(keys.size(), begin + DOWNLOAD_CHUNK_SIZE);
		std::vector<FetchResult> results(end - begin);
		std::atomic_size_t next{0};
		auto worker = [&] {
			for (;;) {
				const auto i = next.fetch_add(1, std::memory_order_relaxed);
				if (i >= results.size())
					return;
				results[i] = fetch_tile(root, keys[begin + i]);
			}
		};
		const auto worker_count =
				std::min<std::size_t>(MAX_TILE_DOWNLOADS, results.size());
		std::vector<std::thread> workers;
		workers.reserve(worker_count);
		for (std::size_t i = 0; i < worker_count; ++i)
			workers.emplace_back(worker);
		for (auto &thread : workers)
			thread.join();

		std::size_t failures = 0;
		for (const auto &result : results) {
			const auto packed = pack_key(result.key);
			state.saw_network_success |= result.network_success;
			if (result.status == FetchStatus::Hit) {
				state.available.insert(packed);
			} else {
				absent.push_back(result.key);
				if (result.status == FetchStatus::Missing) {
					state.confirmed_missing.insert(packed);
					if (result.new_missing)
						state.pending_markers.push_back(result.key);
				} else {
					++state.failed;
					++failures;
				}
			}
		}
		state.consecutive_failures =
				failures == results.size() ? state.consecutive_failures + failures : 0;
		if (state.consecutive_failures >= OUTAGE_FAILURE_THRESHOLD)
			return false;
	}
	return true;
}

double row_lat(const GeoBBox &bbox, std::size_t row, std::size_t height)
{
	return bbox.max_lat - double(row) / std::max<std::size_t>(1, height - 1) *
								  (bbox.max_lat - bbox.min_lat);
}

std::optional<double> sample_pixel(
		const std::unordered_map<std::uint64_t, RgbRaster> &tiles, const XyzTileKey &key,
		int px, int py)
{
	std::int64_t tx = key.x, ty = key.y;
	if (px < 0) {
		--tx;
		px += TILE_PIXELS;
	} else if (px >= static_cast<int>(TILE_PIXELS)) {
		++tx;
		px -= TILE_PIXELS;
	}
	if (py < 0) {
		--ty;
		py += TILE_PIXELS;
	} else if (py >= static_cast<int>(TILE_PIXELS)) {
		++ty;
		py -= TILE_PIXELS;
	}
	const auto count = std::int64_t{1} << key.zoom;
	if (tx < 0 || ty < 0 || tx >= count || ty >= count)
		return std::nullopt;
	const XyzTileKey neighbour{
			key.zoom, static_cast<unsigned>(tx), static_cast<unsigned>(ty)};
	const auto found = tiles.find(pack_key(neighbour));
	if (found == tiles.end() || std::size_t(px) >= found->second.width ||
			std::size_t(py) >= found->second.height)
		return std::nullopt;
	return terrarium_height(
			found->second
					.pixels[std::size_t(py) * found->second.width + std::size_t(px)]);
}

std::optional<double> sample_height(
		const std::unordered_map<std::uint64_t, RgbRaster> &tiles, unsigned top_zoom,
		double norm_x, double norm_y)
{
	const unsigned floor = std::min<unsigned>(MIN_ZOOM, top_zoom);
	for (int z = static_cast<int>(top_zoom); z >= static_cast<int>(floor); --z) {
		const auto zoom = static_cast<unsigned>(z);
		const auto key = tile_key_at(zoom, norm_x, norm_y);
		if (!tiles.contains(pack_key(key)))
			continue;
		const double n = std::ldexp(1.0, z);
		const double fx = norm_x * n * TILE_PIXELS;
		const double fy = norm_y * n * TILE_PIXELS;
		double px = fx - key.x * TILE_PIXELS;
		double py = fy - key.y * TILE_PIXELS;
		if (px >= TILE_PIXELS)
			px = TILE_PIXELS - 1.0;
		if (py >= TILE_PIXELS)
			py = TILE_PIXELS - 1.0;
		const int x0 = static_cast<int>(std::floor(px));
		const int y0 = static_cast<int>(std::floor(py));
		const double dx = std::clamp(px - x0, 0.0, 1.0);
		const double dy = std::clamp(py - y0, 0.0, 1.0);
		const auto v00 = sample_pixel(tiles, key, x0, y0);
		const auto v10 = sample_pixel(tiles, key, x0 + 1, y0);
		const auto v01 = sample_pixel(tiles, key, x0, y0 + 1);
		const auto v11 = sample_pixel(tiles, key, x0 + 1, y0 + 1);
		const double value = blend_finite_samples(
				v00.value_or(std::numeric_limits<double>::quiet_NaN()),
				v10.value_or(std::numeric_limits<double>::quiet_NaN()),
				v01.value_or(std::numeric_limits<double>::quiet_NaN()),
				v11.value_or(std::numeric_limits<double>::quiet_NaN()), dx, dy);
		if (std::isfinite(value))
			return value;
	}
	return std::nullopt;
}

void load_chunk_tiles(const std::filesystem::path &root, const GeoBBox &bbox,
		unsigned top_zoom, std::size_t row_start, std::size_t row_end,
		std::size_t grid_height, FetchState &state,
		std::unordered_map<std::uint64_t, RgbRaster> &tiles)
{
	const double top_y = norm_y_for_lat(row_lat(bbox, row_start, grid_height));
	const double bottom_y =
			norm_y_for_lat(row_lat(bbox, row_end ? row_end - 1 : row_end, grid_height));
	const unsigned floor = std::min<unsigned>(MIN_ZOOM, top_zoom);
	for (unsigned zoom = floor; zoom <= top_zoom; ++zoom) {
		const auto count = std::int64_t{1} << zoom;
		const auto x0 = tile_key_at(zoom, (bbox.min_lon + 180.0) / 360.0, top_y).x;
		const auto x1 = tile_key_at(zoom, (bbox.max_lon + 180.0) / 360.0, top_y).x;
		const auto y0 = tile_key_at(zoom, 0.0, top_y).y;
		const auto y1 = tile_key_at(zoom, 0.0, bottom_y).y;
		const auto min_x = std::min(x0, x1);
		const auto max_x = static_cast<unsigned>(
				std::min<std::int64_t>(count - 1, std::int64_t(std::max(x0, x1)) + 1));
		const auto min_y = std::min(y0, y1);
		const auto max_y = static_cast<unsigned>(
				std::min<std::int64_t>(count - 1, std::int64_t(std::max(y0, y1)) + 1));
		for (unsigned y = min_y; y <= max_y; ++y)
			for (unsigned x = min_x; x <= max_x; ++x) {
				const XyzTileKey key{zoom, x, y};
				const auto packed = pack_key(key);
				if (!state.available.contains(packed) ||
						state.unreadable.contains(packed))
					continue;
				RgbRaster raster;
				if (read_webp(tile_path(root, key), raster)) {
					tiles.emplace(packed, std::move(raster));
				} else {
					state.unreadable.insert(packed);
				}
			}
	}
}

} // namespace

std::optional<std::vector<std::vector<double>>> fetch_mapterhorn_terrain_grid(
		const std::filesystem::path &cache_root, const GeoBBox &bbox, std::size_t width,
		std::size_t height)
{
#ifndef FM_HAVE_WEBP
	(void)cache_root;
	(void)bbox;
	(void)width;
	(void)height;
	return std::nullopt;
#else
	if (!width || !height)
		return std::vector<std::vector<double>>{};
	const auto root = cache_root / "mapterhorn";
	std::error_code ec;
	std::filesystem::create_directories(root, ec);
	if (ec)
		return std::nullopt;
	const unsigned zoom = choose_mapterhorn_zoom(bbox, width, height, 0, MAX_ZOOM, 2048);
	const unsigned floor = std::min<unsigned>(MIN_ZOOM, zoom);
	FetchState state;
	std::vector<XyzTileKey> level_keys = covering_xyz_tiles(bbox, zoom);
	unsigned level = zoom;
	// A serial canary catches a total outage before fan-out. For large areas,
	// probe spread-out keys before fetching an entire level with no local data.
	if (!level_keys.empty()) {
		const auto canary = fetch_tile(root, level_keys.front());
		if (canary.status == FetchStatus::Failed)
			return std::nullopt;
		state.saw_network_success |= canary.network_success;
		const auto packed = pack_key(canary.key);
		if (canary.status == FetchStatus::Hit)
			state.available.insert(packed);
		else {
			state.confirmed_missing.insert(packed);
			if (canary.new_missing)
				state.pending_markers.push_back(canary.key);
		}
	}
	while (state.available.empty() && level > floor &&
			level_keys.size() > PROBE_MIN_LEVEL_TILES) {
		const auto stride =
				std::max<std::size_t>(1, level_keys.size() / PROBE_TILE_COUNT);
		std::vector<XyzTileKey> probe_keys;
		probe_keys.reserve(PROBE_TILE_COUNT);
		for (std::size_t i = 0;
				i < level_keys.size() && probe_keys.size() < PROBE_TILE_COUNT;
				i += stride)
			probe_keys.push_back(level_keys[i]);
		std::vector<XyzTileKey> probe_absent;
		if (!fetch_level(root, probe_keys, state, probe_absent))
			return std::nullopt;
		if (!state.available.empty() || state.failed)
			break;
		level_keys = covering_xyz_tiles(bbox, --level);
	}
	while (!level_keys.empty()) {
		std::vector<XyzTileKey> absent;
		if (!fetch_level(root, level_keys, state, absent))
			return std::nullopt;
		if (absent.empty() || level <= floor)
			break;
		std::unordered_set<std::uint64_t> seen;
		std::vector<XyzTileKey> parents;
		parents.reserve(absent.size());
		for (const auto &key : absent) {
			if (!key.zoom)
				continue;
			const auto parent = parent_key(key);
			const auto packed = pack_key(parent);
			if (state.available.contains(packed) || !seen.insert(packed).second)
				continue;
			parents.push_back(parent);
		}
		if (parents.empty())
			break;
		level_keys = std::move(parents);
		--level;
	}

	if (state.available.empty() && state.failed)
		return std::nullopt;
	if (state.available.empty()) {
		const XyzTileKey known_land{10, 534, 364};
		const auto sanity = fetch_tile(root, known_land);
		if (sanity.status != FetchStatus::Hit)
			return std::nullopt;
		state.available.insert(pack_key(known_land));
		state.saw_network_success |= sanity.network_success;
	}
	if (state.saw_network_success)
		for (const auto &key : state.pending_markers) {
			std::ofstream marker(missing_path(root, key), std::ios::binary);
			(void)marker;
		}

	std::vector<std::vector<double>> grid(
			height, std::vector<double>(width, std::numeric_limits<double>::quiet_NaN()));
	for (std::size_t chunk_start = 0; chunk_start < height; chunk_start += 1024) {
		const auto chunk_end = std::min(height, chunk_start + 1024);
		std::unordered_map<std::uint64_t, RgbRaster> tiles;
		load_chunk_tiles(root, bbox, zoom, chunk_start, chunk_end, height, state, tiles);
		for (std::size_t gy = chunk_start; gy < chunk_end; ++gy) {
			const double lat = row_lat(bbox, gy, height);
			const double norm_y = norm_y_for_lat(lat);
			for (std::size_t gx = 0; gx < width; ++gx) {
				const double lon =
						bbox.min_lon + double(gx) / std::max<std::size_t>(1, width - 1) *
											   (bbox.max_lon - bbox.min_lon);
				const double norm_x = (lon + 180.0) / 360.0;
				if (const auto sample = sample_height(tiles, zoom, norm_x, norm_y)) {
					grid[gy][gx] = *sample;
				} else if (state.confirmed_missing.contains(
								   pack_key(tile_key_at(floor, norm_x, norm_y)))) {
					grid[gy][gx] = 0.0;
				}
			}
		}
	}
	return grid;
#endif
}

} // namespace arnis::elevation::providers
