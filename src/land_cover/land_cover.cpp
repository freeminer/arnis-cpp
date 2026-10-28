#include "land_cover.h"
#include "../grid_ops.h"
#include "../cache_root.h"
#include "cog.h"
#include "shoreline.h"
#include "../../http.h"
#include <filesystem>
#include <fstream>
#include "../../arnis_adapter.h"
#include "../element_processing/waterways.h"
#include "../element_processing/way_segments.h"
#include "../bresenham.h"
#include "../clipping.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <array>
#include <deque>
#include <limits>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <utility>
#include <atomic>
#include <cstdlib>
#include <string_view>
#include <thread>
#include <iostream>
#include <mutex>

namespace arnis::land_cover
{
namespace
{
// COG reads are HTTP byte ranges. Cache each range rather than downloading the
// full 3-degree TIFF: Rust uses the same header + internal-tile cache, and this
// also lets separate chunk generations reuse the same ESA source data.
std::vector<std::uint8_t> fetch_cached_esa_range(const std::string &url,
		std::uint64_t offset, std::uint64_t length,
		const std::filesystem::path &cache_dir)
{
	if (length == 0 || length > std::numeric_limits<std::size_t>::max())
		return {};
	const auto slash = url.find_last_of('/');
	const auto basename = slash == std::string::npos ? url : url.substr(slash + 1);
	const auto path = cache_dir / (basename + "_range_" + std::to_string(offset) + "_" +
										  std::to_string(length) + ".bin");
	// A fixed set of locks avoids duplicate concurrent downloads without keeping
	// an unbounded mutex entry for every TIFF tile/range ever visited.
	static std::array<std::mutex, 97> range_mutexes;
	auto &mutex =
			range_mutexes[std::hash<std::string>{}(path.string()) % range_mutexes.size()];
	const std::lock_guard<std::mutex> lock(mutex);
	std::error_code ec;
	if (std::filesystem::is_regular_file(path, ec) && !ec &&
			std::filesystem::file_size(path, ec) == length && !ec) {
		std::ifstream input(path, std::ios::binary);
		if (input) {
			std::vector<std::uint8_t> bytes(static_cast<std::size_t>(length));
			input.read(reinterpret_cast<char *>(bytes.data()),
					static_cast<std::streamsize>(bytes.size()));
			if (input.gcount() == static_cast<std::streamsize>(bytes.size()))
				return bytes;
		}
	}
	const auto response = http_get_range(url, offset, length);
	std::vector<std::uint8_t> bytes(response.begin(), response.end());
	if (bytes.empty() || bytes.size() != static_cast<std::size_t>(length))
		return {};
	std::filesystem::create_directories(cache_dir, ec);
	if (ec)
		return bytes;
	// Write through a temporary file so an interrupted request cannot leave a
	// partial range that a later generation mistakes for a valid cache hit.
	const auto temporary =
			path.string() + ".tmp." +
			std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id()));
	{
		std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
		if (!output)
			return bytes;
		output.write(reinterpret_cast<const char *>(bytes.data()),
				static_cast<std::streamsize>(bytes.size()));
		if (!output)
			return bytes;
	}
	std::filesystem::rename(temporary, path, ec);
	if (ec) {
		std::filesystem::remove(temporary, ec);
		// Another process may have published the same complete range meanwhile.
	}
	return bytes;
}

template <typename Function>
void parallel_rows(std::size_t begin, std::size_t end, Function &&function)
{
	if (begin >= end)
		return;
	std::size_t worker_limit = 0;
	if (const char *configured = std::getenv("RAYON_NUM_THREADS")) {
		const std::string_view text(configured);
		const auto [ptr, ec] =
				std::from_chars(text.data(), text.data() + text.size(), worker_limit);
		if (ec != std::errc{} || ptr != text.data() + text.size() || worker_limit == 0)
			worker_limit = 0;
	}
	if (worker_limit == 0) {
		const auto hardware = std::max(1u, std::thread::hardware_concurrency());
		worker_limit = std::max<std::size_t>(1, static_cast<std::size_t>(hardware * 0.9));
	}
	const auto workers = std::min<std::size_t>(end - begin, worker_limit);
	if (workers == 1) {
		for (auto row = begin; row < end; ++row)
			function(row);
		return;
	}
	std::atomic_size_t next{begin};
	auto worker = [&] {
		for (;;) {
			const auto row = next.fetch_add(1, std::memory_order_relaxed);
			if (row >= end)
				return;
			function(row);
		}
	};
	std::vector<std::thread> threads;
	threads.reserve(workers - 1);
	for (std::size_t i = 1; i < workers; ++i) {
		try {
			threads.emplace_back(worker);
		} catch (...) {
			break;
		}
	}
	worker();
	for (auto &thread : threads)
		thread.join();
}

double cells_per_meter(const GeographicBounds &bbox, std::size_t grid_width)
{
	constexpr double earth_radius_m = 6371000.0;
	constexpr double pi = 3.14159265358979323846;
	const double latitude = (bbox.min_lat + bbox.max_lat) * 0.5 * pi / 180.0;
	const double delta = (bbox.max_lng - bbox.min_lng) * pi / 180.0;
	const double sine = std::cos(latitude) * std::sin(delta * 0.5);
	const double q = std::clamp(sine * sine, 0.0, 1.0);
	const double width_m =
			earth_radius_m * 2.0 * std::atan2(std::sqrt(q), std::sqrt(1.0 - q));
	return width_m > 0.0 && grid_width > 0 ? grid_width / width_m : 1.0;
}
}

std::string esa_tile_url(int lat, int lng)
{
	char ns = 'N', ew = 'E';
	int la = lat, lo = lng;
	if (la < 0) {
		ns = 'S';
		la = -la;
	}
	if (lo < 0) {
		ew = 'W';
		lo = -lo;
	}
	std::ostringstream name;
	name << "https://esa-worldcover.s3.eu-central-1.amazonaws.com/v200/2021/map/ESA_WorldCover_10m_2021_v200_"
		 << ns << std::setw(2) << std::setfill('0') << la << ew << std::setw(3)
		 << std::setfill('0') << lo << "_Map.tif";
	return name.str();
}
std::vector<std::tuple<int, int, std::string>> esa_tiles_for_bbox(
		double a, double b, double c, double d)
{
	std::vector<std::tuple<int, int, std::string>> o;
	if (c < -60 || a > 84)
		return o;
	a = std::max(a, -60.0);
	// The last valid south-west tile is 81N/177E.  At the exclusive dataset
	// boundary, floor(84 / 3) would otherwise manufacture an invalid N84 tile.
	c = std::min(c, 84.0 - 0.001);
	b = std::max(b, -180.0);
	d = std::min(d, 180.0 - 0.001);
	int y0 = int(std::floor(a / 3.0)) * 3, y1 = int(std::floor(c / 3.0)) * 3;
	int x0 = int(std::floor(b / 3.0)) * 3, x1 = int(std::floor(d / 3.0)) * 3;
	for (int y = y0; y <= y1; y += 3)
		for (int x = x0; x <= x1; x += 3)
			o.emplace_back(y, x, esa_tile_url(y, x));
	return o;
}

LandCoverData fetch_land_cover_data(const GeographicBounds &bbox, std::size_t width,
		std::size_t height, bool smooth_boundaries)
{
	if (!bbox.valid() || width == 0 || height == 0)
		return {};
	const auto cache_dir = cache::provider_cache_root("land_cover");
	std::error_code cache_error;
	std::filesystem::create_directories(cache_dir, cache_error);
	const auto fetch = [cache_dir](const std::string &url, std::uint64_t offset,
							   std::uint64_t length) {
		return fetch_cached_esa_range(url, offset, length, cache_dir);
	};
	std::vector<EsaRasterTile> tiles;
	for (const auto &[lat, lng, url] :
			esa_tiles_for_bbox(bbox.min_lat, bbox.min_lng, bbox.max_lat, bbox.max_lng)) {
		EsaRasterTile tile;
		if (read_esa_cog_into_raster(url, lat, lng, bbox.min_lat, bbox.min_lng,
					bbox.max_lat, bbox.max_lng, tile, fetch))
			tiles.push_back(std::move(tile));
	}
	return assemble_land_cover_data(bbox, width, height, tiles, smooth_boundaries);
}
bool fetch_esa_tile(const std::string &url, const std::filesystem::path &file)
{
	if (std::filesystem::exists(file) && std::filesystem::file_size(file) > 0)
		return true;
	return http_to_file(url, file.string()) > 0;
}

uint8_t EsaRasterTile::sample(double lat, double lng) const
{
	if (!valid() || !contains(lat, lng))
		return 0;
	// ESA COG scanlines run north to south. Clamp the north/east edge to the
	// final pixel: neighbouring 3-degree tiles share that geometric boundary.
	const double south = min_lat != max_lat ? min_lat : south_lat;
	const double north = min_lat != max_lat ? max_lat : south_lat + 3.0;
	const double west = min_lng != max_lng ? min_lng : west_lng;
	const double east = min_lng != max_lng ? max_lng : west_lng + 3.0;
	const double u =
			east != west ? std::clamp((lng - west) / (east - west), 0.0, 1.0) : 0.0;
	const double v =
			north != south ? std::clamp((north - lat) / (north - south), 0.0, 1.0) : 0.0;
	const auto x =
			std::min(width - 1, static_cast<std::size_t>(std::floor(u * double(width))));
	const auto z = std::min(
			height - 1, static_cast<std::size_t>(std::floor(v * double(height))));
	return pixels[z * width + x];
}

void fill_land_cover_gaps(
		std::vector<std::vector<uint8_t>> &grid, std::size_t width, std::size_t height)
{
	width = std::min(width, grid.empty() ? std::size_t{0} : grid.front().size());
	height = std::min(height, grid.size());
	bool has_gap = false;
	for (std::size_t z = 0; z < height && !has_gap; ++z)
		has_gap = std::find(grid[z].begin(), grid[z].begin() + width, 0) !=
				  grid[z].begin() + width;
	if (!has_gap)
		return;
	for (int pass = 0; pass < 10; ++pass) {
		const auto snapshot = grid;
		bool changed = false;
		for (std::size_t z = 0; z < height; ++z)
			for (std::size_t x = 0; x < width; ++x) {
				if (snapshot[z][x] != 0)
					continue;
				for (const auto &[dx, dz] : std::array<std::pair<int, int>, 4>{
							 {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}}) {
					const int nx = int(x) + dx, nz = int(z) + dz;
					if (nx >= 0 && nz >= 0 && nx < int(width) && nz < int(height) &&
							snapshot[std::size_t(nz)][std::size_t(nx)] != 0) {
						grid[z][x] = snapshot[std::size_t(nz)][std::size_t(nx)];
						changed = true;
						break;
					}
				}
			}
		if (!changed)
			break;
	}
}

void smooth_land_cover_boundaries(std::vector<std::vector<uint8_t>> &grid,
		std::size_t width, std::size_t height, double cells_per_meter_value)
{
	width = std::min(width, grid.empty() ? std::size_t{0} : grid.front().size());
	height = std::min(height, grid.size());
	if (width == 0 || height == 0)
		return;
	const double sigma = std::max(2.0, 2.0 * cells_per_meter_value);
	const int radius = static_cast<int>(std::ceil(sigma * 3.0));
	const int side = radius * 2 + 1;
	std::vector<double> kernel(static_cast<std::size_t>(side * side));
	for (int z = -radius; z <= radius; ++z)
		for (int x = -radius; x <= radius; ++x)
			kernel[std::size_t(z + radius) * side + std::size_t(x + radius)] =
					std::exp(-(x * x + z * z) / (2.0 * sigma * sigma));
	const auto snapshot = grid;
	parallel_rows(0, height, [&](std::size_t z) {
		std::array<double, 256> votes{};
		std::array<std::uint8_t, 16> seen{};
		std::size_t seen_len = 0;
		for (std::size_t x = 0; x < width; ++x) {
			const auto center = snapshot[z][x];
			if (center == 0)
				continue;
			bool boundary = false;
			for (const auto &[dx, dz] : std::array<std::pair<int, int>, 4>{
						 {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}}) {
				const int nx = int(x) + dx, nz = int(z) + dz;
				if (nx >= 0 && nz >= 0 && nx < int(width) && nz < int(height) &&
						snapshot[std::size_t(nz)][std::size_t(nx)] != 0 &&
						snapshot[std::size_t(nz)][std::size_t(nx)] != center) {
					boundary = true;
					break;
				}
			}
			if (!boundary)
				continue;

			for (std::size_t i = 0; i < seen_len; ++i)
				votes[seen[i]] = 0.0;
			seen_len = 0;
			for (int dz = -radius; dz <= radius; ++dz)
				for (int dx = -radius; dx <= radius; ++dx) {
					const int nx = int(x) + dx, nz = int(z) + dz;
					if (nx >= 0 && nz >= 0 && nx < int(width) && nz < int(height)) {
						const auto cls = snapshot[std::size_t(nz)][std::size_t(nx)];
						if (!cls)
							continue;
						const double previous = votes[cls];
						votes[cls] = previous + kernel[std::size_t(dz + radius) * side +
														std::size_t(dx + radius)];
						if (previous == 0.0 && seen_len < seen.size())
							seen[seen_len++] = cls;
					}
				}
			std::uint8_t best = 0;
			double best_vote = 0.0;
			for (std::size_t i = 0; i < seen_len; ++i) {
				const auto cls = seen[i];
				if (votes[cls] > best_vote) {
					best = cls;
					best_vote = votes[cls];
				}
			}
			if (best_vote > 0.0)
				grid[z][x] = best;
		}
	});
}

LandCoverData assemble_land_cover_data(const GeographicBounds &bbox, std::size_t width,
		std::size_t height, const std::vector<EsaRasterTile> &tiles,
		bool smooth_boundaries)
{
	LandCoverData out;
	if (!bbox.valid() || width == 0 || height == 0)
		return out;
	out.source_bounds = bbox;
	out.width = width;
	out.height = height;
	out.cells_per_meter = cells_per_meter(bbox, width);
	out.grid.assign(height, std::vector<uint8_t>(width));
	// The shoreline assembler also performs Rust-compatible pixel-span
	// sampling. Do not pre-sample at grid endpoints here: a coarse output grid
	// can miss all valid ESA pixels even though the requested raster overlaps.
	reconstruct_water_shoreline(out, tiles);
	const bool has_data =
			std::any_of(out.grid.begin(), out.grid.end(), [](const auto &row) {
				return std::any_of(
						row.begin(), row.end(), [](std::uint8_t c) { return c != 0; });
			});
	if (!has_data)
		return {};
	fill_land_cover_gaps(out.grid, width, height);
	if (smooth_boundaries)
		smooth_land_cover_boundaries(out.grid, width, height, out.cells_per_meter);
	out.water_distance = compute_water_distance(out.grid, width, height);
	return out;
}

uint64_t coord_hash(int32_t x, int32_t z)
{
	// Keep the Rust arithmetic and wrapping points verbatim: this hash drives
	// deterministic surface dithering, so a merely "good" different hash would
	// visibly diverge between implementations.
	uint64_t h = uint64_t(uint32_t(x)) * 0x9E3779B97F4A7C15ULL;
	h ^= uint64_t(uint32_t(z)) * 0x517CC1B727220A95ULL;
	h *= 0x6C62272E07BB0142ULL;
	return h ^ (h >> 32);
}

std::optional<uint8_t> nearest_land_class(const std::vector<std::vector<uint8_t>> &grid,
		std::size_t width, std::size_t height, std::size_t x, std::size_t z, int radius)
{
	if (x >= width || z >= height || grid.size() < height || radius <= 0)
		return std::nullopt;
	const auto center_x = static_cast<std::int64_t>(x);
	const auto center_z = static_cast<std::int64_t>(z);
	for (int r = 1; r <= radius; ++r) {
		for (int dz = -r; dz <= r; ++dz) {
			const auto nz = center_z + dz;
			if (nz < 0 || nz >= static_cast<std::int64_t>(height) ||
					grid[static_cast<std::size_t>(nz)].size() < width)
				continue;
			const int step = std::abs(dz) == r ? 1 : 2 * r;
			for (int dx = -r; dx <= r; dx += step) {
				const auto nx = center_x + dx;
				if (nx < 0 || nx >= static_cast<std::int64_t>(width))
					continue;
				const auto cls =
						grid[static_cast<std::size_t>(nz)][static_cast<std::size_t>(nx)];
				if (cls != 0 && cls != LC_WATER)
					return cls;
			}
		}
	}
	return std::nullopt;
}

static std::vector<double> gaussian_kernel(double sigma)
{
	const int radius = std::max(1, static_cast<int>(std::ceil(sigma * 3.0)));
	std::vector<double> kernel(static_cast<std::size_t>(radius * 2 + 1));
	double sum = 0.0;
	for (int i = -radius; i <= radius; ++i) {
		const double v = std::exp(-(static_cast<double>(i * i)) / (2.0 * sigma * sigma));
		kernel[static_cast<std::size_t>(i + radius)] = v;
		sum += v;
	}
	for (double &v : kernel)
		v /= sum;
	return kernel;
}

std::vector<std::vector<float>> compute_water_blend_smooth(
		const std::vector<std::vector<uint8_t>> &grid, std::size_t width,
		std::size_t height, double cells_per_meter_value)
{
	if (width == 0 || height == 0)
		return {};
	height = std::min(height, grid.size());
	if (height == 0 || grid.front().empty())
		return {};
	width = std::min(width, grid.front().size());
	if (width == 0)
		return {};

	const double sigma = std::max(3.0, 3.0 * cells_per_meter_value);
	const auto kernel = gaussian_kernel(sigma);
	const int radius = static_cast<int>(kernel.size() / 2);

	std::vector<std::vector<double>> tmp(height, std::vector<double>(width, 0.0));
	for (std::size_t z = 0; z < height; ++z) {
		std::vector<std::uint32_t> water_prefix(width + 1, 0);
		for (std::size_t x = 0; x < width; ++x)
			water_prefix[x + 1] = water_prefix[x] + (grid[z][x] == LC_WATER ? 1u : 0u);
		for (std::size_t x = 0; x < width; ++x) {
			const auto lo = x > static_cast<std::size_t>(radius)
									? x - static_cast<std::size_t>(radius)
									: 0;
			const auto hi = std::min(width, x + static_cast<std::size_t>(radius) + 1);
			const auto hits = water_prefix[hi] - water_prefix[lo];
			if (hits == 0) {
				tmp[z][x] = 0.0;
				continue;
			}
			if (hits == hi - lo) {
				tmp[z][x] = 1.0;
				continue;
			}
			double sum = 0.0;
			double weight = 0.0;
			for (int k = -radius; k <= radius; ++k) {
				const int sx = static_cast<int>(x) + k;
				if (sx < 0 || sx >= static_cast<int>(width))
					continue;
				const double w = kernel[static_cast<std::size_t>(k + radius)];
				sum += (grid[z][static_cast<std::size_t>(sx)] == LC_WATER ? 1.0 : 0.0) *
					   w;
				weight += w;
			}
			tmp[z][x] = weight > 0.0 ? sum / weight : 0.0;
		}
	}

	std::vector<std::vector<float>> out(height, std::vector<float>(width, 0.0f));
	for (std::size_t x = 0; x < width; ++x) {
		std::vector<std::uint32_t> ones_prefix(height + 1, 0),
				zeros_prefix(height + 1, 0);
		for (std::size_t y = 0; y < height; ++y) {
			ones_prefix[y + 1] = ones_prefix[y] + (tmp[y][x] == 1.0 ? 1u : 0u);
			zeros_prefix[y + 1] = zeros_prefix[y] + (tmp[y][x] == 0.0 ? 1u : 0u);
		}
		for (std::size_t z = 0; z < height; ++z) {
			const auto lo = z > static_cast<std::size_t>(radius)
									? z - static_cast<std::size_t>(radius)
									: 0;
			const auto hi = std::min(height, z + static_cast<std::size_t>(radius) + 1);
			const auto window = static_cast<std::uint32_t>(hi - lo);
			if (ones_prefix[hi] - ones_prefix[lo] == window) {
				out[z][x] = 1.0f;
				continue;
			}
			if (zeros_prefix[hi] - zeros_prefix[lo] == window) {
				out[z][x] = 0.0f;
				continue;
			}
			double sum = 0.0;
			double weight = 0.0;
			for (int k = -radius; k <= radius; ++k) {
				const int sz = static_cast<int>(z) + k;
				if (sz < 0 || sz >= static_cast<int>(height))
					continue;
				const double w = kernel[static_cast<std::size_t>(k + radius)];
				sum += tmp[static_cast<std::size_t>(sz)][x] * w;
				weight += w;
			}
			out[z][x] = static_cast<float>(weight > 0.0 ? sum / weight : 0.0);
		}
	}
	return out;
}

std::vector<std::vector<uint8_t>> compute_water_distance(
		const std::vector<std::vector<uint8_t>> &grid, std::size_t width,
		std::size_t height)
{
	std::vector<std::vector<uint8_t>> dist(height, std::vector<uint8_t>(width, 0));
	if (width == 0 || height == 0)
		return dist;

	std::deque<std::pair<int, int>> q;
	const int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

	for (std::size_t z = 0; z < height; ++z) {
		for (std::size_t x = 0; x < width; ++x) {
			if (grid[z][x] != LC_WATER)
				continue;
			bool shore = false;
			for (const auto &d : dirs) {
				const int nx = static_cast<int>(x) + d[0];
				const int nz = static_cast<int>(z) + d[1];
				if (nx < 0 || nz < 0 || nx >= static_cast<int>(width) ||
						nz >= static_cast<int>(height) ||
						grid[static_cast<std::size_t>(nz)]
							[static_cast<std::size_t>(nx)] != LC_WATER) {
					shore = true;
					break;
				}
			}
			if (shore) {
				dist[z][x] = 1;
				q.emplace_back(static_cast<int>(x), static_cast<int>(z));
			}
		}
	}

	while (!q.empty()) {
		auto [x, z] = q.front();
		q.pop_front();
		if (dist[static_cast<std::size_t>(z)][static_cast<std::size_t>(x)] >= 15)
			continue;
		const uint8_t next =
				dist[static_cast<std::size_t>(z)][static_cast<std::size_t>(x)] ==
								std::numeric_limits<uint8_t>::max()
						? std::numeric_limits<uint8_t>::max()
						: static_cast<uint8_t>(dist[static_cast<std::size_t>(z)]
												   [static_cast<std::size_t>(x)] +
											   1);
		for (const auto &d : dirs) {
			const int nx = x + d[0];
			const int nz = z + d[1];
			if (nx < 0 || nz < 0 || nx >= static_cast<int>(width) ||
					nz >= static_cast<int>(height))
				continue;
			auto &cell = dist[static_cast<std::size_t>(nz)][static_cast<std::size_t>(nx)];
			if (grid[static_cast<std::size_t>(nz)][static_cast<std::size_t>(nx)] !=
							LC_WATER ||
					cell != 0)
				continue;
			cell = next;
			q.emplace_back(nx, nz);
		}
	}
	return dist;
}

bool WaterBlendMask::contains(int x, int z) const
{
	if (width == 0 || height == 0 || words_per_row == 0)
		return false;
	const auto cx =
			x <= 0 ? std::size_t(0) : std::min(static_cast<std::size_t>(x), width - 1);
	const auto cz =
			z <= 0 ? std::size_t(0) : std::min(static_cast<std::size_t>(z), height - 1);
	const auto index = cz * words_per_row + (cx >> 6);
	return index < bits.size() && ((bits[index] >> (cx & 63)) & 1U) != 0;
}

const WaterBlendMask &LandCoverData::water_blend_mask(
		std::size_t world_width, std::size_t world_height) const
{
	if (!water_blend_cache)
		water_blend_cache = std::make_shared<WaterBlendCache>();
	const auto cache = water_blend_cache;
	std::lock_guard lock(cache->mutex);
	if (!cache->mask || cache->mask_width != world_width ||
			cache->mask_height != world_height) {
		WaterBlendMask mask;
		mask.width = world_width;
		mask.height = world_height;
		mask.words_per_row = world_width / 64 + (world_width % 64 != 0);
		if (mask.height && mask.words_per_row <=
								   std::numeric_limits<std::size_t>::max() / mask.height)
			mask.bits.assign(mask.words_per_row * mask.height, 0);
		if (width && height && mask.bits.size() && grid.size() >= height) {
			const auto smooth =
					compute_water_blend_smooth(grid, width, height, cells_per_meter);
			parallel_rows(0, world_height, [&](std::size_t z) {
				const double fz = std::clamp(double(z) / double(std::max<std::size_t>(
																 1, world_height - 1)),
										  0.0, 1.0) *
								  double(height - 1);
				const auto z0 = std::min<std::size_t>(std::floor(fz), height - 1);
				const auto z1 = std::min(z0 + 1, height - 1);
				const double tz = fz - std::floor(fz);
				if (smooth[z0].size() < width || smooth[z1].size() < width)
					return;
				for (std::size_t x = 0; x < world_width; ++x) {
					const double fx = std::clamp(double(x) / double(std::max<std::size_t>(
																	 1, world_width - 1)),
											  0.0, 1.0) *
									  double(width - 1);
					const auto x0 = std::min<std::size_t>(std::floor(fx), width - 1);
					const auto x1 = std::min(x0 + 1, width - 1);
					const double tx = fx - std::floor(fx);
					const double top = smooth[z0][x0] * (1.0 - tx) + smooth[z0][x1] * tx;
					const double bottom =
							smooth[z1][x0] * (1.0 - tx) + smooth[z1][x1] * tx;
					if (top * (1.0 - tz) + bottom * tz > .5)
						mask.bits[z * mask.words_per_row + (x >> 6)] |= std::uint64_t(1)
																		<< (x & 63);
				}
			});
		}
		cache->mask = std::move(mask);
		cache->mask_width = world_width;
		cache->mask_height = world_height;
	}
	return *cache->mask;
}

void LandCoverData::invalidate_water_blend_grid()
{
	water_blend_cache = std::make_shared<WaterBlendCache>();
	beach_tile_cache = std::make_shared<BeachTileCache>();
}

bool LandCoverData::beach_within(std::size_t x, std::size_t z, std::size_t reach) const
{
	constexpr std::size_t TILE_SHIFT = 4;
	constexpr std::size_t TILE_SIZE = std::size_t{1} << TILE_SHIFT;
	if (width == 0 || height == 0 || grid.size() < height || x >= width || z >= height)
		return false;
	auto cache = beach_tile_cache;
	if (!cache)
		return false;
	std::shared_ptr<const BeachTileIndex> index;
	{
		std::lock_guard<std::mutex> lock(cache->mutex);
		index = cache->index;
		if (!index) {
			auto built = std::make_shared<BeachTileIndex>();
			built->width = (width + TILE_SIZE - 1) >> TILE_SHIFT;
			const auto tiles_h = (height + TILE_SIZE - 1) >> TILE_SHIFT;
			const auto tile_count = built->width * tiles_h;
			built->bits.resize((tile_count + 63) / 64);
			for (std::size_t tz = 0; tz < tiles_h; ++tz) {
				const auto z0 = tz << TILE_SHIFT;
				const auto z1 = std::min(height, z0 + TILE_SIZE);
				for (std::size_t tx = 0; tx < built->width; ++tx) {
					const auto x0 = tx << TILE_SHIFT;
					const auto x1 = std::min(width, x0 + TILE_SIZE);
					bool has_beach = false;
					for (std::size_t row = z0; row < z1 && !has_beach; ++row) {
						if (grid[row].size() < x1)
							continue;
						has_beach =
								std::find(grid[row].begin() + x0, grid[row].begin() + x1,
										LC_BEACH) != grid[row].begin() + x1;
					}
					if (!has_beach)
						continue;
					const auto bit = tz * built->width + tx;
					built->bits[bit >> 6] |= std::uint64_t{1} << (bit & 63);
					built->any = true;
				}
			}
			cache->index = built;
			index = std::move(built);
		}
	}
	if (!index->any)
		return false;
	const auto x0 = x > reach ? x - reach : 0;
	const auto z0 = z > reach ? z - reach : 0;
	const auto x1 = x + std::min(reach, width - 1 - x);
	const auto z1 = z + std::min(reach, height - 1 - z);
	for (std::size_t tz = z0 >> TILE_SHIFT; tz <= (z1 >> TILE_SHIFT); ++tz)
		for (std::size_t tx = x0 >> TILE_SHIFT; tx <= (x1 >> TILE_SHIFT); ++tx) {
			const auto bit = tz * index->width + tx;
			if ((index->bits[bit >> 6] >> (bit & 63)) & 1U) {
				for (std::size_t row = z0; row <= z1; ++row) {
					if (grid[row].size() <= x1)
						continue;
					if (std::find(grid[row].begin() + x0, grid[row].begin() + x1 + 1,
								LC_BEACH) != grid[row].begin() + x1 + 1)
						return true;
				}
			}
		}
	return false;
}

void LandCoverData::remap_rows_to_mercator(double lat_top, double lat_bottom)
{
	const auto rows = height;
	const auto source = [=](std::size_t z) {
		return grid_ops::mercator_source_row(lat_top, lat_bottom, rows, z);
	};
	const auto nearest = [](std::uint8_t a, std::uint8_t b, double t) {
		return t < 0.5 ? a : b;
	};
	grid_ops::remap_rows_in_place(grid, source, nearest);
	grid_ops::remap_rows_in_place(water_distance, source, nearest);
	invalidate_water_blend_grid();
}

void LandCoverData::crop(
		std::size_t x0, std::size_t z0, std::size_t new_width, std::size_t new_height)
{
	grid_ops::crop_rows(grid, x0, z0, new_width, new_height);
	grid_ops::crop_rows(water_distance, x0, z0, new_width, new_height);
	width = new_width;
	height = new_height;
	invalidate_water_blend_grid();
}

void mark_beaches(LandCoverData &data)
{
	if (data.width == 0 || data.height == 0)
		return;
	data.invalidate_water_blend_grid();
	const double raw_reach = std::round(50.0 * data.cells_per_meter);
	const int reach = std::isnan(raw_reach)
							  ? 0
							  : static_cast<int>(std::clamp(raw_reach, 1.0, 255.0));
	std::vector<std::pair<std::uint32_t, std::uint32_t>> frontier;
	for (std::size_t z = 0; z < data.height; ++z)
		for (std::size_t x = 0; x < data.width; ++x) {
			if (data.grid[z][x] != LC_BARE)
				continue;
			const bool touches_water =
					(x > 0 && data.grid[z][x - 1] == LC_WATER) ||
					(x + 1 < data.width && data.grid[z][x + 1] == LC_WATER) ||
					(z > 0 && data.grid[z - 1][x] == LC_WATER) ||
					(z + 1 < data.height && data.grid[z + 1][x] == LC_WATER);
			if (touches_water)
				frontier.emplace_back(
						static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(z));
		}
	for (const auto &[x, z] : frontier)
		data.grid[z][x] = LC_BEACH;

	std::vector<std::pair<std::uint32_t, std::uint32_t>> next;
	for (int step = 1; step < reach && !frontier.empty(); ++step) {
		next.clear();
		for (const auto &[x32, z32] : frontier) {
			const auto x = static_cast<std::size_t>(x32);
			const auto z = static_cast<std::size_t>(z32);
			const std::array<std::pair<std::size_t, std::size_t>, 4> candidates{
					{{x > 0 ? x - 1 : data.width, z}, {x + 1, z},
							{x, z > 0 ? z - 1 : data.height}, {x, z + 1}}};
			for (const auto &[nx, nz] : candidates) {
				if (nx >= data.width || nz >= data.height || data.grid[nz][nx] != LC_BARE)
					continue;
				data.grid[nz][nx] = LC_BEACH;
				next.emplace_back(
						static_cast<std::uint32_t>(nx), static_cast<std::uint32_t>(nz));
			}
		}
		frontier.swap(next);
	}
}

std::size_t clear_land_cover_cache(const std::filesystem::path &cache_dir)
{
	std::size_t removed = 0;
	if (!std::filesystem::exists(cache_dir))
		return 0;
	for (const auto &entry : std::filesystem::directory_iterator(cache_dir)) {
		std::error_code ec;
		if (entry.is_regular_file(ec) && std::filesystem::remove(entry.path(), ec) && !ec)
			++removed;
	}
	return removed;
}

}
