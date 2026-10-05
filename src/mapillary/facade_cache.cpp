#include "facade_cache.h"

#include "cache.h"
#include "stb_image.h"

#include <algorithm>
#include <atomic>
#include <fstream>
#include <nlohmann/json.hpp>
#include <png.h>
#include <limits>
#include <system_error>

#if defined(_WIN32)
#include <process.h>
#define ARNIS_GETPID _getpid
#else
#include <unistd.h>
#define ARNIS_GETPID getpid
#endif

namespace arnis::mapillary::cache
{
namespace
{
constexpr std::size_t max_png_pixels = 16 * 1024 * 1024;
constexpr std::size_t max_cache_bytes = 256 * 1024 * 1024;
std::atomic<std::uint64_t> temp_counter{0};

std::string shard(const std::string &key)
{
	return key.size() < 2 ? key : key.substr(key.size() - 2);
}

double stored_reach(const std::filesystem::path &path)
{
	const auto bytes = read_cached(path);
	if (!bytes)
		return 0.0;
	try {
		const auto record = nlohmann::json::parse(bytes->begin(), bytes->end());
		return record.value("cols", 0u) == 0 ? record.value("reach_m", 0.0) : 0.0;
	} catch (...) {
		return 0.0;
	}
}

std::optional<nlohmann::json> read_record(const std::filesystem::path &path)
{
	const auto bytes = read_cached(path);
	if (!bytes)
		return std::nullopt;
	try {
		return nlohmann::json::parse(bytes->begin(), bytes->end());
	} catch (...) {
		return std::nullopt;
	}
}

std::optional<std::vector<std::uint8_t>> decode_rgba(const std::filesystem::path &path,
		std::uint32_t expected_width = 0, std::uint32_t expected_height = 0,
		std::uint32_t *width_out = nullptr, std::uint32_t *height_out = nullptr)
{
	const auto bytes = read_cached(path);
	if (!bytes ||
			bytes->size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
		return std::nullopt;
	int width = 0, height = 0, channels = 0;
	stbi_uc *pixels = stbi_load_from_memory(bytes->data(),
			static_cast<int>(bytes->size()), &width, &height, &channels, 4);
	if (!pixels || width <= 0 || height <= 0 ||
			static_cast<std::size_t>(width) * height > max_png_pixels ||
			(expected_width && static_cast<std::uint32_t>(width) != expected_width) ||
			(expected_height && static_cast<std::uint32_t>(height) != expected_height)) {
		if (pixels)
			stbi_image_free(pixels);
		return std::nullopt;
	}
	const auto byte_count = static_cast<std::size_t>(width) * height * 4;
	std::vector<std::uint8_t> result(pixels, pixels + byte_count);
	stbi_image_free(pixels);
	if (width_out)
		*width_out = static_cast<std::uint32_t>(width);
	if (height_out)
		*height_out = static_cast<std::uint32_t>(height);
	return result;
}

std::vector<std::uint8_t> png_bytes(
		const std::vector<std::uint8_t> &rgba, std::uint32_t width, std::uint32_t height)
{
	if (!width || !height || rgba.size() != std::size_t(width) * height * 4)
		return {};
	png_image image{};
	image.version = PNG_IMAGE_VERSION;
	image.width = width;
	image.height = height;
	image.format = PNG_FORMAT_RGBA;
	png_alloc_size_t size = 0;
	if (!png_image_write_to_memory(&image, nullptr, &size, 0, rgba.data(), 0, nullptr) ||
			size == 0)
		return {};
	std::vector<std::uint8_t> encoded(static_cast<std::size_t>(size));
	if (!png_image_write_to_memory(
				&image, encoded.data(), &size, 0, rgba.data(), 0, nullptr))
		return {};
	encoded.resize(static_cast<std::size_t>(size));
	return encoded;
}
} // namespace

std::string wall_cache_key(const std::vector<std::int64_t> &node_ids, std::size_t piece)
{
	// Match Rust's FNV-1a 64-bit hasher over little-endian node ids and piece.
	std::uint64_t hash = 0xcbf29ce484222325ULL;
	auto add = [&](std::uint64_t value) {
		for (unsigned byte = 0; byte < 8; ++byte) {
			hash ^= static_cast<std::uint8_t>(value >> (byte * 8));
			hash *= 0x100000001b3ULL;
		}
	};
	for (const auto id : node_ids)
		add(static_cast<std::uint64_t>(id));
	add(static_cast<std::uint64_t>(piece));
	constexpr char digits[] = "0123456789abcdef";
	std::string result(16, '0');
	for (int i = 15; i >= 0; --i) {
		result[static_cast<std::size_t>(i)] = digits[hash & 0x0f];
		hash >>= 4;
	}
	return result;
}

std::vector<std::int64_t> wall_node_ids(const WallProduct &product)
{
	if (product.edges.empty())
		return {product.node_a, product.node_b};
	std::vector<std::int64_t> ids;
	ids.reserve(product.edges.size() + 1);
	ids.push_back(product.edges.front().node_a);
	for (const auto &edge : product.edges)
		ids.push_back(edge.node_b);
	return ids;
}

std::optional<
		std::tuple<std::filesystem::path, std::filesystem::path, std::filesystem::path>>
wall_paths(const std::filesystem::path &directory, const std::string &key)
{
	if (!Layout::safe_key(key))
		return std::nullopt;
	const auto subdirectory = directory / shard(key);
	return std::tuple{subdirectory / (key + ".json"), subdirectory / (key + ".png"),
			subdirectory / (key + "_tex.png")};
}

bool write_atomic(const std::filesystem::path &path,
		const std::vector<std::uint8_t> &bytes, std::string *error)
{
	const auto parent = path.parent_path();
	if (parent.empty()) {
		if (error)
			*error = path.string() + " has no parent directory";
		return false;
	}
	std::error_code ec;
	std::filesystem::create_directories(parent, ec);
	if (ec) {
		if (error)
			*error = "create " + parent.string() + ": " + ec.message();
		return false;
	}
	auto temp = path;
	temp += ".tmp" + std::to_string(ARNIS_GETPID()) +
			std::to_string(temp_counter.fetch_add(1, std::memory_order_relaxed));
	{
		std::ofstream output(temp, std::ios::binary | std::ios::trunc);
		if (!output) {
			if (error)
				*error = "open " + temp.string() + " for writing failed";
			return false;
		}
		if (!bytes.empty())
			output.write(reinterpret_cast<const char *>(bytes.data()),
					static_cast<std::streamsize>(bytes.size()));
		output.close();
		if (!output) {
			std::filesystem::remove(temp, ec);
			if (error)
				*error = "write " + temp.string() + " failed";
			return false;
		}
	}
	std::filesystem::rename(temp, path, ec);
	if (ec) {
		const auto rename_error = ec.message();
		std::filesystem::remove(temp, ec);
		if (error)
			*error = "rename cache entry into " + path.string() + ": " + rename_error;
		return false;
	}
	return true;
}

std::optional<std::vector<std::uint8_t>> read_cached(const std::filesystem::path &path)
{
	std::error_code ec;
	const auto size = std::filesystem::file_size(path, ec);
	if (ec || size == 0 || size > max_cache_bytes)
		return std::nullopt;
	std::ifstream input(path, std::ios::binary);
	if (!input)
		return std::nullopt;
	std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
	input.read(reinterpret_cast<char *>(bytes.data()),
			static_cast<std::streamsize>(bytes.size()));
	if (!input)
		return std::nullopt;
	return bytes;
}

std::string store_wall(const std::filesystem::path &directory, const WallProduct &product,
		double reach_m, std::string *error)
{
	if (error)
		error->clear();
	const auto key = wall_cache_key(wall_node_ids(product), product.piece);
	const auto paths = wall_paths(directory, key);
	if (!paths)
		return {};
	const auto &[json_path, png_path, texture_path] = *paths;
	const auto cells = product.cell_count();
	if (cells != product.rgb.size() || cells != product.classes.size() ||
			(cells && product.observed.size() != cells)) {
		if (error)
			*error = "wall " + product.wall_key + ": cell arrays do not match dimensions";
		return {};
	}
	if (cells > max_png_pixels ||
			(!product.texture_rgba.empty() &&
					(product.texture_width == 0 || product.texture_height == 0 ||
							product.texture_rgba.size() !=
									std::size_t(product.texture_width) *
											product.texture_height * 4 ||
							std::size_t(product.texture_width) * product.texture_height >
									max_png_pixels))) {
		if (error)
			*error = "wall " + product.wall_key +
					 ": invalid or oversized image dimensions";
		return {};
	}
	if (cells == 0) {
		std::error_code ignored;
		std::filesystem::remove(png_path, ignored);
		std::filesystem::remove(texture_path, ignored);
	} else {
		std::vector<std::uint8_t> rgba(cells * 4);
		for (std::size_t i = 0; i < cells; ++i) {
			std::copy(product.rgb[i].begin(), product.rgb[i].end(), rgba.begin() + i * 4);
			rgba[i * 4 + 3] = product.classes[i];
		}
		std::string image_error;
		const auto png = png_bytes(rgba, product.cols, product.rows);
		if (png.empty() || !write_atomic(png_path, png, &image_error)) {
			if (error)
				*error = "write wall image: " +
						 (image_error.empty() ? std::string("PNG encoding failed")
											  : image_error);
			return {};
		}
		if (!product.texture_rgba.empty()) {
			const auto texture_png = png_bytes(
					product.texture_rgba, product.texture_width, product.texture_height);
			if (texture_png.empty() ||
					!write_atomic(texture_path, texture_png, &image_error)) {
				if (error)
					*error = "write wall texture: " +
							 (image_error.empty() ? std::string("PNG encoding failed")
												  : image_error);
				return {};
			}
		} else {
			std::error_code ignored;
			std::filesystem::remove(texture_path, ignored);
		}
	}

	if (product.cols == 0)
		reach_m = std::max(reach_m, stored_reach(json_path));
	nlohmann::json edges = nlohmann::json::array();
	for (const auto &edge : product.edges)
		edges.push_back({{"edge_idx", edge.edge_idx}, {"node_a", edge.node_a},
				{"node_b", edge.node_b}, {"s0", edge.s0}, {"s1", edge.s1}});
	std::string observed;
	observed.reserve(product.observed.size());
	for (bool value : product.observed)
		observed.push_back(value ? '1' : '0');
	nlohmann::json bands = nlohmann::json::array();
	for (const auto &band : product.bands)
		bands.push_back(band);
	const nlohmann::json record{{"wall_key", product.wall_key},
			{"building_key", product.building_key}, {"node_a", product.node_a},
			{"node_b", product.node_b}, {"piece", product.piece},
			{"n_pieces", product.pieces}, {"edges", std::move(edges)},
			{"col0_m", product.col0_m}, {"cols", product.cols}, {"rows", product.rows},
			{"observed", observed}, {"bands", std::move(bands)},
			{"tier", tier_name(product.tier)}, {"confidence", product.confidence},
			{"height_used_m", product.height_used_m},
			{"unknown_share", product.unknown_share}, {"views", product.views},
			{"flags", product.flags},
			{"has_tex", cells > 0 && !product.texture_rgba.empty()},
			{"reach_m", reach_m}};
	const auto serialized = record.dump();
	if (!write_atomic(json_path,
				std::vector<std::uint8_t>(serialized.begin(), serialized.end()), error))
		return {};
	return key;
}

std::optional<CachedWall> load_wall(
		const std::filesystem::path &directory, const std::string &key)
{
	const auto paths = wall_paths(directory, key);
	if (!paths)
		return std::nullopt;
	const auto &[json_path, png_path, texture_path] = *paths;
	const auto json = read_record(json_path);
	if (!json)
		return std::nullopt;
	try {
		const auto cols = json->at("cols").get<std::uint32_t>();
		const auto rows = json->at("rows").get<std::uint32_t>();
		const std::size_t cells = std::size_t(cols) * rows;
		if ((rows && cells / rows != cols) || cells > max_png_pixels)
			return std::nullopt;
		const auto observed_text = json->at("observed").get<std::string>();
		if (observed_text.size() != cells)
			return std::nullopt;
		WallProduct product;
		product.wall_key = json->at("wall_key").get<std::string>();
		product.building_key = json->at("building_key").get<std::string>();
		product.node_a = json->at("node_a").get<std::int64_t>();
		product.node_b = json->at("node_b").get<std::int64_t>();
		product.piece = json->value("piece", std::size_t{0});
		product.pieces = json->value("n_pieces", std::size_t{1});
		product.col0_m = json->at("col0_m").get<double>();
		product.cols = cols;
		product.rows = rows;
		product.observed.reserve(cells);
		for (const char value : observed_text) {
			if (value != '0' && value != '1')
				return std::nullopt;
			product.observed.push_back(value == '1');
		}
		for (const auto &edge : json->at("edges"))
			product.edges.push_back({edge.at("edge_idx").get<std::size_t>(),
					edge.at("node_a").get<std::int64_t>(),
					edge.at("node_b").get<std::int64_t>(), edge.at("s0").get<double>(),
					edge.at("s1").get<double>()});
		for (const auto &band : json->at("bands"))
			product.bands.push_back(band.get<std::array<std::uint8_t, 3>>());
		product.tier = parse_tier(json->at("tier").get<std::string>());
		product.confidence = json->at("confidence").get<double>();
		product.height_used_m = json->at("height_used_m").get<double>();
		product.unknown_share = json->at("unknown_share").get<double>();
		product.views = json->at("views").get<std::vector<std::string>>();
		product.flags = json->at("flags").get<std::vector<std::string>>();
		if (cells) {
			auto rgba = decode_rgba(png_path, cols, rows);
			if (!rgba)
				return std::nullopt;
			product.rgb.resize(cells);
			product.classes.resize(cells);
			for (std::size_t i = 0; i < cells; ++i) {
				std::copy_n(rgba->begin() + i * 4, 3, product.rgb[i].begin());
				product.classes[i] = (*rgba)[i * 4 + 3];
			}
		}
		if (json->value("has_tex", false)) {
			product.texture_rgba = decode_rgba(
					texture_path, 0, 0, &product.texture_width, &product.texture_height)
										   .value_or(std::vector<std::uint8_t>{});
			if (product.texture_rgba.empty())
				return std::nullopt;
		}
		return CachedWall{std::move(product), json->value("reach_m", 0.0)};
	} catch (...) {
		return std::nullopt;
	}
}
} // namespace arnis::mapillary::cache
