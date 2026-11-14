#include "climate.h"
#include "assets_root.h"
#include "land_cover/land_cover.h"
#include "block_definitions.h"
#include "geo_grid.h"
#include "ground_generation.h"
#include "ecoregion.h"
#include "coordinate_system/geographic/llbbox.h"
#include <png.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <array>
#include <map>
#include <utility>
#include <memory>
#include <mutex>
#include <vector>
namespace arnis::climate
{
namespace
{
constexpr std::size_t KOPPEN_COLS = 3600, KOPPEN_ROWS = 1800;
const std::vector<std::uint8_t> &koppen_grid()
{
	static const std::vector<std::uint8_t> grid = [] {
		const auto path = assets::path("climate/koppen.grid");
		std::ifstream in(path, std::ios::binary);
		if (!in)
			return std::vector<std::uint8_t>{};
		return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(in), {});
	}();
	return grid;
}

const std::optional<geo_grid::TiledGrid> &koppen_tiled_grid()
{
	static const auto grid = geo_grid::TiledGrid::parse(koppen_grid());
	return grid;
}

std::uint8_t koppen_value(std::pair<std::size_t, std::size_t> cell)
{
	const auto &grid = koppen_tiled_grid();
	if (!grid)
		return 0;
	struct Tile
	{
		std::once_flag once;
		std::optional<std::vector<std::uint8_t>> data;
	};
	static const auto tiles = [] {
		std::vector<std::unique_ptr<Tile>> result;
		const auto &parsed = koppen_tiled_grid();
		if (parsed) {
			result.reserve(parsed->tile_count());
			for (std::size_t i = 0; i < parsed->tile_count(); ++i)
				result.push_back(std::make_unique<Tile>());
		}
		return result;
	}();
	const auto index = grid->tile_of(cell);
	if (index >= tiles.size())
		return 0;
	auto &tile = *tiles[index];
	std::call_once(tile.once, [&] { tile.data = grid->decode(index); });
	return tile.data ? static_cast<std::uint8_t>(
							   grid->value(*tile.data, grid->index_in_tile(cell)))
					 : 0;
}

std::uint8_t koppen_class(double latitude, double longitude)
{
	const auto &bytes = koppen_grid();
	const auto &grid = koppen_tiled_grid();
	if (grid) {
		const auto [col, row] = grid->position(latitude, longitude);
		return koppen_value(grid->cell(col, row));
	}
	// Keep accepting older raw assets supplied by library users.
	if (bytes.size() != KOPPEN_COLS * KOPPEN_ROWS)
		return 0;
	// Match TiledGrid::cell: longitude wraps at the dateline, while latitude
	// clamps to the first/last grid row.
	const auto col_unwrapped =
			static_cast<std::int64_t>(std::floor((longitude + 180.) / .1));
	const auto col = static_cast<std::size_t>(
			((col_unwrapped % static_cast<std::int64_t>(KOPPEN_COLS)) +
					static_cast<std::int64_t>(KOPPEN_COLS)) %
			static_cast<std::int64_t>(KOPPEN_COLS));
	const auto row = static_cast<std::size_t>(
			std::clamp<double>(std::floor((90. - latitude) / .1), 0, KOPPEN_ROWS - 1));
	return bytes[row * KOPPEN_COLS + col];
}
}
std::optional<std::pair<Block, Block>> surface_palette(
		arnis::biome::Climate c, std::uint8_t cover, int x, int z, bool red_earth)
{
	using namespace arnis::biome;
	using namespace arnis::land_cover;
	using namespace arnis::block_definitions;
	const bool veg = cover == LC_TREE_COVER || cover == LC_SHRUBLAND ||
					 cover == LC_GRASSLAND || cover == LC_CROPLAND || cover == LC_MOSS;
	const bool bare = cover == LC_BARE || cover == LC_SNOW_ICE;
	if (!veg && !bare)
		return std::nullopt;
	const double n = ground_generation::ground_noise(x, z, 9, 0x00C11A7Eu);
	const auto pick = [n, x, z](const std::initializer_list<
							  std::pair<double, std::pair<Block, Block>>> &bands) {
		for (const auto &band : bands)
			if (n < band.first)
				return std::pair{speckle(band.second.first, x, z), band.second.second};
		const auto &last = *(bands.end() - 1);
		return std::pair{speckle(last.second.first, x, z), last.second.second};
	};
	if (red_earth && cover != LC_SNOW_ICE) {
		return bare ? pick({{.55, {RED_SAND, RED_SANDSTONE}},
							  {.75, {TERRACOTTA, RED_SANDSTONE}},
							  {.87, {COARSE_DIRT, DIRT}},
							  {1., {RED_SANDSTONE, RED_SANDSTONE}}})
					: pick({{.5, {RED_SAND, RED_SANDSTONE}}, {.8, {COARSE_DIRT, DIRT}},
							  {.9, {GRASS_BLOCK, DIRT}},
							  {1., {TERRACOTTA, RED_SANDSTONE}}});
	}
	switch (c) {
	case Climate::IceCap:
		return ground_generation::patch_noise(x, z, 20, 0x001C'ECA9u) < .17
					   ? std::pair{PACKED_ICE, PACKED_ICE}
					   : std::pair{SNOW_BLOCK, SNOW_BLOCK};
	case Climate::HotDesert:
		if (ground_generation::ground_noise(x, z, 12, 0x00DE5E27u) < .06)
			return std::pair{SANDSTONE, SANDSTONE};
		return std::pair{speckle(SAND, x, z), SANDSTONE};
	case Climate::HotSteppe:
		return bare ? pick({{.35, {SAND, SANDSTONE}}, {1., {COARSE_DIRT, DIRT}}})
					: pick({{.15, {SAND, SANDSTONE}}, {.5, {COARSE_DIRT, DIRT}},
							  {1., {GRASS_BLOCK, DIRT}}});
	case Climate::ColdDesert:
		return bare ? pick({{.42, {GRAVEL, STONE}}, {.75, {COARSE_DIRT, DIRT}},
							  {1., {STONE, STONE}}})
					: pick({{.5, {COARSE_DIRT, DIRT}}, {.8, {GRAVEL, STONE}},
							  {1., {GRASS_BLOCK, DIRT}}});
	case Climate::ColdSteppe:
		return bare ? pick({{.6, {COARSE_DIRT, DIRT}}, {1., {GRAVEL, STONE}}})
					: pick({{.3, {COARSE_DIRT, DIRT}}, {1., {GRASS_BLOCK, DIRT}}});
	case Climate::Boreal:
		return bare ? pick({{.5, {COARSE_DIRT, DIRT}}, {1., {GRAVEL, STONE}}})
					: pick({{.4, {PODZOL, DIRT}}, {.6, {COARSE_DIRT, DIRT}},
							  {1., {GRASS_BLOCK, DIRT}}});
	case Climate::Tundra:
		return bare ? pick({{.5, {GRAVEL, STONE}}, {.8, {COARSE_DIRT, DIRT}},
							  {1., {STONE, STONE}}})
					: pick({{.4, {COARSE_DIRT, DIRT}}, {.6, {MOSS_BLOCK, DIRT}},
							  {1., {GRASS_BLOCK, DIRT}}});
	case Climate::TropicalSavanna:
		return std::nullopt;
	case Climate::Temperate:
	case Climate::DryContinental:
		return std::nullopt;
	}
	return std::nullopt;
}

Block speckle(Block block, int x, int z)
{
	using namespace arnis::block_definitions;
	const double n = ground_generation::value_noise_salted(x, z, 2, 0x5EC17A3Bu);
	if (block == GRAVEL && n < .21)
		return ANDESITE;
	if (block == GRAVEL && n > .86)
		return COBBLESTONE;
	if (block == STONE && n < .23)
		return ANDESITE;
	if (block == STONE && n > .85)
		return TUFF;
	if (block == COARSE_DIRT && n < .21)
		return DIRT;
	if (block == SAND && n > .89)
		return SMOOTH_SANDSTONE;
	if (block == RED_SAND && n < .17)
		return TERRACOTTA;
	if (block == RED_SAND && n > .87)
		return RED_SANDSTONE;
	if (block == TERRACOTTA && n > .84)
		return ORANGE_TERRACOTTA;
	return block;
}
arnis::biome::Climate from_koppen_class(unsigned char c)
{
	switch (c) {
	case 3:
		return arnis::biome::Climate::TropicalSavanna;
	case 4:
		return arnis::biome::Climate::HotDesert;
	case 5:
		return arnis::biome::Climate::ColdDesert;
	case 6:
		return arnis::biome::Climate::HotSteppe;
	case 7:
		return arnis::biome::Climate::ColdSteppe;
	case 17:
	case 18:
		return arnis::biome::Climate::DryContinental;
	case 19:
	case 20:
	case 23:
	case 24:
	case 27:
	case 28:
		return arnis::biome::Climate::Boreal;
	case 29:
		return arnis::biome::Climate::Tundra;
	case 30:
		return arnis::biome::Climate::IceCap;
	default:
		return arnis::biome::Climate::Temperate;
	}
}
arnis::biome::Climate classify(double latitude, double longitude)
{
	if (!std::isfinite(latitude) || !std::isfinite(longitude))
		return arnis::biome::Climate::Temperate;
	return from_koppen_class(koppen_class(latitude, longitude));
}
double koppen_cell_degrees()
{
	const auto &grid = koppen_tiled_grid();
	return grid && grid->cells_per_degree() > 0.0 ? 1.0 / grid->cells_per_degree() : 0.1;
}

std::optional<std::string> render_climate_map(const std::filesystem::path &prefix,
		const geographic::LLBBox &bbox, std::string *error)
{
	constexpr std::uint32_t max_side = 512;
	const double south = bbox.min().lat(), north = bbox.max().lat();
	const double west = bbox.min().lng(), east = bbox.max().lng();
	const double ground_width =
			(east - west) * std::cos(((south + north) * 0.5) * M_PI / 180.0);
	const double ground_height = north - south;
	const double longest = std::max(ground_width, ground_height);
	auto side = [longest](double length) -> std::uint32_t {
		if (!(longest > 0.0) || !std::isfinite(longest) || !std::isfinite(length))
			return 1;
		return std::max<std::uint32_t>(
				1, static_cast<std::uint32_t>(
						   std::clamp(std::round(double(max_side) * length / longest),
								   1.0, double(max_side))));
	};
	const std::uint32_t width = side(ground_width), height = side(ground_height);
	auto color = [](arnis::biome::Climate climate) {
		using Climate = arnis::biome::Climate;
		switch (climate) {
		case Climate::Temperate:
			return std::pair{std::array<std::uint8_t, 3>{120, 180, 90}, "temperate"};
		case Climate::TropicalSavanna:
			return std::pair{
					std::array<std::uint8_t, 3>{170, 190, 80}, "tropical_savanna"};
		case Climate::HotDesert:
			return std::pair{std::array<std::uint8_t, 3>{237, 201, 120}, "hot_desert"};
		case Climate::HotSteppe:
			return std::pair{std::array<std::uint8_t, 3>{214, 178, 110}, "hot_steppe"};
		case Climate::ColdDesert:
			return std::pair{std::array<std::uint8_t, 3>{200, 190, 160}, "cold_desert"};
		case Climate::ColdSteppe:
			return std::pair{std::array<std::uint8_t, 3>{190, 185, 140}, "cold_steppe"};
		case Climate::DryContinental:
			return std::pair{
					std::array<std::uint8_t, 3>{200, 140, 80}, "dry_continental"};
		case Climate::Boreal:
			return std::pair{std::array<std::uint8_t, 3>{80, 150, 120}, "boreal"};
		case Climate::Tundra:
			return std::pair{std::array<std::uint8_t, 3>{170, 200, 205}, "tundra"};
		case Climate::IceCap:
			return std::pair{std::array<std::uint8_t, 3>{240, 248, 255}, "ice_cap"};
		}
		return std::pair{std::array<std::uint8_t, 3>{120, 180, 90}, "temperate"};
	};
	std::vector<std::uint8_t> pixels(std::size_t(width) * height * 3);
	std::map<std::string, std::uint32_t> counts;
	for (std::uint32_t y = 0; y < height; ++y) {
		const double latitude = north - (double(y) + 0.5) / height * ground_height;
		for (std::uint32_t x = 0; x < width; ++x) {
			const double longitude = west + (double(x) + 0.5) / width * (east - west);
			const auto [rgb, name] = color(classify_warped(latitude, longitude));
			++counts[name];
			const auto offset = (std::size_t(y) * width + x) * 3;
			std::copy(rgb.begin(), rgb.end(), pixels.begin() + offset);
		}
	}
	const auto path = std::filesystem::path(prefix.string() + ".png");
	png_image image{};
	image.version = PNG_IMAGE_VERSION;
	image.width = width;
	image.height = height;
	image.format = PNG_FORMAT_RGB;
	if (!png_image_write_to_file(
				&image, path.string().c_str(), 0, pixels.data(), 0, nullptr)) {
		if (error)
			*error = "write " + path.string() + ": " + image.message;
		return std::nullopt;
	}
	const double total = double(width) * height;
	nlohmann::json shares = nlohmann::json::object();
	for (const auto &[name, count] : counts) {
		const double percent = std::round(double(count) * 1000.0 / total) / 10.0;
		shares[name] = percent;
	}
	return nlohmann::json{{"shares", std::move(shares)}, {"_file", path.string()},
			{"_size", {width, height}}}
			.dump();
}

arnis::biome::Climate classify_warped(double latitude, double longitude)
{
	const auto &grid = koppen_tiled_grid();
	if (!grid || !std::isfinite(latitude) || !std::isfinite(longitude))
		return arnis::biome::Climate::Temperate;
	return from_koppen_class(static_cast<unsigned char>(
			koppen_value(ecoregion::warped_cell(*grid, latitude, longitude))));
}
arnis::biome::Climate classify_bbox(double min_latitude, double min_longitude,
		double max_latitude, double max_longitude)
{
	return classify(
			(min_latitude + max_latitude) * .5, (min_longitude + max_longitude) * .5);
}

ClimateField::ClimateField(const GeoFunction &geo, std::pair<int, int> alignment,
		std::size_t world_width, std::size_t world_height) : geo_(geo)
{
	const double lon_per_block = std::abs(geo_(1.0, 0.0).second - geo_(0.0, 0.0).second);
	const double step = koppen_cell_degrees() / std::max(lon_per_block, 1e-12) / 64.0;
	const int bounded_step = std::clamp(static_cast<int>(step), 4, 32);
	cell_ = 1;
	while (cell_ <= bounded_step / 2)
		cell_ *= 2;
	const auto rem = [](int value, int divisor) {
		const int r = value % divisor;
		return r < 0 ? r + divisor : r;
	};
	origin_x_ = -rem(alignment.first, cell_);
	origin_z_ = -rem(alignment.second, cell_);
	const auto span = [&](std::size_t len, int from) {
		const auto numerator = static_cast<std::int64_t>(len) - from + cell_ - 1;
		return static_cast<std::size_t>(std::max<std::int64_t>(1, numerator / cell_));
	};
	width_ = span(world_width, origin_x_);
	height_ = span(world_height, origin_z_);
	climates_.reserve(width_ * height_);
	const double half = cell_ / 2.0;
	for (std::size_t i = 0; i < width_ * height_; ++i) {
		const double gx = origin_x_ + static_cast<double>(i % width_) * cell_ + half;
		const double gz = origin_z_ + static_cast<double>(i / width_) * cell_ + half;
		const auto [lat, lon] = geo_(gx, gz);
		climates_.push_back(classify_warped(lat, lon));
	}
}

arnis::biome::Climate ClimateField::climate(int x, int z) const
{
	const auto index = [&](int value, int origin, std::size_t len) {
		const int delta = value - origin;
		const int quotient = delta >= 0 ? delta / cell_ : (delta - cell_ + 1) / cell_;
		return static_cast<std::size_t>(
				std::clamp(quotient, 0, static_cast<int>(len) - 1));
	};
	return climates_[index(z, origin_z_, height_) * width_ + index(x, origin_x_, width_)];
}

double ClimateField::latitude(int x, int z) const
{
	return geo_(x + 0.5, z + 0.5).first;
}
}
