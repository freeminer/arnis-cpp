#include "climate.h"
#include "assets_root.h"
#include "land_cover/land_cover.h"
#include "block_definitions.h"
#include "geo_grid.h"
#include "ground_generation.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <utility>
#include <memory>
#include <mutex>
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

std::uint8_t koppen_class(double latitude, double longitude)
{
	const auto &bytes = koppen_grid();
	static const auto grid = geo_grid::TiledGrid::parse(bytes);
	if (grid) {
		struct Tile
		{
			std::once_flag once;
			std::optional<std::vector<std::uint8_t>> data;
		};
		static const auto tiles = [] {
			std::vector<std::unique_ptr<Tile>> result;
			result.reserve(grid->tile_count());
			for (std::size_t i = 0; i < grid->tile_count(); ++i)
				result.push_back(std::make_unique<Tile>());
			return result;
		}();
		const auto [col, row] = grid->position(latitude, longitude);
		const auto cell = grid->cell(col, row);
		const auto index = grid->tile_of(cell);
		if (index >= tiles.size())
			return 0;
		auto &tile = *tiles[index];
		std::call_once(tile.once, [&] { tile.data = grid->decode(index); });
		return tile.data ? static_cast<std::uint8_t>(
								   grid->value(*tile.data, grid->index_in_tile(cell)))
						 : 0;
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
arnis::biome::Climate classify_bbox(double min_latitude, double min_longitude,
		double max_latitude, double max_longitude)
{
	return classify(
			(min_latitude + max_latitude) * .5, (min_longitude + max_longitude) * .5);
}
}
