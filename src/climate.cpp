#include "climate.h"
#include "assets_root.h"
#include "land_cover/land_cover.h"
#include "block_definitions.h"
#include "ground_generation.h"
#include "deterministic_rng.h"
#include "geo_grid.h"
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
	const auto col =
			std::clamp<double>(std::floor((longitude + 180.) / .1), 0, KOPPEN_COLS - 1);
	const auto row =
			std::clamp<double>(std::floor((90. - latitude) / .1), 0, KOPPEN_ROWS - 1);
	return bytes[std::size_t(row) * KOPPEN_COLS + std::size_t(col)];
}
}
std::optional<std::pair<Block, Block>> surface_palette(
		arnis::biome::Climate c, std::uint8_t cover, int x, int z)
{
	using namespace arnis::biome;
	using namespace arnis::land_cover;
	using namespace arnis::block_definitions;
	if (c == Climate::Temperate || c == Climate::TropicalSavanna ||
			c == Climate::DryContinental)
		return std::nullopt;
	const bool veg = cover == LC_TREE_COVER || cover == LC_SHRUBLAND ||
					 cover == LC_GRASSLAND || cover == LC_CROPLAND || cover == LC_MOSS,
			   bare = cover == LC_BARE || cover == LC_SNOW_ICE;
	if (!veg && !bare)
		return std::nullopt;
	const auto n = ground_generation::patch_noise(x, z, 9, 0x00C11A7Eu);
	const auto pick =
			[n](std::initializer_list<std::pair<double, std::pair<Block, Block>>> bands) {
				for (const auto &band : bands)
					if (n < band.first)
						return band.second;
				return bands.end()[-1].second;
			};
	if (c == Climate::IceCap)
		return ground_generation::patch_noise(x, z, 20, 0x001CECA9u) < .17
					   ? std::make_pair(PACKED_ICE, PACKED_ICE)
					   : std::make_pair(SNOW_BLOCK, SNOW_BLOCK);
	if (c == Climate::HotDesert) {
		if (ground_generation::patch_noise(x, z, 12, 0x00DE5E27u) < .06)
			return std::make_pair(SANDSTONE, SANDSTONE);
		return coord_hash(x, z) % 40 == 0 ? std::make_pair(SMOOTH_SANDSTONE, SANDSTONE)
										  : std::make_pair(SAND, SANDSTONE);
	}
	if (c == Climate::HotSteppe)
		return bare ? pick({{.5, {SAND, SANDSTONE}}, {1., {COARSE_DIRT, DIRT}}})
					: pick({{.3, {SAND, SANDSTONE}}, {.6, {COARSE_DIRT, DIRT}},
							  {1., {GRASS_BLOCK, DIRT}}});
	if (c == Climate::ColdDesert)
		return bare ? pick({{.42, {GRAVEL, STONE}}, {.75, {COARSE_DIRT, DIRT}},
							  {1., {STONE, STONE}}})
					: pick({{.5, {COARSE_DIRT, DIRT}}, {.8, {GRAVEL, STONE}},
							  {1., {GRASS_BLOCK, DIRT}}});
	if (c == Climate::ColdSteppe)
		return bare ? pick({{.6, {COARSE_DIRT, DIRT}}, {1., {GRAVEL, STONE}}})
					: pick({{.3, {COARSE_DIRT, DIRT}}, {1., {GRASS_BLOCK, DIRT}}});
	if (c == Climate::Boreal)
		return bare ? pick({{.5, {COARSE_DIRT, DIRT}}, {1., {GRAVEL, STONE}}})
					: pick({{.4, {PODZOL, DIRT}}, {.6, {COARSE_DIRT, DIRT}},
							  {1., {GRASS_BLOCK, DIRT}}});
	if (c == Climate::Tundra)
		return bare ? pick({{.5, {GRAVEL, STONE}}, {.8, {COARSE_DIRT, DIRT}},
							  {1., {STONE, STONE}}})
					: pick({{.4, {COARSE_DIRT, DIRT}}, {.6, {MOSS_BLOCK, DIRT}},
							  {1., {GRASS_BLOCK, DIRT}}});
	return std::nullopt;
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
	case 21:
	case 22:
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
