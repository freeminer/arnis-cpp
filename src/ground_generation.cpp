#include "ground_generation.h"

#include "block_definitions.h"
#include "element_processing/tree.h"
#include "land_cover/land_cover.h"
#include "trees/schematic.h"
#include "climate.h"
#include "deterministic_rng.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace arnis::ground_generation
{

namespace
{

Block rocky_surface_for(int x, int z)
{
	const auto h = land_cover::coord_hash(x, z) % 12;
	if (h <= 4)
		return STONE;
	if (h <= 6)
		return ANDESITE;
	if (h <= 8)
		return COBBLESTONE;
	if (h == 9)
		return GRAVEL;
	if (h == 10)
		return TUFF;
	return COARSE_DIRT;
}

double value_noise_01_impl(int x, int z, int scale)
{
	const int s = std::max(1, scale);
	const auto floor_to = [s](int v) { return (v >= 0 ? v / s : -(((-v) + s - 1) / s)) * s; };
	const int x0 = floor_to(x), z0 = floor_to(z), x1 = x0 + s, z1 = z0 + s;
	const double tx = double(x - x0) / s, tz = double(z - z0) / s;
	const auto smooth=[](double t){ return t*t*(3.0-2.0*t); };
	const double fx=smooth(tx), fz=smooth(tz);
	const auto sample=[](int sx,int sz){return double(land_cover::coord_hash(sx,sz)%1000)/1000.0;};
	const double a=sample(x0,z0)*(1-fx)+sample(x1,z0)*fx;
	const double b=sample(x0,z1)*(1-fx)+sample(x1,z1)*fx;
	return a*(1-fz)+b*fz;
}

std::pair<Block, Block> slope_palette(int slope, int x, int z)
{
	const auto h=land_cover::coord_hash(x,z);
	if(slope>8) return h%2 ? std::make_pair(DEEPSLATE,DEEPSLATE) : std::make_pair(COBBLED_DEEPSLATE,COBBLED_DEEPSLATE);
	if(slope>6) { const auto k=h%20; return k<12?std::make_pair(STONE,DEEPSLATE):k<17?std::make_pair(COBBLESTONE,DEEPSLATE):std::make_pair(ANDESITE,DEEPSLATE); }
	if(slope>4) { switch(h%12) { case 0: case 1: case 2: case 3:return {ANDESITE,STONE}; case 4: case 5:return {TUFF,STONE}; case 6: case 7:return {STONE,STONE}; case 8: case 9:return {COBBLESTONE,STONE}; default:return {GRAVEL,STONE}; } }
	return {GRASS_BLOCK,DIRT};
}

int local_slope(WorldEditor &editor, int x, int z)
{
	const int center = editor.get_ground_level(x, z);
	int max_delta = 0;
	for (int dx = -1; dx <= 1; ++dx) {
		for (int dz = -1; dz <= 1; ++dz) {
			if (dx == 0 && dz == 0)
				continue;
			max_delta = std::max(max_delta,
					std::abs(editor.get_ground_level(x + dx, z + dz) - center));
		}
	}
	return max_delta;
}

bool has_nearby_water(WorldEditor &editor, int x, int ground_y, int z)
{
	for (int dx = -2; dx <= 2; ++dx) {
		for (int dz = -2; dz <= 2; ++dz) {
			for (int dy = -1; dy <= 1; ++dy) {
				if (editor.check_for_block_absolute(x + dx, ground_y + dy, z + dz,
							std::optional<std::vector<Block>>(
									std::vector<Block>{WATER}))) {
					return true;
				}
			}
		}
	}
	return false;
}

bool is_protected_surface(WorldEditor &editor, int x, int y, int z)
{
	return editor.check_for_block_absolute(x, y, z,
			std::optional<std::vector<Block>>(std::vector<Block>{
					BLACK_CONCRETE,
					GRAY_CONCRETE_POWDER,
					CYAN_TERRACOTTA,
					GRAY_CONCRETE,
					LIGHT_GRAY_CONCRETE,
					WHITE_CONCRETE,
					DIRT_PATH,
					SMOOTH_STONE,
					WATER,
			}));
}

bool is_replaceable_surface(WorldEditor &editor, int x, int y, int z)
{
	// Match the Rust natural-surface palette while protecting authored OSM blocks.
	return !editor.block_exists_absolute(x, y, z) ||
		   editor.check_for_block_absolute(x, y, z,
				   std::optional<std::vector<Block>>(std::vector<Block>{
						   STONE,
						   DIRT,
						   GRASS_BLOCK,
						   GRASS,
						   SAND,
						   SANDSTONE,
						   GRAVEL,
						   CLAY,
						   COARSE_DIRT,
						   PODZOL,
						   MUD,
						   ANDESITE,
						   COBBLESTONE,
						   TUFF,
						   DEEPSLATE,
						   COBBLED_DEEPSLATE,
						   MOSS_BLOCK,
						   SNOW_BLOCK,
						   ICE,
						   PACKED_ICE,
						   BLACKSTONE,
				   }));
}

Block natural_surface_for(WorldEditor &editor, int x, int ground_y, int z)
{
	const auto cover = editor.ground
							   ? editor.ground->cover_class({x - editor.mg->node_min.X,
										 z - editor.mg->node_min.Z})
							   : 0;
	const int slope = local_slope(editor, x, z);
	if (slope>4) return slope_palette(slope,x,z).first;
	if(editor.ground) {
		auto climate_palette=climate::surface_palette(editor.ground->climate(),cover,x,z);
		if(climate_palette) return climate_palette->first;
	}
	if (cover == land_cover::LC_CROPLAND) return FARMLAND;
	if (cover == land_cover::LC_BUILT_UP) { const auto h=land_cover::coord_hash(x,z)%100; return h<72?STONE_BRICKS:h<87?CRACKED_STONE_BRICKS:h<92?STONE:COBBLESTONE; }
	if (cover == land_cover::LC_BARE || cover == land_cover::LC_SNOW_ICE) {
		int nearby=0; if(editor.ground) for(auto [dx,dz]:std::vector<std::pair<int,int>>{{-1,0},{1,0},{0,-1},{0,1}}) { auto c=editor.ground->cover_class({x+dx-editor.mg->node_min.X,z+dz-editor.mg->node_min.Z});nearby+=c==land_cover::LC_BARE||c==land_cover::LC_SNOW_ICE; }
		if(!nearby) return GRASS_BLOCK;
		const auto h=land_cover::coord_hash(x,z); if(value_noise_01(x,z,6)<.45) return h%10<8?COARSE_DIRT:STONE; return rocky_surface_for(x,z);
	}
	if (cover == land_cover::LC_WETLAND || cover == land_cover::LC_MANGROVES) return MUD;
	if (cover == land_cover::LC_SHRUBLAND) return value_noise_01(x,z,5)<.4 && land_cover::coord_hash(x,z)%5 ? COARSE_DIRT : GRASS_BLOCK;
	if (has_nearby_water(editor, x, ground_y, z))
		return SAND;

	const auto h = land_cover::coord_hash(x, z) % 100;
	if (h < 2)
		return COARSE_DIRT;
	if (h < 5)
		return PODZOL;
	return GRASS_BLOCK;
}

std::optional<bool> canopy_tree_verdict(WorldEditor &editor, int x, int z,
		int origin_x, int origin_z)
{
	if (!editor.ground || !editor.ground->has_canopy())
		return std::nullopt;
	const int spacing = std::max(1, editor.get_tree_slot_spacing());
	const auto floor_div = [spacing](int v) {
		return v >= 0 ? v / spacing : -(((-v) + spacing - 1) / spacing);
	};
	const int cell_x = floor_div(x) * spacing;
	const int cell_z = floor_div(z) * spacing;
	const auto fraction = editor.ground->canopy_fraction(
			XZPoint{cell_x - origin_x, cell_z - origin_z}, spacing);
	if (!fraction)
		return std::nullopt;
	const auto [slot_x, slot_z] = trees::trunk_slot_s(x, z, spacing);
	if (x != slot_x || z != slot_z)
		return false;
	const double p = canopy::slot_probability(*fraction, spacing, editor.place_schematics);
	const double roll = double(land_cover::coord_hash(x ^ 0x434d, z ^ 0x484d) % 10000) /
			10000.0;
	return roll < p;
}

void maybe_place_vegetation(WorldEditor &editor, int x, int ground_y, int z,
		const BuildingFootprintBitmap &building_footprints, int origin_x, int origin_z)
{
	if (building_footprints.contains(x, z) ||
			editor.check_for_block_absolute(x, ground_y + 1, z))
		return;
	if (!editor.check_for_block_absolute(x, ground_y, z,
			std::optional<std::vector<Block>>(
					std::vector<Block>{GRASS_BLOCK, PODZOL, COARSE_DIRT, DIRT, MUD, FARMLAND})))
		return;

	const auto cover=editor.ground?editor.ground->cover_class({x-editor.mg->node_min.X,z-editor.mg->node_min.Z}):0;
	const auto h = land_cover::coord_hash(x, z);
	if (cover==land_cover::LC_TREE_COVER) {
		// Measured canopy owns density where available.  On no-data cells retain
		// the old land-cover-only probability, matching Rust's fallback contract.
		const auto canopy_wants_tree = canopy_tree_verdict(editor, x, z, origin_x, origin_z);
		if (canopy_wants_tree.value_or(h % 30 == 0)) {
			if (!editor.place_regional_tree(x, ground_y + 1, z, cover))
				Tree::create(editor, Coord{x, 1, z}, &building_footprints);
		}
	} else if (cover==land_cover::LC_CROPLAND) {
		// The Rust pass uses deterministic crop/irrigation choices.  Keep water
		// only at the sparse lattice; arbitrary irrigation would flow through a
		// library host's terrain policy.
		if(x%9==0 && z%9==0) editor.set_block_absolute(WATER,x,ground_y,z,
				std::optional<std::vector<Block>>(std::vector<Block>{FARMLAND}),std::nullopt);
		else if(h%76==0 && h%10<4) editor.set_block_absolute(HAY_BALE,x,ground_y+1,z,std::nullopt,std::nullopt);
		else { const Block crop=(h%3==0?WHEAT:h%3==1?CARROTS:POTATOES);editor.set_block_absolute(crop,x,ground_y+1,z,std::nullopt,std::nullopt); }
	} else if ((cover==land_cover::LC_WETLAND || cover==land_cover::LC_MANGROVES) && h%100<30) {
		editor.set_block_absolute(WATER,x,ground_y,z,
				std::optional<std::vector<Block>>(std::vector<Block>{MUD,GRASS_BLOCK}),std::nullopt);
	} else if ((cover==land_cover::LC_WETLAND || cover==land_cover::LC_MANGROVES) && h%100<75) {
		editor.set_block_absolute(GRASS,x,ground_y+1,z,std::nullopt,std::nullopt);
	} else if (cover==land_cover::LC_BARE && h%100==0) {
		editor.set_block_absolute(DEAD_BUSH,x,ground_y+1,z,std::nullopt,std::nullopt);
	} else if (cover==land_cover::LC_SHRUBLAND && h%100<2) {
		editor.set_block_absolute(
				OAK_LEAVES, x, ground_y + 1, z, std::nullopt, std::nullopt);
	} else if (cover==land_cover::LC_GRASSLAND && h%100==55) {
		const Block flower = h % 4 == 0	  ? RED_FLOWER
							 : h % 4 == 1 ? BLUE_FLOWER
							 : h % 4 == 2 ? YELLOW_FLOWER
										  : WHITE_FLOWER;
		editor.set_block_absolute(flower, x, ground_y + 1, z, std::nullopt, std::nullopt);
	} else if ((cover==land_cover::LC_GRASSLAND && h%100<55) ||
			(cover==land_cover::LC_SHRUBLAND && h%100<30) ||
			(cover==land_cover::LC_TREE_COVER && h%30<=13)) {
		editor.set_block_absolute(GRASS, x, ground_y + 1, z, std::nullopt, std::nullopt);
	}
}

void clear_road_vegetation(WorldEditor &editor,int x,int y,int z)
{
	const std::vector<Block> roads{BLACK_CONCRETE,GRAY_CONCRETE_POWDER,CYAN_TERRACOTTA,
		GRAY_CONCRETE,LIGHT_GRAY_CONCRETE,WHITE_CONCRETE,DIRT_PATH};
	const std::vector<Block> vegetation{GRASS,OAK_LEAVES,DEAD_BUSH,TALL_GRASS_BOTTOM,
		RED_FLOWER,BLUE_FLOWER,WHITE_FLOWER,YELLOW_FLOWER};
	if(!editor.check_for_block_absolute(x,y,z,roads) ||
			!editor.check_for_block_absolute(x,y+1,z,vegetation)) return;
	editor.set_block_absolute(AIR,x,y+1,z,vegetation,std::nullopt);
	if(editor.check_for_block_absolute(x,y+2,z,std::optional<std::vector<Block>>(std::vector<Block>{TALL_GRASS_TOP})))
		editor.set_block_absolute(AIR,x,y+2,z,std::nullopt,std::nullopt);
}

}

double value_noise_01(int x, int z, int scale)
{
	return value_noise_01_impl(x, z, scale);
}

void generate_ground_region(WorldEditor &editor, const Args &args, const XZBBox &xzbbox,
		const BuildingFootprintBitmap &building_footprints, int iter_min_x, int iter_max_x,
		int iter_min_z, int iter_max_z, const CoordinateBitmap *tunnel_footprint)
{
	// Rust parity: src/ground_generation.rs::generate_ground_layer ordering.
	// xzbbox remains the shared-grid origin; callers may supply strict tile
	// bounds so streamed passes sample the same land-cover/canopy cells.
	const int min_x = std::max(iter_min_x, xzbbox.min_x());
	const int max_x = std::min(iter_max_x, xzbbox.max_x());
	const int min_z = std::max(iter_min_z, xzbbox.min_z());
	const int max_z = std::min(iter_max_z, xzbbox.max_z());
	if (min_x > max_x || min_z > max_z)
		return;
	for (int x = min_x; x <= max_x; ++x) {
		for (int z = min_z; z <= max_z; ++z) {
			const bool in_tunnel = tunnel_footprint && tunnel_footprint->contains(x, z);
			const int ground_y = editor.get_ground_level(x, z);
			const int slope = local_slope(editor, x, z);
			const auto relative = XZPoint{x - xzbbox.min_x(), z - xzbbox.min_z()};
			const bool has_cover = editor.ground && editor.ground->has_land_cover();
			const double water_blend = has_cover ? editor.ground->water_blend(relative) : 0.0;
			const bool grid_water = has_cover && editor.ground->water_distance(relative) > 0;
			const bool existing_water = editor.check_for_block_absolute(x, ground_y, z,
					std::optional<std::vector<Block>>(std::vector<Block>{WATER}));

			// Rust uses a smoothed ESA water mask, but never retracts a hard water
			// cell.  Keep OSM water too, and avoid flooding cliff faces.
			if (!in_tunnel && (grid_water || existing_water || water_blend > .5) && slope <= 4) {
				const int water_y = editor.get_water_level(x, z);
				if (ground_y <= water_y && !is_protected_surface(editor, x, water_y, z)) {
					editor.set_block_absolute(
							WATER, x, water_y, z, std::nullopt, std::nullopt);
					if (water_y - 1 > -64)
						editor.set_block_absolute(
								SAND, x, water_y - 1, z, std::nullopt, std::nullopt);
					if (water_y - 2 > -64)
						editor.set_block_absolute(
								SANDSTONE, x, water_y - 2, z, std::nullopt, std::nullopt);
				}
				continue;
			}

			if (!in_tunnel && !is_protected_surface(editor, x, ground_y, z) &&
					is_replaceable_surface(editor, x, ground_y, z)) {
				const Block surface = natural_surface_for(editor, x, ground_y, z);
				std::optional<Block> climate_under;
				if (has_cover && editor.ground) {
					auto palette=climate::surface_palette(editor.ground->climate(),editor.ground->cover_class(relative),x,z);
					if(palette) climate_under=palette->second;
				}
				editor.set_block_absolute(
						surface, x, ground_y, z, std::nullopt, std::nullopt);

				if (climate_under) {
					editor.set_block_absolute(*climate_under, x, ground_y - 1, z, std::nullopt, std::nullopt);
				} else if (surface == SAND) {
					editor.set_block_absolute(
							SANDSTONE, x, ground_y - 1, z, std::nullopt, std::nullopt);
				} else if (surface == STONE || surface == ANDESITE ||
						   surface == COBBLESTONE || surface == TUFF) {
					editor.set_block_absolute(
							STONE, x, ground_y - 1, z, std::nullopt, std::nullopt);
				} else {
					editor.set_block_absolute(
							DIRT, x, ground_y - 1, z, std::nullopt, std::nullopt);
				}
			}

			if (!in_tunnel)
				maybe_place_vegetation(editor, x, ground_y, z, building_footprints,
						xzbbox.min_x(), xzbbox.min_z());

			// Rust's universal depth pass closes visible gaps below all terrain
			// columns, including ones whose surface was supplied by OSM.
			if (!in_tunnel && !editor.check_for_block_absolute(x, ground_y, z,
					std::optional<std::vector<Block>>(std::vector<Block>{WATER}))) {
				int lowest=ground_y;
				for (int dx=-1;dx<=1;++dx) for (int dz=-1;dz<=1;++dz)
					if(dx||dz) lowest=std::min(lowest,editor.get_ground_level(x+dx,z+dz));
				const int depth=std::clamp(ground_y-lowest+1,2,64);
				editor.fill_column_absolute(STONE,x,z,std::max(-63,ground_y-depth),ground_y-1,true);
			}

			// Snow is a separate cap, so climate/land-cover material selection is
			// preserved below it just as in the Rust ground pass.
			if (!in_tunnel && editor.ground && editor.ground->snow_capped(ground_y) && water_blend<=.5 &&
					!editor.check_for_block_absolute(x,ground_y,z,std::optional<std::vector<Block>>(std::vector<Block>{WATER})))
				editor.set_block_if_absent_absolute(SNOW_LAYER,x,ground_y+1,z);

			if (!in_tunnel)
				clear_road_vegetation(editor,x,ground_y,z);

			if (!in_tunnel && args.fillground) {
				const int min_fill_y = std::max(-64, ground_y - 32);
				for (int y = min_fill_y; y < ground_y; ++y) {
					if (!editor.check_for_block_absolute(x, y, z))
						editor.set_block_absolute(y < ground_y - 8 ? STONE : DIRT, x, y,
								z, std::nullopt, std::nullopt);
				}
				editor.set_block_absolute(
						BEDROCK, x, min_fill_y - 1, z, std::nullopt, std::nullopt);
			}
		}
	}
}

void generate_ground_layer(WorldEditor &editor, const Args &args, const XZBBox &xzbbox,
		const BuildingFootprintBitmap &building_footprints,
		const CoordinateBitmap *tunnel_footprint)
{
	generate_ground_region(editor, args, xzbbox, building_footprints, xzbbox.min_x(),
			xzbbox.max_x(), xzbbox.min_z(), xzbbox.max_z(), tunnel_footprint);
}

}
