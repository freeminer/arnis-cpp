#include <algorithm>
#include <array>
#include <vector>
#include <string>
#include <unordered_map>
#include <optional>
#include <tuple>
#include <cstdlib>
#include <initializer_list>

#include "../../../arnis_adapter.h"
#include "../floodfill.h"
#include "../floodfill_cache.h"
#include "../structures/structures.h"
#include "surfaces.h"
#include "signage.h"

namespace arnis
{

namespace amenities
{

std::vector<std::tuple<std::string, int, int>> recycling_items(
		const std::unordered_map<std::string, std::string> &tags, int x, int z)
{
	std::vector<std::string> pool;
	for (const auto &[tag, item] :
			std::initializer_list<std::pair<const char *, const char *>>{
					{"recycling:glass_bottles", "minecraft:glass_bottle"},
					{"recycling:paper", "minecraft:paper"},
					{"recycling:glass", "minecraft:glass"},
					{"recycling:glass", "minecraft:glass_pane"},
					{"recycling:clothes", "minecraft:leather_chestplate"},
					{"recycling:shoes", "minecraft:leather_boots"},
					{"recycling:cans", "minecraft:bucket"},
					{"recycling:scrap_metal", "minecraft:iron_ingot"},
					{"recycling:green_waste", "minecraft:oak_sapling"}})
		if (auto it = tags.find(tag); it != tags.end() && it->second == "yes")
			pool.emplace_back(item);
	if (pool.empty())
		return {};
	auto rng = coord_rng(x, z, 0xBA773ULL);
	std::vector<std::tuple<std::string, int, int>> items;
	for (int slot = 0; slot < 27; ++slot)
		if (rng() % 5 == 0) {
			std::string item = pool[rng() % pool.size()];
			if (item == "minecraft:iron_ingot") {
				const std::array<const char *, 3> metals{{"minecraft:copper_ingot",
						"minecraft:iron_ingot", "minecraft:gold_ingot"}};
				item = metals[rng() % metals.size()];
			} else if (item == "minecraft:oak_sapling") {
				const std::array<const char *, 5> green{{"minecraft:oak_sapling",
						"minecraft:birch_sapling", "minecraft:spruce_sapling",
						"minecraft:tall_grass", "minecraft:wheat_seeds"}};
				item = green[rng() % green.size()];
			}
			items.emplace_back(std::move(item), slot, 1 + rng() % 4);
		}
	return items;
}

bool place_furniture_decals(WorldEditor &editor,
		const std::unordered_map<std::string, std::string> &tags, int x, int y, int z)
{
	tags_t signage_tags;
	signage_tags.insert(tags.begin(), tags.end());
	const auto key = signage::furniture_pictogram(signage_tags);
	if (!key || !editor.decal_registry || !editor.decal_registry->contains(*key))
		return false;
	bool placed = false;
	for (std::int8_t facing : {2, 3, 4, 5})
		placed |= editor.place_decal_panel(x, y, z, facing, *key, false, false);
	return placed;
}

static std::optional<std::pair<int, int>> get_nearest_road_block(
		int x, int z, int max_radius, const RoadMaskBitmap &road_mask)
{
	for (int dist = 0; dist <= max_radius; ++dist) {
		for (int dx = -dist; dx <= dist; ++dx) {
			for (int dz = -dist; dz <= dist; ++dz) {
				if (std::max(std::abs(dx), std::abs(dz)) != dist)
					continue;
				int cx = x + dx;
				int cz = z + dz;
				if (road_mask.contains(cx, cz))
					return std::make_pair(cx, cz);
			}
		}
	}
	return std::nullopt;
}

void generate_amenities(crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedElement &element, const crate::args::Args &args,
		const FloodFillCache &flood_fill_cache, const RoadMaskBitmap &road_mask)
{
	// Skip if 'layer' or 'level' is negative in the tags
	{
		const std::unordered_map<std::string, std::string> &t = element.tags();
		auto it_layer = t.find("layer");
		if (it_layer != t.end()) {
			try {
				int layer = std::stoi(it_layer->second);
				if (layer < 0)
					return;
			} catch (...) {
			}
		}
		auto it_level = t.find("level");
		if (it_level != t.end()) {
			try {
				int level = std::stoi(it_level->second);
				if (level < 0)
					return;
			} catch (...) {
			}
		}
	}

	const std::unordered_map<std::string, std::string> &tags = element.tags();
	auto it_amenity = tags.find("amenity");
	if (it_amenity == tags.end())
		return;
	const std::string &amenity_type = it_amenity->second;

	std::optional<crate::coordinate_system::cartesian::XZPoint> first_node = std::nullopt;
	if (const auto node = element.first_node(); node.has_value())
		first_node.emplace(
				crate::coordinate_system::cartesian::XZPoint(node->x, node->z));

	// Handle recycling containers
	if (amenity_type == "recycling") {
		// Check if it's a container type
		auto it_recycling_type = tags.find("recycling_type");
		bool is_container = (it_recycling_type != tags.end() &&
							 it_recycling_type->second == "container");

		if (is_container && first_node.has_value()) {
			// Rust uses a barrel block entity for recycling containers.  The
			// host can attach loot through the editor's block-entity sink.
			const auto items = recycling_items(tags, first_node->x, first_node->z);
			editor.set_barrel_with_items_absolute(first_node->x,
					editor.get_ground_level(first_node->x, first_node->z) + 1,
					first_node->z, items);
			const bool decals_placed = place_furniture_decals(editor, tags, first_node->x,
					editor.get_ground_level(first_node->x, first_node->z) + 3,
					first_node->z);
			if (!decals_placed && !items.empty() && editor.item_frame_sink)
				editor.item_frame_sink(first_node->x + 1,
						editor.get_ground_level(first_node->x + 1, first_node->z) + 3,
						first_node->z, std::get<0>(items.front()));
		}
		return;
	}

	if (amenity_type == "waste_disposal" || amenity_type == "waste_basket") {
		if (first_node.has_value()) {
			editor.set_block(crate::block_definitions::CAULDRON, first_node->x, 1,
					first_node->z, std::nullopt, std::nullopt);
			place_furniture_decals(editor, tags, first_node->x,
					editor.get_ground_level(first_node->x, first_node->z) + 3,
					first_node->z);
		}
		return;
	}

	if (amenity_type == "vending_machine" || amenity_type == "atm") {
		if (first_node.has_value()) {
			editor.set_block(crate::block_definitions::IRON_BLOCK, first_node->x, 1,
					first_node->z, std::nullopt, std::nullopt);
			editor.set_block(crate::block_definitions::IRON_BLOCK, first_node->x, 2,
					first_node->z, std::nullopt, std::nullopt);
			place_furniture_decals(editor, tags, first_node->x,
					editor.get_ground_level(first_node->x, first_node->z) + 4,
					first_node->z);
		}
		return;
	}

	if (amenity_type == "bicycle_parking") {
		Block ground_block = OAK_PLANKS;
		if (auto it = element.tags().find("surface"); it != element.tags().end())
			if (const auto *blocks = surfaces::get_blocks_for_surface(it->second);
					blocks && !blocks->empty())
				ground_block = blocks->front();
		const crate::block_definitions::Block roof_block =
				crate::block_definitions::STONE_BLOCK_SLAB;

		std::vector<std::pair<int, int>> polygon_coords;
		for (const crate::osm_parser::ProcessedNode &n : element.nodes())
			polygon_coords.emplace_back(n.x, n.z);
		if (polygon_coords.empty())
			return;

		std::vector<std::pair<int, int>> floor_area =
				*flood_fill_cache.get_or_compute_element(element, args.timeout);

		for (const auto &p : floor_area) {
			editor.set_block(
					ground_block, p.first, 0, p.second, std::nullopt, std::nullopt);
		}

		for (const crate::osm_parser::ProcessedNode &node : element.nodes()) {
			int x = node.x;
			int z = node.z;
			editor.set_block(ground_block, x, 0, z, std::nullopt, std::nullopt);
			for (int y = 1; y <= 4; ++y)
				editor.set_block(crate::block_definitions::OAK_FENCE, x, y, z,
						std::nullopt, std::nullopt);
			editor.set_block(roof_block, x, 5, z, std::nullopt, std::nullopt);
		}

		for (const auto &p : floor_area) {
			editor.set_block(
					roof_block, p.first, 5, p.second, std::nullopt, std::nullopt);
		}
		return;
	}

	if (amenity_type == "bench") {
		if (first_node.has_value()) {
			auto road_pos =
					get_nearest_road_block(first_node->x, first_node->z, 4, road_mask);
			bool use_east_west = false;
			if (road_pos.has_value()) {
				int dx = std::abs(road_pos->first - first_node->x);
				int dz = std::abs(road_pos->second - first_node->z);
				use_east_west = dz >= dx;
			} else {
				use_east_west = (static_cast<unsigned int>(element.id()) & 1) != 0;
			}

			Block bench = crate::block_definitions::EARTH_BENCH;
			bench.setParam2(use_east_west ? 1 : 0);
			editor.set_block(
					bench, first_node->x, 1, first_node->z, std::nullopt, std::nullopt);
		}
		return;
	}

	if (amenity_type == "bbq") {
		if (first_node)
			editor.set_block(crate::block_definitions::EARTH_BARBECUE, first_node->x, 1,
					first_node->z, std::nullopt, std::nullopt);
		return;
	}

	if (amenity_type == "shelter") {
		const crate::block_definitions::Block roof_block =
				crate::block_definitions::STONE_BRICK_SLAB;
		std::vector<std::pair<int, int>> polygon_coords;
		for (const crate::osm_parser::ProcessedNode &n : element.nodes())
			polygon_coords.emplace_back(n.x, n.z);
		std::vector<std::pair<int, int>> roof_area =
				*flood_fill_cache.get_or_compute_element(element, args.timeout);

		for (const crate::osm_parser::ProcessedNode &node : element.nodes()) {
			int x = node.x;
			int z = node.z;
			for (int fence_height = 1; fence_height <= 4; ++fence_height)
				editor.set_block(crate::block_definitions::OAK_FENCE, x, fence_height, z,
						std::nullopt, std::nullopt);
			editor.set_block(roof_block, x, 5, z, std::nullopt, std::nullopt);
		}

		for (const auto &p : roof_area) {
			editor.set_block(
					roof_block, p.first, 5, p.second, std::nullopt, std::nullopt);
		}
		return;
	}

	if (amenity_type == "drinking_water") {
		if (first_node.has_value()) {
			int x = first_node->x;
			int z = first_node->z;
			editor.set_block(COBBLESTONE_WALL, x, 1, z, std::nullopt, std::nullopt);
			int abs_y = editor.get_absolute_y(x, 1, z);
			editor.set_block_absolute(
					LEVER, x - 1, abs_y + 1, z, std::nullopt, std::nullopt);
			editor.set_block_absolute(
					COBBLESTONE_WALL, x, abs_y + 1, z, std::nullopt, std::nullopt);
			editor.set_block_absolute(
					WATER_CAULDRON, x - 1, abs_y, z, std::nullopt, std::nullopt);
		}
		return;
	}

	if (amenity_type == "fountain") {
		std::vector<std::pair<int, int>> flood_area =
				*flood_fill_cache.get_or_compute_element(element, args.timeout);
		if (flood_area.empty()) {
			if (first_node.has_value())
				structures::fountain::place(editor, first_node->x, first_node->z, 0);
		} else {
			long long sx = 0;
			long long sz = 0;
			for (const auto &p : flood_area) {
				sx += p.first;
				sz += p.second;
			}
			const int cx0 =
					static_cast<int>(sx / static_cast<long long>(flood_area.size()));
			const int cz0 =
					static_cast<int>(sz / static_cast<long long>(flood_area.size()));
			const auto best = std::min_element(flood_area.begin(), flood_area.end(),
					[cx0, cz0](const auto &a, const auto &b) {
						const long long adx = static_cast<long long>(a.first - cx0);
						const long long adz = static_cast<long long>(a.second - cz0);
						const long long bdx = static_cast<long long>(b.first - cx0);
						const long long bdz = static_cast<long long>(b.second - cz0);
						return adx * adx + adz * adz < bdx * bdx + bdz * bdz;
					});
			structures::fountain::place(
					editor, best->first, best->second, flood_area.size());
		}
		return;
	}

	if (amenity_type == "parking") {
		using B = crate::block_definitions::Block;
		std::vector<B> block_types{crate::block_definitions::GRAY_CONCRETE_POWDER,
				crate::block_definitions::CYAN_TERRACOTTA};
		if (const auto surface = tags.find("surface"); surface != tags.end())
			if (const auto *mapped = surfaces::get_blocks_for_surface(surface->second);
					mapped && !mapped->empty())
				block_types = *mapped;
		const std::optional<std::vector<B>> outline_replacements =
				std::vector<B>{crate::block_definitions::BLACK_CONCRETE,
						crate::block_definitions::GRAY_CONCRETE_POWDER,
						crate::block_definitions::CYAN_TERRACOTTA};
		const std::optional<std::vector<B>> fill_replacements =
				std::vector<B>{crate::block_definitions::BLACK_CONCRETE,
						crate::block_definitions::GRAY_CONCRETE_POWDER,
						crate::block_definitions::CYAN_TERRACOTTA,
						crate::block_definitions::GRAY_CONCRETE};
		constexpr int space_width = 4;
		constexpr int space_length = 8;
		constexpr int lane_width = 5;
		std::optional<crate::coordinate_system::cartesian::XZPoint> previous_node;

		for (const crate::osm_parser::ProcessedNode &node : element.nodes()) {
			crate::coordinate_system::cartesian::XZPoint pt = node.xz();
			if (previous_node)
				for (const auto &[bx, by, bz] : crate::bresenham::bresenham_line(
							 previous_node->x, 0, previous_node->z, pt.x, 0, pt.z)) {
					(void)by;
					editor.set_block(surfaces::semirandom_surface(bx, bz, block_types),
							bx, 0, bz, outline_replacements, std::nullopt);
				}
			previous_node.emplace(pt);
		}

		const auto flood_area =
				flood_fill_cache.get_or_compute_element(element, args.timeout);
		for (const auto &[x, z] : *flood_area) {
			if (editor.nested_area_owns(x, z))
				continue;
			editor.set_block(surfaces::semirandom_surface(x, z, block_types), x, 0, z,
					fill_replacements, std::nullopt);
			const int zone_x = x / space_width;
			const int zone_z = z / (space_length + lane_width);
			const int local_x = x % space_width;
			const int local_z = z % (space_length + lane_width);
			if ((local_z < space_length && (local_x == 0 || local_z == 0)) ||
					local_z == space_length)
				editor.set_block(crate::block_definitions::WHITE_CONCRETE, x, 0, z,
						fill_replacements, std::nullopt);

			if (local_x == 0 && local_z == 0 && zone_x % 3 == 0 && zone_z % 2 == 0) {
				editor.set_block(crate::block_definitions::SMOOTH_STONE, x, 1, z,
						std::nullopt, std::nullopt);
				editor.set_block(crate::block_definitions::ANDESITE_WALL, x, 2, z,
						std::nullopt, std::nullopt);
				for (int dy = 3; dy <= 5; ++dy)
					editor.set_block(crate::block_definitions::IRON_BARS, x, dy, z,
							std::nullopt, std::nullopt);
				editor.set_block(crate::block_definitions::SEA_LANTERN, x, 6, z,
						std::nullopt, std::nullopt);
				editor.set_block(crate::block_definitions::SMOOTH_STONE_SLAB, x, 7, z,
						std::nullopt, std::nullopt);
			}
		}

		// Cars are only placed when the entire bay footprint is within the lot.
		std::vector<std::pair<int, int>> lot;
		lot.reserve(flood_area->size());
		for (const auto &[x, z] : *flood_area)
			if (!editor.nested_area_owns(x, z))
				lot.emplace_back(x, z);
		std::sort(lot.begin(), lot.end());
		if (!flood_area->empty()) {
			int min_x = flood_area->front().first, max_x = flood_area->front().first;
			int min_z = flood_area->front().second, max_z = flood_area->front().second;
			for (const auto &[x, z] : *flood_area) {
				min_x = std::min(min_x, x);
				max_x = std::max(max_x, x);
				min_z = std::min(min_z, z);
				max_z = std::max(max_z, z);
			}
			constexpr int period_z = space_length + lane_width;
			for (int zx = min_x / space_width; zx <= max_x / space_width; ++zx) {
				for (int zz = min_z / period_z; zz <= max_z / period_z; ++zz) {
					const int x0 = zx * space_width, z0 = zz * period_z;
					bool inside = true;
					for (int dx = 0; inside && dx <= space_width; ++dx)
						for (int dz = 0; dz <= space_length; ++dz)
							if (!std::binary_search(lot.begin(), lot.end(),
										std::pair{x0 + dx, z0 + dz})) {
								inside = false;
								break;
							}
					if (inside)
						structures::car::maybe_place_car(editor, x0 + 2, z0 + 4, 0);
				}
			}
		}

		return;
	}
	return;
}

void generate_amenities(crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedElement &element, const crate::args::Args &args)
{
	FloodFillCache cache;
	RoadMaskBitmap road_mask;
	generate_amenities(editor, element, args, cache, road_mask);
}

} // namespace amenities
} // namespace arnis
