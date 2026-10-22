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
#include "road_mask_search.h"
#include "street_furniture.h"
#include "../strict_parse.h"

namespace arnis
{

namespace amenities
{

enum class RecyclingLootKind
{
	GlassBottle,
	Paper,
	GlassBlock,
	GlassPane,
	LeatherArmor,
	EmptyBucket,
	LeatherBoots,
	ScrapMetal,
	GreenWaste
};

enum class LootCategory
{
	GlassBottle,
	Paper,
	Glass,
	Leather,
	EmptyBucket,
	ScrapMetal,
	GreenWaste
};

bool tag_enabled(
		const std::unordered_map<std::string, std::string> &tags, const char *key)
{
	const auto it = tags.find(key);
	return it != tags.end() && it->second == "yes";
}

int random_inclusive(ChaCha8Rng &rng, int low, int high)
{
	return low +
		   static_cast<int>(rng.uniform(static_cast<std::uint32_t>(high - low + 1)));
}

nlohmann::json basic_recycling_item(const std::string &id, int slot, int count)
{
	return {{"id", id}, {"Slot", slot}, {"Count", count}};
}

int biased_damage(int maximum, ChaCha8Rng &rng)
{
	const int upper = std::max(0, maximum - 1);
	const int lower = std::min(maximum / 2, upper);
	return std::max(random_inclusive(rng, lower, upper), random_inclusive(rng, 0, upper));
}

nlohmann::json leather_recycling_item(
		const std::string &id, int max_damage, int slot, ChaCha8Rng &rng)
{
	auto item = basic_recycling_item(id, slot, 1);
	const int damage = biased_damage(max_damage, rng);
	nlohmann::json tag = {{"Damage", damage}};
	if (rng.random_bool(.3))
		tag["display"] = {{"color", random_inclusive(rng, 0, 0x00ffffff)}};
	item["tag"] = std::move(tag);
	item["components"] = {{"minecraft:damage", damage}};
	return item;
}

const std::vector<RecyclingLootKind> recycling_loot_pool(
		const std::unordered_map<std::string, std::string> &tags)
{
	std::vector<RecyclingLootKind> pool;
	if (tag_enabled(tags, "recycling:glass_bottles"))
		pool.push_back(RecyclingLootKind::GlassBottle);
	if (tag_enabled(tags, "recycling:paper"))
		pool.push_back(RecyclingLootKind::Paper);
	if (tag_enabled(tags, "recycling:glass")) {
		pool.push_back(RecyclingLootKind::GlassBlock);
		pool.push_back(RecyclingLootKind::GlassPane);
	}
	if (tag_enabled(tags, "recycling:clothes"))
		pool.push_back(RecyclingLootKind::LeatherArmor);
	if (tag_enabled(tags, "recycling:cans"))
		pool.push_back(RecyclingLootKind::EmptyBucket);
	if (tag_enabled(tags, "recycling:shoes"))
		pool.push_back(RecyclingLootKind::LeatherBoots);
	if (tag_enabled(tags, "recycling:scrap_metal"))
		pool.push_back(RecyclingLootKind::ScrapMetal);
	if (tag_enabled(tags, "recycling:green_waste"))
		pool.push_back(RecyclingLootKind::GreenWaste);
	return pool;
}

nlohmann::json build_recycling_item(RecyclingLootKind kind, int slot, ChaCha8Rng &rng)
{
	switch (kind) {
	case RecyclingLootKind::GlassBottle:
		return basic_recycling_item(
				"minecraft:glass_bottle", slot, random_inclusive(rng, 1, 4));
	case RecyclingLootKind::Paper:
		return basic_recycling_item(
				"minecraft:paper", slot, random_inclusive(rng, 1, 10));
	case RecyclingLootKind::GlassBlock:
	case RecyclingLootKind::GlassPane: {
		static constexpr std::array<const char *, 16> colors{{"white", "orange",
				"magenta", "light_blue", "yellow", "lime", "pink", "gray", "light_gray",
				"cyan", "purple", "blue", "brown", "green", "red", "black"}};
		const bool pane = kind == RecyclingLootKind::GlassPane;
		std::string id = pane ? "minecraft:glass_pane" : "minecraft:glass";
		if (!rng.random_bool(.7)) {
			const std::string color =
					colors[rng.uniform(static_cast<std::uint32_t>(colors.size()))];
			id = "minecraft:" + color + (pane ? "_stained_glass_pane" : "_stained_glass");
		}
		return basic_recycling_item(id, slot,
				pane ? random_inclusive(rng, 4, 16) : random_inclusive(rng, 1, 6));
	}
	case RecyclingLootKind::LeatherArmor: {
		static constexpr std::array<std::pair<const char *, int>, 4> armor{
				{{"minecraft:leather_helmet", 55}, {"minecraft:leather_chestplate", 80},
						{"minecraft:leather_leggings", 75},
						{"minecraft:leather_boots", 65}}};
		const auto [id, max_damage] =
				armor[rng.uniform(static_cast<std::uint32_t>(armor.size()))];
		return leather_recycling_item(id, max_damage, slot, rng);
	}
	case RecyclingLootKind::EmptyBucket:
		return basic_recycling_item("minecraft:bucket", slot, 1);
	case RecyclingLootKind::LeatherBoots:
		return leather_recycling_item("minecraft:leather_boots", 65, slot, rng);
	case RecyclingLootKind::ScrapMetal: {
		static constexpr std::array<const char *, 3> metals{
				{"copper_ingot", "iron_ingot", "gold_ingot"}};
		return basic_recycling_item("minecraft:" + std::string(metals[rng.uniform(3)]),
				slot, random_inclusive(rng, 1, 3));
	}
	case RecyclingLootKind::GreenWaste: {
		static constexpr std::array<const char *, 8> options{
				{"minecraft:tall_grass", "minecraft:sweet_berries",
						"minecraft:oak_sapling", "minecraft:birch_sapling",
						"minecraft:spruce_sapling", "minecraft:jungle_sapling",
						"minecraft:acacia_sapling", "minecraft:dark_oak_sapling"}};
		const unsigned choice = rng.uniform(8);
		int count = choice == 0	  ? random_inclusive(rng, 1, 4)
					: choice == 1 ? random_inclusive(rng, 2, 6)
								  : random_inclusive(rng, 1, 2);
		const char *id = options[choice];
		if (rng.random_bool(.25)) {
			static constexpr std::array<const char *, 4> seeds{
					{"minecraft:wheat_seeds", "minecraft:pumpkin_seeds",
							"minecraft:melon_seeds", "minecraft:beetroot_seeds"}};
			id = seeds[rng.uniform(4)];
		}
		return basic_recycling_item(id, slot, count);
	}
	}
	return {};
}

std::vector<nlohmann::json> build_recycling_items(
		const std::vector<RecyclingLootKind> &pool, ChaCha8Rng &rng)
{
	std::vector<nlohmann::json> items;
	if (pool.empty())
		return items;
	for (int slot = 0; slot < 27; ++slot)
		if (rng.random_bool(.2))
			items.push_back(build_recycling_item(
					pool[rng.uniform(static_cast<std::uint32_t>(pool.size()))], slot,
					rng));
	return items;
}

std::optional<LootCategory> single_loot_category(
		const std::vector<RecyclingLootKind> &pool)
{
	std::optional<LootCategory> category;
	for (const auto kind : pool) {
		LootCategory current;
		switch (kind) {
		case RecyclingLootKind::GlassBottle:
			current = LootCategory::GlassBottle;
			break;
		case RecyclingLootKind::Paper:
			current = LootCategory::Paper;
			break;
		case RecyclingLootKind::GlassBlock:
		case RecyclingLootKind::GlassPane:
			current = LootCategory::Glass;
			break;
		case RecyclingLootKind::LeatherArmor:
		case RecyclingLootKind::LeatherBoots:
			current = LootCategory::Leather;
			break;
		case RecyclingLootKind::EmptyBucket:
			current = LootCategory::EmptyBucket;
			break;
		case RecyclingLootKind::ScrapMetal:
			current = LootCategory::ScrapMetal;
			break;
		case RecyclingLootKind::GreenWaste:
			current = LootCategory::GreenWaste;
			break;
		}
		if (category && *category != current)
			return std::nullopt;
		category = current;
	}
	return category;
}

std::optional<nlohmann::json> build_display_item_for_category(
		LootCategory category, ChaCha8Rng &rng)
{
	switch (category) {
	case LootCategory::GlassBottle:
		return nlohmann::json{{"id", "minecraft:glass_bottle"}, {"Count", 1}};
	case LootCategory::Paper:
		return nlohmann::json{
				{"id", "minecraft:paper"}, {"Count", random_inclusive(rng, 1, 4)}};
	case LootCategory::Glass:
		return nlohmann::json{{"id", "minecraft:glass"}, {"Count", 1}};
	case LootCategory::Leather: {
		auto item = leather_recycling_item("minecraft:leather_chestplate", 80, 0, rng);
		item.erase("Slot");
		return item;
	}
	case LootCategory::EmptyBucket:
		return nlohmann::json{{"id", "minecraft:bucket"}, {"Count", 1}};
	case LootCategory::ScrapMetal: {
		static constexpr std::array<const char *, 3> metals{{"minecraft:copper_ingot",
				"minecraft:iron_ingot", "minecraft:gold_ingot"}};
		return nlohmann::json{
				{"id", metals[rng.uniform(3)]}, {"Count", random_inclusive(rng, 1, 2)}};
	}
	case LootCategory::GreenWaste: {
		static constexpr std::array<const char *, 5> options{{"minecraft:oak_sapling",
				"minecraft:birch_sapling", "minecraft:tall_grass",
				"minecraft:sweet_berries", "minecraft:wheat_seeds"}};
		return nlohmann::json{
				{"id", options[rng.uniform(5)]}, {"Count", random_inclusive(rng, 1, 3)}};
	}
	}
	return std::nullopt;
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
	(void)placed;
	// Rust reports that the registered decal was handled even if a particular
	// face could not accept a panel.
	return true;
}

static std::optional<std::pair<int, int>> get_nearest_road_block(
		int x, int z, int max_radius, const RoadMaskBitmap &road_mask)
{
	return element_processing::get_nearest_road_block(x, z, max_radius, road_mask);
}

void generate_amenities(crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedElement &element, const crate::args::Args &args,
		const FloodFillCache &flood_fill_cache, const RoadMaskBitmap &road_mask)
{
	// Match Rust parse::<i32>().unwrap_or(0): malformed tags default to zero.
	const auto &t = element.tags();
	for (const char *key : {"layer", "level"}) {
		if (const auto it = t.find(key);
				it != t.end() && strict_parse::i32_or_zero(it->second) < 0)
			return;
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
			const auto pool = recycling_loot_pool(tags);
			auto loot_rng =
					element_rng_salted(static_cast<std::uint64_t>(element.id()), 0x5EC1);
			const auto items = build_recycling_items(pool, loot_rng);
			const int barrel_y = editor.get_absolute_y(first_node->x, 1, first_node->z);
			editor.set_barrel_with_nbt_items_absolute(
					first_node->x, barrel_y, first_node->z, items);
			const bool decals_placed = place_furniture_decals(
					editor, tags, first_node->x, barrel_y, first_node->z);
			if (!decals_placed) {
				const auto category = single_loot_category(pool);
				const auto display_item =
						category ? build_display_item_for_category(*category, loot_rng)
								 : std::nullopt;
				if (!display_item)
					return;
				using Direction = std::tuple<int, int, std::int8_t>;
				std::array<Direction, 4> directions{
						{{0, -1, 2}, {0, 1, 3}, {-1, 0, 4}, {1, 0, 5}}};
				auto rng = coord_rng(first_node->x, first_node->z, 0xF7A3E);
				for (std::size_t i = directions.size() - 1; i > 0; --i)
					std::swap(directions[i],
							directions[rng.uniform(static_cast<std::uint32_t>(i + 1))]);
				const auto [min_x, min_z] = editor.get_min_coords();
				const auto [max_x, max_z] = editor.get_max_coords();
				Direction selected{0, 1, 3}; // Rust's south-facing fallback.
				for (const auto &direction : directions) {
					const auto &[dx, dz, facing] = direction;
					const int target_x = first_node->x + dx;
					const int target_z = first_node->z + dz;
					if (target_x >= min_x && target_x <= max_x && target_z >= min_z &&
							target_z <= max_z) {
						selected = direction;
						break;
					}
				}
				const auto &[dx, dz, facing] = selected;
				const int target_x = first_node->x + dx;
				const int target_z = first_node->z + dz;
				const int target_y = barrel_y;
				const int relative_y =
						target_y - editor.get_absolute_y(target_x, 0, target_z);
				const std::string item_id = display_item->at("id").get<std::string>();
				nlohmann::json extra = {{"Facing", facing}, {"ItemRotation", 0},
						{"Item", *display_item}, {"ItemDropChance", 1.0f},
						{"block_pos", {target_x, target_y, target_z}},
						{"TileX", target_x}, {"TileY", target_y}, {"TileZ", target_z},
						{"Fixed", 1}};
				if (!editor.add_entity("minecraft:item_frame", target_x, relative_y,
							target_z, extra) &&
						editor.item_frame_sink)
					editor.item_frame_sink(target_x, target_y, target_z, item_id);
			}
		}
		return;
	}

	if (amenity_type == "waste_disposal" || amenity_type == "waste_basket") {
		if (first_node.has_value()) {
			editor.set_block(crate::block_definitions::CAULDRON, first_node->x, 1,
					first_node->z, std::nullopt, std::nullopt);
			place_furniture_decals(editor, tags, first_node->x,
					editor.get_absolute_y(first_node->x, 1, first_node->z),
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
					editor.get_absolute_y(first_node->x, 2, first_node->z),
					first_node->z);
		}
		return;
	}

	if (amenity_type == "bicycle_parking") {
		if (element.is_node())
			return;
		const auto floor_area =
				flood_fill_cache.get_or_compute_element(element, args.timeout);
		if (!floor_area->empty())
			street_furniture::generate_bicycle_parking_area(editor, element, *floor_area);
		return;
	}

	if (amenity_type == "bench") {
		if (first_node.has_value()) {
			auto rng = element_rng(static_cast<std::uint64_t>(element.id()));
			auto road_pos =
					get_nearest_road_block(first_node->x, first_node->z, 4, road_mask);
			bool use_east_west;
			if (road_pos.has_value()) {
				int dx = std::abs(road_pos->first - first_node->x);
				int dz = std::abs(road_pos->second - first_node->z);
				use_east_west = dz >= dx;
			} else {
				use_east_west = rng.random_bool(0.5);
			}

			const StairFacing facing_a =
					use_east_west ? StairFacing::West : StairFacing::North;
			const StairFacing facing_b =
					use_east_west ? StairFacing::East : StairFacing::South;
			const int dx = use_east_west ? 1 : 0;
			const int dz = use_east_west ? 0 : 1;
			const int abs_y = editor.get_absolute_y(first_node->x, 1, first_node->z);
			const std::vector<Block> blacklist{crate::block_definitions::OAK_LOG,
					crate::block_definitions::SPRUCE_LOG};
			const auto stair_a = top_stair(
					create_stair_with_properties(crate::block_definitions::OAK_STAIRS,
							facing_a, StairShape::Straight));
			editor.set_block_with_properties_absolute(stair_a, first_node->x - dx, abs_y,
					first_node->z - dz, std::nullopt, blacklist);
			editor.set_block(crate::block_definitions::OAK_SLAB_TOP, first_node->x, 1,
					first_node->z, std::nullopt, blacklist);
			const auto stair_b = top_stair(
					create_stair_with_properties(crate::block_definitions::OAK_STAIRS,
							facing_b, StairShape::Straight));
			editor.set_block_with_properties_absolute(stair_b, first_node->x + dx, abs_y,
					first_node->z + dz, std::nullopt, blacklist);
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
		if (element.is_node()) {
			street_furniture::generate_shelter_node(editor, element.as_node(), road_mask);
			return;
		}
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
			const BlockWithProperties lever{
					LEVER, {{"facing", "west"}, {"powered", "true"}}};
			editor.set_block_with_properties_absolute(
					lever, x - 1, abs_y + 1, z, std::nullopt, std::nullopt);
			const BlockWithProperties spout{COBBLESTONE_WALL, {{"west", "low"}}};
			editor.set_block_with_properties_absolute(
					spout, x, abs_y + 1, z, std::nullopt, std::nullopt);
			const BlockWithProperties cauldron{WATER_CAULDRON, {{"level", "3"}}};
			editor.set_block_with_properties_absolute(
					cauldron, x - 1, abs_y, z, std::nullopt, std::nullopt);
		}
		return;
	}

	if (amenity_type == "fountain") {
		if (element.nodes().size() < 3) {
			if (first_node.has_value())
				structures::fountain::place(editor, first_node->x, first_node->z, 0);
			return;
		}
		std::vector<std::pair<int, int>> flood_area =
				*flood_fill_cache.get_or_compute_element(element, args.timeout);
		if (flood_area.empty())
			return;
		long long sx = 0;
		long long sz = 0;
		for (const auto &p : flood_area) {
			sx += p.first;
			sz += p.second;
		}
		const int cx0 = static_cast<int>(sx / static_cast<long long>(flood_area.size()));
		const int cz0 = static_cast<int>(sz / static_cast<long long>(flood_area.size()));
		const auto best = std::min_element(flood_area.begin(), flood_area.end(),
				[cx0, cz0](const auto &a, const auto &b) {
					const long long adx = static_cast<long long>(a.first - cx0);
					const long long adz = static_cast<long long>(a.second - cz0);
					const long long bdx = static_cast<long long>(b.first - cx0);
					const long long bdz = static_cast<long long>(b.second - cz0);
					return adx * adx + adz * adz < bdx * bdx + bdz * bdz;
				});
		structures::fountain::place(editor, best->first, best->second, flood_area.size());
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
