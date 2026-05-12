#include <vector>
#include <string>
#include <unordered_map>
#include <optional>
#include <tuple>
#include <cstdlib>

#include "../../../arnis_adapter.h"
#include "../floodfill.h"
#include "../floodfill_cache.h"

namespace arnis
{

namespace amenities
{

static std::optional<std::pair<int, int>> get_nearest_road_block(
        int x, int z, int max_radius, const RoadMaskBitmap& road_mask)
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

void generate_amenities(crate::world_editor::WorldEditor& editor,
        const crate::osm_parser::ProcessedElement& element,
        const crate::args::Args& args,
        const FloodFillCache& flood_fill_cache,
        const RoadMaskBitmap& road_mask) {
    // Skip if 'layer' or 'level' is negative in the tags
    {
        const std::unordered_map<std::string,std::string>& t = element.tags();
        auto it_layer = t.find("layer");
        if (it_layer != t.end()) {
            try {
                int layer = std::stoi(it_layer->second);
                if (layer < 0) return;
            } catch (...) {}
        }
        auto it_level = t.find("level");
        if (it_level != t.end()) {
            try {
                int level = std::stoi(it_level->second);
                if (level < 0) return;
            } catch (...) {}
        }
    }

    const std::unordered_map<std::string,std::string>& tags = element.tags();
    auto it_amenity = tags.find("amenity");
    if (it_amenity == tags.end()) return;
    const std::string& amenity_type = it_amenity->second;

    std::optional<crate::coordinate_system::cartesian::XZPoint> first_node = std::nullopt;
    {
        const std::vector<crate::osm_parser::ProcessedNode>& nodes = element.nodes();
        if (!nodes.empty()) first_node.emplace( crate::coordinate_system::cartesian::XZPoint(nodes.front().x, nodes.front().z));
    }

    // Handle recycling containers
    if (amenity_type == "recycling") {
        // Check if it's a container type
        auto it_recycling_type = tags.find("recycling_type");
        bool is_container = (it_recycling_type != tags.end() && it_recycling_type->second == "container");
        
        if (is_container && first_node.has_value()) {
            // For now, just place a cauldron for recycling containers
            editor.set_block(crate::block_definitions::CAULDRON, first_node->x, 1, first_node->z, std::nullopt, std::nullopt);
        }
        return;
    }

    if (amenity_type == "waste_disposal" || amenity_type == "waste_basket") {
        if (first_node.has_value()) {
            editor.set_block(crate::block_definitions::CAULDRON, first_node->x, 1, first_node->z, std::nullopt, std::nullopt);
        }
        return;
    }

    if (amenity_type == "vending_machine" || amenity_type == "atm") {
        if (first_node.has_value()) {
            editor.set_block(crate::block_definitions::IRON_BLOCK, first_node->x, 1, first_node->z, std::nullopt, std::nullopt);
            editor.set_block(crate::block_definitions::IRON_BLOCK, first_node->x, 2, first_node->z, std::nullopt, std::nullopt);
        }
        return;
    }

    if (amenity_type == "bicycle_parking") {
        const crate::block_definitions::Block ground_block = crate::block_definitions::OAK_PLANKS;
        const crate::block_definitions::Block roof_block = crate::block_definitions::STONE_BLOCK_SLAB;

        std::vector<std::pair<int,int>> polygon_coords;
        for (const crate::osm_parser::ProcessedNode& n : element.nodes()) polygon_coords.emplace_back(n.x, n.z);
        if (polygon_coords.empty()) return;

        std::vector<std::pair<int,int>> floor_area =
                flood_fill_cache.get_or_compute_element(element, args.timeout);

        for (const auto& p : floor_area) {
            editor.set_block(ground_block, p.first, 0, p.second, std::nullopt, std::nullopt);
        }

        for (const crate::osm_parser::ProcessedNode& node : element.nodes()) {
            int x = node.x; int z = node.z;
            editor.set_block(ground_block, x, 0, z, std::nullopt, std::nullopt);
            for (int y = 1; y <= 4; ++y) editor.set_block(crate::block_definitions::OAK_FENCE, x, y, z, std::nullopt, std::nullopt);
            editor.set_block(roof_block, x, 5, z, std::nullopt, std::nullopt);
        }

        for (const auto& p : floor_area) {
            editor.set_block(roof_block, p.first, 5, p.second, std::nullopt, std::nullopt);
        }
        return;
    }

    if (amenity_type == "bench") {
        if (first_node.has_value()) {
            auto road_pos = get_nearest_road_block(first_node->x, first_node->z, 4, road_mask);
            bool use_east_west = false;
            if (road_pos.has_value()) {
                int dx = std::abs(road_pos->first - first_node->x);
                int dz = std::abs(road_pos->second - first_node->z);
                use_east_west = dz >= dx;
            } else {
                use_east_west = (static_cast<unsigned int>(element.id()) & 1) != 0;
            }

            const int dx = use_east_west ? 1 : 0;
            const int dz = use_east_west ? 0 : 1;
            const auto facing_a = use_east_west ? StairFacing::West : StairFacing::North;
            const auto facing_b = use_east_west ? StairFacing::East : StairFacing::South;
            const int abs_y = editor.get_absolute_y(first_node->x, 1, first_node->z);

            editor.set_block_with_properties_absolute(
                    top_stair(create_stair_with_properties(OAK_STAIRS, facing_a, StairShape::Straight)),
                    first_node->x - dx, abs_y, first_node->z - dz, nullptr, nullptr);
            editor.set_block(OAK_SLAB_TOP, first_node->x, 1, first_node->z, std::nullopt, std::nullopt);
            editor.set_block_with_properties_absolute(
                    top_stair(create_stair_with_properties(OAK_STAIRS, facing_b, StairShape::Straight)),
                    first_node->x + dx, abs_y, first_node->z + dz, nullptr, nullptr);
        }
        return;
    }

    if (amenity_type == "shelter") {
        const crate::block_definitions::Block roof_block = crate::block_definitions::STONE_BRICK_SLAB;
        std::vector<std::pair<int,int>> polygon_coords;
        for (const crate::osm_parser::ProcessedNode& n : element.nodes()) polygon_coords.emplace_back(n.x, n.z);
        std::vector<std::pair<int,int>> roof_area =
                flood_fill_cache.get_or_compute_element(element, args.timeout);

        for (const crate::osm_parser::ProcessedNode& node : element.nodes()) {
            int x = node.x; int z = node.z;
            for (int fence_height = 1; fence_height <= 4; ++fence_height) editor.set_block(crate::block_definitions::OAK_FENCE, x, fence_height, z, std::nullopt, std::nullopt);
            editor.set_block(roof_block, x, 5, z, std::nullopt, std::nullopt);
        }

        for (const auto& p : roof_area) {
            editor.set_block(roof_block, p.first, 5, p.second, std::nullopt, std::nullopt);
        }
        return;
    }

    if (amenity_type == "drinking_water") {
        if (first_node.has_value()) {
            int x = first_node->x;
            int z = first_node->z;
            editor.set_block(COBBLESTONE_WALL, x, 1, z, std::nullopt, std::nullopt);
            int abs_y = editor.get_absolute_y(x, 1, z);
            editor.set_block_absolute(LEVER, x - 1, abs_y + 1, z, std::nullopt, std::nullopt);
            editor.set_block_absolute(COBBLESTONE_WALL, x, abs_y + 1, z, std::nullopt, std::nullopt);
            editor.set_block_absolute(WATER_CAULDRON, x - 1, abs_y, z, std::nullopt, std::nullopt);
        }
        return;
    }

    if (amenity_type == "fountain") {
        std::vector<std::pair<int,int>> flood_area =
                flood_fill_cache.get_or_compute_element(element, args.timeout);
        for (const auto& p : flood_area) {
            editor.set_block(WATER, p.first, 0, p.second, std::nullopt, std::nullopt);
        }
        for (const crate::osm_parser::ProcessedNode& node : element.nodes()) {
            editor.set_block(LIGHT_GRAY_CONCRETE, node.x, 0, node.z, std::nullopt, std::nullopt);
        }
        return;
    }

    if (amenity_type == "parking") {
        std::optional<crate::coordinate_system::cartesian::XZPoint> previous_node = std::nullopt;
        std::tuple<int,int,int> corner_addup = std::make_tuple(0,0,0);
        std::vector<std::pair<int,int>> current_amenity;

        const crate::block_definitions::Block block_type = crate::block_definitions::GRAY_CONCRETE;

        for (const crate::osm_parser::ProcessedNode& node : element.nodes()) {
            crate::coordinate_system::cartesian::XZPoint pt = node.xz();
            if (previous_node.has_value()) {
                std::vector<std::tuple<int,int,int>> bresenham_points = crate::bresenham::bresenham_line(previous_node->x, 0, previous_node->z, pt.x, 0, pt.z);
                for (const auto& t : bresenham_points) {
                    int bx = std::get<0>(t);
                    int bz = std::get<2>(t);
                    // Use replacement whitelist for better block placement
                    editor.set_block(block_type, bx, 0, bz, std::optional<std::vector<const crate::block_definitions::Block*>>(std::vector<const crate::block_definitions::Block*>{ &crate::block_definitions::BLACK_CONCRETE }), std::nullopt);
                    
                    current_amenity.emplace_back(node.x, node.z);
                    std::get<0>(corner_addup) += node.x;
                    std::get<1>(corner_addup) += node.z;
                    std::get<2>(corner_addup) += 1;
                }
            }
            previous_node.emplace(pt);
        }

        if (std::get<2>(corner_addup) > 0) {
            std::vector<std::pair<int,int>> flood_area =
                    flood_fill_cache.get_or_compute_element(element, args.timeout);

            for (const auto& p : flood_area) {
                int x = p.first; int z = p.second;
                editor.set_block(block_type, x, 0, z, std::optional<std::vector<const crate::block_definitions::Block*>>(std::vector<const crate::block_definitions::Block*>{ &crate::block_definitions::BLACK_CONCRETE, &crate::block_definitions::GRAY_CONCRETE }), std::nullopt);

                if (amenity_type == "parking") {
                    // Create defined parking spaces with realistic layout
                    int space_width = 4; // Width of each parking space
                    int space_length = 6; // Length of each parking space
                    int lane_width = 5; // Width of driving lanes

                    // Calculate which "zone" this coordinate falls into
                    int zone_x = x / space_width;
                    int zone_z = z / (space_length + lane_width);
                    int local_x = x % space_width;
                    if (local_x < 0) local_x += space_width;
                    int local_z = z % (space_length + lane_width);
                    if (local_z < 0) local_z += (space_length + lane_width);

                    // Create parking space boundaries (only within parking areas, not in driving lanes)
                    if (local_z < space_length) {
                        // We're in a parking space area, not in the driving lane
                        if (local_x == 0) {
                            // Vertical parking space lines (only on the left edge)
                            editor.set_block(crate::block_definitions::LIGHT_GRAY_CONCRETE, x, 0, z, std::optional<std::vector<const crate::block_definitions::Block*>>(std::vector<const crate::block_definitions::Block*>{ &crate::block_definitions::BLACK_CONCRETE, &crate::block_definitions::GRAY_CONCRETE }), std::nullopt);
                        } else if (local_z == 0) {
                            // Horizontal parking space lines (only on the top edge)
                            editor.set_block(crate::block_definitions::LIGHT_GRAY_CONCRETE, x, 0, z, std::optional<std::vector<const crate::block_definitions::Block*>>(std::vector<const crate::block_definitions::Block*>{ &crate::block_definitions::BLACK_CONCRETE, &crate::block_definitions::GRAY_CONCRETE }), std::nullopt);
                        }
                    } else if (local_z == space_length) {
                        // Bottom edge of parking spaces (border with driving lane)
                        editor.set_block(crate::block_definitions::LIGHT_GRAY_CONCRETE, x, 0, z, std::optional<std::vector<const crate::block_definitions::Block*>>(std::vector<const crate::block_definitions::Block*>{ &crate::block_definitions::BLACK_CONCRETE, &crate::block_definitions::GRAY_CONCRETE }), std::nullopt);
                    } else if (local_z > space_length && local_z < space_length + lane_width) {
                        // Driving lane - use darker concrete
                        editor.set_block(crate::block_definitions::BLACK_CONCRETE, x, 0, z, std::optional<std::vector<const crate::block_definitions::Block*>>(std::vector<const crate::block_definitions::Block*>{ &crate::block_definitions::GRAY_CONCRETE }), std::nullopt);
                    }

                    // Add light posts at parking space outline corners
                    if (local_x == 0 && local_z == 0 && zone_x % 3 == 0 && zone_z % 2 == 0) {
                        // Light posts at regular intervals on parking space corners
                        editor.set_block(crate::block_definitions::COBBLESTONE_WALL, x, 1, z, std::nullopt, std::nullopt);
                        for (int dy = 2; dy <= 4; ++dy) editor.set_block(crate::block_definitions::OAK_FENCE, x, dy, z, std::nullopt, std::nullopt);
                        editor.set_block(crate::block_definitions::GLOWSTONE, x, 5, z, std::nullopt, std::nullopt);
                    }
                }
            }
        }
        return;
    }

    return;
}

void generate_amenities(crate::world_editor::WorldEditor& editor,
        const crate::osm_parser::ProcessedElement& element,
        const crate::args::Args& args) {
    FloodFillCache cache;
    RoadMaskBitmap road_mask;
    generate_amenities(editor, element, args, cache, road_mask);
}

} // namespace amenities
} // namespace arnis
