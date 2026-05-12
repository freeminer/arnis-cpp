#include <vector>
#include <unordered_map>
#include <string>
#include <optional>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <cstddef>

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"
#include "bridges.h"
#include "surfaces.h"
#include "../floodfill.h"
namespace arnis
{

namespace highways
{



#if 0
namespace crate {
namespace args {
struct Args {
    double scale = 1.0;
    std::optional<int> timeout;
};
} // namespace args

namespace bresenham {
std::vector<std::tuple<int, int, int>> bresenham_line(int x1, int y1, int z1, int x2, int y2, int z2);
} // namespace bresenham

namespace floodfill {
std::vector<std::pair<int, int>> flood_fill_area(const std::vector<std::pair<int, int>>& polygon, const std::optional<int>& timeout);
} // namespace floodfill

namespace coordinate_system {
namespace cartesian {
struct XZPoint {
    int x;
    int z;
};
} // namespace cartesian
} // namespace coordinate_system

namespace osm_parser {
struct ProcessedNode {
    int x;
    int z;
    coordinate_system::cartesian::XZPoint xz() const { return {x, z}; }
};

struct ProcessedWay {
    std::vector<ProcessedNode> nodes;
    std::unordered_map<std::string, std::string> tags;
};

enum class ElementType { Node, Way };

struct ProcessedElement {
    ElementType type;
    std::optional<ProcessedNode> node;
    std::optional<ProcessedWay> way;
    std::unordered_map<std::string, std::string> tags;

    const std::unordered_map<std::string, std::string>& tags_map() const { return tags; }
};
} // namespace osm_parser

namespace block_definitions {
enum Block {
    STONE,
    STONE_BRICKS,
    STONE_BRICK_SLAB,
    BLACK_CONCRETE,
    WHITE_CONCRETE,
    GRAY_CONCRETE,
    LIGHT_GRAY_CONCRETE,
    BRICK,
    OAK_PLANKS,
    DIRT,
    GRAVEL,
    SAND,
    GRASS_BLOCK,
    DIRT_PATH,
    COBBLESTONE_WALL,
    OAK_FENCE,
    GLOWSTONE,
    GREEN_WOOL,
    YELLOW_WOOL,
    RED_WOOL,
    WHITE_WOOL,
    SAND_TRAP, // placeholder
    STONE_BRICK_SLABS // placeholder
};
} // namespace block_definitions

namespace world_editor {
using crate::block_definitions::Block;

struct WorldEditor {
    void set_block(Block block, int x, int y, int z, const std::optional<std::vector<Block>>& overlay, const std::optional<std::vector<Block>>& palette) {
        // Implementation provided by host environment
        (void)block; (void)x; (void)y; (void)z; (void)overlay; (void)palette;
    }

    bool check_for_block(int x, int y, int z, const std::optional<std::vector<Block>>& blocks) const {
        // Implementation provided by host environment
        (void)x; (void)y; (void)z; (void)blocks;
        return false;
    }
};
} // namespace world_editor

} // namespace crate

#endif

// Hash for pair<int,int> used in unordered_map
struct PairHash {
    std::size_t operator()(const std::pair<int,int>& p) const noexcept {
        return std::hash<long long>()( (static_cast<long long>(p.first) << 32) ^ static_cast<unsigned long long>(p.second) );
    }
};

// Minimum terrain dip (in blocks) below max endpoint elevation to classify a bridge as valley-spanning
const int VALLEY_BRIDGE_THRESHOLD = 7;

const std::vector<crate::block_definitions::Block> DEFAULT_ROAD_MIX = {
    crate::block_definitions::GRAY_CONCRETE_POWDER,
    crate::block_definitions::CYAN_TERRACOTTA,
};

const std::vector<crate::block_definitions::Block> ROAD_PROTECTED_SURFACES = {
    crate::block_definitions::BLACK_CONCRETE,
    crate::block_definitions::GRAY_CONCRETE_POWDER,
    crate::block_definitions::CYAN_TERRACOTTA,
    crate::block_definitions::WHITE_CONCRETE,
};

std::unordered_map<std::pair<int,int>, std::vector<int>, PairHash> build_highway_connectivity_map(const std::vector<crate::osm_parser::ProcessedElement>& elements) {
    std::unordered_map<std::pair<int,int>, std::vector<int>, PairHash> connectivity_map;
    for (const auto& element : elements) {
        if (element.type == crate::osm_parser::ElementType::Way /* && element.way.has_value()*/) {
            const crate::osm_parser::ProcessedWay& way = *element.way;
            auto it_highway = way.tags.find("highway");
            if (it_highway != way.tags.end()) {
                int layer_value = 0;
                auto it_layer = way.tags.find("layer");
                if (it_layer != way.tags.end()) {
                    try {
                        layer_value = std::stoi(it_layer->second);
                    } catch (...) {
                        layer_value = 0;
                    }
                }
                // Treat negative layers as ground level (0) for connectivity
                if (layer_value < 0) {
                    layer_value = 0;
                }
                if (!way.nodes.empty()) {
                    const crate::osm_parser::ProcessedNode& start_node = way.nodes.front();
                    const crate::osm_parser::ProcessedNode& end_node = way.nodes.back();
                    std::pair<int,int> start_coord = { start_node.x, start_node.z };
                    std::pair<int,int> end_coord = { end_node.x, end_node.z };
                    connectivity_map[start_coord].push_back(layer_value);
                    connectivity_map[end_coord].push_back(layer_value);
                }
            }
        }
    }
    return connectivity_map;
}

void add_highway_support_pillar(crate::world_editor::WorldEditor& editor, int x, int highway_y, int z, int dx, int dz, int /*_block_range*/) {
    using crate::block_definitions::STONE_BRICKS;
    if (dx == 0 && dz == 0 && ((x + z) % 8) == 0) {
        for (int y = 1; y < highway_y; ++y) {
            editor.set_block(STONE_BRICKS, x, y, z, std::optional<std::vector<crate::block_definitions::Block>>(), std::optional<std::vector<crate::block_definitions::Block>>());
        }
        for (int base_dx = -1; base_dx <= 1; ++base_dx) {
            for (int base_dz = -1; base_dz <= 1; ++base_dz) {
                editor.set_block(STONE_BRICKS, x + base_dx, 0, z + base_dz, std::optional<std::vector<crate::block_definitions::Block>>(), std::optional<std::vector<crate::block_definitions::Block>>());
            }
        }
    }
}

// Add support pillars for bridges using absolute Y coordinates
// Pillars extend from ground level up to the bridge deck
void add_highway_support_pillar_absolute(crate::world_editor::WorldEditor& editor, int x, int bridge_deck_y, int z, int dx, int dz, int /*_block_range*/) {
    using crate::block_definitions::STONE_BRICKS;
    if (dx == 0 && dz == 0 && ((x + z) % 8) == 0) {
        // Get the actual ground level at this position
        int ground_y = 0;
        auto* ground = editor.get_ground();
        if (ground) {
            ground_y = ground->level(crate::coordinate_system::cartesian::XZPoint(x, z));
        }

        // Add pillar from ground up to bridge deck
        // Only if the bridge is actually above the ground
        if (bridge_deck_y > ground_y) {
            for (int y = ground_y + 1; y < bridge_deck_y; ++y) {
                editor.set_block_absolute(STONE_BRICKS, x, y, z, std::optional<std::vector<crate::block_definitions::Block>>(), std::optional<std::vector<crate::block_definitions::Block>>());
            }

            // Add pillar base at ground level
            for (int base_dx = -1; base_dx <= 1; ++base_dx) {
                for (int base_dz = -1; base_dz <= 1; ++base_dz) {
                    editor.set_block_absolute(STONE_BRICKS, x + base_dx, ground_y, z + base_dz, std::optional<std::vector<crate::block_definitions::Block>>(), std::optional<std::vector<crate::block_definitions::Block>>());
                }
            }
        }
    }
}

bool should_add_slope_at_node(const crate::osm_parser::ProcessedNode& node, int current_layer, const std::unordered_map<std::pair<int,int>, std::vector<int>, PairHash>& highway_connectivity) {
    std::pair<int,int> node_coord = { node.x, node.z };
    if (highway_connectivity.empty()) {
        return current_layer != 0;
    }
    auto it = highway_connectivity.find(node_coord);
    if (it != highway_connectivity.end()) {
        const std::vector<int>& connected_layers = it->second;
        std::size_t same_layer_count = 0;
        for (int layer : connected_layers) {
            if (layer == current_layer) {
                ++same_layer_count;
            }
        }
        if (same_layer_count <= 1) {
            return current_layer != 0;
        }
        return false;
    } else {
        return current_layer != 0;
    }
}

std::size_t calculate_way_length(const crate::osm_parser::ProcessedWay& way) {
    std::size_t total_length = 0;
    const crate::osm_parser::ProcessedNode* prev = nullptr;
    for (const auto& node : way.nodes) {
        if (prev != nullptr) {
            int dx = (node.x - prev->x);
            int dz = (node.z - prev->z);
            double seg = std::sqrt(static_cast<double>(dx*dx + dz*dz));
            total_length += static_cast<std::size_t>(seg);
        }
        prev = &node;
    }
    return total_length;
}

std::size_t calculate_total_bresenham_length(const crate::osm_parser::ProcessedWay& way) {
    if (way.nodes.size() < 2) {
        return 0;
    }
    std::size_t total_length = 1;
    for (std::size_t i = 1; i < way.nodes.size(); ++i) {
        const auto& prev = way.nodes[i - 1];
        const auto& cur = way.nodes[i];
        total_length += static_cast<std::size_t>(
                std::max(std::abs(cur.x - prev.x), std::abs(cur.z - prev.z)));
    }
    return total_length;
}

std::vector<std::pair<int, int>> stair_fill_cells(std::pair<int, int> prev,
        std::pair<int, int> curr) {
    std::vector<std::pair<int, int>> cells;
    int x = prev.first;
    int z = prev.second;
    while (x != curr.first || z != curr.second) {
        if (x != curr.first) {
            x += (curr.first - x) > 0 ? 1 : -1;
            cells.emplace_back(x, z);
        }
        if (z != curr.second) {
            z += (curr.second - z) > 0 ? 1 : -1;
            cells.emplace_back(x, z);
        }
    }
    if (cells.empty()) {
        cells.push_back(curr);
    }
    return cells;
}

int highway_block_range(const std::string& highway_type,
        const std::unordered_map<std::string, std::string>& tags,
        double scale) {
    int block_range = 2;
    if (highway_type == "footway" || highway_type == "pedestrian" ||
            highway_type == "path" || highway_type == "track" ||
            highway_type == "secondary_link" || highway_type == "tertiary_link" ||
            highway_type == "escape" || highway_type == "steps") {
        block_range = 1;
    } else if (highway_type == "motorway" || highway_type == "primary" ||
            highway_type == "trunk") {
        block_range = 5;
    } else if (highway_type == "secondary") {
        block_range = 4;
    } else if (highway_type == "tertiary") {
        block_range = 2;
    } else if (highway_type == "service") {
        block_range = 2;
    } else {
        auto it_lanes = tags.find("lanes");
        if (it_lanes != tags.end()) {
            if (it_lanes->second == "2")
                block_range = 3;
            else if (it_lanes->second != "1")
                block_range = 4;
        }
    }
    if (scale < 1.0)
        block_range = static_cast<int>(std::floor(block_range * scale));
    return std::max(0, block_range);
}

int calculate_point_elevation(std::size_t segment_index, std::size_t point_index, std::size_t segment_length, std::size_t total_segments, int base_elevation, bool needs_start_slope, bool needs_end_slope, std::size_t slope_length) {
    if (!needs_start_slope && !needs_end_slope) {
        return base_elevation;
    }
    std::size_t total_distance_from_start = segment_index * segment_length + point_index;
    std::size_t total_way_length = total_segments * segment_length;
    if (total_way_length == 0 || slope_length == 0) {
        return base_elevation;
    }
    if (needs_start_slope && total_distance_from_start <= slope_length) {
        float slope_progress = static_cast<float>(total_distance_from_start) / static_cast<float>(slope_length);
        int elevation_offset = static_cast<int>(static_cast<float>(base_elevation) * slope_progress);
        return elevation_offset;
    }
    if (needs_end_slope && total_distance_from_start >= (total_way_length > slope_length ? total_way_length - slope_length : 0)) {
        std::size_t distance_from_end = (total_way_length > total_distance_from_start) ? (total_way_length - total_distance_from_start) : 0;
        float slope_progress = static_cast<float>(distance_from_end) / static_cast<float>(slope_length);
        int elevation_offset = static_cast<int>(static_cast<float>(base_elevation) * slope_progress);
        return elevation_offset;
    }
    return base_elevation;
}

void generate_highways_internal(crate::world_editor::WorldEditor& editor,
        const crate::osm_parser::ProcessedElement& element,
        const crate::args::Args& args,
        const std::unordered_map<std::pair<int,int>, std::vector<int>, PairHash>& highway_connectivity,
        const std::optional<std::chrono::duration<double>>& floodfill_timeout,
        const crate::RoadMaskBitmap& road_mask,
        const crate::bridges::BridgeStructureMap& bridge_structures,
        const crate::bridges::BridgeSurfaceMap& bridge_surface) {
    using crate::block_definitions::Block;
    using crate::block_definitions::COBBLESTONE_WALL;
    using crate::block_definitions::OAK_FENCE;
    using crate::block_definitions::GLOWSTONE;
    using crate::block_definitions::GREEN_WOOL;
    using crate::block_definitions::YELLOW_WOOL;
    using crate::block_definitions::RED_WOOL;
    using crate::block_definitions::WHITE_WOOL;
    (void)COBBLESTONE_WALL; (void)OAK_FENCE; (void)GLOWSTONE; (void)GREEN_WOOL; (void)YELLOW_WOOL; (void)RED_WOOL; (void)WHITE_WOOL;
    (void)road_mask;

    auto it_highway = element.tags().find("highway");
    if (it_highway == element.tags().end()) {
        return;
    }
    const std::string& highway_type = it_highway->second;
    
    // Check if this is a bridge - bridges need special elevation handling
    // to span across valleys instead of following terrain
    // Accept any bridge tag value except "no" (e.g., "yes", "viaduct", "aqueduct", etc.)
    bool is_bridge = false;
    auto it_bridge = element.tags().find("bridge");
    if (it_bridge != element.tags().end() && it_bridge->second != "no") {
        is_bridge = true;
    }
    
    if (highway_type == "street_lamp") {
        if (element.type == crate::osm_parser::ElementType::Node && element.node.has_value()) {
            int x = element.node->x;
            int z = element.node->z;
            editor.set_block(crate::block_definitions::COBBLESTONE_WALL, x, 1, z, std::optional<std::vector<Block>>(), std::optional<std::vector<Block>>());
            for (int dy = 2; dy <= 4; ++dy) {
                editor.set_block(crate::block_definitions::OAK_FENCE, x, dy, z, std::optional<std::vector<Block>>(), std::optional<std::vector<Block>>());
            }
            editor.set_block(crate::block_definitions::GLOWSTONE, x, 5, z, std::optional<std::vector<Block>>(), std::optional<std::vector<Block>>());
        }
        return;
    } else if (highway_type == "crossing") {
        auto it_crossing = element.tags().find("crossing");
        if (it_crossing != element.tags().end() && it_crossing->second == "traffic_signals") {
            if (element.type == crate::osm_parser::ElementType::Node && element.node.has_value()) {
                int x = element.node->x;
                int z = element.node->z;
                for (int dy = 1; dy <= 3; ++dy) {
                    editor.set_block(crate::block_definitions::COBBLESTONE_WALL, x, dy, z, std::optional<std::vector<Block>>(), std::optional<std::vector<Block>>());
                }
                editor.set_block(GREEN_WOOL, x, 4, z, std::optional<std::vector<Block>>(), std::optional<std::vector<Block>>());
                editor.set_block(YELLOW_WOOL, x, 5, z, std::optional<std::vector<Block>>(), std::optional<std::vector<Block>>());
                editor.set_block(RED_WOOL, x, 6, z, std::optional<std::vector<Block>>(), std::optional<std::vector<Block>>());
            }
        }
        return;
    } else if (highway_type == "bus_stop") {
        if (element.type == crate::osm_parser::ElementType::Node && element.node.has_value()) {
            int x = element.node->x;
            int z = element.node->z;
            for (int dy = 1; dy <= 3; ++dy) {
                editor.set_block(crate::block_definitions::COBBLESTONE_WALL, x, dy, z, std::optional<std::vector<Block>>(), std::optional<std::vector<Block>>());
            }
            editor.set_block(crate::block_definitions::WHITE_WOOL, x, 4, z, std::optional<std::vector<Block>>(), std::optional<std::vector<Block>>());
            editor.set_block(crate::block_definitions::WHITE_WOOL, x + 1, 4, z, std::optional<std::vector<Block>>(), std::optional<std::vector<Block>>());
        }
        return;
    } else {
        auto it_area = element.tags().find("area");
        if (it_area != element.tags().end() && it_area->second == "yes") {
            if (element.type != crate::osm_parser::ElementType::Way || !element.way.has_value()) {
                return;
            }
            const crate::osm_parser::ProcessedWay& way = *element.way;
            auto surface_blocks = crate::surfaces::get_blocks_for_surface_way(
                    way, std::vector<crate::block_definitions::Block>{crate::block_definitions::STONE});
            std::vector<std::pair<int,int>> polygon_coords;
            polygon_coords.reserve(way.nodes.size());
            for (const auto& n : way.nodes) {
                polygon_coords.emplace_back(n.x, n.z);
            }
            std::vector<std::pair<int,int>> filled_area = crate::floodfill::flood_fill_area(polygon_coords, floodfill_timeout);
            for (const auto& p : filled_area) {
                const auto surface_block = crate::surfaces::semirandom_surface(
                        p.first, p.second, surface_blocks);
                editor.set_block(surface_block, p.first, 0, p.second, std::optional<std::vector<Block>>(), std::optional<std::vector<Block>>());
            }
            return;
        }
    }

    // Main highway/walkway processing below
    crate::block_definitions::Block block_type = crate::block_definitions::BLACK_CONCRETE;
    int block_range = 2;
    bool add_stripe = false;
    bool add_outline = false;
    double scale_factor = args.scale;

    int layer_value = 0;
    auto it_layer = element.tags().find("layer");
    if (it_layer != element.tags().end()) {
        try {
            layer_value = std::stoi(it_layer->second);
        } catch (...) {
            layer_value = 0;
        }
    }
    // Treat negative layers as ground level (0)
    if (layer_value < 0) {
        layer_value = 0;
    }

    auto it_level = element.tags().find("level");
    if (it_level != element.tags().end()) {
        try {
            int level_val = std::stoi(it_level->second);
            if (level_val < 0) return;
        } catch (...) {
            // ignore parse errors
        }
    }

    if (highway_type == "footway" || highway_type == "pedestrian") {
        block_type = crate::block_definitions::GRAY_CONCRETE;
        block_range = 1;
    } else if (highway_type == "path") {
        block_type = crate::block_definitions::DIRT_PATH;
        block_range = 1;
    } else if (highway_type == "motorway" || highway_type == "primary" || highway_type == "trunk") {
        block_range = 5;
        add_stripe = true;
    } else if (highway_type == "secondary") {
        block_range = 4;
        add_stripe = true;
    } else if (highway_type == "tertiary") {
        add_stripe = true;
    } else if (highway_type == "track") {
        block_range = 1;
    } else if (highway_type == "service") {
        block_type = crate::block_definitions::GRAY_CONCRETE;
        block_range = 2;
    } else if (highway_type == "secondary_link" || highway_type == "tertiary_link") {
        // Exit ramps, sliproads
        block_type = crate::block_definitions::BLACK_CONCRETE;
        block_range = 1;
    } else if (highway_type == "escape") {
        // Sand trap for vehicles on mountainous roads
        block_type = crate::block_definitions::SAND;
        block_range = 1;
    } else if (highway_type == "steps") {
        // TODO: Add correct stairs respecting height, step_count, etc.
        block_type = crate::block_definitions::GRAY_CONCRETE;
        block_range = 1;
    } else {
        auto it_lanes = element.tags().find("lanes");
        if (it_lanes != element.tags().end()) {
            const std::string& lanes = it_lanes->second;
            if (lanes == "2") {
                block_range = 3;
                add_stripe = true;
                add_outline = true;
            } else if (lanes != "1") {
                block_range = 4;
                add_stripe = true;
                add_outline = true;
            }
        }
    }

    if (element.type != crate::osm_parser::ElementType::Way || !element.way.has_value()) {
        return;
    }
    const crate::osm_parser::ProcessedWay& way = *element.way;
    const auto* bridge_member = bridge_structures.lookup_member(way.id);
    const auto* bridge_ramp = bridge_structures.lookup_ramp(way.id);
    const bool is_bridge_member = bridge_member != nullptr;
    const bool is_bridge_ramp = bridge_ramp != nullptr;

    std::vector<crate::block_definitions::Block> default_surface = DEFAULT_ROAD_MIX;
    if (highway_type == "footway" || highway_type == "pedestrian" || highway_type == "steps") {
        default_surface = {crate::block_definitions::SMOOTH_STONE};
    } else if (highway_type == "path" || highway_type == "track") {
        default_surface = {crate::block_definitions::DIRT_PATH};
    } else if (highway_type == "escape") {
        default_surface = {crate::block_definitions::SAND};
    } else if (block_type != crate::block_definitions::BLACK_CONCRETE) {
        default_surface = {block_type};
    }
    const auto block_types = crate::surfaces::get_blocks_for_surface_way(way, default_surface);

    if (scale_factor < 1.0) {
        block_range = static_cast<int>(std::floor(static_cast<double>(block_range) * scale_factor));
    }

    const int LAYER_HEIGHT_STEP = 6;
    int base_elevation = layer_value * LAYER_HEIGHT_STEP;

    bool needs_start_slope = false;
    bool needs_end_slope = false;
    if (!way.nodes.empty()) {
        needs_start_slope = should_add_slope_at_node(way.nodes.front(), layer_value, highway_connectivity);
        needs_end_slope = should_add_slope_at_node(way.nodes.back(), layer_value, highway_connectivity);
    }

    std::size_t total_way_length = calculate_way_length(way);

    // For bridges: detect if this spans a valley by checking terrain profile
    // A valley bridge has terrain that dips significantly below the endpoints
    // Skip valley detection entirely if terrain is disabled (no valleys in flat terrain)
    // Skip very short bridges (< 25 blocks) as they're unlikely to span significant valleys
    bool terrain_enabled = false;
    auto* ground = editor.get_ground();
    if (ground) {
        terrain_enabled = true; // Assuming elevation is always enabled in this context
    }

    bool is_valley_bridge = false;
    int bridge_deck_y = 0;
    if (is_bridge && terrain_enabled && way.nodes.size() >= 2 && total_way_length >= 25) {
        const crate::osm_parser::ProcessedNode& start_node = way.nodes.front();
        const crate::osm_parser::ProcessedNode& end_node = way.nodes.back();
        // Get ground reference from editor
        auto* ground = editor.get_ground();
        int start_y = 0;
        int end_y = 0;
        if (ground) {
            start_y = ground->level(crate::coordinate_system::cartesian::XZPoint(start_node.x, start_node.z));
            end_y = ground->level(crate::coordinate_system::cartesian::XZPoint(end_node.x, end_node.z));
        }
        int max_endpoint_y = std::max(start_y, end_y);

        // Sample terrain at middle nodes only (excluding endpoints we already have)
        // This avoids redundant get_ground_level() calls
        const std::vector<crate::osm_parser::ProcessedNode> middle_nodes(way.nodes.begin() + 1, way.nodes.end() - 1);
        int sampled_min = max_endpoint_y;
        if (!middle_nodes.empty()) {
            // Sample up to 3 middle points (5 total with endpoints) for performance
            // Valleys are wide terrain features, so sparse sampling is sufficient
            std::size_t sample_count = std::min(static_cast<std::size_t>(3), middle_nodes.size());
            std::size_t step = (sample_count > 1) ? (middle_nodes.size() - 1) / (sample_count - 1) : 1;

            for (std::size_t i = 0; i < middle_nodes.size(); i += std::max(static_cast<std::size_t>(1), step)) {
                const auto& node = middle_nodes[i];
                int node_y = 0;
                if (ground) {
                    node_y = ground->level(crate::coordinate_system::cartesian::XZPoint(node.x, node.z));
                }
                sampled_min = std::min(sampled_min, node_y);
            }
        }

        // Include endpoint elevations in the minimum calculation
        int min_terrain_y = std::min({sampled_min, start_y, end_y});

        // If ANY sampled point along the bridge is significantly lower than the max endpoint,
        // treat as valley bridge
        is_valley_bridge = (min_terrain_y < max_endpoint_y - VALLEY_BRIDGE_THRESHOLD);

        if (is_valley_bridge) {
            bridge_deck_y = max_endpoint_y;
        }
    }

    const std::size_t total_bresenham_length = calculate_total_bresenham_length(way);
    const std::size_t raw_bridge_ramp_length = static_cast<std::size_t>(
            std::clamp(static_cast<float>(total_bresenham_length) * 0.35f, 15.0f, 50.0f));
    const std::size_t bridge_internal_ramp_length = std::clamp<std::size_t>(
            raw_bridge_ramp_length, 1,
            std::max<std::size_t>(1, total_bresenham_length / 2));

    // Check if this is a short isolated elevated segment (layer > 0), if so, treat as ground level
    bool is_short_isolated_elevated = (!is_bridge_member && !is_bridge_ramp &&
            needs_start_slope && needs_end_slope && layer_value > 0 && total_way_length <= 35);

    // Override elevation and slopes for short isolated segments
    int effective_elevation = 0;
    bool effective_start_slope = false;
    bool effective_end_slope = false;
    if (is_bridge_member || is_bridge_ramp || is_short_isolated_elevated) {
        effective_elevation = 0;
        effective_start_slope = false;
        effective_end_slope = false;
    } else {
        effective_elevation = base_elevation;
        effective_start_slope = needs_start_slope;
        effective_end_slope = needs_end_slope;
    }

    std::size_t slope_length = static_cast<std::size_t>(std::clamp(static_cast<double>(total_way_length) * 0.35, 15.0, 50.0));

    std::optional<std::pair<int,int>> previous_node;
    std::size_t segment_index = 0;
    std::size_t total_segments = (way.nodes.size() > 0) ? (way.nodes.size() - 1) : 0;
    std::size_t cumulative_distance_from_start = 0;
    std::optional<int> previous_bridge_y;
    std::optional<std::pair<int, int>> previous_rail_left;
    std::optional<std::pair<int, int>> previous_rail_right;

    for (const auto& node : way.nodes) {
        if (previous_node.has_value()) {
            int x1 = previous_node->first;
            int z1 = previous_node->second;
            int x2 = node.x;
            int z2 = node.z;
            std::vector<std::tuple<int,int,int>> bresenham_points = crate::bresenham::bresenham_line(x1, 0, z1, x2, 0, z2);
            std::size_t segment_length = bresenham_points.size();

            int stripe_length_counter = 0;
            int dash_length = static_cast<int>(std::ceil(5.0 * scale_factor));
            int gap_length = static_cast<int>(std::ceil(5.0 * scale_factor));
            const bool bridge_like = is_bridge_member || is_bridge_ramp;
            const std::size_t skip_first = bridge_like && segment_index > 0 ? 1 : 0;
            std::optional<std::pair<float, float>> bridge_rail_perp;
            if (bridge_like) {
                const float dx_seg = static_cast<float>(x2 - x1);
                const float dz_seg = static_cast<float>(z2 - z1);
                const float seg_len = std::sqrt(dx_seg * dx_seg + dz_seg * dz_seg);
                if (seg_len > 0.0f) {
                    bridge_rail_perp = std::make_pair(-dz_seg / seg_len, dx_seg / seg_len);
                }
            }

            for (std::size_t point_index = skip_first; point_index < bresenham_points.size(); ++point_index) {
                int x = std::get<0>(bresenham_points[point_index]);
                int z = std::get<2>(bresenham_points[point_index]);
                const std::size_t tds = cumulative_distance_from_start + point_index;
                
                // Calculate Y elevation for this point
                // Bridge structures drive absolute Y for full bridge groups and their ramps.
                int current_y;
                bool use_absolute_y = false;
                if (bridge_member) {
                    current_y = bridge_member->y_at(
                            tds, total_bresenham_length, bridge_internal_ramp_length);
                    use_absolute_y = true;
                } else if (bridge_ramp) {
                    current_y = bridge_ramp->y_at(tds, total_bresenham_length);
                    use_absolute_y = true;
                } else if (is_valley_bridge) {
                    // Valley bridge deck is level at the maximum endpoint elevation
                    // Don't add base_elevation - the layer tag indicates it's above water/road,
                    // not that it should be higher than the terrain endpoints
                    current_y = bridge_deck_y;
                    use_absolute_y = true;
                } else {
                    // Regular road or overpass: use terrain-relative calculation with ramps
                    current_y = calculate_point_elevation(segment_index, point_index, segment_length, total_segments, effective_elevation, effective_start_slope, effective_end_slope, slope_length);
                    use_absolute_y = false;
                }
                if (auto deck_y = bridge_surface.deck_y_at(x, z)) {
                    current_y = *deck_y;
                    use_absolute_y = true;
                }

                if (bridge_like) {
                    if (previous_bridge_y.has_value()) {
                        int fill_lo = 0;
                        int fill_hi = -1;
                        if (current_y >= *previous_bridge_y + 3) {
                            fill_lo = *previous_bridge_y + 1;
                            fill_hi = current_y - 2;
                        } else if (current_y <= *previous_bridge_y - 3) {
                            fill_lo = current_y + 1;
                            fill_hi = *previous_bridge_y - 2;
                        }
                        if (fill_lo <= fill_hi) {
                            for (int fill_y = fill_lo; fill_y <= fill_hi; ++fill_y) {
                                for (int fdx = -block_range; fdx <= block_range; ++fdx) {
                                    for (int fdz = -block_range; fdz <= block_range; ++fdz) {
                                        editor.set_block_absolute(
                                                crate::block_definitions::STONE_BRICKS,
                                                x + fdx, fill_y, z + fdz,
                                                std::optional<std::vector<crate::block_definitions::Block>>(),
                                                std::optional<std::vector<crate::block_definitions::Block>>(ROAD_PROTECTED_SURFACES));
                                    }
                                }
                            }
                        }
                    }
                    previous_bridge_y = current_y;
                }

                for (int dx = -block_range; dx <= block_range; ++dx) {
                    for (int dz = -block_range; dz <= block_range; ++dz) {
                        int set_x = x + dx;
                        int set_z = z + dz;

                        bool zebra = false;
                        if (highway_type == "footway") {
                            auto it_footway = element.tags().find("footway");
                            if (it_footway != element.tags().end() && it_footway->second == "crossing") {
                                bool is_horizontal = (std::abs(x2 - x1) >= std::abs(z2 - z1));
                                if (is_horizontal) {
                                    if ((set_x % 2 + 2) % 2 < 1) zebra = true;
                                } else {
                                    if ((set_z % 2 + 2) % 2 < 1) zebra = true;
                                }
                            }
                        }

                        if (zebra) {
                            if (use_absolute_y) {
                                editor.set_block_absolute(crate::block_definitions::WHITE_CONCRETE, set_x, current_y, set_z, std::optional<std::vector<crate::block_definitions::Block>>({ crate::block_definitions::BLACK_CONCRETE }), std::optional<std::vector<crate::block_definitions::Block>>());
                            } else {
                                editor.set_block(crate::block_definitions::WHITE_CONCRETE, set_x, current_y, set_z, std::optional<std::vector<crate::block_definitions::Block>>({ crate::block_definitions::BLACK_CONCRETE }), std::optional<std::vector<crate::block_definitions::Block>>());
                            }
                        } else {
                            const auto road_block = crate::surfaces::semirandom_surface(set_x, set_z, block_types);
                            if (use_absolute_y) {
                                editor.set_block_absolute(road_block, set_x, current_y, set_z, std::optional<std::vector<crate::block_definitions::Block>>(), std::optional<std::vector<crate::block_definitions::Block>>(ROAD_PROTECTED_SURFACES));
                            } else {
                                editor.set_block(road_block, set_x, current_y, set_z, std::optional<std::vector<crate::block_definitions::Block>>(), std::optional<std::vector<crate::block_definitions::Block>>(ROAD_PROTECTED_SURFACES));
                            }
                        }

                        // Add stone brick foundation underneath elevated highways/bridges for thickness
                        if ((effective_elevation > 0 || use_absolute_y) && current_y > 0) {
                            // Add 1 layer of stone bricks underneath the highway surface
                            if (use_absolute_y) {
                                editor.set_block_absolute(crate::block_definitions::STONE_BRICKS, set_x, current_y - 1, set_z, std::optional<std::vector<crate::block_definitions::Block>>(), std::optional<std::vector<crate::block_definitions::Block>>());
                            } else {
                                editor.set_block(crate::block_definitions::STONE_BRICKS, set_x, current_y - 1, set_z, std::optional<std::vector<crate::block_definitions::Block>>(), std::optional<std::vector<crate::block_definitions::Block>>());
                            }
                        }

                        // Add support pillars for elevated highways/bridges
                        if ((effective_elevation != 0 || use_absolute_y) && current_y > 0) {
                            if (use_absolute_y) {
                                add_highway_support_pillar_absolute(editor, set_x, current_y, set_z, dx, dz, block_range);
                            } else {
                                add_highway_support_pillar(editor, set_x, current_y, set_z, dx, dz, block_range);
                            }
                        }
                    }
                }

                if (bridge_like && bridge_rail_perp.has_value()) {
                    const float perp_x = bridge_rail_perp->first;
                    const float perp_z = bridge_rail_perp->second;
                    const float rail_dist = static_cast<float>(block_range) *
                            (std::abs(perp_x) + std::abs(perp_z)) + 1.0f;
                    for (int side = 0; side < 2; ++side) {
                        const float sign = side == 0 ? 1.0f : -1.0f;
                        auto& previous_rail = side == 0 ? previous_rail_left : previous_rail_right;
                        const int rail_x = static_cast<int>(
                                std::round(static_cast<float>(x) + perp_x * rail_dist * sign));
                        const int rail_z = static_cast<int>(
                                std::round(static_cast<float>(z) + perp_z * rail_dist * sign));
                        const std::pair<int, int> rail_cell{rail_x, rail_z};
                        const auto cells_to_fill = previous_rail.has_value() ?
                                stair_fill_cells(*previous_rail, rail_cell) :
                                std::vector<std::pair<int, int>>{rail_cell};
                        for (const auto& rail_fill : cells_to_fill) {
                            const int rx = rail_fill.first;
                            const int rz = rail_fill.second;
                            if (bridge_surface.contains(rx, rz)) {
                                continue;
                            }
                            editor.set_block_absolute(
                                    crate::block_definitions::LIGHT_GRAY_CONCRETE,
                                    rx, current_y, rz,
                                    std::optional<std::vector<crate::block_definitions::Block>>(),
                                    std::optional<std::vector<crate::block_definitions::Block>>(ROAD_PROTECTED_SURFACES));
                            if (current_y > 0) {
                                editor.set_block_absolute(
                                        crate::block_definitions::STONE_BRICKS,
                                        rx, current_y - 1, rz,
                                        std::optional<std::vector<crate::block_definitions::Block>>(),
                                        std::optional<std::vector<crate::block_definitions::Block>>());
                            }
                            editor.set_block_absolute(
                                    crate::block_definitions::BRICK_WALL,
                                    rx, current_y + 1, rz,
                                    std::optional<std::vector<crate::block_definitions::Block>>(),
                                    std::optional<std::vector<crate::block_definitions::Block>>());
                        }
                        previous_rail = rail_cell;
                    }
                }

                if (add_outline) {
                    // Left outline
                    for (int dz = -block_range; dz <= block_range; ++dz) {
                        int outline_x = x - block_range - 1;
                        int outline_z = z + dz;
                        if (use_absolute_y) {
                            editor.set_block_absolute(crate::block_definitions::LIGHT_GRAY_CONCRETE, outline_x, current_y, outline_z, std::optional<std::vector<crate::block_definitions::Block>>(), std::optional<std::vector<crate::block_definitions::Block>>());
                        } else {
                            editor.set_block(crate::block_definitions::LIGHT_GRAY_CONCRETE, outline_x, current_y, outline_z, std::optional<std::vector<crate::block_definitions::Block>>(), std::optional<std::vector<crate::block_definitions::Block>>());
                        }
                    }
                    // Right outline
                    for (int dz = -block_range; dz <= block_range; ++dz) {
                        int outline_x = x + block_range + 1;
                        int outline_z = z + dz;
                        if (use_absolute_y) {
                            editor.set_block_absolute(crate::block_definitions::LIGHT_GRAY_CONCRETE, outline_x, current_y, outline_z, std::optional<std::vector<crate::block_definitions::Block>>(), std::optional<std::vector<crate::block_definitions::Block>>());
                        } else {
                            editor.set_block(crate::block_definitions::LIGHT_GRAY_CONCRETE, outline_x, current_y, outline_z, std::optional<std::vector<crate::block_definitions::Block>>(), std::optional<std::vector<crate::block_definitions::Block>>());
                        }
                    }
                }

                if (add_stripe) {
                    if (stripe_length_counter < dash_length) {
                        int stripe_x = x;
                        int stripe_z = z;
                        if (use_absolute_y) {
                            editor.set_block_absolute(crate::block_definitions::WHITE_CONCRETE, stripe_x, current_y, stripe_z, std::optional<std::vector<crate::block_definitions::Block>>(block_types), std::optional<std::vector<crate::block_definitions::Block>>());
                        } else {
                            editor.set_block(crate::block_definitions::WHITE_CONCRETE, stripe_x, current_y, stripe_z, std::optional<std::vector<crate::block_definitions::Block>>(block_types), std::optional<std::vector<crate::block_definitions::Block>>());
                        }
                    }
                    ++stripe_length_counter;
                    if (stripe_length_counter >= dash_length + gap_length) {
                        stripe_length_counter = 0;
                    }
                }
            }

            ++segment_index;
            if (!bresenham_points.empty()) {
                cumulative_distance_from_start += bresenham_points.size() - 1;
            }
        }
        previous_node = std::make_optional(std::pair<int,int>{ node.x, node.z });
    }
}

void generate_highways(crate::world_editor::WorldEditor& editor,
        const crate::osm_parser::ProcessedElement& element,
        const crate::args::Args& args,
        const std::vector<crate::osm_parser::ProcessedElement>& all_elements,
        const std::optional<std::chrono::duration<double>>& floodfill_timeout,
        const crate::RoadMaskBitmap& road_mask,
        const crate::bridges::BridgeStructureMap& bridge_structures,
        const crate::bridges::BridgeSurfaceMap& bridge_surface) {
    auto highway_connectivity = build_highway_connectivity_map(all_elements);
    generate_highways_internal(editor, element, args, highway_connectivity, floodfill_timeout,
            road_mask, bridge_structures, bridge_surface);
}

void generate_highways(crate::world_editor::WorldEditor& editor, const crate::osm_parser::ProcessedElement& element, const crate::args::Args& args, const std::vector<crate::osm_parser::ProcessedElement>& all_elements, const std::optional<std::chrono::duration<double>>& floodfill_timeout) {
    static const crate::RoadMaskBitmap empty_road_mask = crate::RoadMaskBitmap::new_empty();
    static const crate::bridges::BridgeStructureMap empty_bridge_structures;
    static const crate::bridges::BridgeSurfaceMap empty_bridge_surface;
    generate_highways(editor, element, args, all_elements, floodfill_timeout,
            empty_road_mask, empty_bridge_structures, empty_bridge_surface);
}

void generate_siding(crate::world_editor::WorldEditor& editor,
        const crate::osm_parser::ProcessedWay& element,
        const crate::bridges::BridgeSurfaceMap& bridge_surface) {
    std::optional<crate::coordinate_system::cartesian::XZPoint> previous_node;
    crate::block_definitions::Block siding_block = crate::block_definitions::STONE_BRICK_SLAB;
    for (const auto& node : element.nodes) {
        crate::coordinate_system::cartesian::XZPoint current_node = node.xz();
        if (previous_node.has_value()) {
            std::vector<std::tuple<int,int,int>> bresenham_points = crate::bresenham::bresenham_line(previous_node->x, 0, previous_node->z, current_node.x, 0, current_node.z);
            for (const auto& p : bresenham_points) {
                int bx = std::get<0>(p);
                int bz = std::get<2>(p);
                if (auto deck_y = bridge_surface.deck_y_at(bx, bz)) {
                    if (!editor.check_for_block_absolute(bx, *deck_y, bz, std::optional<std::vector<crate::block_definitions::Block>>(ROAD_PROTECTED_SURFACES))) {
                        editor.set_block_absolute(siding_block, bx, *deck_y + 1, bz, std::optional<std::vector<crate::block_definitions::Block>>(), std::optional<std::vector<crate::block_definitions::Block>>());
                    }
                } else if (!editor.check_for_block(bx, 0, bz, std::optional<std::vector<crate::block_definitions::Block>>(ROAD_PROTECTED_SURFACES))) {
                    editor.set_block(siding_block, bx, 1, bz, std::optional<std::vector<crate::block_definitions::Block>>(), std::optional<std::vector<crate::block_definitions::Block>>());
                }
            }
        }
        previous_node.emplace(current_node);
    }
}

void generate_siding(crate::world_editor::WorldEditor& editor, const crate::osm_parser::ProcessedWay& element) {
    static const crate::bridges::BridgeSurfaceMap empty_bridge_surface;
    generate_siding(editor, element, empty_bridge_surface);
}

void generate_aeroway(crate::world_editor::WorldEditor& editor, const crate::osm_parser::ProcessedWay& way, const crate::args::Args& args) {
    std::optional<std::pair<int,int>> previous_node;
    crate::block_definitions::Block surface_block = crate::block_definitions::LIGHT_GRAY_CONCRETE;
    for (const auto& node : way.nodes) {
        if (previous_node.has_value()) {
            int x1 = previous_node->first;
            int z1 = previous_node->second;
            int x2 = node.x;
            int z2 = node.z;
            std::vector<std::tuple<int,int,int>> points = crate::bresenham::bresenham_line(x1, 0, z1, x2, 0, z2);
            int way_width = static_cast<int>(std::ceil(12.0 * args.scale));
            for (const auto& p : points) {
                int x = std::get<0>(p);
                int z = std::get<2>(p);
                for (int dx = -way_width; dx <= way_width; ++dx) {
                    for (int dz = -way_width; dz <= way_width; ++dz) {
                        int set_x = x + dx;
                        int set_z = z + dz;
                        editor.set_block(surface_block, set_x, 0, set_z, std::optional<std::vector<crate::block_definitions::Block>>(), std::optional<std::vector<crate::block_definitions::Block>>());
                    }
                }
            }
        }
        previous_node = std::make_optional(std::pair<int,int>{ node.x, node.z });
    }
}

crate::CoordinateBitmap collect_road_surface_coords(
        const std::vector<crate::osm_parser::ProcessedElement>& elements,
        const ::XZBBox& xzbbox,
        double scale) {
    crate::CoordinateBitmap bitmap(xzbbox);
    for (const auto& element : elements) {
        if (!element.is_way())
            continue;
        const auto& way = element.as_way();
        auto it_highway = way.tags.find("highway");
        if (it_highway == way.tags.end())
            continue;
        const auto& highway_type = it_highway->second;
        if (highway_type == "street_lamp" || highway_type == "crossing" ||
                highway_type == "bus_stop")
            continue;
        if (way.tags.get("area") == "yes" || way.tags.get("indoor") == "yes")
            continue;
        try {
            auto it_level = way.tags.find("level");
            if (it_level != way.tags.end() && std::stoi(it_level->second) < 0)
                continue;
        } catch (...) {
        }

        const int block_range = highway_block_range(highway_type, way.tags, scale);
        for (std::size_t i = 1; i < way.nodes.size(); ++i) {
            const auto& prev = way.nodes[i - 1];
            const auto& cur = way.nodes[i];
            const auto points = crate::bresenham::bresenham_line(prev.x, 0, prev.z, cur.x, 0, cur.z);
            for (const auto& p : points) {
                const int bx = std::get<0>(p);
                const int bz = std::get<2>(p);
                for (int dx = -block_range; dx <= block_range; ++dx) {
                    for (int dz = -block_range; dz <= block_range; ++dz) {
                        bitmap.set(bx + dx, bz + dz);
                    }
                }
            }
        }
    }
    return bitmap;
}

crate::CoordinateBitmap collect_building_passage_coords(
        const std::vector<crate::osm_parser::ProcessedElement>& elements,
        const ::XZBBox& xzbbox,
        double scale) {
    bool has_any = false;
    for (const auto& element : elements) {
        if (element.is_way()) {
            const auto& way = element.as_way();
            if (way.tags.get("tunnel") == "building_passage" &&
                    way.tags.contains("highway")) {
                has_any = true;
                break;
            }
        }
    }
    if (!has_any)
        return crate::CoordinateBitmap::new_empty();

    crate::CoordinateBitmap bitmap(xzbbox);
    for (const auto& element : elements) {
        if (!element.is_way())
            continue;
        const auto& way = element.as_way();
        if (way.tags.get("tunnel") != "building_passage")
            continue;
        auto it_highway = way.tags.find("highway");
        if (it_highway == way.tags.end())
            continue;

        const int block_range = highway_block_range(it_highway->second, way.tags, scale);
        for (std::size_t i = 1; i < way.nodes.size(); ++i) {
            const auto& prev = way.nodes[i - 1];
            const auto& cur = way.nodes[i];
            const auto points = crate::bresenham::bresenham_line(prev.x, 0, prev.z, cur.x, 0, cur.z);
            for (const auto& p : points) {
                const int bx = std::get<0>(p);
                const int bz = std::get<2>(p);
                for (int dx = -block_range; dx <= block_range; ++dx) {
                    for (int dz = -block_range; dz <= block_range; ++dz) {
                        bitmap.set(bx + dx, bz + dz);
                    }
                }
            }
        }
    }
    return bitmap;
}

}
}
