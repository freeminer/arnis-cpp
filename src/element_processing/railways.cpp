#include <vector>
#include <tuple>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <optional>
#include <cmath>
#include <algorithm>
#include <limits>

using std::vector;
using std::tuple;
using std::get;
using std::string;
using std::unordered_map;
using std::optional;
using std::nullopt;
using std::pair;

#include "../bresenham.h"
#include "../../../arnis_adapter.h"
#include "bridge_styles.h"
#undef stoi
namespace arnis
{


namespace railways
{

const int SUBWAY_DEPTH = 3;
const int RAIL_BRIDGE_FLAT_CLEARANCE = 4;
const int RAIL_BRIDGE_DIP_THRESHOLD = 4;
const std::size_t RAIL_BRIDGE_RAMP_MIN = 8;
const std::size_t RAIL_BRIDGE_RAMP_MAX = 30;
const float RAIL_BRIDGE_RAMP_FRACTION = 0.25f;
const int WALL_RADIUS = 2;
const int AIR_RADIUS = 1;
const int INTERIOR_HEIGHT = 4;
const std::size_t LIGHT_INTERVAL = 8;
const int MIN_Y = -64;

struct PairHash {
    std::size_t operator()(const pair<int, int>& p) const noexcept {
        return std::hash<long long>()(
                (static_cast<long long>(p.first) << 32) ^
                static_cast<unsigned long long>(p.second));
    }
};

using RailBridgeInternalEndpoints = vector<pair<int, int>>;

bool contains_endpoint(const RailBridgeInternalEndpoints& endpoints, const pair<int, int>& xz) {
    return std::find(endpoints.begin(), endpoints.end(), xz) != endpoints.end();
}

Block subway_shell_block(int x, int y, int z) {
    uint32_t h = static_cast<uint32_t>(x) * 73856093u +
            static_cast<uint32_t>(y) * 19349663u +
            static_cast<uint32_t>(z) * 83492791u;
    const uint32_t v = h % 100u;
    if (v < 15)
        return CRACKED_STONE_BRICKS;
    if (v < 18)
        return MOSSY_STONE_BRICKS;
    return STONE_BRICKS;
}


// --- Block definitions (example) ---
#if 0
enum class Block {
    GRAVEL,
    OAK_LOG,
    RAIL_NORTH_SOUTH,
    RAIL_EAST_WEST,
    RAIL_NORTH_WEST,
    RAIL_NORTH_EAST,
    RAIL_SOUTH_WEST,
    RAIL_SOUTH_EAST,
    IRON_BLOCK,
    // ... add more as needed
};

// --- Minimal stubs for types used in functions ---
struct XZ {
    int x;
    int z;
};

struct Node {
    int bx;
    int bz;
    XZ xz() const { return {bx, bz}; }
};

struct ProcessedWay {
    vector<Node> nodes;
    unordered_map<string, string> tags;
};

// WorldEditor stub - replace with your real implementation
struct WorldEditor {
    // metadata arguments are optional placeholders to match Rust's None, None
    void set_block(Block block, int x, int y, int z, optional<int> = nullopt, optional<int> = nullopt) {
        // Implement block placement in your world/editor here.
        // This stub does nothing.
    }
};

// Assume this function exists somewhere in your codebase:
vector<tuple<int,int,int>> bresenham_line(int x1, int y1, int z1, int x2, int y2, int z2);

// --- Helper functions translated from Rust ---

#endif

vector<tuple<int,int,int>> smooth_diagonal_rails(const vector<tuple<int,int,int>>& points) {
    vector<tuple<int,int,int>> smoothed;
    smoothed.reserve(points.size() * 2);

    for (size_t i = 0; i < points.size(); ++i) {
        auto current = points[i];
        smoothed.push_back(current);

        if (i + 1 >= points.size()) continue;

        auto next = points[i + 1];
        int x1 = get<0>(current);
        int y1 = get<1>(current);
        int z1 = get<2>(current);
        int x2 = get<0>(next);
        int z2 = get<2>(next);

        // If points are diagonally adjacent
        if (std::abs(x2 - x1) == 1 && std::abs(z2 - z1) == 1) {
            optional<tuple<int,int,int>> look_ahead = (i + 2 < points.size()) ? optional<tuple<int,int,int>>(points[i + 2]) : nullopt;
            optional<tuple<int,int,int>> look_behind = (i > 0) ? optional<tuple<int,int,int>>(points[i - 1]) : nullopt;

            tuple<int,int,int> intermediate;
            if (look_behind) {
                int prev_x = get<0>(*look_behind);
                //int prev_z = get<2>(*look_behind);
                if (prev_x == x1) {
                    // Coming from vertical, keep x constant
                    intermediate = {x1, y1, z2};
                } else {
                    // Coming from horizontal, keep z constant
                    intermediate = {x2, y1, z1};
                }
            } else if (look_ahead) {
                int next_x = get<0>(*look_ahead);
                if (next_x == x2) {
                    // Going to vertical, keep x constant
                    intermediate = {x2, y1, z1};
                } else {
                    // Going to horizontal, keep z constant
                    intermediate = {x1, y1, z2};
                }
            } else {
                // Default to horizontal first if no context
                intermediate = {x2, y1, z1};
            }

            smoothed.push_back(intermediate);
        }
    }

    return smoothed;
}

Block determine_rail_direction(
    const pair<int,int>& current,
    const optional<pair<int,int>>& prev,
    const optional<pair<int,int>>& next
) {
    int x = current.first;
    int z = current.second;

    if (prev && next) {
        int px = prev->first;
        int pz = prev->second;
        int nx = next->first;
        int nz = next->second;

        if (px == nx) {
            return RAIL_NORTH_SOUTH;
        } else if (pz == nz) {
            return RAIL_EAST_WEST;
        } else {
            pair<int,int> from_prev = {px - x, pz - z};
            pair<int,int> to_next   = {nx - x, nz - z};

            // East to North or North to East
            if ((from_prev == pair<int,int>{-1,0} && to_next == pair<int,int>{0,-1})
                || (from_prev == pair<int,int>{0,-1} && to_next == pair<int,int>{-1,0})) {
                return RAIL_NORTH_WEST;
            }
            // West to North or North to West
            if ((from_prev == pair<int,int>{1,0} && to_next == pair<int,int>{0,-1})
                || (from_prev == pair<int,int>{0,-1} && to_next == pair<int,int>{1,0})) {
                return RAIL_NORTH_EAST;
            }
            // East to South or South to East
            if ((from_prev == pair<int,int>{-1,0} && to_next == pair<int,int>{0,1})
                || (from_prev == pair<int,int>{0,1} && to_next == pair<int,int>{-1,0})) {
                return RAIL_SOUTH_WEST;
            }
            // West to South or South to West
            if ((from_prev == pair<int,int>{1,0} && to_next == pair<int,int>{0,1})
                || (from_prev == pair<int,int>{0,1} && to_next == pair<int,int>{1,0})) {
                return RAIL_SOUTH_EAST;
            }

            if (std::abs(px - x) > std::abs(pz - z)) {
                return RAIL_EAST_WEST;
            } else {
                return RAIL_NORTH_SOUTH;
            }
        }
    } else if (prev || next) {
        pair<int,int> p = prev ? *prev : *next;
        int px = p.first;
        int pz = p.second;

        if (px == x) {
            return RAIL_NORTH_SOUTH;
        } else if (pz == z) {
            return RAIL_EAST_WEST;
        } else {
            return RAIL_NORTH_SOUTH;
        }
    } else {
        return RAIL_NORTH_SOUTH;
    }
}

Block ascending_toward(const pair<int,int>& from, const pair<int,int>& to) {
    int dx = to.first - from.first;
    int dz = to.second - from.second;
    if (std::abs(dx) >= std::abs(dz)) {
        return dx > 0 ? RAIL_ASCENDING_EAST : RAIL_ASCENDING_WEST;
    }
    return dz < 0 ? RAIL_ASCENDING_NORTH : RAIL_ASCENDING_SOUTH;
}

Block determine_rail_with_slope(
    const pair<int,int>& current,
    const optional<pair<int,int>>& prev,
    const optional<pair<int,int>>& next,
    int prev_ground,
    int current_ground,
    int next_ground
) {
    if (next_ground > current_ground && next)
        return ascending_toward(current, *next);
    if (prev_ground > current_ground && prev)
        return ascending_toward(current, *prev);
    return determine_rail_direction(current, prev, next);
}

bool is_rail_bridge(const ProcessedWay& way) {
    if (way.tags.get("indoor") == "yes")
        return false;
    auto it = way.tags.find("bridge");
    return it != way.tags.end() && it->second != "no";
}

bool renders_as_rail_bridge(const ProcessedWay& way) {
    auto it = way.tags.find("railway");
    if (it == way.tags.end() || way.nodes.size() < 2 || !is_rail_bridge(way))
        return false;
    const string& railway_type = it->second;
    if (railway_type == "subway" || way.tags.get("subway") == "yes")
        return false;
    const vector<string> skip_types = {
        "proposed", "abandoned", "construction", "razed", "turntable"
    };
    for (const auto& skip : skip_types) {
        if (railway_type == skip)
            return false;
    }
    return way.tags.get("tunnel") != "yes";
}

RailBridgeInternalEndpoints collect_rail_bridge_internal_endpoints(
        const vector<ProcessedElement>& elements) {
    std::unordered_map<pair<int,int>, int, PairHash> counts;
    for (const auto& element : elements) {
        if (!element.is_way())
            continue;
        const auto& way = element.as_way();
        if (!renders_as_rail_bridge(way))
            continue;
        const auto& s = way.nodes.front();
        const auto& e = way.nodes.back();
        ++counts[{s.x, s.z}];
        if (s.x != e.x || s.z != e.z)
            ++counts[{e.x, e.z}];
    }

    RailBridgeInternalEndpoints endpoints;
    for (const auto& [xz, count] : counts) {
        if (count > 1)
            endpoints.push_back(xz);
    }
    return endpoints;
}

// --- Main generation functions ---

void generate_at_grade_rail(WorldEditor& editor, const ProcessedWay& element) {
    std::size_t tds = 0;
    for (size_t i = 1; i < element.nodes.size(); ++i) {
        XZ prev_node = element.nodes[i - 1].xz();
        XZ cur_node  = element.nodes[i].xz();

        auto points = bresenham_line(prev_node.x, 0, prev_node.z, cur_node.x, 0, cur_node.z);
        auto smoothed_points = smooth_diagonal_rails(points);
        const std::size_t skip_first = i > 1 ? 1 : 0;

        for (size_t j = skip_first; j < smoothed_points.size(); ++j) {
            int bx = get<0>(smoothed_points[j]);
            int bz = get<2>(smoothed_points[j]);

            const int prev_ground = j > 0
                    ? editor.get_ground_level(get<0>(smoothed_points[j - 1]), get<2>(smoothed_points[j - 1]))
                    : editor.get_ground_level(bx, bz);
            const int next_ground = j + 1 < smoothed_points.size()
                    ? editor.get_ground_level(get<0>(smoothed_points[j + 1]), get<2>(smoothed_points[j + 1]))
                    : editor.get_ground_level(bx, bz);
            const int current_ground = editor.get_ground_level(bx, bz);

            if (prev_ground < current_ground) {
                for (int fill_y = prev_ground; fill_y < current_ground; ++fill_y)
                    editor.set_block_absolute(GRAVEL, bx, fill_y, bz, nullopt, nullopt);
            }

            editor.set_block(GRAVEL, bx, 0, bz, nullopt, nullopt);

            optional<pair<int,int>> prev_opt = nullopt;
            optional<pair<int,int>> next_opt = nullopt;
            if (j > 0)
                prev_opt = pair<int,int>{get<0>(smoothed_points[j - 1]), get<2>(smoothed_points[j - 1])};
            if (j + 1 < smoothed_points.size())
                next_opt = pair<int,int>{get<0>(smoothed_points[j + 1]), get<2>(smoothed_points[j + 1])};

            Block rail_block = determine_rail_with_slope(
                    {bx, bz}, prev_opt, next_opt, prev_ground, current_ground, next_ground);
            editor.set_block(rail_block, bx, 1, bz, nullopt, nullopt);

            if ((tds % 4) == 0)
                editor.set_block(OAK_LOG, bx, 0, bz, nullopt, nullopt);
            ++tds;
        }
    }
}

void generate_rail_bridge(WorldEditor& editor, const ProcessedWay& way,
        const RailBridgeInternalEndpoints& internal_endpoints,
        const bridge_styles::BridgeOutlineIndex& bridge_outlines) {
    if (way.nodes.size() < 2)
        return;

    const bridge_styles::BridgeStyle style =
            bridge_styles::resolve_bridge_style_with_outline(way, bridge_outlines);

    vector<pair<int,int>> all_points;
    for (size_t i = 1; i < way.nodes.size(); ++i) {
        auto points = bresenham_line(way.nodes[i - 1].x, 0, way.nodes[i - 1].z,
                way.nodes[i].x, 0, way.nodes[i].z);
        auto smoothed = smooth_diagonal_rails(points);
        for (const auto& point : smoothed) {
            pair<int,int> xz{get<0>(point), get<2>(point)};
            if (all_points.empty() || all_points.back() != xz)
                all_points.push_back(xz);
        }
    }
    if (all_points.empty())
        return;

    vector<int> terrain_ys;
    terrain_ys.reserve(all_points.size());
    int max_y = std::numeric_limits<int>::min();
    int min_y = std::numeric_limits<int>::max();
    for (const auto& [bx, bz] : all_points) {
        int y = editor.get_ground_level(bx, bz);
        terrain_ys.push_back(y);
        max_y = std::max(max_y, y);
        min_y = std::min(min_y, y);
    }

    const int flat_clearance = style == bridge_styles::BridgeStyle::Arch
            ? std::max(RAIL_BRIDGE_FLAT_CLEARANCE, 8)
            : RAIL_BRIDGE_FLAT_CLEARANCE;
    const int deck_y = (max_y - min_y) < RAIL_BRIDGE_DIP_THRESHOLD
            ? max_y + flat_clearance
            : max_y;
    const std::size_t total = all_points.size();
    const std::size_t last_idx = total - 1;
    const int start_ground = terrain_ys.front();
    const int end_ground = terrain_ys.back();
    const bool start_internal = contains_endpoint(internal_endpoints, all_points.front());
    const bool end_internal = contains_endpoint(internal_endpoints, all_points.back());

    std::size_t needed = 0;
    if (!start_internal)
        needed = std::max<std::size_t>(needed, std::max(0, deck_y - start_ground));
    if (!end_internal)
        needed = std::max<std::size_t>(needed, std::max(0, deck_y - end_ground));
    needed += 1;
    const std::size_t raw_ramp = static_cast<std::size_t>(total * RAIL_BRIDGE_RAMP_FRACTION);
    const std::size_t ramp_length = std::max(needed,
            std::clamp(raw_ramp, RAIL_BRIDGE_RAMP_MIN, RAIL_BRIDGE_RAMP_MAX));
    const float denom = static_cast<float>(std::max<std::size_t>(1, ramp_length - 1));

    vector<int> bridge_ys;
    bridge_ys.reserve(total);
    for (std::size_t tds = 0; tds < total; ++tds) {
        const int start_ramp_y = start_internal ? deck_y :
                static_cast<int>(std::round(start_ground + (deck_y - start_ground) *
                        std::min(1.0f, static_cast<float>(tds) / denom)));
        const std::size_t dist_from_end = last_idx > tds ? last_idx - tds : 0;
        const int end_ramp_y = end_internal ? deck_y :
                static_cast<int>(std::round(end_ground + (deck_y - end_ground) *
                        std::min(1.0f, static_cast<float>(dist_from_end) / denom)));
        bridge_ys.push_back(std::max(std::min(start_ramp_y, end_ramp_y), terrain_ys[tds]));
    }

    const Block foundation_block = bridge_styles::foundation_block(style);
    vector<bridge_styles::BridgePathSample> bridge_path;
    bridge_path.reserve(total);

    for (std::size_t i = 0; i < total; ++i) {
        const auto [bx, bz] = all_points[i];
        const int y = bridge_ys[i];
        optional<pair<int,int>> prev_xz = i > 0 ? optional<pair<int,int>>(all_points[i - 1]) : nullopt;
        optional<pair<int,int>> next_xz = i + 1 < total ? optional<pair<int,int>>(all_points[i + 1]) : nullopt;
        const int prev_y = i > 0 ? bridge_ys[i - 1] : y;
        const int next_y = i + 1 < total ? bridge_ys[i + 1] : y;
        const Block rail_block = determine_rail_with_slope({bx, bz}, prev_xz, next_xz, prev_y, y, next_y);

        editor.set_block_absolute(foundation_block, bx, y - 1, bz, nullopt, nullopt);
        editor.set_block_absolute((i % 4) == 0 ? OAK_LOG : GRAVEL, bx, y, bz, nullopt, nullopt);
        editor.set_block_absolute(rail_block, bx, y + 1, bz, nullopt, nullopt);

        const pair<int,int> p_prev = prev_xz.value_or(pair<int,int>{bx, bz});
        const pair<int,int> p_next = next_xz.value_or(pair<int,int>{bx, bz});
        const float dxp = static_cast<float>(p_next.first - p_prev.first);
        const float dzp = static_cast<float>(p_next.second - p_prev.second);
        const float mag = std::max(std::sqrt(dxp * dxp + dzp * dzp), 1.0e-6f);
        bridge_path.push_back({bx, y, bz, -dzp / mag, dxp / mag});

        const std::size_t pillar_interval = std::max<std::size_t>(bridge_styles::pillar_interval(style), 1);
        const bool is_pillar = (i % pillar_interval) == 0;
        bridge_styles::place_bridge_support_below_deck(editor, style, bx, y, bz,
                terrain_ys[i], i, total, true, true, is_pillar);
    }

    const bool start_is_boundary = !contains_endpoint(internal_endpoints, all_points.front());
    const bool end_is_boundary = !contains_endpoint(internal_endpoints, all_points.back());
    bridge_styles::decorate_bridge_above_deck(
            editor, style, bridge_path, 0, start_is_boundary, end_is_boundary);
}

void generate_subway_shell(WorldEditor& editor, const ProcessedWay& element,
        vector<pair<int,int>>& subway_points) {
    for (size_t i = 1; i < element.nodes.size(); ++i) {
        XZ prev_node = element.nodes[i - 1].xz();
        XZ cur_node  = element.nodes[i].xz();
        auto points = bresenham_line(prev_node.x, 0, prev_node.z, cur_node.x, 0, cur_node.z);
        auto smoothed = smooth_diagonal_rails(points);

        for (size_t j = 0; j < smoothed.size(); ++j) {
            int bx = get<0>(smoothed[j]);
            int bz = get<2>(smoothed[j]);
            if (subway_points.empty() || subway_points.back() != pair<int,int>{bx, bz})
                subway_points.emplace_back(bx, bz);

            const int ground_y = editor.get_ground_level(bx, bz);
            const int ceil_y = ground_y - SUBWAY_DEPTH;
            const int floor_y = ceil_y - INTERIOR_HEIGHT - 1;
            if (floor_y <= MIN_Y)
                continue;

            const int prev_ground = j > 0
                    ? editor.get_ground_level(get<0>(smoothed[j - 1]), get<2>(smoothed[j - 1]))
                    : ground_y;
            const int next_ground = j + 1 < smoothed.size()
                    ? editor.get_ground_level(get<0>(smoothed[j + 1]), get<2>(smoothed[j + 1]))
                    : ground_y;

            for (int dx = -WALL_RADIUS; dx <= WALL_RADIUS; ++dx) {
                for (int dz = -WALL_RADIUS; dz <= WALL_RADIUS; ++dz) {
                    for (int y = floor_y; y <= ceil_y; ++y) {
                        const bool wall_or_ceiling =
                                dx == -WALL_RADIUS || dx == WALL_RADIUS ||
                                dz == -WALL_RADIUS || dz == WALL_RADIUS ||
                                y == ceil_y;
                        Block block = y == floor_y ? POLISHED_DEEPSLATE :
                                (wall_or_ceiling ? subway_shell_block(bx + dx, y, bz + dz) : STONE_BRICKS);
                        editor.set_block_absolute(block, bx + dx, y, bz + dz, nullopt, nullopt);
                    }
                }
            }

            optional<pair<int,int>> prev_xz = j > 0
                    ? optional<pair<int,int>>(pair<int,int>{get<0>(smoothed[j - 1]), get<2>(smoothed[j - 1])})
                    : nullopt;
            optional<pair<int,int>> next_xz = j + 1 < smoothed.size()
                    ? optional<pair<int,int>>(pair<int,int>{get<0>(smoothed[j + 1]), get<2>(smoothed[j + 1])})
                    : nullopt;
            const Block rail_block = determine_rail_with_slope(
                    {bx, bz}, prev_xz, next_xz, prev_ground, ground_y, next_ground);
            editor.set_block_absolute(rail_block, bx, floor_y + 1, bz,
                    std::optional<std::vector<Block>>({STONE_BRICKS, CRACKED_STONE_BRICKS, MOSSY_STONE_BRICKS}),
                    std::optional<std::vector<Block>>());
        }
    }
}

void carve_subway_interior(WorldEditor& editor, const vector<pair<int,int>>& subway_points) {
    const std::optional<std::vector<Block>> carve_whitelist(
            std::vector<Block>{STONE_BRICKS, CRACKED_STONE_BRICKS, MOSSY_STONE_BRICKS, STONE});
    for (std::size_t idx = 0; idx < subway_points.size(); ++idx) {
        const auto [bx, bz] = subway_points[idx];
        const int ground_y = editor.get_ground_level(bx, bz);
        const int ceil_y = ground_y - SUBWAY_DEPTH;
        const int floor_y = ceil_y - INTERIOR_HEIGHT - 1;
        if (floor_y <= MIN_Y)
            continue;
        for (int dx = -AIR_RADIUS; dx <= AIR_RADIUS; ++dx) {
            for (int dz = -AIR_RADIUS; dz <= AIR_RADIUS; ++dz) {
                for (int y = floor_y + 1; y < ceil_y; ++y) {
                    if (dx == 0 && dz == 0 && y == floor_y + 1)
                        continue;
                    editor.set_block_absolute(AIR, bx + dx, y, bz + dz,
                            carve_whitelist, std::optional<std::vector<Block>>());
                }
            }
        }
        if (idx % LIGHT_INTERVAL == 0)
            editor.set_block_absolute(SEA_LANTERN, bx, ceil_y - 1, bz, nullopt, nullopt);
    }
}

void generate_railways(WorldEditor& editor, const ProcessedWay& element,
        vector<pair<int,int>>& subway_points,
        const RailBridgeInternalEndpoints& rail_bridge_internal_endpoints,
        const bridge_styles::BridgeOutlineIndex& bridge_outlines) {
    auto it = element.tags.find("railway");
    if (it == element.tags.end()) return;

    const string& railway_type = it->second;
    if (railway_type == "subway" || element.tags.get("subway") == "yes") {
        generate_subway_shell(editor, element, subway_points);
        return;
    }

    const vector<string> skip_types = {
        "proposed", "abandoned", "construction", "razed", "turntable"
    };
    for (const auto& s : skip_types) {
        if (railway_type == s) return;
    }

    if (auto it_tunnel = element.tags.find("tunnel"); it_tunnel != element.tags.end() && it_tunnel->second == "yes") {
        return;
    }

    if (element.nodes.size() < 2) return;
    if (is_rail_bridge(element)) {
        generate_rail_bridge(editor, element, rail_bridge_internal_endpoints, bridge_outlines);
    } else {
        generate_at_grade_rail(editor, element);
    }
}

void generate_railways(WorldEditor& editor, const ProcessedWay& element) {
    vector<pair<int,int>> subway_points;
    RailBridgeInternalEndpoints internal_endpoints;
    bridge_styles::BridgeOutlineIndex bridge_outlines;
    generate_railways(editor, element, subway_points, internal_endpoints, bridge_outlines);
}

void generate_roller_coaster(WorldEditor& editor, const ProcessedWay& element) {
    auto it = element.tags.find("roller_coaster");
    if (it == element.tags.end()) return;

    if (it->second != "track") return;

    // Skip indoor
    if (auto it_indoor = element.tags.find("indoor"); it_indoor != element.tags.end() && it_indoor->second == "yes") {
        return;
    }

    // Skip negative layer
    if (auto it_layer = element.tags.find("layer"); it_layer != element.tags.end()) {
        try {
            int layer_value = std::stoi(it_layer->second);
            if (layer_value < 0) return;
        } catch (...) {
            // parse error -> ignore layer
        }
    }

    const int elevation_height = 4; // 4 blocks in the air
    const int pillar_interval = 6;  // supports every 6 blocks

    if (element.nodes.size() < 2) return;

    for (size_t i = 1; i < element.nodes.size(); ++i) {
        XZ prev_node = element.nodes[i - 1].xz();
        XZ cur_node  = element.nodes[i].xz();

        auto points = bresenham_line(prev_node.x, 0, prev_node.z, cur_node.x, 0, cur_node.z);
        auto smoothed_points = smooth_diagonal_rails(points);

        for (size_t j = 0; j < smoothed_points.size(); ++j) {
            int bx = get<0>(smoothed_points[j]);
            //int by = get<1>(smoothed_points[j]);
            int bz = get<2>(smoothed_points[j]);

            // Foundation at elevation_height
            editor.set_block(IRON_BLOCK, bx, elevation_height, bz, nullopt, nullopt);

            optional<pair<int,int>> prev_opt = nullopt;
            optional<pair<int,int>> next_opt = nullopt;

            if (j > 0) {
                int px = get<0>(smoothed_points[j - 1]);
                int pz = get<2>(smoothed_points[j - 1]);
                prev_opt = pair<int,int>{px, pz};
            }
            if (j + 1 < smoothed_points.size()) {
                int nx = get<0>(smoothed_points[j + 1]);
                int nz = get<2>(smoothed_points[j + 1]);
                next_opt = pair<int,int>{nx, nz};
            }

            Block rail_block = determine_rail_direction({bx, bz}, prev_opt, next_opt);
            // Place rail on top of foundation
            editor.set_block(rail_block, bx, elevation_height + 1, bz, nullopt, nullopt);

            // Place support pillars every pillar_interval blocks
            if ((bx % pillar_interval) == 0 && (bz % pillar_interval) == 0) {
                for (int y = 1; y < elevation_height; ++y) {
                    editor.set_block(IRON_BLOCK, bx, y, bz, nullopt, nullopt);
                }
            }
        }
    }
}

}
}
