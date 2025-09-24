#include <array>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <string>
#include <optional>
#include <utility>

#include "../../../../arnis_adapter.h"
namespace arnis {



#if 0 
    namespace world_editor {

class WorldEditor {
public:
    void set_block_absolute(Block /*block*/,
                            int /*x*/, int /*y*/, int /*z*/,
                            std::optional<int> /*meta1*/,
                            std::optional<int> /*meta2*/) {
        // Stub: implementation depends on environment
    }
};

} // namespace world_editor

namespace args {
struct Args {
    bool roof = false;
};
} // namespace args

namespace osm_parser {
struct ProcessedWay {
    std::unordered_map<std::string, std::string> tags;
};
} // namespace osm_parser
} // namespace crate

#endif

// INTERIOR1_LAYER1
static constexpr std::array<std::array<char, 23>, 23> INTERIOR1_LAYER1 = {{
    {{'1', 'U', ' ', 'W', 'C', ' ', ' ', ' ', 'S', 'S', 'W', 'B', 'T', 'T', 'B', 'W', '7', '8', ' ', ' ', ' ', ' ', 'W',}},
    {{'2', ' ', ' ', 'W', 'F', ' ', ' ', ' ', 'U', 'U', 'W', 'B', 'T', 'T', 'B', 'W', '7', '8', ' ', ' ', ' ', 'B', 'W',}},
    {{' ', ' ', ' ', 'W', 'F', ' ', ' ', ' ', ' ', ' ', 'W', 'B', 'T', 'T', 'B', 'W', 'W', 'W', 'D', 'W', 'W', 'W', 'W',}},
    {{'W', 'W', 'D', 'W', 'L', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'A', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'W', 'W', 'D', 'W', 'W', ' ', ' ', 'D',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', 'B', 'B', ' ', ' ', 'J', 'W', ' ', ' ', ' ', 'B', 'W', 'W', 'W',}},
    {{'W', 'W', 'W', 'W', 'D', 'W', ' ', ' ', 'W', 'T', 'S', 'S', 'T', ' ', ' ', 'W', 'S', 'S', ' ', 'B', 'W', 'W', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', 'T', 'T', 'T', 'T', ' ', ' ', 'W', 'U', 'U', ' ', 'B', 'W', ' ', ' ',}},
    {{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'D', 'T', 'T', 'T', 'T', ' ', 'B', 'W', ' ', ' ', ' ', 'B', 'W', ' ', ' ',}},
    {{'L', ' ', 'A', 'L', 'W', 'W', ' ', ' ', 'W', 'J', 'U', 'U', ' ', ' ', 'B', 'W', 'W', 'D', 'W', 'W', 'W', ' ', ' ',}},
    {{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', ' ', ' ', 'W', 'C', 'C', 'W', 'W',}},
    {{'B', 'B', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', 'W', ' ', ' ', 'W', 'W',}},
    {{' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', 'D',}},
    {{' ', '6', ' ', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'D', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{'U', '5', ' ', 'W', ' ', ' ', 'W', 'C', 'F', 'F', ' ', ' ', 'W', ' ', ' ', 'W', 'W', 'D', 'W', 'W', ' ', ' ', 'W',}},
    {{'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', 'W', 'L', ' ', 'W', 'A', ' ', 'B', 'W', ' ', ' ', 'W',}},
    {{'B', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', ' ', ' ', 'B', 'W', 'J', ' ', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W', 'U', ' ', ' ', 'W', 'B', ' ', 'D',}},
    {{'J', ' ', ' ', 'C', 'B', 'B', 'W', 'L', 'F', ' ', 'W', 'F', ' ', 'W', 'L', 'W', '7', '8', ' ', 'W', 'B', ' ', 'W',}},
    {{'B', ' ', ' ', 'B', 'W', 'W', 'W', 'W', 'W', ' ', 'W', 'A', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'C', ' ', 'W',}},
    {{'B', ' ', ' ', 'B', 'W', ' ', ' ', ' ', 'D', ' ', 'W', 'C', ' ', ' ', 'W', 'W', 'B', 'B', 'B', 'B', 'W', 'D', 'W',}},
    {{'W', 'W', 'D', 'W', 'C', ' ', ' ', ' ', 'W', 'W', 'W', 'B', 'T', 'T', 'B', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
}};

// INTERIOR1_LAYER2
static constexpr std::array<std::array<char, 23>, 23> INTERIOR1_LAYER2 = {{
    {{' ', 'P', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', 'B', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'P', 'P', 'W', 'B', ' ', ' ', 'B', 'W', ' ', ' ', ' ', ' ', ' ', 'B', 'W',}},
    {{' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', 'B', 'W', 'W', 'W', 'D', 'W', 'W', 'W', 'W',}},
    {{'W', 'W', 'D', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'W', 'W', 'D', 'W', 'W', ' ', ' ', 'D',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', 'B', 'B', ' ', ' ', ' ', 'W', ' ', ' ', ' ', 'B', 'W', 'W', 'W',}},
    {{'W', 'W', 'W', 'W', 'D', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', 'B', 'W', 'W', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'P', 'P', ' ', 'B', 'W', ' ', ' ',}},
    {{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', 'B', 'W', ' ', ' ', ' ', 'B', 'W', ' ', ' ',}},
    {{' ', ' ', ' ', ' ', 'W', 'W', ' ', ' ', 'W', ' ', 'P', 'P', ' ', ' ', 'B', 'W', 'W', 'D', 'W', 'W', 'W', ' ', ' ',}},
    {{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', ' ', ' ', 'W', 'C', 'C', 'W', 'W',}},
    {{'B', 'B', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', 'W', ' ', ' ', 'W', 'W',}},
    {{' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', 'D',}},
    {{' ', ' ', ' ', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'D', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{'P', ' ', ' ', 'W', ' ', ' ', 'W', 'N', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', 'W', 'D', 'W', 'W', ' ', ' ', 'W',}},
    {{'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', ' ', ' ', 'B', 'W', ' ', ' ', 'W',}},
    {{'B', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', ' ', ' ', 'C', 'W', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W', 'P', ' ', ' ', 'W', 'B', ' ', 'D',}},
    {{' ', ' ', ' ', ' ', 'B', 'B', 'W', ' ', ' ', ' ', 'W', ' ', ' ', 'W', 'P', 'W', ' ', ' ', ' ', 'W', 'B', ' ', 'W',}},
    {{'B', ' ', ' ', 'B', 'W', 'W', 'W', 'W', 'W', ' ', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W',}},
    {{'B', ' ', ' ', 'B', 'W', ' ', ' ', ' ', 'D', ' ', 'W', 'N', ' ', ' ', 'W', 'W', 'B', 'B', 'B', 'B', 'W', 'D', 'W',}},
    {{'W', 'W', 'D', 'W', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'B', ' ', ' ', 'B', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
}};

// INTERIOR2_LAYER1
static constexpr std::array<std::array<char, 23>, 23> INTERIOR2_LAYER1 = {{
    {{'W', 'W', 'W', 'D', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'W',}},
    {{'U', ' ', ' ', ' ', ' ', ' ', 'C', 'W', 'L', ' ', ' ', 'L', 'W', 'A', 'A', 'W', ' ', ' ', ' ', ' ', ' ', 'L', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', 'S', 'S', 'S', ' ', 'W',}},
    {{' ', ' ', 'W', 'F', ' ', ' ', ' ', 'W', 'C', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'J', ' ', 'U', 'U', 'U', ' ', 'D',}},
    {{'U', ' ', 'W', 'F', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W',}},
    {{'U', ' ', 'W', 'F', ' ', ' ', ' ', 'D', ' ', ' ', 'T', 'T', 'W', ' ', ' ', ' ', ' ', ' ', 'U', 'W', ' ', 'L', 'W',}},
    {{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', 'T', 'J', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'W', ' ', ' ', 'W', 'L', ' ', 'W',}},
    {{'J', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'C', ' ', ' ', ' ', 'B', 'W', ' ', ' ', 'W', ' ', ' ', 'W',}},
    {{'W', 'W', 'W', 'W', 'W', 'L', ' ', ' ', ' ', ' ', 'W', 'C', ' ', ' ', ' ', 'B', 'W', ' ', ' ', 'W', 'W', 'D', 'W',}},
    {{' ', 'A', 'B', 'B', 'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'B', 'W', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', 'B', 'W', 'L', ' ', ' ', ' ', ' ', 'W', 'L', ' ', ' ', 'B', 'W', 'W', 'B', 'B', 'W', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', 'B', 'W', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'D',}},
    {{' ', ' ', ' ', ' ', 'D', ' ', ' ', 'U', ' ', ' ', ' ', 'D', ' ', ' ', 'F', 'F', 'W', 'A', 'A', 'W', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', ' ', 'W', ' ', ' ', 'U', ' ', ' ', 'W', 'W', ' ', ' ', ' ', ' ', 'C', ' ', ' ', 'W', ' ', ' ', 'W',}},
    {{'C', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', ' ', ' ', 'L', ' ', ' ', 'W', 'W', 'D', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{'L', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'L', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'U', 'U', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', ' ', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'U', 'U', ' ', 'W', 'B', ' ', 'U', 'U', 'B', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{'S', 'S', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'B', ' ', 'W',}},
    {{'U', 'U', ' ', ' ', ' ', 'L', 'B', 'B', 'B', ' ', ' ', 'W', 'B', 'B', 'B', 'B', 'B', 'B', 'B', ' ', 'B', 'D', 'W',}},
}};

// INTERIOR2_LAYER2
static constexpr std::array<std::array<char, 23>, 23> INTERIOR2_LAYER2 = {{
    {{'W', 'W', 'W', 'D', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'W',}},
    {{'P', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'E', ' ', ' ', 'E', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', 'E', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{' ', ' ', 'W', 'F', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'P', 'P', 'P', ' ', 'D',}},
    {{'P', ' ', 'W', 'F', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W',}},
    {{'P', ' ', 'W', 'F', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', 'P', 'W', ' ', 'P', 'W',}},
    {{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'P', ' ', ' ', ' ', 'B', 'W', ' ', ' ', 'W', ' ', ' ', 'W',}},
    {{'W', 'W', 'W', 'W', 'W', 'E', ' ', ' ', ' ', ' ', 'W', 'P', ' ', ' ', ' ', 'B', 'W', ' ', ' ', 'W', 'W', 'D', 'W',}},
    {{' ', ' ', 'B', 'B', 'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'B', 'W', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', 'B', 'W', 'E', ' ', ' ', ' ', ' ', 'W', 'E', ' ', ' ', 'B', 'W', 'W', 'B', 'B', 'W', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', 'B', 'W', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'D',}},
    {{' ', ' ', ' ', ' ', 'D', ' ', ' ', 'P', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', ' ', 'W', ' ', ' ', 'P', ' ', ' ', 'W', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', ' ', ' ', 'E', ' ', ' ', 'W', 'W', 'D', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{'E', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'E', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'P', 'P', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', ' ', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'P', 'P', ' ', 'W', 'B', ' ', 'P', 'P', 'B', ' ', ' ', ' ', ' ', ' ', 'W',}},
    {{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'B', ' ', 'W',}},
    {{'P', 'P', ' ', ' ', ' ', 'E', 'B', 'B', 'B', ' ', ' ', 'W', 'B', 'B', 'B', 'B', 'B', 'B', 'B', ' ', 'B', ' ', 'D',}},
}};

inline std::optional<Block> get_interior_block(char c, bool is_layer2, Block wall_block) {
    switch (c) {
        case ' ': return {};
        case 'W': return wall_block;
        case 'U': return OAK_FENCE;
        case 'S': return OAK_STAIRS;
        case 'B': return BOOKSHELF;
        case 'C': return CRAFTING_TABLE;
        case 'F': return FURNACE;
        case '1': return RED_BED_NORTH_HEAD;
        case '2': return RED_BED_NORTH_FOOT;
        case '3': return RED_BED_EAST_HEAD;
        case '4': return RED_BED_EAST_FOOT;
        case '5': return RED_BED_SOUTH_HEAD;
        case '6': return RED_BED_SOUTH_FOOT;
        case '7': return RED_BED_WEST_HEAD;
        case '8': return RED_BED_WEST_FOOT;
        case 'L': return CAULDRON;
        case 'A': return ANVIL;
        case 'P': return OAK_PRESSURE_PLATE;
        case 'D': return is_layer2 ? std::optional<Block>(DARK_OAK_DOOR_UPPER) : std::optional<Block>(DARK_OAK_DOOR_LOWER);
        case 'J': return NOTE_BLOCK;
        case 'G': return GLOWSTONE;
        case 'N': return BREWING_STAND;
        case 'T': return WHITE_CARPET;
        case 'E': return OAK_LEAVES;
        default:  return {};
    }
}

void generate_building_interior(
    WorldEditor & editor,
    const std::vector<std::pair<int,int>> & floor_area,
    int min_x,
    int min_z,
    int max_x,
    int max_z,
    int start_y_offset,
    int building_height,
    Block wall_block,
    const std::vector<int> & floor_levels,
    const Args & args,
    const ProcessedWay & element,
    int abs_terrain_offset
) {
    int width = max_x - min_x + 1;
    int depth = max_z - min_z + 1;
    if (width < 8 || depth < 8) {
        return;
    }

    //std::unordered_set<std::pair<int,int>, std::hash<long long>> floor_area_set;
    //floor_area_set.reserve(floor_area.size() * 2);
    for (auto const & p : floor_area) {
        long long key = (static_cast<long long>(p.first) << 32) ^ static_cast<unsigned int>(p.second);
        // custom hashing via pair wrapper: store pair in set with lambda-style hash isn't trivial here;
        // instead use unordered_set of pair via concatenated key by wrapping in unordered_set of pair requires custom hasher;
        // We'll emulate by storing encoded pairs in a separate set if needed; for simplicity, use manual check below.
    }

    // Build a vector for fast membership checking (we'll use unordered_set of encoded keys)
    std::unordered_set<long long> encoded_floor;
    encoded_floor.reserve(floor_area.size()*2);
    for (auto const & p : floor_area) {
        long long key = (static_cast<long long>(p.first) << 32) | (static_cast<unsigned int>(p.second));
        encoded_floor.insert(key);
    }

    int buffer = 2;
    int interior_min_x = min_x + buffer;
    int interior_min_z = min_z + buffer;
    int interior_max_x = max_x - buffer;
    int interior_max_z = max_z - buffer;

    for (size_t floor_index = 0; floor_index < floor_levels.size(); ++floor_index) {
        int floor_y = floor_levels[floor_index];

        std::vector<std::pair<int,int>> wall_positions;
        std::vector<std::pair<int,int>> door_positions;

        int current_floor_ceiling;
        if (floor_index < floor_levels.size() - 1) {
            current_floor_ceiling = floor_levels[floor_index + 1] - 1;
        } else {
            auto it = element.tags.find(std::string("roof:shape"));
            bool has_roof_shape = (it != element.tags.end());
            if (args.roof && has_roof_shape && it->second != "flat") {
                current_floor_ceiling = start_y_offset + building_height;
            } else {
                current_floor_ceiling = start_y_offset + building_height + 1;
            }
        }

        const std::array<std::array<char,23>,23> * layer1;
        const std::array<std::array<char,23>,23> * layer2;
        if (floor_index == 0) {
            layer1 = &INTERIOR1_LAYER1;
            layer2 = &INTERIOR1_LAYER2;
        } else {
            layer1 = &INTERIOR2_LAYER1;
            layer2 = &INTERIOR2_LAYER2;
        }

        int pattern_height = static_cast<int>(layer1->size());
        int pattern_width = static_cast<int>((*layer1)[0].size());

        int y_offset = 1;

        for (int z = interior_min_z; z <= interior_max_z; ++z) {
            for (int x = interior_min_x; x <= interior_max_x; ++x) {
                long long key = (static_cast<long long>(x) << 32) | (static_cast<unsigned int>(z));
                if (encoded_floor.find(key) == encoded_floor.end()) {
                    continue;
                }

                int pattern_x = ( (x - interior_min_x + static_cast<int>(floor_index)) % pattern_width + pattern_width ) % pattern_width;
                int pattern_z = ( (z - interior_min_z + static_cast<int>(floor_index)) % pattern_height + pattern_height ) % pattern_height;

                char cell1 = (*layer1)[pattern_z][pattern_x];
                char cell2 = (*layer2)[pattern_z][pattern_x];

                auto opt_block1 = get_interior_block(cell1, false, wall_block);
                if (opt_block1.has_value()) {
                    editor.set_block_absolute(opt_block1.value(),
                                              x,
                                              floor_y + y_offset + abs_terrain_offset,
                                              z,
                                              std::nullopt,
                                              std::nullopt);
                    if (cell1 == 'W') {
                        wall_positions.emplace_back(x, z);
                    } else if (cell1 == 'D') {
                        door_positions.emplace_back(x, z);
                    }
                }

                auto opt_block2 = get_interior_block(cell2, true, wall_block);
                if (opt_block2.has_value()) {
                    editor.set_block_absolute(opt_block2.value(),
                                              x,
                                              floor_y + y_offset + abs_terrain_offset + 1,
                                              z,
                                              std::nullopt,
                                              std::nullopt);
                }
            }
        }

        for (auto const & p : wall_positions) {
            int x = p.first;
            int z = p.second;
            for (int y = floor_y + y_offset + 2; y <= current_floor_ceiling; ++y) {
                editor.set_block_absolute(wall_block, x, y + abs_terrain_offset, z, std::nullopt, std::nullopt);
            }
        }

        for (auto const & p : door_positions) {
            int x = p.first;
            int z = p.second;
            for (int y = floor_y + y_offset + 2; y <= current_floor_ceiling; ++y) {
                editor.set_block_absolute(wall_block, x, y + abs_terrain_offset, z, std::nullopt, std::nullopt);
            }
        }
    }
}

}