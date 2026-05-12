#include "block_definitions.h"
#include "bresenham.h"
#include "../osm_parser.h"
#include "world_editor.h"

#include <vector>
#include <string>
#include <unordered_map>
#include <optional>
#include <utility>
#include <tuple>
#include <algorithm>
#include <stdexcept>

#include "../../../arnis_adapter.h"
namespace arnis
{

    namespace waterways{

int get_waterway_width(const std::string& waterway_type) {
    if (waterway_type == "river") return 8;
    if (waterway_type == "canal") return 6;
    if (waterway_type == "stream") return 3;
    if (waterway_type == "fairway") return 12;
    if (waterway_type == "flowline") return 2;
    if (waterway_type == "brook") return 2;
    if (waterway_type == "ditch") return 2;
    if (waterway_type == "drain") return 1;
    return 4;
}

void create_water_channel(
    WorldEditor& editor,
    int center_x,
    int center_z,
    int width,
    int flat_water_y
) {
    const int BANK_TOLERANCE = 2;
    int half_width = width / 2;
    for (int x = center_x - half_width - 1; x <= center_x + half_width + 1; ++x) {
        for (int z = center_z - half_width - 1; z <= center_z + half_width + 1; ++z) {
            int dx = std::abs(x - center_x);
            int dz = std::abs(z - center_z);
            int distance_from_center = std::max(dx, dz);

            if (distance_from_center <= half_width + 1) {
                const int ground_y = editor.get_ground_level(x, z);
                std::optional<int> water_y;
                if (ground_y <= flat_water_y) {
                    water_y = flat_water_y;
                } else if (ground_y <= flat_water_y + BANK_TOLERANCE &&
                        !editor.block_exists_absolute(x, ground_y, z)) {
                    water_y = ground_y;
                }
                if (!water_y.has_value())
                    continue;

                editor.set_block_absolute(WATER, x, water_y.value(), z, std::nullopt, std::nullopt);
                editor.set_block_absolute(
                        AIR,
                        x,
                        water_y.value() + 1,
                        z,
                        std::optional<std::vector<Block>>(std::vector<Block>{GRASS, WHEAT, CARROTS, POTATOES}),
                        std::nullopt);
            }
        }
    }
}

void generate_waterways(WorldEditor& editor, const ProcessedWay& element) {
    auto it = element.tags.find("waterway");
    if (it == element.tags.end()) {
        return;
    }

    int waterway_width = get_waterway_width(it->second);

    auto width_it = element.tags.find("width");
    if (width_it != element.tags.end()) {
        const std::string& width_str = width_it->second;
        try {
            waterway_width = std::stoi(width_str);
        } catch (const std::exception&) {
            try {
                float f = std::stof(width_str);
                waterway_width = static_cast<int>(f);
            } catch (const std::exception&) {
                // keep default width
            }
        }
    }

    auto layer_it = element.tags.find("layer");
    if (layer_it != element.tags.end()) {
        const std::string& layer_val = layer_it->second;
        if (layer_val == "-1" || layer_val == "-2" || layer_val == "-3") {
            return;
        }
    }

    for (std::size_t i = 0; i + 1 < element.nodes.size(); ++i) {
        auto prev_node = element.nodes[i].xz();
        auto current_node = element.nodes[i + 1].xz();
        int seg_water_y = std::min(
                editor.get_water_level(prev_node.x, prev_node.z),
                editor.get_water_level(current_node.x, current_node.z));

        std::vector<std::tuple<int,int,int>> bresenham_points = bresenham_line(
            prev_node.x, 0, prev_node.z,
            current_node.x, 0, current_node.z
        );

        for (const auto& pt : bresenham_points) {
            int bx = std::get<0>(pt);
            int bz = std::get<2>(pt);
            create_water_channel(editor, bx, bz, waterway_width, seg_water_y);
        }
    }
}

}
}
