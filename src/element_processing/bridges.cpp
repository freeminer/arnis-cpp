#include <vector>
#include <tuple>
#include <cstddef>
#include <optional>
#include <string>
#include <algorithm>
#include <cmath>

#include "block_definitions.hpp"
#include "bresenham.hpp"
#include "../osm_parser.hpp"
#include "world_editor.hpp"

#include "../../../arnis_adapter.h"

namespace arnis
{

void generate_bridges(world_editor::WorldEditor& editor, osm_parser::ProcessedWay const& element) {
    auto it = element.tags.find(std::string("bridge"));
    if (it != element.tags.end()) {
        int bridge_height = 3; // Height above the ground level

        // Need at least 2 nodes for a bridge
        if (element.nodes.size() < 2) {
            return;
        }

        // Get start and end node elevations and use MAX for level bridge deck
        // Using MAX ensures bridges don't dip when multiple bridge ways meet in a valley
        int bridge_deck_ground_y = 0;
        if (!element.nodes.empty()) {
            const auto& start_node = element.nodes.front();
            const auto& end_node = element.nodes.back();
            // Get ground reference from editor
            auto* ground = editor.get_ground();
            if (ground) {
                int start_y = ground->level(XZPoint(start_node.x, start_node.z));
                int end_y = ground->level(XZPoint(end_node.x, end_node.z));
                bridge_deck_ground_y = std::max(start_y, end_y);
            }
        }

        // Calculate total bridge length for ramp positioning
        double total_length = 0.0;
        for (std::size_t i = 1; i < element.nodes.size(); ++i) {
            const auto& prev = element.nodes[i - 1];
            const auto& cur = element.nodes[i];
            double dx = static_cast<double>(cur.x - prev.x);
            double dz = static_cast<double>(cur.z - prev.z);
            total_length += std::sqrt(dx * dx + dz * dz);
        }

        if (total_length == 0.0) {
            return;
        }

        double accumulated_length = 0.0;

        for (std::size_t i = 1; i < element.nodes.size(); ++i) {
            const auto& prev = element.nodes[i - 1];
            const auto& cur = element.nodes[i];

            double segment_dx = static_cast<double>(cur.x - prev.x);
            double segment_dz = static_cast<double>(cur.z - prev.z);
            double segment_length = std::sqrt(segment_dx * segment_dx + segment_dz * segment_dz);

            std::vector<std::tuple<int, int, int>> points = bresenham::bresenham_line(prev.x, 0, prev.z, cur.x, 0, cur.z);

            // 15% of bridge, min 6, max 20 blocks
            int ramp_length = static_cast<int>(std::clamp(total_length * 0.15, 6.0, 20.0));

            for (std::size_t idx = 0; idx < points.size(); ++idx) {
                int x, y, z;
                std::tie(x, y, z) = points[idx];

                // Calculate progress along this segment
                double segment_progress = (points.size() > 1) ? 
                    static_cast<double>(idx) / static_cast<double>(points.size() - 1) : 0.0;

                // Calculate overall progress along the entire bridge
                double point_distance = accumulated_length + segment_progress * segment_length;
                double overall_progress = std::clamp(point_distance / total_length, 0.0, 1.0);
                int total_len_int = static_cast<int>(total_length);
                int overall_idx = static_cast<int>(overall_progress * static_cast<double>(total_len_int));

                // Calculate ramp height offset
                int ramp_offset;
                if (overall_idx < ramp_length) {
                    // Start ramp (rising)
                    ramp_offset = static_cast<int>(static_cast<double>(overall_idx) * static_cast<double>(bridge_height) / static_cast<double>(ramp_length));
                } else if (overall_idx >= total_len_int - ramp_length) {
                    // End ramp (descending)
                    int dist_from_end = total_len_int - overall_idx;
                    ramp_offset = static_cast<int>(static_cast<double>(dist_from_end) * static_cast<double>(bridge_height) / static_cast<double>(ramp_length));
                } else {
                    // Middle section (constant height)
                    ramp_offset = bridge_height;
                }

                // Use fixed bridge deck height (max of endpoints) plus ramp offset
                int bridge_y = bridge_deck_ground_y + ramp_offset;

                // Place bridge blocks
                for (int dx = -2; dx <= 2; ++dx) {
                    editor.set_block_absolute(block_definitions::LIGHT_GRAY_CONCRETE, x + dx, bridge_y, z, std::optional<std::vector<Block>>{}, std::optional<std::vector<Block>>{});
                }
            }

            accumulated_length += segment_length;
        }
    }
}

}
