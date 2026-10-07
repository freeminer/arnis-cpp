#include "block_definitions.h"
#include "bresenham.h"
#include "../osm_parser.h"
#include "world_editor.h"
#include "waterways.h"
#include "../strict_parse.h"

#include <vector>
#include <string>
#include <unordered_map>
#include <optional>
#include <utility>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <stdexcept>
#include <cstdint>

#include "../../../arnis_adapter.h"
namespace arnis
{

namespace waterways
{

int get_waterway_width(const std::string &waterway_type)
{
	if (waterway_type == "river")
		return 8;
	if (waterway_type == "canal")
		return 6;
	if (waterway_type == "stream")
		return 3;
	if (waterway_type == "fairway")
		return 12;
	if (waterway_type == "flowline")
		return 2;
	if (waterway_type == "brook")
		return 2;
	if (waterway_type == "ditch")
		return 2;
	if (waterway_type == "drain")
		return 1;
	return 4;
}

bool is_channel_waterway(const std::string &type)
{
	static const std::vector<std::string> excluded{"dam", "weir", "lock_gate",
			"waterfall", "rapids", "boatyard", "fuel", "dock", "riverbank", "water_point",
			"turning_point", "sluice_gate", "fish_pass", "security_lock", "milestone",
			"check_dam", "floating_barrier"};
	return std::find(excluded.begin(), excluded.end(), type) == excluded.end();
}

bool is_underground_waterway(const tags_t &tags)
{
	if (const auto it = tags.find("tunnel"); it != tags.end() && it->second != "no" &&
											 it->second != "0" && it->second != "false")
		return true;
	if (const auto it = tags.find("layer"); it != tags.end()) {
		const auto &value = it->second;
		const auto first = value.find_first_not_of(" \t\r\n\f\v");
		if (first == std::string::npos)
			return false;
		const auto last = value.find_last_not_of(" \t\r\n\f\v");
		if (const auto layer = strict_parse::i32(
					std::string_view(value).substr(first, last - first + 1)))
			return *layer < 0;
	}
	return false;
}

int waterway_width(const std::string &type, const tags_t &tags)
{
	if (const auto it = tags.find("width"); it != tags.end()) {
		// Rust trims the whole tag and splits only on a literal space. In
		// particular, reject unit-suffixed values and tab-separated tokens
		// instead of silently accepting a numeric prefix.
		const auto first_nonspace = it->second.find_first_not_of(" \t\r\n\f\v");
		if (first_nonspace == std::string::npos)
			return get_waterway_width(type);
		const auto last_nonspace = it->second.find_last_not_of(" \t\r\n\f\v");
		const auto trimmed =
				it->second.substr(first_nonspace, last_nonspace - first_nonspace + 1);
		const auto first_space = trimmed.find(' ');
		const auto token = trimmed.substr(0, first_space);
		std::size_t consumed = 0;
		double width = 0.0;
		try {
			width = std::stod(token, &consumed);
		} catch (...) {
			consumed = 0;
		}
		if (consumed == token.size() && std::isfinite(width))
			return width >= 1 ? std::min(MAX_WATERWAY_WIDTH, int(std::lround(width)))
							  : get_waterway_width(type);
	}
	return get_waterway_width(type);
}

void create_water_channel(
		WorldEditor &editor, int center_x, int center_z, int width, int flat_water_y)
{
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

				editor.set_block_absolute(
						WATER, x, water_y.value(), z, std::nullopt, std::nullopt);
				editor.set_block_absolute(AIR, x, water_y.value() + 1, z,
						std::optional<std::vector<Block>>(
								std::vector<Block>{GRASS, WHEAT, CARROTS, POTATOES}),
						std::nullopt);
			}
		}
	}
}

void generate_waterways(WorldEditor &editor, const ProcessedWay &element)
{
	auto it = element.tags.find("waterway");
	if (it == element.tags.end()) {
		return;
	}
	if (!is_channel_waterway(it->second) || is_underground_waterway(element.tags))
		return;

	int channel_width = waterway_width(it->second, element.tags);

	auto layer_it = element.tags.find("layer");
	if (layer_it != element.tags.end()) {
		const std::string &layer_val = layer_it->second;
		if (layer_val == "-1" || layer_val == "-2" || layer_val == "-3") {
			return;
		}
	}

	for (std::size_t i = 0; i + 1 < element.nodes.size(); ++i) {
		auto prev_node = element.nodes[i].xz();
		auto current_node = element.nodes[i + 1].xz();
		const int y0 = editor.get_water_level(prev_node.x, prev_node.z);
		const int y1 = editor.get_water_level(current_node.x, current_node.z);

		std::vector<std::tuple<int, int, int>> bresenham_points = bresenham_line(
				prev_node.x, 0, prev_node.z, current_node.x, 0, current_node.z);

		// Rust waterways.rs: ramp between endpoints, rounding half steps away
		// from zero, then clamp to the local surface to avoid floating sheets.
		const auto last = static_cast<std::int64_t>(bresenham_points.size()) - 1;
		const auto dy = static_cast<std::int64_t>(y1) - y0;
		for (std::size_t point_index = 0; point_index < bresenham_points.size();
				++point_index) {
			const auto &pt = bresenham_points[point_index];
			int bx = std::get<0>(pt);
			int bz = std::get<2>(pt);
			const auto num = dy * static_cast<std::int64_t>(point_index);
			const int ramped =
					last > 0 ? static_cast<int>(
									   y0 + (2 * num + last * ((dy > 0) - (dy < 0))) /
													(2 * last))
							 : std::min(y0, y1);
			const int seg_water_y = std::min(ramped, editor.get_water_level(bx, bz));
			create_water_channel(editor, bx, bz, channel_width, seg_water_y);
		}
	}
}

}
}
