#include "power.h"
#include "../bresenham.h"
#include "../structures/structures.h"
#include <algorithm>
#include <cmath>

namespace arnis
{
namespace power
{

template <typename Tags>
static bool is_mounted_or_micro_turbine(const Tags &tags)
{
	auto positive = [&tags](const char *key) {
		auto it = tags.find(key);
		if (it == tags.end())
			return false;
		try {
			std::string value = it->second;
			if (!value.empty() && value.back() == 'm')
				value.pop_back();
			return std::stod(value) > 0.0;
		} catch (...) {
			return false;
		}
	};
	if (positive("min_height") || positive("level"))
		return true;
	if (auto it = tags.find("location");
			it != tags.end() && (it->second == "roof" || it->second == "rooftop"))
		return true;
	if (auto it = tags.find("generator:type");
			it != tags.end() && it->second == "vertical_axis")
		return true;
	if (auto it = tags.find("rotor:diameter"); it != tags.end()) {
		try {
			std::string value = it->second;
			if (!value.empty() && value.back() == 'm')
				value.pop_back();
			return std::stod(value) < 10.0;
		} catch (...) {
		}
	}
	return false;
}

void generate_power(WorldEditor &editor, const ProcessedElement &element,
		const CoordinateBitmap &building_footprints,
		const FloodFillCache &flood_fill_cache,
		const std::optional<std::chrono::milliseconds> &timeout)
{
	// Skip if 'layer' or 'level' is negative in the tags
	auto it_layer = element.tags().find("layer");
	if (it_layer != element.tags().end()) {
		try {
			if (std::stoi(it_layer->second) < 0) {
				return;
			}
		} catch (...) {
			// ignore parse errors
		}
	}

	auto it_level = element.tags().find("level");
	if (it_level != element.tags().end()) {
		try {
			if (std::stoi(it_level->second) < 0) {
				return;
			}
		} catch (...) {
			// ignore parse errors
		}
	}

	// Skip underground power infrastructure
	auto it_location = element.tags().find("location");
	if (it_location != element.tags().end() &&
			(it_location->second == "underground" ||
					it_location->second == "underwater")) {
		return;
	}

	auto it_tunnel = element.tags().find("tunnel");
	if (it_tunnel != element.tags().end() && it_tunnel->second == "yes") {
		return;
	}

	auto it_power = element.tags().find("power");
	if (it_power != element.tags().end()) {
		const std::string &power_type = it_power->second;
		if (power_type == "line" || power_type == "minor_line") {
			if (element.is_way()) {
				generate_power_line(editor, element.as_way());
			}
		} else if (power_type == "tower") {
			generate_power_tower(editor, element);
		} else if (power_type == "pole") {
			generate_power_pole(editor, element);
		} else if (power_type == "generator") {
			auto source = element.tags().find("generator:source");
			if (source != element.tags().end() && source->second == "wind" &&
					!is_mounted_or_micro_turbine(element.tags()) &&
					!element.nodes().empty()) {
				int64_t sx = 0, sz = 0;
				for (const auto &node : element.nodes()) {
					sx += node.x;
					sz += node.z;
				}
				structures::windturbine::place(editor,
						static_cast<int>(sx / element.nodes().size()),
						static_cast<int>(sz / element.nodes().size()));
			} else if (source != element.tags().end() && source->second == "solar" &&
					   element.is_way() &&
					   (element.tags().find("location") == element.tags().end() ||
							   element.tags().find("location")->second != "roof")) {
				const auto cells =
						flood_fill_cache.get_or_compute(element.as_way(), timeout);
				if (cells.size() < 60)
					return;
				int min_x = std::numeric_limits<int>::max(),
					max_x = std::numeric_limits<int>::min();
				int min_z = std::numeric_limits<int>::max(),
					max_z = std::numeric_limits<int>::min();
				for (const auto &[x, z] : cells) {
					min_x = std::min(min_x, static_cast<int>(x));
					max_x = std::max(max_x, static_cast<int>(x));
					min_z = std::min(min_z, static_cast<int>(z));
					max_z = std::max(max_z, static_cast<int>(z));
				}
				const bool rows_along_x = max_x - min_x >= max_z - min_z;
				for (const auto &[x0, z0] : cells) {
					const int x = x0, z = z0;
					if (building_footprints.contains(x, z) || editor.is_lc_water(x, z))
						continue;
					const int lane = rows_along_x ? (z - min_z + 4000000) % 4
												  : (x - min_x + 4000000) % 4;
					editor.set_block(lane < 3 ? DAYLIGHT_DETECTOR : GRAVEL, x,
							lane < 3 ? 1 : 0, z, std::nullopt, std::nullopt);
				}
			}
		}
	}
}

void generate_power(WorldEditor &editor, const ProcessedElement &element)
{
	const CoordinateBitmap empty_footprints = CoordinateBitmap::new_empty();
	const FloodFillCache empty_cache;
	generate_power(editor, element, empty_footprints, empty_cache, std::nullopt);
}

void generate_power_nodes(WorldEditor &editor, const ProcessedNode &node)
{
	// Skip if 'layer' or 'level' is negative in the tags
	auto it_layer = node.tags.find("layer");
	if (it_layer != node.tags.end()) {
		try {
			if (std::stoi(it_layer->second) < 0) {
				return;
			}
		} catch (...) {
			// ignore parse errors
		}
	}

	auto it_level = node.tags.find("level");
	if (it_level != node.tags.end()) {
		try {
			if (std::stoi(it_level->second) < 0) {
				return;
			}
		} catch (...) {
			// ignore parse errors
		}
	}

	// Skip underground power infrastructure
	auto it_location = node.tags.find("location");
	if (it_location != node.tags.end() && (it_location->second == "underground" ||
												  it_location->second == "underwater")) {
		return;
	}

	auto it_tunnel = node.tags.find("tunnel");
	if (it_tunnel != node.tags.end() && it_tunnel->second == "yes") {
		return;
	}

	auto it_power = node.tags.find("power");
	if (it_power != node.tags.end()) {
		const std::string &power_type = it_power->second;
		if (power_type == "tower") {
			generate_power_tower_from_node(editor, node);
		} else if (power_type == "pole") {
			generate_power_pole_from_node(editor, node);
		} else if (power_type == "generator") {
			auto source = node.tags.find("generator:source");
			if (source != node.tags.end() && source->second == "wind" &&
					!is_mounted_or_micro_turbine(node.tags))
				structures::windturbine::place(editor, node.x, node.z);
		}
	}
}

void generate_power_tower(WorldEditor &editor, const ProcessedElement &element)
{
	auto first_node_it = element.nodes().begin();
	if (first_node_it == element.nodes().end()) {
		return;
	}

	int height = 25;
	auto it_height = element.tags().find("height");
	if (it_height != element.tags().end()) {
		try {
			height = std::stoi(it_height->second);
		} catch (...) {
			// ignore parse errors
		}
	}
	height = std::clamp(height, 15, 40);

	generate_power_tower_impl(editor, first_node_it->x, first_node_it->z, height);
}

void generate_power_tower_from_node(WorldEditor &editor, const ProcessedNode &node)
{
	int height = 25;
	auto it_height = node.tags.find("height");
	if (it_height != node.tags.end()) {
		try {
			height = std::stoi(it_height->second);
		} catch (...) {
			// ignore parse errors
		}
	}
	height = std::clamp(height, 15, 40);

	generate_power_tower_impl(editor, node.x, node.z, height);
}

void generate_power_tower_impl(WorldEditor &editor, int x, int z, int height)
{
	// Tower design constants
	int base_width = 3;			 // Half-width at base (so 7x7 footprint)
	int top_width = 1;			 // Half-width at top (so 3x3)
	int arm_height = height - 4; // Height where arms extend
	int arm_length = 5;			 // How far arms extend horizontally

	// Build the four corner legs with tapering
	for (int y = 1; y <= height; ++y) {
		// Calculate taper: legs get closer together as we go up
		float progress = static_cast<float>(y) / static_cast<float>(height);
		int current_width =
				base_width - static_cast<int>((base_width - top_width) * progress);

		// Four corner positions
		std::vector<std::pair<int, int>> corners = {
				{x - current_width, z - current_width},
				{x + current_width, z - current_width},
				{x - current_width, z + current_width},
				{x + current_width, z + current_width},
		};

		for (const auto &corner : corners) {
			editor.set_block(IRON_BLOCK, corner.first, y, corner.second, std::nullopt,
					std::nullopt);
		}

		// Add horizontal cross-bracing every 5 blocks
		if (y % 5 == 0 && y < height - 2) {
			// Connect corners horizontally
			for (int dx = -current_width; dx <= current_width; ++dx) {
				editor.set_block(IRON_BLOCK, x + dx, y, z - current_width, std::nullopt,
						std::nullopt);
				editor.set_block(IRON_BLOCK, x + dx, y, z + current_width, std::nullopt,
						std::nullopt);
			}
			for (int dz = -current_width; dz <= current_width; ++dz) {
				editor.set_block(IRON_BLOCK, x - current_width, y, z + dz, std::nullopt,
						std::nullopt);
				editor.set_block(IRON_BLOCK, x + current_width, y, z + dz, std::nullopt,
						std::nullopt);
			}
		}

		// Add diagonal bracing between cross-brace levels
		if (y % 5 >= 1 && y % 5 <= 4 && y > 1 && y < height - 2) {
			int prev_width =
					base_width - static_cast<int>((base_width - top_width) *
												  ((y - 1) / static_cast<float>(height)));

			// Only add center vertical support if the width changed
			if (current_width != prev_width || y % 5 == 2) {
				editor.set_block(IRON_BARS, x, y, z, std::nullopt, std::nullopt);
			}
		}
	}

	// Create the cross-arms at arm_height for holding power lines
	// These extend outward in two directions (perpendicular to typical line direction)
	for (int arm_offset : {-arm_length, arm_length}) {
		// Main arm beam (iron blocks for strength)
		for (int dx = 0; dx <= arm_length; ++dx) {
			int arm_x = (arm_offset < 0) ? x - dx : x + dx;
			editor.set_block(
					IRON_BLOCK, arm_x, arm_height, z, std::nullopt, std::nullopt);
			// Add second arm perpendicular
			editor.set_block(IRON_BLOCK, x, arm_height, z + ((arm_offset < 0) ? -dx : dx),
					std::nullopt, std::nullopt);
		}

		// Insulators hanging from arm ends (end rods to simulate ceramic insulators)
		int end_x = (arm_offset < 0) ? x - arm_length : x + arm_length;
		editor.set_block(END_ROD, end_x, arm_height - 1, z, std::nullopt, std::nullopt);
		editor.set_block(
				END_ROD, x, arm_height - 1, z + arm_offset, std::nullopt, std::nullopt);
	}

	// Add a second, smaller arm set lower for additional circuits
	int lower_arm_height = arm_height - 6;
	if (lower_arm_height > 5) {
		int lower_arm_length = arm_length - 1;
		for (int arm_offset : {-lower_arm_length, lower_arm_length}) {
			for (int dx = 0; dx <= lower_arm_length; ++dx) {
				int arm_x = (arm_offset < 0) ? x - dx : x + dx;
				editor.set_block(IRON_BLOCK, arm_x, lower_arm_height, z, std::nullopt,
						std::nullopt);
			}
			int end_x = (arm_offset < 0) ? x - lower_arm_length : x + lower_arm_length;
			editor.set_block(
					END_ROD, end_x, lower_arm_height - 1, z, std::nullopt, std::nullopt);
		}
	}

	// Top finial/lightning rod
	editor.set_block(IRON_BLOCK, x, height, z, std::nullopt, std::nullopt);
	editor.set_block(LIGHTNING_ROD, x, height + 1, z, std::nullopt, std::nullopt);

	// Concrete foundation at base
	for (int dx = -3; dx <= 3; ++dx) {
		for (int dz = -3; dz <= 3; ++dz) {
			editor.set_block(
					GRAY_CONCRETE, x + dx, 0, z + dz, std::nullopt, std::nullopt);
		}
	}
}

void generate_power_pole(WorldEditor &editor, const ProcessedElement &element)
{
	auto first_node_it = element.nodes().begin();
	if (first_node_it == element.nodes().end()) {
		return;
	}

	int height = 10;
	auto it_height = element.tags().find("height");
	if (it_height != element.tags().end()) {
		try {
			height = std::stoi(it_height->second);
		} catch (...) {
			// ignore parse errors
		}
	}
	height = std::clamp(height, 6, 15);

	std::string pole_material = "wood";
	auto it_material = element.tags().find("material");
	if (it_material != element.tags().end()) {
		pole_material = it_material->second;
	}

	generate_power_pole_impl(
			editor, first_node_it->x, first_node_it->z, height, pole_material);
}

void generate_power_pole_from_node(WorldEditor &editor, const ProcessedNode &node)
{
	int height = 10;
	auto it_height = node.tags.find("height");
	if (it_height != node.tags.end()) {
		try {
			height = std::stoi(it_height->second);
		} catch (...) {
			// ignore parse errors
		}
	}
	height = std::clamp(height, 6, 15);

	std::string pole_material = "wood";
	auto it_material = node.tags.find("material");
	if (it_material != node.tags.end()) {
		pole_material = it_material->second;
	}

	generate_power_pole_impl(editor, node.x, node.z, height, pole_material);
}

void generate_power_pole_impl(
		WorldEditor &editor, int x, int z, int height, const std::string &pole_material)
{
	Block pole_block = OAK_LOG; // Default to wood
	if (pole_material == "concrete") {
		pole_block = LIGHT_GRAY_CONCRETE;
	} else if (pole_material == "steel" || pole_material == "metal") {
		pole_block = IRON_BLOCK;
	}

	// Build the main pole
	for (int y = 1; y <= height; ++y) {
		editor.set_block(pole_block, x, y, z, std::nullopt, std::nullopt);
	}

	// Cross-arm at top (perpendicular beam for wires)
	int arm_length = 2;
	for (int dx = -arm_length; dx <= arm_length; ++dx) {
		editor.set_block(OAK_FENCE, x + dx, height, z, std::nullopt, std::nullopt);
	}

	// Insulators at arm ends
	editor.set_block(END_ROD, x - arm_length, height + 1, z, std::nullopt, std::nullopt);
	editor.set_block(END_ROD, x + arm_length, height + 1, z, std::nullopt, std::nullopt);
	editor.set_block(
			END_ROD, x, height + 1, z, std::nullopt, std::nullopt); // Center insulator
}

void generate_power_line(WorldEditor &editor, const ProcessedWay &way)
{
	if (way.nodes.size() < 2) {
		return;
	}

	// Determine line height based on voltage (higher voltage = taller structures)
	int base_height = 15;
	auto it_voltage = way.tags.find("voltage");
	if (it_voltage != way.tags.end()) {
		try {
			int voltage = std::stoi(it_voltage->second);
			if (voltage >= 220000) {
				base_height = 22; // High voltage transmission
			} else if (voltage >= 110000) {
				base_height = 18;
			} else if (voltage >= 33000) {
				base_height = 14;
			} else {
				base_height = 10; // Distribution lines
			}
		} catch (...) {
			// ignore parse errors
		}
	}

	// Process consecutive node pairs
	for (size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &start = way.nodes[i - 1];
		const auto &end = way.nodes[i];

		// Calculate distance between nodes
		double dx = static_cast<double>(end.x - start.x);
		double dz = static_cast<double>(end.z - start.z);
		double distance = std::sqrt(dx * dx + dz * dz);

		// Calculate sag based on span length (longer spans = more sag)
		int max_sag = static_cast<int>(std::clamp(distance / 15.0, 1.0, 6.0));

		// Determine chain orientation based on line direction
		// If the line runs more along X-axis, use CHAIN_X; if more along Z-axis, use CHAIN_Z
		Block chain_block = (std::abs(dx) >= std::abs(dz)) ? CHAIN_X : CHAIN_Z;

		// Generate points along the line using Bresenham
		std::vector<std::tuple<int, int, int>> line_points =
				bresenham_line(start.x, 0, start.z, end.x, 0, end.z);

		for (size_t idx = 0; idx < line_points.size(); ++idx) {
			const auto &[lx, _, lz] = line_points[idx];

			// Calculate position along the span (0.0 to 1.0)
			// Use len-1 as denominator so last point reaches t=1.0
			double denom = (line_points.size() > 1)
								   ? static_cast<double>(line_points.size() - 1)
								   : 1.0;
			double t = static_cast<double>(idx) / denom;

			// Catenary approximation: sag is maximum at center, zero at ends
			// Using parabola: sag = 4 * max_sag * t * (1 - t)
			int sag = static_cast<int>(4.0 * max_sag * t * (1.0 - t));

			// Ensure wire doesn't go underground (minimum height of 3 blocks above ground)
			int wire_y = std::max(3, base_height - sag);

			// Place the wire block (chain aligned with line direction)
			editor.set_block(chain_block, lx, wire_y, lz, std::nullopt, std::nullopt);

			// For high voltage lines, add parallel wires offset to sides
			if (base_height >= 18) {
				// Three-phase power: 3 parallel lines
				// Offset perpendicular to the line direction
				if (std::abs(dx) >= std::abs(dz)) {
					// Line runs along X, offset in Z
					editor.set_block(
							chain_block, lx, wire_y, lz + 1, std::nullopt, std::nullopt);
					editor.set_block(
							chain_block, lx, wire_y, lz - 1, std::nullopt, std::nullopt);
				} else {
					// Line runs along Z, offset in X
					editor.set_block(
							chain_block, lx + 1, wire_y, lz, std::nullopt, std::nullopt);
					editor.set_block(
							chain_block, lx - 1, wire_y, lz, std::nullopt, std::nullopt);
				}
			}
		}
	}
}

}
}
