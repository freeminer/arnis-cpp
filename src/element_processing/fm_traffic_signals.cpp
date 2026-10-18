#include "fm_traffic_signals.h"
#include "highways.h"
#include "../block_definitions.h"

#include <array>

namespace arnis::highways
{
bool fm_place_native_traffic_signals(
		world_editor::WorldEditor &editor, int x, int head_base, int z)
{
	using namespace block_definitions;
	// Each Streets node is a complete three-lamp head. Prefer red, and allow
	// content packs that register only a subset of the available light states.
	const Block *head = nullptr;
	for (const auto &candidate : STREETS_TRAFFIC_LIGHTS) {
		if (candidate != AIR) {
			head = &candidate;
			break;
		}
	}
	if (!head)
		return false;

	for (int dy = 4; dy <= 5; ++dy)
		editor.set_block_absolute(
				STREETS_POLE, x, head_base + dy, z, std::nullopt, std::nullopt);

	// Streets heads face -Z at facedir 0; the bracket extends towards the
	// central pole. These rotations also match trafficlight/old2new.lua.
	constexpr std::array<std::array<int, 3>, 4> mounts{
			{{0, -1, 0}, {-1, 0, 1}, {0, 1, 2}, {1, 0, 3}}};
	for (const auto &mount : mounts) {
		auto node = *head;
		node.setParam2(static_cast<std::uint8_t>(mount[2]));
		editor.set_block_absolute(node, x + mount[0], head_base + 5, z + mount[1],
				std::nullopt, std::nullopt);
	}
	return true;
}

bool fm_place_native_traffic_signal_head(world_editor::WorldEditor &editor, int x, int y,
		int z, int facing_x, int facing_z)
{
	using namespace block_definitions;
	const Block *head = nullptr;
	for (const auto &candidate : STREETS_TRAFFIC_LIGHTS) {
		if (candidate != AIR) {
			head = &candidate;
			break;
		}
	}
	if (!head)
		return false;

	auto node = *head;
	std::uint8_t param2 = 0; // Facing -Z.
	int offset_x = 0;
	int offset_z = -1;
	if (facing_x < 0)
		param2 = 1, offset_x = -1, offset_z = 0;
	else if (facing_z > 0)
		param2 = 2, offset_x = 0, offset_z = 1;
	else if (facing_x > 0)
		param2 = 3, offset_x = 1, offset_z = 0;
	node.setParam2(param2);
	editor.set_block_absolute(
			node, x + offset_x, y, z + offset_z, std::nullopt, std::nullopt);
	return true;
}
}
