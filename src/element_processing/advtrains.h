#pragma once

#include <cstddef>
#include <array>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

#include "../../../arnis_adapter.h"

namespace arnis::railways::advtrains
{

using XZ = std::pair<int, int>; // Arnis editor coordinates.
using DirectionSet = std::array<bool, 16>;
extern const std::array<XZ, 16> DIR_VECTORS;
std::optional<int> direction_between(const XZ &from, const XZ &to);
int closest_direction(int dx, int dz);
std::optional<Block> two_connection_rail(int first, int second);
std::optional<Block> junction_rail(const DirectionSet &directions);
std::optional<Block> rail_for_directions(const DirectionSet &directions);

struct RailPlan
{
	std::vector<XZ> line;
	std::vector<int> heights;
	std::vector<Block> rails;
};
const RailPlan *get_plan(const ProcessedWay &way);
void mark_generated(const ProcessedWay &way);
void finish_network(world_editor::WorldEditor &editor);
void fit_profile(const std::vector<XZ> &line, std::vector<int> &heights,
		const std::vector<bool> &flat);
std::vector<XZ> route(const XZ &from, const XZ &to, int departure, int arrival,
		const std::function<bool(const XZ &)> &blocked = {});
extern thread_local std::vector<v3pos_t> tunnel_floors;

bool available();
void prepare_network(
		const std::vector<ProcessedElement> &elements, world_editor::WorldEditor &editor);
std::vector<std::pair<int, int>> build_centerline(const ProcessedWay &way);
std::optional<Block> connected_rail(const std::vector<std::pair<int, int>> &line,
		std::size_t index, bool use_network = false);
std::vector<int> height_profile(
		world_editor::WorldEditor &editor, const std::vector<std::pair<int, int>> &line);
std::optional<Block> slope_rail(const std::vector<std::pair<int, int>> &line,
		const std::vector<int> &heights, std::size_t index);

} // namespace arnis::railways::advtrains
