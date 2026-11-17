#pragma once

#include <optional>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "bridges.h"

namespace arnis::bridge_decks
{

using bridges::BridgeMemberInfo;
using bridges::BridgeStructureMap;
using bridges::BridgeSurfaceMap;

// Material/edge profile selected once for all members of a bridge structure.
using DeckDesign = bridges::DeckDesign;

struct DeckParts
{
	std::vector<std::tuple<int, int, int>> fills;
	std::vector<std::tuple<int, int, int>> rails;
	std::vector<std::tuple<int, int, int>> steps;
	std::vector<std::tuple<int, int, int>> lamps;
	std::vector<std::tuple<int, int, int>> piers;
	std::vector<std::pair<int, int>> footed;
	std::vector<std::tuple<int, int, int>> caps;
	std::vector<std::tuple<int, int, int>> girders;
};

DeckDesign pick(
		bridge_styles::BridgeStyle style, const std::vector<const ProcessedWay *> &ways);
Block slab(DeckDesign design);
std::optional<Block> parapet(DeckDesign design);

// Add the Rust planner's material-specific deck underside and edge railings to
// the existing sampled bridge path. Structural supports remain owned by the
// bridge-style and bridge-module renderers.
void render(WorldEditor &editor, const BridgeSurfaceMap &surface, DeckDesign design,
		const std::vector<bridge_styles::BridgePathSample> &path, int block_range,
		bool lit);
std::unordered_map<std::uint64_t, DeckParts>
plan(const std::vector<ProcessedElement> &elements, const BridgeStructureMap &structures,
		const BridgeSurfaceMap &surface, const WorldEditor &editor, double scale);
void render(WorldEditor &editor, DeckDesign design, const DeckParts &parts);

} // namespace arnis::bridge_decks
