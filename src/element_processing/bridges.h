#pragma once

#include <cstddef>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"
#include "bridge_styles.h"

namespace arnis::railways
{
extern const int RAIL_BRIDGE_FLAT_CLEARANCE;
extern const int RAIL_BRIDGE_DIP_THRESHOLD;
bool renders_as_rail_bridge(const ProcessedWay &way);
std::vector<std::pair<int, int>> build_smoothed_centerline(const ProcessedWay &way);
}

namespace arnis::bridges
{

enum class RailDeckKind
{
	Level,
	Carried,
};

struct RailDeckInfo
{
	RailDeckKind kind = RailDeckKind::Level;
	int level_y = 0;
	std::vector<int> ys;
};

struct BridgeMemberInfo
{
	int deck_y = 0;
	bridge_styles::BridgeStyle style = bridge_styles::BridgeStyle::Beam;
	std::optional<int> start_internal_ramp;
	std::optional<int> end_internal_ramp;
	std::optional<std::size_t> module_idx;
	int module_half_width = 0;
	bool covered_by_wider = false;
	std::vector<int> ys;
	std::optional<std::vector<std::pair<int, int>>> cable_pylons;

	int y_at(std::size_t tds, std::size_t total_bresenham, std::size_t ramp_length) const;
};

struct BridgeRampInfo
{
	bool bridge_side_at_start = false;
	int deck_y = 0;
	int ground_y = 0;

	int y_at(std::size_t tds, std::size_t total_bresenham) const;
};

class BridgeStructureMap
{
public:
	static BridgeStructureMap build(const std::vector<ProcessedElement> &elements,
			const WorldEditor &editor, double scale = 1.0);
	static BridgeStructureMap build(const std::vector<ProcessedElement> &elements,
			const WorldEditor &editor, const bridge_styles::BridgeOutlineIndex &outlines,
			double scale = 1.0);

	const BridgeMemberInfo *lookup_member(std::uint64_t way_id) const;
	const BridgeRampInfo *lookup_ramp(std::uint64_t way_id) const;
	const RailDeckInfo *rail_deck(std::uint64_t way_id) const;

private:
	std::unordered_map<std::uint64_t, BridgeMemberInfo> members_;
	std::unordered_map<std::uint64_t, BridgeRampInfo> ramps_;
	std::unordered_map<std::uint64_t, RailDeckInfo> rail_decks_;
};

class BridgeSurfaceMap
{
public:
	static BridgeSurfaceMap build(const std::vector<ProcessedElement> &elements,
			const BridgeStructureMap &structures, double scale);

	std::optional<int> deck_y_at(int x, int z) const;
	std::optional<int> nearby_deck_y(int x, int z, int radius) const;
	bool deck_near(int x, int z, int y, int tolerance) const;
	bool support_blocked(int x, int z, int deck_y) const;
	std::optional<int> supported_top(
			const WorldEditor &editor, int x, int z, int radius) const;
	bool contains(int x, int z) const { return deck_y_at(x, z).has_value(); }
	// Rust water-carving contract: a deck clears the water only when its lowest
	// structural block (including girders) is above the water surface.
	bool deck_clears(int x, int z, int water_y) const;
	bool over_grade_way(int x, int z) const;

private:
	struct PairHash
	{
		std::size_t operator()(const std::pair<int, int> &p) const noexcept
		{
			return std::hash<long long>()((static_cast<long long>(p.first) << 32) ^
										  static_cast<unsigned long long>(p.second));
		}
	};
	std::unordered_map<std::pair<int, int>, int, PairHash> deck_y_;
	std::unordered_map<std::pair<int, int>, int, PairHash> deck_low_y_;
	std::unordered_map<std::pair<int, int>, int, PairHash> rail_deck_y_;
	std::unordered_set<std::pair<int, int>, PairHash> grade_crossings_;
};

bool is_bridge_way(const ProcessedWay &way);

}
