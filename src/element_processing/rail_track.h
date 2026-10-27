#pragma once

#include "../../../arnis_adapter.h"

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace arnis::railways
{

enum class TrackLevel
{
	Surface,
	Tunnel
};

struct TrackEnd
{
	std::vector<std::pair<int, int>> beyond;
	std::optional<std::tuple<std::uint64_t, std::size_t, int>> into;
	bool owns = true;
	bool shared = false;
	bool meets_surface = false;
};

std::optional<TrackLevel> track_level(const ProcessedWay &way);

std::vector<bool> straight_cells(const std::vector<std::pair<int, int>> &cells,
		std::optional<std::pair<int, int>> before = std::nullopt,
		std::optional<std::pair<int, int>> after = std::nullopt);
std::vector<int> rail_profile(const std::vector<bool> &straight,
		const std::function<std::pair<int, int>(std::size_t)> &range,
		const std::function<double(std::size_t, int)> &cost, double step_cost);
std::vector<int> fit_rail_deck(const std::vector<std::pair<int, int>> &cells,
		const std::vector<int> &terrain, std::vector<int> heights,
		std::array<bool, 2> internal);
std::vector<Block> rail_shapes(const std::vector<std::pair<int, int>> &cells,
		const std::vector<int> &heights,
		std::optional<std::tuple<std::pair<int, int>, int>> before = std::nullopt,
		std::optional<std::tuple<std::pair<int, int>, int>> after = std::nullopt);

// Precomputed once over the complete OSM rail network. Keeping it immutable
// makes adjoining ways and independently processed generation tiles agree.
class RailJoins
{
public:
	using LevelFn = std::function<std::optional<TrackLevel>(const ProcessedWay &)>;
	static RailJoins build(
			const std::vector<ProcessedElement> &elements, const LevelFn &level);
	std::array<TrackEnd, 2> ends(std::uint64_t way) const;
	void meet_at_edge_of(std::array<int, 4> bounds) { area_bounds_ = bounds; }
	std::vector<std::array<std::size_t, 2>> edge_crossings(
			const std::vector<std::pair<int, int>> &cells) const;

private:
	std::unordered_map<std::uint64_t, std::array<TrackEnd, 2>> ends_;
	// The One World area whose independently generated neighbors must share
	// track levels across its boundary.
	std::optional<std::array<int, 4>> area_bounds_;
};

} // namespace arnis::railways
