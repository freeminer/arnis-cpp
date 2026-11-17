#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace arnis::trains
{
class TrainRouteNetwork;
}

namespace arnis::element_processing::level_crossings
{
/// A timetable stretch as (state, first step, last step), inclusive at both ends.
using Window = std::tuple<std::uint8_t, std::size_t, std::size_t>;

struct ScheduleTrack
{
	std::size_t service{};
	double at{};
	double half{};
};

struct WalkStop
{
	std::uint32_t crossing{};
	std::pair<double, double> at{};
	std::pair<double, double> track{};
};

struct CrossingSchedule
{
	std::vector<ScheduleTrack> tracks;
	std::vector<WalkStop> walk_stops;
	std::vector<std::pair<int, int>> posts;
	std::vector<std::pair<double, double>> spots;
};

/// Distances around a road/rail crossing used to stop traffic clear of the tracks.
struct Break
{
	std::size_t crossing{};
	double below_stop{}, below_exit{}, above_stop{}, above_exit{};

	double stop(bool forward) const { return forward ? below_stop : above_stop; }
	double exit(bool forward) const { return forward ? above_exit : below_exit; }
};

using BreakKey = std::pair<std::uint64_t, std::uint64_t>; // way id, node id
using BreakMap = std::map<BreakKey, Break>;

/// Convert occupied timetable ticks into the warning/lowering/down/rising intervals
/// used by the Java crossing functions.
std::vector<Window> windows(const std::vector<bool> &busy);

/// Shared function that interpolates item-display booms between up and level.
std::string boom_function();

/// Generate the level-crossing datapack functions from the scheduled train services.
std::vector<std::pair<std::string, std::string>> pack_functions(
		const std::vector<CrossingSchedule> &crossings,
		const trains::TrainRouteNetwork &trains,
		const std::function<bool(std::size_t)> &runs, double keep_clear);
} // namespace arnis::element_processing::level_crossings
