#include "level_crossings.h"

#include "trains.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <tuple>
#include <utility>

namespace arnis::element_processing::level_crossings
{
namespace
{
constexpr std::int64_t WARN = 30;
constexpr std::int64_t LOWER = 40;
constexpr std::int64_t HOLD = 50;
constexpr std::int64_t CLEAR = 10;
constexpr std::int64_t RAISE = 30;
constexpr std::int64_t MIN_OPEN = 80;
constexpr std::uint8_t RISING = 1;
constexpr std::uint8_t WARNING = 2;
constexpr std::uint8_t LOWERING = 3;
constexpr std::uint8_t DOWN = 4;
constexpr double BOOM_UP_DEG = 80.0;

std::int64_t euclidean_remainder(std::int64_t value, std::int64_t modulus)
{
	const auto remainder = value % modulus;
	return remainder < 0 ? remainder + modulus : remainder;
}

std::string switch_function(
		std::size_t crossing_index, const CrossingSchedule &crossing, double keep_clear)
{
	std::ostringstream out;
	out << "scoreboard players operation #x" << crossing_index
		<< " arnis.xing = #n arnis.xing\n";
	if (!crossing.posts.empty()) {
		int min_x = crossing.posts.front().first;
		int max_x = min_x;
		int min_z = crossing.posts.front().second;
		int max_z = min_z;
		for (const auto &[x, z] : crossing.posts) {
			min_x = std::min(min_x, x);
			max_x = std::max(max_x, x);
			min_z = std::min(min_z, z);
			max_z = std::max(max_z, z);
		}
		out << "execute as @e[type=item_display,tag=arnis.x" << crossing_index
			<< ",x=" << min_x - 2 << ",y=-512,z=" << min_z - 2
			<< ",dx=" << max_x - min_x + 4 << ",dy=2048,dz=" << max_z - min_z + 4
			<< "] run function arnis:xing/apply\n";
	}
	const double side = std::round(2.0 * keep_clear - 1.0);
	for (const auto &[spot_x, spot_z] : crossing.spots) {
		std::ostringstream coords;
		coords << std::fixed << std::setprecision(1) << spot_x - keep_clear;
		const auto x = coords.str();
		coords.str({});
		coords.clear();
		coords << std::fixed << std::setprecision(1) << spot_z - keep_clear;
		out << "execute if score #n arnis.xing matches "
			<< static_cast<unsigned>(LOWERING)
			<< " as @e[type=marker,tag=arnis.car,tag=arnis.car.still,x=" << x
			<< ",y=-512,z=" << coords.str() << ",dx=" << std::fixed
			<< std::setprecision(1) << side << ",dy=2048,dz=" << side
			<< "] at @s run function arnis:cars/despawn\n";
	}
	out << "function arnis:xing/changed\n";
	return out.str();
}
} // namespace

std::vector<Window> windows(const std::vector<bool> &busy)
{
	if (busy.empty())
		return {};
	const auto n = static_cast<std::int64_t>(busy.size());
	const auto free = std::find(busy.begin(), busy.end(), false);
	if (free == busy.end())
		return {{DOWN, 0, busy.size() - 1}};

	// Start at a free tick so a run crossing the timetable boundary stays whole.
	const auto free_index = static_cast<std::int64_t>(free - busy.begin());
	std::vector<std::pair<std::int64_t, std::int64_t>> runs;
	for (std::int64_t k = 1; k <= n; ++k) {
		const auto i = free_index + k;
		if (!busy[static_cast<std::size_t>(i % n)])
			continue;
		if (!runs.empty() && runs.back().second == i - 1)
			runs.back().second = i;
		else
			runs.emplace_back(i, i);
	}
	if (runs.empty())
		return {};

	const auto lead = WARN + LOWER + HOLD;
	const auto opens = [](const auto &run) { return run.second + 1 + CLEAR + RAISE; };
	std::vector<std::pair<std::int64_t, std::int64_t>> closures;
	for (const auto &run : runs) {
		if (!closures.empty() && run.first - lead - opens(closures.back()) < MIN_OPEN)
			closures.back().second = run.second;
		else
			closures.push_back(run);
	}
	// Join the last and first closure when there is not enough open time over the wrap.
	while (closures.size() > 1) {
		const auto first = closures.front();
		const auto last = closures.back();
		if (first.first + n - lead - opens(last) >= MIN_OPEN)
			break;
		closures.pop_back();
		closures.erase(closures.begin());
		closures.emplace_back(last.first, first.second + n);
	}
	if (closures.size() == 1) {
		const auto &only = closures.front();
		if (only.first + n - lead - opens(only) < MIN_OPEN)
			return {{DOWN, 0, busy.size() - 1}};
	}

	std::vector<Window> result;
	const auto append_stretch = [&](std::uint8_t state, std::int64_t from,
										std::int64_t until) {
		const auto first = euclidean_remainder(from, n);
		const auto last = euclidean_remainder(until - 1, n);
		if (until - from >= n) {
			result.emplace_back(state, 0, busy.size() - 1);
		} else if (first <= last) {
			result.emplace_back(state, static_cast<std::size_t>(first),
					static_cast<std::size_t>(last));
		} else {
			result.emplace_back(state, static_cast<std::size_t>(first), busy.size() - 1);
			result.emplace_back(state, 0, static_cast<std::size_t>(last));
		}
	};
	for (const auto &[first, last] : closures) {
		append_stretch(WARNING, first - lead, first - LOWER - HOLD);
		append_stretch(LOWERING, first - LOWER - HOLD, first - HOLD);
		append_stretch(DOWN, first - HOLD, last + 1 + CLEAR);
		append_stretch(RISING, last + 1 + CLEAR, last + 1 + CLEAR + RAISE);
	}
	return result;
}

std::string boom_function()
{
	const auto turn = [](bool up) {
		if (!up)
			return std::string("[0.0000f,0.0000f,0.0000f,1.0000f]");
		const double half = BOOM_UP_DEG * 3.14159265358979323846 / 360.0;
		std::ostringstream value;
		value.setf(std::ios::fixed);
		value.precision(4);
		value << "[0.0000f,0.0000f," << -std::sin(half) << "f," << std::cos(half) << "f]";
		return value.str();
	};
	std::ostringstream out;
	out << "# A boom is down from when the booms start down until the train has gone.\n"
		   "scoreboard players set #want arnis.xing 0\n"
		   "execute if score #n arnis.xing matches "
		<< static_cast<unsigned>(LOWERING) << ".." << static_cast<unsigned>(DOWN)
		<< " run scoreboard players set #want arnis.xing 1\n"
		   "execute if score @s arnis.xb = #want arnis.xing run return 0\n"
		   "scoreboard players operation @s arnis.xb = #want arnis.xing\n"
		   "execute if score #want arnis.xing matches 1 run data merge entity @s "
		   "{start_interpolation:0,interpolation_duration:"
		<< LOWER * 2 << ",transformation:{left_rotation:" << turn(false)
		<< "}}\n"
		   "execute if score #want arnis.xing matches 0 run data merge entity @s "
		   "{start_interpolation:0,interpolation_duration:"
		<< RAISE * 2 << ",transformation:{left_rotation:" << turn(true) << "}}\n";
	return out.str();
}

std::vector<std::pair<std::string, std::string>> pack_functions(
		const std::vector<CrossingSchedule> &crossings,
		const trains::TrainRouteNetwork &trains,
		const std::function<bool(std::size_t)> &runs, double keep_clear)
{
	using TimedWindow = std::tuple<std::uint8_t, std::size_t, std::size_t, std::size_t>;
	std::vector<std::pair<std::string, std::string>> out;
	std::ostringstream body;
	std::map<std::size_t, std::size_t> clocks;
	for (std::size_t crossing_index = 0; crossing_index < crossings.size();
			++crossing_index) {
		const auto &crossing = crossings[crossing_index];
		std::set<TimedWindow> lines;
		for (const auto &track : crossing.tracks) {
			if (!runs(track.service))
				continue;
			const auto busy = trains.busy(track.service, track.at, track.half);
			if (busy.empty())
				continue;
			clocks[track.service] = busy.size();
			for (const auto &[state, from, to] : windows(busy))
				lines.emplace(state, track.service, from, to);
		}
		if (lines.empty())
			continue;
		body << "# Crossing " << crossing_index
			 << "\nscoreboard players set #n arnis.xing 0\n";
		for (const auto &[state, service, from, to] : lines)
			body << "execute if score #i" << service << " arnis.xing matches " << from
				 << ".." << to << " run scoreboard players set #n arnis.xing "
				 << static_cast<unsigned>(state) << "\n";
		body << "execute unless score #n arnis.xing = #x" << crossing_index
			 << " arnis.xing run function arnis:xing/c/" << crossing_index << "\n";
		out.emplace_back("data/arnis/function/xing/c/" + std::to_string(crossing_index) +
								 ".mcfunction",
				switch_function(crossing_index, crossing, keep_clear));
	}
	if (out.empty())
		return out;
	std::ostringstream check;
	check << "# Where each timetable is now, then what each crossing on it shows: 0 open, 1 its\n"
			 "# booms rising, 2 warning, 3 its booms coming down, 4 down.\n";
	for (const auto &[service, steps] : clocks)
		check << "scoreboard players operation #i" << service
			  << " arnis.xing = #now arnis.train\nscoreboard players set #k arnis.xing "
			  << steps << "\nscoreboard players operation #i" << service
			  << " arnis.xing %= #k arnis.xing\n";
	check << body.str();
	out.emplace_back("data/arnis/function/xing/check.mcfunction", check.str());
	out.emplace_back("data/arnis/function/xing/boom.mcfunction", boom_function());
	return out;
}
} // namespace arnis::element_processing::level_crossings
