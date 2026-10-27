#include "fm_datapack_disasters.h"

#include "world_editor/floor_state.h"
#include "biome.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iterator>
#include <limits>
#include <set>
#include <sstream>
#include <string_view>
#include <cstdint>
#include <tuple>
#include <unordered_map>

namespace arnis::datapack::disasters
{
namespace
{
constexpr std::string_view DIR = "ov80/data/arnis/function/disaster";

struct KindText
{
	int id;
	const char *name_earth;
	const char *name_moon;
	const char *name_mars;
	const char *advice;
	const char *over_earth;
	const char *over_moon;
	const char *over_mars;
	const char *phrase_earth;
	const char *phrase_moon;
	const char *phrase_mars;
};

constexpr std::array<KindText, 11> KINDS{{
		{1, "Meteor Shower", "Meteor Shower", "Meteor Shower",
				"Keep out of the red rings", "The meteor shower is over.",
				"The meteor shower is over.", "The meteor shower is over.",
				"a meteor shower", "a meteor shower", "a meteor shower"},
		{2, "Thunderstorm", "Thunderstorm", "Thunderstorm", "Stay off the rooftops",
				"The storm has passed.", "The storm has passed.", "The storm has passed.",
				"a thunderstorm", "a thunderstorm", "a thunderstorm"},
		{3, "Hailstorm", "Hailstorm", "Hailstorm", "Get under a roof",
				"The hail has stopped.", "The hail has stopped.", "The hail has stopped.",
				"a hailstorm", "a hailstorm", "a hailstorm"},
		{4, "Sinkholes", "Sinkholes", "Sinkholes", "Move when the ground cracks",
				"The ground has settled.", "The ground has settled.",
				"The ground has settled.", "sinkholes", "sinkholes", "sinkholes"},
		{5, "Tornado", "Tornado", "Dust Devil", "Stay out of its path",
				"The tornado is gone.", "The tornado is gone.", "The dust devil is gone.",
				"a tornado", "a tornado", "a dust devil"},
		{6, "Earthquake", "Moonquake", "Marsquake", "Get out of the buildings",
				"The shaking has stopped.", "The shaking has stopped.",
				"The shaking has stopped.", "an earthquake", "a moonquake",
				"a marsquake"},
		{7, "Flood", "Flood", "Flood", "Get to high ground", "The water has gone down.",
				"The water has gone down.", "The water has gone down.", "a flood",
				"a flood", "a flood"},
		{8, "Wildfire", "Wildfire", "Wildfire", "Keep ahead of the fire front",
				"The fire front has passed.", "The fire front has passed.",
				"The fire front has passed.", "a wildfire", "a wildfire", "a wildfire"},
		{9, "Tsunami", "Tsunami", "Tsunami", "Get away from the coast",
				"The sea has pulled back.", "The sea has pulled back.",
				"The sea has pulled back.", "a tsunami", "a tsunami", "a tsunami"},
		{10, "Kaiju", "Kaiju", "Kaiju", "Run, it can't be stopped",
				"The kaiju went back underground.", "The kaiju went back underground.",
				"The kaiju went back underground.", "a kaiju", "a kaiju", "a kaiju"},
		{11, "Snowstorm", "Snowstorm", "Snowstorm", "Get inside and keep warm",
				"The snow has stopped.", "The snow has stopped.", "The snow has stopped.",
				"a snowstorm", "a snowstorm", "a snowstorm"},
}};

bool has_kind(const std::vector<int> &kinds, int id)
{
	return std::find(kinds.begin(), kinds.end(), id) != kinds.end();
}

const char *body_text(
		CelestialBody body, const char *earth, const char *moon, const char *mars)
{
	return body == CelestialBody::Moon	 ? moon
		   : body == CelestialBody::Mars ? mars
										 : earth;
}

std::string json_quote(const std::string &value)
{
	return nlohmann::json(value).dump();
}

std::string number_word(std::size_t n)
{
	static constexpr std::array<const char *, 13> words{{"zero", "one", "two", "three",
			"four", "five", "six", "seven", "eight", "nine", "ten", "eleven", "twelve"}};
	return n < words.size() ? words[n] : std::to_string(n);
}

std::string count_text(std::size_t n)
{
	return n == 1 ? "one disaster" : number_word(n) + " disasters";
}

std::string ring(double radius, int points)
{
	std::ostringstream out;
	for (int p = 0; p < points; ++p)
		out << "execute rotated ~" << p * 360 / points
			<< " 0 run particle minecraft:dust{color:[1.0,0.25,0.05],scale:1.6} ^"
			<< radius << " ^0.2 ^ 0 0 0 0 1 force\n";
	out << "particle minecraft:dust{color:[1.0,0.55,0.1],scale:1.2} ~ ~0.3 ~ "
		   "0.3 0.1 0.3 0 3 force\n";
	return out.str();
}

int ring_of(int dx, int dz)
{
	return static_cast<int>(std::lround(std::sqrt(double(dx * dx + dz * dz))));
}

std::string sink_ring(int r)
{
	std::ostringstream out;
	for (int dz = -r; dz <= r; ++dz) {
		std::vector<std::pair<int, int>> runs;
		for (int dx = -r; dx <= r; ++dx) {
			if (ring_of(dx, dz) != r)
				continue;
			if (!runs.empty() && runs.back().second == dx - 1)
				runs.back().second = dx;
			else
				runs.emplace_back(dx, dx);
		}
		for (const auto &[x0, x1] : runs) {
			out << "$clone ~" << x0 << " ~-2 ~" << dz << " ~" << x1 << " ~30 ~" << dz
				<< " ~" << x0 << " ~-$(dd) ~" << dz << " replace force\n";
			out << "$fill ~" << x0 << " ~$(cl) ~" << dz << " ~" << x1 << " ~30 ~" << dz
				<< " minecraft:air\n";
		}
	}
	return out.str();
}

std::string funnel(CelestialBody body)
{
	const bool mars = body == CelestialBody::Mars;
	const char *color = mars ? "0.72,0.42,0.25" : "0.45,0.45,0.48";
	const char *cloud = mars ? "0.62,0.36,0.22" : "0.32,0.32,0.36";
	const char *ground = mars ? "minecraft:red_sand" : "minecraft:dirt";
	std::ostringstream out;
	for (int level = 0; level < 15; ++level) {
		const double h = level * 3.0;
		const double radius = 1.0 + .1 * h + .0028 * h * h;
		const int arms = h < 24.0 ? 3 : 4;
		for (int arm = 0; arm < arms; ++arm) {
			const int angle = (level * 17 + arm * 360 / arms) % 360;
			out << "execute rotated ~" << angle
				<< " 0 run particle minecraft:dust{color:[" << color << "],scale:2.5} ^"
				<< std::fixed << std::setprecision(1) << radius << " ^" << h
				<< " ^ 0.15 0.2 0.15 0 1 force\n";
		}
		if (level % 2 == 1)
			out << "execute rotated ~" << (level * 17 + 60) % 360
				<< " 0 run particle minecraft:large_smoke ^" << std::fixed
				<< std::setprecision(1) << radius * .8 << " ^" << h
				<< " ^ 0.2 0.2 0.2 0.01 1 force\n";
	}
	for (int arm = 0; arm < 8; ++arm)
		out << "execute rotated ~" << arm * 45 << " 0 run particle minecraft:dust{color:["
			<< cloud << "],scale:4.0} ^12 ^45 ^ 1 0.6 1 0 2 force\n";
	out << "particle minecraft:block{block_state:\"" << ground
		<< "\"} ~ ~0.3 ~ 2.5 0.2 2.5 0.15 6 force\n"
		   "particle minecraft:cloud ~ ~0.5 ~ 2 0.3 2 0.05 3 force\n";
	return out.str();
}

std::string json_text(const nlohmann::json &value)
{
	return value.dump();
}

constexpr int CELL_SIZE = 64;
constexpr std::uint64_t MAX_SAMPLES = 16'000'000;
constexpr std::size_t FILL_BUDGET = 120'000;
constexpr int SHORE_REACH = 48;
constexpr int MAX_DROP = 2;
constexpr std::int64_t MAX_FILL_VOLUME = 32'768;
constexpr std::int64_t RIVER_AREA = 6'000;
constexpr std::int64_t SEA_AREA = 20'000;
constexpr const char *FLOOD_WATER = "minecraft:light[level=0,waterlogged=true]";
constexpr const char *FLOOD_DRAIN = "minecraft:light[level=0]";
constexpr std::uint8_t LAND = 0;
constexpr std::uint8_t WATER = 1;
constexpr std::uint8_t OUTSIDE = 2;
constexpr std::uint8_t UNREACHED = std::numeric_limits<std::uint8_t>::max();
constexpr int LO_SPREAD = 3;

struct Grid
{
	int x0 = 0, z0 = 0;
	int x_min = 0, z_min = 0, x_max = 0, z_max = 0;
	int step = 1;
	std::size_t width = 0, height = 0;
	std::vector<std::int16_t> floor;
	std::vector<std::uint8_t> kind;

	static Grid sample(const Surface &surface)
	{
		Grid grid;
		grid.x0 = surface.x_origin();
		grid.z0 = surface.z_origin();
		grid.step = surface.step();
		grid.width = surface.width();
		grid.height = surface.height();
		grid.x_min = surface.min_x();
		grid.z_min = surface.min_z();
		grid.x_max = surface.max_x();
		grid.z_max = surface.max_z();
		grid.floor.assign(grid.width * grid.height, 0);
		grid.kind.assign(grid.width * grid.height, OUTSIDE);
		for (std::size_t gz = 0; gz < grid.height; ++gz) {
			for (std::size_t gx = 0; gx < grid.width; ++gx) {
				const auto col = surface.column(
						grid.x0 + static_cast<int>(gx) * grid.step + grid.step / 2,
						grid.z0 + static_cast<int>(gz) * grid.step + grid.step / 2);
				const auto i = gz * grid.width + gx;
				grid.floor[i] = static_cast<std::int16_t>(
						std::clamp(col.y, int(std::numeric_limits<std::int16_t>::min()),
								int(std::numeric_limits<std::int16_t>::max())));
				grid.kind[i] = col.kind == ColumnKind::Land	   ? LAND
							   : col.kind == ColumnKind::Water ? WATER
															   : OUTSIDE;
			}
		}
		grid.drop_perched_water();
		return grid;
	}

	template <typename Fn>
	void for_each_neighbour(std::size_t i, Fn &&fn) const
	{
		const auto x = i % width, z = i / width;
		if (x > 0)
			fn(i - 1);
		if (x + 1 < width)
			fn(i + 1);
		if (z > 0)
			fn(i - width);
		if (z + 1 < height)
			fn(i + width);
	}

	void drop_perched_water()
	{
		const auto radius = static_cast<std::size_t>(std::max(3 / step, 1));
		std::vector<std::int16_t> land(
				floor.size(), std::numeric_limits<std::int16_t>::max());
		for (std::size_t i = 0; i < floor.size(); ++i)
			if (kind[i] == LAND)
				land[i] = floor[i];
		std::vector<std::int16_t> rows(
				width * height, std::numeric_limits<std::int16_t>::max());
		for (std::size_t z = 0; z < height; ++z) {
			for (std::size_t x = 0; x < width; ++x) {
				const auto begin = x > radius ? x - radius : 0;
				const auto end = std::min(width, x + radius + 1);
				for (auto i = begin; i < end; ++i)
					rows[z * width + x] =
							std::min(rows[z * width + x], land[z * width + i]);
			}
		}
		std::vector<bool> perched(kind.size(), false);
		for (std::size_t i = 0; i < kind.size(); ++i) {
			if (kind[i] != WATER)
				continue;
			const auto x = i % width, z = i / width;
			const auto begin = z > radius ? z - radius : 0;
			const auto end = std::min(height, z + radius + 1);
			std::int16_t low = std::numeric_limits<std::int16_t>::max();
			for (auto j = begin; j < end; ++j)
				low = std::min(low, rows[j * width + x]);
			perched[i] = low != std::numeric_limits<std::int16_t>::max() &&
						 int(floor[i]) > int(low) + 1;
		}
		for (std::size_t i = 0; i < perched.size(); ++i)
			if (perched[i])
				kind[i] = LAND;
	}

	std::vector<bool> sources(std::optional<int> sea_y) const
	{
		auto low = [&](std::size_t i) {
			return kind[i] == WATER && (!sea_y || (*sea_y - 3 <= int(floor[i]) &&
														  int(floor[i]) <= *sea_y + 1));
		};
		const auto min_area = sea_y ? SEA_AREA : RIVER_AREA;
		std::vector<bool> found(kind.size(), false), seen(kind.size(), false);
		std::deque<std::size_t> queue;
		std::vector<std::size_t> members;
		for (std::size_t start = 0; start < kind.size(); ++start) {
			if (seen[start] || !low(start))
				continue;
			seen[start] = true;
			queue.push_back(start);
			members.clear();
			bool edge = false;
			while (!queue.empty()) {
				const auto i = queue.front();
				queue.pop_front();
				members.push_back(i);
				const auto x = i % width, z = i / width;
				edge |= x == 0 || z == 0 || x + 1 == width || z + 1 == height;
				for_each_neighbour(i, [&](std::size_t n) {
					if (!seen[n] && low(n)) {
						seen[n] = true;
						queue.push_back(n);
					}
				});
			}
			const auto area = static_cast<std::int64_t>(members.size()) * step * step;
			if (edge || area >= min_area)
				for (const auto i : members)
					found[i] = true;
		}
		return found;
	}

	std::pair<std::vector<std::uint8_t>, std::vector<std::int16_t>> spread(
			const std::vector<bool> &source, std::uint8_t levels) const
	{
		std::vector<std::uint8_t> need(kind.size(), UNREACHED);
		std::vector<std::int16_t> src(kind.size(), 0);
		std::vector<std::vector<std::size_t>> buckets(std::size_t(levels) + 1);
		for (std::size_t i = 0; i < kind.size(); ++i)
			if (source[i]) {
				need[i] = 0;
				src[i] = floor[i];
				buckets[0].push_back(i);
			}
		for (std::uint16_t b = 0; b <= levels; ++b) {
			auto &bucket = buckets[b];
			while (!bucket.empty()) {
				const auto i = bucket.back();
				bucket.pop_back();
				if (need[i] != b)
					continue;
				for_each_neighbour(i, [&](std::size_t j) {
					const int ground = floor[j];
					if (kind[j] == OUTSIDE || ground < int(src[i]) - MAX_DROP)
						return;
					const int rise = ground - int(src[i]) + 1;
					const int next = std::max(rise, int(b));
					if (next <= levels && next < need[j]) {
						need[j] = static_cast<std::uint8_t>(next);
						src[j] = src[i];
						buckets[std::size_t(next)].push_back(j);
					}
				});
			}
		}
		return {std::move(need), std::move(src)};
	}

	std::vector<bool> wet(
			const std::vector<std::uint8_t> &need, std::uint8_t levels) const
	{
		std::vector<bool> wet_cells(kind.size(), false);
		std::vector<std::uint16_t> reach(
				kind.size(), std::numeric_limits<std::uint16_t>::max());
		std::deque<std::size_t> queue;
		for (std::size_t i = 0; i < kind.size(); ++i)
			if (kind[i] == LAND && need[i] <= levels) {
				wet_cells[i] = true;
				reach[i] = 0;
				queue.push_back(i);
			}
		const auto max_reach =
				static_cast<std::uint16_t>(std::max(SHORE_REACH / step, 1));
		while (!queue.empty()) {
			const auto i = queue.front();
			queue.pop_front();
			if (reach[i] >= max_reach)
				continue;
			for_each_neighbour(i, [&](std::size_t j) {
				if (kind[j] == WATER && need[j] <= levels &&
						reach[j] == std::numeric_limits<std::uint16_t>::max()) {
					reach[j] = reach[i] + 1;
					wet_cells[j] = true;
					queue.push_back(j);
				}
			});
		}
		return wet_cells;
	}
};

struct Fill
{
	int lo = 0;
	std::int16_t src = 0;
	std::uint8_t need = 0;
	bool held = false, water = false, dry = false;
};

struct Merged
{
	Fill first;
	int top = 0, lo_min = 0, lo_max = 0;
	std::uint8_t need = 0;

	Merged(Fill value, std::uint8_t levels) :
			first(value), top(int(value.src) + int(levels)), lo_min(value.lo),
			lo_max(value.lo), need(value.need)
	{
	}
	bool takes(const Fill &fill) const
	{
		int low = lo_min, high = lo_max;
		if (fill.dry)
			return fill.lo > top;
		if (fill.src != first.src || fill.water != first.water ||
				fill.held != first.held || (fill.held && fill.need != first.need))
			return false;
		low = std::min(low, fill.lo);
		high = std::max(high, fill.lo);
		return high - low <= LO_SPREAD;
	}
	bool takes(const std::vector<Fill> &fills) const
	{
		int low = lo_min, high = lo_max;
		for (const auto &fill : fills) {
			if (fill.dry) {
				if (fill.lo <= top)
					return false;
				continue;
			}
			if (fill.src != first.src || fill.water != first.water ||
					fill.held != first.held || (fill.held && fill.need != first.need))
				return false;
			low = std::min(low, fill.lo);
			high = std::max(high, fill.lo);
		}
		return high - low <= LO_SPREAD;
	}
	void add(const Fill &fill)
	{
		if (fill.dry)
			return;
		lo_min = std::min(lo_min, fill.lo);
		lo_max = std::max(lo_max, fill.lo);
		need = std::min(need, fill.need);
	}
	void add(const std::vector<Fill> &fills)
	{
		for (const auto &fill : fills)
			add(fill);
	}
};

Fill fill_of(std::uint8_t kind, std::int16_t floor, std::int16_t src, std::uint8_t need)
{
	const int lo = int(floor) + 1;
	if (kind == WATER)
		return {lo, src, need, false, true, false};
	const int own = std::max(lo - int(src), 1);
	need = std::max<std::uint8_t>(need, 1);
	return {lo, src, need, int(need) > own, false, false};
}

Fill dry_fill(std::int16_t floor)
{
	return {int(floor) + 1, 0, UNREACHED, false, false, true};
}

void split_rect(const WaterRect &rect, int top, std::vector<WaterRect> &out)
{
	const auto height = std::int64_t(std::max(top - rect.lo + 1, 1));
	const int dx = rect.x1 - rect.x0, dz = rect.z1 - rect.z0;
	const auto area = (std::int64_t(dx) + 1) * (std::int64_t(dz) + 1);
	if (area * height <= MAX_FILL_VOLUME || (dx == 0 && dz == 0)) {
		out.push_back(rect);
	} else if (dx >= dz) {
		const int mid = rect.x0 + dx / 2;
		auto left = rect, right = rect;
		left.x1 = mid;
		right.x0 = mid + 1;
		split_rect(left, top, out);
		split_rect(right, top, out);
	} else {
		const int mid = rect.z0 + dz / 2;
		auto top_rect = rect, bottom_rect = rect;
		top_rect.z1 = mid;
		bottom_rect.z0 = mid + 1;
		split_rect(top_rect, top, out);
		split_rect(bottom_rect, top, out);
	}
}

std::vector<WaterCell> water_cells(const Grid &grid,
		const std::vector<std::uint8_t> &need, const std::vector<std::int16_t> &src,
		std::uint8_t levels)
{
	const auto wet = grid.wet(need, levels);
	const auto per = static_cast<std::size_t>(CELL_SIZE / grid.step);
	auto cell_of = [&](int g, int origin) {
		return (origin + g * grid.step) >= 0
					   ? (origin + g * grid.step) / CELL_SIZE
					   : -((-origin - g * grid.step + CELL_SIZE - 1) / CELL_SIZE);
	};
	const int cx0 = cell_of(0, grid.x0), cz0 = cell_of(0, grid.z0);
	const int cx1 = cell_of(int(grid.width) - 1, grid.x0);
	const int cz1 = cell_of(int(grid.height) - 1, grid.z0);
	std::vector<WaterCell> out;
	std::vector<bool> used(per * per, false);
	for (int cz = cz0; cz <= cz1; ++cz) {
		for (int cx = cx0; cx <= cx1; ++cx) {
			const int gx0 = (cx * CELL_SIZE - grid.x0) / grid.step;
			const int gz0 = (cz * CELL_SIZE - grid.z0) / grid.step;
			auto at = [&](std::size_t lx, std::size_t lz) -> std::optional<Fill> {
				const int gx = gx0 + int(lx), gz = gz0 + int(lz);
				if (gx < 0 || gz < 0 || std::size_t(gx) >= grid.width ||
						std::size_t(gz) >= grid.height)
					return std::nullopt;
				const auto i = std::size_t(gz) * grid.width + std::size_t(gx);
				if (wet[i])
					return fill_of(grid.kind[i], grid.floor[i], src[i], need[i]);
				if (grid.kind[i] == LAND)
					return dry_fill(grid.floor[i]);
				return std::nullopt;
			};
			std::fill(used.begin(), used.end(), false);
			WaterCell cell{cx, cz, {}};
			for (std::size_t lz = 0; lz < per; ++lz) {
				for (std::size_t lx = 0; lx < per; ++lx) {
					if (used[lz * per + lx])
						continue;
					auto first = at(lx, lz);
					if (!first || first->dry)
						continue;
					Merged merged(*first, levels);
					std::size_t x_end = lx + 1;
					while (x_end < per && !used[lz * per + x_end]) {
						auto next = at(x_end, lz);
						if (!next || !merged.takes(*next))
							break;
						merged.add(*next);
						++x_end;
					}
					std::size_t z_end = lz + 1;
					while (z_end < per) {
						std::vector<Fill> row;
						row.reserve(x_end - lx);
						bool complete = true;
						for (std::size_t x = lx; x < x_end; ++x) {
							if (used[z_end * per + x]) {
								complete = false;
								break;
							}
							auto value = at(x, z_end);
							if (!value) {
								complete = false;
								break;
							}
							row.push_back(*value);
						}
						if (!complete || !merged.takes(row))
							break;
						merged.add(row);
						++z_end;
					}
					for (auto z = lz; z < z_end; ++z)
						for (auto x = lx; x < x_end; ++x)
							used[z * per + x] = true;
					auto bx = [&](std::size_t local) {
						return grid.x0 + (gx0 + int(local)) * grid.step;
					};
					auto bz = [&](std::size_t local) {
						return grid.z0 + (gz0 + int(local)) * grid.step;
					};
					WaterRect rect{std::max(bx(lx), grid.x_min),
							std::max(bz(lz), grid.z_min),
							std::min(bx(x_end) - 1, grid.x_max),
							std::min(bz(z_end) - 1, grid.z_max), merged.lo_min,
							int(first->src), merged.need};
					split_rect(rect, rect.src + levels, cell.rects);
				}
			}
			if (!cell.rects.empty())
				out.push_back(std::move(cell));
		}
	}
	return out;
}

std::optional<WaterMap> make_water_map(
		const Grid &grid, std::uint8_t requested_levels, std::optional<int> sea_y)
{
	const auto source = grid.sources(sea_y);
	if (std::none_of(source.begin(), source.end(), [](bool b) { return b; }))
		return std::nullopt;
	auto [need, src] = grid.spread(source, requested_levels);
	for (auto levels = requested_levels;; --levels) {
		auto cells = water_cells(grid, need, src, levels);
		std::size_t fills = 0;
		for (const auto &cell : cells)
			fills += cell.rects.size();
		if (fills == 0)
			return std::nullopt;
		if (fills <= FILL_BUDGET)
			return WaterMap{levels, std::move(cells)};
		if (levels <= 2)
			return std::nullopt;
	}
}

std::string water_key(char prefix, int surface)
{
	return std::string(1, prefix) +
		   (surface < 0 ? "m" + std::to_string(-surface) : std::to_string(surface));
}

constexpr std::array<const char *, 8> SNOW_BIOMES{
		{"minecraft:snowy_plains", "minecraft:snowy_taiga", "minecraft:snowy_slopes",
				"minecraft:snowy_beach", "minecraft:grove", "minecraft:ice_spikes",
				"minecraft:frozen_peaks", "minecraft:jagged_peaks"}};

std::vector<std::pair<std::string, std::string>> snow_swaps(
		const std::vector<std::pair<std::string, std::uint64_t>> &seen)
{
	std::vector<std::pair<std::string, std::uint64_t>> rainy;
	for (const auto &[name, count] : seen) {
		if (name.find("ocean") != std::string::npos ||
				name.find("river") != std::string::npos ||
				biome::winter_biome(name) == name)
			continue;
		rainy.emplace_back(name, count);
	}
	std::sort(rainy.begin(), rainy.end(), [](const auto &a, const auto &b) {
		return a.second != b.second ? a.second > b.second : a.first < b.first;
	});
	std::unordered_map<std::string, bool> used;
	for (const auto &[name, count] : seen) {
		(void)count;
		used.emplace(name, true);
	}
	std::vector<std::string> free;
	for (const auto *snow : SNOW_BIOMES)
		if (!used.contains(snow))
			free.emplace_back(snow);
	std::vector<std::pair<std::string, std::string>> out;
	for (const auto &[name, count] : rainy) {
		(void)count;
		if (free.empty())
			break;
		const auto own = biome::winter_biome(name);
		const auto pick = std::find(free.begin(), free.end(), own);
		const auto at = pick == free.end() ? free.begin() : pick;
		out.emplace_back(name, *at);
		free.erase(at);
	}
	return out;
}

std::pair<std::string, std::string> snow_functions(int floor, int snow_top,
		const std::vector<std::pair<std::string, std::string>> &swaps)
{
	std::pair<std::string, std::string> out;
	if (swaps.empty()) {
		out.first = out.second = "# It snows here as it is.\n";
		return out;
	}
	for (int y0 = floor; y0 <= snow_top; y0 += 384) {
		const int y1 = std::min(y0 + 383, snow_top);
		for (const auto &[from, to] : swaps) {
			out.first += "$fillbiome $(x0) " + std::to_string(y0) + " $(z0) $(x1) " +
						 std::to_string(y1) + " $(z1) " + to + " replace " + from + "\n";
			out.second += "$fillbiome $(x0) " + std::to_string(y0) + " $(z0) $(x1) " +
						  std::to_string(y1) + " $(z1) " + from + " replace " + to + "\n";
		}
	}
	return out;
}
} // namespace

Surface::Surface(int min_x, int min_z, int max_x, int max_z) :
		min_x_(min_x), min_z_(min_z), max_x_(max_x), max_z_(max_z)
{
	const auto blocks =
			(std::int64_t(max_x) - min_x + 1) * (std::int64_t(max_z) - min_z + 1);
	while (step_ < 16 &&
			std::uint64_t(blocks) / std::uint64_t(step_ * step_) > MAX_SAMPLES)
		step_ *= 2;
	auto aligned = [&](int value) {
		return value >= 0 ? (value / step_) * step_
						  : -(((-value + step_ - 1) / step_) * step_);
	};
	x0_ = aligned(min_x_);
	z0_ = aligned(min_z_);
	width_ = static_cast<std::size_t>((max_x_ - x0_) / step_ + 1);
	height_ = static_cast<std::size_t>((max_z_ - z0_) / step_ + 1);
	cells_.resize(width_ * height_);
}

SurfaceClass Surface::classify(std::string_view block_name)
{
	constexpr std::array<std::string_view, 32> pass_parts{{"leaves", "sapling", "torch",
			"carpet", "rail", "sign", "banner", "button", "pressure_plate", "candle",
			"flower", "tulip", "orchid", "allium", "bluet", "daisy", "poppy", "dandelion",
			"lily", "rose", "peony", "lilac", "eyeblossom", "grass", "fern", "bush",
			"vine", "petals", "fence", "pane", "bars", "chain"}};
	const auto colon = block_name.find(':');
	auto name =
			colon == std::string_view::npos ? block_name : block_name.substr(colon + 1);
	if (name == "water" || name == "water_source" || name == "water_flowing" ||
			name == "bubble_column" || name == "kelp" || name == "kelp_plant" ||
			name == "seagrass" || name == "tall_seagrass")
		return SurfaceClass::Fluid;
	const bool solid_kin = name.ends_with("_block") || name.ends_with("_stem");
	const bool passable = std::any_of(pass_parts.begin(), pass_parts.end(),
								  [&](std::string_view part) {
									  return name.find(part) != std::string_view::npos;
								  }) ||
						  name.ends_with("_wall") || name == "air" ||
						  name == "cave_air" || name == "void_air" || name == "light" ||
						  name == "snow" || name == "ladder" || name == "lever" ||
						  name == "tripwire" || name == "cobweb" ||
						  name == "sugar_cane" || name == "leaf_litter";
	return passable && !solid_kin ? SurfaceClass::Pass : SurfaceClass::Solid;
}

std::optional<std::size_t> Surface::index(int x, int z) const
{
	const int dx = x - x0_, dz = z - z0_, alignment = step_ / 2;
	if (dx < 0 || dz < 0 || dx % step_ != alignment || dz % step_ != alignment)
		return std::nullopt;
	const auto gx = static_cast<std::size_t>(dx / step_);
	const auto gz = static_cast<std::size_t>(dz / step_);
	if (gx >= width_ || gz >= height_)
		return std::nullopt;
	return gz * width_ + gx;
}

bool Surface::is_sample(int x, int z) const
{
	return index(x, z).has_value();
}

void Surface::observe(int x, int y, int z, SurfaceClass block_class)
{
	if (block_class == SurfaceClass::Pass)
		return;
	const auto i = index(x, z);
	if (!i)
		return;
	auto &cell = cells_[*i];
	if (cell.kind != ColumnKind::Outside && y < cell.y)
		return;
	cell.y = static_cast<std::int16_t>(
			std::clamp(y, int(std::numeric_limits<std::int16_t>::min()),
					int(std::numeric_limits<std::int16_t>::max())));
	cell.kind = block_class == SurfaceClass::Fluid ? ColumnKind::Water : ColumnKind::Land;
}

Column Surface::column(int x, int z) const
{
	const auto i = index(x, z);
	if (!i || cells_[*i].kind == ColumnKind::Outside)
		return {};
	return {cells_[*i].kind, cells_[*i].y};
}

std::optional<int> Surface::highest() const
{
	std::optional<int> result;
	for (const auto &cell : cells_)
		if (cell.kind != ColumnKind::Outside)
			result = std::max(result.value_or(cell.y), int(cell.y));
	return result;
}

std::size_t WaterMap::fill_count() const
{
	std::size_t count = 0;
	for (const auto &cell : cells)
		count += cell.rects.size();
	return count;
}

std::string WaterMap::tops(const std::string &name) const
{
	std::set<int> surfaces;
	for (const auto &cell : cells)
		for (const auto &rect : cell.rects)
			surfaces.insert(rect.src);
	std::ostringstream out;
	for (const int surface : surfaces) {
		out << "scoreboard players set #top arnis.ds " << surface << '\n'
			<< "scoreboard players operation #top arnis.ds += #lv_" << name
			<< " arnis.ds\n"
			<< "execute store result storage arnis:ds " << name << ".t."
			<< water_key('t', surface)
			<< " int 1 run scoreboard players get #top arnis.ds\n"
			<< "execute store result storage arnis:ds " << name << ".t."
			<< water_key('a', surface)
			<< " int 1 run scoreboard players add #top arnis.ds 1\n";
	}
	return out.str();
}

std::vector<std::pair<std::string, std::string>> WaterMap::cell_functions(
		const std::string &name) const
{
	std::vector<std::pair<std::string, std::string>> out;
	out.reserve(cells.size() * 2);
	for (const auto &cell : cells) {
		std::ostringstream fill, drain;
		for (const auto &rect : cell.rects) {
			const auto reach = "fill " + std::to_string(rect.x0) + " " +
							   std::to_string(rect.lo) + " " + std::to_string(rect.z0) +
							   " " + std::to_string(rect.x1) + " $(" +
							   water_key('t', rect.src) + ") " + std::to_string(rect.z1) +
							   " " + FLOOD_WATER + " replace #arnis:floodable strict";
			if (rect.need <= 1)
				fill << '$' << reach << '\n';
			else
				fill << "$execute if score #lv_" << name << " arnis.ds matches "
					 << unsigned(rect.need) << ".. run " << reach << '\n';
			const int top = rect.src + levels;
			const auto min_level = std::max<unsigned>(rect.need, 1);
			const auto above = "fill " + std::to_string(rect.x0) + " $(" +
							   water_key('a', rect.src) + ") " + std::to_string(rect.z0) +
							   " " + std::to_string(rect.x1) + " " + std::to_string(top) +
							   " " + std::to_string(rect.z1) + " minecraft:air replace " +
							   FLOOD_DRAIN;
			if (rect.lo >= rect.src + int(min_level)) {
				drain << '$' << above << '\n';
				continue;
			}
			drain << "$execute if score #lv_" << name << " arnis.ds matches " << min_level
				  << ".. run " << above << '\n'
				  << "execute unless score #lv_" << name << " arnis.ds matches "
				  << min_level << ".. run fill " << rect.x0 << ' ' << rect.lo << ' '
				  << rect.z0 << ' ' << rect.x1 << ' ' << top << ' ' << rect.z1
				  << " minecraft:air replace " << FLOOD_DRAIN << '\n';
		}
		out.emplace_back(std::string(DIR) + "/" + name + "/c/" + std::to_string(cell.x) +
								 "_" + std::to_string(cell.z) + ".mcfunction",
				fill.str());
		out.emplace_back(std::string(DIR) + "/" + name + "/d/" + std::to_string(cell.x) +
								 "_" + std::to_string(cell.z) + ".mcfunction",
				drain.str());
	}
	return out;
}

Plan::Plan(CelestialBody body, int floor, int world_max_y, bool wooded,
		const std::vector<std::pair<std::string, std::uint64_t>> &biomes,
		std::optional<int> highest_surface, const Surface *surface,
		std::optional<int> sea_y) :
		body_(body), floor_(floor),
		snow_top_(std::min(
				std::max(floor + 383, highest_surface ? *highest_surface + 32 : 0),
				world_max_y)),
		snow_(snow_swaps(biomes))
{
	if (body == CelestialBody::Earth && surface) {
		const auto grid = Grid::sample(*surface);
		flood_ = make_water_map(grid, 8, std::nullopt);
		if (sea_y)
			tsunami_ = make_water_map(grid, 7, sea_y);
	}
	if (body == CelestialBody::Earth) {
		kinds_ = {1, 2, 3, 4, 5, 6};
		if (flood_)
			kinds_.push_back(7);
		if (wooded)
			kinds_.push_back(8);
		if (tsunami_)
			kinds_.push_back(9);
		kinds_.insert(kinds_.end(), {10, 11});
	} else if (body == CelestialBody::Moon)
		kinds_ = {1, 4, 6};
	else
		kinds_ = {1, 4, 5, 6};
}

std::string Plan::coming() const
{
	std::vector<std::string> phrases;
	for (const int id : kinds_)
		phrases.emplace_back(body_text(body_, KINDS[id - 1].phrase_earth,
				KINDS[id - 1].phrase_moon, KINDS[id - 1].phrase_mars));
	if (phrases.empty())
		return {};
	if (phrases.size() == 1)
		return phrases.front();
	std::string result;
	for (std::size_t i = 0; i + 1 < phrases.size(); ++i) {
		if (i)
			result += ", ";
		result += phrases[i];
	}
	return result + " and " + phrases.back();
}

std::string Plan::summary() const
{
	auto describe = [](const std::optional<WaterMap> &map) {
		return map ? std::to_string(map->levels) + " levels, " +
							   std::to_string(map->fill_count()) + " fills"
				   : std::string("none");
	};
	return std::to_string(kinds_.size()) + " disasters; flood map " + describe(flood_) +
		   "; tsunami map " + describe(tsunami_);
}

std::vector<std::pair<std::string, std::string>> Plan::files(
		const std::string &area) const
{
	constexpr const char *root = "ov80/data/arnis/function/disaster/";
	std::vector<std::pair<std::string, std::string>> out;
	auto add = [&](std::string path, std::string content) {
		out.emplace_back(root + path, std::move(content));
	};
	std::ostringstream area_fn;
	area_fn << "data modify storage arnis:ds pool set value [";
	for (std::size_t i = 0; i < kinds_.size(); ++i)
		area_fn << (i ? "," : "") << kinds_[i];
	area_fn << "]\nscoreboard players set #floor arnis.ds " << floor_
			<< "\nscoreboard players set #has_snow arnis.ds "
			<< (has_kind(kinds_, 11) && !snow_.empty() ? 1 : 0)
			<< "\ndata modify storage arnis:ds wind_p set value \""
			<< (body_ == CelestialBody::Mars ? "minecraft:dust_plume" : "minecraft:cloud")
			<< "\"\n";
	// Body-specific labels: the table stores each variant separately.
	for (const char *field : {"names", "subs", "ends"}) {
		nlohmann::json values = nlohmann::json::array({""});
		for (const auto &kind : KINDS) {
			if (std::string_view(field) == "names")
				values.push_back(body_text(
						body_, kind.name_earth, kind.name_moon, kind.name_mars));
			else if (std::string_view(field) == "subs")
				values.push_back(kind.advice);
			else
				values.push_back(body_text(
						body_, kind.over_earth, kind.over_moon, kind.over_mars));
		}
		area_fn << "data modify storage arnis:ds " << field << " set value "
				<< values.dump() << '\n';
	}
	for (const auto &[name, map] :
			std::array<std::pair<const char *, const std::optional<WaterMap> *>, 2>{
					{{"flood", &flood_}, {"tsunami", &tsunami_}}}) {
		area_fn << "scoreboard players set #has_" << name << " arnis.ds "
				<< (map->has_value() ? 1 : 0) << '\n'
				<< "scoreboard players set #k_" << name << " arnis.ds "
				<< (map->has_value() ? unsigned(map->value().levels) : 0) << '\n'
				<< "data modify storage arnis:ds " << name << ".cells set value {}\n";
		if (!map->has_value())
			continue;
		const auto &cells = map->value().cells;
		for (std::size_t begin = 0; begin < cells.size(); begin += 400) {
			std::ostringstream entries;
			for (std::size_t i = begin; i < std::min(cells.size(), begin + 400); ++i) {
				if (i != begin)
					entries << ',';
				entries << '\"' << cells[i].x << '_' << cells[i].z << "\":1b";
			}
			area_fn << "data modify storage arnis:ds " << name << ".cells merge value {"
					<< entries.str() << "}\n";
		}
	}
	add("area.mcfunction", area_fn.str());

	const std::string n = number_word(kinds_.size());
	add("text/intro.mcfunction",
			"title @s subtitle " +
					json_text({{"text", "Survive " + count_text(kinds_.size()) + " in " +
												area},
							{"color", "gray"}}) +
					"\n");
	add("text/coming.mcfunction",
			"tellraw @s " +
					json_text({{"text", "Coming to " + area +
												", in no set order: " + coming() + "."},
							{"color", "gray"}}) +
					"\n");
	add("text/victory.mcfunction",
			"title @a subtitle " +
					json_text({{"text", area + " survived all " + n},
							{"color", "yellow"}}) +
					"\n");
	add("tornado/funnel.mcfunction", funnel(body_));
	add("meteor/ring.mcfunction", ring(3.0, 12));
	add("meteor/ring_big.mcfunction", ring(5.0, 18));
	for (int r = 0; r <= 15; ++r)
		add("sinkhole/ring/" + std::to_string(r) + ".mcfunction", sink_ring(r));
	const auto [swap_snow, thaw_snow] = snow_functions(floor_, snow_top_, snow_);
	add("snow/swap.mcfunction", swap_snow);
	add("snow/thaw.mcfunction", thaw_snow);
	for (const auto &[name, map] :
			std::array<std::pair<const char *, const std::optional<WaterMap> *>, 2>{
					{{"flood", &flood_}, {"tsunami", &tsunami_}}}) {
		add(std::string(name) + "/tops.mcfunction",
				map->has_value() ? map->value().tops(name)
								 : "# No water here to rise.\n");
		if (map->has_value()) {
			auto generated = map->value().cell_functions(name);
			out.insert(out.end(), std::make_move_iterator(generated.begin()),
					std::make_move_iterator(generated.end()));
		}
	}
	return out;
}

nlohmann::json Plan::dialog(const std::string &area, bool has_pause_menu) const
{
	auto text = [](std::string value, const char *color) {
		return nlohmann::json{{"text", std::move(value)}, {"color", color}};
	};
	auto run = [](const char *command) {
		return nlohmann::json{{"type", "run_command"}, {"command", command}};
	};
	nlohmann::json exit = has_pause_menu
								  ? nlohmann::json{{"label", "Back"},
											{"action", {{"type", "show_dialog"},
															   {"dialog", "arnis:menu"}}}}
								  : nlohmann::json{{"label", "Close"}};
	return {{"type", "minecraft:multi_action"}, {"title", text("Disasters", "red")},
			{"body",
					nlohmann::json::array(
							{{{"type", "minecraft:plain_message"},
									 {"contents",
											 nlohmann::json::array({text("A disaster hits ",
																			"gray"),
													 text(area, "white"),
													 text(" every few minutes, in random order, each with a short warning first.",
															 "gray")})},
									 {"width", 300}},
									{{"type", "minecraft:plain_message"},
											{"contents", text("Coming: " + coming() + ".",
																 "gray")},
											{"width", 300}}})},
			{"columns", 3},
			{"actions",
					nlohmann::json::array(
							{{{"label", "Restart"},
									 {"tooltip", "Start again from the first disaster"},
									 {"action", run("trigger arnis.ds.ctl set 1")}},
									{{"label", "Skip the Wait"},
											{"tooltip", "Bring on the next disaster now"},
											{"action",
													run("trigger arnis.ds.ctl set 2")}},
									{{"label", "Stop"}, {"tooltip", "End the disasters"},
											{"action",
													run("trigger arnis.ds.ctl set 3")}}})},
			{"exit_action", std::move(exit)}, {"pause", false}};
}
} // namespace arnis::datapack::disasters
