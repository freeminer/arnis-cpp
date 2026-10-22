#include "postprocess.h"
#include "elevation.h"
#include "../land_cover/land_cover.h"
#include "../water_depth.h"
#include "../world_editor/floor_state.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <thread>
#include <tuple>
#include <unordered_set>
#include <string_view>
namespace arnis::elevation
{
namespace
{
// Match Rust's saturating float-to-usize `as` conversion (NaN/negative -> 0,
// positive overflow -> usize::MAX) without invoking undefined C++ behavior.
std::size_t rust_float_to_size_t(double value)
{
	if (std::isnan(value) || value <= 0.0)
		return 0;
	if (!std::isfinite(value) ||
			value >= static_cast<double>(std::numeric_limits<std::size_t>::max()))
		return std::numeric_limits<std::size_t>::max();
	return static_cast<std::size_t>(value);
}

std::size_t elevation_worker_count()
{
	if (const char *value = std::getenv("RAYON_NUM_THREADS")) {
		const std::string_view text(value);
		std::size_t requested = 0;
		const auto [end, error] =
				std::from_chars(text.data(), text.data() + text.size(), requested);
		if (error == std::errc{} && end == text.data() + text.size() && requested > 0)
			return requested;
	}
	const auto hardware = std::max(1u, std::thread::hardware_concurrency());
	return std::max<std::size_t>(1, static_cast<std::size_t>(hardware * 0.9));
}

template <typename Function>
void parallel_rows(std::size_t begin, std::size_t end, Function &&function)
{
	if (begin >= end)
		return;
	const auto workers_count =
			std::min<std::size_t>(end - begin, elevation_worker_count());
	if (workers_count == 1) {
		for (auto row = begin; row < end; ++row)
			function(row);
		return;
	}
	std::atomic_size_t next{begin};
	auto worker = [&] {
		for (;;) {
			const auto row = next.fetch_add(1, std::memory_order_relaxed);
			if (row >= end)
				return;
			function(row);
		}
	};
	std::vector<std::thread> threads;
	threads.reserve(workers_count - 1);
	for (std::size_t i = 1; i < workers_count; ++i) {
		try {
			threads.emplace_back(worker);
		} catch (...) {
			break;
		}
	}
	worker();
	for (auto &thread : threads)
		thread.join();
}
} // namespace

void repair_terrain_anomalies(std::vector<std::vector<double>> &h, double meters_per_cell)
{
	if (h.size() < 5 || h[0].size() < 5)
		return;
	const std::size_t H = h.size(), W = h[0].size();
	const double abs_threshold = std::max(6.0, 0.25 * meters_per_cell);
	const int passes = meters_per_cell > 4.0 ? 2 : 10;
	constexpr std::size_t RADIUS = 2;
	const std::size_t words = W / 64 + (W % 64 != 0);
	std::vector<std::uint64_t> candidates;
	std::size_t total_repaired = 0;
	int passes_ran = 0;
	for (int pass = 0; pass < passes; ++pass) {
		std::vector<std::vector<std::tuple<std::size_t, std::size_t, double>>>
				row_repairs(H);
		auto judge = [&](std::size_t y, std::size_t x, std::vector<double> &neighbors,
							 std::vector<double> &abs_devs) -> std::optional<double> {
			// All workers inspect the same unchanged grid; repairs are committed only
			// after the parallel pass, so a full per-pass snapshot is unnecessary.
			const double c = h[y][x];
			if (!std::isfinite(c))
				return std::nullopt;
			neighbors.clear();
			double lo = std::numeric_limits<double>::infinity();
			double hi = -std::numeric_limits<double>::infinity();
			for (int dy = -2; dy <= 2; ++dy)
				for (int dx = -2; dx <= 2; ++dx) {
					if (dx == 0 && dy == 0)
						continue;
					const double value =
							h[static_cast<std::size_t>(static_cast<int>(y) + dy)]
							 [static_cast<std::size_t>(static_cast<int>(x) + dx)];
					if (!std::isfinite(value))
						continue;
					neighbors.push_back(value);
					lo = std::min(lo, value);
					hi = std::max(hi, value);
				}
			if (neighbors.size() < 8 ||
					(c - lo <= abs_threshold && hi - c <= abs_threshold))
				return std::nullopt;
			const auto mid = neighbors.begin() + neighbors.size() / 2;
			std::nth_element(neighbors.begin(), mid, neighbors.end());
			const double median = *mid;
			abs_devs.clear();
			for (const double value : neighbors)
				abs_devs.push_back(std::abs(value - median));
			const auto mad_mid = abs_devs.begin() + abs_devs.size() / 2;
			std::nth_element(abs_devs.begin(), mad_mid, abs_devs.end());
			const double mad = *mad_mid;
			if (std::abs(c - median) > abs_threshold &&
					std::abs(c - median) > 3.0 * std::max(1.0, mad))
				return median;
			return std::nullopt;
		};
		parallel_rows(
				pass == 0 ? RADIUS : 0, pass == 0 ? H - RADIUS : H, [&](std::size_t y) {
					thread_local std::vector<double> neighbors;
					thread_local std::vector<double> abs_devs;
					neighbors.reserve(24);
					abs_devs.reserve(24);
					auto &found = row_repairs[y];
					auto inspect = [&](std::size_t x) {
						if (x >= RADIUS && x < W - RADIUS)
							if (const auto median = judge(y, x, neighbors, abs_devs))
								found.emplace_back(y, x, *median);
					};
					if (pass == 0) {
						for (std::size_t x = RADIUS; x < W - RADIUS; ++x)
							inspect(x);
					} else {
						for (std::size_t word_index = 0; word_index < words;
								++word_index) {
							auto bits = candidates[y * words + word_index];
							while (bits) {
								const auto bit =
										static_cast<unsigned>(std::countr_zero(bits));
								bits &= bits - 1;
								inspect(word_index * 64 + bit);
							}
						}
					}
				});
		std::vector<std::tuple<std::size_t, std::size_t, double>> repairs;
		for (auto &row : row_repairs)
			repairs.insert(repairs.end(), row.begin(), row.end());
		if (repairs.empty())
			break;
		for (const auto &[y, x, median] : repairs)
			h[y][x] = median;
		total_repaired += repairs.size();
		passes_ran = pass + 1;
		if (passes_ran == passes)
			break;
		candidates.assign(words * H, 0);
		for (const auto &[y, x, median] : repairs) {
			(void)median;
			for (std::size_t ny = std::max(RADIUS, y > RADIUS ? y - RADIUS : 0);
					ny < std::min(H - RADIUS, y + RADIUS + 1); ++ny)
				for (std::size_t nx = std::max(RADIUS, x > RADIUS ? x - RADIUS : 0);
						nx < std::min(W - RADIUS, x + RADIUS + 1); ++nx)
					candidates[ny * words + (nx >> 6)] |= std::uint64_t(1) << (nx & 63);
		}
	}
	if (total_repaired > 0)
		std::clog << "Repaired " << total_repaired << " terrain anomalies in "
				  << passes_ran << " pass" << (passes_ran == 1 ? "" : "es") << '\n';
}

namespace
{
using HeightGrid = std::vector<std::vector<double>>;
using CoverGrid = std::vector<std::vector<std::uint8_t>>;
using MaskGrid = std::vector<std::vector<std::uint8_t>>;
using Cell = std::pair<std::size_t, std::size_t>;

struct FlowingWaterCell
{
	std::uint32_t x, y;
	float local_surface;
};

constexpr std::array<std::pair<int, int>, 4> CARDINAL{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
constexpr double MAX_STEEP_WATER_AREA_M2 = 250000.0;
constexpr double MIN_STEEP_WATER_SLOPE = 0.35;
constexpr double STEEP_WATER_LAND_BELOW_M = 2.0;
constexpr double MIN_PERCHED_FRACTION = 0.10;
constexpr double WATER_UP_TOLERANCE_M = 2.0;
constexpr double CHANNEL_WALL_SLOPE = 0.35;
constexpr double MIN_BANK_AREA_M2 = 25000.0;
constexpr double FALL_REACH_M = 128.0;
constexpr double LEVEL_RUN_M = 30.0;

bool inside(int x, int y, std::size_t w, std::size_t h);
double median(std::vector<double> &values);

std::vector<double> blur_values(const std::vector<double> &values,
		const std::vector<double> &kernel, int half, bool binary)
{
	const auto len = values.size();
	std::vector<std::uint32_t> bad(len + 1, 0), ones, zeros;
	for (std::size_t i = 0; i < len; ++i) {
		bad[i + 1] = bad[i] + !std::isfinite(values[i]);
		if (binary) {
			if (ones.empty()) {
				ones.assign(len + 1, 0);
				zeros.assign(len + 1, 0);
			}
			ones[i + 1] = ones[i] + (values[i] == 1.0);
			zeros[i + 1] = zeros[i] + (values[i] == 0.0);
		}
	}
	double full_weight = 0.0;
	for (const auto weight : kernel)
		full_weight += weight;
	const auto radius = static_cast<std::size_t>(half);
	std::vector<double> out;
	out.reserve(len);
	std::size_t i = 0;
	while (i < len) {
		if (binary) {
			const auto left = i > radius ? i - radius : 0;
			const auto right = std::min(len, i + radius + 1);
			const auto window = right - left;
			if (ones[right] - ones[left] == window) {
				out.push_back(1.0);
				++i;
				continue;
			}
			if (zeros[right] - zeros[left] == window) {
				out.push_back(0.0);
				++i;
				continue;
			}
		}
		constexpr std::size_t LANES = 8;
		if (i >= radius && i + LANES + radius <= len &&
				bad[i + LANES + radius] == bad[i - radius]) {
			std::array<double, LANES> sums{};
			for (std::size_t tap = 0; tap < kernel.size(); ++tap)
				for (std::size_t lane = 0; lane < LANES; ++lane)
					sums[lane] += values[i - radius + tap + lane] * kernel[tap];
			for (const auto sum : sums)
				out.push_back(sum / full_weight);
			i += LANES;
			continue;
		}
		if (i >= radius && i + radius < len && bad[i + radius + 1] == bad[i - radius]) {
			double sum = 0.0;
			for (std::size_t tap = 0; tap < kernel.size(); ++tap)
				sum += values[i - radius + tap] * kernel[tap];
			out.push_back(sum / full_weight);
			++i;
			continue;
		}
		double sum = 0.0, weight_sum = 0.0;
		for (std::size_t tap = 0; tap < kernel.size(); ++tap) {
			const auto index =
					static_cast<std::int64_t>(i) + static_cast<std::int64_t>(tap) - half;
			if (index < 0 || index >= static_cast<std::int64_t>(len))
				continue;
			const auto value = values[static_cast<std::size_t>(index)];
			if (!std::isfinite(value))
				continue;
			sum += value * kernel[tap];
			weight_sum += kernel[tap];
		}
		out.push_back(weight_sum > 0.0 ? sum / weight_sum
									   : std::numeric_limits<double>::quiet_NaN());
		++i;
	}
	return out;
}

struct ColumnBlurGroup
{
	std::size_t first;
	std::size_t width;
	// Row-major values for `width` consecutive columns.
	std::vector<double> values;
};

// Blur a band of adjacent columns while gathering them row-major. Keeping the
// grouped result row-major makes the destination writes contiguous too.
std::vector<ColumnBlurGroup> blur_column_chunk(
		const std::vector<std::vector<double>> &src, std::size_t x0, std::size_t x1,
		const std::vector<double> &kernel, int half, bool binary)
{
	constexpr std::size_t GROUP = 8;
	const auto count = (x1 - x0 + GROUP - 1) / GROUP;
	std::vector<ColumnBlurGroup> groups(count);
	parallel_rows(0, count, [&](std::size_t group) {
		const auto gx = x0 + group * GROUP;
		const auto gw = std::min(GROUP, x1 - gx);
		std::vector<std::vector<double>> columns(gw, std::vector<double>(src.size()));
		for (std::size_t y = 0; y < src.size(); ++y)
			for (std::size_t c = 0; c < gw; ++c)
				columns[c][y] = src[y][gx + c];

		std::vector<double> row_major(src.size() * gw);
		for (std::size_t c = 0; c < gw; ++c) {
			const auto blurred = blur_values(columns[c], kernel, half, binary);
			for (std::size_t y = 0; y < src.size(); ++y)
				row_major[y * gw + c] = blurred[y];
		}
		groups[group] = {gx, gw, std::move(row_major)};
	});
	return groups;
}

template <typename T, typename Convert>
void store_column_chunk(std::vector<std::vector<T>> &dst,
		const std::vector<ColumnBlurGroup> &groups, Convert &&convert)
{
	parallel_rows(0, dst.size(), [&](std::size_t y) {
		for (const auto &group : groups)
			for (std::size_t c = 0; c < group.width; ++c)
				dst[y][group.first + c] = convert(group.values[y * group.width + c]);
	});
}

bool get_bit(const std::vector<std::uint64_t> &mask, std::size_t i)
{
	return ((mask[i >> 6] >> (i & 63)) & 1u) != 0;
}

void set_bit(std::vector<std::uint64_t> &mask, std::size_t i)
{
	mask[i >> 6] |= std::uint64_t{1} << (i & 63);
}

void clear_bit(std::vector<std::uint64_t> &mask, std::size_t i)
{
	mask[i >> 6] &= ~(std::uint64_t{1} << (i & 63));
}

std::int64_t slope_step(double meters_per_cell)
{
	if (!(meters_per_cell > 0.0) || !std::isfinite(meters_per_cell))
		return 1;
	return std::clamp<std::int64_t>(static_cast<std::int64_t>(std::llround(std::clamp(
											10.0 / meters_per_cell, 0.0, 16.0))),
			1, 16);
}

struct Walk
{
	Cell end;
	double run{0.0};
	Cell mid;
	double mid_run{0.0};
};

Walk walk_from_bank(const std::vector<std::uint8_t> &bank_dist,
		const std::vector<std::uint64_t> &body, std::size_t w, std::size_t x,
		std::size_t y, std::int64_t step)
{
	const auto h = bank_dist.size() / w;
	std::array<std::pair<Cell, double>, 16> path{};
	std::size_t taken = 0;
	Cell current{x, y};
	double run = 0.0;
	while (taken < static_cast<std::size_t>(std::min<std::int64_t>(step, path.size()))) {
		const double here = bank_dist[current.second * w + current.first];
		std::optional<std::tuple<double, std::size_t, std::size_t, double>> best;
		for (const auto &[dx, dy] : std::array<std::pair<int, int>, 8>{{{1, 0}, {-1, 0},
					 {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}}) {
			const int nx = static_cast<int>(current.first) + dx;
			const int ny = static_cast<int>(current.second) + dy;
			if (!inside(nx, ny, w, h))
				continue;
			const auto ux = static_cast<std::size_t>(nx),
					   uy = static_cast<std::size_t>(ny);
			const auto index = uy * w + ux;
			if (!get_bit(body, index))
				continue;
			const double length = dx != 0 && dy != 0 ? std::sqrt(2.0) : 1.0;
			const double gain = (bank_dist[index] - here) / length;
			if (gain > 0.0 && (!best || gain > std::get<0>(*best)))
				best = {gain, ux, uy, length};
		}
		if (!best)
			break;
		current = {std::get<1>(*best), std::get<2>(*best)};
		run += std::get<3>(*best);
		path[taken++] = {current, run};
	}
	const auto middle = taken >= 2 ? path[taken / 2 - 1] : std::pair{Cell{x, y}, 0.0};
	return {current, run, middle.first, middle.second};
}

bool on_channel_wall(const HeightGrid &heights, const Walk &walk, std::size_t x,
		std::size_t y, double meters_per_cell)
{
	const double here = heights[y][x];
	const double there = heights[walk.end.second][walk.end.first];
	const double mid = heights[walk.mid.second][walk.mid.first];
	if (walk.run == 0.0 || !std::isfinite(here) || !std::isfinite(there) ||
			!std::isfinite(mid))
		return false;
	const auto falls = [&](double from, double to, double length, double slope) {
		return length <= 0.0 || (from - to) / (length * meters_per_cell) > slope;
	};
	return falls(here, there, walk.run, CHANNEL_WALL_SLOPE) &&
		   falls(here, mid, walk.mid_run, CHANNEL_WALL_SLOPE / 2.0) &&
		   falls(mid, there, walk.run - walk.mid_run, CHANNEL_WALL_SLOPE / 2.0);
}

std::vector<std::uint8_t> bank_distances(const CoverGrid &cover, double meters_per_cell)
{
	const std::size_t h = cover.size(), w = cover.front().size();
	std::vector<std::uint8_t> distance;
	distance.reserve(w * h);
	for (const auto &row : cover)
		for (const auto value : row)
			distance.push_back(value == land_cover::LC_WATER ? 255 : 0);
	const auto min_bank_cells =
			meters_per_cell > 0.0 && std::isfinite(meters_per_cell)
					? rust_float_to_size_t(
							  MIN_BANK_AREA_M2 / (meters_per_cell * meters_per_cell))
					: 0;
	if (min_bank_cells > 0) {
		std::vector<std::uint64_t> seen((w * h + 63) / 64), big((w * h + 63) / 64);
		std::vector<std::size_t> patch;
		for (std::size_t start = 0; start < w * h; ++start) {
			if (distance[start] != 0 || get_bit(seen, start))
				continue;
			const std::size_t x = start % w, y = start / w;
			const bool shore = (x > 0 && distance[start - 1] != 0) ||
							   (x + 1 < w && distance[start + 1] != 0) ||
							   (y > 0 && distance[start - w] != 0) ||
							   (y + 1 < h && distance[start + w] != 0);
			if (!shore)
				continue;
			patch.assign(1, start);
			set_bit(seen, start);
			bool too_big = false;
			std::size_t head = 0;
			while (head < patch.size()) {
				const auto i = patch[head++], px = i % w, py = i / w;
				if (px == 0 || py == 0 || px + 1 == w || py + 1 == h ||
						patch.size() >= min_bank_cells) {
					too_big = true;
					break;
				}
				for (const auto n : {i - 1, i + 1, i - w, i + w}) {
					if (distance[n] != 0)
						continue;
					if (get_bit(big, n)) {
						too_big = true;
						break;
					}
					if (!get_bit(seen, n)) {
						set_bit(seen, n);
						patch.push_back(n);
					}
				}
				if (too_big)
					break;
			}
			for (const auto i : patch) {
				if (too_big)
					set_bit(big, i);
				else
					distance[i] = 255;
			}
		}
	}
	water_depth::chamfer_3_4_dt(distance, w, h);
	return distance;
}

void keep_anchored_walls(std::vector<std::uint64_t> &mask, const CoverGrid &cover,
		const std::vector<Cell> &cells, std::vector<bool> &walls)
{
	const int h = static_cast<int>(cover.size()),
			  w = static_cast<int>(cover.front().size());
	std::unordered_set<std::uint32_t> reached;
	std::vector<Cell> queue;
	for (std::size_t i = 0; i < cells.size(); ++i) {
		if (!walls[i])
			continue;
		const auto [x, y] = cells[i];
		bool on_land = false;
		for (const auto &[dx, dy] : std::array<std::pair<int, int>, 8>{{{-1, -1}, {0, -1},
					 {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}}}) {
			const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
			if (!inside(nx, ny, w, h) || cover[ny][nx] != land_cover::LC_WATER) {
				on_land = true;
				break;
			}
		}
		const auto key = static_cast<std::uint32_t>(y * cover.front().size() + x);
		if (on_land && reached.insert(key).second)
			queue.emplace_back(x, y);
	}
	while (!queue.empty()) {
		const auto [x, y] = queue.back();
		queue.pop_back();
		for (int dy = -1; dy <= 1; ++dy)
			for (int dx = -1; dx <= 1; ++dx) {
				if (!dx && !dy)
					continue;
				const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
				if (!inside(nx, ny, w, h))
					continue;
				const auto index = static_cast<std::size_t>(ny) * w + nx;
				if (get_bit(mask, index) &&
						reached.insert(static_cast<std::uint32_t>(index)).second)
					queue.emplace_back(nx, ny);
			}
	}
	for (std::size_t i = 0; i < cells.size(); ++i) {
		const auto [x, y] = cells[i];
		if (walls[i] && !reached.contains(static_cast<std::uint32_t>(
								y * cover.front().size() + x))) {
			walls[i] = false;
			clear_bit(mask, y * cover.front().size() + x);
		}
	}
}

bool under_level_water(const HeightGrid &heights, const CoverGrid &cover,
		const std::vector<std::uint8_t> &bank_dist,
		const std::vector<std::uint64_t> &falling, std::size_t x, std::size_t y,
		double run)
{
	const int h = static_cast<int>(heights.size()),
			  w = static_cast<int>(heights.front().size());
	const double here = heights[y][x];
	Cell current{x, y};
	double level = 0.0;
	for (;;) {
		const auto cur = bank_dist[current.second * w + current.first];
		std::optional<std::tuple<double, std::size_t, std::size_t, double>> best;
		for (const auto &[dx, dy] : std::array<std::pair<int, int>, 8>{{{1, 0}, {-1, 0},
					 {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}}) {
			const int nx = static_cast<int>(current.first) + dx;
			const int ny = static_cast<int>(current.second) + dy;
			if (!inside(nx, ny, w, h))
				continue;
			const auto ux = static_cast<std::size_t>(nx),
					   uy = static_cast<std::size_t>(ny);
			const auto d = bank_dist[uy * w + ux];
			if (d >= cur)
				continue;
			const double len = dx && dy ? std::sqrt(2.0) : 1.0;
			const double drop = (cur - d) / len;
			if (!best || drop > std::get<0>(*best))
				best = {drop, ux, uy, len};
		}
		if (!best)
			return false;
		const auto [_, nx, ny, len] = *best;
		const auto idx = ny * w + nx;
		if (bank_dist[idx] == 0)
			return false;
		const bool is_level = cover[ny][nx] == land_cover::LC_WATER &&
							  !get_bit(falling, idx) && heights[ny][nx] >= here;
		level = is_level ? level + len : 0.0;
		if (level >= run)
			return true;
		current = {nx, ny};
	}
}

std::optional<double> local_water_median(const HeightGrid &heights,
		const std::vector<std::uint64_t> &body, std::size_t x, std::size_t y, int radius,
		std::size_t min_samples)
{
	const int h = static_cast<int>(heights.size()),
			  w = static_cast<int>(heights.front().size());
	std::vector<double> samples;
	samples.reserve(static_cast<std::size_t>((radius * 2 + 1) * (radius * 2 + 1)));
	for (int dy = -radius; dy <= radius; ++dy)
		for (int dx = -radius; dx <= radius; ++dx) {
			const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
			if (!inside(nx, ny, w, h) ||
					!get_bit(body, static_cast<std::size_t>(ny) * w + nx))
				continue;
			const double v = heights[ny][nx];
			if (std::isfinite(v))
				samples.push_back(v);
		}
	if (samples.size() < min_samples)
		return std::nullopt;
	return median(samples);
}

std::optional<double> local_reach_median(const HeightGrid &heights,
		const std::vector<std::uint64_t> &reach, const CoverGrid &cover, std::size_t x,
		std::size_t y, int radius, std::size_t min_samples)
{
	const int h = static_cast<int>(heights.size()),
			  w = static_cast<int>(heights.front().size());
	std::vector<std::pair<double, int>> samples;
	int clear = radius;
	for (int dy = -radius; dy <= radius; ++dy)
		for (int dx = -radius; dx <= radius; ++dx) {
			const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
			if (!inside(nx, ny, w, h))
				continue;
			const int d = std::max(std::abs(dx), std::abs(dy));
			const auto index = static_cast<std::size_t>(ny) * w + nx;
			if (get_bit(reach, index)) {
				if (std::isfinite(heights[ny][nx]))
					samples.emplace_back(heights[ny][nx], d);
			} else if (cover[ny][nx] == land_cover::LC_WATER) {
				clear = std::min(clear, d - 1);
			}
		}
	if (clear < radius) {
		std::erase_if(samples, [clear](const auto &v) { return v.second > clear; });
		const auto side = static_cast<std::size_t>(clear * 2 + 1);
		if (clear < 1 || samples.size() < std::min(min_samples, side * side / 2))
			return std::isfinite(heights[y][x]) ? std::optional<double>{heights[y][x]}
												: std::nullopt;
	} else if (samples.size() < min_samples) {
		return std::nullopt;
	}
	std::vector<double> values;
	values.reserve(samples.size());
	for (const auto &v : samples)
		values.push_back(v.first);
	return median(values);
}

struct Reach
{
	bool below{false};
	std::vector<Cell> cells;
};

struct OtherLevels
{
	std::vector<Reach> reaches;
	std::vector<Cell> shadow;
};

std::optional<double> water_surface_slope(const HeightGrid &heights, std::size_t x,
		std::size_t y, std::int64_t step, double meters_per_cell,
		const std::function<bool(std::size_t, std::size_t)> &is_water)
{
	const int h = static_cast<int>(heights.size()),
			  w = static_cast<int>(heights.front().size());
	const double here = heights[y][x];
	if (!std::isfinite(here))
		return std::nullopt;
	const auto at = [&](int dx, int dy) -> std::optional<std::pair<double, double>> {
		for (int d = static_cast<int>(step); d >= 1; --d) {
			const int nx = static_cast<int>(x) + dx * d;
			const int ny = static_cast<int>(y) + dy * d;
			if (!inside(nx, ny, w, h) || !is_water(nx, ny) ||
					!std::isfinite(heights[ny][nx]))
				continue;
			return std::pair{heights[ny][nx], d * meters_per_cell};
		}
		return std::nullopt;
	};
	const auto axis = [here](const auto &low, const auto &high) -> std::optional<double> {
		if (low && high)
			return (high->first - low->first) / (low->second + high->second);
		if (low)
			return (here - low->first) / low->second;
		if (high)
			return (high->first - here) / high->second;
		return std::nullopt;
	};
	const auto gx = axis(at(-1, 0), at(1, 0));
	const auto gz = axis(at(0, -1), at(0, 1));
	if (!gx && !gz)
		return std::nullopt;
	return std::hypot(gx.value_or(0.0), gz.value_or(0.0));
}

OtherLevels split_off_other_levels(const std::vector<Cell> &body, double level,
		const HeightGrid &heights, const CoverGrid &cover,
		std::vector<std::uint64_t> &in_body, std::vector<std::uint64_t> &marks,
		double meters_per_cell)
{
	constexpr double LEVEL_SPLIT_M = 5.0;
	constexpr double SPILL_BANK_M = 2.5;
	constexpr std::size_t MIN_SPILL_EDGES = 4;
	const int h = static_cast<int>(heights.size()),
			  w = static_cast<int>(heights.front().size());
	const auto side = [level](double v) -> int {
		return v < level - LEVEL_SPLIT_M ? -1 : v > level + LEVEL_SPLIT_M ? 1 : 0;
	};
	const std::size_t max_shadow_cells =
			meters_per_cell > 0.0 && std::isfinite(meters_per_cell)
					? rust_float_to_size_t(MAX_STEEP_WATER_AREA_M2 /
										   (meters_per_cell * meters_per_cell))
					: 0;
	const auto step = slope_step(meters_per_cell);
	OtherLevels out;
	std::vector<Cell> part, queue;
	for (const auto &[sx, sy] : body) {
		const int s = side(heights[sy][sx]);
		const auto start = sy * cover.front().size() + sx;
		if (s == 0 || get_bit(marks, start))
			continue;
		set_bit(marks, start);
		part.clear();
		queue.clear();
		queue.emplace_back(sx, sy);
		std::size_t head = 0, land = 0, spill = 0, water = 0;
		while (head < queue.size()) {
			const auto [x, y] = queue[head++];
			part.emplace_back(x, y);
			for (const auto &[dx, dy] : CARDINAL) {
				const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
				if (!inside(nx, ny, w, h))
					continue;
				const auto ux = static_cast<std::size_t>(nx),
						   uy = static_cast<std::size_t>(ny);
				const double v = heights[uy][ux];
				if (cover[uy][ux] != land_cover::LC_WATER) {
					++land;
					if (v < level - SPILL_BANK_M)
						++spill;
					continue;
				}
				const auto index = uy * cover.front().size() + ux;
				if (get_bit(in_body, index) && side(v) == s) {
					if (!get_bit(marks, index)) {
						set_bit(marks, index);
						queue.emplace_back(ux, uy);
					}
					continue;
				}
				++water;
			}
		}
		if (s < 0) {
			if (spill >= MIN_SPILL_EDGES && 2 * spill >= land && 2 * land >= water)
				out.reaches.push_back({true, part});
			continue;
		}
		if (land == 0 || land < 2 * water)
			continue;
		bool steep = false;
		if (part.size() <= max_shadow_cells) {
			std::vector<double> slopes;
			for (const auto &[x, y] : part) {
				if (auto slope = water_surface_slope(heights, x, y, step, meters_per_cell,
							[&](std::size_t nx, std::size_t ny) {
								return get_bit(in_body, ny * cover.front().size() + nx);
							}))
					slopes.push_back(*slope);
			}
			steep = !slopes.empty() && median(slopes) > MIN_STEEP_WATER_SLOPE;
		}
		if (steep)
			out.shadow.insert(out.shadow.end(), part.begin(), part.end());
		else
			out.reaches.push_back({false, part});
	}
	for (const auto &[x, y] : body)
		clear_bit(marks, y * cover.front().size() + x);
	for (const auto &reach : out.reaches)
		for (const auto &[x, y] : reach.cells)
			clear_bit(in_body, y * cover.front().size() + x);
	for (const auto &[x, y] : out.shadow)
		clear_bit(in_body, y * cover.front().size() + x);
	return out;
}

void grow_reaches(std::vector<Reach> &reaches, double surface, double level,
		const HeightGrid &heights, std::vector<std::uint64_t> &in_body)
{
	const int h = static_cast<int>(heights.size()),
			  w = static_cast<int>(heights.front().size());
	const double below_from = surface - WATER_UP_TOLERANCE_M;
	const double above_from = std::max(surface, level - 1.0) + WATER_UP_TOLERANCE_M;
	for (auto &reach : reaches) {
		for (std::size_t i = 0; i < reach.cells.size(); ++i) {
			const auto [x, y] = reach.cells[i];
			for (const auto &[dx, dy] : CARDINAL) {
				const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
				if (!inside(nx, ny, w, h))
					continue;
				const auto ux = static_cast<std::size_t>(nx),
						   uy = static_cast<std::size_t>(ny);
				const auto index = uy * w + ux;
				const double v = heights[uy][ux];
				const bool past = reach.below ? v < below_from : v > above_from;
				if (past && get_bit(in_body, index)) {
					clear_bit(in_body, index);
					reach.cells.emplace_back(ux, uy);
				}
			}
		}
	}
}

bool next_to(const std::vector<std::uint64_t> &mask, std::size_t w, std::size_t h,
		std::size_t x, std::size_t y)
{
	return (x > 0 && get_bit(mask, y * w + x - 1)) ||
		   (x + 1 < w && get_bit(mask, y * w + x + 1)) ||
		   (y > 0 && get_bit(mask, (y - 1) * w + x)) ||
		   (y + 1 < h && get_bit(mask, (y + 1) * w + x));
}

using OtherWaterSteps = std::unordered_map<std::uint32_t, std::uint16_t>;

OtherWaterSteps steps_from_other_water(const std::vector<Cell> &reach,
		const std::vector<std::uint64_t> &members, const CoverGrid &cover,
		std::uint16_t cap)
{
	const int h = static_cast<int>(cover.size()),
			  w = static_cast<int>(cover.front().size());
	OtherWaterSteps steps;
	std::vector<Cell> frontier, next;
	for (const auto &[x, y] : reach) {
		bool adjacent = false;
		for (const auto &[dx, dy] : CARDINAL) {
			const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
			if (inside(nx, ny, w, h) && cover[ny][nx] == land_cover::LC_WATER &&
					!get_bit(members, static_cast<std::size_t>(ny) * w + nx)) {
				adjacent = true;
				break;
			}
		}
		if (adjacent) {
			steps.emplace(static_cast<std::uint32_t>(y * cover.front().size() + x), 0);
			frontier.emplace_back(x, y);
		}
	}
	std::uint16_t distance = 0;
	while (!frontier.empty() && distance < cap) {
		++distance;
		next.clear();
		for (const auto &[x, y] : frontier)
			for (const auto &[dx, dy] : CARDINAL) {
				const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
				if (!inside(nx, ny, w, h))
					continue;
				const auto ux = static_cast<std::size_t>(nx),
						   uy = static_cast<std::size_t>(ny);
				const auto index = uy * cover.front().size() + ux;
				if (get_bit(members, index)) {
					const auto key = static_cast<std::uint32_t>(index);
					if (steps.emplace(key, distance).second)
						next.emplace_back(ux, uy);
				}
			}
		frontier.swap(next);
	}
	return steps;
}

bool falls_away(const OtherWaterSteps &steps, const Walk &walk, std::size_t w,
		std::size_t x, std::size_t y)
{
	const auto start = steps.find(static_cast<std::uint32_t>(y * w + x));
	if (start == steps.end())
		return false;
	const auto end =
			steps.find(static_cast<std::uint32_t>(walk.end.second * w + walk.end.first));
	const auto end_steps =
			end == steps.end() ? std::numeric_limits<std::uint16_t>::max() : end->second;
	return walk.run > 0.0 && end_steps > start->second &&
		   double(end_steps - start->second) >= 0.5 * walk.run;
}

double median(std::vector<double> &values)
{
	if (values.empty())
		return std::numeric_limits<double>::quiet_NaN();
	const auto mid = values.begin() + values.size() / 2;
	std::nth_element(values.begin(), mid, values.end());
	return *mid;
}

double interquartile_range(std::vector<double> &values)
{
	if (values.size() < 4)
		return 0.0;
	const std::size_t q1i = values.size() / 4, q3i = values.size() * 3 / 4;
	std::nth_element(values.begin(), values.begin() + q1i, values.end());
	const double q1 = values[q1i];
	std::nth_element(values.begin(), values.begin() + q3i, values.end());
	return std::max(0.0, values[q3i] - q1);
}

bool inside(int x, int y, std::size_t w, std::size_t h)
{
	return x >= 0 && y >= 0 && static_cast<std::size_t>(x) < w &&
		   static_cast<std::size_t>(y) < h;
}

std::size_t coarse_smoothing_step(
		std::size_t bbox_width, std::size_t bbox_height, double sigma_cells)
{
	constexpr double COARSE_PER_SIGMA = 8.0;
	constexpr double MAX_COARSE_CELLS = 4.0 * 1024.0 * 1024.0;
	const double from_sigma = std::max(1.0, std::floor(sigma_cells / COARSE_PER_SIGMA));
	const double from_budget =
			std::max(1.0, std::ceil(std::sqrt(static_cast<double>(bbox_width) *
											  bbox_height / MAX_COARSE_CELLS)));
	return static_cast<std::size_t>(std::max(from_sigma, from_budget));
}

std::vector<double> smooth_sparse_water_field(
		const std::vector<FlowingWaterCell> &cells, double sigma_cells)
{
	if (cells.empty())
		return {};
	std::uint32_t min_x = std::numeric_limits<std::uint32_t>::max();
	std::uint32_t min_y = std::numeric_limits<std::uint32_t>::max();
	std::uint32_t max_x = 0, max_y = 0;
	for (const auto &cell : cells) {
		min_x = std::min(min_x, cell.x);
		min_y = std::min(min_y, cell.y);
		max_x = std::max(max_x, cell.x);
		max_y = std::max(max_y, cell.y);
	}
	const std::size_t bbox_width = static_cast<std::size_t>(max_x - min_x) + 1;
	const std::size_t bbox_height = static_cast<std::size_t>(max_y - min_y) + 1;
	const std::size_t step = coarse_smoothing_step(bbox_width, bbox_height, sigma_cells);
	const std::size_t coarse_width = (bbox_width - 1) / step + 1;
	const std::size_t coarse_height = (bbox_height - 1) / step + 1;
	std::vector<std::vector<double>> sums(
			coarse_height, std::vector<double>(coarse_width, 0.0));
	std::vector<std::vector<std::uint32_t>> counts(
			coarse_height, std::vector<std::uint32_t>(coarse_width, 0));
	for (const auto &cell : cells) {
		const std::size_t x = (cell.x - min_x) / step;
		const std::size_t y = (cell.y - min_y) / step;
		sums[y][x] += cell.local_surface;
		++counts[y][x];
	}
	HeightGrid coarse(coarse_height, std::vector<double>(coarse_width));
	for (std::size_t y = 0; y < coarse_height; ++y)
		for (std::size_t x = 0; x < coarse_width; ++x)
			coarse[y][x] = counts[y][x] ? sums[y][x] / counts[y][x]
										: std::numeric_limits<double>::quiet_NaN();
	std::vector<std::vector<double>>().swap(sums);
	std::vector<std::vector<std::uint32_t>>().swap(counts);
	const auto blurred = gaussian_blur_grid(coarse, sigma_cells / step);
	std::vector<double> out;
	out.reserve(cells.size());
	for (const auto &cell : cells) {
		const double fx = static_cast<double>(cell.x - min_x) / step - 0.5;
		const double fy = static_cast<double>(cell.y - min_y) / step - 0.5;
		const double ix = std::floor(fx), iy = std::floor(fy);
		const double tx = fx - ix, ty = fy - iy;
		double weighted = 0.0, weight_sum = 0.0;
		for (int dy = 0; dy <= 1; ++dy)
			for (int dx = 0; dx <= 1; ++dx) {
				const auto x = static_cast<std::int64_t>(ix) + dx;
				const auto y = static_cast<std::int64_t>(iy) + dy;
				if (x < 0 || y < 0 || static_cast<std::size_t>(x) >= coarse_width ||
						static_cast<std::size_t>(y) >= coarse_height)
					continue;
				const double weight = (dx ? tx : 1.0 - tx) * (dy ? ty : 1.0 - ty);
				const double value =
						blurred[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)];
				if (std::isfinite(value)) {
					weighted += value * weight;
					weight_sum += weight;
				}
			}
		out.push_back(weight_sum > 0.0 ? weighted / weight_sum : cell.local_surface);
	}
	return out;
}

bool has_non_water_neighbor(const CoverGrid &cover, std::size_t x, std::size_t y)
{
	const auto h = cover.size(), w = cover.front().size();
	for (const auto &[dx, dy] : CARDINAL) {
		const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
		if (!inside(nx, ny, w, h) ||
				cover[static_cast<std::size_t>(ny)][static_cast<std::size_t>(nx)] !=
						land_cover::LC_WATER)
			return true;
	}
	return false;
}

std::optional<std::uint8_t> nearest_non_water_class(
		const CoverGrid &cover, std::size_t x, std::size_t y, int radius)
{
	const int h = static_cast<int>(cover.size());
	const int w = h ? static_cast<int>(cover.front().size()) : 0;
	for (int r = 1; r <= radius; ++r)
		for (int dy = -r; dy <= r; ++dy)
			for (int dx = -r; dx <= r; ++dx) {
				if (std::abs(dx) != r && std::abs(dy) != r)
					continue;
				const int nx = static_cast<int>(x) + dx;
				const int ny = static_cast<int>(y) + dy;
				if (!inside(nx, ny, w, h))
					continue;
				const auto value = cover[ny][nx];
				if (value != 0 && value != land_cover::LC_WATER)
					return value;
			}
	return std::nullopt;
}

double histogram_mode(const std::vector<double> &values, double bin_size)
{
	const auto &[lo, hi] = std::minmax_element(values.begin(), values.end());
	if (*hi - *lo < bin_size)
		return *lo;
	const auto count = static_cast<std::size_t>(std::ceil((*hi - *lo) / bin_size)) + 1;
	std::vector<std::size_t> bins(count);
	for (double value : values) {
		const auto index =
				std::min(count - 1, static_cast<std::size_t>((value - *lo) / bin_size));
		++bins[index];
	}
	// Rust's Iterator::max_by_key keeps the last item on ties. std::max_element
	// keeps the first, so select with >= to preserve the same water level when
	// two elevation bins have equal counts.
	std::size_t peak = 0;
	for (std::size_t i = 1; i < bins.size(); ++i)
		if (bins[i] >= bins[peak])
			peak = i;
	return *lo + (static_cast<double>(peak) + 0.5) * bin_size;
}

double clamp_by_adjacent_land(double proposed, const std::vector<Cell> &component,
		const HeightGrid &heights, const CoverGrid &cover)
{
	const auto h = heights.size(), w = heights.front().size();
	std::unordered_set<std::uint64_t> seen;
	std::vector<double> adjacent;
	for (const auto &[x, y] : component)
		for (const auto &[dx, dy] : CARDINAL) {
			const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
			if (!inside(nx, ny, w, h) || cover[ny][nx] == land_cover::LC_WATER)
				continue;
			const auto key = static_cast<std::uint64_t>(ny) * w + nx;
			if (!seen.insert(key).second || !std::isfinite(heights[ny][nx]))
				continue;
			adjacent.push_back(heights[ny][nx]);
		}
	if (adjacent.empty())
		return proposed;
	const auto q = adjacent.begin() + adjacent.size() / 4;
	std::nth_element(adjacent.begin(), q, adjacent.end());
	return std::min(proposed, *q);
}

std::optional<double> lowest_adjacent_land(
		const HeightGrid &heights, const CoverGrid &cover, std::size_t x, std::size_t y)
{
	const auto h = heights.size(), w = heights.front().size();
	double lowest = std::numeric_limits<double>::infinity();
	for (const auto &[dx, dy] : CARDINAL) {
		const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
		if (!inside(nx, ny, w, h) || cover[ny][nx] == land_cover::LC_WATER)
			continue;
		if (std::isfinite(heights[ny][nx]))
			lowest = std::min(lowest, heights[ny][nx]);
	}
	return std::isfinite(lowest) ? std::optional<double>{lowest} : std::nullopt;
}

std::size_t drop_water_on_steep_terrain(
		const HeightGrid &heights, CoverGrid &cover, double meters_per_cell)
{
	constexpr int SEARCH_RADIUS = 8;
	if (heights.empty() || meters_per_cell <= 0.0 || !std::isfinite(meters_per_cell))
		return 0;
	const std::size_t h = heights.size(), w = heights.front().size();
	if (!w || cover.size() != h || cover.front().size() != w)
		return 0;
	// Rust's float-to-usize cast saturates before the 1..=16 clamp. Bound
	// before rounding so tiny but finite cell sizes cannot overflow llround.
	const int step = std::clamp(
			static_cast<int>(std::llround(std::clamp(10.0 / meters_per_cell, 0.0, 16.0))),
			1, 16);
	const double max_cell_count =
			MAX_STEEP_WATER_AREA_M2 / (meters_per_cell * meters_per_cell);
	const std::size_t max_cells = rust_float_to_size_t(max_cell_count);
	const std::size_t cell_count = w * h;
	std::vector<std::uint64_t> visited((cell_count + 63) / 64);
	std::vector<std::uint64_t> in_component((cell_count + 63) / 64);
	const auto get_bit = [](const std::vector<std::uint64_t> &mask, std::size_t i) {
		return (mask[i >> 6] >> (i & 63)) & 1u;
	};
	const auto set_bit = [](std::vector<std::uint64_t> &mask, std::size_t i) {
		mask[i >> 6] |= std::uint64_t{1} << (i & 63);
	};
	const auto clear_bit = [](std::vector<std::uint64_t> &mask, std::size_t i) {
		mask[i >> 6] &= ~(std::uint64_t{1} << (i & 63));
	};
	const auto cell_slope = [&](std::size_t x, std::size_t y) -> std::optional<double> {
		const double here = heights[y][x];
		if (!std::isfinite(here))
			return std::nullopt;
		const auto at = [&](int dx, int dy) -> std::optional<std::pair<double, double>> {
			for (int d = step; d >= 1; --d) {
				const int nx = static_cast<int>(x) + dx * d;
				const int ny = static_cast<int>(y) + dy * d;
				if (!inside(nx, ny, w, h) || cover[ny][nx] != land_cover::LC_WATER)
					continue;
				const double value = heights[ny][nx];
				if (std::isfinite(value))
					return std::pair{value, d * meters_per_cell};
			}
			return std::nullopt;
		};
		const auto axis = [here](const auto &low,
								  const auto &high) -> std::optional<double> {
			if (low && high)
				return (high->first - low->first) / (low->second + high->second);
			if (low)
				return (here - low->first) / low->second;
			if (high)
				return (high->first - here) / high->second;
			return std::nullopt;
		};
		const auto gx = axis(at(-1, 0), at(1, 0));
		const auto gz = axis(at(0, -1), at(0, 1));
		if (!gx && !gz)
			return std::nullopt;
		return std::hypot(gx.value_or(0.0), gz.value_or(0.0));
	};
	std::vector<std::pair<std::uint32_t, std::uint32_t>> component;
	std::deque<Cell> queue;
	std::vector<std::pair<std::uint32_t, std::uint32_t>> dropped;
	std::size_t dropped_components = 0;
	for (std::size_t sy = 0; sy < h; ++sy)
		for (std::size_t sx = 0; sx < w; ++sx) {
			const std::size_t start = sy * w + sx;
			if (get_bit(visited, start) || cover[sy][sx] != land_cover::LC_WATER)
				continue;
			component.clear();
			queue.clear();
			std::size_t size = 0;
			queue.emplace_back(sx, sy);
			set_bit(visited, start);
			while (!queue.empty()) {
				const auto [x, y] = queue.front();
				queue.pop_front();
				++size;
				if (max_cells == std::numeric_limits<std::size_t>::max() ||
						size <= max_cells + 1)
					component.emplace_back(
							static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y));
				for (const auto &[dx, dy] : CARDINAL) {
					const int nx = static_cast<int>(x) + dx;
					const int ny = static_cast<int>(y) + dy;
					if (!inside(nx, ny, w, h) || cover[ny][nx] != land_cover::LC_WATER)
						continue;
					const std::size_t index = static_cast<std::size_t>(ny) * w + nx;
					if (!get_bit(visited, index)) {
						set_bit(visited, index);
						queue.emplace_back(static_cast<std::size_t>(nx),
								static_cast<std::size_t>(ny));
					}
				}
			}
			if (size > max_cells)
				continue;
			std::vector<double> slopes;
			slopes.reserve(component.size());
			for (const auto &[x, y] : component)
				if (const auto slope = cell_slope(x, y))
					slopes.push_back(*slope);
			if (slopes.empty() || median(slopes) <= MIN_STEEP_WATER_SLOPE)
				continue;
			for (const auto &[x, y] : component)
				set_bit(in_component, y * w + x);
			std::size_t edge_cells = 0, perched = 0;
			for (const auto &[x, y] : component) {
				const double here = heights[y][x];
				if (!std::isfinite(here))
					continue;
				double lowest = std::numeric_limits<double>::infinity();
				for (const auto &[dx, dy] : CARDINAL) {
					const int nx = static_cast<int>(x) + dx * step;
					const int ny = static_cast<int>(y) + dy * step;
					if (!inside(nx, ny, w, h))
						continue;
					const std::size_t index = static_cast<std::size_t>(ny) * w + nx;
					if (get_bit(in_component, index))
						continue;
					if (std::isfinite(heights[ny][nx]))
						lowest = std::min(lowest, heights[ny][nx]);
				}
				if (!std::isfinite(lowest))
					continue;
				++edge_cells;
				perched += lowest < here - STEEP_WATER_LAND_BELOW_M;
			}
			for (const auto &[x, y] : component)
				clear_bit(in_component, y * w + x);
			if (!edge_cells ||
					static_cast<double>(perched) < MIN_PERCHED_FRACTION * edge_cells)
				continue;
			++dropped_components;
			dropped.insert(dropped.end(), component.begin(), component.end());
		}
	std::vector<std::tuple<std::size_t, std::size_t, std::uint8_t>> replacements;
	replacements.reserve(dropped.size());
	for (const auto &[x, y] : dropped)
		replacements.emplace_back(x, y,
				nearest_non_water_class(cover, x, y, SEARCH_RADIUS)
						.value_or(land_cover::LC_BARE));
	for (const auto &[x, y, value] : replacements)
		cover[y][x] = value;
	if (dropped_components)
		std::clog << "Land cover repair: dropped " << dropped_components
				  << " water blob(s) (" << replacements.size()
				  << " cells) sitting on steep terrain (ESA shadow misclassification)\n";
	return replacements.size();
}

MaskGrid level_water_surfaces(
		HeightGrid &heights, const CoverGrid &cover, double meters_per_cell)
{
	constexpr int LOCAL_SURFACE_RADIUS = 12;
	constexpr double FLOWING_IQR_THRESHOLD = 5.0;
	constexpr double FLOW_SMOOTH_SIGMA_M = 40.0;
	constexpr double FLOW_SMOOTH_MAX_M = 1.5;
	constexpr std::size_t MIN_MODE_SAMPLES = 16;
	if (heights.empty() || heights.front().empty())
		return {};
	const std::size_t h = heights.size(), w = heights.front().size();
	if (cover.size() != h || cover.front().size() != w)
		return {};
	const std::size_t cell_count = w * h;
	std::vector<std::uint64_t> visited((cell_count + 63) / 64);
	std::vector<std::uint64_t> in_body((cell_count + 63) / 64);
	std::vector<std::uint64_t> marks((cell_count + 63) / 64);
	std::vector<std::uint64_t> walls_mask((cell_count + 63) / 64);
	MaskGrid surface(h, std::vector<std::uint8_t>(w));
	std::vector<std::vector<FlowingWaterCell>> flowing_components;
	std::optional<std::vector<std::uint8_t>> bank_dist;
	const auto wall_step = slope_step(meters_per_cell);
	const auto fall_reach = static_cast<std::uint16_t>(
			std::clamp(std::isfinite(meters_per_cell) && meters_per_cell > 0.0
							   ? std::round(FALL_REACH_M / meters_per_cell)
							   : 1.0,
					1.0, 65535.0));
	const double level_run =
			std::max(2.0, meters_per_cell > 0.0 && std::isfinite(meters_per_cell)
								  ? LEVEL_RUN_M / meters_per_cell
								  : 2.0);
	const auto flowing_surfaces = [&](const std::vector<Cell> &cells,
										  std::vector<std::uint64_t> &members,
										  double fallback, bool reach) {
		if (!bank_dist)
			bank_dist = bank_distances(cover, meters_per_cell);
		std::optional<OtherWaterSteps> from_body;
		if (reach)
			from_body = steps_from_other_water(cells, members, cover, fall_reach);
		std::vector<std::uint8_t> falls(cells.size());
		const auto calculate_falls = [&](std::size_t i) {
			const auto [x, y] = cells[i];
			const auto walk = walk_from_bank(*bank_dist, members, w, x, y, wall_step);
			falls[i] = on_channel_wall(heights, walk, x, y, meters_per_cell) &&
					   (!from_body || !falls_away(*from_body, walk, w, x, y));
		};
		if (cells.size() >= 4096)
			parallel_rows(0, cells.size(), calculate_falls);
		else
			for (std::size_t i = 0; i < cells.size(); ++i)
				calculate_falls(i);
		for (std::size_t i = 0; i < cells.size(); ++i) {
			const auto [x, y] = cells[i];
			if (falls[i])
				set_bit(walls_mask, y * w + x);
		}
		std::vector<std::uint8_t> wall_flags(cells.size());
		const auto classify_walls = [&](std::size_t i) {
			if (!falls[i])
				return;
			const auto [x, y] = cells[i];
			wall_flags[i] = !under_level_water(
					heights, cover, *bank_dist, walls_mask, x, y, level_run);
		};
		if (cells.size() >= 4096)
			parallel_rows(0, cells.size(), classify_walls);
		else
			for (std::size_t i = 0; i < cells.size(); ++i)
				classify_walls(i);
		for (std::size_t i = 0; i < cells.size(); ++i) {
			if (falls[i] && !wall_flags[i]) {
				const auto [x, y] = cells[i];
				clear_bit(walls_mask, y * w + x);
			}
		}
		std::vector<bool> walls(cells.size());
		for (std::size_t i = 0; i < cells.size(); ++i)
			walls[i] = wall_flags[i] != 0;
		keep_anchored_walls(walls_mask, cover, cells, walls);
		std::size_t wall_count = 0;
		for (std::size_t i = 0; i < cells.size(); ++i) {
			if (!walls[i])
				continue;
			const auto [x, y] = cells[i];
			clear_bit(members, y * w + x);
			++wall_count;
		}
		std::vector<FlowingWaterCell> local_surfaces(cells.size());
		std::vector<std::uint8_t> sampled(cells.size());
		const auto sample_surface = [&](std::size_t i) {
			if (walls[i])
				return;
			const auto [x, y] = cells[i];
			if (!std::isfinite(heights[y][x]))
				return;
			const auto local = reach ? local_reach_median(heights, members, cover, x, y,
											   LOCAL_SURFACE_RADIUS, 8)
									 : local_water_median(heights, members, x, y,
											   LOCAL_SURFACE_RADIUS, 8);
			local_surfaces[i] = FlowingWaterCell{static_cast<std::uint32_t>(x),
					static_cast<std::uint32_t>(y),
					static_cast<float>(local.value_or(fallback))};
			sampled[i] = 1;
		};
		if (cells.size() >= 4096)
			parallel_rows(0, cells.size(), sample_surface);
		else
			for (std::size_t i = 0; i < cells.size(); ++i)
				sample_surface(i);
		std::size_t sampled_count = 0;
		for (std::size_t i = 0; i < sampled.size(); ++i) {
			if (!sampled[i])
				continue;
			if (sampled_count != i)
				local_surfaces[sampled_count] = local_surfaces[i];
			++sampled_count;
		}
		local_surfaces.resize(sampled_count);
		return std::pair{std::move(local_surfaces), wall_count};
	};
	std::size_t components_leveled = 0, still_components = 0, flowing_count = 0;
	std::size_t split_reaches = 0, cells_leveled = 0, cells_skipped = 0;
	double max_flowing_iqr = 0.0;
	for (std::size_t sy = 0; sy < h; ++sy)
		for (std::size_t sx = 0; sx < w; ++sx) {
			const auto start = sy * w + sx;
			if (get_bit(visited, start) || cover[sy][sx] != land_cover::LC_WATER)
				continue;
			std::vector<Cell> component;
			std::deque<Cell> queue{{sx, sy}};
			std::vector<double> values;
			set_bit(visited, start);
			while (!queue.empty()) {
				const auto [x, y] = queue.front();
				queue.pop_front();
				component.emplace_back(x, y);
				if (std::isfinite(heights[y][x]))
					values.push_back(heights[y][x]);
				for (const auto &[dx, dy] : CARDINAL) {
					const int nx = static_cast<int>(x) + dx;
					const int ny = static_cast<int>(y) + dy;
					if (inside(nx, ny, w, h) &&
							!get_bit(visited, std::size_t(ny) * w + nx) &&
							cover[ny][nx] == land_cover::LC_WATER) {
						set_bit(visited, std::size_t(ny) * w + nx);
						queue.emplace_back(static_cast<std::size_t>(nx),
								static_cast<std::size_t>(ny));
					}
				}
			}
			if (values.empty())
				continue;
			const double iqr = interquartile_range(values);
			const double fallback = median(values);
			const double raw_surface =
					iqr <= FLOWING_IQR_THRESHOLD && values.size() >= MIN_MODE_SAMPLES
							? histogram_mode(values, 1.0)
							: fallback;
			std::vector<double>().swap(values);
			std::vector<Cell> body = std::move(component);
			for (const auto &[x, y] : body)
				set_bit(in_body, y * w + x);
			if (iqr > FLOWING_IQR_THRESHOLD) {
				++flowing_count;
				max_flowing_iqr = std::max(max_flowing_iqr, iqr);
				auto [cells, walls] = flowing_surfaces(body, in_body, fallback, false);
				flowing_components.push_back(std::move(cells));
				cells_skipped += walls;
			} else {
				++still_components;
				auto other = split_off_other_levels(body, raw_surface, heights, cover,
						in_body, marks, meters_per_cell);
				if (!other.reaches.empty() || !other.shadow.empty())
					std::erase_if(body, [&](const Cell &cell) {
						return !get_bit(in_body, cell.second * w + cell.first);
					});
				const double level =
						clamp_by_adjacent_land(raw_surface, body, heights, cover);
				if (!other.reaches.empty()) {
					grow_reaches(other.reaches, level, raw_surface, heights, in_body);
					std::erase_if(body, [&](const Cell &cell) {
						return !get_bit(in_body, cell.second * w + cell.first);
					});
				}
				for (const auto &[x, y] : body) {
					const double original = heights[y][x];
					if (!std::isfinite(original))
						continue;
					if (original <= level + WATER_UP_TOLERANCE_M ||
							!has_non_water_neighbor(cover, x, y)) {
						heights[y][x] = level;
						surface[y][x] = 1;
						++cells_leveled;
					} else {
						++cells_skipped;
					}
				}
				cells_skipped += other.shadow.size();
				for (auto &reach : other.reaches) {
					++split_reaches;
					for (const auto &[x, y] : reach.cells)
						set_bit(marks, y * w + x);
					std::vector<double> reach_values;
					reach_values.reserve(reach.cells.size());
					for (const auto &[x, y] : reach.cells)
						reach_values.push_back(heights[y][x]);
					const double reach_median = median(reach_values);
					auto [cells, walls] =
							flowing_surfaces(reach.cells, marks, reach_median, true);
					flowing_components.push_back(std::move(cells));
					cells_skipped += walls;
					for (const auto &[x, y] : reach.cells)
						clear_bit(marks, y * w + x);
				}
			}
			for (const auto &[x, y] : body)
				clear_bit(in_body, y * w + x);
			++components_leveled;
		}
	visited.clear();
	visited.shrink_to_fit();
	in_body.clear();
	in_body.shrink_to_fit();
	marks.clear();
	marks.shrink_to_fit();
	bank_dist.reset();
	const double sigma = meters_per_cell > 0.0 && std::isfinite(meters_per_cell)
								 ? std::min(64.0, FLOW_SMOOTH_SIGMA_M / meters_per_cell)
								 : 0.0;
	for (const auto &component : flowing_components) {
		if (component.empty())
			continue;
		const auto smoothed = sigma >= 1.5 ? smooth_sparse_water_field(component, sigma)
										   : std::vector<double>{};
		for (std::size_t i = 0; i < component.size(); ++i) {
			const auto &cell = component[i];
			const auto x = static_cast<std::size_t>(cell.x);
			const auto y = static_cast<std::size_t>(cell.y);
			const double original = heights[y][x];
			const double local = cell.local_surface;
			const double blurred = smoothed.empty() ? local : smoothed[i];
			double level = local + std::clamp(blurred - local, -FLOW_SMOOTH_MAX_M,
										   FLOW_SMOOTH_MAX_M);
			if (const auto land = lowest_adjacent_land(heights, cover, x, y))
				level = std::min(level, std::max(original, *land));
			if (original <= level + WATER_UP_TOLERANCE_M ||
					(!has_non_water_neighbor(cover, x, y) &&
							!next_to(walls_mask, w, h, x, y))) {
				heights[y][x] = level;
				surface[y][x] = 1;
				++cells_leveled;
			} else {
				++cells_skipped;
			}
		}
	}
	walls_mask.clear();
	if (components_leveled > 0) {
		std::clog << "Land cover repair: leveled " << components_leveled
				  << " water component(s)";
		if (flowing_count)
			std::clog << " (" << still_components << " still, " << flowing_count
					  << " flowing, max IQR " << max_flowing_iqr << "m)";
		std::clog << ", " << cells_leveled << " surface cells flattened, "
				  << cells_skipped << " off-surface cells kept as terrain\n";
		if (split_reaches)
			std::clog << "Land cover repair: leveled " << split_reaches
					  << " reach(es) below or above a waterfall at their own level\n";
	}
	return surface;
}

std::size_t reclassify_non_surface_water_cells(CoverGrid &cover, const MaskGrid &surface)
{
	std::vector<std::tuple<std::size_t, std::size_t, std::uint8_t>> replacements;
	for (std::size_t y = 0; y < cover.size(); ++y)
		for (std::size_t x = 0; x < cover[y].size(); ++x)
			if (cover[y][x] == land_cover::LC_WATER && !surface[y][x])
				replacements.emplace_back(x, y,
						nearest_non_water_class(cover, x, y, 8)
								.value_or(land_cover::LC_BARE));
	for (const auto &[x, y, value] : replacements)
		cover[y][x] = value;
	if (!replacements.empty())
		std::clog << "Land cover repair: reclassified " << replacements.size()
				  << " LC_WATER cells not on the water surface (embankments / piers / "
					 "shoreline walls)\n";
	return replacements.size();
}

void pull_coastal_land_toward_water(
		HeightGrid &heights, const MaskGrid &surface, std::uint32_t max_distance)
{
	if (!max_distance)
		return;
	const auto h = heights.size(), w = heights.front().size();
	std::vector<std::vector<std::uint32_t>> distance(
			h, std::vector<std::uint32_t>(w, std::numeric_limits<std::uint32_t>::max()));
	HeightGrid water_level(
			h, std::vector<double>(w, std::numeric_limits<double>::quiet_NaN()));
	std::deque<Cell> queue;
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x)
			if (surface[y][x]) {
				distance[y][x] = 0;
				water_level[y][x] = heights[y][x];
				queue.emplace_back(x, y);
			}
	while (!queue.empty()) {
		auto [x, y] = queue.front();
		queue.pop_front();
		if (distance[y][x] >= max_distance)
			continue;
		for (const auto &[dx, dy] : CARDINAL) {
			const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
			if (inside(nx, ny, w, h) && distance[y][x] + 1 < distance[ny][nx]) {
				distance[ny][nx] = distance[y][x] + 1;
				water_level[ny][nx] = water_level[y][x];
				queue.emplace_back(nx, ny);
			}
		}
	}
	std::size_t affected = 0;
	std::size_t skipped_cliff = 0;
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x) {
			const auto d = distance[y][x];
			const double water = water_level[y][x], original = heights[y][x];
			if (!d || d > max_distance || !std::isfinite(water) ||
					!std::isfinite(original))
				continue;
			if (original - water > 15.0) {
				++skipped_cliff;
				continue;
			}
			const double weight = static_cast<double>(max_distance - d) / max_distance;
			heights[y][x] = original * (1.0 - weight) + water * weight;
			++affected;
		}
	if (affected || skipped_cliff)
		std::clog << "Land cover repair: pulled " << affected
				  << " coastal land cells toward water (within " << max_distance
				  << " cells); kept " << skipped_cliff
				  << " cells above 15 m as real cliffs\n";
}

std::vector<std::vector<float>> blur_cover_mask_to_float(const CoverGrid &cover,
		std::uint8_t target, double sigma, const std::function<void(double)> &report)
{
	const std::size_t h = cover.size();
	if (h == 0)
		return {};
	const std::size_t w = cover.front().size();
	if (w == 0)
		return std::vector<std::vector<float>>(h);
	const int radius = static_cast<int>(std::ceil(sigma * 3.0));
	std::vector<double> kernel(static_cast<std::size_t>(radius * 2 + 1));
	double kernel_sum = 0.0;
	for (int dx = -radius; dx <= radius; ++dx) {
		const double d = static_cast<double>(dx);
		const double weight = std::exp(-(d * d) / (2.0 * sigma * sigma));
		kernel[static_cast<std::size_t>(dx + radius)] = weight;
		kernel_sum += weight;
	}
	for (auto &weight : kernel)
		weight /= kernel_sum;

	// Preserve f64 accumulation while avoiding the full f64 input mask and
	// full f64 feathered result that dominated the old city's peak allocation.
	std::vector<std::vector<double>> horizontal(h, std::vector<double>(w));
	constexpr std::size_t BLUR_CHUNKS = 10;
	const auto row_chunk = (h + BLUR_CHUNKS - 1) / BLUR_CHUNKS;
	for (std::size_t y0 = 0; y0 < h; y0 += row_chunk) {
		const auto y1 = std::min(h, y0 + row_chunk);
		parallel_rows(y0, y1, [&](std::size_t y) {
			std::vector<double> values(w);
			for (std::size_t x = 0; x < w; ++x)
				values[x] = cover[y][x] == target ? 1.0 : 0.0;
			horizontal[y] = blur_values(values, kernel, radius, true);
		});
		if (report)
			report(0.5 * static_cast<double>(y1) / h);
	}

	std::vector<std::vector<float>> output(h, std::vector<float>(w));
	const auto col_chunk = (w + BLUR_CHUNKS - 1) / BLUR_CHUNKS;
	// Match the Rust row-major grouped-column pass: each vertical convolution
	// keeps its original tap order, while gathering and storing use contiguous
	// runs across the row-major grid.
	for (std::size_t x0 = 0; x0 < w; x0 += col_chunk) {
		const auto x1 = std::min(w, x0 + col_chunk);
		const auto groups = blur_column_chunk(horizontal, x0, x1, kernel, radius, true);
		store_column_chunk(
				output, groups, [](double value) { return static_cast<float>(value); });
		if (report)
			report(0.5 + 0.5 * static_cast<double>(x1) / w);
	}
	return output;
}

// Separable blur of the elevation grid, treating real water-surface cells as
// missing samples. Keep the horizontal pass in f64 and reuse that grid for the
// vertical pass, matching Rust's masked blur without allocating another full
// height-sized intermediate.
HeightGrid gaussian_blur_grid_masked(const HeightGrid &heights, const MaskGrid &masked,
		double sigma, const std::function<void(double)> &report)
{
	const std::size_t h = std::min(heights.size(), masked.size());
	if (h == 0)
		return {};
	const std::size_t w = std::min(heights.front().size(), masked.front().size());
	if (w == 0)
		return HeightGrid(h);

	const int radius = static_cast<int>(std::ceil(sigma * 3.0));
	std::vector<double> kernel(static_cast<std::size_t>(radius * 2 + 1));
	double kernel_sum = 0.0;
	for (int k = -radius; k <= radius; ++k) {
		const double d = static_cast<double>(k);
		const double weight = std::exp(-(d * d) / (2.0 * sigma * sigma));
		kernel[static_cast<std::size_t>(k + radius)] = weight;
		kernel_sum += weight;
	}
	for (auto &weight : kernel)
		weight /= kernel_sum;

	constexpr std::size_t BLUR_CHUNKS = 10;
	const auto row_chunk = std::max<std::size_t>(1, (h + BLUR_CHUNKS - 1) / BLUR_CHUNKS);
	HeightGrid after_h(h, std::vector<double>(w));
	for (std::size_t y0 = 0; y0 < h; y0 += row_chunk) {
		const auto y1 = std::min(h, y0 + row_chunk);
		parallel_rows(y0, y1, [&](std::size_t y) {
			const auto len = std::min(w, std::min(heights[y].size(), masked[y].size()));
			std::vector<double> values(len);
			for (std::size_t x = 0; x < len; ++x)
				values[x] = masked[y][x] ? std::numeric_limits<double>::quiet_NaN()
										 : heights[y][x];
			const auto blurred = blur_values(values, kernel, radius, false);
			std::copy(blurred.begin(), blurred.end(), after_h[y].begin());
		});
		if (report)
			report(0.5 * static_cast<double>(y1) / h);
	}

	// Gather and compute row-major column groups, then write each row in
	// contiguous spans without allocating a second full-sized output grid.
	const auto col_chunk = std::max<std::size_t>(1, (w + BLUR_CHUNKS - 1) / BLUR_CHUNKS);
	for (std::size_t x0 = 0; x0 < w; x0 += col_chunk) {
		const auto x1 = std::min(w, x0 + col_chunk);
		const auto groups = blur_column_chunk(after_h, x0, x1, kernel, radius, false);
		store_column_chunk(after_h, groups, [](double value) { return value; });
		if (report)
			report(0.5 + 0.5 * static_cast<double>(x1) / w);
	}
	return after_h;
}

void smooth_built_up_gaussian(HeightGrid &heights, const CoverGrid &cover,
		const MaskGrid &surface, double sigma, const std::function<void(double)> &report)
{
	if (sigma < 1.5)
		return;
	const auto h = heights.size(), w = heights.front().size();
	const bool any = std::any_of(cover.begin(), cover.end(), [](const auto &row) {
		return std::find(row.begin(), row.end(), land_cover::LC_BUILT_UP) != row.end();
	});
	if (!any)
		return;
	auto feathered = blur_cover_mask_to_float(
			cover, land_cover::LC_BUILT_UP, sigma, [&](double fraction) {
				if (report)
					report(0.5 * fraction);
			});
	// Synthesize masked NaNs while reading each row, as Rust does; avoiding a
	// full-sized copy of the elevation grid lowers the peak during city smoothing.
	auto blurred =
			gaussian_blur_grid_masked(heights, surface, sigma, [&](double fraction) {
				if (report)
					report(0.5 + 0.5 * fraction);
			});
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x) {
			if (surface[y][x])
				continue;
			const double weight =
					static_cast<double>(std::clamp(feathered[y][x], 0.0f, 1.0f));
			if (weight > 1e-4 && std::isfinite(heights[y][x]) &&
					std::isfinite(blurred[y][x]))
				heights[y][x] = heights[y][x] * (1.0 - weight) + blurred[y][x] * weight;
		}
	if (report)
		report(1.0);
}
}

void apply_land_cover_repair(HeightGrid &heights, land_cover::LandCoverData &data,
		double built_up_sigma_cells, std::uint32_t coastal_pull_distance_cells,
		double meters_per_cell, const std::function<void(double)> &report)
{
	if (heights.empty() || heights.front().empty())
		return;
	if (data.width != heights.front().size() || data.height != heights.size() ||
			data.grid.size() != data.height) {
		std::clog << "Warning: land cover grid (" << data.width << 'x' << data.height
				  << ") does not match elevation grid ("
				  << (heights.empty() ? 0 : heights.front().size()) << 'x'
				  << heights.size() << "); skipping land-cover-aware repair\n";
		return;
	}
	const auto dropped = drop_water_on_steep_terrain(heights, data.grid, meters_per_cell);
	auto surface = level_water_surfaces(heights, data.grid, meters_per_cell);
	const auto reclassified = reclassify_non_surface_water_cells(data.grid, surface);
	if (dropped + reclassified) {
		data.water_distance =
				land_cover::compute_water_distance(data.grid, data.width, data.height);
		data.invalidate_water_blend_grid();
	}
	smooth_built_up_gaussian(heights, data.grid, surface, built_up_sigma_cells, report);
	pull_coastal_land_toward_water(heights, surface, coastal_pull_distance_cells);
}

namespace
{
constexpr double LOWEST_LAND_M = -430.0;
constexpr double HIGHEST_LAND_M = 8849.0;
constexpr double SOFT_TOP_BLOCKS = 800.0;
constexpr double HEADROOM_MAX_BLOCKS = 96.0;
constexpr int TERRAIN_HEIGHT_BUFFER = 15;

int terrain_ceiling(bool disable_height_limit, int extended_max_y)
{
	return (disable_height_limit ? extended_max_y : 319) - TERRAIN_HEIGHT_BUFFER;
}

std::pair<int, double> fit_geometry(double height_range, double scale, int ceiling,
		int ground_level, int min_ground_level, bool disable_height_limit)
{
	const double ideal_scaled_range = height_range * scale;
	// Sink only as far as the relief requires. An explicit ground level is
	// preserved unless the extended-height floor is active.
	if (disable_height_limit && min_ground_level < ground_level &&
			std::isfinite(ideal_scaled_range)) {
		const double rounded = std::ceil(ideal_scaled_range);
		const int needed = rounded >= std::numeric_limits<int>::max()
								   ? std::numeric_limits<int>::max()
						   : rounded <= std::numeric_limits<int>::min()
								   ? std::numeric_limits<int>::min()
								   : static_cast<int>(rounded);
		const long long candidate = static_cast<long long>(ceiling) - needed;
		ground_level = static_cast<int>(
				std::clamp(candidate, static_cast<long long>(min_ground_level),
						static_cast<long long>(ground_level)));
	}
	const double available_y_range = static_cast<double>(ceiling - ground_level);
	const double scaled_range =
			ideal_scaled_range <= available_y_range
					? ideal_scaled_range
					: height_range * (available_y_range / height_range);
	return {ground_level, scaled_range};
}

double headroom_margin(int ceiling, int ground_level, double scaled_range)
{
	const double free = static_cast<double>(ceiling - ground_level) - scaled_range;
	return std::max(0.0, std::floor(std::min(free * 0.5, HEADROOM_MAX_BLOCKS)));
}

struct FitRange
{
	double height_range;
	double scaled_range;
};

std::pair<ElevationAffine, FitRange> derive_affine(
		const std::vector<std::vector<double>> &grid, double scale, int ground_level,
		int min_ground_level, bool disable_height_limit, int extended_max_y,
		int top_reserve = 0)
{
	std::vector<std::pair<double, double>> row_ranges(grid.size(),
			{std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()});
	parallel_rows(0, grid.size(), [&](std::size_t y) {
		auto &[row_lo, row_hi] = row_ranges[y];
		for (double value : grid[y])
			if (std::isfinite(value)) {
				row_lo = std::min(row_lo, value);
				row_hi = std::max(row_hi, value);
			}
	});
	double lo = std::numeric_limits<double>::max();
	double hi = std::numeric_limits<double>::lowest();
	for (const auto &[row_lo, row_hi] : row_ranges) {
		lo = std::min(lo, row_lo);
		hi = std::max(hi, row_hi);
	}
	double height_range;
	if (!std::isfinite(lo) || !std::isfinite(hi) || lo >= hi) {
		const double real_min = std::isfinite(lo) && lo <= hi ? lo : 0.0;
		lo = real_min;
		height_range = 0.0;
	} else {
		height_range = hi - lo;
	}
	const double ideal_scaled_range = height_range * scale;
	const int ceiling = terrain_ceiling(disable_height_limit, extended_max_y) -
						std::max(0, top_reserve);
	const auto [fitted_ground_level, scaled_range] = fit_geometry(height_range, scale,
			ceiling, ground_level, min_ground_level, disable_height_limit);
	ground_level = fitted_ground_level;
	const double available_y_range = static_cast<double>(ceiling - ground_level);
	if (ideal_scaled_range <= available_y_range) {
		std::ostringstream message;
		message << "Realistic elevation: " << std::fixed << std::setprecision(1)
				<< height_range << "m range fits in " << std::setprecision(0)
				<< available_y_range << " available blocks\n";
		std::clog << message.str();
	} else {
		std::ostringstream message;
		message << "Elevation compressed: " << std::fixed << std::setprecision(1)
				<< height_range << "m range -> " << std::setprecision(0) << scaled_range
				<< " blocks (" << std::setprecision(2) << height_range / scaled_range
				<< ":1 ratio, 1 block = " << scaled_range / height_range << "m)\n";
		std::clog << message.str();
	}
	const double blocks_per_meter =
			height_range > 0.0 ? scaled_range / height_range : 0.0;
	return {{lo, blocks_per_meter, ground_level, std::nullopt},
			{height_range, scaled_range}};
}
} // namespace

double ElevationAffine::y_for_metres(double height_m) const
{
	if (soft_top && height_m > soft_top->knee_m) {
		const double knee_y =
				ground_level + (soft_top->knee_m - min_height_m) * blocks_per_meter;
		const double rise = (height_m - soft_top->knee_m) * blocks_per_meter;
		return knee_y +
			   soft_top->width_blocks * std::asinh(rise / soft_top->width_blocks);
	}
	return ground_level + (height_m - min_height_m) * blocks_per_meter;
}

ElevationAffine ElevationAffine::whole_earth(double scale, int floor, int extended_max_y)
{
	ElevationAffine affine{LOWEST_LAND_M, scale, floor, std::nullopt};
	const double ceiling = terrain_ceiling(true, extended_max_y);
	const double knee_y = ceiling - SOFT_TOP_BLOCKS;
	if (affine.y_for_metres(HIGHEST_LAND_M) <= ceiling || knee_y <= floor)
		return affine;
	const double knee_m = LOWEST_LAND_M + (knee_y - floor) / scale;
	const double rise = (HIGHEST_LAND_M - knee_m) * scale;
	double lo = 1e-3, hi = 1e9;
	for (int i = 0; i < 200; ++i) {
		const double mid = (lo + hi) * 0.5;
		if (mid * std::asinh(rise / mid) > SOFT_TOP_BLOCKS)
			hi = mid;
		else
			lo = mid;
	}
	affine.soft_top = SoftTop{knee_m, lo};
	return affine;
}

std::pair<std::vector<std::vector<double>>, ElevationAffine> scale_to_minecraft_with(
		const std::vector<std::vector<double>> &in, double scale, int ground_level,
		int min_ground_level, bool disable_height_limit, int extended_max_y,
		const AffinePolicy &policy,
		const std::function<void(std::string_view)> &message_callback, int top_reserve)
{
	const int ceiling = terrain_ceiling(disable_height_limit, extended_max_y);
	const int fit_ceiling = ceiling - std::max(0, top_reserve);
	std::optional<FitRange> fit;
	ElevationAffine affine;
	switch (policy.kind) {
	case AffinePolicy::Kind::Fit: {
		auto derived = derive_affine(in, scale, ground_level, min_ground_level,
				disable_height_limit, extended_max_y, top_reserve);
		affine = derived.first;
		fit = derived.second;
		break;
	}
	case AffinePolicy::Kind::FitWithHeadroom: {
		auto derived = derive_affine(in, scale, ground_level, min_ground_level,
				disable_height_limit, extended_max_y, top_reserve);
		affine = derived.first;
		fit = derived.second;
		if (affine.blocks_per_meter <= 0.0)
			affine.blocks_per_meter = scale;
		const double margin =
				headroom_margin(fit_ceiling, affine.ground_level, fit->scaled_range);
		if (margin > 0.0 && affine.blocks_per_meter > 0.0) {
			affine.min_height_m -= margin / affine.blocks_per_meter;
			std::clog << "One World: keeping " << static_cast<int>(margin)
					  << " blocks below the lowest terrain for neighbouring areas\n";
		}
		fit.reset();
		break;
	}
	case AffinePolicy::Kind::Fixed: {
		affine = policy.fixed;
		std::ostringstream message;
		message << "One World: using the world's elevation mapping (1 block = "
				<< std::fixed << std::setprecision(2)
				<< (affine.blocks_per_meter > 0.0
								   ? 1.0 / affine.blocks_per_meter
								   : std::numeric_limits<double>::infinity())
				<< " m, " << std::setprecision(0) << affine.min_height_m << " m at Y "
				<< affine.ground_level;
		if (affine.soft_top)
			message << ", compressed above " << affine.soft_top->knee_m << " m";
		message << ")\n";
		std::clog << message.str();
		break;
	}
	}
	const double base = affine.ground_level;
	const double upper = ceiling;
	std::vector<std::vector<double>> out = in;
	// Match Rust's Rayon row mapping. Elevation grids can contain millions of
	// cells, and each output row is independent of every other row.
	parallel_rows(0, in.size(), [&](std::size_t y) {
		for (std::size_t x = 0; x < in[y].size(); ++x) {
			const double value = in[y][x];
			double mapped;
			if (fit) {
				const double relative =
						fit->height_range > 0.0
								? (value - affine.min_height_m) / fit->height_range
								: 0.0;
				mapped = base + relative * fit->scaled_range;
			} else {
				mapped = affine.y_for_metres(value);
			}
			out[y][x] = std::clamp(mapped, base, upper);
		}
	});
	if (policy.kind == AffinePolicy::Kind::Fixed) {
		std::vector<std::pair<std::size_t, std::size_t>> row_counts(in.size());
		parallel_rows(0, in.size(), [&](std::size_t y) {
			auto &[row_clamped, row_total] = row_counts[y];
			for (double value : in[y])
				if (std::isfinite(value)) {
					++row_total;
					const double mapped = affine.y_for_metres(value);
					row_clamped +=
							(mapped < base && value > LOWEST_LAND_M) || mapped > upper;
				}
		});
		std::size_t total = 0, clamped = 0;
		for (const auto &[row_clamped, row_total] : row_counts) {
			total += row_total;
			clamped += row_clamped;
		}
		if (total && clamped * 200 > total) {
			const double percentage = 100.0 * clamped / total;
			std::ostringstream message;
			message << "Warning: " << std::fixed << std::setprecision(1) << percentage
					<< "% of this area lies outside the world's height band (Y "
					<< affine.ground_level << " to " << static_cast<int>(upper)
					<< ") and is flattened there."
					<< (disable_height_limit
									   ? ""
									   : " The band was fixed by the first area; a new One World has room for all of Earth.")
					<< '\n';
			std::clog << message.str();
			if (message_callback) {
				std::ostringstream status;
				status << "Note: " << std::fixed << std::setprecision(0) << percentage
					   << "% of the area exceeds the world's height band";
				message_callback(status.str());
			}
		}
	}
	double top = base;
	if (fit)
		top = affine.ground_level + fit->scaled_range;
	else {
		std::vector<double> row_tops(out.size(), std::numeric_limits<double>::lowest());
		parallel_rows(0, out.size(), [&](std::size_t y) {
			for (double value : out[y])
				if (std::isfinite(value))
					row_tops[y] = std::max(row_tops[y], value);
		});
		for (double row_top : row_tops)
			top = std::max(top, row_top);
	}
	world_editor::set_terrain_top_y(static_cast<int>(std::lround(std::min(top, upper))));
	return {std::move(out), affine};
}

std::pair<std::vector<std::vector<double>>, ElevationAffine> scale_to_minecraft_reserving(
		const std::vector<std::vector<double>> &in, double scale, int ground_level,
		int min_ground_level, bool disable_height_limit, int extended_max_y,
		const AffinePolicy &policy, int top_reserve,
		const std::function<void(std::string_view)> &message_callback)
{
	return scale_to_minecraft_with(in, scale, ground_level, min_ground_level,
			disable_height_limit, extended_max_y, policy, message_callback, top_reserve);
}

int top_reserve_for(const std::vector<std::vector<double>> &heights,
		const std::vector<std::pair<double, double>> &models, double scale,
		double model_scale, int ground_level, int min_ground_level,
		bool disable_height_limit, int extended_max_y, bool headroom)
{
	if (models.empty())
		return 0;
	double lo = std::numeric_limits<double>::max();
	double hi = std::numeric_limits<double>::lowest();
	for (const auto &row : heights)
		for (const double h : row)
			if (std::isfinite(h)) {
				lo = std::min(lo, h);
				hi = std::max(hi, h);
			}
	if (lo > hi)
		return 0;
	const double range = hi - lo;
	const int ceiling = terrain_ceiling(disable_height_limit, extended_max_y);
	const double limit = static_cast<double>(ceiling + TERRAIN_HEIGHT_BUFFER - 2);
	auto model_top = [&](int reserve) {
		const int top = ceiling - reserve;
		const auto [base, scaled] = fit_geometry(
				range, scale, top, ground_level, min_ground_level, disable_height_limit);
		const double bpm = range > 0.0 ? scaled / range : 0.0;
		const double lift = headroom ? headroom_margin(top, base, scaled) : 0.0;
		double max_top = std::numeric_limits<double>::lowest();
		for (const auto &[ground_m, height_m] : models)
			max_top = std::max(max_top,
					base + lift + (ground_m - lo) * bpm + height_m * model_scale);
		return max_top;
	};
	if (model_top(0) <= limit)
		return 0;
	const int cap = std::max(ceiling - ground_level, 0) / 2;
	if (model_top(cap) > limit)
		return cap;
	int fits = cap, short_of_fit = 0;
	while (fits - short_of_fit > 1) {
		const int mid = short_of_fit + (fits - short_of_fit) / 2;
		if (model_top(mid) <= limit)
			fits = mid;
		else
			short_of_fit = mid;
	}
	return fits;
}

std::tuple<std::vector<std::vector<double>>, double, double, int> scale_to_minecraft(
		const std::vector<std::vector<double>> &in, double scale, int ground_level,
		int min_ground_level, bool disable_height_limit, int extended_max_y)
{
	auto [heights, affine] = scale_to_minecraft_with(in, scale, ground_level,
			min_ground_level, disable_height_limit, extended_max_y, AffinePolicy::fit());
	return {std::move(heights), affine.min_height_m, affine.blocks_per_meter,
			affine.ground_level};
}
}
