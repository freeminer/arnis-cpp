#include <vector>
#include <chrono>
#include <algorithm>
#include <bit>
#include <cstdint>
#include <deque>
#include <optional>
#include <tuple>
#include <cmath>

#include "floodfill.h"

namespace arnis
{

namespace floodfill
{

// Rust parity: floodfill.rs::scanline_fill_area.  The bitmap path is bounded
// by its full bounding box, while this path is bounded by actual filled cells.
constexpr std::int64_t MAX_SCANLINE_EDGE_TESTS = 200'000'000;

// Check if the current time exceeds the timeout duration
inline bool timeout_exceeded(const std::chrono::steady_clock::time_point &start_time,
		const std::chrono::milliseconds &timeout_duration)
{
	// Compare the full elapsed duration. Rust compares Instant::elapsed() directly
	// with Duration, so truncating to whole milliseconds here can let a fill run
	// almost one extra millisecond past its configured limit.
	return std::chrono::steady_clock::now() - start_time > timeout_duration;
}

// Even-odd scanline fill for closed polygons. It leaves boundary cells to the
// caller's outline pass, like geo::Contains.
static std::vector<std::pair<int, int>> scanline_fill_area(
		const std::vector<std::pair<int, int>> &polygon_coords, int32_t min_x,
		int32_t max_x, int32_t min_z, int32_t max_z)
{
	const std::int64_t rows = static_cast<std::int64_t>(max_z) - min_z + 1;
	const std::int64_t edges = static_cast<std::int64_t>(polygon_coords.size()) - 1;
	if (rows <= 0 || edges <= 0 || rows > MAX_SCANLINE_EDGE_TESTS / edges)
		return {};

	struct Span
	{
		int32_t z, first, last;
	};
	std::vector<Span> spans;
	std::vector<double> crossings;
	std::int64_t cells = 0;
	for (std::int64_t zi = min_z; zi <= max_z; ++zi) {
		const auto z = static_cast<int32_t>(zi);
		const double zf = static_cast<double>(z);
		crossings.clear();
		for (std::size_t i = 1; i < polygon_coords.size(); ++i) {
			const auto &[ix0, iz0] = polygon_coords[i - 1];
			const auto &[ix1, iz1] = polygon_coords[i];
			const double x0 = static_cast<double>(ix0), z0 = static_cast<double>(iz0);
			const double x1 = static_cast<double>(ix1), z1 = static_cast<double>(iz1);
			// Half-open in z: a vertex on the row contributes once, not twice.
			if ((z0 <= zf) == (z1 <= zf))
				continue;
			const double t = (zf - z0) / (z1 - z0);
			crossings.push_back(x0 + t * (x1 - x0));
		}
		if (crossings.size() < 2)
			continue;
		std::sort(crossings.begin(), crossings.end());
		auto append_span = [&](std::int64_t first, std::int64_t last) {
			if (last < first)
				return true;
			cells += last - first + 1;
			if (cells > MAX_FLOOD_FILL_AREA)
				return false;
			spans.push_back({z, static_cast<int32_t>(first), static_cast<int32_t>(last)});
			return true;
		};
		for (std::size_t i = 1; i < crossings.size(); i += 2) {
			const auto raw_first =
					static_cast<std::int64_t>(std::floor(crossings[i - 1])) + 1;
			const auto raw_last = static_cast<std::int64_t>(std::ceil(crossings[i])) - 1;
			const auto first_clamped = std::clamp<std::int64_t>(raw_first, min_x, max_x);
			const auto last_clamped = std::clamp<std::int64_t>(raw_last, min_x, max_x);
			if (raw_last < min_x || raw_first > max_x || last_clamped < first_clamped)
				continue;

			if (!append_span(first_clamped, last_clamped))
				return {};
		}
	}

	std::vector<std::pair<int, int>> filled;
	filled.reserve(static_cast<std::size_t>(cells));
	for (const auto &[z, first, last] : spans) {
		for (std::int64_t xi = first; xi <= last; ++xi) {
			filled.emplace_back(static_cast<int>(xi), z);
		}
	}
	return filled;
}

// Bit-packed mask of exactly the integer cells considered inside by the Rust
// nonzero-winding scan conversion. It avoids re-testing every edge during BFS.
class InsideBitmap
{
	std::vector<std::uint64_t> bits_;
	int32_t min_x_, min_z_;
	std::size_t width_;

public:
	InsideBitmap(int32_t min_x, int32_t max_x, int32_t min_z, int32_t max_z) :
			min_x_(min_x), min_z_(min_z),
			width_(static_cast<std::size_t>(static_cast<std::int64_t>(max_x) - min_x + 1))
	{
		const auto height =
				static_cast<std::size_t>(static_cast<std::int64_t>(max_z) - min_z + 1);
		bits_.resize((width_ * height + 63) / 64);
	}

	std::size_t index(int32_t x, int32_t z) const
	{
		return static_cast<std::size_t>(static_cast<std::int64_t>(z) - min_z_) * width_ +
			   static_cast<std::size_t>(static_cast<std::int64_t>(x) - min_x_);
	}

	bool take(std::size_t i)
	{
		auto &word = bits_[i / 64];
		const std::uint64_t mask = std::uint64_t{1} << (i % 64);
		const bool was_set = (word & mask) != 0;
		word &= ~mask;
		return was_set;
	}

	void unset(int32_t x, int32_t z)
	{
		const auto i = index(x, z);
		bits_[i / 64] &= ~(std::uint64_t{1} << (i % 64));
	}

	void write_span(int32_t z, int32_t x0, int32_t x1, bool value)
	{
		if (x1 < x0)
			return;
		const auto first = index(x0, z), last = index(x1, z);
		const auto first_word = first / 64, last_word = last / 64;
		const auto first_mask = UINT64_MAX << (first % 64);
		const auto last_mask = UINT64_MAX >> (63 - last % 64);
		auto set = [&](std::size_t word, std::uint64_t mask) {
			if (value)
				bits_[word] |= mask;
			else
				bits_[word] &= ~mask;
		};
		if (first_word == last_word) {
			set(first_word, first_mask & last_mask);
			return;
		}
		set(first_word, first_mask);
		for (auto word = first_word + 1; word < last_word; ++word)
			set(word, UINT64_MAX);
		set(last_word, last_mask);
	}

	std::size_t count() const
	{
		std::size_t total = 0;
		for (const auto word : bits_)
			total += static_cast<std::size_t>(std::popcount(word));
		return total;
	}
};

struct ScanEdge
{
	int32_t z0, z1;
	std::int64_t x0, dx, dz;
	int winding;
};

static std::optional<InsideBitmap> inside_mask(
		const std::vector<std::pair<int, int>> &ring, int32_t min_x, int32_t max_x,
		int32_t min_z, int32_t max_z, const std::chrono::milliseconds *timeout,
		const std::chrono::steady_clock::time_point &start)
{
	InsideBitmap mask(min_x, max_x, min_z, max_z);
	std::vector<ScanEdge> edges;
	edges.reserve(ring.size());
	for (std::size_t i = 1; i < ring.size(); ++i) {
		const auto [ax, az] = ring[i - 1];
		const auto [bx, bz] = ring[i];
		if (az == bz)
			continue;
		const bool up = az < bz;
		const auto [x0, z0, x1, z1] =
				up ? std::tuple{ax, az, bx, bz} : std::tuple{bx, bz, ax, az};
		edges.push_back(
				{z0, z1, x0, std::int64_t(x1) - x0, std::int64_t(z1) - z0, up ? 1 : -1});
	}
	std::sort(edges.begin(), edges.end(),
			[](const auto &a, const auto &b) { return a.z0 < b.z0; });
	std::vector<ScanEdge> active;
	active.reserve(edges.size());
	std::size_t next_edge = 0;
	struct Crossing
	{
		std::int64_t first;
		int winding;
		bool on_cell;
	};
	std::vector<Crossing> crossings;
	crossings.reserve(edges.size());
	for (std::int64_t zi = min_z; zi <= max_z; ++zi) {
		const auto z = static_cast<int32_t>(zi);
		if (timeout && (zi - min_z) % 64 == 0 && timeout_exceeded(start, *timeout))
			return std::nullopt;
		active.erase(std::remove_if(active.begin(), active.end(),
							 [&](const auto &edge) { return edge.z1 <= z; }),
				active.end());
		while (next_edge < edges.size() && edges[next_edge].z0 == z)
			active.push_back(edges[next_edge++]);
		crossings.clear();
		for (const auto &edge : active) {
			const auto t = (std::int64_t(z) - edge.z0) * edge.dx;
			const auto quotient = t >= 0 ? t / edge.dz : -((-t + edge.dz - 1) / edge.dz);
			crossings.push_back({edge.x0 + quotient + 1, edge.winding, t % edge.dz == 0});
		}
		std::sort(crossings.begin(), crossings.end(),
				[](const auto &a, const auto &b) { return a.first < b.first; });
		int winding = 0;
		for (std::size_t i = 1; i < crossings.size(); ++i) {
			winding += crossings[i - 1].winding;
			if (winding == 0 || crossings[i - 1].first >= crossings[i].first)
				continue;
			const auto first = std::max<std::int64_t>(crossings[i - 1].first, min_x);
			const auto last = std::min<std::int64_t>(crossings[i].first - 1, max_x);
			if (last >= first)
				mask.write_span(
						z, static_cast<int32_t>(first), static_cast<int32_t>(last), true);
		}
		for (const auto &crossing : crossings)
			if (crossing.on_cell)
				mask.unset(static_cast<int32_t>(crossing.first - 1), z);
	}
	for (std::size_t i = 1; i < ring.size(); ++i) {
		const auto [ax, az] = ring[i - 1];
		const auto [bx, bz] = ring[i];
		if (az == bz)
			mask.write_span(az, std::min(ax, bx), std::max(ax, bx), false);
		else
			mask.unset(ax, az);
	}
	return mask;
}

static std::vector<std::pair<int, int>> seeded_inside_fill(
		const std::vector<std::pair<int, int>> &ring, int32_t min_x, int32_t max_x,
		int32_t min_z, int32_t max_z, const std::chrono::milliseconds *timeout,
		int step_divisor, int step_cap, bool check_timeout_each_seed)
{
	const auto start = std::chrono::steady_clock::now();
	auto remaining = inside_mask(ring, min_x, max_x, min_z, max_z, timeout, start);
	if (!remaining)
		return {};
	const auto row =
			static_cast<std::size_t>(static_cast<std::int64_t>(max_x) - min_x + 1);
	const int step_x = std::clamp((max_x - min_x + 1) / step_divisor, 1, step_cap);
	const int step_z = std::clamp((max_z - min_z + 1) / step_divisor, 1, step_cap);
	std::vector<std::pair<int, int>> filled;
	filled.reserve(remaining->count());
	std::deque<std::pair<int, int>> queue;
	for (std::int64_t zi = min_z; zi <= max_z; zi += step_z) {
		for (std::int64_t xi = min_x; xi <= max_x; xi += step_x) {
			if ((check_timeout_each_seed || filled.size() % 100 == 0) && timeout &&
					timeout_exceeded(start, *timeout))
				goto done;
			const auto x = static_cast<int32_t>(xi), z = static_cast<int32_t>(zi);
			if (!remaining->take(remaining->index(x, z)))
				continue;
			queue.clear();
			queue.emplace_back(x, z);
			while (!queue.empty()) {
				const auto [cx, cz] = queue.front();
				queue.pop_front();
				filled.emplace_back(cx, cz);
				const auto index = remaining->index(cx, cz);
				if (cx > min_x && remaining->take(index - 1))
					queue.emplace_back(cx - 1, cz);
				if (cx < max_x && remaining->take(index + 1))
					queue.emplace_back(cx + 1, cz);
				if (cz > min_z && remaining->take(index - row))
					queue.emplace_back(cx, cz - 1);
				if (cz < max_z && remaining->take(index + row))
					queue.emplace_back(cx, cz + 1);
			}
		}
	}
done:
	filled.shrink_to_fit();
	return filled;
}

/// Main flood fill function with automatic algorithm selection
/// Chooses the best algorithm based on polygon size and complexity
std::vector<std::pair<int, int>> flood_fill_area(
		const std::vector<std::pair<int, int>> &polygon_coords,
		const std::chrono::milliseconds *timeout)
{
	if (polygon_coords.size() < 3) {
		return {}; // Not a valid polygon
	}
	// Do not invent a closing edge for an open OSM way. Besides being more
	// accurate, this prevents long roads and cliffs from becoming huge fills.
	if (polygon_coords.front() != polygon_coords.back())
		return {};

	// Calculate bounding box of the polygon
	auto minmax_x = std::minmax_element(polygon_coords.begin(), polygon_coords.end(),
			[](const std::pair<int, int> &a, const std::pair<int, int> &b) {
				return a.first < b.first;
			});
	auto minmax_z = std::minmax_element(polygon_coords.begin(), polygon_coords.end(),
			[](const std::pair<int, int> &a, const std::pair<int, int> &b) {
				return a.second < b.second;
			});

	int32_t min_x = minmax_x.first->first;
	int32_t max_x = minmax_x.second->first;
	int32_t min_z = minmax_z.first->second;
	int32_t max_z = minmax_z.second->second;

	const std::int64_t width = static_cast<std::int64_t>(max_x) - min_x + 1;
	const std::int64_t height = static_cast<std::int64_t>(max_z) - min_z + 1;
	// Match Rust's strategy: keep bitmap flood fills for bounded rings, and use
	// scanline rasterization only when the visited bitmap would exceed its cap.
	if (width <= 0 || height <= 0 || width > MAX_FLOOD_FILL_AREA / height) {
		return scanline_fill_area(polygon_coords, min_x, max_x, min_z, max_z);
	}
	const std::int64_t area = width * height;
	if (area > MAX_FLOOD_FILL_AREA)
		return scanline_fill_area(polygon_coords, min_x, max_x, min_z, max_z);
	if (area < 50'000)
		return seeded_inside_fill(
				polygon_coords, min_x, max_x, min_z, max_z, timeout, 6, 8, false);
	return seeded_inside_fill(
			polygon_coords, min_x, max_x, min_z, max_z, timeout, 8, 12, true);
}

}
}
