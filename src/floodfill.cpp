#include <iostream>
#include <vector>
#include <queue>
#include <chrono>
#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <tuple>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <cmath>

#include "floodfill.h"

namespace arnis
{

namespace floodfill
{

namespace bg = boost::geometry;

// Type aliases for convenience
using Point2D = bg::model::d2::point_xy<int>;
using Point2D_Float = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point2D_Float>;

// Rust parity: floodfill.rs::scanline_fill_area.  The bitmap path is bounded
// by its full bounding box, while this path is bounded by actual filled cells.
constexpr std::int64_t MAX_SCANLINE_EDGE_TESTS = 200'000'000;

/// A compact bitmap for visited-coordinate tracking during flood fill.
///
/// Uses 1 bit per coordinate instead of ~48 bytes per entry in a `HashSet`.
/// For a 5000×5000 bounding box this is ~3 MB instead of ~1.2 GB.
class FloodBitmap
{
private:
	std::vector<uint8_t> bits;
	int32_t min_x;
	int32_t min_z;
	size_t width;

public:
	FloodBitmap(int32_t min_x, int32_t max_x, int32_t min_z, int32_t max_z) :
			min_x(min_x), min_z(min_z)
	{
		width = static_cast<size_t>(max_x - min_x + 1);
		size_t height = static_cast<size_t>(max_z - min_z + 1);
		size_t num_bytes = (width * height + 7) / 8; // Ceiling division
		bits.resize(num_bytes, 0);
	}

	/// Mark (x, z) as visited. Returns `true` if it was NOT already visited
	/// (i.e. this is the first visit).
	bool insert(int32_t x, int32_t z)
	{
		size_t idx =
				static_cast<size_t>(z - min_z) * width + static_cast<size_t>(x - min_x);
		size_t byte = idx / 8;
		size_t bit = idx % 8;
		uint8_t mask = static_cast<uint8_t>(1U << bit);
		if (bits[byte] & mask) {
			return false; // already visited
		} else {
			bits[byte] |= mask;
			return true;
		}
	}

	bool contains(int32_t x, int32_t z) const
	{
		size_t idx =
				static_cast<size_t>(z - min_z) * width + static_cast<size_t>(x - min_x);
		size_t byte = idx / 8;
		size_t bit = idx % 8;
		return (bits[byte] >> bit) & 1U;
	}
};

// Check if the current time exceeds the timeout duration
inline bool timeout_exceeded(const std::chrono::steady_clock::time_point &start_time,
		const std::chrono::milliseconds &timeout_duration)
{
	// Compare the full elapsed duration. Rust compares Instant::elapsed() directly
	// with Duration, so truncating to whole milliseconds here can let a fill run
	// almost one extra millisecond past its configured limit.
	return std::chrono::steady_clock::now() - start_time > timeout_duration;
}

// Converts polygon coordinates to Boost Polygon
static Polygon create_polygon(const std::vector<std::pair<int, int>> &coords)
{
	Polygon poly;
	for (const auto &c : coords) {
		bg::append(poly.outer(), Point2D_Float(static_cast<double>(c.first),
										 static_cast<double>(c.second)));
	}
	bg::correct(poly);
	return poly;
}

/// Optimized flood fill for larger polygons with multi-seed detection for complex shapes like U-shapes
[[maybe_unused]] static std::vector<std::pair<int, int>> optimized_flood_fill_area(
		const std::vector<std::pair<int, int>> &polygon_coords,
		const std::chrono::milliseconds *timeout, int32_t min_x, int32_t max_x,
		int32_t min_z, int32_t max_z)
{
	auto start_time = std::chrono::steady_clock::now();

	std::vector<std::pair<int, int>> filled_area;
	FloodBitmap visited(min_x, max_x, min_z, max_z);

	// Create polygon for containment testing
	Polygon polygon = create_polygon(polygon_coords);

	// Optimized step sizes: larger steps for efficiency, but still catch U-shapes
	int32_t width = max_x - min_x + 1;
	int32_t height = max_z - min_z + 1;
	int32_t step_x = std::clamp(width / 6, 1, 8); // Balance between coverage and speed
	int32_t step_z = std::clamp(height / 6, 1, 8);

	// Pre-allocate queue with reasonable capacity to avoid reallocations
	std::queue<std::pair<int, int>> queue;
	queue = std::queue<std::pair<int, int>>(); // Clear any existing content

	for (int32_t z = min_z; z <= max_z; z += step_z) {
		for (int32_t x = min_x; x <= max_x; x += step_x) {
			// Fast timeout check, only every few iterations
			if (filled_area.size() % 100 == 0) {
				if (timeout && timeout_exceeded(start_time, *timeout)) {
					return filled_area;
				}
			}

			// Skip if already visited or not inside polygon
			if (visited.contains(x, z) ||
					!bg::within(
							Point2D_Float(static_cast<double>(x), static_cast<double>(z)),
							polygon)) {
				continue;
			}

			// Start flood fill from this seed point
			while (!queue.empty())
				queue.pop(); // Clear queue instead of creating new one
			queue.emplace(x, z);
			visited.insert(x, z);

			while (!queue.empty()) {
				auto [curr_x, curr_z] = queue.front();
				queue.pop();

				// Add current point to filled area
				filled_area.emplace_back(curr_x, curr_z);

				// Check all four directions with optimized bounds checking
				const std::array<std::pair<int, int>, 4> neighbors = {
						{{curr_x - 1, curr_z}, {curr_x + 1, curr_z}, {curr_x, curr_z - 1},
								{curr_x, curr_z + 1}}};

				for (const auto &[nx, nz] : neighbors) {
					if (nx >= min_x && nx <= max_x && nz >= min_z && nz <= max_z &&
							visited.insert(nx, nz)) {
						// Only check polygon containment for unvisited points
						if (bg::within(Point2D_Float(static_cast<double>(nx),
											   static_cast<double>(nz)),
									polygon)) {
							queue.emplace(nx, nz);
						}
					}
				}
			}
		}
	}

	return filled_area;
}

/// Original flood fill algorithm with enhanced multi-seed detection for complex shapes
[[maybe_unused]] static std::vector<std::pair<int, int>> original_flood_fill_area(
		const std::vector<std::pair<int, int>> &polygon_coords,
		const std::chrono::milliseconds *timeout, int32_t min_x, int32_t max_x,
		int32_t min_z, int32_t max_z)
{
	auto start_time = std::chrono::steady_clock::now();
	std::vector<std::pair<int, int>> filled_area;
	FloodBitmap visited(min_x, max_x, min_z, max_z);

	// Create polygon for containment testing
	Polygon polygon = create_polygon(polygon_coords);

	// Optimized step sizes for large polygons - coarser sampling for speed
	int32_t width = max_x - min_x + 1;
	int32_t height = max_z - min_z + 1;
	int32_t step_x = std::clamp(width / 8, 1, 12); // Cap max step size for coverage
	int32_t step_z = std::clamp(height / 8, 1, 12);

	// Pre-allocate queue
	std::queue<std::pair<int, int>> queue;
	queue = std::queue<std::pair<int, int>>(); // Clear any existing content
	filled_area.reserve(1000);				   // Reserve space to reduce reallocations

	// Scan for multiple seed points to handle U-shapes and concave polygons
	for (int32_t z = min_z; z <= max_z; z += step_z) {
		for (int32_t x = min_x; x <= max_x; x += step_x) {
			// Reduced timeout checking frequency for better performance
			if (timeout && timeout_exceeded(start_time, *timeout)) {
				return filled_area;
			}

			// Skip if already processed or not inside polygon
			if (visited.contains(x, z) ||
					!bg::within(
							Point2D_Float(static_cast<double>(x), static_cast<double>(z)),
							polygon)) {
				continue;
			}

			// Start flood-fill from this seed point
			while (!queue.empty())
				queue.pop(); // Clear queue instead of creating new one
			queue.emplace(x, z);
			visited.insert(x, z);

			while (!queue.empty()) {
				auto [curr_x, curr_z] = queue.front();
				queue.pop();

				// Only check polygon containment once per point when adding to filled_area
				if (bg::within(Point2D_Float(static_cast<double>(curr_x),
									   static_cast<double>(curr_z)),
							polygon)) {
					filled_area.push_back({curr_x, curr_z});

					// Check adjacent points with optimized iteration
					const std::array<std::pair<int, int>, 4> neighbors = {
							{{curr_x - 1, curr_z}, {curr_x + 1, curr_z},
									{curr_x, curr_z - 1}, {curr_x, curr_z + 1}}};

					for (const auto &[nx, nz] : neighbors) {
						if (nx >= min_x && nx <= max_x && nz >= min_z && nz <= max_z &&
								visited.insert(nx, nz)) {
							queue.emplace(nx, nz);
						}
					}
				}
			}
		}
	}

	return filled_area;
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
			const auto [ix0, iz0] = polygon_coords[i - 1];
			const auto [ix1, iz1] = polygon_coords[i];
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
		const auto height = static_cast<std::size_t>(
				static_cast<std::int64_t>(max_z) - min_z + 1);
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
		const auto [x0, z0, x1, z1] = up
				? std::tuple{ax, az, bx, bz}
				: std::tuple{bx, bz, ax, az};
		edges.push_back({z0, z1, x0, std::int64_t(x1) - x0,
				std::int64_t(z1) - z0, up ? 1 : -1});
	}
	std::sort(edges.begin(), edges.end(), [](const auto &a, const auto &b) {
		return a.z0 < b.z0;
	});
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
				[&](const auto &edge) { return edge.z1 <= z; }), active.end());
		while (next_edge < edges.size() && edges[next_edge].z0 == z)
			active.push_back(edges[next_edge++]);
		crossings.clear();
		for (const auto &edge : active) {
			const auto t = (std::int64_t(z) - edge.z0) * edge.dx;
			const auto quotient = t >= 0 ? t / edge.dz : -((-t + edge.dz - 1) / edge.dz);
			crossings.push_back({edge.x0 + quotient + 1, edge.winding, t % edge.dz == 0});
		}
		std::sort(crossings.begin(), crossings.end(), [](const auto &a, const auto &b) {
			return a.first < b.first;
		});
		int winding = 0;
		for (std::size_t i = 1; i < crossings.size(); ++i) {
			winding += crossings[i - 1].winding;
			if (winding == 0 || crossings[i - 1].first >= crossings[i].first)
				continue;
			const auto first = std::max<std::int64_t>(crossings[i - 1].first, min_x);
			const auto last = std::min<std::int64_t>(crossings[i].first - 1, max_x);
			if (last >= first)
				mask.write_span(z, static_cast<int32_t>(first), static_cast<int32_t>(last), true);
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
	const auto row = static_cast<std::size_t>(static_cast<std::int64_t>(max_x) - min_x + 1);
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
