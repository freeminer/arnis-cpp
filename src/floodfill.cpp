#include <iostream>
#include <vector>
#include <queue>
#include <chrono>
#include <algorithm>
#include <array>
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
static std::vector<std::pair<int, int>> optimized_flood_fill_area(
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
static std::vector<std::pair<int, int>> original_flood_fill_area(
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
		return optimized_flood_fill_area(
				polygon_coords, timeout, min_x, max_x, min_z, max_z);
	return original_flood_fill_area(polygon_coords, timeout, min_x, max_x, min_z, max_z);
}

}
}
