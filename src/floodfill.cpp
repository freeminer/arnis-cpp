#include <iostream>
#include <vector>
#include <queue>
#include <chrono>
#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <cmath>

#include "floodfill.h"

namespace arnis {

namespace floodfill {

namespace bg = boost::geometry;

// Type aliases for convenience
using Point2D = bg::model::d2::point_xy<int>;
using Point2D_Float = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point2D_Float>;

/// Maximum bounding box area (in blocks) for flood fill.
/// Polygons exceeding this are skipped to prevent excessive memory allocations.
/// 25 million blocks ≈ 5000×5000; bitmap uses only ~3 MB at this size.
const int64_t MAX_FLOOD_FILL_AREA = 25000000;

/// A compact bitmap for visited-coordinate tracking during flood fill.
///
/// Uses 1 bit per coordinate instead of ~48 bytes per entry in a `HashSet`.
/// For a 5000×5000 bounding box this is ~3 MB instead of ~1.2 GB.
class FloodBitmap {
private:
    std::vector<uint8_t> bits;
    int32_t min_x;
    int32_t min_z;
    size_t width;

public:
    FloodBitmap(int32_t min_x, int32_t max_x, int32_t min_z, int32_t max_z) 
        : min_x(min_x), min_z(min_z) {
        width = static_cast<size_t>(max_x - min_x + 1);
        size_t height = static_cast<size_t>(max_z - min_z + 1);
        size_t num_bytes = (width * height + 7) / 8; // Ceiling division
        bits.resize(num_bytes, 0);
    }

    /// Mark (x, z) as visited. Returns `true` if it was NOT already visited
    /// (i.e. this is the first visit).
    bool insert(int32_t x, int32_t z) {
        size_t idx = static_cast<size_t>(z - min_z) * width + static_cast<size_t>(x - min_x);
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

    bool contains(int32_t x, int32_t z) const {
        size_t idx = static_cast<size_t>(z - min_z) * width + static_cast<size_t>(x - min_x);
        size_t byte = idx / 8;
        size_t bit = idx % 8;
        return (bits[byte] >> bit) & 1U;
    }
};

// Check if the current time exceeds the timeout duration
inline bool timeout_exceeded(const std::chrono::steady_clock::time_point& start_time, const std::chrono::milliseconds& timeout_duration) {
    const auto elapsed = std::chrono::steady_clock::now() - start_time;
    return std::chrono::duration_cast<std::chrono::milliseconds>(elapsed) > timeout_duration;
}

// Converts polygon coordinates to Boost Polygon
static Polygon create_polygon(const std::vector<std::pair<int,int>>& coords) {
    Polygon poly;
    for (const auto& c : coords) {
        bg::append(poly.outer(), Point2D_Float(static_cast<double>(c.first), static_cast<double>(c.second)));
    }
    bg::correct(poly);
    return poly;
}

/// Optimized flood fill for larger polygons with multi-seed detection for complex shapes like U-shapes
static std::vector<std::pair<int,int>> optimized_flood_fill_area(
    const std::vector<std::pair<int,int>>& polygon_coords,
    const std::chrono::milliseconds* timeout,
    int32_t min_x, int32_t max_x, int32_t min_z, int32_t max_z
) {
    auto start_time = std::chrono::steady_clock::now();

    std::vector<std::pair<int,int>> filled_area;
    FloodBitmap visited(min_x, max_x, min_z, max_z);

    // Create polygon for containment testing
    Polygon polygon = create_polygon(polygon_coords);

    // Optimized step sizes: larger steps for efficiency, but still catch U-shapes
    int32_t width = max_x - min_x + 1;
    int32_t height = max_z - min_z + 1;
    int32_t step_x = std::clamp(width / 6, 1, 8); // Balance between coverage and speed
    int32_t step_z = std::clamp(height / 6, 1, 8);

    // Pre-allocate queue with reasonable capacity to avoid reallocations
    std::queue<std::pair<int,int>> queue;
    queue = std::queue<std::pair<int,int>>(); // Clear any existing content

    for (int32_t z = min_z; z <= max_z; z += step_z) {
        for (int32_t x = min_x; x <= max_x; x += step_x) {
            // Fast timeout check, only every few iterations
            if (filled_area.size() % 100 == 0) {
                if (timeout && timeout_exceeded(start_time, *timeout)) {
                    return filled_area;
                }
            }

            // Skip if already visited or not inside polygon
            if (visited.contains(x, z) || !bg::within(Point2D_Float(static_cast<double>(x), static_cast<double>(z)), polygon)) {
                continue;
            }

            // Start flood fill from this seed point
            while (!queue.empty()) queue.pop(); // Clear queue instead of creating new one
            queue.emplace(x, z);
            visited.insert(x, z);

            while (!queue.empty()) {
                auto [curr_x, curr_z] = queue.front(); queue.pop();

                // Add current point to filled area
                filled_area.emplace_back(curr_x, curr_z);

                // Check all four directions with optimized bounds checking
                const std::array<std::pair<int,int>,4> neighbors = {
                    {{curr_x - 1, curr_z}, {curr_x + 1, curr_z}, {curr_x, curr_z - 1}, {curr_x, curr_z + 1}}
                };

                for (const auto& [nx, nz] : neighbors) {
                    if (nx >= min_x && nx <= max_x && nz >= min_z && nz <= max_z && visited.insert(nx, nz)) {
                        // Only check polygon containment for unvisited points
                        if (bg::within(Point2D_Float(static_cast<double>(nx), static_cast<double>(nz)), polygon)) {
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
static std::vector<std::pair<int,int>> original_flood_fill_area(
    const std::vector<std::pair<int,int>>& polygon_coords,
    const std::chrono::milliseconds* timeout,
    int32_t min_x, int32_t max_x, int32_t min_z, int32_t max_z
) {
    auto start_time = std::chrono::steady_clock::now();
    std::vector<std::pair<int,int>> filled_area;
    FloodBitmap visited(min_x, max_x, min_z, max_z);

    // Create polygon for containment testing
    Polygon polygon = create_polygon(polygon_coords);

    // Optimized step sizes for large polygons - coarser sampling for speed
    int32_t width = max_x - min_x + 1;
    int32_t height = max_z - min_z + 1;
    int32_t step_x = std::clamp(width / 8, 1, 12); // Cap max step size for coverage
    int32_t step_z = std::clamp(height / 8, 1, 12);

    // Pre-allocate queue
    std::queue<std::pair<int,int>> queue;
    queue = std::queue<std::pair<int,int>>(); // Clear any existing content
    filled_area.reserve(1000); // Reserve space to reduce reallocations

    // Scan for multiple seed points to handle U-shapes and concave polygons
    for (int32_t z = min_z; z <= max_z; z += step_z) {
        for (int32_t x = min_x; x <= max_x; x += step_x) {
            // Reduced timeout checking frequency for better performance
            if (timeout && timeout_exceeded(start_time, *timeout)) {
                return filled_area;
            }

            // Skip if already processed or not inside polygon
            if (visited.contains(x, z) || !bg::within(Point2D_Float(static_cast<double>(x), static_cast<double>(z)), polygon)) {
                continue;
            }

            // Start flood-fill from this seed point
            while (!queue.empty()) queue.pop(); // Clear queue instead of creating new one
            queue.emplace(x, z);
            visited.insert(x, z);

            while (!queue.empty()) {
                auto [curr_x, curr_z] = queue.front(); queue.pop();

                // Only check polygon containment once per point when adding to filled_area
                if (bg::within(Point2D_Float(static_cast<double>(curr_x), static_cast<double>(curr_z)), polygon)) {
                    filled_area.push_back({curr_x, curr_z});

                    // Check adjacent points with optimized iteration
                    const std::array<std::pair<int,int>,4> neighbors = {
                        {{curr_x - 1, curr_z}, {curr_x + 1, curr_z}, {curr_x, curr_z - 1}, {curr_x, curr_z + 1}}
                    };

                    for (const auto& [nx, nz] : neighbors) {
                        if (nx >= min_x && nx <= max_x && nz >= min_z && nz <= max_z && visited.insert(nx, nz)) {
                            queue.emplace(nx, nz);
                        }
                    }
                }
            }
        }
    }

    return filled_area;
}

/// Main flood fill function with automatic algorithm selection
/// Chooses the best algorithm based on polygon size and complexity
std::vector<std::pair<int,int>> flood_fill_area(
    const std::vector<std::pair<int,int>>& polygon_coords,
    const std::chrono::milliseconds* timeout
) {
    if (polygon_coords.size() < 3) {
        return {}; // Not a valid polygon
    }

    // Calculate bounding box of the polygon
    auto minmax_x = std::minmax_element(polygon_coords.begin(), polygon_coords.end(),
        [](const std::pair<int,int>& a, const std::pair<int,int>& b) {
            return a.first < b.first;
        });
    auto minmax_z = std::minmax_element(polygon_coords.begin(), polygon_coords.end(),
        [](const std::pair<int,int>& a, const std::pair<int,int>& b) {
            return a.second < b.second;
        });

    int32_t min_x = minmax_x.first->first;
    int32_t max_x = minmax_x.second->first;
    int32_t min_z = minmax_z.first->second;
    int32_t max_z = minmax_z.second->second;

    int64_t area = static_cast<int64_t>(max_x - min_x + 1) * static_cast<int64_t>(max_z - min_z + 1);

    // Safety cap: reject polygons whose bounding box is too large.
    // This prevents multi-GB memory allocations when ocean-adjacent elements
    // (e.g. natural=water, large landuse) produce huge clipped polygons.
    if (area > MAX_FLOOD_FILL_AREA) {
        return {};
    }

    // For small and medium areas, use optimized flood fill with span filling
    if (area < 50000) {
        return optimized_flood_fill_area(polygon_coords, timeout, min_x, max_x, min_z, max_z);
    } else {
        // For larger areas, use original flood fill with grid sampling
        return original_flood_fill_area(polygon_coords, timeout, min_x, max_x, min_z, max_z);
    }
}

}
}