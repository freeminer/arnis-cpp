#pragma once

#include <vector>
#include <unordered_map>
#include <memory>
#include <chrono>
#include <optional>

#include "mapgen/earth/arnis_adapter.h"
#include "osm_parser.h"

/// Simple bounding box structure for defining world bounds
struct XZBBox {
    int32_t min_x_;
    int32_t min_z_;
    int32_t max_x_;
    int32_t max_z_;
    
    XZBBox(int32_t min_x, int32_t min_z, int32_t max_x, int32_t max_z) 
        : min_x_(min_x), min_z_(min_z), max_x_(max_x), max_z_(max_z) {}
    
    int32_t min_x() const { return min_x_; }
    int32_t min_z() const { return min_z_; }
    int32_t max_x() const { return max_x_; }
    int32_t max_z() const { return max_z_; }
};

namespace arnis {

/// A memory-efficient bitmap for storing coordinates.
///
/// Instead of storing each coordinate individually (~24 bytes per entry in a HashSet),
/// this uses 1 bit per coordinate in the world bounds, reducing memory usage by ~200x.
class CoordinateBitmap {
private:
    /// The bitmap data, where each bit represents one (x, z) coordinate
    std::vector<uint8_t> bits_;
    /// Minimum x coordinate (offset for indexing)
    int32_t min_x_;
    /// Minimum z coordinate (offset for indexing)
    int32_t min_z_;
    /// Width of the world (max_x - min_x + 1)
    size_t width_;
    /// Height of the world (max_z - min_z + 1)
    size_t height_;
    /// Number of coordinates marked
    size_t count_;

public:
    /// Creates a new empty bitmap covering the given world bounds.
    CoordinateBitmap(const XZBBox& xzbbox);
    
    /// Sets a coordinate.
    void set(int32_t x, int32_t z);
    
    /// Checks if a coordinate is set.
    bool contains(int32_t x, int32_t z) const;
    
    /// Returns true if no coordinates are marked.
    bool is_empty() const { return count_ == 0; }
    
    /// Returns the number of coordinates that are set.
    size_t count() const { return count_; }
    
    /// Counts how many coordinates from the given container are set in this bitmap.
    template<typename Container>
    size_t count_contained(const Container& coords) const {
        size_t result = 0;
        for (const auto& coord : coords) {
            if (contains(coord.first, coord.second)) {
                ++result;
            }
        }
        return result;
    }
    
    /// Counts the number of set bits in a rectangular range.
    ///
    /// Returns (urban_count, total_count) for the given range.
    std::pair<size_t, size_t> count_in_range(int32_t min_x, int32_t min_z, int32_t max_x, int32_t max_z) const;
};

/// Type alias for building footprint bitmap (for backwards compatibility).
using BuildingFootprintBitmap = CoordinateBitmap;

/// Forward declaration
class FloodFillCache;

/// A cache of pre-computed flood fill results, keyed by element ID.
class FloodFillCache {
private:
    /// Cached results: element_id -> filled coordinates
    std::unordered_map<uint64_t, std::vector<std::pair<int32_t, int32_t>>> way_cache;

    /// Determines if a way element needs flood fill based on its tags.
    static bool way_needs_flood_fill(const ProcessedWay& way);
    
    /// Computes the centroid of a set of coordinates.
    static std::optional<std::pair<int32_t, int32_t>> compute_centroid(const std::vector<std::pair<int32_t, int32_t>>& coords);

public:
    /// Creates an empty cache.
    FloodFillCache() = default;
    
    /// Pre-computes flood fills for all elements that need them.
    static FloodFillCache precompute(const std::vector<ProcessedElement>& elements, 
                                   const std::optional<std::chrono::milliseconds>& timeout);
    
    /// Gets cached flood fill result for a way, or computes it if not cached.
    std::vector<std::pair<int32_t, int32_t>> get_or_compute(
        const ProcessedWay& way,
        const std::optional<std::chrono::milliseconds>& timeout) const;
    
    /// Gets cached flood fill result for a ProcessedElement (Way only).
    /// For Nodes/Relations, returns empty vec.
    std::vector<std::pair<int32_t, int32_t>> get_or_compute_element(
        const ProcessedElement& element,
        const std::optional<std::chrono::milliseconds>& timeout) const;
    
    /// Collects all building footprint coordinates from the pre-computed cache.
    BuildingFootprintBitmap collect_building_footprints(
        const std::vector<ProcessedElement>& elements,
        const XZBBox& xzbbox) const;
    
    /// Collects centroids of all buildings from the pre-computed cache.
    std::vector<std::pair<int32_t, int32_t>> collect_building_centroids(
        const std::vector<ProcessedElement>& elements) const;
    
    /// Removes a way's cached flood fill result, freeing memory.
    void remove_way(uint64_t way_id);
    
    /// Removes all cached flood fill results for ways in a relation.
    void remove_relation_ways(const std::vector<uint64_t>& way_ids);
};

/// Configures the global thread pool with a CPU usage cap.
void configure_thread_pool(double cpu_fraction);

} // namespace arnis
