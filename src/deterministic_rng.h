#pragma once

#include <random>
#include <cstdint>

namespace arnis {

/// Creates a deterministic RNG seeded from coordinates.
///
/// Use this for per-block randomness that needs to be consistent regardless
/// of processing order (e.g., random flower placement within a natural area).
///
/// @param x X coordinate
/// @param z Z coordinate
/// @param element_id The element ID for additional uniqueness
/// @return A seeded mt19937 that will produce deterministic random values
inline std::mt19937 coord_rng(int32_t x, int32_t z, uint64_t element_id) {
    // Combine coordinates and element_id into a seed.
    // Cast through uint32_t to handle negative coordinates consistently.
    int64_t coord_part = (static_cast<int64_t>(static_cast<uint32_t>(x)) << 32) | 
                         static_cast<uint32_t>(z);
    uint64_t seed = static_cast<uint64_t>(coord_part) ^ element_id;
    return std::mt19937(static_cast<unsigned int>(seed));
}

/// Creates a deterministic RNG seeded from an element ID.
///
/// The same element ID will always produce the same sequence of random values,
/// ensuring consistent results when an element is processed multiple times
/// (e.g., once per region it touches during streaming).
///
/// @param element_id The unique OSM element ID (way ID, node ID, or relation ID)
/// @return A seeded mt19937 that will produce deterministic random values
inline std::mt19937 element_rng(uint64_t element_id) {
    return std::mt19937(static_cast<unsigned int>(element_id));
}

/// Creates a deterministic RNG seeded from an element ID with an additional salt.
///
/// Use this when you need multiple independent random sequences for the same element.
/// For example, one sequence for wall colors and another for roof style.
///
/// @param element_id The unique OSM element ID
/// @param salt Additional value to create a different sequence (e.g., use different
///   salt values for different purposes within the same element)
/// @return A seeded mt19937 that will produce deterministic random values
inline std::mt19937 element_rng_salted(uint64_t element_id, uint64_t salt) {
    // Combine element_id and salt using XOR and bit rotation to avoid collisions
    uint64_t combined = element_id ^ ((salt << 32) | (salt >> 32)); // Simple rotation
    return std::mt19937(static_cast<unsigned int>(combined));
}

} // namespace arnis