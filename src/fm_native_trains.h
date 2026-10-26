#pragma once

#include "block_definitions.h"
#include <cstdint>

namespace arnis::block_definitions
{
// Host-resolved marker: Lua creates Advtrains state only after map activation.
inline Block native_train_marker;
inline std::uint32_t native_train_interval = 512;

inline bool native_train_candidate(pos_t x, pos_t y, pos_t z)
{
	// Stable across overlapping OSM ways, chunks, tiles, and generation order.
	std::uint64_t hash = static_cast<std::uint32_t>(x);
	hash = (hash * 0x9e3779b97f4a7c15ULL) ^ static_cast<std::uint32_t>(z);
	hash = (hash * 0x9e3779b97f4a7c15ULL) ^ static_cast<std::uint32_t>(y);
	hash ^= hash >> 30;
	hash *= 0xbf58476d1ce4e5b9ULL;
	hash ^= hash >> 27;
	hash *= 0x94d049bb133111ebULL;
	hash ^= hash >> 31;
	return hash % native_train_interval == 0;
}
} // namespace arnis::block_definitions
