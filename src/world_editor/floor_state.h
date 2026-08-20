#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace arnis::world_editor
{
inline constexpr int DEFAULT_MIN_Y = -64;
inline constexpr int DEFAULT_MAX_Y = 319;
inline constexpr int DEFAULT_GROUND_LEVEL = -62;
inline constexpr int TERRAIN_FLOOR_DEPTH = 64;
inline constexpr int MAX_Y = 2031;

inline std::atomic<int> WORLD_MIN_Y{DEFAULT_MIN_Y};
inline std::atomic<int> WORLD_MAX_Y{DEFAULT_MAX_Y};
inline std::atomic<int> TERRAIN_FLOOR_Y{DEFAULT_MIN_Y};
inline std::atomic<int> BASE_CHUNK_Y{DEFAULT_GROUND_LEVEL};

inline void set_world_bounds(int minimum, int maximum)
{
	if (minimum % 16 != 0 || maximum % 16 != 15 || minimum >= maximum)
		throw std::invalid_argument("world bounds must cover complete 16-block sections");
	WORLD_MIN_Y.store(minimum, std::memory_order_relaxed);
	WORLD_MAX_Y.store(maximum, std::memory_order_relaxed);
}

inline int min_y()
{
	return WORLD_MIN_Y.load(std::memory_order_relaxed);
}

inline int world_max_y()
{
	return WORLD_MAX_Y.load(std::memory_order_relaxed);
}

inline std::pair<std::int8_t, std::int8_t> world_section_range()
{
	return {static_cast<std::int8_t>(min_y() >> 4),
			static_cast<std::int8_t>(world_max_y() >> 4)};
}

inline void set_terrain_floor_y(int ground_level)
{
	const int floor = std::max(min_y(), ground_level - TERRAIN_FLOOR_DEPTH);
	const int aligned = floor >= 0 ? floor / 16 * 16 : -((-floor + 15) / 16) * 16;
	TERRAIN_FLOOR_Y.store(std::max(min_y(), aligned), std::memory_order_relaxed);
}

inline int terrain_floor_y()
{
	return TERRAIN_FLOOR_Y.load(std::memory_order_relaxed);
}

inline void set_base_chunk_y(int y)
{
	BASE_CHUNK_Y.store(y, std::memory_order_relaxed);
}

inline int base_chunk_y()
{
	return BASE_CHUNK_Y.load(std::memory_order_relaxed);
}
}
