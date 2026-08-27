#pragma once

#include <algorithm>
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

// Every Emerge thread runs one complete generation job at a time. These values
// are generation context, not process-wide configuration: terrain floor and
// filler height depend on that job's elevation data. Keeping one context per
// thread lets independent WorldEditors run concurrently without cross-talk.
struct FloorState
{
	int world_min_y = DEFAULT_MIN_Y;
	int world_max_y = DEFAULT_MAX_Y;
	int terrain_floor_y = DEFAULT_MIN_Y;
	int base_chunk_y = DEFAULT_GROUND_LEVEL;
};

inline thread_local FloorState FLOOR_STATE;

inline void set_world_bounds(int minimum, int maximum)
{
	if (minimum % 16 != 0 || maximum % 16 != 15 || minimum >= maximum)
		throw std::invalid_argument("world bounds must cover complete 16-block sections");
	FLOOR_STATE.world_min_y = minimum;
	FLOOR_STATE.world_max_y = maximum;
}

inline int min_y()
{
	return FLOOR_STATE.world_min_y;
}

inline int world_max_y()
{
	return FLOOR_STATE.world_max_y;
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
	FLOOR_STATE.terrain_floor_y = std::max(min_y(), aligned);
}

inline int terrain_floor_y()
{
	return FLOOR_STATE.terrain_floor_y;
}

inline void set_base_chunk_y(int y)
{
	FLOOR_STATE.base_chunk_y = y;
}

inline int base_chunk_y()
{
	return FLOOR_STATE.base_chunk_y;
}
}
