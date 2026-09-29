#pragma once

#include <cstdint>
#include <algorithm>
#include <tuple>

namespace arnis::caves
{

struct CaveRect
{
	int min_x = 0, max_x = -1, min_z = 0, max_z = -1;
	bool contains(int x, int z) const noexcept
	{
		return x >= min_x && x <= max_x && z >= min_z && z <= max_z;
	}
	CaveRect grow(int amount) const noexcept
	{
		return {min_x - amount, max_x + amount, min_z - amount, max_z + amount};
	}
	CaveRect clip(const CaveRect &other) const noexcept
	{
		return {std::max(min_x, other.min_x), std::min(max_x, other.max_x),
				std::max(min_z, other.min_z), std::min(max_z, other.max_z)};
	}
	std::tuple<int, int, int, int> chunks() const noexcept
	{
		return {floor_div(min_x, 16), floor_div(max_x, 16), floor_div(min_z, 16),
				floor_div(max_z, 16)};
	}
	static int floor_div(int value, int divisor) noexcept
	{
		const int q = value / divisor, r = value % divisor;
		return r < 0 ? q - 1 : q;
	}
};

// Stable signed-coordinate key, matching Rust's pack/unpack use for cave sets.
inline std::int64_t pack_cave_pos(int x, int y, int z) noexcept
{
	return (std::int64_t(std::uint32_t(x)) << 32) ^
		   (std::int64_t(std::uint16_t(y)) << 16) ^ std::uint16_t(z);
}
inline std::tuple<int, int, int> unpack_cave_pos(std::int64_t value) noexcept
{
	return {int(std::uint32_t(std::uint64_t(value) >> 32)),
			int(std::int16_t(std::uint16_t(std::uint64_t(value) >> 16))),
			int(std::int16_t(std::uint16_t(value)))};
}

}
