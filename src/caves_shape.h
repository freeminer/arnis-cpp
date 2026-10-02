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

// Compact cave-cell key matching Rust caves::pack (24-bit X/Z, 12-bit Y with
// an offset that keeps the full supported terrain range nonnegative).
inline std::int64_t pack_cave_pos(int x, int y, int z) noexcept
{
	const auto ux = (std::uint64_t(std::int64_t(x) + (1 << 23)) & 0xFF'FFFFULL);
	const auto uz = (std::uint64_t(std::int64_t(z) + (1 << 23)) & 0xFF'FFFFULL);
	const auto uy = (std::uint64_t(std::int64_t(y) + 2048) & 0xFFFULL);
	return static_cast<std::int64_t>((ux << 36) | (uz << 12) | uy);
}
inline std::tuple<int, int, int> unpack_cave_pos(std::int64_t value) noexcept
{
	const auto packed = static_cast<std::uint64_t>(value);
	return {static_cast<int>((packed >> 36) & 0xFF'FFFFULL) - (1 << 23),
			static_cast<int>((packed & 0xFFFULL)) - 2048,
			static_cast<int>((packed >> 12) & 0xFF'FFFFULL) - (1 << 23)};
}

}
