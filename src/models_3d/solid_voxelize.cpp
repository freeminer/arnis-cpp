#include "voxelize.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace arnis::models_3d
{
namespace
{
struct Column
{
	int x, z;
	bool operator==(const Column &other) const { return x == other.x && z == other.z; }
};

struct ColumnHash
{
	std::size_t operator()(const Column &p) const noexcept
	{
		std::size_t h = std::hash<int>{}(p.x);
		h ^= std::hash<int>{}(p.z) + 0x9e3779b9u + (h << 6) + (h >> 2);
		return h;
	}
};

struct Cell
{
	int x, y, z;
	bool operator==(const Cell &other) const
	{
		return x == other.x && y == other.y && z == other.z;
	}
};

struct CellHash
{
	std::size_t operator()(const Cell &p) const noexcept
	{
		std::size_t h = std::hash<int>{}(p.x);
		h ^= std::hash<int>{}(p.y) + 0x9e3779b9u + (h << 6) + (h >> 2);
		h ^= std::hash<int>{}(p.z) + 0x9e3779b9u + (h << 6) + (h >> 2);
		return h;
	}
};

bool finite_triangle(const std::array<std::array<float, 3>, 3> &triangle)
{
	for (const auto &point : triangle)
		for (float value : point)
			if (!std::isfinite(value))
				return false;
	return true;
}
} // namespace

std::vector<Voxel> voxelize_solid_triangles(
		const std::vector<std::array<std::array<float, 3>, 3>> &triangles,
		const WorldTransform &transform, Block block)
{
	std::vector<std::array<std::array<float, 3>, 3>> world;
	world.reserve(triangles.size());
	for (const auto &triangle : triangles) {
		auto mapped = triangle;
		for (auto &point : mapped)
			point = transform.apply(point);
		if (finite_triangle(mapped))
			world.push_back(mapped);
	}

	std::unordered_set<Cell, CellHash> cells;
	for (const auto &voxel : voxelize_uniform_triangles(triangles, transform, block))
		cells.insert({voxel.position[0], voxel.position[1], voxel.position[2]});

	// Jitter off voxel centres so the ray does not run along a shared mesh edge.
	constexpr float JX = 0.0123f;
	constexpr float JZ = 0.0371f;
	std::unordered_map<Column, std::vector<float>, ColumnHash> crossings;
	for (const auto &t : world) {
		const float ax = t[0][0], az = t[0][2];
		const float bx = t[1][0], bz = t[1][2];
		const float cx = t[2][0], cz = t[2][2];
		const float denominator = (bz - cz) * (ax - cx) + (cx - bx) * (az - cz);
		if (std::abs(denominator) < 1e-9f)
			continue;
		const int x0 = static_cast<int>(std::ceil(std::min({ax, bx, cx}) - JX));
		const int x1 = static_cast<int>(std::floor(std::max({ax, bx, cx}) - JX));
		const int z0 = static_cast<int>(std::ceil(std::min({az, bz, cz}) - JZ));
		const int z1 = static_cast<int>(std::floor(std::max({az, bz, cz}) - JZ));
		for (int x = x0; x <= x1; ++x)
			for (int z = z0; z <= z1; ++z) {
				const float px = x + JX, pz = z + JZ;
				const float w0 =
						((bz - cz) * (px - cx) + (cx - bx) * (pz - cz)) / denominator;
				const float w1 =
						((cz - az) * (px - cx) + (ax - cx) * (pz - cz)) / denominator;
				const float w2 = 1.0f - w0 - w1;
				if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f)
					continue;
				const float y = w0 * t[0][1] + w1 * t[1][1] + w2 * t[2][1];
				crossings[{x, z}].push_back(y);
			}
	}

	for (auto &[column, hits] : crossings) {
		std::sort(hits.begin(), hits.end());
		// An odd crossing count means this column passes through an open mesh;
		// preserve its shell but do not invent an interior.
		if (hits.size() % 2 != 0)
			continue;
		for (std::size_t i = 0; i + 1 < hits.size(); i += 2) {
			const auto first = static_cast<long long>(std::ceil(hits[i]));
			const auto last = static_cast<long long>(std::floor(hits[i + 1]));
			for (long long y = first; y <= last; ++y) {
				if (y < std::numeric_limits<int>::min() ||
						y > std::numeric_limits<int>::max())
					continue;
				cells.insert({column.x, static_cast<int>(y), column.z});
			}
		}
	}

	std::vector<Voxel> result;
	result.reserve(cells.size());
	for (const auto &cell : cells)
		result.push_back({{cell.x, cell.y, cell.z}, block});
	return result;
}
} // namespace arnis::models_3d
