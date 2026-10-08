#include "caves_shape_query.h"

#include "world_editor/floor_state.h"

#include <cmath>

namespace arnis::caves
{
namespace
{
double lerp(double t, double a, double b)
{
	return a + t * (b - a);
}
} // namespace

CaveShapeQuery::CaveShapeQuery(const world_editor::WorldEditor &editor,
		const CaveRect &world, std::int64_t seed, int floor_y,
		const CaveEllipsoids &ellipsoids) :
		editor_(editor), world_(world), floor_y_(floor_y),
		gen_(seed, floor_y, world_editor::world_max_y()), ellipsoids_(ellipsoids)
{
	density_cache_.reserve(4096);
	for (std::size_t i = 0; i < ellipsoids_.size(); ++i) {
		const auto &ellipsoid = ellipsoids_[i];
		const int x0 = CaveRect::floor_div(
				static_cast<int>(std::floor(ellipsoid.x - ellipsoid.horizontal_radius)),
				16);
		const int x1 = CaveRect::floor_div(
				static_cast<int>(std::floor(ellipsoid.x + ellipsoid.horizontal_radius)),
				16);
		const int z0 = CaveRect::floor_div(
				static_cast<int>(std::floor(ellipsoid.z - ellipsoid.horizontal_radius)),
				16);
		const int z1 = CaveRect::floor_div(
				static_cast<int>(std::floor(ellipsoid.z + ellipsoid.horizontal_radius)),
				16);
		for (int cx = x0; cx <= x1; ++cx)
			for (int cz = z0; cz <= z1; ++cz)
				carvers_by_chunk_[{cx, cz}].push_back(i);
	}
}

bool CaveShapeQuery::is_cave(int x, int y, int z) const
{
	if (!world_.contains(x, z) || y <= floor_y_ || y > editor_.get_ground_level(x, z) - 6)
		return false;
	if (const auto it = carvers_by_chunk_.find(
				{CaveRect::floor_div(x, 16), CaveRect::floor_div(z, 16)});
			it != carvers_by_chunk_.end())
		for (const auto i : it->second)
			if (ellipsoids_[i].contains(x, y, z))
				return true;
	return noise_carves(x, y, z);
}

bool CaveShapeQuery::solid(int x, int y, int z) const
{
	return world_.contains(x, z) && y >= floor_y_ &&
		   y <= editor_.get_ground_level(x, z) - 6 && !is_cave(x, y, z);
}

double CaveShapeQuery::density_at(int x, int y, int z) const
{
	const auto key = pack_cave_pos(x, y, z);
	if (const auto it = density_cache_.find(key); it != density_cache_.end())
		return it->second;
	return density_cache_.emplace(key, gen_.combined_density(x, y, z)).first->second;
}

bool CaveShapeQuery::noise_carves(int x, int y, int z) const
{
	const int wx0 = CaveRect::floor_div(x, 4) * 4;
	const int wy0 = CaveRect::floor_div(y, 8) * 8;
	const int wz0 = CaveRect::floor_div(z, 4) * 4;
	const int wx1 = wx0 + 4, wy1 = wy0 + 8, wz1 = wz0 + 4;
	const double n000 = density_at(wx0, wy0, wz0);
	const double n100 = density_at(wx1, wy0, wz0);
	const double n001 = density_at(wx0, wy0, wz1);
	const double n101 = density_at(wx1, wy0, wz1);
	const double n010 = density_at(wx0, wy1, wz0);
	const double n110 = density_at(wx1, wy1, wz0);
	const double n011 = density_at(wx0, wy1, wz1);
	const double n111 = density_at(wx1, wy1, wz1);
	const double fy = double(y - wy0) / 8.0;
	const double xz00 = lerp(fy, n000, n010);
	const double xz10 = lerp(fy, n100, n110);
	const double xz01 = lerp(fy, n001, n011);
	const double xz11 = lerp(fy, n101, n111);
	const double fx = double(x - wx0) / 4.0;
	const double z0 = lerp(fx, xz00, xz10);
	const double z1 = lerp(fx, xz01, xz11);
	return lerp(double(z - wz0) / 4.0, z0, z1) <= 0.0 ||
		   gen_.noodle_density(x, y, z) <= 0.0;
}
} // namespace arnis::caves
