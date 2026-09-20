#include "voxelize.h"
#include <cmath>
#include <functional>
#include <stdexcept>
#include <algorithm>
#include <limits>
#include <unordered_map>
#include <tiny_gltf.h>
#include "../../../arnis_adapter.h"
#include "../colors.h"
#include "../block_definitions.h"
#include "palette.h"
namespace arnis::models_3d
{
namespace
{
int round_block_coordinate(float value)
{
	const float lo = static_cast<float>(std::numeric_limits<int>::min());
	const float hi = static_cast<float>(std::numeric_limits<int>::max());
	return static_cast<int>(std::lround(std::clamp(value, lo, hi)));
}

struct VoxelKey
{
	int x, y, z;
	bool operator==(const VoxelKey &o) const { return x == o.x && y == o.y && z == o.z; }
};
struct VoxelKeyHash
{
	std::size_t operator()(const VoxelKey &k) const noexcept
	{
		std::size_t h = std::hash<int>{}(k.x);
		h ^= std::hash<int>{}(k.y) + 0x9e3779b9u + (h << 6) + (h >> 2);
		h ^= std::hash<int>{}(k.z) + 0x9e3779b9u + (h << 6) + (h >> 2);
		return h;
	}
};
}

WorldTransform WorldTransform::with_world_scale_xyz(double intrinsic_yaw,
		double intrinsic_scale, std::array<double, 3> intrinsic_translation,
		std::array<float, 3> world_scale, double world_yaw, float anchor_x,
		float anchor_y, float anchor_z)
{
	return WorldTransform(intrinsic_yaw, intrinsic_scale, intrinsic_translation,
			world_scale, world_yaw, anchor_x, anchor_y, anchor_z);
}

Block block_for_model_color(std::array<float, 3> c)
{
	if (std::abs(c[0] - 1.f) < .001f && std::abs(c[1]) < .001f &&
			std::abs(c[2] - 1.f) < .001f)
		return block_definitions::GLASS;
	return closest_block(RGBTuple{std::uint8_t(std::clamp(c[0], 0.f, 1.f) * 255),
			std::uint8_t(std::clamp(c[1], 0.f, 1.f) * 255),
			std::uint8_t(std::clamp(c[2], 0.f, 1.f) * 255)});
}
void place_voxels(world_editor::WorldEditor &e, const std::vector<Voxel> &v)
{
	for (const auto &q : v)
		e.set_block_absolute(q.block, q.position[0], q.position[1], q.position[2]);
}
std::vector<Voxel> voxelize_points(const std::vector<std::array<float, 3>> &points,
		const std::vector<std::array<float, 3>> &colors, const WorldTransform &t)
{
	std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> seen;
	std::vector<Voxel> out;
	for (std::size_t i = 0; i < points.size(); ++i) {
		auto q = t.apply(points[i]);
		if (!std::all_of(q.begin(), q.end(), [](float v) { return std::isfinite(v); }))
			continue;
		int x = round_block_coordinate(q[0]), y = round_block_coordinate(q[1]),
			z = round_block_coordinate(q[2]);
		VoxelKey k{x, y, z};
		if (seen.count(k))
			continue;
		auto c = i < colors.size() ? colors[i] : std::array<float, 3>{1, 1, 1};
		out.push_back({{x, y, z}, block_for_model_color(c)});
		seen.emplace(k, out.size() - 1);
	}
	return out;
}
std::pair<std::array<float, 3>, std::array<float, 3>> glb_model_bbox(
		const std::vector<std::uint8_t> &bytes)
{
	std::array<float, 3> lo{INFINITY, INFINITY, INFINITY},
			hi{-INFINITY, -INFINITY, -INFINITY};
	for (const auto &triangle : glb_colored_triangles(bytes))
		for (const auto &p : triangle.vertices)
			for (int j = 0; j < 3; ++j)
				if (std::isfinite(p[j])) {
					lo[j] = std::min(lo[j], p[j]);
					hi[j] = std::max(hi[j], p[j]);
				}
	if (!std::all_of(lo.begin(), lo.end(), [](float v) { return std::isfinite(v); }) ||
			!std::all_of(hi.begin(), hi.end(), [](float v) { return std::isfinite(v); }))
		throw std::runtime_error("GLB has no positions");
	return {lo, hi};
}
WorldTransform::WorldTransform(double yaw, double scale, std::array<double, 3> t,
		std::array<float, 3> w, double wy, float x, float y, float z) :
		is_(scale), itx_(t[0]), ity_(t[1]), itz_(t[2]), wtx_(x), wty_(y), wtz_(z)
{
	// Rust casts angles to f32 before to_radians()/sin()/cos(); preserve that
	// order so model placement remains numerically stable across backends.
	const float a = static_cast<float>(yaw) * 3.14159265358979323846f / 180.0f;
	const float b = static_cast<float>(wy) * 3.14159265358979323846f / 180.0f;
	ic_ = std::cos(a);
	isn_ = std::sin(a);
	wc_ = std::cos(b);
	wsn_ = std::sin(b);
	for (int i = 0; i < 3; ++i)
		ws_[i] = w[i];
}

WorldTransform::WorldTransform(double intrinsic_yaw, double intrinsic_scale,
		std::array<double, 3> intrinsic_translation, double world_scale, double world_yaw,
		float anchor_x, float anchor_y, float anchor_z) :
		WorldTransform(intrinsic_yaw, intrinsic_scale, intrinsic_translation,
				std::array<float, 3>{static_cast<float>(world_scale),
						static_cast<float>(world_scale), static_cast<float>(world_scale)},
				world_yaw, anchor_x, anchor_y, anchor_z)
{
}
WorldTransform WorldTransform::pitched(double d) const
{
	auto r = *this;
	const float a = static_cast<float>(d) * 3.14159265358979323846f / 180.0f;
	r.pc_ = std::cos(a);
	r.ps_ = std::sin(a);
	return r;
}
std::array<float, 3> WorldTransform::apply(std::array<float, 3> p) const
{
	float py = p[1] * pc_ + p[2] * ps_, pz = -p[1] * ps_ + p[2] * pc_, sx = p[0] * is_,
		  sy = py * is_, sz = pz * is_;
	float x = (sx * ic_ - sz * isn_ + itx_) * ws_[0], y = (sy + ity_) * ws_[1],
		  z = (sx * isn_ + sz * ic_ + itz_) * ws_[2];
	return {x * wc_ - z * wsn_ + wtx_, y + wty_, x * wsn_ + z * wc_ + wtz_};
}
}
