#include "voxelize.h"
#include "palette.h"
#include "../block_definitions.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
namespace arnis::models_3d
{
namespace
{
float distance3(const std::array<float, 3> &a, const std::array<float, 3> &b)
{
	return std::hypot(std::hypot(b[0] - a[0], b[1] - a[1]), b[2] - a[2]);
}

// Sampling density must be derived after the complete placement transform.
// Measuring raw model coordinates loses faces on deliberately small GLBs that
// are enlarged to an OSM footprint (or wastes work on a shrunk model).
int surface_steps(const std::array<std::array<float, 3>, 3> &triangle,
		const WorldTransform &transform)
{
	const auto a = transform.apply(triangle[0]);
	const auto b = transform.apply(triangle[1]);
	const auto c = transform.apply(triangle[2]);
	for (const auto &point : {a, b, c})
		for (const float value : point)
			if (!std::isfinite(value))
				return 0;
	const float longest = std::max({distance3(a, b), distance3(a, c), distance3(b, c)});
	// A half-block sampling interval gives overlapping samples across rounded
	// voxel boundaries; DDA's supercover semantics without external dependency.
	return std::clamp(static_cast<int>(std::ceil(longest * 2.0f)), 1, 1024);
}
}

std::vector<Voxel> voxelize_triangles(
		const std::vector<std::array<std::array<float, 3>, 3>> &ts,
		const WorldTransform &t)
{
	std::vector<std::array<float, 3>> p, c;
	for (const auto &q : ts) {
		const int steps = surface_steps(q, t);
		if (!steps)
			continue;
		for (int i = 0; i <= steps; ++i)
			for (int j = 0; j <= steps - i; ++j) {
				const float u = float(i) / steps, v = float(j) / steps, w = 1.0f - u - v;
				p.push_back({w * q[0][0] + u * q[1][0] + v * q[2][0],
						w * q[0][1] + u * q[1][1] + v * q[2][1],
						w * q[0][2] + u * q[1][2] + v * q[2][2]});
				c.push_back({0.7f, 0.7f, 0.7f});
			}
	}
	return voxelize_points(p, c, t);
}

std::vector<Voxel> voxelize_uniform_triangles(
		const std::vector<std::array<std::array<float, 3>, 3>> &ts,
		const WorldTransform &t, Block block)
{
	// Keep the same transformed surface density and first-hit voxel ownership
	// as the coloured path, but do not invent a palette colour for colorless
	// triangle formats.
	std::unordered_map<std::string, bool> seen;
	std::vector<Voxel> out;
	for (const auto &q : ts) {
		const int steps = surface_steps(q, t);
		if (!steps)
			continue;
		for (int i = 0; i <= steps; ++i)
			for (int j = 0; j <= steps - i; ++j) {
				const float u = float(i) / steps, v = float(j) / steps, w = 1.f - u - v;
				auto p = t.apply({w * q[0][0] + u * q[1][0] + v * q[2][0],
						w * q[0][1] + u * q[1][1] + v * q[2][1],
						w * q[0][2] + u * q[1][2] + v * q[2][2]});
				const int x = std::lround(p[0]), y = std::lround(p[1]),
						  z = std::lround(p[2]);
				const auto key = std::to_string(x) + ':' + std::to_string(y) + ':' +
								 std::to_string(z);
				if (seen.emplace(key, true).second)
					out.push_back({{x, y, z}, block});
			}
	}
	return out;
}

std::vector<Voxel> voxelize_colored_triangles(
		const std::vector<ColoredTriangle> &ts, const WorldTransform &t)
{
	std::vector<std::array<float, 3>> p, c;
	std::vector<bool> uncolored;
	for (const auto &q : ts) {
		const int steps = surface_steps(q.vertices, t);
		if (!steps)
			continue;
		for (int i = 0; i <= steps; ++i)
			for (int j = 0; j <= steps - i; ++j) {
				const float u = float(i) / steps, v = float(j) / steps, w = 1.0f - u - v;
				p.push_back({w * q.vertices[0][0] + u * q.vertices[1][0] +
									 v * q.vertices[2][0],
						w * q.vertices[0][1] + u * q.vertices[1][1] +
								v * q.vertices[2][1],
						w * q.vertices[0][2] + u * q.vertices[1][2] +
								v * q.vertices[2][2]});
				c.push_back(q.color);
				uncolored.push_back(q.uncolored);
			}
	}
	std::unordered_map<long long, std::size_t> seen;
	std::vector<Voxel> out;
	for (std::size_t i = 0; i < p.size(); ++i) {
		auto w = t.apply(p[i]);
		const int x = std::lround(w[0]), y = std::lround(w[1]), z = std::lround(w[2]);
		const long long key = (static_cast<long long>(x) << 42) ^
							  (static_cast<long long>(y) << 21) ^
							  static_cast<std::uint32_t>(z);
		if (seen.count(key))
			continue;
		Block block;
		if (uncolored[i])
			block = block_definitions::STONE_BRICKS;
		else if (std::abs(c[i][0] - 1.f) < .001f && std::abs(c[i][1]) < .001f &&
				 std::abs(c[i][2] - 1.f) < .001f)
			block = block_definitions::GLASS;
		else
			block = closest_block(
					RGBTuple{std::uint8_t(std::clamp(c[i][0], 0.f, 1.f) * 255),
							std::uint8_t(std::clamp(c[i][1], 0.f, 1.f) * 255),
							std::uint8_t(std::clamp(c[i][2], 0.f, 1.f) * 255)});
		out.push_back({{x, y, z}, block});
		seen.emplace(key, out.size() - 1);
	}
	return out;
}
}
