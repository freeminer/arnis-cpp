#include "voxelize.h"
#include "palette.h"
#include "../block_definitions.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <limits>
#include <unordered_map>

namespace arnis::models_3d
{
namespace
{
using Vec3 = std::array<float, 3>;

// Adapted from MIERUNE's dda-voxelize-rs (MIT); see dda_voxelizer_license.txt.
// The mesh voxelizer scans a triangle in planes along its dominant in-plane
// axis, then uses 3-D DDA to rasterize each cross-section line.
struct VoxelKey
{
	int x, y, z;
	bool operator==(const VoxelKey &other) const
	{
		return x == other.x && y == other.y && z == other.z;
	}
};

struct VoxelKeyHash
{
	std::size_t operator()(const VoxelKey &key) const noexcept
	{
		std::size_t hash = std::hash<int>{}(key.x);
		hash ^= std::hash<int>{}(key.y) + 0x9e3779b9u + (hash << 6) + (hash >> 2);
		hash ^= std::hash<int>{}(key.z) + 0x9e3779b9u + (hash << 6) + (hash >> 2);
		return hash;
	}
};

Vec3 add(const Vec3 &a, const Vec3 &b)
{
	return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vec3 subtract(const Vec3 &a, const Vec3 &b)
{
	return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 multiply(const Vec3 &value, float scale)
{
	return {value[0] * scale, value[1] * scale, value[2] * scale};
}

Vec3 divide(const Vec3 &value, float scale)
{
	return {value[0] / scale, value[1] / scale, value[2] / scale};
}

float length(const Vec3 &value)
{
	return std::hypot(std::hypot(value[0], value[1]), value[2]);
}

bool finite(const Vec3 &value)
{
	return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

int saturating_trunc(float value)
{
	if (value <= static_cast<float>(std::numeric_limits<int>::min()))
		return std::numeric_limits<int>::min();
	if (value >= static_cast<float>(std::numeric_limits<int>::max()))
		return std::numeric_limits<int>::max();
	return static_cast<int>(value);
}

int rounded_voxel_coordinate(float value)
{
	// Rust's f32::round() resolves exact half-way values away from zero.
	return saturating_trunc(std::round(value));
}

int small_triangle_voxel_coordinate(float value)
{
	// glam's as_ivec3 conversion truncates toward zero, unlike floor().
	return saturating_trunc(value + 0.5f);
}

template <typename Emit>
void draw_line(const Vec3 &start, const Vec3 &end, Emit &&emit)
{
	const Vec3 difference = subtract(end, start);
	std::array<int, 3> current{}, last{}, step{};
	Vec3 tmax{}, tdelta{};
	for (int axis = 0; axis < 3; ++axis) {
		last[axis] = rounded_voxel_coordinate(end[axis]);
		current[axis] = rounded_voxel_coordinate(start[axis]);
		// glam's signum() returns +1 for +0 and -1 for -0; that produces
		// infinite tmax on stationary axes rather than a zero-step DDA stall.
		step[axis] = std::signbit(difference[axis]) ? -1 : 1;
		const float next_boundary =
				static_cast<float>(current[axis]) + 0.5f * static_cast<float>(step[axis]);
		tmax[axis] = (next_boundary - start[axis]) / difference[axis];
		tdelta[axis] = static_cast<float>(step[axis]) / difference[axis];
	}

	while (current != last) {
		emit(VoxelKey{current[0], current[1], current[2]});
		// Keep the exact tie-breaking order used by dda-voxelize: X, then Y,
		// otherwise Z. This makes edge/corner intersections deterministic.
		if (tmax[0] < tmax[1]) {
			if (tmax[0] < tmax[2]) {
				current[0] += step[0];
				tmax[0] += tdelta[0];
			} else {
				current[2] += step[2];
				tmax[2] += tdelta[2];
			}
		} else if (tmax[1] < tmax[2]) {
			current[1] += step[1];
			tmax[1] += tdelta[1];
		} else {
			current[2] += step[2];
			tmax[2] += tdelta[2];
		}
	}
	emit(VoxelKey{last[0], last[1], last[2]});
}

template <typename Emit>
void rasterize_triangle(const std::array<Vec3, 3> &triangle, Emit &&emit)
{
	const Vec3 &p1 = triangle[0];
	const Vec3 &p2 = triangle[1];
	const Vec3 &p3 = triangle[2];
	const auto distance = [](const Vec3 &a, const Vec3 &b) {
		return length(subtract(a, b));
	};
	if (distance(p1, p2) <= 1.0f && distance(p2, p3) <= 1.0f &&
			distance(p3, p1) <= 1.0f) {
		for (const auto &point : triangle)
			emit(VoxelKey{small_triangle_voxel_coordinate(point[0]),
					small_triangle_voxel_coordinate(point[1]),
					small_triangle_voxel_coordinate(point[2])});
		return;
	}

	const Vec3 e1 = subtract(p2, p1);
	const Vec3 e2 = subtract(p3, p1);
	Vec3 normal{e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
			e1[0] * e2[1] - e1[1] * e2[0]};
	const float normal_length = length(normal);
	if (!std::isfinite(normal_length) || normal_length == 0.0f)
		return;
	for (auto &component : normal)
		component /= normal_length;

	// max_by selects the last axis on ties; retain that behavior explicitly.
	int normal_axis = 0;
	if (std::abs(normal[1]) >= std::abs(normal[normal_axis]))
		normal_axis = 1;
	if (std::abs(normal[2]) >= std::abs(normal[normal_axis]))
		normal_axis = 2;

	Vec3 minimum = p1;
	Vec3 maximum = p1;
	for (const auto &point : {p2, p3})
		for (int axis = 0; axis < 3; ++axis) {
			minimum[axis] = std::min(minimum[axis], point[axis]);
			maximum[axis] = std::max(maximum[axis], point[axis]);
		}
	const Vec3 box_size = subtract(maximum, minimum);
	int sweep_axis = 0;
	switch (normal_axis) {
	case 0:
		sweep_axis = box_size[1] >= box_size[2] ? 1 : 2;
		break;
	case 1:
		sweep_axis = box_size[2] >= box_size[0] ? 2 : 0;
		break;
	default:
		sweep_axis = box_size[0] >= box_size[1] ? 0 : 1;
		break;
	}

	std::array<Vec3, 3> vertices = triangle;
	// Preserve the pairwise ordering (including equal-coordinate cases) used
	// by the Rust implementation rather than relying on sort's tie behavior.
	if (vertices[0][sweep_axis] > vertices[1][sweep_axis])
		std::swap(vertices[0], vertices[1]);
	if (vertices[1][sweep_axis] > vertices[2][sweep_axis])
		std::swap(vertices[1], vertices[2]);
	if (vertices[0][sweep_axis] > vertices[1][sweep_axis])
		std::swap(vertices[0], vertices[1]);
	const int axis = sweep_axis;
	const Vec3 &low = vertices[0];
	const Vec3 &middle = vertices[1];
	const Vec3 &high = vertices[2];

	Vec3 start_step{};
	Vec3 start_pos{};
	if (middle[axis] - std::floor(low[axis]) >= 1.0f) {
		start_step = divide(subtract(middle, low), middle[axis] - low[axis]);
		start_pos = add(
				low, multiply(start_step, (1.0f - low[axis] + std::floor(low[axis]))));
	} else {
		start_step = divide(subtract(high, middle), high[axis] - middle[axis]);
		start_pos = add(middle,
				multiply(start_step, (1.0f - middle[axis] + std::floor(middle[axis]))));
	}
	const Vec3 end_step = divide(subtract(high, low), high[axis] - low[axis]);
	Vec3 end_pos =
			add(low, multiply(end_step, (1.0f - low[axis] + std::floor(low[axis]))));
	float end_vertex = middle[axis];
	while (end_pos[axis] <= high[axis]) {
		draw_line(start_pos, end_pos, emit);
		start_pos = add(start_pos, start_step);
		end_pos = add(end_pos, end_step);
		if (start_pos[axis] >= end_vertex) {
			end_vertex = start_pos[axis] - middle[axis];
			start_pos = subtract(start_pos, multiply(start_step, end_vertex));
			if (std::abs(high[axis] - middle[axis]) == 0.0f)
				continue;
			start_step = divide(subtract(high, middle), high[axis] - middle[axis]);
			start_pos = add(start_pos, multiply(start_step, end_vertex));
			end_vertex = high[axis];
		}
	}
}

std::array<Vec3, 3> transformed_triangle(
		const std::array<std::array<float, 3>, 3> &triangle,
		const WorldTransform &transform)
{
	return {transform.apply(triangle[0]), transform.apply(triangle[1]),
			transform.apply(triangle[2])};
}

} // namespace

std::vector<Voxel> voxelize_triangles(
		const std::vector<std::array<std::array<float, 3>, 3>> &triangles,
		const WorldTransform &transform)
{
	std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> seen;
	std::vector<Voxel> output;
	const Block block = block_for_model_color({0.7f, 0.7f, 0.7f});
	for (const auto &triangle : triangles) {
		const auto world = transformed_triangle(triangle, transform);
		if (!finite(world[0]) || !finite(world[1]) || !finite(world[2]))
			continue;
		rasterize_triangle(world, [&](const VoxelKey &key) {
			if (seen.emplace(key, output.size()).second)
				output.push_back({{key.x, key.y, key.z}, block});
		});
	}
	return output;
}

std::vector<Voxel> voxelize_uniform_triangles(
		const std::vector<std::array<std::array<float, 3>, 3>> &triangles,
		const WorldTransform &transform, Block block)
{
	std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> seen;
	std::vector<Voxel> output;
	for (const auto &triangle : triangles) {
		const auto world = transformed_triangle(triangle, transform);
		if (!finite(world[0]) || !finite(world[1]) || !finite(world[2]))
			continue;
		rasterize_triangle(world, [&](const VoxelKey &key) {
			if (seen.emplace(key, output.size()).second)
				output.push_back({{key.x, key.y, key.z}, block});
		});
	}
	return output;
}

std::vector<Voxel> voxelize_colored_triangles(
		const std::vector<ColoredTriangle> &triangles, const WorldTransform &transform)
{
	std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> seen;
	std::vector<Voxel> output;
	for (const auto &triangle : triangles) {
		const auto world = transformed_triangle(triangle.vertices, transform);
		if (!finite(world[0]) || !finite(world[1]) || !finite(world[2]))
			continue;
		Block block;
		if (triangle.uncolored) {
			block = block_definitions::STONE_BRICKS;
		} else if (std::abs(triangle.color[0] - 1.0f) < 0.001f &&
				   std::abs(triangle.color[1]) < 0.001f &&
				   std::abs(triangle.color[2] - 1.0f) < 0.001f) {
			block = block_definitions::GLASS;
		} else {
			block = block_for_model_color(triangle.color);
		}
		rasterize_triangle(world, [&](const VoxelKey &key) {
			if (seen.emplace(key, output.size()).second)
				output.push_back({{key.x, key.y, key.z}, block});
		});
	}
	return output;
}
} // namespace arnis::models_3d
