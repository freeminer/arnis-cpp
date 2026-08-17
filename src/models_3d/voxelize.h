#pragma once
#include <array>
#include <vector>
#include <cstdint>
#include <utility>
#include "../../../arnis_block.h"
namespace arnis::world_editor
{
struct WorldEditor;
}
namespace arnis::models_3d
{
struct Voxel
{
	std::array<int, 3> position{};
	Block block{};
};
struct ColoredTriangle
{
	std::array<std::array<float, 3>, 3> vertices{};
	std::array<float, 3> color{0.7f, 0.7f, 0.7f};
	// glTF's default white factor has no authored material; Rust emits stone bricks.
	bool uncolored = false;
};
std::pair<std::array<float, 3>, std::array<float, 3>> glb_model_bbox(
		const std::vector<std::uint8_t> &bytes);
std::vector<std::array<std::array<float, 3>, 3>> glb_triangles(
		const std::vector<std::uint8_t> &bytes);
std::vector<ColoredTriangle> glb_colored_triangles(
		const std::vector<std::uint8_t> &bytes);
Block block_for_model_color(std::array<float, 3> color);
class WorldTransform
{
	float is_, ic_, isn_, itx_, ity_, itz_, pc_ = 1, ps_ = 0, ws_[3], wc_, wsn_, wtx_,
											wty_, wtz_;

public:
	WorldTransform(double, double, std::array<double, 3>, std::array<float, 3>, double,
			float, float, float);
	WorldTransform pitched(double) const;
	std::array<float, 3> apply(std::array<float, 3>) const;
};
std::vector<Voxel> voxelize_points(const std::vector<std::array<float, 3>> &,
		const std::vector<std::array<float, 3>> &, const WorldTransform &);
std::vector<Voxel> voxelize_triangles(
		const std::vector<std::array<std::array<float, 3>, 3>> &, const WorldTransform &);
// Rust's voxelize_uniform_triangles: preserve a caller-selected material for
// geometry formats (notably STL) that carry no reliable vertex/material color.
std::vector<Voxel> voxelize_uniform_triangles(
		const std::vector<std::array<std::array<float, 3>, 3>> &, const WorldTransform &,
		Block);
std::vector<Voxel> voxelize_colored_triangles(
		const std::vector<ColoredTriangle> &, const WorldTransform &);
std::vector<Voxel> voxelize_glb(
		const std::vector<std::uint8_t> &, const WorldTransform &);
void place_voxels(world_editor::WorldEditor &, const std::vector<Voxel> &);
}
