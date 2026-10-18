#pragma once
#include <array>
#include <cstddef>
#include <vector>
#include <cstdint>
#include <utility>
#include <memory>
#include <optional>
#include <functional>
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
struct GlbMipLevel
{
	std::size_t width = 0, height = 0;
	std::vector<std::array<float, 4>> pixels;
};
struct GlbMipTexture
{
	std::vector<GlbMipLevel> levels;
	int wrap_s = 10497, wrap_t = 10497;
};
struct GlbPaintTriangle
{
	std::array<std::array<float, 3>, 3> vertices{};
	std::array<std::array<float, 2>, 3> uv{};
	std::array<float, 4> factor{1, 1, 1, 1};
	std::array<float, 4> vertex_color{1, 1, 1, 1};
	std::shared_ptr<const GlbMipTexture> texture;
	std::optional<float> alpha_cutoff;
	bool has_uv = false;
	bool has_vertex_color = false;
	bool uncolored = false;
};
struct VoxelPaint
{
	enum class Kind : std::uint8_t
	{
		Rgb,
		Glass,
		Uncolored
	};
	Kind kind = Kind::Uncolored;
	std::array<std::uint8_t, 3> rgb{};
};
std::pair<std::array<float, 3>, std::array<float, 3>> glb_model_bbox(
		const std::vector<std::uint8_t> &bytes);
std::vector<std::array<std::array<float, 3>, 3>> glb_triangles(
		const std::vector<std::uint8_t> &bytes);
// Bounding box of every POSITION vertex in active-scene triangle primitives,
// including unreferenced vertices, matching Rust's glb_model_bbox contract.
std::pair<std::array<float, 3>, std::array<float, 3>> glb_vertex_bounds(
		const std::vector<std::uint8_t> &bytes);
std::vector<ColoredTriangle> glb_colored_triangles(
		const std::vector<std::uint8_t> &bytes);
void for_each_glb_paint_triangle(const std::vector<std::uint8_t> &bytes,
		const std::function<void(const GlbPaintTriangle &)> &sink);
std::vector<GlbPaintTriangle> glb_paint_triangles(const std::vector<std::uint8_t> &bytes);
Block block_for_model_color(std::array<float, 3> color);
class WorldTransform
{
	float is_, ic_, isn_, itx_, ity_, itz_, pc_ = 1, ps_ = 0, ws_[3], wc_, wsn_, wtx_,
											wty_, wtz_;

public:
	static WorldTransform with_world_scale_xyz(double intrinsic_yaw,
			double intrinsic_scale, std::array<double, 3> intrinsic_translation,
			std::array<float, 3> world_scale, double world_yaw, float anchor_x,
			float anchor_y, float anchor_z);
	WorldTransform(double, double, std::array<double, 3>, std::array<float, 3>, double,
			float, float, float);
	// Rust-compatible isotropic constructor (world_scale applied to all axes).
	WorldTransform(double intrinsic_yaw, double intrinsic_scale,
			std::array<double, 3> intrinsic_translation, double world_scale,
			double world_yaw, float anchor_x, float anchor_y, float anchor_z);
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
// Fill closed triangle meshes by pairing vertical-ray crossings per XZ column.
std::vector<Voxel> voxelize_solid_triangles(
		const std::vector<std::array<std::array<float, 3>, 3>> &, const WorldTransform &,
		Block);
std::vector<Voxel> voxelize_colored_triangles(
		const std::vector<ColoredTriangle> &, const WorldTransform &);
std::vector<Voxel> voxelize_colored_triangles(const std::vector<ColoredTriangle> &,
		const WorldTransform &,
		const std::vector<std::pair<std::array<float, 3>, Block>> &palette);
std::vector<Voxel> voxelize_glb(
		const std::vector<std::uint8_t> &, const WorldTransform &);
std::vector<Voxel> voxelize_glb_with_palette(const std::vector<std::uint8_t> &,
		const WorldTransform &,
		const std::vector<std::pair<std::array<float, 3>, Block>> &palette);
std::vector<std::pair<std::array<int, 3>, VoxelPaint>> voxelize_glb_paint(
		const std::vector<std::uint8_t> &, const WorldTransform &);
void visit_triangle_voxels(const std::array<std::array<float, 3>, 3> &,
		const std::function<void(std::array<int, 3>)> &);
class TriangleVoxelVisitCache
{
	struct Impl;
	std::unique_ptr<Impl> impl_;

public:
	TriangleVoxelVisitCache();
	~TriangleVoxelVisitCache();
	TriangleVoxelVisitCache(TriangleVoxelVisitCache &&) noexcept;
	TriangleVoxelVisitCache &operator=(TriangleVoxelVisitCache &&) noexcept;
	TriangleVoxelVisitCache(const TriangleVoxelVisitCache &) = delete;
	TriangleVoxelVisitCache &operator=(const TriangleVoxelVisitCache &) = delete;
	void visit(const std::array<std::array<float, 3>, 3> &,
			const std::function<void(std::array<int, 3>)> &);
};
void place_voxels(world_editor::WorldEditor &, const std::vector<Voxel> &);
}
