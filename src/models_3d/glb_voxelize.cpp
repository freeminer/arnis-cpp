#include "voxelize.h"
#include "palette.h"
#include "../colors.h"
#include "../block_definitions.h"
#include <algorithm>
#include <cmath>
#include <limits>
namespace arnis::models_3d
{
namespace
{
Block paint_block(const VoxelPaint &paint,
		const std::vector<std::pair<std::array<float, 3>, Block>> *palette)
{
	if (paint.kind == VoxelPaint::Kind::Glass)
		return block_definitions::GLASS;
	if (paint.kind == VoxelPaint::Kind::Uncolored)
		return block_definitions::STONE_BRICKS;
	const RGBTuple target{paint.rgb[0], paint.rgb[1], paint.rgb[2]};
	if (!palette || palette->empty())
		return closest_block(target);
	const auto to_rgb = [](const std::array<float, 3> &c) {
		return RGBTuple{static_cast<std::uint8_t>(std::clamp(c[0], 0.0f, 1.0f) * 255.0f),
				static_cast<std::uint8_t>(std::clamp(c[1], 0.0f, 1.0f) * 255.0f),
				static_cast<std::uint8_t>(std::clamp(c[2], 0.0f, 1.0f) * 255.0f)};
	};
	Block best_block = palette->front().second;
	float best_distance = std::numeric_limits<float>::infinity();
	for (const auto &[color, block] : *palette) {
		const float distance = ::oklab_distance(target, to_rgb(color));
		if (distance < best_distance) {
			best_distance = distance;
			best_block = block;
		}
	}
	return best_block;
}

std::vector<Voxel> voxelize_paint(const std::vector<std::uint8_t> &bytes,
		const WorldTransform &transform,
		const std::vector<std::pair<std::array<float, 3>, Block>> *palette)
{
	const auto painted = voxelize_glb_paint(bytes, transform);
	std::vector<Voxel> voxels;
	voxels.reserve(painted.size());
	for (const auto &[position, paint] : painted)
		voxels.push_back({position, paint_block(paint, palette)});
	return voxels;
}
} // namespace

std::vector<Voxel> voxelize_glb(
		const std::vector<std::uint8_t> &bytes, const WorldTransform &t)
{
	return voxelize_paint(bytes, t, nullptr);
}

std::vector<Voxel> voxelize_glb_with_palette(const std::vector<std::uint8_t> &bytes,
		const WorldTransform &t,
		const std::vector<std::pair<std::array<float, 3>, Block>> &palette)
{
	return voxelize_paint(bytes, t, &palette);
}
}
