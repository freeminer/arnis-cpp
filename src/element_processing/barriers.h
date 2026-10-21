#pragma once

#include "bridges.h"
#include "../floodfill_cache.h"

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace arnis::barriers
{

class RoadClearance
{
public:
	enum class PlacementKind : std::uint8_t
	{
		Post,
		PathGap,
		RoadGap
	};
	struct Placement
	{
		PlacementKind kind{PlacementKind::Post};
		int x{};
		int z{};
	};

	RoadClearance();
	static RoadClearance build(const std::vector<ProcessedElement> &elements,
			const world_editor::WorldEditor &editor, double scale,
			std::shared_ptr<const CoordinateBitmap> roads);
	Placement place(int x, int z, std::pair<float, float> direction, bool crosses_roads,
			const CoordinateBitmap &buildings) const;
	bool on_road(int x, int z) const;

private:
	struct Impl;
	std::shared_ptr<const Impl> impl_;
	explicit RoadClearance(std::shared_ptr<const Impl> impl) : impl_(std::move(impl)) {}
};

void generate_barriers(world_editor::WorldEditor &editor, const ProcessedElement &element,
		const bridges::BridgeSurfaceMap &bridge_surface, const RoadClearance &clearance,
		const CoordinateBitmap &building_footprints);
void generate_barriers(world_editor::WorldEditor &editor, const ProcessedElement &element,
		const bridges::BridgeSurfaceMap &bridge_surface);
void generate_barriers(
		world_editor::WorldEditor &editor, const ProcessedElement &element);
void generate_barrier_nodes(world_editor::WorldEditor &editor, const ProcessedNode &node,
		const bridges::BridgeSurfaceMap &bridge_surface);
void generate_barrier_nodes(world_editor::WorldEditor &editor, const ProcessedNode &node);

} // namespace arnis::barriers
