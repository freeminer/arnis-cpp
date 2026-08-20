#pragma once
#include "../args.h"
#include "../decals/registry.h"
#include "../osm_parser.h"
#include <memory>
namespace arnis::signage
{
struct WaySigns
{
	std::optional<decals::DecalKey> speed, shield, no_entry, cycleway;
};
WaySigns highway_way_signs(const tags_t &tags, decals::SignRegion region);
std::optional<decals::DecalKey> highway_node_sign(const tags_t &tags, SignageLevel level);
std::optional<decals::DecalKey> power_sign(const tags_t &tags);
std::vector<decals::DecalKey> advertising_keys(const tags_t &tags, std::uint64_t id);
std::optional<decals::DecalKey> information_key(const ProcessedNode &node);
std::optional<decals::DecalKey> furniture_pictogram(const tags_t &tags);
std::shared_ptr<const decals::DecalRegistry> build_registry(
		const std::vector<ProcessedElement> &elements, SignageLevel level,
		decals::SignRegion region);
void place_node_signage(world_editor::WorldEditor &editor, const ProcessedNode &node,
		SignageLevel level, decals::SignRegion region);
void place_way_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		decals::SignRegion region);
}
