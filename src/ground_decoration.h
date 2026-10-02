#pragma once

#include "biome.h"
#include "ecoregion.h"

#include <cstdint>
#include <optional>

//#include "args.h"
//#include "mapgen/earth/arnis_world_editor.h"

struct XZBBox;

namespace arnis
{
struct Args;
class Block;

namespace world_editor
{
struct WorldEditor;
}
}

namespace arnis::ground_decoration
{
enum class Habitat
{
	Meadow,
	Alpine,
	Forest,
	Taiga,
	Jungle,
	Shrub,
	Steppe,
	Desert,
	Tundra,
	Wetland,
	Prairie,
	Maquis,
	Savanna
};

enum class FlowerSetting
{
	Meadow,
	Forest,
	Garden
};
std::optional<Habitat> habitat(std::uint8_t cover, biome::Climate climate,
		double absolute_latitude, bool alpine, std::optional<ecoregion::Ecoregion> eco);
// Used by schematic trees to remove ground plants displaced by a trunk, while
// keeping the loose-plant classification shared with Rust's ground pass.
bool is_undergrowth(const Block &block);
void clear_undergrowth_under_trunk(
		world_editor::WorldEditor &editor, int x, int y, int z);
// Rust exposes these helpers to mapped-area processors.  Keeping them public
// lets parks/meadows share the same deterministic flower fields as the main
// ground-decoration pass.
void place_scattered_flower(
		world_editor::WorldEditor &editor, int x, int z, FlowerSetting setting);
void place_bed_flower(world_editor::WorldEditor &editor, int x, int z);
// Rust ground_decoration::decorate_region counterpart.  Origins are chunk
// stable, so decorating adjacent streamed tiles produces the same patches.
void decorate_region(world_editor::WorldEditor &editor, const Args &args,
		const ::XZBBox &bbox, int min_x, int max_x, int min_z, int max_z);
}
