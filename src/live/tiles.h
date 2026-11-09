#pragma once

#include "../coordinate_system/cartesian/xzbbox/rectangle.h"
#include "../coordinate_system/geographic/llbbox.h"
#include "../world_utils.h"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace arnis
{
struct Args;
namespace one_world
{
struct Manifest;
}
namespace projection
{
class WebMercatorProjection;
}
} // namespace arnis

namespace arnis::live::tiles
{
inline constexpr const char *LIVE_DIR = "arnis_one_world/live";

// Serializable tile-run settings and environment. These are kept separate from
// process management so embedding applications can schedule generation themselves.
std::vector<std::string> tile_flags(
		const Args &args, const one_world::Manifest &manifest);
std::vector<std::pair<std::string, std::string>> tile_environment(const Args &args);

// Return the exact block rectangle and geographic bbox for a 512x512-block
// region. Invalid coordinates, non-round-tripping projections and polar regions
// are rejected with a Rust-compatible diagnostic.
std::optional<std::pair<cartesian::XZBBoxRect, geographic::LLBBox>> region_bbox(
		const projection::WebMercatorProjection &projection, std::int32_t rx,
		std::int32_t rz, std::string *error = nullptr);

std::filesystem::path live_root(const std::filesystem::path &world_dir);
std::string staging_name(std::size_t index);
std::optional<std::filesystem::path> prepare_staging(
		const std::filesystem::path &world_dir, std::size_t index,
		std::int32_t first_map_id, std::string *error = nullptr);
std::int32_t first_map_id(std::int32_t live_next, std::size_t index);
std::optional<std::int32_t> map_id_of(const std::string &name);
std::string map_file_name(world_utils::WorldLayout::Kind layout, std::int32_t id);

struct Staged
{
	std::filesystem::path region;
	std::vector<std::pair<std::int32_t, std::filesystem::path>> maps;
	std::optional<std::filesystem::path> preview;
};

// Locate the region, maps and optional preview produced in a staging world.
std::optional<Staged> collect(const std::filesystem::path &staging, std::int32_t rx,
		std::int32_t rz, std::string *error = nullptr);
} // namespace arnis::live::tiles
