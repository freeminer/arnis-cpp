#include "tiles.h"

#include "../args.h"
#include "../one_world.h"
#include "../projection/projection_spec.h"
#include "../trees/tree_library.h"
#include "../world_utils.h"
#include "../map_item.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <limits>
#include <string_view>
#include <system_error>

namespace arnis::live::tiles
{
namespace
{
std::string number(double value)
{
	return std::to_string(value);
}

std::string tree_size_name(trees::TreeSize size)
{
	switch (size) {
	case trees::TreeSize::Small:
		return "small";
	case trees::TreeSize::Medium:
		return "medium";
	case trees::TreeSize::Big:
		return "big";
	case trees::TreeSize::Tall:
		return "tall";
	case trees::TreeSize::Giant:
		return "giant";
	}
	return "giant";
}

std::string trim(std::string value)
{
	const auto first = value.find_first_not_of(" \t\r\n");
	if (first == std::string::npos)
		return {};
	const auto last = value.find_last_not_of(" \t\r\n");
	return value.substr(first, last - first + 1);
}

bool fail(std::string *error, std::string message)
{
	if (error)
		*error = std::move(message);
	return false;
}

bool copy_dir(const std::filesystem::path &source,
		const std::filesystem::path &destination, std::string *error)
{
	std::error_code ec;
	std::filesystem::create_directories(destination, ec);
	if (ec)
		return fail(error, ec.message());
	std::filesystem::directory_iterator it(source, ec), end;
	if (ec)
		return fail(error, ec.message());
	while (it != end) {
		const auto target = destination / it->path().filename();
		const auto type = it->symlink_status(ec).type();
		if (ec)
			return fail(error, ec.message());
		if (type == std::filesystem::file_type::directory) {
			if (!copy_dir(it->path(), target, error))
				return false;
		} else {
			ec.clear();
			std::filesystem::copy_file(it->path(), target,
					std::filesystem::copy_options::overwrite_existing, ec);
			if (ec)
				return fail(error, ec.message());
		}
		it.increment(ec);
		if (ec)
			return fail(error, ec.message());
	}
	return true;
}
} // namespace

std::vector<std::string> tile_flags(const Args &args, const one_world::Manifest &manifest)
{
	const auto mode = generation_mode_terrain(args.mode) == manifest.terrain
							  ? args.mode
							  : (manifest.terrain ? GenerationMode::GeoTerrain
												  : GenerationMode::GeoOnly);
	auto flag = [](const char *name, bool on) {
		return std::string("--") + name + (on ? "=true" : "=false");
	};
	std::vector<std::string> flags{
			"--live-tile",
			"--map-item=false",
			"--scale=" + number(manifest.scale),
			"--ground-level=" + std::to_string(manifest.ground_level),
			std::string("--mode=") + generation_mode_name(mode),
			flag("winter", args.winter),
			flag("post-apocalyptic", args.post_apocalyptic),
			flag("interior", args.interior),
			flag("canopy-height", args.canopy_height),
			flag("overture", args.overture),
			std::string("--overture-source=") +
					overture_source_name(args.overture_source),
			std::string("--fields=") +
					std::string(field_texture::field_preset_name(args.fields)),
			std::string("--max-tree-size=") + tree_size_name(args.max_tree_size),
			std::string("--signage=") + (signage_full(args.signage)				? "full"
												: signage_enabled(args.signage) ? "basic"
																				: "none"),
			"--osm-tiles-url=" + args.osm_tiles_url,
	};
	if (manifest.height_multiplier != 1.0)
		flags.push_back("--height-multiplier=" + number(manifest.height_multiplier));
	const std::pair<bool, const char *> enabled_flags[] = {
			{args.fillground, "--fillground"}, {args.caves, "--caves"},
			{args.legacy_trees, "--legacy-trees"}, {!args.use_3d, "--no-3d"},
			{args.no_tile_archive, "--no-tile-archive"},
			{manifest.aws_only_elevation, "--aws-only-elevation"},
			{args.bake_lighting, "--bake-lighting"}};
	for (const auto &[enabled, name] : enabled_flags)
		if (enabled)
			flags.emplace_back(name);
	if (args.cave_asset_pack)
		flags.push_back("--cave-asset-pack=" + *args.cave_asset_pack);
	if (args.cave_biomes)
		flags.push_back("--cave-biomes=" + *args.cave_biomes);
	if (args.mapillary_facades)
		flags.push_back(flag("mapillary-facades", *args.mapillary_facades));
	flags.push_back(
			std::string("--mapillary-facade-mode=") +
			(args.mapillary_facade_mode == FacadeMode::Blocks ? "blocks" : "photos"));
	if (args.mapillary_facades_dir)
		flags.push_back("--mapillary-facades-dir=" + *args.mapillary_facades_dir);
	if (args.timeout) {
		const auto seconds = std::max<std::int64_t>(1,
				std::chrono::duration_cast<std::chrono::seconds>(*args.timeout).count());
		flags.push_back("--timeout=" + std::to_string(seconds));
	}
	return flags;
}

std::vector<std::pair<std::string, std::string>> tile_environment(const Args &args)
{
	if (!args.mapillary_token)
		return {};
	const auto token = trim(*args.mapillary_token);
	return token.empty() ? std::vector<std::pair<std::string, std::string>>{}
						 : std::vector<std::pair<std::string, std::string>>{
								   {"MAPILLARY_TOKEN", token}};
}

std::optional<std::pair<cartesian::XZBBoxRect, geographic::LLBBox>> region_bbox(
		const projection::WebMercatorProjection &proj, std::int32_t rx, std::int32_t rz,
		std::string *error)
{
	const std::int64_t min_x = std::int64_t(rx) * 512;
	const std::int64_t min_z = std::int64_t(rz) * 512;
	const std::int64_t max_x = min_x + 511;
	const std::int64_t max_z = min_z + 511;
	if (min_x < std::numeric_limits<int>::min() ||
			max_x > std::numeric_limits<int>::max() ||
			min_z < std::numeric_limits<int>::min() ||
			max_z > std::numeric_limits<int>::max()) {
		fail(error, "region coordinates exceed the block coordinate range");
		return std::nullopt;
	}
	try {
		const cartesian::XZBBoxRect rect(
				{static_cast<int>(min_x), static_cast<int>(min_z)},
				{static_cast<int>(max_x), static_cast<int>(max_z)});
		const auto ll = projection::llbbox_for_rect(proj, rect);
		const auto [snapped, _] = projection::snap_bbox_to_chunks(proj, ll);
		if (!(snapped == rect)) {
			fail(error, "region " + std::to_string(rx) + "," + std::to_string(rz) +
								" does not map onto the world grid");
			return std::nullopt;
		}
		if (ll.min().lat() < -85.0 || ll.max().lat() > 85.0) {
			fail(error, "past the latitudes One World covers");
			return std::nullopt;
		}
		return std::pair{rect, ll};
	} catch (const std::exception &e) {
		fail(error, e.what());
		return std::nullopt;
	}
}

std::filesystem::path live_root(const std::filesystem::path &world_dir)
{
	return world_dir / LIVE_DIR;
}

std::string staging_name(std::size_t index)
{
	return "staging-" + std::to_string(index);
}

std::optional<std::filesystem::path> prepare_staging(
		const std::filesystem::path &world_dir, std::size_t index,
		std::int32_t first_map_id_value, std::string *error)
{
	const auto dir = live_root(world_dir) / staging_name(index);
	std::error_code ec;
	std::filesystem::remove_all(dir, ec);
	ec.clear();
	std::filesystem::create_directories(dir, ec);
	if (ec) {
		fail(error, "Failed to create " + dir.string() + ": " + ec.message());
		return std::nullopt;
	}
	for (const auto *file : {"level.dat", one_world::MANIFEST_FILE}) {
		ec.clear();
		std::filesystem::copy_file(world_dir / file, dir / file,
				std::filesystem::copy_options::overwrite_existing, ec);
		if (ec) {
			fail(error, "Failed to copy " + std::string(file) +
								" into the staging world: " + ec.message());
			return std::nullopt;
		}
	}
	const auto tall = world_dir / "datapacks" /
					  std::filesystem::path(world_utils::TALL_DATAPACK_NAME);
	if (std::filesystem::is_directory(tall, ec) &&
			!copy_dir(tall, dir / "datapacks" / tall.filename(), error))
		return std::nullopt;
	const auto last_id = first_map_id_value == std::numeric_limits<std::int32_t>::min()
								 ? first_map_id_value
								 : first_map_id_value - 1;
	if (!map_item::set_map_counter(dir, last_id, error))
		return std::nullopt;
	return dir;
}

std::int32_t first_map_id(std::int32_t live_next, std::size_t index)
{
	constexpr std::int64_t ids_per_worker = 200'000;
	const auto worker = std::min<std::uint64_t>(
			index, static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()));
	const std::int64_t value = std::int64_t(live_next) + 10'000 +
							   static_cast<std::int64_t>(worker) * ids_per_worker;
	return static_cast<std::int32_t>(
			std::clamp<std::int64_t>(value, std::numeric_limits<std::int32_t>::min(),
					std::numeric_limits<std::int32_t>::max()));
}

std::optional<std::int32_t> map_id_of(const std::string &name)
{
	if (name.size() <= 4 || name.compare(name.size() - 4, 4, ".dat") != 0)
		return std::nullopt;
	auto stem = std::string_view(name).substr(0, name.size() - 4);
	if (stem.starts_with("map_"))
		stem.remove_prefix(4);
	if (stem.empty())
		return std::nullopt;
	std::int32_t id = 0;
	const auto [end, ec] = std::from_chars(stem.data(), stem.data() + stem.size(), id);
	if (ec != std::errc{} || end != stem.data() + stem.size())
		return std::nullopt;
	return id;
}

std::string map_file_name(world_utils::WorldLayout::Kind layout, std::int32_t id)
{
	return layout == world_utils::WorldLayout::Kind::Legacy
				   ? "map_" + std::to_string(id) + ".dat"
				   : std::to_string(id) + ".dat";
}

std::optional<Staged> collect(const std::filesystem::path &staging, std::int32_t rx,
		std::int32_t rz, std::string *error)
{
	const auto layout = world_utils::WorldLayout::of(staging);
	const auto region = layout.overworld_dir(staging) / "region" /
						("r." + std::to_string(rx) + "." + std::to_string(rz) + ".mca");
	if (!std::filesystem::is_regular_file(region)) {
		fail(error, "the tile wrote no region");
		return std::nullopt;
	}
	Staged staged;
	staged.region = region;
	std::error_code ec;
	for (std::filesystem::directory_iterator it(layout.maps_dir(staging), ec), end;
			!ec && it != end; it.increment(ec)) {
		if (!it->is_regular_file(ec)) {
			ec.clear();
			continue;
		}
		const auto name = it->path().filename().string();
		if (const auto id = map_id_of(name))
			staged.maps.emplace_back(*id, it->path());
	}
	if (ec) {
		ec.clear();
		staged.maps.clear();
	}
	if (const auto manifest = one_world::Manifest::load(staging);
			manifest && !manifest->areas.empty() && manifest->areas.back().preview) {
		if (const auto preview = one_world::safe_preview_path(
					staging, *manifest->areas.back().preview);
				preview && std::filesystem::is_regular_file(*preview))
			staged.preview = *preview;
	}
	return staged;
}
} // namespace arnis::live::tiles
