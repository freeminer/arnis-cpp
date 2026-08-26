#pragma once
#include "../../arnis_adapter.h"
#include "floodfill_cache.h"
#include <unordered_map>
#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

namespace arnis
{
// Note: Types are defined in arnis_adapter.h, no forward declarations needed

// Hash for pair<string, uint64_t> used in StillWaterSurfaces
struct PairHashStringUint
{
	std::size_t operator()(const std::pair<std::string, std::uint64_t> &p) const noexcept
	{
		// Simple hash combining string hash and uint64
		std::size_t h1 = std::hash<std::string>{}(p.first);
		std::size_t h2 = std::hash<std::uint64_t>{}(p.second);
		return h1 ^ (h2 << 1);
	}
};

enum class WorldFormat
{
	JavaAnvil,
	BedrockMcWorld,
	LuantiWorld
};
struct GenerationOptions
{
	WorldFormat format = WorldFormat::JavaAnvil;
	std::filesystem::path output_path;
	std::string level_name;
	bool map_preview = false, map_item = true, bake_lighting = false, use_3d = true;
	int ground_level = 0;
	int spawn_x = 0, spawn_y = 0, spawn_z = 0;
	std::string projection_name;
	double projection_scale = 1.0;
	double min_lat = 0.0, max_lat = 0.0, min_lon = 0.0, max_lon = 0.0;
};
inline bool is_java(WorldFormat f)
{
	return f == WorldFormat::JavaAnvil;
}
inline bool is_bedrock(WorldFormat f)
{
	return f == WorldFormat::BedrockMcWorld;
}
inline bool is_luanti(WorldFormat f)
{
	return f == WorldFormat::LuantiWorld;
}
inline const char *format_name(WorldFormat f)
{
	switch (f) {
	case WorldFormat::JavaAnvil:
		return "java";
	case WorldFormat::BedrockMcWorld:
		return "bedrock";
	case WorldFormat::LuantiWorld:
		return "luanti";
	}
	return "unknown";
}
inline const char *format_extension(WorldFormat f)
{
	switch (f) {
	case WorldFormat::JavaAnvil:
		return "mca";
	case WorldFormat::BedrockMcWorld:
		return "mcworld";
	case WorldFormat::LuantiWorld:
		return "world";
	}
	return "";
}
inline std::filesystem::path generation_output_path(const GenerationOptions &o)
{
	if (!o.output_path.empty())
		return o.output_path;
	std::string base = o.level_name.empty() ? "Arnis World" : o.level_name;
	return std::filesystem::path(base + "." + format_extension(o.format));
}
inline bool valid_generation_options(const GenerationOptions &o)
{
	return o.ground_level >= -64 && o.ground_level <= 319 &&
		   (!o.output_path.empty() || o.level_name.empty());
}
inline void apply_generation_options(WorldEditor &editor, const GenerationOptions &o)
{
	editor.set_generation_format(static_cast<int>(o.format));
	editor.set_output_path(o.output_path);
	editor.set_level_name(o.level_name);
	editor.set_spawn(o.spawn_x, o.spawn_y, o.spawn_z);
	editor.set_projection_info(o.projection_name, o.projection_scale);
	editor.set_geographic_bounds(o.min_lat, o.max_lat, o.min_lon, o.max_lon);
	editor.set_bake_lighting(o.bake_lighting);
	editor.set_place_schematics(o.use_3d);
	editor.set_start_with_map(o.map_item);
	editor.set_map_decals(is_java(o.format));
}

// Rust pipeline policy: region streaming is worthwhile only when enough tiles
// exist to amortize flush-worker setup and memory bookkeeping.
bool should_stream_to_disk(std::size_t tile_count);
bool should_use_parallel_tiles(std::size_t tile_count, bool java_format);
struct GenerationFeatureFlags
{
	bool map_item = false, map_preview = false, branding = false, map_decals = false;
};
GenerationFeatureFlags generation_features(
		bool java_format, bool luanti_format, bool map_item, bool map_preview);
struct GenerationTilePolicy
{
	bool parallel = false;
	bool stream_to_disk = false;
};
GenerationTilePolicy generation_tile_policy(std::size_t tile_count, bool java_format);
struct GenerationProgress
{
	double processing = 19.5, world_start = 20.0, world_end = 70.0, ground_start = 70.0,
		   ground_end = 90.0, saving_start = 90.0, saving_end = 97.0,
		   finalizing_start = 97.0, finalizing_end = 100.0;
};

/// Water surface Y of every still OSM water body, resolved once per element.
struct StillWaterSurfaces
{
	std::unordered_map<std::pair<std::string, std::uint64_t>, int, PairHashStringUint> surfaces;
	
	int get(const std::string &kind, std::uint64_t id) const
	{
		auto it = surfaces.find({kind, id});
		return it != surfaces.end() ? it->second : -1000; // Below world
	}
	bool has(const std::string &kind, std::uint64_t id) const
	{
		return surfaces.find({kind, id}) != surfaces.end();
	}
};

/// Pre-scan water polygon surfaces so a body spanning many tiles is measured once.
StillWaterSurfaces prescan_still_surfaces(
		const std::vector<ProcessedElement> &elements, const Ground *ground, const XZBBox &xzbbox);
inline double generation_stage_progress(const GenerationProgress &p,
		double world_fraction, double ground_fraction, bool saving)
{
	if (saving)
		return p.saving_start +
			   (p.saving_end - p.saving_start) * std::clamp(world_fraction, 0.0, 1.0);
	if (ground_fraction > 0)
		return p.ground_start +
			   (p.ground_end - p.ground_start) * std::clamp(ground_fraction, 0.0, 1.0);
	return p.world_start +
		   (p.world_end - p.world_start) * std::clamp(world_fraction, 0.0, 1.0);
}
inline double generation_finalize_progress(const GenerationProgress &p, double fraction)
{
	return p.finalizing_start +
		   (p.finalizing_end - p.finalizing_start) * std::clamp(fraction, 0.0, 1.0);
}
struct GenerationRuntime
{
	GenerationOptions options;
	GenerationFeatureFlags features;
	GenerationTilePolicy tiles;
	GenerationProgress progress;
	bool initialized = false, completed = false, failed = false;
	std::string error;
	void initialize(std::size_t tile_count)
	{
		features = generation_features(is_java(options.format), is_luanti(options.format),
				options.map_item, options.map_preview);
		tiles = generation_tile_policy(tile_count, is_java(options.format));
		progress = GenerationProgress{};
		initialized = true;
		completed = false;
		failed = false;
		error.clear();
	}
	void finish()
	{
		completed = true;
		failed = false;
	}
	void fail(std::string message)
	{
		failed = true;
		completed = false;
		error = std::move(message);
	}
	double world_progress(double fraction) const
	{
		return generation_stage_progress(progress, fraction, 0.0, false);
	}
	double ground_progress(double fraction) const
	{
		return generation_stage_progress(progress, 1.0, fraction, false);
	}
	double saving_progress(double fraction = 1.0) const
	{
		return generation_stage_progress(progress, fraction, 1.0, true);
	}
	double finalizing_progress(double fraction = 1.0) const
	{
		return generation_finalize_progress(progress, fraction);
	}
	bool usable() const { return initialized && !failed; }
	void reset()
	{
		initialized = false;
		completed = false;
		failed = false;
		error.clear();
	}
};
inline GenerationProgress generation_progress()
{
	return {};
}
void release_finished_fills(FloodFillCache &cache,
		const std::unordered_map<std::uint64_t, std::size_t> &last_use,
		const ProcessedElement &element, std::size_t index);
std::unordered_map<std::uint64_t, std::size_t> compute_last_fill_use(
		const std::vector<ProcessedElement> &elements);

bool generate_world(WorldEditor &editor, const std::vector<ProcessedElement> &elements,
		const Args &args, FloodFillCache &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints);
}
