#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <string>

#include "celestial.h"
#include "projection/web_mercator.h"

namespace arnis
{

enum class GenerationMode
{
	GeoTerrain,
	GeoOnly,
	TerrainOnly
};
enum class GameMode
{
	Survival,
	Creative,
	Spectator
};
constexpr int java_game_type(GameMode mode)
{
	return mode == GameMode::Survival ? 0 : (mode == GameMode::Creative ? 1 : 3);
}
constexpr int bedrock_game_type(GameMode mode)
{
	return mode == GameMode::Survival ? 0 : (mode == GameMode::Creative ? 1 : 6);
}
inline GameMode game_mode_from_string(const std::string &value)
{
	if (value == "survival")
		return GameMode::Survival;
	if (value == "spectator")
		return GameMode::Spectator;
	return GameMode::Creative;
}
enum class SignageLevel
{
	None,
	Basic,
	Full
};
enum class FacadeMode
{
	Photos,
	Blocks
};
constexpr bool facade_mode_places_displays(FacadeMode mode)
{
	return mode == FacadeMode::Photos;
}
enum class FacadeDetail
{
	Standard,
	High
};
enum class OvertureSource
{
	Auto,
	Tiles,
	Parquet
};
inline constexpr std::uint32_t ATLAS_SIDE_STANDARD = 8192;
inline constexpr std::uint32_t ATLAS_SIDE_HIGH = 16384;
inline constexpr std::uint32_t facade_atlas_side(FacadeDetail detail)
{
	return detail == FacadeDetail::High ? ATLAS_SIDE_HIGH : ATLAS_SIDE_STANDARD;
}

inline constexpr double OBJECT_SKIP_SCALE = 0.3;
inline constexpr double MIN_SCALE = 0.05;
inline constexpr double MAX_SCALE = 4.0;
inline constexpr std::int64_t DEFAULT_WORLD_TIME = 6000;
inline constexpr std::int64_t MIDNIGHT_TICKS = 18000;

inline bool valid_scale(double scale)
{
	return std::isfinite(scale) && scale >= MIN_SCALE && scale <= MAX_SCALE;
}

inline bool validate_scale(double scale)
{
	return valid_scale(scale);
}

constexpr bool generation_mode_terrain(GenerationMode mode)
{
	return mode != GenerationMode::GeoOnly;
}
constexpr bool generation_mode_skips_objects(GenerationMode mode)
{
	return mode == GenerationMode::TerrainOnly;
}
inline GenerationMode generation_mode_from_string(const std::string &value)
{
	if (value == "geo-only")
		return GenerationMode::GeoOnly;
	if (value == "terrain-only")
		return GenerationMode::TerrainOnly;
	return GenerationMode::GeoTerrain;
}
inline const char *generation_mode_name(GenerationMode mode)
{
	switch (mode) {
	case GenerationMode::GeoOnly:
		return "geo-only";
	case GenerationMode::TerrainOnly:
		return "terrain-only";
	case GenerationMode::GeoTerrain:
		return "geo-terrain";
	}
	return "geo-terrain";
}
constexpr bool signage_enabled(SignageLevel level)
{
	return level != SignageLevel::None;
}
constexpr bool signage_full(SignageLevel level)
{
	return level == SignageLevel::Full;
}
inline FacadeDetail facade_detail_from_string(const std::string &value)
{
	return value == "high" ? FacadeDetail::High : FacadeDetail::Standard;
}
inline FacadeMode facade_mode_from_string(const std::string &value)
{
	return value == "blocks" ? FacadeMode::Blocks : FacadeMode::Photos;
}
inline OvertureSource overture_source_from_string(const std::string &value)
{
	if (value == "tiles")
		return OvertureSource::Tiles;
	if (value == "parquet")
		return OvertureSource::Parquet;
	return OvertureSource::Auto;
}
inline const char *overture_source_name(OvertureSource source)
{
	switch (source) {
	case OvertureSource::Tiles:
		return "tiles";
	case OvertureSource::Parquet:
		return "parquet";
	case OvertureSource::Auto:
		return "auto";
	}
	return "auto";
}
inline SignageLevel signage_level_from_string(const std::string &value)
{
	return value == "none"	 ? SignageLevel::None
		   : value == "full" ? SignageLevel::Full
							 : SignageLevel::Basic;
}

struct Args
{
	// Bounding box of the area (min_lat,min_lng,max_lat,max_lng) (required)
	//LLBBox bbox{};

	// JSON file containing OSM data (optional)
	std::optional<std::string> file{std::nullopt};

	// JSON file to save OSM data to (optional)
	std::optional<std::string> save_json_file{std::nullopt};

	// Output directory for the generated world (required for Java, optional for Bedrock).
	// Use --output-dir (or the deprecated --path alias) to specify where the world is created.
	std::optional<std::string> path{std::nullopt};

	// Generate a Bedrock Edition world (.mcworld) instead of Java Edition
	bool bedrock{false};
	// Select the Luanti-compatible output path when embedding Arnis as a library.
	bool luanti{false};

	// Downloader method (requests/curl/wget) (optional)
	std::string downloader{std::string("requests")};

	// World scale to use, in blocks per meter (1.0 = real size)
	double scale{1.0};
	// Moon and Mars use the same geographic projection but a body-specific
	// terrain scale/palette. Latitude is supplied by library callers for Mars'
	// polar caps when no geographic bbox is retained in Args.
	CelestialBody body{CelestialBody::Earth};
	projection::ProjectionKind projection{projection::ProjectionKind::Local};
	double celestial_latitude_degrees{0.0};

	// Ground level to use in the Minecraft world
	int ground_level{-62};

	// Generation mode mirrors Rust. `terrain` remains as a compatibility cache for
	// the existing C++ processors and defaults to the Rust geo-terrain behaviour.
	GenerationMode mode{GenerationMode::GeoTerrain};
	bool terrain{true};
	bool legacy_terrain{false};
	bool terrain_enabled() const { return mode != GenerationMode::GeoOnly; }
	bool skip_objects() const
	{
		return mode == GenerationMode::TerrainOnly || !is_earth(body) ||
			   scale < OBJECT_SKIP_SCALE;
	}
	bool skip_objects_due_to_scale() const
	{
		return mode != GenerationMode::TerrainOnly &&
			   (!is_earth(body) || scale < OBJECT_SKIP_SCALE);
	}
	void apply_body_defaults()
	{
		if (is_earth(body))
			return;
		scale = celestial_world_scale(body);
		mode = GenerationMode::TerrainOnly;
		overture = false;
		canopy_height = false;
		use_3d = false;
		interior = false;
		legacy_trees = false;
		aws_only_elevation = false;
		building_facades = false;
		mapillary_facades = false;
		disable_height_limit = false;
		// Preserve an explicitly selected time; otherwise airless worlds use
		// Minecraft midnight (Rust's MIDNIGHT_TICKS).
		if (world_time == DEFAULT_WORLD_TIME)
			world_time = MIDNIGHT_TICKS;
	}
	// Keep the legacy boolean cache synchronized with the Rust generation mode.
	// Library callers may still inspect `terrain` while the generator uses the
	// richer mode enum directly.
	void apply_mode_defaults() { terrain = generation_mode_terrain(mode); }
	bool mapillary_facades_wanted() const
	{
		return mapillary_facades_on() || mapillary_facades_dir.has_value();
	}
	bool mapillary_facades_on() const
	{
		return !building_facades && mapillary_facades.value_or(true) &&
			   mapillary_api_token().has_value();
	}
	std::optional<std::string> mapillary_api_token() const
	{
		if (mapillary_token && !mapillary_token->empty())
			return mapillary_token;
		if (const char *env = std::getenv("MAPILLARY_TOKEN"); env && *env)
			return std::string(env);
		return std::nullopt;
	}
	bool mapillary_pipeline_on() const
	{
		return mapillary_facades_on() && !mapillary_facades_dir.has_value();
	}

	// Enable interior generation (optional)
	bool interior{true};
	// Retained for parity with Rust argument validation; exporters may consume
	// these options when they support facade panels.
	bool building_facades{false};
	std::optional<bool> mapillary_facades{std::nullopt};
	bool mapillary_probe{false};
	std::optional<std::string> mapillary_token{std::nullopt};
	std::optional<std::string> mapillary_facades_dir{std::nullopt};
	std::optional<std::string> mapillary_facade_debug_dir{std::nullopt};
	std::string mapillary_facade_debug_walls;
	FacadeMode mapillary_facade_mode{FacadeMode::Photos};
	FacadeDetail facade_detail{FacadeDetail::Standard};
	std::uint32_t facade_px{16};
	std::optional<std::string> building_facades_dir{std::nullopt};

	// Enable roof generation (optional)
	bool roof{true};

	bool legacy_trees{false};
	bool canopy_height{true};
	bool overture{true};
	bool use_3d{true};
	bool disable_height_limit{false};
	bool aws_only_elevation{false};
	bool bake_lighting{false};
	bool map_preview{false};
	bool map_item{true};
	bool benchmark{false};
	bool voxy_lod{false};
	bool debug{false};
	std::optional<double> spawn_lat{std::nullopt};
	std::optional<double> spawn_lng{std::nullopt};
	double rotation{0.0};
	OvertureSource overture_source{OvertureSource::Auto};
	GameMode gamemode{GameMode::Creative};
	int64_t world_time{6000};
	SignageLevel signage{SignageLevel::Basic};

	// Enable filling ground (optional)
	bool fillground{false};

	// Enable city boundary ground generation (optional)
	// When enabled, detects building clusters and places stone ground in urban areas.
	// Isolated buildings in rural areas will keep grass around them.
	bool city_boundaries{true};

	// Set floodfill timeout (seconds) (optional)
	std::optional<std::chrono::milliseconds> timeout{std::nullopt};
	std::chrono::milliseconds timeout_ref() const
	{
		return timeout.value_or(std::chrono::milliseconds(3000));
	}
	// Rust's validate_args equivalent for library callers.  Keep validation
	// side-effect free so CLI adapters can turn the boolean into their own
	// diagnostic/error type.
	bool valid() const
	{
		// Planetary scales intentionally sit below the Earth object scale
		// minimum; Rust validates scale only for geographic Earth worlds.
		if ((is_earth(body) && !valid_scale(scale)) || timeout_ref().count() < 0)
			return false;
		if (bedrock && luanti)
			return false;
		if (legacy_terrain && mode == GenerationMode::GeoOnly)
			return false;
		if (map_preview && luanti)
			return false;
		// Web Mercator is retained for compatibility but Rust rejects it because
		// its latitude scale distorts terrain and object placement.
		if (projection == projection::ProjectionKind::WebMercator)
			return false;
		if ((mapillary_facades == std::optional<bool>(true) || mapillary_probe) &&
				!mapillary_api_token().has_value())
			return false;
		if (facade_px != 4 && facade_px != 8 && facade_px != 16 && facade_px != 32)
			return false;
		if (mapillary_facade_debug_walls.size() > 4096)
			return false;
		if (mapillary_facades_wanted() && (bedrock || luanti) &&
				facade_mode_places_displays(mapillary_facade_mode))
			return false;
		if (building_facades && (bedrock || luanti))
			return false;
		if (!mapillary_facade_debug_walls.empty() && !mapillary_facade_debug_dir)
			return false;
		if (mapillary_facade_debug_dir && !mapillary_pipeline_on())
			return false;
		if (mapillary_facades_dir &&
				!std::filesystem::is_directory(*mapillary_facades_dir))
			return false;
		return true;
	}
};
}
