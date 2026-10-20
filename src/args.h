#pragma once

#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>

#include "celestial.h"
#include "projection/web_mercator.h"
#include "one_world.h"
#include "trees/tree_library.h"
#include "caves_biomes.h"

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
enum class WorldType
{
	Void,
	Flat
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
inline constexpr double MIN_HEIGHT_MULTIPLIER = 0.1;
inline constexpr double MAX_HEIGHT_MULTIPLIER = 10.0;
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
	auto normalized = value;
	const auto first = normalized.find_first_not_of(" \t\n\r\f\v");
	if (first == std::string::npos)
		return FacadeMode::Photos;
	const auto last = normalized.find_last_not_of(" \t\n\r\f\v");
	normalized = normalized.substr(first, last - first + 1);
	std::transform(normalized.begin(), normalized.end(), normalized.begin(),
			[](unsigned char c) {
				return static_cast<char>(c >= 'A' && c <= 'Z' ? c + ('a' - 'A') : c);
			});
	return normalized == "blocks" ? FacadeMode::Blocks : FacadeMode::Photos;
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
	// Optional custom world folder name (--world-name in the Rust CLI).
	std::optional<std::string> world_name{std::nullopt};

	// Generate a Bedrock Edition world (.mcworld) instead of Java Edition
	bool bedrock{false};
	// Select the Luanti-compatible output path when embedding Arnis as a library.
	bool luanti{false};

	// Downloader method (requests/curl/wget) (optional)
	std::string downloader{std::string("requests")};
	// OSM tile archive source. Rust tries this before Overpass unless disabled.
	std::string osm_tiles_url{"https://tiles.arnisproject.com/v1"};
	bool no_tile_archive{false};

	// World scale to use, in blocks per meter (1.0 = real size)
	double scale{1.0};
	// Vertical-only relief multiplier; horizontal geometry keeps using `scale`.
	double height_multiplier{1.0};
	// Moon and Mars use the same geographic projection but a body-specific
	// terrain scale/palette. Latitude is supplied by library callers for Mars'
	// polar caps when no geographic bbox is retained in Args.
	CelestialBody body{CelestialBody::Earth};
	projection::ProjectionKind projection{projection::ProjectionKind::Local};
	double celestial_latitude_degrees{0.0};
	// Fixed world frame inherited when this run extends a One World manifest.
	std::optional<one_world::RunContext> one_world_run;

	// Ground level to use in the Minecraft world
	int ground_level{-62};

	// Generation mode mirrors Rust. `terrain` remains as a compatibility cache for
	// the existing C++ processors and defaults to the Rust geo-terrain behaviour.
	GenerationMode mode{GenerationMode::GeoTerrain};
	bool terrain{true};
	bool legacy_terrain{false};
	// Largest schematic tree tier permitted by the Rust tree-pack selector.
	trees::TreeSize max_tree_size{trees::TreeSize::Giant};
	bool terrain_enabled() const { return mode != GenerationMode::GeoOnly; }
	bool skip_objects() const
	{
		return mode == GenerationMode::TerrainOnly || scale < OBJECT_SKIP_SCALE;
	}
	bool skip_objects_due_to_scale() const
	{
		return mode != GenerationMode::TerrainOnly && scale < OBJECT_SKIP_SCALE;
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
		// Cave carving and its themed decorations are Earth-only; do not carry
		// an explicit library setting onto Moon or Mars generation.
		caves = false;
		building_facades = false;
		mapillary_facades = false;
		disable_height_limit = false;
		// Preserve an explicitly selected time; otherwise airless worlds use
		// Minecraft midnight (Rust's MIDNIGHT_TICKS).
		if (world_time == DEFAULT_WORLD_TIME)
			world_time = MIDNIGHT_TICKS;
		// Roads and their signal/car extras are Earth-only. Match Rust's body
		// defaults even when a library caller enabled these before adapting the
		// requested celestial body.
		traffic_lights = false;
		moving_cars = false;
		// Body defaults replace the requested generation mode, so refresh the
		// compatibility cache after that normalization as well.
		apply_mode_defaults();
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
		const auto nonblank = [](const std::string &value) -> std::optional<std::string> {
			const auto first = value.find_first_not_of(" \t\n\r\f\v");
			if (first == std::string::npos)
				return std::nullopt;
			const auto last = value.find_last_not_of(" \t\n\r\f\v");
			return value.substr(first, last - first + 1);
		};
		if (mapillary_token)
			return nonblank(*mapillary_token);
		if (const char *env = std::getenv("MAPILLARY_TOKEN"); env)
			return nonblank(env);
		return std::nullopt;
	}
	bool mapillary_pipeline_on() const
	{
		return mapillary_facades_on() && !mapillary_facades_dir.has_value();
	}

	// Rust's CLI leaves interior generation off unless explicitly requested.
	bool interior{false};
	// Retained for parity with Rust argument validation; exporters may consume
	// these options when they support facade panels.
	bool building_facades{false};
	std::optional<bool> mapillary_facades{std::nullopt};
	bool mapillary_probe{false};
	std::optional<std::string> mapillary_token{std::nullopt};
	// Probe output directory, matching Rust's --mapillary-debug-dir.
	std::optional<std::string> mapillary_debug_dir{std::nullopt};
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
	WorldType world_type{WorldType::Void};
	SignageLevel signage{SignageLevel::Basic};
	// Java backend extras mirrored from Rust. Entity generation is opt-in.
	bool traffic_lights{false};
	bool moving_cars{false};

	// Enable filling ground (optional)
	bool fillground{false};
	// Minecraft-style cave generation options.  The CLI adapter may populate these;
	// the generator keeps them optional so existing library callers remain stable.
	bool caves{false};
	std::optional<std::string> cave_biomes{std::nullopt};
	std::optional<std::string> cave_asset_pack{std::nullopt};
	// Rust's diagnostic cave-zone preview controls. The renderer itself is
	// exposed as a library API; callers provide the already-projected block bbox.
	std::optional<std::filesystem::path> cave_zone_map{std::nullopt};
	std::optional<std::uint32_t> cave_zone_map_step{std::nullopt};

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
	std::optional<std::string> validation_error() const
	{
		// Planetary scales intentionally sit below the Earth object scale
		// minimum; Rust validates scale only for geographic Earth worlds.
		if ((is_earth(body) && !valid_scale(scale)) ||
				height_multiplier < MIN_HEIGHT_MULTIPLIER ||
				height_multiplier > MAX_HEIGHT_MULTIPLIER ||
				!std::isfinite(height_multiplier) || timeout_ref().count() < 0)
			return "invalid scale, height multiplier, or timeout";
		if (world_time < 0 || world_time >= 24000)
			return "world time must be between 0 and 23999";
		if (bedrock && luanti)
			return "Bedrock and Luanti output cannot both be enabled";
		if ((bedrock || luanti) && (traffic_lights || moving_cars))
			return "traffic lights and moving cars require Java Edition output";
		if (one_world_run && (traffic_lights || moving_cars))
			return "traffic lights and moving cars are not available with One World yet";
		if (spawn_lat.has_value() != spawn_lng.has_value())
			return "Both --spawn-lat and --spawn-lng must be provided together.";
		if (spawn_lat && spawn_lng &&
				(!std::isfinite(*spawn_lat) || !std::isfinite(*spawn_lng) ||
						*spawn_lat < -90.0 || *spawn_lat > 90.0 || *spawn_lng < -180.0 ||
						*spawn_lng > 180.0))
			return "Invalid spawn coordinates: latitude/longitude out of range";
		if (!std::isfinite(rotation) || rotation < -90.0 || rotation > 90.0)
			return "Rotation angle must be between -90 and 90 degrees.";
		const int floor = (disable_height_limit && !bedrock && !luanti ? -2032 : -64) + 2;
		const int top = !disable_height_limit || luanti ? 319 : bedrock ? 511 : 2031;
		const int ceiling = top - 15;
		if (ground_level < floor || ground_level > ceiling)
			return "--ground-level must be between " + std::to_string(floor) + " and " +
				   std::to_string(ceiling) + " for this world format (got " +
				   std::to_string(ground_level) + ").";
		if (legacy_terrain && mode == GenerationMode::GeoOnly)
			return "legacy terrain cannot be combined with geo-only mode";
		if (map_preview && luanti)
			return "map preview is not supported for Luanti output";
		if (one_world_run &&
				(bedrock || luanti || !is_earth(body) ||
						std::abs(rotation) > std::numeric_limits<double>::epsilon() ||
						mapillary_probe))
			return "One World mode conflicts with the selected format, body, rotation, or Mapillary probe";
		if (!one_world_run && world_name)
			return "--world-name only applies to --one-world.";
		// Rust accepts standalone Web Mercator and warns that scale varies with
		// latitude; the library validator must not turn that warning into a rejection.
		if ((mapillary_facades == std::optional<bool>(true) || mapillary_probe) &&
				!mapillary_api_token().has_value())
			return "Mapillary facade/probe mode requires an API token";
		if (mapillary_debug_dir && !mapillary_probe)
			return "Mapillary debug output requires probe mode";
		if (facade_px != 4 && facade_px != 8 && facade_px != 16 && facade_px != 32)
			return "facade pixel size must be 4, 8, 16, or 32";
		if (mapillary_facade_debug_walls.size() > 4096)
			return "Mapillary facade debug wall list exceeds 4096 characters";
		if (mapillary_facades_wanted() && (bedrock || luanti) &&
				facade_mode_places_displays(mapillary_facade_mode))
			return "Mapillary photo facades are incompatible with Bedrock/Luanti output";
		if (building_facades && (bedrock || luanti))
			return "building facades are incompatible with Bedrock/Luanti output";
		if (building_facades_dir) {
			std::error_code ec;
			if (!std::filesystem::is_directory(*building_facades_dir, ec))
				return "building facade directory does not exist or is not a directory";
		}
		const bool cave_preview = cave_zone_map.has_value();
		if (cave_biomes && !caves && !cave_preview)
			return "cave biome weights require cave generation or a cave-zone preview";
		if (cave_biomes) {
			std::string error;
			if (caves::BiomeAmounts::parse(*cave_biomes, &error), !error.empty())
				return "invalid cave biome weights: " + error;
		}
		if (cave_asset_pack && !caves)
			return "cave asset pack requires cave generation";
		if (cave_asset_pack &&
				!std::filesystem::is_regular_file(
						std::filesystem::path(*cave_asset_pack) / "cave_pack.json"))
			return "cave asset pack is missing cave_pack.json";
		if (cave_zone_map_step && !cave_preview)
			return "cave-zone-map step requires a cave-zone preview";
		if (cave_preview && rotation != 0.0)
			return "cave-zone preview does not support rotation";
		if (!mapillary_facade_debug_walls.empty() && !mapillary_facade_debug_dir)
			return "Mapillary facade debug walls require a debug output directory";
		if (mapillary_facade_debug_dir && !mapillary_pipeline_on())
			return "Mapillary facade debug output requires the Mapillary pipeline";
		if (mapillary_facades_dir &&
				!std::filesystem::is_directory(*mapillary_facades_dir))
			return "Mapillary facade export directory does not exist";
		return std::nullopt;
	}
	bool valid() const { return !validation_error().has_value(); }
};
}
