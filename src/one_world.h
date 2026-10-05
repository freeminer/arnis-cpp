#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <optional>
#include <filesystem>
#include <vector>
#include <array>

namespace arnis::one_world
{
inline constexpr const char *MANIFEST_FILE = "arnis_one_world.json";
inline constexpr const char *PREVIEW_DIR = "arnis_one_world/previews";
inline constexpr std::uint32_t MANIFEST_VERSION = 3;
inline constexpr int CLIP_PAD_BLOCKS = 64;
inline int ground_pad_blocks(double scale)
{
	return std::max(96, int(std::ceil(100.0 * scale)));
}
inline double stable(double value, int decimals)
{
	const double factor = std::pow(10.0, decimals);
	return std::round(value * factor) / factor;
}
struct GeneratedArea
{
	std::uint32_t id = 0;
	std::uint64_t generated_at = 0;
	std::string arnis_version;
	int min_x = 0, min_z = 0, max_x = -1, max_z = -1;
	double min_lat = 0, min_lon = 0, max_lat = 0, max_lon = 0;
	std::optional<std::string> preview;
	bool contains(int x0, int z0, int x1, int z1) const
	{
		return x0 <= min_x && z0 <= min_z && x1 >= max_x && z1 >= max_z;
	}
};
struct SoftTop
{
	double knee_m = 0;
	double width_blocks = 0;
};
struct ElevationAffine
{
	double min_height_m = 0;
	double blocks_per_meter = 0;
	int ground_level = 0;
	std::optional<SoftTop> soft_top;
	double y_for_metres(double height_m) const
	{
		if (soft_top && blocks_per_meter > 0.0) {
			const double knee_y =
					ground_level + (soft_top->knee_m - min_height_m) * blocks_per_meter;
			if (height_m > soft_top->knee_m)
				return knee_y +
					   soft_top->width_blocks *
							   std::asinh((height_m - soft_top->knee_m) *
										  blocks_per_meter / soft_top->width_blocks);
		}
		return ground_level + (height_m - min_height_m) * blocks_per_meter;
	}
};
struct Manifest
{
	std::uint32_t version = MANIFEST_VERSION;
	std::string created_with;
	std::uint64_t created_at = 0;
	double origin_lat = 0, origin_lon = 0, scale = 1;
	int ground_level = 0;
	bool terrain = true, disable_height_limit = false, aws_only_elevation = false;
	double height_multiplier = 1;
	std::optional<ElevationAffine> elevation;
	std::uint32_t next_area_id = 1;
	std::vector<GeneratedArea> areas;
	static std::filesystem::path path_in(const std::filesystem::path &world);
	static std::optional<Manifest> load(
			const std::filesystem::path &world, std::string *error = nullptr);
	bool save(const std::filesystem::path &world, std::string *error = nullptr) const;
	std::optional<std::array<int, 4>> extent() const;
	bool valid(std::string *error = nullptr) const;
};
std::uint64_t existing_chunks(const std::filesystem::path &, int, int, int, int);
std::optional<std::filesystem::path> safe_preview_path(
		const std::filesystem::path &, const std::string &);
struct RunContext
{
	std::filesystem::path world_dir;
	double origin_lat = 0, origin_lon = 0;
	std::optional<ElevationAffine> elevation;
	bool extending = false;
	std::uint64_t replaced_chunks = 0;
	std::uint32_t area_id = 0;
	std::filesystem::path preview_path() const
	{
		return world_dir / PREVIEW_DIR / ("area-" + std::to_string(area_id) + ".png");
	}
};
}
