#pragma once
#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
#include <vector>
#include <tuple>

namespace arnis::land_cover
{
struct LandCoverData;
}

namespace arnis::elevation
{
struct SoftTop
{
	double knee_m{0.0};
	double width_blocks{0.0};
};

struct ElevationAffine
{
	double min_height_m{0.0};
	double blocks_per_meter{0.0};
	int ground_level{0};
	std::optional<SoftTop> soft_top;

	double y_for_metres(double height_m) const;
	static ElevationAffine whole_earth(double scale, int floor, int extended_max_y);
};

struct AffinePolicy
{
	enum class Kind
	{
		Fit,
		FitWithHeadroom,
		Fixed
	};
	Kind kind{Kind::Fit};
	ElevationAffine fixed{};

	static AffinePolicy fit() { return {}; }
	static AffinePolicy fit_with_headroom() { return {Kind::FitWithHeadroom, {}}; }
	static AffinePolicy fixed_mapping(ElevationAffine value)
	{
		return {Kind::Fixed, std::move(value)};
	}
};

void repair_terrain_anomalies(
		std::vector<std::vector<double>> &heights, double meters_per_cell = 0.0);
void apply_land_cover_repair(std::vector<std::vector<double>> &heights,
		land_cover::LandCoverData &land_cover, double built_up_sigma_cells,
		std::uint32_t coastal_pull_distance_cells, double meters_per_cell,
		const std::function<void(double)> &report = {});
}
namespace arnis::elevation
{
void fill_nan_values(std::vector<std::vector<double>> &heights);
}
namespace arnis::elevation
{
void filter_elevation_outliers(std::vector<std::vector<double>> &heights);
}
namespace arnis::elevation
{
std::pair<std::vector<std::vector<double>>, ElevationAffine> scale_to_minecraft_with(
		const std::vector<std::vector<double>> &, double scale, int ground_level,
		int min_ground_level, bool disable_height_limit, int extended_max_y,
		const AffinePolicy &policy = {});

std::tuple<std::vector<std::vector<double>>, double, double, int> scale_to_minecraft(
		const std::vector<std::vector<double>> &, double scale, int ground_level,
		int min_ground_level, bool disable_height_limit, int extended_max_y);
}
