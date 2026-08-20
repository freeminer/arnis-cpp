#pragma once
#include <vector>
#include <tuple>
namespace arnis::elevation
{
void repair_terrain_anomalies(std::vector<std::vector<double>> &heights);
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
std::tuple<std::vector<std::vector<double>>, double, double, int> scale_to_minecraft(
		const std::vector<std::vector<double>> &, double scale, int ground_level,
		int min_ground_level, bool disable_height_limit, int extended_max_y);
}
