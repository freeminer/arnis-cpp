#pragma once
#include <cstddef>
#include <vector>
namespace arnis::mapillary
{
double weighted_median(
		const std::vector<double> &values, const std::vector<double> &weights);
double geometric_score(double distance_m, double incidence_deg, double angular_width_deg);
}
