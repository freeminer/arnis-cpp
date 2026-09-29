#include "refine_stats.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <limits>
namespace arnis::mapillary
{
double weighted_median(
		const std::vector<double> &values, const std::vector<double> &weights)
{
	if (values.empty() || values.size() != weights.size())
		return std::numeric_limits<double>::quiet_NaN();
	std::vector<std::pair<double, double>> v;
	v.reserve(values.size());
	for (size_t i = 0; i < values.size(); ++i)
		if (std::isfinite(values[i]) && weights[i] > 0 && std::isfinite(weights[i]))
			v.emplace_back(values[i], weights[i]);
	if (v.empty())
		return std::numeric_limits<double>::quiet_NaN();
	std::sort(v.begin(), v.end());
	double total = 0;
	for (auto &p : v)
		total += p.second;
	double acc = 0;
	for (auto &p : v) {
		acc += p.second;
		if (acc * 2 >= total)
			return p.first;
	}
	return v.back().first;
}
double geometric_score(double distance_m, double incidence_deg, double angular_width_deg)
{
	if (!std::isfinite(distance_m) || !std::isfinite(incidence_deg) ||
			!std::isfinite(angular_width_deg))
		return 0;
	const double distance_term =
			std::clamp(1.0 - std::abs(distance_m - 12.0) / 30.0, 0.2, 1.0);
	return std::cos(incidence_deg * 3.141592653589793 / 180.0) * distance_term *
		   std::min(angular_width_deg / 40.0, 1.0);
}
}
