#pragma once
#include <cstddef>
#include <vector>
namespace arnis::mapillary
{
double weighted_median(
		const std::vector<double> &values, const std::vector<double> &weights);
}
