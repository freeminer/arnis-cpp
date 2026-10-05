#include "refine_stats.h"
#include "imgops.h"
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <utility>
namespace arnis::mapillary
{
namespace
{
// Visibility scoring lives in visibility.cpp; keep refinement statistics free
// of a second geometric_score definition.
double interp_median(
		double x, const std::vector<double> &xs, const std::vector<double> &ys)
{
	if (xs.empty())
		return std::numeric_limits<double>::quiet_NaN();
	if (x <= xs.front())
		return ys.front();
	if (x >= xs.back())
		return ys.back();
	auto upper = std::upper_bound(xs.begin(), xs.end(), x);
	const auto hi = static_cast<std::size_t>(upper - xs.begin());
	const auto lo = hi - 1;
	const double t = (x - xs[lo]) / std::max(xs[hi] - xs[lo], 1e-300);
	return ys[lo] + t * (ys[hi] - ys[lo]);
}

std::uint64_t total_order_key(double value)
{
	const auto bits = std::bit_cast<std::uint64_t>(value);
	return (bits >> 63) ? ~bits : (bits ^ (std::uint64_t{1} << 63));
}
} // namespace

double weighted_median(
		const std::vector<double> &values, const std::vector<double> &weights)
{
	if (values.empty() || values.size() != weights.size())
		return std::numeric_limits<double>::quiet_NaN();
	std::vector<std::size_t> order(values.size());
	std::iota(order.begin(), order.end(), 0);
	std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
		return total_order_key(values[a]) < total_order_key(values[b]);
	});
	std::vector<double> sorted_values, sorted_weights;
	sorted_values.reserve(values.size());
	sorted_weights.reserve(weights.size());
	for (const auto i : order) {
		sorted_values.push_back(values[i]);
		sorted_weights.push_back(weights[i]);
	}
	const double total =
			std::accumulate(sorted_weights.begin(), sorted_weights.end(), 0.0);
	if (!std::isfinite(total) || total <= 0.0)
		return imgops::median(std::move(sorted_values));
	std::vector<double> positions;
	positions.reserve(sorted_weights.size());
	double cumulative = 0.0;
	for (const double weight : sorted_weights) {
		positions.push_back((cumulative + 0.5 * weight) / total);
		cumulative += weight;
	}
	return interp_median(0.5, positions, sorted_values);
}
}
