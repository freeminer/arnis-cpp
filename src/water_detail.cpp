#include "water_detail.h"

#include <algorithm>
#include <cmath>
#include <string_view>

#include "ground_generation.h"
#include "water_depth.h"

namespace arnis::water_detail
{
namespace
{
constexpr double BOWL_BELOW_SCALE = 0.5;
constexpr double DROWN_AT_OR_BELOW_SCALE = 0.5;
constexpr std::size_t MAX_DROWNED_RUN = 12;
constexpr std::size_t DRY_ENDS = 2;

bool tag_set(const tags_t &tags, std::string_view key)
{
	const auto it = tags.find(std::string(key));
	return it != tags.end() && it->second != "no";
}

bool zero_layer(const std::string &value)
{
	const auto first = value.find_first_not_of(" \t\n\r\f\v");
	if (first == std::string::npos)
		return false;
	const auto last = value.find_last_not_of(" \t\n\r\f\v");
	return last == first && value[first] == '0';
}

int bowl_steps(double d_blocks, double hw_blocks)
{
	constexpr double SHOAL_BLOCKS = 1.0;
	const double run = std::max(0.0, hw_blocks - SHOAL_BLOCKS);
	const int target = std::clamp(
			static_cast<int>(std::min(std::round(hw_blocks * 0.7), std::floor(run))), 0,
			BOWL_MAX_DEPTH);
	return std::clamp(static_cast<int>(std::floor(d_blocks - SHOAL_BLOCKS)), 0, target);
}
} // namespace

bool rounds_depth(WaterDetail detail)
{
	return detail == WaterDetail::Scaled;
}

bool uses_bowl(WaterDetail detail, double scale)
{
	return detail == WaterDetail::Scaled && scale < BOWL_BELOW_SCALE;
}

int channel_width_cap(WaterDetail detail, double scale)
{
	return detail == WaterDetail::Scaled && scale < 0.7 ? 3 : 0;
}

bool drowns_crossing(WaterDetail detail, double scale, const tags_t &tags)
{
	if (detail != WaterDetail::Scaled || !(scale <= DROWN_AT_OR_BELOW_SCALE))
		return false;
	for (const auto key : {"bridge", "tunnel", "covered", "embankment", "causeway"})
		if (tag_set(tags, key))
			return false;
	if (const auto it = tags.find("layer"); it != tags.end() && !zero_layer(it->second))
		return false;
	if (const auto it = tags.find("man_made");
			it != tags.end() &&
			(it->second == "dyke" || it->second == "embankment" || it->second == "pier" ||
					it->second == "breakwater" || it->second == "groyne"))
		return false;
	return true;
}

int max_carve_depth(WaterDetail detail, int estimate, double scale)
{
	if (detail == WaterDetail::Default)
		return estimate;
	if (!uses_bowl(detail, scale))
		return std::min(estimate + 1, 6);
	return BOWL_MAX_DEPTH;
}

int bowl_depth(int x, int z, std::uint16_t dt_units, std::uint16_t component_max_units,
		double scale)
{
	const double s = std::min(scale, 1.0);
	const int wavelength = std::max(12, static_cast<int>(std::round(12.0 / s)));
	const double wobble =
			(ground_generation::value_noise_01(x, z, wavelength) - 0.5) * (4.0 * s);
	const double d_blocks = (static_cast<double>(dt_units) + wobble) / 3.0;
	return bowl_steps(d_blocks, static_cast<double>(component_max_units) / 3.0);
}

std::vector<bool> drowned_points(const std::vector<std::pair<int, int>> &points,
		int half_width, const std::function<bool(int, int)> &wet)
{
	const std::size_t n = points.size();
	std::vector<bool> drowned(n, false), is_wet;
	is_wet.reserve(n);
	for (const auto &[x, z] : points)
		is_wet.push_back(wet(x, z));
	std::size_t i = 0;
	while (i < n) {
		if (!is_wet[i]) {
			++i;
			continue;
		}
		const std::size_t start = i;
		while (i < n && is_wet[i])
			++i;
		if (start < DRY_ENDS || i + DRY_ENDS > n || i - start > MAX_DROWNED_RUN)
			continue;
		std::size_t both_sides = 0;
		for (std::size_t k = start; k < i; ++k) {
			const auto &[ax, az] = points[k > 1 ? k - 2 : 0];
			const auto &[bx, bz] = points[std::min(k + 2, n - 1)];
			const double dx = static_cast<double>(bx - ax);
			const double dz = static_cast<double>(bz - az);
			const double len = std::max(1.0, std::hypot(dx, dz));
			const double reach = static_cast<double>(half_width + 2);
			const int px = static_cast<int>(std::round(-dz / len * reach));
			const int pz = static_cast<int>(std::round(dx / len * reach));
			const auto &[x, z] = points[k];
			if (wet(x + px, z + pz) && wet(x - px, z - pz))
				++both_sides;
		}
		if (both_sides * 3 >= (i - start) * 2)
			std::fill(drowned.begin() + start, drowned.begin() + i, true);
	}
	return drowned;
}

int CarveDepth::crossing(int estimate) const
{
	return std::max(max_carve_depth(detail, estimate, scale), floor);
}

int CarveDepth::bound(int estimate) const
{
	return estimate == 0 ? floor : crossing(estimate);
}

} // namespace arnis::water_detail
