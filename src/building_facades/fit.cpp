#include "fit.h"

#include <cmath>

namespace arnis::building_facades
{
namespace
{
double modulo(double value, double period)
{
	const auto result = std::fmod(value, period);
	return result < 0.0 ? result + period : result;
}
}

std::pair<double, Run> Fit::map_u(double metres) const
{
	const auto texture_width = std::max(0.001, entry.metres_wide);
	if (width <= texture_width)
		return {metres + (texture_width - std::max(0.0, width)) * .5, Run::Forward};
	const auto shifted = metres + phase;
	if (entry.tiles_horizontally)
		return {modulo(shifted, texture_width), Run::Forward};
	const auto period = texture_width * 2.0;
	const auto position = modulo(shifted, period);
	return position < texture_width ? std::pair{position, Run::Forward}
									: std::pair{period - position, Run::Mirrored};
}

double Fit::map_v(double metres) const
{
	const auto texture_height = std::max(0.001, entry.metres_tall);
	if (height <= texture_height)
		return metres;
	const auto storey = entry.storey_m();
	const auto ground = entry.has_ground_floor ? storey : 0.0;
	const auto upper = std::max(storey, texture_height - ground);
	return metres < ground ? metres : ground + modulo(metres - ground, upper);
}
} // namespace arnis::building_facades
