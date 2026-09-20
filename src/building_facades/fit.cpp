#include "fit.h"

#include <algorithm>
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

Fit Fit::make(const Entry &source, double wall_width, double wall_height, double phase_m)
{
	return make(source, 1.0, wall_width, wall_height, phase_m);
}

Fit Fit::new_fit(const Entry &source, double pixels_per_metre, double wall_width,
		double wall_height, double phase_m)
{
	return make(source, pixels_per_metre, wall_width, wall_height, phase_m);
}

Fit Fit::make(const Entry &source, double pixels_per_metre, double wall_width,
		double wall_height, double phase_m)
{
	return Fit{source, std::max(0.0, pixels_per_metre), std::max(0.0, wall_width),
			std::max(0.0, wall_height), phase_m};
}

bool Fit::repeats_across() const
{
	return width > std::max(0.001, entry.metres_wide);
}

bool Fit::repeats_up() const
{
	return height > std::max(0.001, entry.metres_tall);
}

bool Fit::mirrors() const
{
	return repeats_across() && !entry.tiles_horizontally;
}

std::uint64_t Fit::region_key_at(
		double pixels_per_metre, double x0_m, double x1_m, double y0_m, double y1_m) const
{
	// Mirror Rust's region identity: hash the actual separable source lookup
	// tables, not floating-point metre bounds.  This lets adjacent walls share
	// an atlas crop when rounding makes their gathered pixels identical.
	const auto ppm = std::max(0.0, pixels_per_metre);
	const auto source_width = entry.scaled_size(ppm).first;
	const auto source_height = entry.scaled_size(ppm).second;
	if (!source_width || !source_height || ppm <= 0.0)
		return 0;
	Fit at = *this;
	at.pixels_per_metre = ppm;
	const auto out_width = static_cast<std::uint32_t>(
			std::clamp(std::llround((x1_m - x0_m) * ppm), 0LL, 1LL << 15));
	const auto out_height = static_cast<std::uint32_t>(
			std::clamp(std::llround((y1_m - y0_m) * ppm), 0LL, 1LL << 15));
	if (!out_width || !out_height)
		return 0;
	std::uint64_t key = 1469598103934665603ULL;
	const auto add = [&](std::uint64_t value, unsigned bytes) {
		for (unsigned i = 0; i != bytes; ++i) {
			key ^= static_cast<std::uint8_t>(value);
			key *= 1099511628211ULL;
			value >>= 8;
		}
	};
	add(source_width, 4);
	add(source_height, 4);
	add(out_width, 8);
	add(out_height, 8);
	for (std::uint32_t i = 0; i < out_width; ++i)
		add(static_cast<std::uint64_t>(at.source_x(
					x0_m + (static_cast<double>(i) + .5) / ppm, source_width)) *
						3,
				8);
	for (std::uint32_t j = 0; j < out_height; ++j)
		add(at.source_y(y1_m - (static_cast<double>(j) + .5) / ppm, source_height), 4);
	return key;
}

std::uint32_t Fit::source_x(double metres, std::uint32_t source_width) const
{
	if (!source_width)
		return 0;
	const auto [u, run] = map_u(metres);
	const auto scaled = static_cast<long long>(std::floor(u * pixels_per_metre));
	const auto clamped = std::clamp<long long>(scaled, 0, source_width - 1);
	return run == Run::Mirrored ? source_width - 1 - clamped : clamped;
}

std::uint32_t Fit::source_y(double metres, std::uint32_t source_height) const
{
	if (!source_height)
		return 0;
	const auto v = map_v(metres);
	const auto scaled = static_cast<long long>(std::floor(v * pixels_per_metre));
	// Image rows start at the top, while wall metres start at the foot.  Rust's
	// fitter therefore reads the corresponding row from the bottom upward.
	const auto from_bottom = std::clamp<long long>(scaled, 0, source_height - 1);
	return source_height - 1 - static_cast<std::uint32_t>(from_bottom);
}

std::array<std::uint32_t, 4> Fit::source_bounds(double x0, double x1, double y0,
		double y1, std::uint32_t source_width, std::uint32_t source_height) const
{
	const auto left = source_x(std::min(x0, x1), source_width);
	const auto right = source_x(std::max(x0, x1), source_width);
	const auto bottom = source_y(std::min(y0, y1), source_height);
	const auto top = source_y(std::max(y0, y1), source_height);
	return {std::min(left, right), std::max(left, right), std::min(bottom, top),
			std::max(bottom, top)};
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
	const auto ground = entry.ground_m();
	// Repeat a whole number of storeys above the optional ground-floor band;
	// `upper_m` is the shared Rust/C++ manifest invariant.
	const auto upper = entry.upper_m();
	return metres < ground ? metres : ground + modulo(metres - ground, upper);
}
} // namespace arnis::building_facades
