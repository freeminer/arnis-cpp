#pragma once
#include "choose.h"
#include <array>
namespace arnis::building_facades
{
enum class Run
{
	Forward,
	Mirrored
};
struct Fit
{
	Entry entry;
	double pixels_per_metre = 1.0, width = 0.0, height = 0.0, phase = 0.0;
	static Fit new_fit(const Entry &, double pixels_per_metre, double wall_width,
			double wall_height, double phase_m);
	static Fit make(const Entry &, double pixels_per_metre, double wall_width,
			double wall_height, double phase_m);
	static Fit make(const Entry &, double wall_width, double wall_height, double phase_m);
	bool repeats_across() const;
	bool repeats_up() const;
	bool mirrors() const;
	std::pair<double, Run> map_u(double) const;
	double map_v(double) const;
	std::uint32_t source_x(double, std::uint32_t) const;
	std::uint32_t source_y(double, std::uint32_t) const;
	std::array<std::uint32_t, 4> source_bounds(
			double, double, double, double, std::uint32_t, std::uint32_t) const;
	// Stable identity for the source pixel region read by a wall crop.  The
	// renderer can use this before decoding the image to share atlas regions.
	std::uint64_t region_key_at(double pixels_per_metre, double x0_m, double x1_m,
			double y0_m, double y1_m) const;
};
}
