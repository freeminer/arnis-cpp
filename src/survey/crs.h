#pragma once

#include "utm.h"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace arnis::survey::crs
{
enum class Kind
{
	Utm,
	Lambert93,
	RdNew,
	Lv95,
	Lest97,
	Luref,
	Nad83Utm,
	WebMercator,
	Lks92
};

struct Crs
{
	Kind kind = Kind::Utm;
	unsigned zone = 0;
	std::pair<double, double> forward(double latitude, double longitude) const;
	std::pair<double, double> inverse(double easting, double northing) const;
	std::uint32_t epsg() const;
	bool north_first() const;
};

unsigned utm_zone(double longitude);
std::optional<Crs> from_wkt(const std::string &wkt);

// Local affine approximation `to = M * from + translation`.
struct Affine
{
	std::array<double, 6> m{};
	static Affine fit(Crs from, Crs to, double x, double y, double step);
	std::pair<double, double> apply(double x, double y) const;
	Affine inverse() const;
	std::array<double, 4> rect(double x0, double y0, double x1, double y1) const;
};
} // namespace arnis::survey::crs
