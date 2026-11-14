#include "llbbox.h"
#include <array>
#include <charconv>
#include <cmath>
#include <stdexcept>
#include <string>
namespace arnis::geographic
{
LLBBox::LLBBox(double a, double b, double c, double d) : min_(a, b), max_(c, d)
{
	if (b >= d || a >= c)
		throw std::invalid_argument("invalid geographic bbox");
}
LLBBox LLBBox::from_str(const std::string &s)
{
	std::array<double, 4> values{};
	std::size_t count = 0;
	std::size_t begin = 0;
	const auto parse_field = [&](std::size_t end) {
		if (end == begin)
			return;
		if (count == values.size())
			throw std::invalid_argument("Invalid LLBBox: expected 4 values, got more");
		const std::string field = s.substr(begin, end - begin);
		double value = 0.0;
		const auto [parsed_end, error] =
				std::from_chars(field.data(), field.data() + field.size(), value);
		if (error != std::errc{} || parsed_end != field.data() + field.size())
			throw std::invalid_argument(
					"Invalid LLBBox: '" + field + "' is not a number");
		if (!std::isfinite(value))
			throw std::invalid_argument(
					"Invalid LLBBox: '" + field + "' is not a finite number");
		values[count++] = value;
	};
	for (std::size_t i = 0; i <= s.size(); ++i) {
		if (i != s.size() && s[i] != ',' && s[i] != ' ')
			continue;
		parse_field(i);
		begin = i + 1;
	}
	if (count != values.size())
		throw std::invalid_argument(
				"Invalid LLBBox: expected 4 values, got " + std::to_string(count));
	return LLBBox(values[0], values[1], values[2], values[3]);
}
bool LLBBox::contains(const LLPoint &p) const
{
	return p.lat() >= min_.lat() && p.lat() <= max_.lat() && p.lng() >= min_.lng() &&
		   p.lng() <= max_.lng();
}

double LLBBox::area_km2() const
{
	const double middle_lat = (min_.lat() + max_.lat()) * 0.5 * std::acos(-1.0) / 180.0;
	const double width_m = (max_.lng() - min_.lng()) * 111320.0 * std::cos(middle_lat);
	const double height_m = (max_.lat() - min_.lat()) * 111320.0;
	return std::abs(width_m * height_m) / 1000000.0;
}
}
