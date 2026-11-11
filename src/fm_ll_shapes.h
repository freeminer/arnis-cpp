#pragma once

#include "coordinate_system/geographic/llbbox.h"
#include "coordinate_system/geographic/llpoint.h"

#include <optional>
#include <string>
#include <vector>

namespace arnis
{
class LLShapes
{
public:
	using Ring = std::vector<geographic::LLPoint>;
	using Polygon = std::vector<Ring>;
	using Polygons = std::vector<Polygon>;
	static constexpr std::size_t MAX_POINTS = 200'000;

	static std::optional<LLShapes> create(
			const std::vector<Polygon> &polygons, std::string *error = nullptr);
	static std::optional<std::vector<std::pair<double, double>>> parse_points(
			const std::string &text, std::string *error = nullptr);
	static std::optional<std::vector<Polygon>> from_geojson(
			const std::string &text, std::string *error = nullptr);
	const Polygons &polygons() const { return polygons_; }
	geographic::LLBBox bbox() const;
	bool contains(const geographic::LLPoint &point) const;

private:
	Polygons polygons_;
};
}
