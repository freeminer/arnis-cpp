#pragma once

#include "source.h"

#include <nlohmann/json.hpp>

#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace arnis::survey::index
{
// One file or complete survey footprint from a GeoJSON index. Rings retain
// GeoJSON's (longitude, latitude) ordering; containment takes (latitude,
// longitude), as the other survey geographic APIs do.
struct Footprint
{
	std::string url;
	std::string survey;
	std::uint16_t year = 0;
	double density = 0.0;
	std::array<double, 4> bbox{}; // west, south, east, north
	std::vector<std::vector<std::pair<double, double>>> rings;

	static std::optional<Footprint> from_geojson(const std::string &url,
			const std::string &survey, std::uint16_t year, const nlohmann::json &geometry,
			double point_count);
	bool contains(double latitude, double longitude) const;
};

std::uint16_t survey_year(const std::string &name);

// Pick one survey per 1km tile, favoring the newest/densest survey among those
// whose coverage reaches at least 75% of the best candidate's sampled coverage.
source::TileUrls choose(const std::vector<Footprint> &footprints,
		const source::Sheet &sheet, const source::Rect &rect);

// Load a GeoJSON index from the survey cache, refreshing it after max_age_days.
// If refresh fails, a previously cached document is still returned.
std::variant<nlohmann::json, std::string> cached_geojson(
		const std::string &provider, const std::string &url, std::uint64_t max_age_days);
} // namespace arnis::survey::index
