#include "source.h"

#include "bavaria.h"
#include "canada.h"
#include "france.h"
#include "switzerland.h"
#include "usa.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arnis::survey::source
{
const std::vector<Source> &sources()
{
	static const std::vector<Source> providers{bavaria(), france(), netherlands(),
			switzerland(), nrw(), estonia(), luxembourg(), usa(), canada(), latvia()};
	return providers;
}

const Source *for_bbox(const geographic::LLBBox &bbox)
{
	return for_bbox(bbox, sources());
}

bool Source::contains(double latitude, double longitude) const
{
	bool inside = false;
	for (const auto &points : rings) {
		if (points.empty())
			continue;
		std::size_t j = points.size() - 1;
		for (std::size_t i = 0; i < points.size(); ++i) {
			const auto yi = points[i].lat(), xi = points[i].lng();
			const auto yj = points[j].lat(), xj = points[j].lng();
			if ((yi > latitude) != (yj > latitude) &&
					longitude < (xj - xi) * (latitude - yi) / (yj - yi) + xi)
				inside = !inside;
			j = i;
		}
	}
	return inside;
}

double Source::depth(double latitude, double longitude) const
{
	constexpr double metres_per_degree_lat = 110574.0;
	constexpr double pi = 3.14159265358979323846;
	const double metres_per_degree_lon = 111320.0 * std::cos(latitude * pi / 180.0);
	double best = std::numeric_limits<double>::max();
	for (const auto &points : rings) {
		for (std::size_t i = 1; i < points.size(); ++i) {
			const double ax = (points[i - 1].lng() - longitude) * metres_per_degree_lon;
			const double ay = (points[i - 1].lat() - latitude) * metres_per_degree_lat;
			const double bx = (points[i].lng() - longitude) * metres_per_degree_lon;
			const double by = (points[i].lat() - latitude) * metres_per_degree_lat;
			const double dx = bx - ax, dy = by - ay;
			const double length_squared = dx * dx + dy * dy;
			const double t =
					length_squared > 0.0
							? std::clamp(-(ax * dx + ay * dy) / length_squared, 0.0, 1.0)
							: 0.0;
			best = std::min(best, std::hypot(ax + t * dx, ay + t * dy));
		}
	}
	return best;
}

bool Source::covers(const geographic::LLBBox &bbox) const
{
	const auto south = bbox.min().lat(), west = bbox.min().lng();
	const auto north = bbox.max().lat(), east = bbox.max().lng();
	const auto steps_lat = std::clamp<std::size_t>(
			static_cast<std::size_t>(std::ceil((north - south) / .00225)), 1, 400);
	const auto steps_lon = std::clamp<std::size_t>(
			static_cast<std::size_t>(std::ceil((east - west) / .0033)), 1, 400);
	for (std::size_t i = 0; i <= steps_lat; ++i) {
		const double latitude = south + (north - south) * i / steps_lat;
		for (std::size_t j = 0; j <= steps_lon; ++j) {
			const double longitude = west + (east - west) * j / steps_lon;
			if (contains(latitude, longitude))
				return true;
		}
	}
	return false;
}

const Source *for_bbox(const geographic::LLBBox &bbox, const std::vector<Source> &sources)
{
	const double latitude = (bbox.min().lat() + bbox.max().lat()) * .5;
	const double longitude = (bbox.min().lng() + bbox.max().lng()) * .5;
	const Coverage *deepest = nullptr;
	double deepest_distance = -1.0;
	for (const auto &source : sources) {
		if (!source.contains(latitude, longitude))
			continue;
		const double distance = source.depth(latitude, longitude);
		if (!deepest || distance >= deepest_distance) {
			deepest = &source;
			deepest_distance = distance;
		}
	}
	if (deepest)
		return deepest;
	for (const auto &source : sources)
		if (source.covers(bbox))
			return &source;
	return nullptr;
}

Sheet Sheet::at(const Source &coverage, double longitude)
{
	if (coverage.grid.kind == Grid::Kind::Nad83Utm)
		return {&coverage, {crs::Kind::Nad83Utm, crs::utm_zone(longitude)}, true};
	return {&coverage, coverage.grid.fixed, false};
}

std::string Sheet::cache_dir() const
{
	if (zone_specific)
		return source->id + "/utm" + std::to_string(projection.zone);
	return source->id;
}

std::vector<std::pair<std::int32_t, std::int32_t>> Sheet::tiles(const Rect &rect) const
{
	std::vector<std::pair<std::int32_t, std::int32_t>> out;
	const auto min_e = static_cast<std::int32_t>(std::floor(rect[0] / 1000.0));
	const auto max_e = static_cast<std::int32_t>(std::floor((rect[2] - 1e-6) / 1000.0));
	const auto min_n = static_cast<std::int32_t>(std::floor(rect[1] / 1000.0));
	const auto max_n = static_cast<std::int32_t>(std::floor((rect[3] - 1e-6) / 1000.0));
	constexpr std::array<std::pair<double, double>, 5> probes{
			{{0.5, 0.5}, {0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}}};
	for (std::int32_t east = min_e; east <= max_e; ++east) {
		for (std::int32_t north = min_n; north <= max_n; ++north) {
			const bool covered =
					std::any_of(probes.begin(), probes.end(), [&](const auto &probe) {
						const auto [lat, lon] =
								projection.inverse((east + probe.first) * 1000.0,
										(north + probe.second) * 1000.0);
						return source->contains(lat, lon);
					});
			if (covered)
				out.emplace_back(east, north);
		}
	}
	return out;
}

Rect Sheet::rect(const geographic::LLBBox &bbox, double margin_m) const
{
	const std::array corners{projection.forward(bbox.min().lat(), bbox.min().lng()),
			projection.forward(bbox.min().lat(), bbox.max().lng()),
			projection.forward(bbox.max().lat(), bbox.min().lng()),
			projection.forward(bbox.max().lat(), bbox.max().lng())};
	Rect bounds{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
			std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};
	for (const auto &[east, north] : corners) {
		bounds[0] = std::min(bounds[0], east);
		bounds[1] = std::min(bounds[1], north);
		bounds[2] = std::max(bounds[2], east);
		bounds[3] = std::max(bounds[3], north);
	}
	return {std::floor(bounds[0] - margin_m), std::floor(bounds[1] - margin_m),
			std::ceil(bounds[2] + margin_m), std::ceil(bounds[3] + margin_m)};
}
} // namespace arnis::survey::source
