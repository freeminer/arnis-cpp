#include "heights.h"

#include "../../arnis_ground.h"
#include "../floodfill_cache.h"
#include "../osm_parser.h"
#include "../overture.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace arnis::survey::heights
{
namespace
{
constexpr double MIN_COVER = 0.6;
constexpr std::size_t MIN_BLOCKS = 8;
constexpr float SATURATED = 51.0f;
constexpr double HOUSE_FOOTPRINT_M2 = 250.0;
constexpr unsigned HOME_STOREYS = 7;
constexpr float TALL_STOREY_M = 4.0f;
constexpr float WING_STEP_M = 1.5f;

struct Measured
{
	float height;
	bool flat;
	float top;
};

bool inside(const std::vector<std::pair<double, double>> &ring, double x, double z)
{
	if (ring.size() < 3)
		return false;
	bool odd = false;
	std::size_t j = ring.size() - 1;
	for (std::size_t i = 0; i < ring.size(); ++i) {
		const auto [xi, zi] = ring[i];
		const auto [xj, zj] = ring[j];
		if ((zi > z) != (zj > z) && x < (xj - xi) * (z - zi) / (zj - zi) + xi)
			odd = !odd;
		j = i;
	}
	return odd;
}

float quantile(const std::vector<float> &sorted, double q)
{
	return sorted[static_cast<std::size_t>(std::llround((sorted.size() - 1) * q))];
}

std::vector<float> main_roof(const std::vector<float> &roof)
{
	const auto n = roof.size();
	const auto k = std::max<std::size_t>(n / 20, 1);
	std::size_t from = 0;
	for (std::size_t i = n / 20; i <= n / 2; ++i)
		if (i + k < n && roof[i + k] - roof[i] >= WING_STEP_M)
			from = i + 1;
	if (from > 0 && n - from >= n / 2)
		return {roof.begin() + static_cast<std::ptrdiff_t>(from), roof.end()};
	return roof;
}

float median(std::vector<float> &values)
{
	std::sort(values.begin(), values.end());
	return values[values.size() / 2];
}

std::optional<Measured> measure(
		const ProcessedWay &way, const Ground &ground, const ::XZBBox &bbox, double scale)
{
	std::vector<std::pair<double, double>> points;
	points.reserve(way.nodes.size());
	for (const auto &node : way.nodes)
		points.emplace_back(node.x, node.z);
	if (points.size() < 4)
		return std::nullopt;
	double x0 = points.front().first, x1 = x0, z0 = points.front().second, z1 = z0;
	for (const auto &[x, z] : points) {
		x0 = std::min(x0, x);
		x1 = std::max(x1, x);
		z0 = std::min(z0, z);
		z1 = std::max(z1, z);
	}
	if (x1 < bbox.min_x() || x0 > bbox.max_x() || z1 < bbox.min_z() || z0 > bbox.max_z())
		return std::nullopt;
	const int min_x =
			static_cast<int>(std::max(std::floor(x0), static_cast<double>(bbox.min_x())));
	const int max_x =
			static_cast<int>(std::min(std::ceil(x1), static_cast<double>(bbox.max_x())));
	const int min_z =
			static_cast<int>(std::max(std::floor(z0), static_cast<double>(bbox.min_z())));
	const int max_z =
			static_cast<int>(std::min(std::ceil(z1), static_cast<double>(bbox.max_z())));
	std::size_t blocks = 0;
	std::vector<float> roof, rim, middle;
	for (int z = min_z; z <= max_z; ++z) {
		for (int x = min_x; x <= max_x; ++x) {
			const double cx = x + .5, cz = z + .5;
			if (!inside(points, cx, cz))
				continue;
			++blocks;
			const auto cell = ground.laser_cell({x - bbox.min_x(), z - bbox.min_z()});
			if (!cell || cell->kind != Kind::Building || cell->height <= 1.5f)
				continue;
			roof.push_back(cell->height);
			const bool on_rim =
					!inside(points, cx + 1, cz) || !inside(points, cx - 1, cz) ||
					!inside(points, cx, cz + 1) || !inside(points, cx, cz - 1);
			(on_rim ? rim : middle).push_back(cell->height);
		}
	}
	if (blocks < MIN_BLOCKS || roof.size() < MIN_COVER * blocks || roof.empty())
		return std::nullopt;
	std::sort(roof.begin(), roof.end());
	const float top = quantile(roof, .95);
	const auto principal = main_roof(roof);
	const float eave = quantile(principal, .1);
	const bool flat_main = quantile(roof, .5) >= top - 1.0f;
	const auto building = way.tags.find("building");
	const std::string kind = building == way.tags.end() ? "yes" : building->second;
	static const std::vector<std::string> houses = {"house", "detached",
			"semidetached_house", "terrace", "bungalow", "villa", "farm", "cabin"};
	const bool house = std::find(houses.begin(), houses.end(), kind) != houses.end() ||
					   (scale > 0 && blocks / (scale * scale) <= HOUSE_FOOTPRINT_M2);
	const bool level =
			!middle.empty() && !rim.empty() && median(middle) - median(rim) < .1f;
	const bool flat =
			house ? (top - eave < 1.0f && level) : (top - eave < 1.0f || flat_main);
	return Measured{flat ? top : std::max(eave, 2.0f), flat, top};
}

std::optional<unsigned> storeys(const std::string &kind, float wall_m)
{
	static const std::vector<std::string> homes = {"house", "detached",
			"semidetached_house", "terrace", "bungalow", "villa", "residential",
			"apartments", "dormitory"};
	if (wall_m < 3.5f)
		return std::nullopt;
	const auto levels =
			std::max(1u, static_cast<unsigned>(std::lround((wall_m - .4f) / 2.8f)));
	if (std::find(homes.begin(), homes.end(), kind) != homes.end())
		return levels;
	if (kind != "yes")
		return std::nullopt;
	if (levels <= HOME_STOREYS)
		return levels;
	const auto tall = static_cast<unsigned>(std::lround((wall_m - 1.0f) / TALL_STOREY_M));
	return tall > HOME_STOREYS ? std::optional<unsigned>(tall) : std::nullopt;
}

void fill(ProcessedWay &way, float height, bool flat)
{
	const auto it = way.tags.find("building");
	const std::string kind = it == way.tags.end() ? "yes" : it->second;
	if (const auto levels = storeys(kind, height))
		way.tags.insert_or_assign("building:levels", std::to_string(*levels));
	else {
		char formatted[32];
		std::snprintf(formatted, sizeof(formatted), "%.1f", height);
		way.tags.insert_or_assign("height", formatted);
	}
	if (flat && !way.tags.contains("roof:shape"))
		way.tags.insert_or_assign("roof:shape", "flat");
}
} // namespace

ApplyResult apply(std::vector<ProcessedElement> &elements, const Ground &ground,
		const ::XZBBox &bbox, double scale)
{
	ApplyResult result;
	if (!ground.has_laser() || !ground.laser_heights_wanted())
		return result;
	for (std::size_t i = 0; i < elements.size(); ++i) {
		auto *way = std::get_if<ProcessedWay>(static_cast<variant_t *>(&elements[i]));
		// Rust measures the parsed OSM set before appending Overture's distinct
		// footprints. The host caches those together for chunk selection, so use
		// Overture's reserved ID bit to preserve that source boundary here.
		if (!way || (way->id & ::arnis::overture::OVERTURE_ID_HIGH_BIT) != 0 ||
				!way->tags.contains("building") || way->tags.contains("height") ||
				way->tags.contains("building:levels") ||
				way->tags.contains("building:part"))
			continue;
		if (const auto measured = measure(*way, ground, bbox, scale)) {
			if (measured->top >= SATURATED)
				result.deferred.push_back({i, measured->height, measured->flat});
			else {
				fill(*way, measured->height, measured->flat);
				++result.filled;
			}
		}
	}
	return result;
}

std::size_t apply_deferred(
		std::vector<ProcessedElement> &elements, const std::vector<Deferred> &deferred)
{
	std::size_t filled = 0;
	for (const auto &item : deferred) {
		if (item.index >= elements.size())
			continue;
		auto *way = std::get_if<ProcessedWay>(
				static_cast<variant_t *>(&elements[item.index]));
		if (!way || way->tags.contains("height") || way->tags.contains("building:levels"))
			continue;
		fill(*way, item.height, item.flat);
		++filled;
	}
	return filled;
}

} // namespace arnis::survey::heights
