#include "peaks.h"

#include "../coordinate_system/geographic/llbbox.h"
#include "../building_suppression.h"
#include "../data_processing.h"
#include "../element_processing/attractions.h"
#include "../element_processing/buildings.h"
#include "../element_processing/stations.h"
#include "../landmarks.h"
#include "../models_3d/wikidata/wikidata_index.h"
#include "../mapillary/geometry.h"
#include "../osm_parser.h"
#include "../overture.h"
#include "../strict_parse.h"
#include "../unicode_text.h"
#include "postprocess.h"
#include "../land_cover/land_cover.h"
#include "../progress.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace arnis::elevation
{
namespace
{
constexpr double SIGMA_M = 60.0;
constexpr double CONE_SLOPE = 1.0;
constexpr double SEARCH_M = 15.0;
constexpr double MIN_DEFICIT_M = 3.0;
constexpr double MAX_DEFICIT_M = 400.0;
constexpr double RELIEF_M = 500.0;
constexpr double RELIEF_SAMPLES = 50.0;
constexpr double DOME_SEARCH_M = 45.0;
constexpr double NEARBY_M = 300.0;
constexpr double LEVEL_SHARE = 0.25;
constexpr std::size_t SIDES = 16;
constexpr std::size_t HOLLOW_SIDES = 12;
constexpr double SUMMIT_NOISE_M = 10.0;
constexpr double STRETCH_RADIUS_M = 200.0;
constexpr double STRETCH_REACH_M = 1.5 * STRETCH_RADIUS_M;
constexpr double STRETCH_DEPTH = 2.0;
constexpr double PLATEAU_M = 100.0;

std::ptrdiff_t radius_cells(double meters, double meters_per_cell)
{
	const double cells = std::ceil(meters / meters_per_cell);
	if (std::isnan(cells) || cells <= 0.0)
		return 0;
	if (!std::isfinite(cells) ||
			cells >= static_cast<double>(std::numeric_limits<std::ptrdiff_t>::max()))
		return std::numeric_limits<std::ptrdiff_t>::max();
	return static_cast<std::ptrdiff_t>(cells);
}

std::optional<double> parse_elevation(const std::string &input)
{
	std::string value = input;
	auto trim = [](std::string &s) {
		const auto [begin, end] = mapillary::geodesy::trim_rust_whitespace(s);
		s = s.substr(begin, end - begin);
	};
	trim(value);
	if (!value.empty() && value.back() == 'm') {
		value.pop_back();
		trim(value);
	}
	if (value.empty())
		return std::nullopt;
	const auto parsed = strict_parse::f64(value);
	if (!parsed || !std::isfinite(*parsed))
		return std::nullopt;
	return *parsed;
}

// Rust's Wikidata tall-model fallback uses placement::parse_meters: unlike a
// surveyed summit's `ele`, an OSM model `height` must be positive, and may
// carry either lower- or upper-case metre suffixes.
std::optional<double> parse_model_height(const std::string &input)
{
	const auto [begin, end] = unicode_text::trim_whitespace(input);
	std::string_view value(input.data() + begin, end - begin);
	while (!value.empty() && (value.back() == 'm' || value.back() == 'M'))
		value.remove_suffix(1);
	const auto [number_begin, number_end] = unicode_text::trim_whitespace(value);
	value = value.substr(number_begin, number_end - number_begin);
	const auto parsed = strict_parse::f64(value);
	if (!parsed || !std::isfinite(*parsed) || *parsed <= 0.0)
		return std::nullopt;
	return *parsed;
}

std::optional<double> model_height(const tags_t &tags)
{
	const auto qid = tags.find("wikidata");
	if (qid == tags.end())
		return std::nullopt;
	const auto [first, last] = unicode_text::trim_whitespace(qid->second);
	if (first == last)
		return std::nullopt;
	const auto *entry =
			models_3d::lookup_wikidata(qid->second.substr(first, last - first));
	// Rust's terrain fit reserves space only for models in the bundled
	// Wikidata index. An arbitrary/unknown QID with a height tag is not a model
	// that the placement pipeline can actually generate.
	if (!entry)
		return std::nullopt;
	if (entry->height_m)
		return entry->height_m;
	const auto height = tags.find("height");
	return height == tags.end() ? std::nullopt : parse_model_height(height->second);
}

template <typename F>
void cells_within(std::size_t cx, std::size_t cy, std::ptrdiff_t radius,
		std::size_t width, std::size_t height, F &&fn)
{
	const auto r = static_cast<std::size_t>(radius);
	const auto x0 = cx > r ? cx - r : 0;
	const auto x1 = r >= width - 1 - cx ? width - 1 : cx + r;
	const auto y0 = cy > r ? cy - r : 0;
	const auto y1 = r >= height - 1 - cy ? height - 1 : cy + r;
	for (auto y = y0;;) {
		for (auto x = x0;;) {
			fn(x, y);
			if (x == x1)
				break;
			++x;
		}
		if (y == y1)
			break;
		++y;
	}
}

double local_relief(const std::vector<std::vector<double>> &grid, std::size_t cx,
		std::size_t cy, double meters_per_cell)
{
	const auto height = grid.size();
	const auto width = grid.front().size();
	const auto radius = radius_cells(RELIEF_M, meters_per_cell);
	const auto step = std::max<std::size_t>(
			1, static_cast<std::size_t>(
					   std::ceil(static_cast<double>(radius) / RELIEF_SAMPLES)));
	const auto r = static_cast<std::size_t>(radius);
	const auto x0 = cx > r ? cx - r : 0;
	const auto y0 = cy > r ? cy - r : 0;
	const auto x1 = r >= width - 1 - cx ? width - 1 : cx + r;
	const auto y1 = r >= height - 1 - cy ? height - 1 : cy + r;
	double lo = std::numeric_limits<double>::max();
	double hi = std::numeric_limits<double>::lowest();
	for (std::size_t y = y0;;) {
		for (std::size_t x = x0;;) {
			const auto v = grid[y][x];
			if (std::isfinite(v)) {
				lo = std::min(lo, v);
				hi = std::max(hi, v);
			}
			if (step > x1 - x)
				break;
			x += step;
		}
		if (step > y1 - y)
			break;
		y += step;
	}
	return std::isfinite(lo) && std::isfinite(hi) ? std::max(hi - lo, 0.0) : 0.0;
}

struct Summit
{
	double gx{}, gy{}, ele{};
};

struct Shape
{
	enum class Kind
	{
		Stretch,
		Bump
	};
	Kind kind = Kind::Bump;
	double base{}, gain{}, lift{};
};

struct Lift
{
	double gx{}, gy{}, ele{};
	std::ptrdiff_t reach{};
	Shape shape;
};

double highest_within(const std::vector<std::vector<double>> &grid, double gx, double gy,
		double radius_m, double meters_per_cell)
{
	const auto height = grid.size(), width = grid.front().size();
	const double r = radius_m / meters_per_cell;
	const auto cx = static_cast<std::size_t>(std::llround(gx));
	const auto cy = static_cast<std::size_t>(std::llround(gy));
	double top = -std::numeric_limits<double>::infinity();
	cells_within(cx, cy, radius_cells(radius_m, meters_per_cell), width, height,
			[&](std::size_t x, std::size_t y) {
				if (std::pow(static_cast<double>(x) - gx, 2.0) +
										std::pow(static_cast<double>(y) - gy, 2.0) <=
								r * r &&
						std::isfinite(grid[y][x]))
					top = std::max(top, grid[y][x]);
			});
	return top;
}

std::optional<double> ring_mean(const std::vector<std::vector<double>> &grid, double gx,
		double gy, double radius_m, double meters_per_cell)
{
	const auto height = grid.size(), width = grid.front().size();
	const double radius = radius_m / meters_per_cell;
	double sum = 0.0;
	std::size_t count = 0;
	for (std::size_t side = 0; side < SIDES; ++side) {
		const double angle = side * 2.0 * std::numbers::pi / SIDES;
		const auto x =
				static_cast<std::ptrdiff_t>(std::llround(gx + std::cos(angle) * radius));
		const auto y =
				static_cast<std::ptrdiff_t>(std::llround(gy + std::sin(angle) * radius));
		if (x < 0 || y < 0 || static_cast<std::size_t>(x) >= width ||
				static_cast<std::size_t>(y) >= height)
			continue;
		const double value =
				grid[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)];
		if (std::isfinite(value)) {
			sum += value;
			++count;
		}
	}
	return count ? std::optional<double>(sum / count) : std::nullopt;
}

std::size_t rising_sides(const std::vector<std::vector<double>> &grid, double gx,
		double gy, double target, double meters_per_cell)
{
	const auto height = grid.size(), width = grid.front().size();
	const double radius = RELIEF_M / meters_per_cell;
	const double step = std::max(std::ceil(radius / RELIEF_SAMPLES), 1.0);
	std::size_t count = 0;
	for (std::size_t side = 0; side < SIDES; ++side) {
		const double angle = side * 2.0 * std::numbers::pi / SIDES;
		bool rises = false;
		const auto samples = static_cast<std::size_t>(radius / step);
		for (std::size_t i = 1; i <= samples; ++i) {
			const auto x = static_cast<std::ptrdiff_t>(
					std::llround(gx + std::cos(angle) * step * i));
			const auto y = static_cast<std::ptrdiff_t>(
					std::llround(gy + std::sin(angle) * step * i));
			if (x >= 0 && y >= 0 && static_cast<std::size_t>(x) < width &&
					static_cast<std::size_t>(y) < height &&
					grid[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)] >
							target) {
				rises = true;
				break;
			}
		}
		count += rises;
	}
	return count;
}

double stretch_fade(double distance)
{
	const auto fade = [](double d) {
		const double ratio = d / STRETCH_RADIUS_M;
		return std::exp(-ratio * ratio * ratio * ratio);
	};
	const double edge = fade(STRETCH_REACH_M);
	return std::max((fade(distance) - edge) / (1.0 - edge), 0.0);
}

std::optional<Lift> plan_lift(const std::vector<std::vector<double>> &grid, double ele,
		double gx, double gy, double meters_per_cell, std::optional<std::size_t> seam_pad)
{
	const auto height = grid.size(), width = grid.front().size();
	const auto cx = static_cast<std::size_t>(std::llround(gx));
	const auto cy = static_cast<std::size_t>(std::llround(gy));
	const double at_node = grid[cy][cx];
	std::size_t top_x = cx, top_y = cy;
	double dem_top = -std::numeric_limits<double>::infinity();
	cells_within(cx, cy, radius_cells(SEARCH_M, meters_per_cell), width, height,
			[&](std::size_t x, std::size_t y) {
				if (std::isfinite(grid[y][x]) && grid[y][x] > dem_top) {
					dem_top = grid[y][x];
					top_x = x;
					top_y = y;
				}
			});
	const double shortfall = ele - dem_top;
	if (!std::isfinite(at_node) ||
			!(shortfall >= MIN_DEFICIT_M && shortfall <= MAX_DEFICIT_M) ||
			shortfall > local_relief(grid, cx, cy, meters_per_cell))
		return std::nullopt;
	const double level = std::max(shortfall * LEVEL_SHARE, MIN_DEFICIT_M);
	const double near_top = highest_within(grid, gx, gy, NEARBY_M, meters_per_cell);
	if (near_top >= ele - level ||
			rising_sides(grid, gx, gy, dem_top + level, meters_per_cell) >= HOLLOW_SIDES)
		return std::nullopt;
	if (near_top > dem_top + SUMMIT_NOISE_M) {
		double dome_top = -std::numeric_limits<double>::infinity();
		cells_within(cx, cy, radius_cells(DOME_SEARCH_M, meters_per_cell), width, height,
				[&](std::size_t x, std::size_t y) {
					if (std::isfinite(grid[y][x]) && grid[y][x] > dome_top) {
						dome_top = grid[y][x];
						top_x = x;
						top_y = y;
					}
				});
		if (std::isfinite(dome_top))
			dem_top = dome_top;
	}
	const double dome_shortfall = ele - dem_top;
	const bool keeps_summit = near_top <= dem_top + SUMMIT_NOISE_M;
	const double tx = static_cast<double>(top_x), ty = static_cast<double>(top_y);
	const auto drop_at = [&](double radius) -> std::optional<double> {
		const auto mean = ring_mean(grid, tx, ty, radius, meters_per_cell);
		return mean ? std::optional<double>(dem_top - *mean) : std::nullopt;
	};
	if (keeps_summit) {
		const auto plateau_drop = drop_at(PLATEAU_M);
		if (plateau_drop && *plateau_drop < level)
			return std::nullopt;
	}
	const auto stretch_drop = drop_at(STRETCH_RADIUS_M);
	const bool stretch = keeps_summit && stretch_drop &&
						 *stretch_drop >= STRETCH_DEPTH * dome_shortfall;
	const double reach_m = stretch ? STRETCH_REACH_M : 3.0 * SIGMA_M;
	const auto reach = radius_cells(reach_m, meters_per_cell);
	if (seam_pad) {
		const auto r = static_cast<std::size_t>(reach);
		if (*seam_pad > std::numeric_limits<std::size_t>::max() - r)
			return std::nullopt;
		const auto margin = *seam_pad + r;
		if (cx < margin || cy < margin || margin >= width - cx || margin >= height - cy)
			return std::nullopt;
	}
	Shape shape;
	if (stretch) {
		const double d_top = std::hypot(tx - gx, ty - gy) * meters_per_cell;
		shape = {Shape::Kind::Stretch, dem_top - STRETCH_DEPTH * dome_shortfall,
				1.0 / (STRETCH_DEPTH * stretch_fade(d_top)), 0.0};
	} else {
		shape = {Shape::Kind::Bump, 0.0, 0.0, ele - at_node};
	}
	return Lift{gx, gy, ele, reach, shape};
}

void apply_lift(const Lift &lift, std::vector<std::vector<double>> &grid,
		double meters_per_cell, const std::vector<Summit> &lower)
{
	const auto height = grid.size(), width = grid.front().size();
	const auto cx = static_cast<std::size_t>(std::llround(lift.gx));
	const auto cy = static_cast<std::size_t>(std::llround(lift.gy));
	cells_within(cx, cy, lift.reach, width, height, [&](std::size_t x, std::size_t y) {
		double &value = grid[y][x];
		if (!std::isfinite(value))
			return;
		const auto distance = [&](double gx, double gy) {
			return std::hypot(static_cast<double>(x) - gx, static_cast<double>(y) - gy) *
				   meters_per_cell;
		};
		const double d = distance(lift.gx, lift.gy);
		double cap = lift.ele;
		for (const auto &summit : lower)
			cap = std::min(cap, summit.ele + CONE_SLOPE * distance(summit.gx, summit.gy));
		if (lift.shape.kind == Shape::Kind::Stretch) {
			if (value > lift.shape.base)
				value = std::max(
						value, std::min(value + (value - lift.shape.base) *
														lift.shape.gain * stretch_fade(d),
									   cap));
		} else {
			const double room =
					std::max(std::min(lift.ele - CONE_SLOPE * d, cap) - value, 0.0);
			value += std::min(
					lift.shape.lift * std::exp(-std::pow(d / SIGMA_M, 2.0)), room);
		}
	});
}

bool grid_cell(const geographic::LLBBox &bbox, std::size_t width, std::size_t height,
		double lat, double lon, std::size_t &x, std::size_t &y)
{
	const double min_lat = bbox.min().lat(), max_lat = bbox.max().lat();
	const double min_lon = bbox.min().lng(), max_lon = bbox.max().lng();
	if (!(lat >= min_lat && lat <= max_lat && lon >= min_lon && lon <= max_lon) ||
			max_lat <= min_lat || max_lon <= min_lon)
		return false;
	const double gx = (lon - min_lon) / (max_lon - min_lon) * (width - 1);
	const double gy = (max_lat - lat) / (max_lat - min_lat) * (height - 1);
	x = static_cast<std::size_t>(std::llround(gx));
	y = static_cast<std::size_t>(std::llround(gy));
	return x < width && y < height;
}
} // namespace

bool PeakSender::send(TerrainHints hints)
{
	if (!promise_)
		return false;
	try {
		promise_->set_value(std::move(hints));
		promise_.reset();
		return true;
	} catch (const std::future_error &) {
		promise_.reset();
		return false;
	}
}

bool PeakSender::send_overture(TerrainHints hints)
{
	if (!overture_promise_)
		return false;
	try {
		overture_promise_->set_value(std::move(hints));
		overture_promise_.reset();
		return true;
	} catch (const std::future_error &) {
		overture_promise_.reset();
		return false;
	}
}

PeakChannel PeakSource::channel()
{
	auto promise = std::make_shared<std::promise<TerrainHints>>();
	auto future = promise->get_future().share();
	auto overture_promise = std::make_shared<std::promise<TerrainHints>>();
	auto overture_future = overture_promise->get_future().share();
	return {PeakSender(std::move(promise), std::move(overture_promise)),
			PeakSource(std::move(future), std::move(overture_future))};
}

PeakSource PeakSource::none()
{
	std::promise<TerrainHints> promise;
	auto future = promise.get_future().share();
	promise.set_value({});
	std::promise<TerrainHints> overture_promise;
	auto overture_future = overture_promise.get_future().share();
	overture_promise.set_value({});
	return PeakSource(std::move(future), std::move(overture_future));
}

TerrainHints PeakSource::take() const
{
	std::shared_future<TerrainHints> future;
	{
		std::lock_guard lock(take_mutex_);
		if (taken_ || !future_.valid())
			return {};
		taken_ = true;
		future = future_;
	}
	try {
		auto hints = future.wait_for(std::chrono::seconds(0)) == std::future_status::ready
							 ? future.get()
							 : progress::while_waiting("Waiting for map data...",
									   [&] { return future.get(); });
		{
			std::lock_guard lock(take_mutex_);
			overture_allowed_ = hints.overture_follows;
			overture_taken_ = !overture_allowed_;
		}
		return hints;
	} catch (const std::future_error &) {
		return {};
	}
}

std::vector<TallModel> PeakSource::take_overture() const
{
	std::shared_future<TerrainHints> future;
	{
		std::lock_guard lock(take_mutex_);
		if (!overture_allowed_ || overture_taken_ || !overture_future_.valid())
			return {};
		overture_taken_ = true;
		future = overture_future_;
	}
	try {
		if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
			return future.get().tall_structures;
		return progress::while_waiting("Waiting for Overture buildings...",
				[&] { return future.get().tall_structures; });
	} catch (const std::future_error &) {
		return {};
	}
}

std::optional<PeakHint> peak_from_tags(
		double latitude, double longitude, const tags_t &tags)
{
	const auto natural = tags.find("natural");
	const auto elevation = tags.find("ele");
	if (natural == tags.end() || elevation == tags.end() ||
			(natural->second != "peak" && natural->second != "volcano") ||
			!std::isfinite(latitude) || !std::isfinite(longitude))
		return std::nullopt;
	const auto ele = parse_elevation(elevation->second);
	if (!ele)
		return std::nullopt;
	return PeakHint{latitude, longitude, *ele};
}

std::vector<PeakHint> peaks_in(const osm_parser::RawOsmDocument &data)
{
	std::vector<PeakHint> result;
	for (const auto &node : data.nodes)
		if (node.lat && node.lon)
			if (auto peak = peak_from_tags(*node.lat, *node.lon, node.tags))
				result.push_back(*peak);
	return result;
}

std::vector<PeakHint> peaks_in(const std::vector<ProcessedElement> &elements)
{
	std::vector<PeakHint> result;
	for (const auto &element : elements)
		if (element.is_node()) {
			const auto &node = element.as_node();
			if (auto peak = peak_from_tags(node.latitude, node.longitude, node.tags))
				result.push_back(*peak);
		}
	return result;
}

std::vector<TallModel> tall_models_in(const osm_parser::RawOsmDocument &data)
{
	std::vector<TallModel> result;
	for (const auto &node : data.nodes)
		if (node.lat && node.lon)
			if (auto h = model_height(node.tags);
					h && std::isfinite(*node.lat) && std::isfinite(*node.lon))
				result.push_back({*node.lat, *node.lon, *h});

	std::unordered_map<std::uint64_t, std::pair<double, double>> coordinates;
	std::unordered_set<std::uint64_t> wanted;
	for (const auto &way : data.ways)
		if (model_height(way.tags))
			wanted.insert(way.node_refs.begin(), way.node_refs.end());
	if (!wanted.empty())
		for (const auto &node : data.nodes)
			if (wanted.contains(node.id) && node.lat && node.lon)
				coordinates.emplace(node.id, std::pair{*node.lat, *node.lon});

	for (const auto &way : data.ways) {
		const auto h = model_height(way.tags);
		if (!h)
			continue;
		double lat = 0.0, lon = 0.0;
		std::size_t count = 0;
		for (const auto id : way.node_refs)
			if (const auto it = coordinates.find(id); it != coordinates.end()) {
				lat += it->second.first;
				lon += it->second.second;
				++count;
			}
		if (count)
			result.push_back({lat / count, lon / count, *h});
	}
	return result;
}

std::vector<TallModel> landmark_tall_models_in(
		const osm_parser::RawOsmDocument &data, const geographic::LLBBox &bbox)
{
	const auto identity_matches = [](const landmarks::Landmark &landmark,
										  std::string_view type, std::uint64_t id,
										  const tags_t &tags) {
		const bool osm_match = std::any_of(landmark.osm_ids.begin(),
				landmark.osm_ids.end(),
				[&](const auto &key) { return key.first == type && key.second == id; });
		if (osm_match)
			return true;
		const auto qid = tags.find("wikidata");
		if (qid == tags.end())
			return false;
		const auto [begin, end] = unicode_text::trim_whitespace(qid->second);
		return qid->second.substr(begin, end - begin) == landmark.qid;
	};
	std::vector<TallModel> result;
	for (const auto &landmark : landmarks::catalogue()) {
		bool in_data = false;
		for (const auto &node : data.nodes)
			if (identity_matches(landmark, "node", node.id, node.tags)) {
				in_data = true;
				break;
			}
		if (!in_data)
			for (const auto &way : data.ways)
				if (identity_matches(landmark, "way", way.id, way.tags)) {
					in_data = true;
					break;
				}
		if (!in_data)
			for (const auto &relation : data.relations)
				if (identity_matches(landmark, "relation", relation.id, relation.tags)) {
					in_data = true;
					break;
				}
		const double lat_m = 111'320.0;
		const double lon_m =
				lat_m *
				std::max(0.01, std::cos(landmark.latitude * std::numbers::pi / 180.0));
		const double reach = landmark.reach_m;
		const bool in_reach = landmark.latitude + reach / lat_m >= bbox.min().lat() &&
							  landmark.latitude - reach / lat_m <= bbox.max().lat() &&
							  landmark.longitude + reach / lon_m >= bbox.min().lng() &&
							  landmark.longitude - reach / lon_m <= bbox.max().lng();
		if (in_data || in_reach)
			result.push_back({landmark.latitude, landmark.longitude,
					static_cast<double>(landmark.height)});
	}
	return result;
}

bool is_building_feature(const tags_t &tags)
{
	return tags.contains("building") || tags.contains("building:part") ||
		   tags.get("man_made") == "tower";
}

std::pair<double, double> extent_m(const std::vector<std::pair<double, double>> &points)
{
	if (points.empty())
		return {};
	double min_lat = std::numeric_limits<double>::max();
	double max_lat = std::numeric_limits<double>::lowest();
	double min_lon = std::numeric_limits<double>::max();
	double max_lon = std::numeric_limits<double>::lowest();
	for (const auto &[lat, lon] : points) {
		if (!std::isfinite(lat) || !std::isfinite(lon))
			continue;
		min_lat = std::min(min_lat, lat);
		max_lat = std::max(max_lat, lat);
		min_lon = std::min(min_lon, lon);
		max_lon = std::max(max_lon, lon);
	}
	if (min_lat > max_lat)
		return {};
	constexpr double meters_per_degree = 111320.0;
	const double middle_lat = (min_lat + max_lat) * 0.5 * std::numbers::pi / 180.0;
	return {(max_lon - min_lon) * meters_per_degree * std::cos(middle_lat),
			(max_lat - min_lat) * meters_per_degree};
}

std::optional<double> mast_height_m(const tags_t &tags, bool on_roof)
{
	const auto kind = tags.find("man_made");
	if (kind == tags.end())
		return std::nullopt;
	const bool lighting = tags.get("tower:type") == "lighting";
	double fallback = 0.0;
	if (kind->second == "chimney")
		fallback = on_roof ? 2.0 : 40.0;
	else if (kind->second == "antenna")
		fallback = on_roof ? 3.0 : lighting ? 20.0 : 10.0;
	else if (kind->second == "mast")
		fallback = on_roof ? 6.0 : lighting ? 20.0 : 30.0;
	else
		return std::nullopt;
	if (const auto height = tags.find("height"); height != tags.end()) {
		auto parsed = mapillary::geodesy::parse_length_m(height->second);
		if (parsed && *parsed > 0.0 && *parsed < 1000.0)
			return parsed;
	}
	return fallback;
}

double top_over_roof_m(double metres, double roof_m, double scale)
{
	const double blocks = std::round(metres * scale);
	const double roof = std::round(roof_m * scale);
	return roof > 0.0 && blocks > roof + 1.0 ? metres : roof_m + metres;
}

using GeoRing = std::vector<std::pair<double, double>>;

bool point_in_ring(double lat, double lon, const GeoRing &ring)
{
	if (ring.size() < 3)
		return false;
	bool inside = false;
	std::size_t j = ring.size() - 1;
	for (std::size_t i = 0; i < ring.size(); ++i) {
		const auto &[yi, xi] = ring[i];
		const auto &[yj, xj] = ring[j];
		if ((yi > lat) != (yj > lat) && lon < (xj - xi) * (lat - yi) / (yj - yi) + xi)
			inside = !inside;
		j = i;
	}
	return inside;
}

std::vector<GeoRing> stitch_geo_rings(std::vector<std::vector<std::uint64_t>> segments,
		const std::unordered_map<std::uint64_t, std::pair<double, double>> &coordinates)
{
	std::vector<std::vector<std::uint64_t>> open;
	open.reserve(segments.size());
	for (auto &segment : segments)
		if (segment.size() >= 2)
			open.push_back(std::move(segment));
	std::vector<GeoRing> rings;
	while (!open.empty()) {
		auto ring = std::move(open.back());
		open.pop_back();
		while (ring.size() >= 2) {
			const auto head = ring.front(), tail = ring.back();
			if (ring.size() >= 4 && head == tail)
				break;
			const auto found =
					std::find_if(open.begin(), open.end(), [&](const auto &segment) {
						return !segment.empty() &&
							   (segment.front() == tail || segment.back() == tail ||
									   segment.front() == head || segment.back() == head);
					});
			if (found == open.end())
				break;
			auto segment = std::move(*found);
			if (found != std::prev(open.end()))
				*found = std::move(open.back());
			open.pop_back();
			const auto first = segment.front(), last = segment.back();
			if (first == tail) {
				ring.insert(ring.end(), std::next(segment.begin()), segment.end());
			} else if (last == tail) {
				std::reverse(segment.begin(), segment.end());
				ring.insert(ring.end(), std::next(segment.begin()), segment.end());
			} else if (last == head) {
				segment.pop_back();
				segment.insert(segment.end(), ring.begin(), ring.end());
				ring = std::move(segment);
			} else {
				std::reverse(segment.begin(), segment.end());
				segment.pop_back();
				segment.insert(segment.end(), ring.begin(), ring.end());
				ring = std::move(segment);
			}
		}
		if (ring.size() < 4 || ring.front() != ring.back())
			continue;
		GeoRing points;
		points.reserve(ring.size());
		for (const auto id : ring)
			if (const auto it = coordinates.find(id); it != coordinates.end())
				points.push_back(it->second);
		if (points.size() >= 4)
			rings.push_back(std::move(points));
	}
	return rings;
}

std::vector<TallModel> tall_structures_in(const osm_parser::RawOsmDocument &data,
		double scale, const overture::OvertureHints *overture_hints)
{
	std::vector<TallModel> found;
	if (!std::isfinite(scale) || scale <= 0.0)
		return found;
	std::unordered_map<std::uint64_t, std::pair<double, double>> coordinates;
	coordinates.reserve(data.nodes.size());
	for (const auto &node : data.nodes)
		if (node.lat && node.lon && std::isfinite(*node.lat) && std::isfinite(*node.lon))
			coordinates.emplace(node.id, std::pair{*node.lat, *node.lon});

	struct Roof
	{
		std::vector<GeoRing> outer, inner;
		double top = 0.0;
	};
	std::vector<Roof> roofs;
	std::unordered_map<std::uint64_t, double> node_tops;
	std::optional<PreparedBuildingData> prepared_suppression;
	auto get_prepared_suppression = [&]() -> const PreparedBuildingData & {
		if (!prepared_suppression) {
			std::unordered_set<std::uint64_t> relevant_ways;
			std::unordered_set<std::uint64_t> relevant_relations;
			for (const auto &way : data.ways)
				if (is_building_feature(way.tags) ||
						attractions::replaces_parts(way.tags))
					relevant_ways.insert(way.id);
			for (const auto &relation : data.relations) {
				if (!is_building_feature(relation.tags))
					continue;
				relevant_relations.insert(relation.id);
				for (const auto &member : relation.members)
					if (member.type == "way")
						relevant_ways.insert(member.ref);
			}

			std::vector<ProcessedElement> suppression_input;
			suppression_input.reserve(relevant_ways.size() + relevant_relations.size());
			for (const auto &way : data.ways) {
				if (!relevant_ways.contains(way.id))
					continue;
				ProcessedWay converted;
				converted.id = way.id;
				converted.tags = way.tags;
				converted.source_node_refs = way.node_refs;
				converted.nodes.reserve(way.node_refs.size());
				for (const auto node_id : way.node_refs) {
					const auto point = coordinates.find(node_id);
					if (point == coordinates.end())
						continue;
					ProcessedNode node{};
					node.id = node_id;
					node.x = 0;
					node.z = 0;
					node.latitude = point->second.first;
					node.longitude = point->second.second;
					converted.nodes.push_back(std::move(node));
				}
				suppression_input.emplace_back(converted);
			}
			for (const auto &relation : data.relations) {
				if (!relevant_relations.contains(relation.id))
					continue;
				ProcessedRelation converted;
				converted.id = relation.id;
				converted.tags = relation.tags;
				converted.source_members.reserve(relation.members.size());
				for (const auto &member : relation.members)
					converted.source_members.push_back(
							{member.type, member.ref, member.role});
				suppression_input.emplace_back(converted);
			}
			prepared_suppression.emplace(prepare_building_data(suppression_input));
		}
		return *prepared_suppression;
	};
	auto add_outline = [&](std::uint64_t id, bool relation, const tags_t &tags,
							   const std::vector<std::vector<std::uint64_t>> &outer_ids,
							   const std::vector<std::vector<std::uint64_t>> &inner_ids) {
		// Rust classifies a mast/antenna/chimney before checking building tags;
		// a multiply-tagged mast is placed as a mast, never counted as a roof.
		if (!is_building_feature(tags) || outer_ids.empty() || mast_height_m(tags, false))
			return;
		std::vector<std::pair<double, double>> points;
		std::vector<std::uint64_t> outline_nodes;
		for (const auto &ring : outer_ids) {
			outline_nodes.insert(outline_nodes.end(), ring.begin(), ring.end());
			for (const auto node_id : ring)
				if (const auto it = coordinates.find(node_id); it != coordinates.end())
					points.push_back(it->second);
		}
		for (const auto &ring : inner_ids)
			outline_nodes.insert(outline_nodes.end(), ring.begin(), ring.end());
		if (points.empty())
			return;
		const auto [width_m, length_m] = extent_m(points);
		buildings::HeightHint hint;
		if (overture_hints) {
			const auto value = overture_hints->hint_for({id, relation}, tags);
			if (!value)
				return;
			hint.height_m = value->height_m;
			if (value->num_floors)
				hint.levels = *value->num_floors;
		}
		const auto walls = buildings::mapped_walls_m(tags, hint);
		if (!walls)
			return;
		bool reaches = may_pass_vanilla(walls->first, scale);
		if (!reaches && walls->second) {
			constexpr double widest_footprint_m = 1000.0;
			const auto maximum_top = buildings::mapped_top_m(
					tags, hint, widest_footprint_m, widest_footprint_m);
			reaches = maximum_top && may_pass_vanilla(*maximum_top, scale);
		}
		if (!reaches)
			return;
		const auto &suppression = get_prepared_suppression();
		const auto kind = relation ? std::string("relation") : std::string("way");
		if (suppression.outline_suppression.contains({kind, id}) ||
				(relation && suppression.suppressed_relations.contains(id)))
			return;
		const auto top = buildings::mapped_top_m(tags, hint, width_m, length_m);
		if (!top)
			return;
		if (may_pass_vanilla(*top, scale))
			for (const auto node_id : outline_nodes)
				if (const auto it = coordinates.find(node_id); it != coordinates.end()) {
					auto &at = node_tops[node_id];
					at = std::max(at, *top);
				}
		Roof roof;
		roof.outer = stitch_geo_rings(outer_ids, coordinates);
		roof.inner = stitch_geo_rings(inner_ids, coordinates);
		roof.top = *top;
		if (!roof.outer.empty())
			roofs.push_back(std::move(roof));
	};
	for (const auto &way : data.ways)
		add_outline(way.id, false, way.tags, {way.node_refs}, {});
	std::unordered_map<std::uint64_t, const osm_parser::RawWay *> ways;
	ways.reserve(data.ways.size());
	for (const auto &way : data.ways)
		ways.emplace(way.id, &way);
	for (const auto &relation : data.relations) {
		if (!is_building_feature(relation.tags))
			continue;
		std::vector<std::vector<std::uint64_t>> outer, inner;
		for (const auto &member : relation.members) {
			if (member.type != "way")
				continue;
			const auto way = ways.find(member.ref);
			if (way == ways.end())
				continue;
			(member.role == "inner" ? inner : outer).push_back(way->second->node_refs);
		}
		add_outline(relation.id, true, relation.tags, outer, inner);
	}
	for (const auto &[node_id, top] : node_tops)
		if (const auto it = coordinates.find(node_id); it != coordinates.end())
			found.push_back({it->second.first, it->second.second, top});

	// Rust stores only outlines that can exceed the vanilla ceiling, then places
	// an OSM mast on their mapped roof when its point is inside the outer ring and
	// outside every hole. This raw-OSM equivalent runs before projected geometry.
	auto add_mast_at = [&](double lat, double lon, const tags_t &tags) {
		std::optional<double> roof_top;
		for (const auto &roof : roofs) {
			const bool inside_outer = std::any_of(roof.outer.begin(), roof.outer.end(),
					[&](const GeoRing &ring) { return point_in_ring(lat, lon, ring); });
			if (!inside_outer)
				continue;
			const bool inside_hole = std::any_of(roof.inner.begin(), roof.inner.end(),
					[&](const GeoRing &ring) { return point_in_ring(lat, lon, ring); });
			if (!inside_hole)
				roof_top = std::max(roof_top.value_or(roof.top), roof.top);
		}
		const auto metres = mast_height_m(tags, roof_top.has_value());
		if (!metres)
			return;
		const double top =
				roof_top ? top_over_roof_m(*metres, *roof_top, scale) : *metres;
		if (may_pass_vanilla(top, scale))
			found.push_back({lat, lon, top});
	};
	for (const auto &node : data.nodes)
		if (node.lat && node.lon) {
			if (!overture_hints)
				add_mast_at(*node.lat, *node.lon, node.tags);
		}
	if (!overture_hints)
		for (const auto &way : data.ways)
			if (mast_height_m(way.tags, false))
				for (const auto node_id : way.node_refs)
					if (const auto it = coordinates.find(node_id);
							it != coordinates.end())
						add_mast_at(it->second.first, it->second.second, way.tags);
	std::sort(found.begin(), found.end(), [](const TallModel &a, const TallModel &b) {
		return std::tie(a.lat, a.lon, a.height_m) < std::tie(b.lat, b.lon, b.height_m);
	});
	found.erase(std::unique(found.begin(), found.end(),
						[](const TallModel &a, const TallModel &b) {
							return a.lat == b.lat && a.lon == b.lon &&
								   a.height_m == b.height_m;
						}),
			found.end());
	return found;
}

bool may_pass_vanilla(double height_m, double scale)
{
	return std::isfinite(height_m) && std::isfinite(scale) &&
		   height_m * scale > TERRAIN_HEIGHT_BUFFER - 2;
}

std::vector<std::pair<std::size_t, std::size_t>> bore_water_cells(
		const land_cover::LandCoverData &cover, const geographic::LLBBox &bbox,
		const std::vector<std::pair<double, double>> &bores)
{
	const auto width = cover.width;
	const auto height = cover.height;
	if (width < 2 || height < 2)
		return {};
	const double min_lat = bbox.min().lat(), max_lat = bbox.max().lat();
	const double min_lon = bbox.min().lng(), max_lon = bbox.max().lng();
	if (!(max_lat > min_lat) || !(max_lon > min_lon))
		return {};
	std::vector<std::pair<std::size_t, std::size_t>> cells;
	cells.reserve(bores.size());
	for (const auto &[lat, lon] : bores) {
		if (!std::isfinite(lat) || !std::isfinite(lon) || lat < min_lat ||
				lat > max_lat || lon < min_lon || lon > max_lon)
			continue;
		const auto x = static_cast<std::size_t>(
				std::llround((lon - min_lon) / (max_lon - min_lon) * (width - 1)));
		const auto z = static_cast<std::size_t>(
				std::llround((max_lat - lat) / (max_lat - min_lat) * (height - 1)));
		const auto cell_x = std::min(x, width - 1);
		const auto cell_z = std::min(z, height - 1);
		if (cell_z < cover.grid.size() && cell_x < cover.grid[cell_z].size() &&
				cover.grid[cell_z][cell_x] == land_cover::LC_WATER)
			cells.emplace_back(cell_x, cell_z);
	}
	std::sort(cells.begin(), cells.end());
	cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
	return cells;
}

TerrainHints terrain_hints_in(const osm_parser::RawOsmDocument &data)
{
	return terrain_hints_in(data, 1.0, true, false);
}

TerrainHints terrain_hints_in(const osm_parser::RawOsmDocument &data, double scale,
		bool include_models, bool include_structures, bool overture_follows)
{
	return terrain_hints_in(data, scale, include_models, include_structures,
			overture_follows, false, false);
}

TerrainHints terrain_hints_in(const osm_parser::RawOsmDocument &data, double scale,
		bool include_models, bool include_structures, bool overture_follows,
		bool include_graded, bool include_roads)
{
	TerrainHints hints;
	hints.peaks = peaks_in(data);
	auto ways = terrain_ways_in(data, include_graded, include_roads);
	hints.bores = std::move(ways.bores);
	hints.graded = std::move(ways.graded);
	if (include_models)
		hints.tall_models = tall_models_in(data);
	if (include_structures) {
		hints.tall_structures = tall_structures_in(data, scale);
		hints.stations = station_hints_in(data);
	}
	hints.overture_follows = overture_follows;
	return hints;
}

std::vector<std::vector<std::pair<double, double>>> graded_ways_in(
		const osm_parser::RawOsmDocument &data, bool include_roads)
{
	const auto value = [](const tags_t &tags, std::string_view key) -> std::string_view {
		const auto found = tags.find(std::string(key));
		return found == tags.end() ? std::string_view{} : std::string_view(found->second);
	};
	const auto one_of = [](std::string_view text,
								std::initializer_list<std::string_view> values) {
		return std::find(values.begin(), values.end(), text) != values.end();
	};
	const auto graded = [&](const tags_t &tags) {
		for (const auto key : {"tunnel", "bridge"}) {
			const auto found = tags.find(key);
			if (found != tags.end() && found->second != "no")
				return false;
		}
		const auto waterway = value(tags, "waterway");
		if (one_of(waterway, {"river", "stream", "canal", "ditch", "drain", "brook",
									 "tidal_channel"}))
			return true;
		if (!include_roads)
			return false;
		const auto building = tags.find("building");
		if (building != tags.end() && building->second != "no")
			return true;
		const auto highway = tags.find("highway");
		if (highway != tags.end() &&
				!one_of(highway->second,
						{"path", "footway", "bridleway", "steps", "via_ferrata",
								"corridor", "elevator", "platform", "proposed",
								"abandoned", "razed", "disused"}))
			return true;
		return one_of(value(tags, "railway"),
				{"rail", "light_rail", "narrow_gauge", "tram", "funicular", "monorail",
						"preserved", "miniature"});
	};

	std::unordered_set<std::uint64_t> wanted;
	for (const auto &way : data.ways)
		if (graded(way.tags))
			wanted.insert(way.node_refs.begin(), way.node_refs.end());
	if (wanted.empty())
		return {};
	std::unordered_map<std::uint64_t, std::pair<double, double>> coordinates;
	coordinates.reserve(wanted.size());
	for (const auto &node : data.nodes)
		if (wanted.contains(node.id) && node.lat && node.lon &&
				std::isfinite(*node.lat) && std::isfinite(*node.lon))
			coordinates.emplace(node.id, std::pair{*node.lat, *node.lon});

	std::vector<std::vector<std::pair<double, double>>> result;
	for (const auto &way : data.ways) {
		if (!graded(way.tags))
			continue;
		std::vector<std::pair<double, double>> line;
		for (const auto id : way.node_refs) {
			const auto found = coordinates.find(id);
			if (found == coordinates.end()) {
				if (!line.empty()) {
					result.push_back(std::move(line));
					line.clear();
				}
			} else {
				line.push_back(found->second);
			}
		}
		if (!line.empty())
			result.push_back(std::move(line));
	}
	return result;
}

TerrainWays terrain_ways_in(
		const osm_parser::RawOsmDocument &data, bool include_graded, bool include_roads)
{
	using Point = std::pair<double, double>;
	const auto value = [](const tags_t &tags, std::string_view key) -> std::string_view {
		const auto found = tags.find(std::string(key));
		return found == tags.end() ? std::string_view{} : std::string_view(found->second);
	};
	const auto is_bore = [&](const tags_t &tags) {
		const auto tunnel = value(tags, "tunnel");
		const auto railway = value(tags, "railway");
		return (tunnel == "yes" &&
					   (tags.contains("highway") || tags.contains("railway"))) ||
			   (railway == "subway" && tunnel != "no");
	};
	const auto one_of = [](std::string_view text,
								std::initializer_list<std::string_view> values) {
		return std::find(values.begin(), values.end(), text) != values.end();
	};
	const auto is_graded = [&](const tags_t &tags) {
		for (const auto key : {"tunnel", "bridge"}) {
			const auto found = tags.find(key);
			if (found != tags.end() && found->second != "no")
				return false;
		}
		if (one_of(value(tags, "waterway"), {"river", "stream", "canal", "ditch", "drain",
													"brook", "tidal_channel"}))
			return true;
		if (!include_roads)
			return false;
		const auto building = tags.find("building");
		if (building != tags.end() && building->second != "no")
			return true;
		const auto highway = tags.find("highway");
		if (highway != tags.end() &&
				!one_of(highway->second,
						{"path", "footway", "bridleway", "steps", "via_ferrata",
								"corridor", "elevator", "platform", "proposed",
								"abandoned", "razed", "disused"}))
			return true;
		return one_of(value(tags, "railway"),
				{"rail", "light_rail", "narrow_gauge", "tram", "funicular", "monorail",
						"preserved", "miniature"});
	};
	struct SelectedWay
	{
		const osm_parser::RawWay *way{};
		bool bore{};
		bool graded{};
	};
	std::vector<SelectedWay> selected;
	std::unordered_set<std::uint64_t> wanted;
	for (const auto &way : data.ways) {
		const bool bore = is_bore(way.tags);
		const bool graded = include_graded && is_graded(way.tags);
		if (!bore && !graded)
			continue;
		selected.push_back({&way, bore, graded});
		wanted.insert(way.node_refs.begin(), way.node_refs.end());
	}
	TerrainWays result;
	if (selected.empty())
		return result;
	std::unordered_map<std::uint64_t, Point> coordinates;
	coordinates.reserve(wanted.size());
	for (const auto &node : data.nodes)
		if (wanted.contains(node.id) && node.lat && node.lon &&
				std::isfinite(*node.lat) && std::isfinite(*node.lon))
			coordinates.emplace(node.id, Point{*node.lat, *node.lon});
	for (const auto &item : selected) {
		std::vector<Point> line;
		const auto flush_bore = [&] {
			if (line.empty())
				return;
			result.bores.push_back(line.front());
			for (std::size_t i = 1; i < line.size(); ++i) {
				const auto [lat_a, lon_a] = line[i - 1];
				const auto [lat_b, lon_b] = line[i];
				const double mean_lat = (lat_a + lat_b) * 0.5 * std::numbers::pi / 180.0;
				const double dx = (lon_b - lon_a) * 111'320.0 * std::cos(mean_lat);
				const double dz = (lat_b - lat_a) * 111'320.0;
				const auto steps = std::max<std::size_t>(
						1, static_cast<std::size_t>(std::ceil(std::hypot(dx, dz) / 2.0)));
				for (std::size_t k = 1; k <= steps; ++k) {
					const double t = static_cast<double>(k) / steps;
					result.bores.emplace_back(
							lat_a + (lat_b - lat_a) * t, lon_a + (lon_b - lon_a) * t);
				}
			}
			line.clear();
		};
		const auto flush_graded = [&] {
			if (line.empty())
				return;
			result.graded.push_back(std::move(line));
			line.clear();
		};
		for (const auto id : item.way->node_refs) {
			const auto found = coordinates.find(id);
			if (found != coordinates.end()) {
				line.push_back(found->second);
				continue;
			}
			if (item.bore)
				flush_bore();
			else if (item.graded)
				flush_graded();
		}
		if (item.bore)
			flush_bore();
		else if (item.graded)
			flush_graded();
	}
	return result;
}

namespace
{
constexpr double BORE_STEP_M = 2.0;
constexpr double BORE_PLATFORM_M = 12.0;
constexpr double INTERCHANGE_M = 250.0;
constexpr double METRES_PER_DEGREE = 111'320.0;

std::string_view tag_value(const tags_t &tags, std::string_view key)
{
	const auto found = tags.find(std::string(key));
	return found == tags.end() ? std::string_view{} : std::string_view(found->second);
}

bool is_bored(const tags_t &tags)
{
	const auto tunnel = tag_value(tags, "tunnel");
	const auto railway = tag_value(tags, "railway");
	return (tunnel == "yes" && (tags.contains("highway") || tags.contains("railway"))) ||
		   (railway == "subway" && tunnel != "no");
}

using GeoPoint = std::pair<double, double>;

std::unordered_map<std::uint64_t, GeoPoint> wanted_coordinates(
		const osm_parser::RawOsmDocument &data,
		const std::unordered_set<std::uint64_t> &wanted)
{
	std::unordered_map<std::uint64_t, GeoPoint> coordinates;
	coordinates.reserve(wanted.size());
	for (const auto &node : data.nodes)
		if (wanted.contains(node.id) && node.lat && node.lon &&
				std::isfinite(*node.lat) && std::isfinite(*node.lon))
			coordinates.emplace(node.id, GeoPoint{*node.lat, *node.lon});
	return coordinates;
}
} // namespace

std::vector<std::pair<double, double>> bored_ways_in(
		const osm_parser::RawOsmDocument &data)
{
	std::unordered_set<std::uint64_t> wanted;
	for (const auto &way : data.ways)
		if (is_bored(way.tags))
			wanted.insert(way.node_refs.begin(), way.node_refs.end());
	if (wanted.empty())
		return {};
	const auto coordinates = wanted_coordinates(data, wanted);
	std::vector<std::pair<double, double>> points;
	for (const auto &way : data.ways) {
		if (!is_bored(way.tags))
			continue;
		std::vector<GeoPoint> line;
		const auto flush = [&] {
			if (line.empty())
				return;
			points.push_back(line.front());
			for (std::size_t i = 1; i < line.size(); ++i) {
				const auto [lat_a, lon_a] = line[i - 1];
				const auto [lat_b, lon_b] = line[i];
				const double mean_lat = (lat_a + lat_b) * 0.5 * std::numbers::pi / 180.0;
				const double dx =
						(lon_b - lon_a) * METRES_PER_DEGREE * std::cos(mean_lat);
				const double dz = (lat_b - lat_a) * METRES_PER_DEGREE;
				const auto steps = std::max<std::size_t>(
						1, static_cast<std::size_t>(
								   std::ceil(std::hypot(dx, dz) / BORE_STEP_M)));
				for (std::size_t k = 1; k <= steps; ++k) {
					const double t = static_cast<double>(k) / steps;
					points.emplace_back(
							lat_a + (lat_b - lat_a) * t, lon_a + (lon_b - lon_a) * t);
				}
			}
			line.clear();
		};
		for (const auto id : way.node_refs) {
			const auto found = coordinates.find(id);
			if (found == coordinates.end())
				flush();
			else
				line.push_back(found->second);
		}
		flush();
	}
	return points;
}

std::vector<StationHint> station_hints_in(const osm_parser::RawOsmDocument &data)
{
	struct PlatformNodes
	{
		int depth{};
		std::vector<std::uint64_t> nodes;
		bool tagged_underground{};
	};
	std::unordered_map<std::uint64_t, const osm_parser::RawWay *> ways;
	for (const auto &way : data.ways)
		ways.emplace(way.id, &way);
	std::vector<PlatformNodes> platforms;
	const auto add_platform = [&](const tags_t &tags, std::vector<std::uint64_t> nodes) {
		if (!stations::is_rail_platform(tags) || stations::tagged_surface(tags) ||
				stations::is_low(tags))
			return;
		platforms.push_back({stations::depth_key(tags).value_or(-1), std::move(nodes),
				stations::tagged_underground(tags)});
	};
	for (const auto &way : data.ways)
		add_platform(way.tags, way.node_refs);
	for (const auto &relation : data.relations) {
		std::vector<std::uint64_t> nodes;
		for (const auto &member : relation.members) {
			if (member.type != "way")
				continue;
			const auto found = ways.find(member.ref);
			if (found != ways.end())
				nodes.insert(nodes.end(), found->second->node_refs.begin(),
						found->second->node_refs.end());
		}
		add_platform(relation.tags, std::move(nodes));
	}
	if (platforms.empty())
		return {};
	const bool all_tagged = std::all_of(platforms.begin(), platforms.end(),
			[](const PlatformNodes &platform) { return platform.tagged_underground; });
	std::vector<const osm_parser::RawWay *> bores;
	if (!all_tagged) {
		const auto is_station_railway_bore = [](const tags_t &tags) {
			const auto railway = tag_value(tags, "railway");
			return (railway == "rail" || railway == "subway" || railway == "light_rail" ||
						   railway == "narrow_gauge" || railway == "monorail") &&
				   is_bored(tags);
		};
		for (const auto &way : data.ways)
			if (is_station_railway_bore(way.tags))
				bores.push_back(&way);
	}
	std::unordered_set<std::uint64_t> wanted;
	for (const auto &platform : platforms)
		wanted.insert(platform.nodes.begin(), platform.nodes.end());
	for (const auto *way : bores)
		wanted.insert(way->node_refs.begin(), way->node_refs.end());
	const auto at = wanted_coordinates(data, wanted);
	std::optional<GeoPoint> origin;
	for (const auto &platform : platforms) {
		const auto found = std::find_if(platform.nodes.begin(), platform.nodes.end(),
				[&](std::uint64_t id) { return at.contains(id); });
		if (found != platform.nodes.end()) {
			origin = at.at(*found);
			break;
		}
	}
	if (!origin)
		return {};
	const double m_per_lon =
			METRES_PER_DEGREE * std::cos(origin->first * std::numbers::pi / 180.0);
	const auto local = [&](GeoPoint point) {
		return GeoPoint{(point.second - origin->second) * m_per_lon,
				(point.first - origin->first) * METRES_PER_DEGREE};
	};
	std::vector<std::pair<GeoPoint, GeoPoint>> segments;
	for (const auto *way : bores) {
		std::optional<GeoPoint> previous;
		for (const auto id : way->node_refs) {
			const auto found = at.find(id);
			if (found == at.end()) {
				previous.reset();
				continue;
			}
			const auto current = local(found->second);
			if (previous)
				segments.emplace_back(*previous, current);
			previous = current;
		}
	}
	const auto beside_bore = [&](GeoPoint point) {
		for (const auto &[a, b] : segments) {
			const double dx = b.first - a.first, dz = b.second - a.second;
			const double t = std::clamp(
					((point.first - a.first) * dx + (point.second - a.second) * dz) /
							(dx * dx + dz * dz + 1e-9),
					0.0, 1.0);
			if (std::hypot(a.first + t * dx - point.first,
						a.second + t * dz - point.second) <= BORE_PLATFORM_M)
				return true;
		}
		return false;
	};
	struct Shape
	{
		int depth{};
		std::vector<GeoPoint> points;
		GeoPoint centre{}, heading{};
	};
	std::vector<Shape> shapes;
	for (const auto &platform : platforms) {
		Shape shape;
		shape.depth = platform.depth;
		for (const auto id : platform.nodes) {
			const auto found = at.find(id);
			if (found != at.end())
				shape.points.push_back(found->second);
		}
		if (shape.points.empty())
			continue;
		GeoPoint centre{};
		for (const auto point : shape.points) {
			const auto p = local(point);
			centre.first += p.first;
			centre.second += p.second;
		}
		centre.first /= shape.points.size();
		centre.second /= shape.points.size();
		if (!platform.tagged_underground && !beside_bore(centre))
			continue;
		double xx = 0.0, zz = 0.0, xz = 0.0;
		for (const auto point : shape.points) {
			const auto p = local(point);
			const double dx = p.first - centre.first, dz = p.second - centre.second;
			xx += dx * dx;
			zz += dz * dz;
			xz += dx * dz;
		}
		const double angle = 0.5 * std::atan2(2.0 * xz, xx - zz);
		shape.centre = centre;
		shape.heading = {std::cos(angle), std::sin(angle)};
		shapes.push_back(std::move(shape));
	}
	const auto parallel = [](GeoPoint a, GeoPoint b) {
		return std::abs(a.first * b.first + a.second * b.second) >= 0.9;
	};
	std::vector<StationHint> result;
	for (std::size_t i = 0; i < shapes.size(); ++i) {
		std::vector<GeoPoint> crossing_headings;
		for (std::size_t j = 0; j < shapes.size(); ++j) {
			const auto &a = shapes[i];
			const auto &b = shapes[j];
			// Match Rust: equal-depth crossing platforms may both rank each other.
			if (i == j || b.depth < a.depth ||
					std::hypot(a.centre.first - b.centre.first,
							a.centre.second - b.centre.second) > INTERCHANGE_M ||
					parallel(a.heading, b.heading))
				continue;
			if (std::none_of(crossing_headings.begin(), crossing_headings.end(),
						[&](GeoPoint heading) { return parallel(heading, b.heading); }))
				crossing_headings.push_back(b.heading);
		}
		const int rank =
				static_cast<int>(std::min<std::size_t>(2, crossing_headings.size()));
		const int depth = stations::HALL_DEPTH + stations::HALL_STACK * rank;
		for (const auto point : shapes[i].points)
			result.push_back({point.first, point.second, depth});
	}
	return result;
}

TerrainHints overture_terrain_hints_in(const osm_parser::RawOsmDocument &data,
		double scale, const ::arnis::overture::OvertureData &overture_data)
{
	TerrainHints hints;
	if (!overture_data.hints.empty())
		hints.tall_structures = tall_structures_in(data, scale, &overture_data.hints);
	hints.tall_structures.insert(hints.tall_structures.end(),
			overture_data.tall_structures.begin(), overture_data.tall_structures.end());
	return hints;
}

void raise_peaks(std::vector<std::vector<double>> &grid, const geographic::LLBBox &bbox,
		const std::vector<PeakHint> &peaks, double meters_per_cell,
		std::optional<std::size_t> seam_pad)
{
	const auto height = grid.size();
	const auto width = height ? grid.front().size() : 0;
	if (width < 2 || height < 2 || !std::isfinite(meters_per_cell) ||
			meters_per_cell <= 0.0)
		return;
	const double min_lat = bbox.min().lat(), max_lat = bbox.max().lat();
	const double min_lon = bbox.min().lng(), max_lon = bbox.max().lng();
	if (!(max_lat > min_lat) || !(max_lon > min_lon))
		return;
	std::vector<Summit> summits;
	for (const auto &peak : peaks) {
		if (!std::isfinite(peak.ele_m) || peak.lat < min_lat || peak.lat > max_lat ||
				peak.lon < min_lon || peak.lon > max_lon)
			continue;
		summits.push_back({(peak.lon - min_lon) / (max_lon - min_lon) * (width - 1),
				(max_lat - peak.lat) / (max_lat - min_lat) * (height - 1), peak.ele_m});
	}
	std::vector<Summit> tagged;
	for (const auto &summit : summits)
		if (summit.ele + SUMMIT_NOISE_M >=
				highest_within(grid, summit.gx, summit.gy, SEARCH_M, meters_per_cell))
			tagged.push_back(summit);
	std::vector<Lift> lifts;
	for (const auto &summit : summits)
		if (auto lift = plan_lift(
					grid, summit.ele, summit.gx, summit.gy, meters_per_cell, seam_pad))
			lifts.push_back(std::move(*lift));
	std::sort(lifts.begin(), lifts.end(), [](const Lift &a, const Lift &b) {
		if (a.ele != b.ele)
			return a.ele > b.ele;
		if (a.gx != b.gx)
			return a.gx < b.gx;
		return a.gy < b.gy;
	});
	for (const auto &lift : lifts) {
		if (highest_within(grid, lift.gx, lift.gy, SEARCH_M, meters_per_cell) >=
				lift.ele - MIN_DEFICIT_M)
			continue;
		const double reach_m = static_cast<double>(lift.reach) * meters_per_cell;
		std::vector<Summit> lower;
		for (const auto &summit : tagged) {
			const double distance = std::hypot(summit.gx - lift.gx, summit.gy - lift.gy) *
									meters_per_cell;
			if (summit.ele < lift.ele &&
					distance < reach_m + (lift.ele - summit.ele) / CONE_SLOPE)
				lower.push_back(summit);
		}
		apply_lift(lift, grid, meters_per_cell, lower);
	}
}

std::vector<std::pair<double, double>> model_cells(
		const std::vector<std::vector<double>> &grid, const geographic::LLBBox &bbox,
		const std::vector<TallModel> &models)
{
	const auto height = grid.size();
	const auto width = height ? grid.front().size() : 0;
	std::vector<std::pair<double, double>> result;
	if (width < 2 || height < 2)
		return result;
	for (const auto &model : models) {
		std::size_t x, y;
		if (grid_cell(bbox, width, height, model.lat, model.lon, x, y) &&
				std::isfinite(grid[y][x]))
			result.emplace_back(grid[y][x], model.height_m);
	}
	return result;
}
} // namespace arnis::elevation
