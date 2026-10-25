#include "peaks.h"

#include "../coordinate_system/geographic/llbbox.h"
#include "../models_3d/wikidata/wikidata_index.h"
#include "../osm_parser.h"
#include "../strict_parse.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
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
		const auto first = s.find_first_not_of(" \t\r\n");
		if (first == std::string::npos) {
			s.clear();
			return;
		}
		const auto last = s.find_last_not_of(" \t\r\n");
		s = s.substr(first, last - first + 1);
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
	std::string value = input;
	auto trim = [](std::string &s) {
		const auto first = s.find_first_not_of(" \t\r\n");
		if (first == std::string::npos) {
			s.clear();
			return;
		}
		const auto last = s.find_last_not_of(" \t\r\n");
		s = s.substr(first, last - first + 1);
	};
	trim(value);
	while (!value.empty() && (value.back() == 'm' || value.back() == 'M'))
		value.pop_back();
	trim(value);
	if (value.empty())
		return std::nullopt;
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
	const auto first = qid->second.find_first_not_of(" \t\r\n");
	if (first == std::string::npos)
		return std::nullopt;
	const auto last = qid->second.find_last_not_of(" \t\r\n");
	const auto *entry =
			models_3d::lookup_wikidata(qid->second.substr(first, last - first + 1));
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

PeakChannel PeakSource::channel()
{
	auto promise = std::make_shared<std::promise<TerrainHints>>();
	auto future = promise->get_future().share();
	return {PeakSender(std::move(promise)), PeakSource(std::move(future))};
}

PeakSource PeakSource::none()
{
	std::promise<TerrainHints> promise;
	auto future = promise.get_future().share();
	promise.set_value({});
	return PeakSource(std::move(future));
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
		return future.get();
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

TerrainHints terrain_hints_in(const osm_parser::RawOsmDocument &data)
{
	return {peaks_in(data), tall_models_in(data)};
}

void raise_peaks(std::vector<std::vector<double>> &grid, const geographic::LLBBox &bbox,
		const std::vector<PeakHint> &peaks, double meters_per_cell)
{
	const auto height = grid.size();
	const auto width = height ? grid.front().size() : 0;
	if (width < 2 || height < 2 || !std::isfinite(meters_per_cell) ||
			meters_per_cell <= 0.0)
		return;
	for (const auto &peak : peaks) {
		// `RangeInclusive::contains` in the Rust implementation rejects NaN.
		// Ordered comparisons alone do not, so reject non-finite public hints
		// here before they can propagate NaNs through the elevation grid.
		if (!std::isfinite(peak.ele_m))
			continue;
		std::size_t cx, cy;
		if (!grid_cell(bbox, width, height, peak.lat, peak.lon, cx, cy))
			continue;
		const double gx = (peak.lon - bbox.min().lng()) /
						  (bbox.max().lng() - bbox.min().lng()) * (width - 1);
		const double gy = (bbox.max().lat() - peak.lat) /
						  (bbox.max().lat() - bbox.min().lat()) * (height - 1);
		const double at_node = grid[cy][cx];
		if (!std::isfinite(at_node))
			continue;
		const auto search = radius_cells(SEARCH_M, meters_per_cell);
		double dem_top = std::numeric_limits<double>::lowest();
		cells_within(cx, cy, search, width, height, [&](std::size_t x, std::size_t y) {
			if (std::isfinite(grid[y][x]))
				dem_top = std::max(dem_top, grid[y][x]);
		});
		const double shortfall = peak.ele_m - dem_top;
		if (!(shortfall >= MIN_DEFICIT_M && shortfall <= MAX_DEFICIT_M) ||
				shortfall > local_relief(grid, cx, cy, meters_per_cell))
			continue;
		const double lift = peak.ele_m - at_node;
		const auto reach = radius_cells(3.0 * SIGMA_M, meters_per_cell);
		cells_within(cx, cy, reach, width, height, [&](std::size_t x, std::size_t y) {
			double &v = grid[y][x];
			if (!std::isfinite(v))
				return;
			const double dx = static_cast<double>(x) - gx;
			const double dy = static_cast<double>(y) - gy;
			const double d = std::hypot(dx, dy) * meters_per_cell;
			const double room = std::max(peak.ele_m - CONE_SLOPE * d - v, 0.0);
			v += std::min(lift * std::exp(-std::pow(d / SIGMA_M, 2.0)), room);
		});
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
