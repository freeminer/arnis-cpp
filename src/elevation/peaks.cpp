#include "peaks.h"

#include "../coordinate_system/geographic/llbbox.h"
#include "../building_suppression.h"
#include "../data_processing.h"
#include "../element_processing/attractions.h"
#include "../element_processing/buildings.h"
#include "../models_3d/wikidata/wikidata_index.h"
#include "../mapillary/geometry.h"
#include "../osm_parser.h"
#include "../overture.h"
#include "../strict_parse.h"
#include "../unicode_text.h"
#include "postprocess.h"

#include <algorithm>
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
		auto hints = future.get();
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
		return future.get().tall_structures;
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

bool is_building_feature(const tags_t &tags)
{
	return tags.contains("building") || tags.contains("building:part") ||
		   tags.get("man_made") == "tower";
}

std::pair<double, double> footprint_extent_m(
		const std::vector<std::pair<double, double>> &points)
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
		const auto [width_m, length_m] = footprint_extent_m(points);
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

TerrainHints terrain_hints_in(const osm_parser::RawOsmDocument &data)
{
	return terrain_hints_in(data, 1.0, true, false);
}

TerrainHints terrain_hints_in(const osm_parser::RawOsmDocument &data, double scale,
		bool include_models, bool include_structures, bool overture_follows)
{
	TerrainHints hints;
	hints.peaks = peaks_in(data);
	if (include_models)
		hints.tall_models = tall_models_in(data);
	if (include_structures)
		hints.tall_structures = tall_structures_in(data, scale);
	hints.overture_follows = overture_follows;
	return hints;
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
	for (const auto &peak : peaks) {
		// `RangeInclusive::contains` in the Rust implementation rejects NaN.
		// Ordered comparisons alone do not, so reject non-finite public hints
		// here before they can propagate NaNs through the elevation grid.
		if (!std::isfinite(peak.ele_m))
			continue;
		std::size_t cx, cy;
		if (!grid_cell(bbox, width, height, peak.lat, peak.lon, cx, cy))
			continue;
		const auto reach = radius_cells(3.0 * SIGMA_M, meters_per_cell);
		if (seam_pad) {
			const auto reach_size = static_cast<std::size_t>(reach);
			if (*seam_pad > std::numeric_limits<std::size_t>::max() - reach_size)
				continue;
			const auto margin = *seam_pad + reach_size;
			if (cx < margin || cy < margin || margin >= width - cx ||
					margin >= height - cy)
				continue;
		}
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
