#include "fm_ll_shapes.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace arnis
{
namespace
{
using Json = nlohmann::json;
using RawRing = std::vector<std::pair<double, double>>;
using RawPolygon = std::vector<RawRing>;

void set_error(std::string *error, std::string value)
{
	if (error)
		*error = std::move(value);
}

std::optional<RawRing> json_ring(const Json &value, std::string *error)
{
	if (!value.is_array()) {
		set_error(error, "a ring is not a list of positions");
		return std::nullopt;
	}
	RawRing ring;
	for (const auto &position : value) {
		if (!position.is_array() || position.size() < 2 || !position[0].is_number() ||
				!position[1].is_number()) {
			set_error(error, "a position is not [longitude, latitude]");
			return std::nullopt;
		}
		// GeoJSON coordinate order is longitude, latitude.
		ring.emplace_back(position[1].get<double>(), position[0].get<double>());
	}
	return ring;
}

bool collect_geojson(const Json &value, std::vector<RawPolygon> &out, unsigned depth,
		std::string *error)
{
	if (depth > 16) {
		set_error(error, "GeoJSON nests too deep");
		return false;
	}
	const auto type =
			value.is_object() && value.contains("type") && value["type"].is_string()
					? value["type"].get<std::string>()
					: std::string{};
	if (type == "FeatureCollection") {
		if (!value.contains("features") || !value["features"].is_array()) {
			set_error(error, "FeatureCollection has no features");
			return false;
		}
		for (const auto &feature : value["features"])
			if (!collect_geojson(feature, out, depth + 1, error))
				return false;
		return true;
	}
	if (type == "Feature") {
		if (!value.contains("geometry") || value["geometry"].is_null())
			return true;
		return collect_geojson(value["geometry"], out, depth + 1, error);
	}
	if (type == "GeometryCollection") {
		if (!value.contains("geometries") || !value["geometries"].is_array()) {
			set_error(error, "GeometryCollection has no geometries");
			return false;
		}
		for (const auto &geometry : value["geometries"])
			if (!collect_geojson(geometry, out, depth + 1, error))
				return false;
		return true;
	}
	const Json *coordinates = value.is_object() && value.contains("coordinates")
									  ? &value["coordinates"]
									  : nullptr;
	if (type != "Polygon" && type != "MultiPolygon")
		return true;
	if (!coordinates) {
		set_error(error, "no Polygon or MultiPolygon geometry found");
		return false;
	}
	auto parse_polygon = [&](const Json &rings) {
		if (!rings.is_array()) {
			set_error(error, "Polygon coordinates are not a list of rings");
			return false;
		}
		RawPolygon polygon;
		for (const auto &source_ring : rings) {
			auto ring = json_ring(source_ring, error);
			if (!ring)
				return false;
			polygon.push_back(std::move(*ring));
		}
		out.push_back(std::move(polygon));
		return true;
	};
	if (type == "Polygon")
		return parse_polygon(*coordinates);
	if (type == "MultiPolygon") {
		if (!coordinates->is_array()) {
			set_error(error, "MultiPolygon coordinates are not a list");
			return false;
		}
		for (const auto &polygon : *coordinates)
			if (!parse_polygon(polygon))
				return false;
		return true;
	}
	set_error(error, "no Polygon or MultiPolygon geometry found");
	return false;
}

bool on_ring(const LLShapes::Ring &ring, const geographic::LLPoint &point)
{
	const auto x = point.lng(), y = point.lat();
	for (std::size_t i = 0; i < ring.size(); ++i) {
		const auto &a = ring[i];
		const auto &b = ring[(i + 1) % ring.size()];
		const auto ax = a.lng(), ay = a.lat(), bx = b.lng(), by = b.lat();
		const auto cross = (bx - ax) * (y - ay) - (by - ay) * (x - ax);
		if (std::abs(cross) <= 1e-12 * (1.0 + std::max(std::abs(ax), std::abs(bx))) &&
				x >= std::min(ax, bx) - 1e-12 && x <= std::max(ax, bx) + 1e-12 &&
				y >= std::min(ay, by) - 1e-12 && y <= std::max(ay, by) + 1e-12)
			return true;
	}
	return false;
}

bool inside_ring(const LLShapes::Ring &ring, const geographic::LLPoint &point)
{
	bool inside = false;
	for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
		const auto &a = ring[i];
		const auto &b = ring[j];
		if (((a.lat() > point.lat()) != (b.lat() > point.lat())) &&
				point.lng() < (b.lng() - a.lng()) * (point.lat() - a.lat()) /
											  (b.lat() - a.lat()) +
									  a.lng())
			inside = !inside;
	}
	return inside;
}
}

std::optional<LLShapes> LLShapes::create(
		const std::vector<Polygon> &source, std::string *error)
{
	if (source.empty()) {
		set_error(error, "No shape was given.");
		return std::nullopt;
	}
	std::size_t point_count = 0;
	LLShapes result;
	double global_min_lng = 180, global_max_lng = -180;
	for (const auto &polygon : source) {
		if (polygon.empty()) {
			set_error(error, "shape has no outline");
			return std::nullopt;
		}
		Polygon checked;
		for (const auto &input : polygon) {
			point_count += input.size();
			if (point_count > MAX_POINTS) {
				set_error(error, "the shapes exceed the supported 200000 points");
				return std::nullopt;
			}
			Ring ring;
			for (const auto &point : input) {
				if (ring.empty() || ring.back().lat() != point.lat() ||
						ring.back().lng() != point.lng())
					ring.push_back(point);
			}
			if (ring.size() > 1 && ring.front().lat() == ring.back().lat() &&
					ring.front().lng() == ring.back().lng())
				ring.pop_back();
			if (ring.size() < 3) {
				set_error(error, "an outline needs at least three different points");
				return std::nullopt;
			}
			double min_lat = 90, max_lat = -90, min_lng = 180, max_lng = -180;
			for (const auto &point : ring) {
				min_lat = std::min(min_lat, point.lat());
				max_lat = std::max(max_lat, point.lat());
				min_lng = std::min(min_lng, point.lng());
				max_lng = std::max(max_lng, point.lng());
			}
			global_min_lng = std::min(global_min_lng, min_lng);
			global_max_lng = std::max(global_max_lng, max_lng);
			double area = 0;
			const auto &origin = ring.front();
			for (std::size_t i = 0; i < ring.size(); ++i) {
				const auto &a = ring[i];
				const auto &b = ring[(i + 1) % ring.size()];
				area += (a.lng() - origin.lng()) * (b.lat() - origin.lat()) -
						(b.lng() - origin.lng()) * (a.lat() - origin.lat());
			}
			if (max_lng - min_lng > 180.0) {
				set_error(
						error, "the outline spans more than half the globe east to west");
				return std::nullopt;
			}
			if (max_lat <= min_lat || max_lng <= min_lng ||
					std::abs(area) <= 1e-9 * (max_lat - min_lat) * (max_lng - min_lng)) {
				set_error(error, "the outline encloses no area");
				return std::nullopt;
			}
			checked.push_back(std::move(ring));
		}
		result.polygons_.push_back(std::move(checked));
	}
	if (global_max_lng - global_min_lng > 180.0) {
		set_error(
				error, "the shapes together span more than half the globe east to west");
		return std::nullopt;
	}
	return result;
}

std::optional<std::vector<std::pair<double, double>>> LLShapes::parse_points(
		const std::string &text, std::string *error)
{
	std::string normalized = text;
	for (auto &c : normalized)
		if (c == ';' || c == '\n' || c == '\t')
			c = ' ';
	std::istringstream input(normalized);
	std::vector<std::pair<double, double>> result;
	std::string token;
	while (input >> token) {
		const auto comma = token.find(',');
		if (comma == std::string::npos ||
				token.find(',', comma + 1) != std::string::npos) {
			set_error(error, "'" + token + "' is not a lat,lng point");
			return std::nullopt;
		}
		try {
			std::size_t used_lat = 0, used_lng = 0;
			const double lat = std::stod(token.substr(0, comma), &used_lat);
			const double lng = std::stod(token.substr(comma + 1), &used_lng);
			if (used_lat != comma || used_lng != token.size() - comma - 1 ||
					!std::isfinite(lat) || !std::isfinite(lng))
				throw std::invalid_argument("invalid coordinate");
			result.emplace_back(lat, lng);
		} catch (...) {
			set_error(error, "'" + token + "' is not a finite lat,lng point");
			return std::nullopt;
		}
	}
	return result;
}

std::optional<std::vector<LLShapes::Polygon>> LLShapes::from_geojson(
		const std::string &text, std::string *error)
{
	try {
		std::vector<RawPolygon> raw;
		if (!collect_geojson(Json::parse(text), raw, 0, error))
			return std::nullopt;
		if (raw.empty()) {
			set_error(error, "no Polygon or MultiPolygon geometry found");
			return std::nullopt;
		}
		std::vector<Polygon> result;
		for (const auto &polygon : raw) {
			Polygon parsed;
			for (const auto &ring : polygon) {
				Ring points;
				for (const auto &[lat, lng] : ring)
					points.emplace_back(lat, lng);
				parsed.push_back(std::move(points));
			}
			result.push_back(std::move(parsed));
		}
		return result;
	} catch (const std::exception &e) {
		set_error(error, std::string("not valid GeoJSON: ") + e.what());
		return std::nullopt;
	}
}

geographic::LLBBox LLShapes::bbox() const
{
	double min_lat = 90, max_lat = -90, min_lng = 180, max_lng = -180;
	for (const auto &polygon : polygons_)
		for (const auto &ring : polygon)
			for (const auto &point : ring) {
				min_lat = std::min(min_lat, point.lat());
				max_lat = std::max(max_lat, point.lat());
				min_lng = std::min(min_lng, point.lng());
				max_lng = std::max(max_lng, point.lng());
			}
	return {min_lat, min_lng, max_lat, max_lng};
}

bool LLShapes::contains(const geographic::LLPoint &point) const
{
	for (const auto &polygon : polygons_) {
		bool inside = false;
		for (const auto &ring : polygon) {
			if (on_ring(ring, point))
				return true;
			if (inside_ring(ring, point))
				inside = !inside;
		}
		if (inside)
			return true;
	}
	return false;
}
}
