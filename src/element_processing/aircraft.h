#pragma once

#include "../osm_parser.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace arnis::element_processing::aircraft
{
using Point = std::pair<double, double>;

enum class Kind : std::uint8_t
{
	Runway,
	Taxi,
	Stand
};

struct Node
{
	Point at{};
	std::uint64_t id{};
};

struct Strip
{
	std::vector<std::size_t> nodes;
	std::vector<double> at;
	bool paved = true;
	std::uint64_t id{};
	double length() const { return at.empty() ? 0.0 : at.back(); }
};

struct Stand
{
	std::uint64_t id{};
	std::vector<std::size_t> path;
};

/// The runway/taxiway/stand graph for a connected airport area. Coordinates are
/// block-centres; ways that land on the same block intentionally share a node.
struct Field
{
	std::vector<Node> nodes;
	std::vector<std::vector<std::pair<std::size_t, Kind>>> next;
	std::vector<Strip> strips;
	std::vector<Stand> stands;
	std::vector<std::optional<std::pair<std::size_t, std::size_t>>> on_strip;

	static Field read(const std::vector<ProcessedElement> &elements, double scale);
	std::vector<std::size_t> fields() const;
	std::optional<std::vector<std::size_t>> taxi_route(
			std::pair<std::size_t, std::size_t> from,
			const std::vector<std::pair<std::size_t, std::size_t>> &free_edges,
			double runway_weight, double scale,
			const std::function<std::optional<double>(std::size_t, std::size_t)> &goal)
			const;

	double span(std::size_t a, std::size_t b) const;
	Point dir(std::size_t a, std::size_t b) const;
	std::optional<Kind> kind(std::size_t a, std::size_t b) const;
	bool taxis(std::size_t node) const;

private:
	void link(std::size_t a, std::size_t b, Kind kind);
	std::vector<Strip> read_strips(
			const std::map<std::pair<std::size_t, std::size_t>, std::uint64_t> &way_of,
			const std::set<std::pair<std::size_t, std::size_t>> &rough,
			double scale) const;
	void add_stand(std::uint64_t id, std::vector<std::size_t> line, double scale);
	void add_dead_ends(double scale);
	void one_line_a_stand(double scale);
	std::optional<std::size_t> reach_taxiway(
			std::size_t node, std::optional<Point> towards, double scale);
	std::optional<std::tuple<std::size_t, std::size_t, double>> nearest_taxiway(
			Point at, std::optional<Point> towards, double near, double far) const;
	std::size_t cut_taxiway(std::size_t a, std::size_t b, double t, std::size_t source);
};

} // namespace arnis::element_processing::aircraft
