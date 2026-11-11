#include "pedestrians.h"

#include "../clipping.h"
#include "cars.h"
#include "highways.h"
#include "traffic_lights.h"
#include "../datapack/mod.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <string_view>
#include <tuple>

namespace arnis::element_processing::pedestrians
{
namespace
{
constexpr double CLEAR = 0.35;
constexpr double DETOUR_MAX = 1.2;
constexpr double FURNITURE_CELL = 8.0;
constexpr double SQUARE_M2 = 100.0;
constexpr double ACROSS_MIN_M = 8.0;
constexpr double ACROSS_MAX_M = 120.0;
constexpr std::size_t ACROSS_EACH = 4;
constexpr std::uint64_t ACROSS_ID = 1ULL << 62;
constexpr double MAX_SPAN = 16.0;
constexpr std::size_t MAX_POINTS = 300'000;
constexpr std::size_t DAY_NEAR = 60;
constexpr std::size_t NIGHT_NEAR = 15;
constexpr std::array<double, 7> PACES{0.75, 0.85, 0.93, 1.0, 1.07, 1.16, 1.3};
constexpr double WALK_MPS = 1.5;
constexpr std::size_t LANES_PER_LINE = 256;
constexpr std::size_t CELLS_PER_LINE = 256;
constexpr double FOOTFALL_M = 75.0;
constexpr double KERB_BACK = 1.5;
constexpr double BESIDE_M = 6.0;
constexpr double SCRAP_M = 30.0;
constexpr double SPOT_CELL = 32.0;
constexpr double SINK = 0.3;
constexpr double FLOAT = 0.6;

struct CellPairHash
{
	std::size_t operator()(const std::pair<int, int> &p) const noexcept
	{
		const auto a = std::hash<int>{}(p.first);
		return a ^ (std::hash<int>{}(p.second) + 0x9e3779b9U + (a << 6) + (a >> 2));
	}
};

std::pair<int, int> cell(double x, double z)
{
	return {static_cast<int>(std::floor(x / FURNITURE_CELL)),
			static_cast<int>(std::floor(z / FURNITURE_CELL))};
}

double segment_distance(std::pair<double, double> p, std::pair<double, double> a,
		std::pair<double, double> b)
{
	const double dx = b.first - a.first, dz = b.second - a.second;
	const double length2 = dx * dx + dz * dz;
	const double t =
			length2 > 0.0
					? std::clamp(((p.first - a.first) * dx + (p.second - a.second) * dz) /
										 length2,
							  0.0, 1.0)
					: 0.0;
	return std::hypot(p.first - a.first - t * dx, p.second - a.second - t * dz);
}

double segment_segment_distance(std::pair<double, double> a, std::pair<double, double> b,
		std::pair<double, double> p, std::pair<double, double> q)
{
	const auto cross = [](auto o, auto u, auto v) {
		return (u.first - o.first) * (v.second - o.second) -
			   (u.second - o.second) * (v.first - o.first);
	};
	const double d1 = cross(a, b, p), d2 = cross(a, b, q);
	const double d3 = cross(p, q, a), d4 = cross(p, q, b);
	if ((d1 > 0.0) != (d2 > 0.0) && (d3 > 0.0) != (d4 > 0.0))
		return 0.0;
	return std::min({segment_distance(a, p, q), segment_distance(b, p, q),
			segment_distance(p, a, b), segment_distance(q, a, b)});
}

std::optional<std::pair<double, double>> unit(double x, double z)
{
	const double len = std::hypot(x, z);
	if (len <= 1e-9)
		return std::nullopt;
	return std::pair{x / len, z / len};
}

struct Measured
{
	std::vector<std::pair<double, double>> points;
	std::vector<double> at{0.0};

	explicit Measured(std::vector<std::pair<double, double>> line) :
			points(std::move(line))
	{
		for (std::size_t i = 1; i < points.size(); ++i)
			at.push_back(at.back() + std::hypot(points[i].first - points[i - 1].first,
											 points[i].second - points[i - 1].second));
	}
	double length() const { return at.empty() ? 0.0 : at.back(); }
	std::pair<double, double> point_at(double distance) const
	{
		if (points.empty())
			return {};
		if (points.size() == 1)
			return points.front();
		distance = std::clamp(distance, 0.0, length());
		const auto found = std::lower_bound(at.begin(), at.end(), distance);
		const std::size_t k = std::clamp<std::size_t>(
				static_cast<std::size_t>(found - at.begin()), 1, points.size() - 1);
		const double span = at[k] - at[k - 1];
		const double t = span > 0.0 ? (distance - at[k - 1]) / span : 0.0;
		return {points[k - 1].first + (points[k].first - points[k - 1].first) * t,
				points[k - 1].second + (points[k].second - points[k - 1].second) * t};
	}
};

using Mark = std::tuple<double, std::optional<std::uint8_t>, bool>;

void refine(double a, double b, const std::function<double(double)> &y_at, unsigned depth,
		std::vector<Mark> &out)
{
	if (depth >= 6 || b - a < 2.0)
		return;
	const auto pieces = static_cast<std::size_t>(std::ceil((b - a) / MAX_SPAN));
	if (pieces > 1) {
		const double step = (b - a) / pieces;
		for (std::size_t j = 0; j < pieces; ++j) {
			const double end = a + step * (j + 1);
			refine(end - step, end, y_at, depth + 1, out);
			if (j + 1 < pieces)
				out.emplace_back(end, std::nullopt, false);
		}
		return;
	}
	const double mid = (a + b) / 2.0;
	if (std::abs(y_at(mid) - (y_at(a) + y_at(b)) / 2.0) >= 1.0) {
		refine(a, mid, y_at, depth + 1, out);
		out.emplace_back(mid, std::nullopt, false);
		refine(mid, b, y_at, depth + 1, out);
	}
}

std::vector<std::pair<double, double>> lane_line(
		const std::vector<const ProcessedNode *> &nodes, double offset, bool keep_left)
{
	std::vector<std::pair<double, double>> centre;
	centre.reserve(nodes.size());
	for (const auto *node : nodes)
		centre.emplace_back(node->x + 0.5, node->z + 0.5);
	const auto direction = [&](std::size_t a, std::size_t b) {
		return unit(
				centre[b].first - centre[a].first, centre[b].second - centre[a].second);
	};
	std::vector<std::pair<double, double>> result;
	result.reserve(centre.size());
	for (std::size_t k = 0; k < centre.size(); ++k) {
		std::optional<std::pair<double, double>> into, out;
		for (std::size_t j = k; j > 0 && !into; --j)
			if (std::hypot(centre[k].first - centre[j - 1].first,
						centre[k].second - centre[j - 1].second) >= 2.0)
				into = direction(j - 1, k);
		for (std::size_t j = k + 1; j < centre.size() && !out; ++j)
			if (std::hypot(centre[j].first - centre[k].first,
						centre[j].second - centre[k].second) >= 2.0)
				out = direction(k, j);
		const auto near_ = [&]() -> std::optional<std::pair<double, double>> {
			if (k > 0)
				if (auto d = direction(k - 1, k))
					return d;
			if (k + 1 < centre.size())
				return direction(k, k + 1);
			return std::nullopt;
		};
		std::pair<double, double> d{1.0, 0.0};
		if (into && out)
			d = unit(into->first + out->first, into->second + out->second).value_or(*out);
		else if (into)
			d = *into;
		else if (out)
			d = *out;
		else if (const auto fallback = near_())
			d = *fallback;
		const std::pair<double, double> side =
				keep_left ? std::pair{d.second, -d.first} : std::pair{-d.second, d.first};
		result.emplace_back(centre[k].first + side.first * offset,
				centre[k].second + side.second * offset);
	}
	return result;
}

bool is_crossing_way(const tags_t &tags)
{
	return tags.get("footway") == "crossing" || tags.get("path") == "crossing" ||
		   tags.get("cycleway") == "crossing";
}

bool crossing_signalled(const tags_t &tags)
{
	return tags.get("crossing") == "traffic_signals" ||
		   tags.get("crossing:signals") == "yes";
}

bool node_signalled(const tags_t &tags)
{
	return crossing_signalled(tags) || tags.get("highway") == "traffic_signals";
}
} // namespace

std::optional<double> walk_density(const tags_t &tags)
{
	const auto highway = tags.get("highway");
	if (highway.empty() || off_limits(tags) || tags.get("area") == "yes")
		return std::nullopt;
	const auto yes = [&tags](const char *key) {
		const auto value = tags.get(key);
		return value == "yes" || value == "designated" || value == "permissive";
	};
	if (highway == "pedestrian")
		return 1.0 / 8.0;
	if (highway == "living_street")
		return 1.0 / 25.0;
	if (highway == "footway")
		return tags.get("footway") == "sidewalk" || tags.get("footway") == "crossing"
					   ? 1.0 / 30.0
					   : 1.0 / 45.0;
	if (highway == "steps")
		return 1.0 / 40.0;
	if (highway == "path")
		return yes("foot") ? 1.0 / 60.0 : 1.0 / 120.0;
	if (highway == "cycleway" && yes("foot"))
		return 1.0 / 150.0;
	if (highway == "track")
		return 1.0 / 300.0;
	if (highway == "bridleway")
		return 1.0 / 400.0;
	return std::nullopt;
}

bool off_limits(const tags_t &tags)
{
	const auto layer = tags.get("layer");
	const auto level = tags.get("level");
	const auto access = tags.get("access");
	const auto foot = tags.get("foot");
	const auto yes = [](const std::string &v) {
		return v == "yes" || v == "designated" || v == "permissive";
	};
	const auto closed = [](const std::string &v) {
		return v == "no" || v == "private" || v == "use_sidepath";
	};
	bool underground_layer = false;
	if (!layer.empty()) {
		int parsed_layer{};
		const auto [end, error] =
				std::from_chars(layer.data(), layer.data() + layer.size(), parsed_layer);
		const auto bridge = tags.get("bridge");
		underground_layer = error == std::errc{} && end == layer.data() + layer.size() &&
							parsed_layer < 0 &&
							(!tags.contains("bridge") || bridge == "no");
	}
	return (tags.contains("tunnel") && tags.get("tunnel") != "no") || underground_layer ||
		   (tags.contains("level") && level != "0") || tags.get("indoor") == "yes" ||
		   tags.contains("conveying") || (closed(access) && !yes(foot)) || closed(foot);
}

bool draws_people(const tags_t &tags)
{
	if (tags.contains("shop"))
		return true;
	const auto amenity = tags.get("amenity");
	if (amenity == "restaurant" || amenity == "cafe" || amenity == "fast_food" ||
			amenity == "bar" || amenity == "pub" || amenity == "ice_cream" ||
			amenity == "bank" || amenity == "pharmacy" || amenity == "marketplace" ||
			amenity == "theatre" || amenity == "cinema" || amenity == "library" ||
			amenity == "townhall" || amenity == "place_of_worship" ||
			amenity == "university" || amenity == "school")
		return true;
	const auto tourism = tags.get("tourism");
	if (tourism == "attraction" || tourism == "museum" || tourism == "gallery" ||
			tourism == "viewpoint" || tourism == "hotel")
		return true;
	const auto railway = tags.get("railway");
	return railway == "station" || railway == "halt" ||
		   tags.get("public_transport") == "station";
}

std::size_t Footfall::CellHash::operator()(const std::pair<int, int> &p) const noexcept
{
	const auto a = std::hash<int>{}(p.first);
	return a ^ (std::hash<int>{}(p.second) + 0x9e3779b9U + (a << 6) + (a >> 2));
}

Footfall::Footfall(const std::vector<std::pair<double, double>> &destinations,
		double cell_size) : cell_size_(std::max(cell_size, 8.0))
{
	for (const auto &[x, z] : destinations)
		++counts_[{static_cast<int>(std::floor(x / cell_size_)),
				static_cast<int>(std::floor(z / cell_size_))}];
}

double Footfall::at(std::pair<double, double> point) const
{
	const int x = static_cast<int>(std::floor(point.first / cell_size_));
	const int z = static_cast<int>(std::floor(point.second / cell_size_));
	std::uint32_t near_ = 0;
	for (int dx = -1; dx <= 1; ++dx)
		for (int dz = -1; dz <= 1; ++dz)
			if (const auto found = counts_.find({x + dx, z + dz}); found != counts_.end())
				near_ += found->second;
	return 1.0 + static_cast<double>(std::min<std::uint32_t>(near_, 30)) / 6.0;
}

namespace
{
std::vector<std::pair<ProcessedWay, double>> across_squares_clipped(
		const std::vector<ProcessedElement> &elements,
		const std::vector<const ProcessedWay *> &walk_ways,
		const std::unordered_map<std::uint64_t, std::size_t> &uses, double scale)
{
	std::unordered_map<std::uint64_t, std::tuple<int, std::uint64_t, bool>> roads;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (way.nodes.size() < 2)
			continue;
		if (highways::cars::drivable(way.tags)) {
			const int half = highways::highway_block_range(
					way.tags.get("highway"), way.tags, scale);
			for (const auto &node : way.nodes) {
				auto [it, inserted] = roads.try_emplace(node.id, half, way.id, false);
				if (!inserted) {
					std::get<0>(it->second) = std::max(std::get<0>(it->second), half);
					std::get<2>(it->second) |= std::get<1>(it->second) != way.id;
				}
			}
		}
	}
	std::vector<Square> squares;
	for (const auto &element : elements)
		if (auto square = Square::of(element, scale))
			squares.push_back(std::move(*square));
	if (squares.empty())
		return {};
	constexpr double GRID = 64.0;
	const auto square_cell = [=](std::pair<double, double> p) {
		return std::pair{static_cast<int>(std::floor(p.first / GRID)),
				static_cast<int>(std::floor(p.second / GRID))};
	};
	std::unordered_map<std::pair<int, int>, std::vector<std::size_t>, CellPairHash> grid;
	for (std::size_t i = 0; i < squares.size(); ++i) {
		const auto a = square_cell(squares[i].min), b = square_cell(squares[i].max);
		for (int cx = a.first - 1; cx <= b.first + 1; ++cx)
			for (int cz = a.second - 1; cz <= b.second + 1; ++cz)
				grid[{cx, cz}].push_back(i);
	}
	const auto centre = [](const ProcessedNode &node) {
		return std::pair{node.x + 0.5, node.z + 0.5};
	};
	using Road = std::tuple<std::pair<double, double>, std::pair<double, double>, double>;
	std::vector<std::vector<Road>> near_roads(squares.size());
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (!highways::cars::drivable(way.tags))
			continue;
		const double half =
				highways::highway_block_range(way.tags.get("highway"), way.tags, scale);
		for (std::size_t s = 1; s < way.nodes.size(); ++s) {
			const auto a = centre(way.nodes[s - 1]), b = centre(way.nodes[s]);
			std::unordered_set<std::size_t> hit;
			const auto steps =
					static_cast<std::size_t>(std::ceil(
							std::hypot(b.first - a.first, b.second - a.second) / GRID)) +
					1;
			for (std::size_t k = 0; k <= steps; ++k) {
				const double t = static_cast<double>(k) / steps;
				const auto p = std::pair{a.first + (b.first - a.first) * t,
						a.second + (b.second - a.second) * t};
				const auto found = grid.find(square_cell(p));
				if (found == grid.end())
					continue;
				for (const auto i : found->second)
					if (hit.insert(i).second && squares[i].near_(p, GRID + half))
						near_roads[i].emplace_back(a, b, half);
			}
		}
	}
	struct Door
	{
		const ProcessedNode *node;
		std::vector<std::uint64_t> ways;
	};
	std::vector<std::vector<Door>> doors(squares.size());
	for (const auto *walk_way : walk_ways) {
		const auto &way = *walk_way;
		for (std::size_t k = 0; k < way.nodes.size(); ++k) {
			const auto &node = way.nodes[k];
			if (roads.contains(node.id) || clipping::is_invented_node_id(node.id))
				continue;
			const auto p = centre(node);
			const auto found = grid.find(square_cell(p));
			if (found == grid.end())
				continue;
			for (const auto i : found->second) {
				const auto &square = squares[i];
				if (!square.near_(p, 2.0))
					continue;
				const bool on = square.outline.contains(node.id);
				const bool inside = square.contains(p);
				const bool edge =
						k == 0 || k + 1 == way.nodes.size() ||
						(uses.contains(node.id) && uses.at(node.id) > 1) ||
						!square.contains(centre(way.nodes[k - (k > 0)])) ||
						!square.contains(
								centre(way.nodes[std::min(k + 1, way.nodes.size() - 1)]));
				if (!(on || (edge && (inside || square.by_edge(p, 2.0)))))
					continue;
				auto &list = doors[i];
				const auto existing = std::find_if(list.begin(), list.end(),
						[&](const Door &door) { return door.node->id == node.id; });
				if (existing != list.end()) {
					existing->ways.push_back(way.id);
				} else if (std::all_of(list.begin(), list.end(), [&](const Door &door) {
							   const auto q = centre(*door.node);
							   return std::hypot(q.first - p.first, q.second - p.second) >
									  3.0;
						   })) {
					list.push_back({&node, {way.id}});
				}
			}
		}
	}
	std::vector<std::pair<ProcessedWay, double>> lines;
	for (std::size_t i = 0; i < squares.size(); ++i) {
		const auto &square = squares[i];
		const auto clear = [&](auto a, auto b) {
			const double length = std::hypot(b.first - a.first, b.second - a.second);
			const auto end = static_cast<int>(length - 1.0);
			for (int k = 2; k < end; ++k) {
				const auto p = std::pair{a.first + (b.first - a.first) * k / length,
						a.second + (b.second - a.second) * k / length};
				if (!square.contains(p) ||
						(k >= 4 && k <= length - 4.0 && square.by_edge(p, 1.5)))
					return false;
			}
			for (const auto &[p, q, half] : near_roads[i])
				if (segment_segment_distance(a, b, p, q) <= half + 1.0)
					return false;
			return true;
		};
		std::vector<std::pair<std::size_t, std::size_t>> pairs;
		for (std::size_t j = 0; j < doors[i].size(); ++j) {
			const auto a = centre(*doors[i][j].node);
			std::vector<std::pair<std::size_t, double>> candidates;
			for (std::size_t k = 0; k < doors[i].size(); ++k) {
				if (j == k)
					continue;
				const auto shared = std::any_of(
						doors[i][k].ways.begin(), doors[i][k].ways.end(), [&](auto id) {
							return std::find(doors[i][j].ways.begin(),
										   doors[i][j].ways.end(),
										   id) != doors[i][j].ways.end();
						});
				if (shared)
					continue;
				const auto b = centre(*doors[i][k].node);
				const double distance =
						std::hypot(b.first - a.first, b.second - a.second);
				if (distance >= ACROSS_MIN_M * scale && distance <= ACROSS_MAX_M * scale)
					candidates.emplace_back(k, distance);
			}
			std::sort(candidates.begin(), candidates.end(),
					[](const auto &x, const auto &y) { return x.second < y.second; });
			if (candidates.size() > 4 * ACROSS_EACH)
				candidates.resize(4 * ACROSS_EACH);
			std::vector<std::pair<double, double>> headings;
			for (const auto &[k, distance] : candidates) {
				(void)distance;
				if (headings.size() >= ACROSS_EACH)
					break;
				const auto b = centre(*doors[i][k].node);
				const auto heading = unit(b.first - a.first, b.second - a.second);
				if (!heading)
					continue;
				if (std::any_of(headings.begin(), headings.end(),
							[&](const auto &h) {
								return h.first * heading->first +
											   h.second * heading->second >
									   0.866;
							}) ||
						!clear(a, b))
					continue;
				headings.push_back(*heading);
				const std::pair<std::size_t, std::size_t> pair{
						std::min(j, k), std::max(j, k)};
				if (std::find(pairs.begin(), pairs.end(), pair) == pairs.end())
					pairs.push_back(pair);
			}
		}
		double metres = 0.0;
		for (const auto &[a, b] : pairs) {
			const auto p = centre(*doors[i][a].node), q = centre(*doors[i][b].node);
			metres += std::hypot(q.first - p.first, q.second - p.second) / scale;
		}
		if (metres <= 0.0)
			continue;
		const double density = std::min(square.people / (2.0 * metres), 1.0 / 8.0);
		for (const auto &[a, b] : pairs) {
			const auto bare = [](const ProcessedNode &node) {
				ProcessedNode copy{};
				copy.id = node.id;
				copy.x = node.x;
				copy.z = node.z;
				return copy;
			};
			ProcessedWay way{};
			way.id = ACROSS_ID | lines.size();
			way.nodes = {bare(*doors[i][a].node), bare(*doors[i][b].node)};
			way.tags.emplace("highway", "footway");
			lines.emplace_back(std::move(way), density);
		}
	}
	return lines;
}
} // namespace

// Keep the original library helper API for callers that do not have a prepared,
// clipped walk network. The generator uses the overload above with Rust's
// precomputed path and node-use inputs.
std::vector<std::pair<ProcessedWay, double>> across_squares(
		const std::vector<ProcessedElement> &elements, double scale)
{
	std::vector<const ProcessedWay *> walk_ways;
	std::unordered_map<std::uint64_t, std::size_t> uses;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (way.nodes.size() < 2 || !walk_density(way.tags))
			continue;
		walk_ways.push_back(&way);
		for (const auto &node : way.nodes)
			++uses[node.id];
	}
	return across_squares_clipped(elements, walk_ways, uses, scale);
}

WalkNetwork WalkNetwork::build(const std::vector<ProcessedElement> &elements,
		double scale, bool keep_left, bool signals, const cartesian::XZBBox &world,
		const SurfaceY &surface_y)
{
	WalkNetwork network;
	network.scale_ = scale;
	network.size_ = std::clamp(scale, 0.4, 3.0);
	network.keep_left_ = keep_left;
	const ::XZBBox clipping_world =
			::XZBBox(world.min_x(), world.min_z(), world.max_x(), world.max_z())
					.with_outline(world.outline());
	struct WayRef
	{
		const ProcessedWay *way;
		int half;
		double density;
		bool across;
	};
	struct RoadUse
	{
		int half;
		std::uint64_t first_way;
		bool shared{};
	};
	std::vector<WayRef> ways;
	std::vector<std::pair<const ProcessedWay *, std::pair<int, double>>> found_walks;
	std::unordered_map<std::uint64_t, RoadUse> roads;
	std::unordered_map<std::uint64_t, std::size_t> uses;
	std::vector<std::pair<double, double>> destinations;
	std::vector<Item> obstacles;
	for (const auto &element : elements) {
		if (element.is_way()) {
			const auto &way = element.as_way();
			if (way.nodes.size() >= 2) {
				if (const auto density = walk_density(way.tags)) {
					const int half = highways::highway_block_range(
							way.tags.get("highway"), way.tags, scale);
					found_walks.emplace_back(&way, std::pair{half, *density});
				}
			}
			if (highways::cars::drivable(way.tags)) {
				const int half = highways::highway_block_range(
						way.tags.get("highway"), way.tags, scale);
				for (const auto &node : way.nodes) {
					auto [it, inserted] =
							roads.try_emplace(node.id, RoadUse{half, way.id, false});
					if (!inserted) {
						it->second.half = std::max(it->second.half, half);
						it->second.shared |= it->second.first_way != way.id;
					}
				}
			}
			if (draws_people(way.tags) && !way.nodes.empty()) {
				double x = 0.0, z = 0.0;
				for (const auto &node : way.nodes) {
					x += node.x;
					z += node.z;
				}
				destinations.emplace_back(x / way.nodes.size(), z / way.nodes.size());
			}
		} else if (element.is_node()) {
			const auto &node = element.as_node();
			if (const auto item = furniture(node))
				obstacles.push_back(*item);
			if (draws_people(node.tags))
				destinations.emplace_back(node.x, node.z);
		}
	}
	const auto clipped_walks = clipping::cut_ways_to_outline(found_walks, clipping_world);
	ways.reserve(clipped_walks.size());
	for (const auto &clipped : clipped_walks) {
		const auto &way = clipped.way();
		if (way.nodes.size() >= 2)
			ways.push_back({&way, clipped.value.first, clipped.value.second, false});
	}
	const Footfall footfall(destinations, FOOTFALL_M * scale);
	const Furniture furniture_index(std::move(obstacles));
	for (const auto &ref : ways)
		for (const auto &node : ref.way->nodes)
			++uses[node.id];
	std::vector<const ProcessedWay *> clipped_walk_ways;
	clipped_walk_ways.reserve(ways.size());
	for (const auto &way : ways)
		clipped_walk_ways.push_back(way.way);
	const auto generated_squares =
			across_squares_clipped(elements, clipped_walk_ways, uses, scale);
	std::vector<std::pair<const ProcessedWay *, double>> found_squares;
	found_squares.reserve(generated_squares.size());
	for (const auto &[way, density] : generated_squares)
		found_squares.emplace_back(&way, density);
	const auto squares = clipping::cut_ways_to_outline(found_squares, clipping_world);
	for (const auto &square : squares)
		for (const auto &node : square.way().nodes)
			++uses[node.id];
	for (const auto &square : squares) {
		const auto &way = square.way();
		if (way.nodes.size() >= 2)
			ways.push_back(
					{&way, highways::highway_block_range("footway", way.tags, scale),
							square.value, true});
	}
	struct Piece
	{
		const WayRef *way;
		std::size_t first, last;
	};
	std::vector<Piece> pieces;
	for (const auto &ref : ways) {
		std::size_t first = 0;
		for (std::size_t k = 1; k < ref.way->nodes.size(); ++k)
			if (k + 1 == ref.way->nodes.size() || uses[ref.way->nodes[k].id] > 1) {
				pieces.push_back({&ref, first, k});
				first = k;
			}
	}
	const auto road_at = [&roads](const ProcessedNode &node,
								 std::uint64_t way) -> std::optional<int> {
		const auto found = roads.find(node.id);
		if (found == roads.end() ||
				(!found->second.shared && found->second.first_way == way))
			return std::nullopt;
		return found->second.half;
	};
	const auto at_edge = [&world](const ProcessedNode &node) {
		return clipping::is_invented_node_id(node.id) || node.x - world.min_x() < 3 ||
			   world.max_x() - node.x < 3 || node.z - world.min_z() < 3 ||
			   world.max_z() - node.z < 3;
	};
	for (std::size_t piece_index = 0; piece_index < pieces.size(); ++piece_index) {
		const auto &piece = pieces[piece_index];
		const auto &way = *piece.way->way;
		const auto highway = way.tags.get("highway");
		const auto lane_count = (highway == "pedestrian" || highway == "living_street")
										? static_cast<std::size_t>(std::clamp(
												  (2 * piece.way->half + 1) / 4, 1, 3))
										: 1U;
		std::vector<const ProcessedNode *> forward;
		for (std::size_t k = piece.first; k <= piece.last; ++k)
			forward.push_back(&way.nodes[k]);
		const bool crossing = is_crossing_way(way.tags);
		const bool signalled =
				signals && crossing &&
				(crossing_signalled(way.tags) ||
						std::any_of(forward.begin(), forward.end(), [](const auto *node) {
							return node_signalled(node->tags);
						}));
		for (const bool backwards : {false, true}) {
			auto nodes = forward;
			if (backwards)
				std::reverse(nodes.begin(), nodes.end());
			for (std::size_t lane = 0; lane < lane_count; ++lane) {
				const double offset =
						(piece.way->half + 0.5) * (lane + 0.5) / lane_count * 0.8;
				const Measured path(lane_line(nodes, std::max(offset, 0.4), keep_left));
				const Measured middle(lane_line(nodes, 0.0, keep_left));
				if (path.points.size() < 2 || path.length() <= 1e-9)
					continue;
				const double stretch =
						path.length() > 0.0 ? middle.length() / path.length() : 1.0;
				std::vector<Mark> marks{
						{0.0, std::nullopt, false}, {path.length(), std::nullopt, false}};
				for (std::size_t k = 1; k + 1 < path.points.size(); ++k)
					marks.emplace_back(path.at[k], std::nullopt, false);
				const auto heading = [&](std::size_t k) {
					const auto a = path.points[std::min(
							k == 0 ? 0 : k - 1, path.points.size() - 2)];
					const auto b = path.points[std::min(k + 1, path.points.size() - 1)];
					return unit(b.first - a.first, b.second - a.second)
							.value_or(std::pair{1.0, 0.0});
				};
				const auto signal_group = [](auto d) -> std::uint8_t {
					return highways::group_of(d.first, d.second) == 'a' ? 1 : 2;
				};
				if (crossing && !road_at(*nodes.front(), way.id)) {
					std::get<1>(marks.front()) =
							signalled ? std::optional<std::uint8_t>(
												signal_group(heading(0)))
									  : std::nullopt;
					std::get<2>(marks.front()) = !signalled;
				}
				for (std::size_t k = 1; k < nodes.size(); ++k) {
					const auto &node = *nodes[k];
					if (k + 1 == nodes.size() && uses[node.id] < 2)
						continue;
					const auto road = road_at(node, way.id);
					if (!road || crossing)
						continue;
					const double s = std::max(path.at[k] - (*road + KERB_BACK), 0.0);
					const auto light = signals && node_signalled(node.tags)
											   ? std::optional<std::uint8_t>(
														 signal_group(heading(k)))
											   : std::nullopt;
					marks.emplace_back(s, light, !light.has_value());
				}
				const auto kerb = [&](const ProcessedNode &node) {
					const auto road = road_at(node, way.id);
					return road && uses[node.id] == 1 ? *road + KERB_BACK : 0.0;
				};
				const double lo = kerb(*nodes.front());
				const double hi = path.length() - kerb(*nodes.back());
				if (lo > 0.0 || hi < path.length()) {
					std::erase_if(marks, [&](const Mark &m) {
						return std::get<0>(m) < lo || std::get<0>(m) > hi;
					});
					if (hi - lo >= 2.0) {
						marks.emplace_back(lo, std::nullopt, false);
						marks.emplace_back(hi, std::nullopt, false);
					} else {
						marks.clear();
					}
				}
				std::sort(marks.begin(), marks.end(), [](const Mark &a, const Mark &b) {
					return std::get<0>(a) < std::get<0>(b);
				});
				std::vector<Mark> merged;
				for (const auto &mark : marks) {
					if (!merged.empty() &&
							std::get<0>(mark) - std::get<0>(merged.back()) < 0.5) {
						if (!std::get<1>(merged.back()))
							std::get<1>(merged.back()) = std::get<1>(mark);
						std::get<2>(merged.back()) |= std::get<2>(mark);
					} else {
						merged.push_back(mark);
					}
				}
				const auto y_at = [&](double distance) {
					const auto [x, z] = path.point_at(distance);
					return static_cast<double>(
							surface_y(way, static_cast<int>(std::floor(x)),
									static_cast<int>(std::floor(z))) +
							1);
				};
				std::vector<Mark> stations;
				for (const auto &mark : merged) {
					if (!stations.empty())
						refine(std::get<0>(stations.back()), std::get<0>(mark), y_at, 0,
								stations);
					stations.push_back(mark);
				}
				std::vector<Point> points, alternate;
				for (const auto &[distance, light, kerb_mark] : stations) {
					const auto to_point = [&](std::pair<double, double> p) {
						return Point{std::round(p.first * 10.0) / 10.0, y_at(distance),
								std::round(p.second * 10.0) / 10.0, light, kerb_mark};
					};
					const auto p = to_point(path.point_at(distance));
					if (world.contains(
								cartesian::XZPoint{static_cast<int>(std::floor(p.x)),
										static_cast<int>(std::floor(p.z))})) {
						points.push_back(p);
						alternate.push_back(
								to_point(middle.point_at(distance * stretch)));
					}
				}
				const bool across = piece.way->across;
				const auto plain = points;
				points = furniture_index.go_round(std::move(points));
				const auto unbent_plain = points == plain ? std::vector<Point>{} : plain;
				if (across)
					alternate.clear();
				else
					alternate = furniture_index.go_round(std::move(alternate));
				Lane built;
				built.points = std::move(points);
				built.alternate = std::move(alternate);
				built.plain = unbent_plain;
				built.density = piece.way->density *
								footfall.at(path.point_at(path.length() / 2.0)) * 2.0 /
								(lane_count + 1.0);
				built.across = across;
				built.out = at_edge(*nodes.back());
				network.lanes_.push_back(std::move(built));
				network.meta_.emplace_back(piece_index, backwards, nodes.front()->id,
						nodes.back()->id, crossing);
			}
		}
	}
	std::size_t point_count = 0;
	for (const auto &lane : network.lanes_)
		point_count += lane.points.size();
	if (point_count > MAX_POINTS) {
		std::vector<std::size_t> order(network.lanes_.size());
		std::iota(order.begin(), order.end(), 0);
		std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
			return network.lanes_[a].density > network.lanes_[b].density;
		});
		std::size_t kept = 0;
		for (const auto index : order) {
			kept += network.lanes_[index].points.size();
			if (kept > MAX_POINTS) {
				network.lanes_[index].points.clear();
				network.lanes_[index].alternate.clear();
			}
		}
	}
	network.link();
	network.untrap();
	network.build_spots();
	return network;
}

bool WalkNetwork::is_empty() const
{
	return spots_.empty();
}

std::size_t WalkNetwork::lane_count() const
{
	return std::count_if(lanes_.begin(), lanes_.end(),
			[](const Lane &lane) { return lane.points.size() >= 2; });
}

std::shared_ptr<water_probe::WalkProbe> WalkNetwork::probe() const
{
	std::vector<water_probe::Column> columns;
	const auto add = [&columns](const std::vector<Point> &points) {
		for (std::size_t i = 1; i < points.size(); ++i) {
			const auto &a = points[i - 1], &b = points[i];
			const auto n = static_cast<std::size_t>(
					std::max(1.0, std::ceil(std::hypot(b.x - a.x, b.z - a.z))));
			for (std::size_t j = 0; j < n; ++j) {
				const double t = static_cast<double>(j) / n;
				const double x = a.x + (b.x - a.x) * t;
				const double z = a.z + (b.z - a.z) * t;
				const double y = a.y + (b.y - a.y) * t;
				columns.emplace_back(static_cast<int>(std::floor(x)),
						static_cast<int>(std::floor(z)),
						static_cast<int>(std::lround(y)) - 1);
			}
		}
		if (!points.empty()) {
			const auto &p = points.back();
			columns.emplace_back(static_cast<int>(std::floor(p.x)),
					static_cast<int>(std::floor(p.z)),
					static_cast<int>(std::lround(p.y)) - 1);
		}
	};
	for (const auto &lane : lanes_) {
		add(lane.points);
		add(lane.alternate);
		add(lane.plain);
	}
	return std::make_shared<water_probe::WalkProbe>(columns);
}

namespace
{
struct WalkStep
{
	double x{}, y{}, z{};
	std::optional<std::size_t> point;
};

struct SettledWalk
{
	std::vector<Point> points;
	std::pair<bool, bool> cut{};
	std::vector<std::pair<int, int>> posts;
	double length() const
	{
		double total = 0.0;
		for (std::size_t i = 1; i < points.size(); ++i)
			total += std::hypot(
					points[i].x - points[i - 1].x, points[i].z - points[i - 1].z);
		return total;
	}
};

std::vector<WalkStep> walk_steps(const std::vector<Point> &points)
{
	std::vector<WalkStep> result;
	for (std::size_t i = 1; i < points.size(); ++i) {
		const auto &a = points[i - 1];
		const auto &b = points[i];
		const auto count = static_cast<std::size_t>(
				std::max(1.0, std::ceil(std::hypot(b.x - a.x, b.z - a.z))));
		for (std::size_t j = 0; j < count; ++j) {
			const double t = static_cast<double>(j) / count;
			result.push_back({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
					a.z + (b.z - a.z) * t, j == 0 ? std::optional(i - 1) : std::nullopt});
		}
	}
	if (!points.empty())
		result.push_back(
				{points.back().x, points.back().y, points.back().z, points.size() - 1});
	return result;
}

Point step_point(const WalkStep &step, double y)
{
	const auto tenth = [](double v) {
		return std::clamp(
				std::round(v * 10.0) / 10.0, std::floor(v), std::floor(v) + 0.9);
	};
	return {tenth(step.x), y, tenth(step.z), std::nullopt, false};
}

void bend_walk(const std::vector<WalkStep> &steps, const std::vector<double> &height,
		std::size_t a, std::size_t b, std::vector<Point> &out)
{
	if (b <= a + 1)
		return;
	const auto line = [&](std::size_t k) {
		return height[a] + (height[b] - height[a]) * static_cast<double>(k - a) /
								   static_cast<double>(b - a);
	};
	const auto offset = [&](std::size_t k) {
		return std::max(height[k] - line(k) - SINK, line(k) - height[k] - FLOAT);
	};
	std::size_t best = a + 1;
	for (std::size_t k = a + 2; k < b; ++k)
		if (offset(k) > offset(best))
			best = k;
	if (offset(best) <= 0.0)
		return;
	bend_walk(steps, height, a, best, out);
	out.push_back(step_point(steps[best], height[best]));
	bend_walk(steps, height, best, b, out);
}

std::optional<SettledWalk> settle_walk(
		const std::vector<Point> &points, const water_probe::WalkProbe &probe)
{
	const auto steps = walk_steps(points);
	std::vector<std::optional<double>> feet;
	feet.reserve(steps.size());
	std::optional<double> previous;
	std::optional<std::pair<int, int>> post;
	std::vector<std::pair<int, int>> posts;
	for (const auto &step : steps) {
		const int x = static_cast<int>(std::floor(step.x));
		const int z = static_cast<int>(std::floor(step.z));
		const int level = static_cast<int>(std::lround(step.y)) - 1;
		const auto footing = probe.get(x, z, level);
		std::optional<double> found;
		if (!footing) {
			found = step.y;
		} else {
			found = footing->nearest(previous.value_or(step.y));
			if (found && previous && std::abs(*found - *previous) > 1.0)
				found.reset();
		}
		if (found) {
			post.reset();
			previous = found;
			feet.push_back(found);
		} else if (footing && footing->thin() && (!post || *post == std::pair{x, z})) {
			if (!post)
				posts.emplace_back(x, z);
			post = std::pair{x, z};
			feet.push_back(previous.value_or(step.y));
		} else {
			post.reset();
			previous.reset();
			feet.emplace_back(std::nullopt);
		}
	}
	std::size_t best_a = 0, best_b = 0;
	std::optional<std::size_t> start;
	for (std::size_t k = 0; k <= feet.size(); ++k) {
		if (k < feet.size() && feet[k]) {
			if (!start)
				start = k;
		} else if (start) {
			if (k - *start > best_b - best_a) {
				best_a = *start;
				best_b = k;
			}
			start.reset();
		}
	}
	if (best_b - best_a < 4)
		return std::nullopt;
	const std::size_t last = best_b - 1;
	std::vector<double> high(feet.size(), 0.0);
	for (std::size_t k = best_a; k <= last; ++k) {
		const auto lo = std::max(best_a, k == 0 ? 0 : k - 1);
		const auto hi = std::min(last, k + 1);
		double value = -std::numeric_limits<double>::infinity();
		for (std::size_t j = lo; j <= hi; ++j)
			if (feet[j])
				value = std::max(value, *feet[j]);
		high[k] = std::isfinite(value) ? value : 0.0;
	}
	std::vector<std::size_t> selected;
	for (std::size_t k = best_a; k <= last; ++k)
		if (k == best_a || k == last || steps[k].point)
			selected.push_back(k);
	const auto point = [&](std::size_t k) {
		if (steps[k].point)
			return Point{points[*steps[k].point].x, high[k], points[*steps[k].point].z,
					points[*steps[k].point].light, points[*steps[k].point].kerb};
		return step_point(steps[k], high[k]);
	};
	SettledWalk settled;
	settled.cut = {best_a > 0, best_b < steps.size()};
	settled.posts = std::move(posts);
	for (std::size_t i = 1; i < selected.size(); ++i) {
		settled.points.push_back(point(selected[i - 1]));
		bend_walk(steps, high, selected[i - 1], selected[i], settled.points);
	}
	settled.points.push_back(point(selected.back()));
	return settled;
}
} // namespace

void WalkNetwork::finish(const water_probe::WalkProbe &probe)
{
	for (auto &lane : lanes_) {
		if (lane.points.size() >= 2) {
			auto side = settle_walk(lane.points, probe);
			if (!side || side->cut != std::pair{false, false}) {
				if (auto straight = settle_walk(lane.plain, probe);
						straight && (!side || straight->length() > side->length()))
					side = std::move(straight);
			}
			std::optional<SettledWalk> best;
			if (side && side->cut == std::pair{false, false}) {
				best = std::move(side);
			} else if (!lane.across) {
				auto middle = settle_walk(lane.alternate, probe);
				if (side && middle && side->length() >= middle->length() &&
						middle->cut != std::pair{false, false})
					best = std::move(side);
				else if (middle)
					best = std::move(middle);
				else
					best = std::move(side);
			}
			if (!best) {
				lane.points.clear();
				lane.cut = {false, false};
			} else {
				const auto final_points = best->points;
				lane.points = std::move(best->points);
				lane.cut = best->cut;
				lane.out &= !best->cut.second;
				if (!best->posts.empty()) {
					std::vector<Item> posts;
					posts.reserve(best->posts.size());
					for (const auto &[x, z] : best->posts)
						posts.push_back({x + 0.5, z + 0.5, 0.4, 0.4, 0.0});
					auto bent = Furniture(std::move(posts)).go_round(final_points);
					if (auto resettled = settle_walk(bent, probe);
							resettled && resettled->cut == std::pair{false, false})
						lane.points = std::move(resettled->points);
				}
			}
		}
		lane.alternate.clear();
		lane.plain.clear();
	}
	std::size_t points = 0;
	for (const auto &lane : lanes_)
		points += lane.points.size();
	if (points > MAX_POINTS) {
		std::vector<std::size_t> order(lanes_.size());
		std::iota(order.begin(), order.end(), 0);
		std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
			return lanes_[a].density > lanes_[b].density;
		});
		std::size_t kept = 0;
		for (const auto index : order) {
			kept += lanes_[index].points.size();
			if (kept > MAX_POINTS)
				lanes_[index].points.clear();
		}
	}
	link();
	untrap();
	build_spots();
}

void WalkNetwork::link()
{
	const auto usable = [](const Lane &lane) { return lane.points.size() >= 2; };
	std::unordered_map<std::uint64_t, std::vector<std::size_t>> starting;
	std::unordered_map<std::size_t, std::vector<std::size_t>> pieces;
	for (std::size_t i = 0; i < meta_.size() && i < lanes_.size(); ++i) {
		const auto &[piece, backwards, start, end, crossing] = meta_[i];
		(void)backwards;
		(void)crossing;
		if (usable(lanes_[i]) && !lanes_[i].cut.first)
			starting[start].push_back(i);
		pieces[piece].push_back(i);
	}
	std::vector<std::optional<std::size_t>> backs(lanes_.size());
	for (std::size_t i = 0; i < lanes_.size() && i < meta_.size(); ++i) {
		const auto [piece, backwards, start, end, crossing] = meta_[i];
		(void)start;
		(void)crossing;
		if (lanes_[i].points.empty())
			continue;
		const auto &last = lanes_[i].points.back();
		double best = std::numeric_limits<double>::infinity();
		for (const auto candidate : pieces[piece]) {
			if (candidate == i || std::get<1>(meta_[candidate]) == backwards ||
					!usable(lanes_[candidate]))
				continue;
			const auto &first = lanes_[candidate].points.front();
			const double gap = std::hypot(first.x - last.x, first.z - last.z);
			if (gap < best) {
				best = gap;
				backs[i] = candidate;
			}
		}
	}
	const double reach = std::max(BESIDE_M * scale_, 2.0);
	const auto cell = [reach](double x, double z) {
		return std::pair{static_cast<int>(std::floor(x / reach)),
				static_cast<int>(std::floor(z / reach))};
	};
	std::vector<std::vector<std::optional<std::size_t>>> beside(lanes_.size());
	for (std::size_t i = 0; i < lanes_.size(); ++i) {
		if (!backs[i])
			continue;
		const auto &back = lanes_[*backs[i]].points;
		std::unordered_map<std::pair<int, int>, std::vector<std::size_t>, CellPairHash>
				grid;
		for (std::size_t k = 0; k < back.size(); ++k)
			grid[cell(back[k].x, back[k].z)].push_back(k);
		for (const auto &point : lanes_[i].points) {
			const auto [cx, cz] = cell(point.x, point.z);
			std::optional<std::size_t> closest;
			double distance = reach;
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz)
					if (const auto found = grid.find({cx + dx, cz + dz});
							found != grid.end())
						for (const auto k : found->second) {
							const double gap =
									std::hypot(back[k].x - point.x, back[k].z - point.z);
							if (gap <= distance) {
								distance = gap;
								closest = k;
							}
						}
			beside[i].push_back(closest);
		}
	}
	for (std::size_t i = 0; i < lanes_.size() && i < meta_.size(); ++i) {
		const auto [piece, backwards, start, end, crossing] = meta_[i];
		(void)backwards;
		(void)start;
		(void)crossing;
		auto &lane = lanes_[i];
		for (const auto candidate : starting[end])
			if (std::get<0>(meta_[candidate]) != piece)
				lane.next.push_back(candidate);
		std::sort(lane.next.begin(), lane.next.end());
		lane.back = backs[i];
		const bool near_ = !beside[i].empty() && beside[i].back().has_value();
		lane.beside = std::move(beside[i]);
		if (lane.out)
			lane.next.clear();
		else if (lane.next.empty() || lane.cut.second)
			lane.next = backs[i] && near_ ? std::vector<std::size_t>{*backs[i]}
										 : std::vector<std::size_t>{};
	}
}

void WalkNetwork::untrap()
{
	const std::size_t n = lanes_.size();
	const auto length = [](const Lane &lane) {
		double sum = 0.0;
		for (std::size_t i = 1; i < lane.points.size(); ++i)
			sum += std::hypot(lane.points[i].x - lane.points[i - 1].x,
					lane.points[i].z - lane.points[i - 1].z);
		return sum;
	};
	for (;;) {
		std::vector<std::size_t> order;
		std::vector<bool> seen(n, false);
		for (std::size_t start = 0; start < n; ++start) {
			if (seen[start])
				continue;
			seen[start] = true;
			std::vector<std::pair<std::size_t, std::size_t>> stack{{start, 0}};
			while (!stack.empty()) {
				auto &[i, k] = stack.back();
				if (k < lanes_[i].next.size()) {
					const auto j = lanes_[i].next[k++];
					stack.emplace_back(i, k);
					if (j < n && !seen[j]) {
						seen[j] = true;
						stack.emplace_back(j, 0);
					}
				} else {
					order.push_back(i);
					stack.pop_back();
				}
			}
		}
		std::vector<std::vector<std::size_t>> into(n);
		for (std::size_t i = 0; i < n; ++i)
			for (const auto j : lanes_[i].next)
				if (j < n)
					into[j].push_back(i);
		std::vector<std::size_t> group(n, std::numeric_limits<std::size_t>::max());
		std::size_t groups = 0;
		for (auto it = order.rbegin(); it != order.rend(); ++it) {
			const auto start = *it;
			if (group[start] != std::numeric_limits<std::size_t>::max())
				continue;
			group[start] = groups;
			std::vector<std::size_t> stack{start};
			while (!stack.empty()) {
				const auto i = stack.back();
				stack.pop_back();
				for (const auto j : into[i])
					if (group[j] == std::numeric_limits<std::size_t>::max()) {
						group[j] = groups;
						stack.push_back(j);
					}
			}
			++groups;
		}
		std::vector<bool> open(groups, false);
		std::vector<double> size(groups, 0.0);
		for (std::size_t i = 0; i < n; ++i) {
			const auto g = group[i];
			size[g] += length(lanes_[i]);
			if (lanes_[i].out || lanes_[i].next.empty() ||
					std::any_of(lanes_[i].next.begin(), lanes_[i].next.end(),
							[&](auto j) { return j < n && group[j] != g; }))
				open[g] = true;
		}
		const auto trap = [&](std::size_t from, std::size_t g) {
			return from != g && !open[g] &&
				   (size[g] < 2.0 * SCRAP_M * scale_ || size[g] < size[from]);
		};
		bool changed = false;
		for (std::size_t i = 0; i < n; ++i) {
			const auto g = group[i];
			const auto before = lanes_[i].next.size();
			std::erase_if(
					lanes_[i].next, [&](auto j) { return j < n && trap(g, group[j]); });
			if (lanes_[i].next.size() == before)
				continue;
			changed = true;
			const bool near_ = !lanes_[i].beside.empty() && lanes_[i].beside.back();
			if (lanes_[i].next.empty() && near_ && lanes_[i].back &&
					!trap(g, group[*lanes_[i].back]))
				lanes_[i].next.push_back(*lanes_[i].back);
		}
		if (!changed)
			return;
	}
}

void WalkNetwork::build_spots()
{
	spots_.clear();
	const auto length = [](const Lane &lane) {
		double sum = 0.0;
		for (std::size_t i = 1; i < lane.points.size(); ++i)
			sum += std::hypot(lane.points[i].x - lane.points[i - 1].x,
					lane.points[i].z - lane.points[i - 1].z);
		return sum;
	};
	std::vector<std::size_t> root(lanes_.size());
	std::iota(root.begin(), root.end(), 0);
	const auto find = [&root](std::size_t k) {
		while (root[k] != k) {
			root[k] = root[root[k]];
			k = root[k];
		}
		return k;
	};
	for (std::size_t i = 0; i < lanes_.size(); ++i)
		for (const auto j : lanes_[i].next)
			if (j < root.size())
				root[find(i)] = find(j);
	std::unordered_map<std::size_t, double> total;
	for (std::size_t i = 0; i < lanes_.size(); ++i)
		total[find(i)] += length(lanes_[i]);
	double owed = 0.5;
	for (std::size_t i = 0; i < lanes_.size(); ++i) {
		const auto &lane = lanes_[i];
		if (lane.points.size() < 2 || lane.next.empty() ||
				(i < meta_.size() && std::get<4>(meta_[i])))
			continue;
		if (total[find(i)] < 2.0 * SCRAP_M * scale_)
			continue;
		const double len = length(lane);
		owed += len / scale_ * lane.density;
		const auto count = static_cast<std::size_t>(std::floor(owed));
		owed -= count;
		double at = 0.0;
		std::size_t k = 0;
		for (std::size_t j = 0; j < count; ++j) {
			const double s = len * (j + 0.5) / count;
			while (k + 2 < lane.points.size()) {
				const auto &p = lane.points[k], &q = lane.points[k + 1];
				const double segment = std::hypot(q.x - p.x, q.z - p.z);
				if (at + segment >= s)
					break;
				at += segment;
				++k;
			}
			if (lane.points[k].light || lane.points[k].kerb)
				continue;
			const auto &p = lane.points[k];
			const std::pair<int, int> key{static_cast<int>(std::floor(p.x / SPOT_CELL)),
					static_cast<int>(std::floor(p.z / SPOT_CELL))};
			spots_[key].emplace_back(i, k);
		}
	}
}

std::vector<std::pair<std::string, std::string>> WalkNetwork::pack_functions() const
{
	const auto format = [this](double value, int precision) {
		std::ostringstream out;
		out << std::fixed << std::setprecision(precision) << value * size_;
		return out.str();
	};
	std::array<int, PACES.size()> paces{};
	for (std::size_t i = 0; i < PACES.size(); ++i)
		paces[i] = static_cast<int>(
				std::max(20.0, std::round(WALK_MPS * PACES[i] * 0.1 * scale_ * 1000.0)));
	std::ostringstream network;
	network << "scoreboard players set #day_cap arnis.walk " << DAY_NEAR << '\n'
			<< "scoreboard players set #night_cap arnis.walk " << NIGHT_NEAR << '\n'
			<< "scoreboard players set #spot_cell arnis.walk "
			<< static_cast<int>(SPOT_CELL) << '\n'
			<< "data modify storage arnis:walk seg set value []\n"
			<< "data modify storage arnis:walk paces set value [I;";
	for (std::size_t i = 0; i < paces.size(); ++i)
		network << (i ? "," : "") << paces[i];
	network << "]\n"
			<< "data modify storage arnis:walk sizes set value [";
	constexpr std::array<double, 10> sizes{
			0.86, 0.9, 0.92, 0.94, 0.96, 0.98, 1.0, 1.02, 0.6, 0.68};
	for (std::size_t i = 0; i < sizes.size(); ++i)
		network << (i ? "," : "") << format(sizes[i], 3) << 'd';
	network << "]\n";
	for (std::size_t first = 0; first < lanes_.size(); first += LANES_PER_LINE) {
		const auto last = std::min(first + LANES_PER_LINE, lanes_.size());
		network << "data modify storage arnis:walk batch set value [";
		for (std::size_t i = first; i < last; ++i) {
			if (i != first)
				network << ',';
			const auto &lane = lanes_[i];
			if (lane.points.size() < 2) {
				network << "{p:[],n:[I;],b:-1,r:[I;],o:0b}";
				continue;
			}
			network << "{p:[";
			for (std::size_t j = 0; j < lane.points.size(); ++j) {
				const auto &point = lane.points[j];
				if (j)
					network << ',';
				network << "{x:" << format(datapack::macro_coord(point.x), 2)
						<< "d,y:" << std::fixed << std::setprecision(1) << point.y
						<< "d,z:" << format(datapack::macro_coord(point.z), 2);
				if (point.light)
					network << ",g:" << static_cast<unsigned>(*point.light) << 'b';
				if (point.kerb)
					network << ",c:1b";
				network << '}';
			}
			network << "],n:[I;";
			for (std::size_t j = 0; j < lane.next.size(); ++j)
				network << (j ? "," : "") << lane.next[j];
			network << "],b:";
			if (lane.back)
				network << *lane.back;
			else
				network << -1;
			network << ",r:[I;";
			for (std::size_t j = 0; j < lane.beside.size(); ++j)
				network << (j ? "," : "")
						<< (lane.beside[j] ? static_cast<long long>(*lane.beside[j])
										   : -1LL);
			network << "],o:" << (lane.out ? 1 : 0) << "b}";
		}
		network << "]\ndata modify storage arnis:walk seg append from storage arnis:walk batch[]\n";
	}
	network << "data remove storage arnis:walk batch\n"
			<< "data modify storage arnis:walk grid set value {}\n";
	for (auto it = spots_.begin(); it != spots_.end();) {
		network << "data modify storage arnis:walk grid merge value {";
		std::size_t in_batch = 0;
		while (it != spots_.end() && in_batch < CELLS_PER_LINE) {
			if (in_batch++)
				network << ',';
			network << '\"' << it->first.first << '_' << it->first.second << "\":[";
			for (std::size_t j = 0; j < it->second.size(); ++j) {
				if (j)
					network << ',';
				network << "[I;" << it->second[j].first << ',' << it->second[j].second
						<< ']';
			}
			network << ']';
			++it;
		}
		network << "}\n";
	}
	std::ostringstream moves;
	for (const int i : {3, 2, 4, 1, 5, 0, 6})
		moves << "execute if score @s arnis.wv matches " << paces[i]
			  << " run return run tp @s ^ ^ ^" << std::fixed << std::setprecision(3)
			  << paces[i] / 1000.0 << '\n';
	const auto ahead =
			std::string(
					"# Someone right in front: 1 to wait for a player; 4 to queue behind a walker who\n"
					"# stands, is all but touching, crosses or queues; 3 to step aside for one coming\n"
					"# the other way or held up behind; 2 to fall in behind one going the same way.\n"
					"# Queued 20 s, a walker goes on to the next point.\n") +
			"execute positioned ^ ^ ^" + format(0.9, 2) +
			" if entity @a[gamemode=!spectator,tag=!arnis.riding,distance=.." +
			format(0.7, 2) +
			"] run return 1\n"
			"execute if score @s arnis.wb matches 200.. run return 0\n"
			"scoreboard players operation #me arnis.walk = @s arnis.wk\n"
			"execute positioned ^ ^ ^" +
			format(0.35, 3) + " as @e[tag=arnis.walker,distance=.." + format(0.32, 3) +
			"] unless score @s arnis.wb matches 1.. run return 4\n"
			"execute positioned ^ ^ ^" +
			format(0.35, 3) + " as @e[tag=arnis.walker,distance=.." + format(0.32, 3) +
			"] if score @s arnis.wk < #me arnis.walk run return 4\n"
			"execute positioned ^ ^ ^" +
			format(0.9, 2) + " unless entity @e[tag=arnis.walker,distance=.." +
			format(0.45, 2) +
			"] run return 0\n"
			"scoreboard players operation #yaw arnis.walk = @s arnis.wy\n"
			"execute positioned ^ ^ ^" +
			format(0.9, 2) + " as @e[tag=arnis.walker,distance=.." + format(0.45, 2) +
			",sort=nearest,limit=1] run function arnis:walk/lead\n"
			"execute unless score #turn arnis.walk matches -135..135 run return 3\n"
			"execute positioned ^ ^ ^" +
			format(0.9, 2) + " if entity @e[tag=arnis.walker,distance=.." +
			format(0.45, 2) +
			",scores={arnis.ww=1..}] run return 4\n"
			"execute positioned ^ ^ ^" +
			format(0.9, 2) + " as @e[tag=arnis.walker,distance=.." + format(0.45, 2) +
			",scores={arnis.wb=1..199}] if score @s arnis.wk < #me arnis.walk run return 4\n"
			"execute if score #turn arnis.walk matches -45..45 run return 2\n"
			"execute if score #free arnis.walk matches 1 run return 4\nreturn 3\n";
	const auto aside = std::string("tag @s add arnis.w_aside\ntp @s ^") +
					   (keep_left_ ? format(0.4, 2) : "-" + format(0.4, 2)) +
					   " ^ ^\nfunction arnis:walk/aim\n";
	std::ostringstream gap;
	gap << "# A car crossing the way ahead coming close: wait for the gap.\n";
	constexpr std::array<std::string_view, 2> along{"-45..45", "135..-135"};
	constexpr std::array<std::string_view, 2> across{"45..135", "-135..-45"};
	for (const bool swapped : {false, true})
		for (const auto walker : (swapped ? across : along))
			for (const auto car : (swapped ? along : across))
				gap << "execute if entity @s[y_rotation=" << walker
					<< "] if entity @e[type=marker,tag=arnis.car,tag=!arnis.car.still,distance=.."
					<< format(9.0, 2) << ",y_rotation=" << car << "] run return 0\n";
	gap << "return 1\n";
	const auto behind = std::string("execute at @s run tp @s ^ ^ ^-") + format(1.7, 2) +
						"\nfunction arnis:walk/aim\n";
	return {{"data/arnis/function/walk/network.mcfunction", network.str()},
			{"data/arnis/function/walk/move.mcfunction", moves.str()},
			{"data/arnis/function/walk/ahead.mcfunction", ahead},
			{"data/arnis/function/walk/gap.mcfunction", gap.str()},
			{"data/arnis/function/walk/behind.mcfunction", behind},
			{"data/arnis/function/walk/aside.mcfunction", aside}};
}

std::optional<Item> furniture(const ProcessedNode &node)
{
	const auto layer = node.tags.get("layer");
	const auto level = node.tags.get("level");
	if ((!layer.empty() && layer.front() == '-') ||
			(!level.empty() && level.front() == '-'))
		return std::nullopt;
	const auto amenity = node.tags.get("amenity");
	const auto highway = node.tags.get("highway");
	const auto advertising = node.tags.get("advertising");
	double half = 0.0, along = 0.0, dx = 0.0;
	if (amenity == "waste_basket" || amenity == "waste_disposal" ||
			amenity == "vending_machine" || amenity == "atm" ||
			(amenity == "recycling" && node.tags.get("recycling_type") == "container")) {
		half = 0.5;
	} else if (amenity == "bench") {
		half = 0.5;
		along = 1.0;
	} else if (amenity == "drinking_water" || advertising == "flag") {
		half = 0.3;
	} else if (highway == "street_lamp" || advertising == "column") {
		half = 0.5;
	} else if (advertising == "billboard") {
		half = 0.5;
		along = 1.0;
	} else if (advertising == "poster_box") {
		half = 0.5;
		dx = 0.5;
	} else if (node.tags.get("natural") == "tree") {
		half = 0.5;
	} else if (node.tags.get("barrier") == "bollard") {
		half = 0.3;
	} else if (node.tags.get("power") == "pole") {
		half = 0.4;
	} else if (node.tags.get("emergency") == "fire_hydrant") {
		const auto type = node.tags.get("fire_hydrant:type");
		if (type == "underground" || type == "wall" || type == "pond")
			return std::nullopt;
		half = 0.5;
	} else {
		return std::nullopt;
	}
	return Item{node.x + 0.5 + dx, node.z + 0.5, half + dx, half, along};
}

std::size_t Furniture::CellHash::operator()(const std::pair<int, int> &p) const noexcept
{
	const auto a = std::hash<int>{}(p.first);
	return a ^ (std::hash<int>{}(p.second) + 0x9e3779b9U + (a << 6) + (a >> 2));
}

Furniture::Furniture(std::vector<Item> items) : items_(std::move(items))
{
	for (std::size_t i = 0; i < items_.size(); ++i)
		cells_[cell(items_[i].x, items_[i].z)].push_back(static_cast<std::uint32_t>(i));
}

std::vector<Point> Furniture::go_round(std::vector<Point> points) const
{
	if (cells_.empty() || points.size() < 2)
		return points;
	std::vector<double> at{0.0};
	for (std::size_t i = 1; i < points.size(); ++i)
		at.push_back(at.back() + std::hypot(points[i].x - points[i - 1].x,
										 points[i].z - points[i - 1].z));
	const double total = at.back();
	const auto dir = [&points](std::size_t i) {
		const auto &a = points[i];
		const auto &b = points[i + 1];
		const double len = std::max(std::hypot(b.x - a.x, b.z - a.z), 1e-9);
		return std::pair{(b.x - a.x) / len, (b.z - a.z) / len};
	};
	std::vector<std::uint32_t> near;
	for (std::size_t i = 0; i + 1 < points.size(); ++i) {
		const auto lo = cell(std::min(points[i].x, points[i + 1].x) - 3.0,
				std::min(points[i].z, points[i + 1].z) - 3.0);
		const auto hi = cell(std::max(points[i].x, points[i + 1].x) + 3.0,
				std::max(points[i].z, points[i + 1].z) + 3.0);
		for (int x = lo.first; x <= hi.first; ++x)
			for (int z = lo.second; z <= hi.second; ++z)
				if (const auto found = cells_.find({x, z}); found != cells_.end())
					near.insert(near.end(), found->second.begin(), found->second.end());
	}
	std::sort(near.begin(), near.end());
	near.erase(std::unique(near.begin(), near.end()), near.end());
	struct Bend
	{
		double at, side, plateau, ramp;
	};
	std::vector<Bend> bends;
	for (const auto index : near) {
		const auto &o = items_[index];
		std::optional<std::tuple<double, double, double, std::size_t>> best;
		for (std::size_t k = 0; k + 1 < points.size(); ++k) {
			const auto [ux, uz] = dir(k);
			const double len = at[k + 1] - at[k];
			if (len < 1e-6)
				continue;
			const double t = std::clamp(
					(o.x - points[k].x) * ux + (o.z - points[k].z) * uz, 0.0, len);
			const double px = points[k].x + ux * t, pz = points[k].z + uz * t;
			const double dist = std::hypot(o.x - px, o.z - pz);
			if (!best || dist < std::get<0>(*best)) {
				const double left = (o.z - points[k].z) * ux - (o.x - points[k].x) * uz;
				best = {dist, at[k] + t, left, k};
			}
		}
		if (!best)
			continue;
		const auto [dist, s, left, k] = *best;
		const auto [ux, uz] = dir(k);
		const double wide = o.hx * std::abs(uz) + o.hz * std::abs(ux) + CLEAR;
		if (dist >= wide)
			continue;
		const double aside = wide - dist + 0.1;
		const double plateau =
				o.hx * std::abs(ux) + o.hz * std::abs(uz) + o.along + CLEAR;
		const double room = std::min(s, total - s) - plateau - 0.1;
		if (aside > DETOUR_MAX || room < 0.6)
			continue;
		bends.push_back({s, left > 0.0 ? -aside : aside, plateau,
				std::min(std::max(1.5 * aside, 1.0), room)});
	}
	if (bends.empty())
		return points;
	const auto offset = [&bends](double q) {
		double result = 0.0;
		for (const auto &bend : bends) {
			const double d = std::abs(q - bend.at);
			const double v =
					bend.side *
					std::max(0.0, 1.0 - std::max(0.0, (d - bend.plateau) / bend.ramp));
			if (std::abs(v) > std::abs(result))
				result = v;
		}
		return result;
	};
	std::vector<std::pair<double, std::optional<std::size_t>>> keys;
	for (std::size_t i = 0; i < at.size(); ++i)
		keys.emplace_back(at[i], i);
	for (const auto &bend : bends)
		for (const double q : {bend.at - bend.plateau - bend.ramp, bend.at - bend.plateau,
					 bend.at + bend.plateau, bend.at + bend.plateau + bend.ramp})
			keys.emplace_back(q, std::nullopt);
	std::sort(keys.begin(), keys.end(), [](const auto &a, const auto &b) {
		if (a.first != b.first)
			return a.first < b.first;
		return a.second.has_value() > b.second.has_value();
	});
	std::vector<Point> out;
	double last = -std::numeric_limits<double>::infinity();
	for (const auto &[q, vertex] : keys) {
		if (!vertex && q - last < 0.05)
			continue;
		last = q;
		const auto upper = std::upper_bound(at.begin(), at.end(), q);
		const std::size_t k =
				std::clamp<std::size_t>(static_cast<std::size_t>(upper - at.begin()), 1,
						points.size() - 1) -
				1;
		double ux{}, uz{};
		if (vertex && *vertex > 0 && *vertex + 1 < points.size()) {
			const auto [px, pz] = dir(*vertex - 1);
			const auto [nx, nz] = dir(*vertex);
			const double x = px + nx, z = pz + nz, len = std::hypot(x, z);
			if (len < 1e-6) {
				std::tie(ux, uz) = dir(*vertex);
			} else {
				const double factor = (2.0 / len) / len;
				ux = x * factor;
				uz = z * factor;
			}
		} else {
			std::tie(ux, uz) = dir(vertex ? std::min(*vertex, points.size() - 2) : k);
		}
		Point p;
		if (vertex) {
			p = points[*vertex];
		} else {
			const double f =
					std::clamp((q - at[k]) / std::max(at[k + 1] - at[k], 1e-9), 0.0, 1.0);
			p = {points[k].x + (points[k + 1].x - points[k].x) * f,
					points[k].y + (points[k + 1].y - points[k].y) * f,
					points[k].z + (points[k + 1].z - points[k].z) * f};
		}
		const double side = offset(q);
		p.x -= uz * side;
		p.z += ux * side;
		out.push_back(p);
	}
	return out;
}

std::optional<Square> Square::of(const ProcessedElement &element, double scale)
{
	const tags_t *tags = nullptr;
	if (element.is_way())
		tags = &element.as_way().tags;
	else if (element.is_relation())
		tags = &element.as_relation().tags;
	if (!tags)
		return std::nullopt;
	const auto highway = tags->get("highway");
	const auto place = tags->get("place");
	double per_m2 = 0.0;
	if (highway == "pedestrian" || (highway.empty() && place == "square"))
		per_m2 = 1.0 / SQUARE_M2;
	else if (highway == "footway")
		per_m2 = 0.5 / SQUARE_M2;
	else
		return std::nullopt;
	if (off_limits(*tags) || tags->contains("building"))
		return std::nullopt;
	std::vector<std::pair<const std::vector<ProcessedNode> *, bool>> source_rings;
	if (element.is_way()) {
		const auto &way = element.as_way();
		if (tags->get("area") != "yes" && !highway.empty())
			return std::nullopt;
		source_rings.emplace_back(&way.nodes, false);
	} else {
		const auto &relation = element.as_relation();
		if (tags->get("type") != "multipolygon")
			return std::nullopt;
		for (const auto &member : relation.members) {
			if (member.role == ProcessedMemberRole::Outer)
				source_rings.emplace_back(&member.way->nodes, false);
			else if (member.role == ProcessedMemberRole::Inner)
				source_rings.emplace_back(&member.way->nodes, true);
		}
	}
	Square square;
	square.min = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
	square.max = {
			std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};
	double area = 0.0;
	for (const auto &[nodes, inner] : source_rings) {
		if (nodes->size() < 4 || nodes->front().x != nodes->back().x ||
				nodes->front().z != nodes->back().z)
			continue;
		std::vector<std::pair<double, double>> ring;
		ring.reserve(nodes->size());
		for (const auto &node : *nodes) {
			const std::pair point{node.x + 0.5, node.z + 0.5};
			ring.push_back(point);
			square.min.first = std::min(square.min.first, point.first);
			square.min.second = std::min(square.min.second, point.second);
			square.max.first = std::max(square.max.first, point.first);
			square.max.second = std::max(square.max.second, point.second);
			square.outline.insert(node.id);
		}
		double twice_area = 0.0;
		for (std::size_t i = 1; i < ring.size(); ++i)
			twice_area += ring[i - 1].first * ring[i].second -
						  ring[i].first * ring[i - 1].second;
		area += (inner ? -0.5 : 0.5) * std::abs(twice_area);
		square.rings.push_back(std::move(ring));
	}
	if (square.rings.empty() || scale == 0.0)
		return std::nullopt;
	square.people = area / (scale * scale) * per_m2;
	return square.people > 0.0 ? std::optional<Square>(std::move(square)) : std::nullopt;
}

bool Square::contains(std::pair<double, double> point) const
{
	bool inside = false;
	for (const auto &ring : rings)
		for (std::size_t i = 1; i < ring.size(); ++i) {
			const auto [x1, z1] = ring[i - 1];
			const auto [x2, z2] = ring[i];
			if ((z1 > point.second) != (z2 > point.second) &&
					point.first < x1 + (point.second - z1) / (z2 - z1) * (x2 - x1))
				inside = !inside;
		}
	return inside;
}

bool Square::near_(std::pair<double, double> point, double reach) const
{
	return point.first >= min.first - reach && point.first <= max.first + reach &&
		   point.second >= min.second - reach && point.second <= max.second + reach;
}

bool Square::by_edge(std::pair<double, double> point, double reach) const
{
	for (const auto &ring : rings)
		for (std::size_t i = 1; i < ring.size(); ++i)
			if (segment_distance(point, ring[i - 1], ring[i]) <= reach)
				return true;
	return false;
}
} // namespace arnis::element_processing::pedestrians
