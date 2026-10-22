#include "barriers.h"
#include "block_definitions.h"
#include "bresenham.h"
#include "connected_blocks.h"
#include "world_editor.h"
#include "highways.h"
#include "highway_tunnels.h"

#include <string>
#include <unordered_map>
#include <vector>
#include <tuple>
#include <cmath>
#include <optional>
#include <utility>
#include <functional>
#include <algorithm>
#include <limits>
#include <memory>
#include <unordered_set>

#include "../../../arnis_adapter.h"
#include "bridges.h"
namespace arnis
{

namespace barriers
{

const int BRIDGE_BARRIER_NEARBY_RADIUS = 2;

namespace
{
constexpr int ROAD_BUCKET_SHIFT = 5;
constexpr float ROAD_SAMPLE_STEP = 8.0f;
constexpr float ROAD_ALONG_COS = .9f;
constexpr int ROAD_MAX_SHIFT = 24;
constexpr int ROAD_EXTRA_SHIFT = 2;

using RoadBucket = std::pair<int, int>;
struct RoadBucketHash
{
	std::size_t operator()(const RoadBucket &p) const noexcept
	{
		const auto key = (std::uint64_t(static_cast<std::uint32_t>(p.first)) << 32) |
						 static_cast<std::uint32_t>(p.second);
		return std::hash<std::uint64_t>{}(key);
	}
};

RoadBucket road_bucket(int x, int z)
{
	return {x >> ROAD_BUCKET_SHIFT, z >> ROAD_BUCKET_SHIFT};
}

struct RoadSegment
{
	float ax{}, az{}, dx{}, dz{}, reach{};
	bool carriageway{};
	float distance(float x, float z) const
	{
		const float px = x - ax, pz = z - az;
		const float len2 = dx * dx + dz * dz;
		const float t = std::clamp((px * dx + pz * dz) / len2, 0.0f, 1.0f);
		return std::hypot(px - t * dx, pz - t * dz);
	}
	float chebyshev(float x, float z) const
	{
		const float px = x - ax, pz = z - az;
		const auto at = [&](float t) {
			return std::max(std::abs(px - t * dx), std::abs(pz - t * dz));
		};
		float best = std::min(at(0), at(1));
		for (const auto &[num, den] : {std::pair{px, dx}, std::pair{pz, dz},
					 std::pair{px - pz, dx - dz}, std::pair{px + pz, dx + dz}}) {
			if (den == 0)
				continue;
			const float t = num / den;
			if (t >= 0 && t <= 1)
				best = std::min(best, at(t));
		}
		return best;
	}
};
} // namespace

struct RoadClearance::Impl
{
	std::shared_ptr<const CoordinateBitmap> roads;
	std::vector<RoadSegment> segments;
	std::unordered_map<RoadBucket, std::vector<std::uint32_t>, RoadBucketHash> buckets;

	const RoadSegment *covering(int x, int z) const
	{
		const auto it = buckets.find(road_bucket(x, z));
		if (it == buckets.end())
			return nullptr;
		const RoadSegment *best = nullptr;
		float best_distance = std::numeric_limits<float>::max();
		for (const auto index : it->second) {
			const auto &segment = segments[index];
			const float d =
					segment.distance(static_cast<float>(x), static_cast<float>(z));
			if (segment.chebyshev(static_cast<float>(x), static_cast<float>(z)) <=
							segment.reach &&
					d < best_distance) {
				best = &segment;
				best_distance = d;
			}
		}
		return best;
	}
};

RoadClearance::RoadClearance() : impl_(std::make_shared<Impl>())
{
}

RoadClearance RoadClearance::build(const std::vector<ProcessedElement> &elements,
		const world_editor::WorldEditor &editor, double scale,
		std::shared_ptr<const CoordinateBitmap> roads)
{
	auto index = std::make_shared<Impl>();
	index->roads = std::move(roads);
	if (!index->roads)
		return RoadClearance(std::move(index));
	std::unordered_set<RoadBucket, RoadBucketHash> wanted;
	for (const auto &element : elements) {
		if (!element.is_way() || !element.tags().contains("barrier"))
			continue;
		const auto &way = element.as_way();
		for (std::size_t i = 1; i < way.nodes.size(); ++i)
			for (const auto &[x, y, z] : bresenham::bresenham_line(way.nodes[i - 1].x, 0,
						 way.nodes[i - 1].z, way.nodes[i].x, 0, way.nodes[i].z)) {
				(void)y;
				if (index->roads->contains(x, z))
					wanted.insert(road_bucket(x, z));
			}
	}
	if (wanted.empty())
		return RoadClearance(std::move(index));
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto highway_type = highways::surface_highway_type(way);
		if (!highway_type || bridges::is_bridge_way(way))
			continue;
		const float reach = static_cast<float>(highways::highway_block_range(
									*highway_type, way.tags, scale)) +
							.5f;
		const bool carriageway = highways::is_carriageway(*highway_type);
		const float margin = reach + ROAD_SAMPLE_STEP / 2.0f;
		std::optional<bool> at_surface;
		for (std::size_t i = 1; i < way.nodes.size(); ++i) {
			const auto &a = way.nodes[i - 1];
			const auto &b = way.nodes[i];
			const float ax = static_cast<float>(a.x), az = static_cast<float>(a.z);
			const float dx = static_cast<float>(b.x - a.x),
						dz = static_cast<float>(b.z - a.z);
			const int n = std::max(1,
					static_cast<int>(std::ceil(std::hypot(dx, dz) / ROAD_SAMPLE_STEP)));
			std::optional<std::uint32_t> segment_id;
			for (int s = 0; s <= n; ++s) {
				const float t = static_cast<float>(s) / n;
				const float x = ax + dx * t, z = az + dz * t;
				const auto lo = road_bucket(static_cast<int>(std::floor(x - margin)),
						static_cast<int>(std::floor(z - margin)));
				const auto hi = road_bucket(static_cast<int>(std::floor(x + margin)),
						static_cast<int>(std::floor(z + margin)));
				for (int bx = lo.first; bx <= hi.first; ++bx)
					for (int bz = lo.second; bz <= hi.second; ++bz) {
						const RoadBucket bucket{bx, bz};
						if (!wanted.contains(bucket))
							continue;
						if (!at_surface)
							at_surface = highways::tunnel_renders_at_surface(
									way, &editor, scale);
						if (!*at_surface)
							break;
						if (!segment_id) {
							if (dx == 0 && dz == 0)
								continue;
							index->segments.push_back(
									{ax, az, dx, dz, reach, carriageway});
							segment_id = static_cast<std::uint32_t>(
									index->segments.size() - 1);
						}
						auto &list = index->buckets[bucket];
						if (std::find(list.begin(), list.end(), *segment_id) ==
								list.end())
							list.push_back(*segment_id);
					}
				if (at_surface && !*at_surface)
					break;
			}
			if (at_surface && !*at_surface)
				break;
		}
	}
	for (auto &[bucket, segments] : index->buckets) {
		(void)bucket;
		segments.shrink_to_fit();
	}
	return RoadClearance(std::move(index));
}

bool RoadClearance::on_road(int x, int z) const
{
	return impl_ && impl_->roads && impl_->roads->contains(x, z);
}

RoadClearance::Placement RoadClearance::place(int x, int z,
		std::pair<float, float> direction, bool crosses_roads,
		const CoordinateBitmap &buildings) const
{
	if (!on_road(x, z))
		return {PlacementKind::Post, x, z};
	const auto *road = impl_->covering(x, z);
	if (!road)
		return {PlacementKind::Post, x, z};
	const float length = std::hypot(road->dx, road->dz);
	const float ux = road->dx / length, uz = road->dz / length;
	if (std::abs(direction.first * ux + direction.second * uz) < ROAD_ALONG_COS) {
		if (crosses_roads)
			return {PlacementKind::Post, x, z};
		return {road->carriageway ? PlacementKind::RoadGap : PlacementKind::PathGap, x,
				z};
	}
	const float side = ux * (static_cast<float>(z) - road->az) -
					   uz * (static_cast<float>(x) - road->ax);
	if (std::abs(side) < .5f)
		return {PlacementKind::Post, x, z};
	const float nx = -uz * (side > 0 ? 1.0f : -1.0f);
	const float nz = ux * (side > 0 ? 1.0f : -1.0f);
	std::optional<std::pair<int, int>> edge;
	int edge_step = 0;
	std::pair<int, int> last{x, z};
	for (int k = 1; k <= ROAD_MAX_SHIFT; ++k) {
		const auto c = std::pair{static_cast<int>(std::lround(x + nx * k)),
				static_cast<int>(std::lround(z + nz * k))};
		if (buildings.contains(c.first, c.second))
			break;
		if (!on_road(c.first, c.second))
			return {PlacementKind::Post, c.first, c.second};
		last = c;
		if (!edge && road->chebyshev(c.first, c.second) > road->reach) {
			edge = c;
			edge_step = k;
		} else if (edge && k >= edge_step + ROAD_EXTRA_SHIFT) {
			break;
		}
	}
	const auto result = edge.value_or(last);
	return {PlacementKind::Post, result.first, result.second};
}

namespace
{
int rust_round_i32(double value)
{
	// Rust's `f64::round() as i32` rounds ties away from zero, converts NaN to
	// zero, and saturates infinities and out-of-range values. Keep malformed or
	// extreme OSM measurements from invoking undefined C++ float-to-int casts.
	if (std::isnan(value))
		return 0;
	if (value >= static_cast<double>(std::numeric_limits<int>::max()))
		return std::numeric_limits<int>::max();
	if (value <= static_cast<double>(std::numeric_limits<int>::min()))
		return std::numeric_limits<int>::min();
	return static_cast<int>(std::round(value));
}

std::optional<double> parse_min_height(const std::string &raw)
{
	std::string value = raw;
	const auto last = value.find_last_not_of(" \t\r\n");
	if (last == std::string::npos)
		return std::nullopt;
	value.erase(last + 1);
	while (!value.empty() && value.back() == 'm')
		value.pop_back();
	const auto first = value.find_first_not_of(" \t\r\n");
	if (first == std::string::npos)
		return std::nullopt;
	value.erase(0, first);
	try {
		std::size_t used = 0;
		const double height = std::stod(value, &used);
		if (used != value.size())
			return std::nullopt;
		return height;
	} catch (...) {
		return std::nullopt;
	}
}

std::vector<std::optional<int>> barrier_bases(world_editor::WorldEditor &editor,
		const bridges::BridgeSurfaceMap &surface,
		const std::vector<std::pair<int, int>> &cells)
{
	const auto ground = [&editor](const std::pair<int, int> &cell) {
		return editor.get_absolute_y(cell.first, 0, cell.second);
	};
	const auto deck = [&surface](const std::pair<int, int> &cell) {
		return surface.nearby_deck_y(
				cell.first, cell.second, BRIDGE_BARRIER_NEARBY_RADIUS);
	};
	std::vector<std::optional<int>> bases;
	bases.reserve(cells.size());
	for (const auto &cell : cells)
		bases.emplace_back(ground(cell));

	std::size_t i = 0;
	while (i < cells.size()) {
		if (!deck(cells[i])) {
			++i;
			continue;
		}
		const auto start = i;
		while (i < cells.size() && deck(cells[i]))
			++i;
		const auto at_grade = [&](std::size_t inside, std::size_t outside) {
			const auto deck_y = deck(cells[inside]);
			return deck_y && *deck_y - ground(cells[outside]) <= 2;
		};
		const bool on_bridge = (start == 0 && i == cells.size()) ||
							   (start > 0 && at_grade(start, start - 1)) ||
							   (i < cells.size() && at_grade(i - 1, i));
		if (!on_bridge)
			continue;
		for (std::size_t n = start; n < i; ++n) {
			const auto [x, z] = cells[n];
			bases[n] = surface.supported_top(editor, x, z, BRIDGE_BARRIER_NEARBY_RADIUS);
			if (!bases[n]) {
				const int y = ground(cells[n]);
				const auto deck_y = deck(cells[n]);
				if (deck_y && *deck_y - y <= 1)
					bases[n] = y;
			}
		}
	}
	return bases;
}
} // namespace

static void generate_barriers_legacy(world_editor::WorldEditor &editor,
		osm_parser::ProcessedElement const &element,
		const bridges::BridgeSurfaceMap &bridge_surface)
{
	block_definitions::Block barrier_material = block_definitions::COBBLESTONE_WALL;
	int barrier_height = 2;

	{
		const std::unordered_map<std::string, std::string> &tags = element.tags();
		auto it = tags.find("barrier");
		if (it != tags.end()) {
			const std::string &val = it->second;
			if (val == "bollard") {
				barrier_material = block_definitions::STREETS_BOLLARD;
				barrier_height = 1;
			} else if (val == "guard_rail" || val == "guardrail") {
				barrier_material = block_definitions::STREETS_GUARDRAIL;
				barrier_height = 1;
			} else if (val == "kerb") {
				return;
			} else if (val == "hedge") {
				barrier_material = block_definitions::OAK_LEAVES;
				barrier_height = 2;
			} else if (val == "fence") {
				auto fit = tags.find("fence_type");
				if (fit != tags.end()) {
					const std::string &f = fit->second;
					if (f == "railing" || f == "bars" || f == "krest") {
						barrier_material = block_definitions::EARTH_FENCE_WROUGHT;
						barrier_height = 1;
					} else if (f == "barbed_wire" || f == "electric") {
						barrier_material = block_definitions::EARTH_FENCE_BARBED;
						barrier_height = 2;
					} else if (f == "chain_link" || f == "metal" || f == "wire" ||
							   f == "corrugated_metal" || f == "electric" ||
							   f == "metal_bars") {
						barrier_material = block_definitions::EARTH_FENCE_CHAINLINK;
						barrier_height = 2;
					} else if (f == "slatted" || f == "paling") {
						barrier_material = block_definitions::EARTH_FENCE_PICKET;
						barrier_height = 1;
					} else if (f == "wood" || f == "split_rail" || f == "panel" ||
							   f == "pole") {
						barrier_material = block_definitions::OAK_FENCE;
						barrier_height = 2;
					} else if (f == "concrete" || f == "stone") {
						barrier_material = block_definitions::STONE_BRICK_WALL;
						barrier_height = 2;
					} else if (f == "glass") {
						barrier_material = block_definitions::GLASS;
						barrier_height = 1;
					}
				}
			} else if (val == "wall") {
				barrier_material = block_definitions::STONE_BRICK_WALL;
				barrier_height = 3;
			}
		}
	}

	{
		const std::unordered_map<std::string, std::string> &tags = element.tags();
		auto mit = tags.find("material");
		if (mit != tags.end()) {
			const std::string &mat = mit->second;
			if (mat == "brick") {
				barrier_material = block_definitions::BRICK;
			}
			if (mat == "concrete") {
				barrier_material = block_definitions::LIGHT_GRAY_CONCRETE;
			}
			if (mat == "metal") {
				barrier_material = block_definitions::STONE_BRICK_WALL;
			}
			if (mat == "wood") {
				barrier_material = block_definitions::OAK_FENCE;
			}
		}
	}

	std::optional<osm_parser::Way> maybe_way = element.as_way();
	if (!maybe_way.has_value()) {
		return;
	}
	const osm_parser::Way &way = maybe_way.value();

	// Mapped barrier parts can be suspended above the ground (for example,
	// railings attached to a bridge tower). Scale both their lift and tagged
	// height with the world, but keep ordinary ground barriers at their legacy
	// one-block-per-metre height.
	const double scale = editor.scale();
	int lift = 0;
	if (const auto it = element.tags().find("min_height"); it != element.tags().end()) {
		if (const auto metres = parse_min_height(it->second)) {
			const double rounded = std::round(*metres * scale);
			lift = rust_round_i32(std::isnan(rounded) ? 0.0 : std::max(0.0, rounded));
		}
	}
	const double height_scale = lift > 0 ? scale : 1.0;
	int wall_height = barrier_height;
	{
		const std::unordered_map<std::string, std::string> &tags = element.tags();
		auto hit = tags.find("height");
		if (hit != tags.end()) {
			try {
				std::size_t used = 0;
				const double h = std::stod(hit->second, &used);
				if (used == hit->second.size()) {
					const auto height =
							static_cast<std::int64_t>(rust_round_i32(h * height_scale));
					wall_height = static_cast<int>(std::clamp<std::int64_t>(height - lift,
							std::numeric_limits<int>::min(),
							std::numeric_limits<int>::max()));
				}
			} catch (...) {
				wall_height = barrier_height;
			}
		}
		const int minimum_height = lift > 0 ? 1 : 2;
		if (wall_height < minimum_height) {
			wall_height = minimum_height;
		}
	}
	const bool joined = connected_blocks::is_connectable(editor, barrier_material);
	const bool capped = wall_height > 1 &&
						barrier_material != block_definitions::OAK_LEAVES &&
						barrier_material != block_definitions::OAK_FENCE;
	const bool hedge = barrier_material == block_definitions::OAK_LEAVES;
	std::vector<std::pair<int, int>> cells;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &a = way.nodes[i - 1];
		const auto &b = way.nodes[i];
		for (const auto &point : bresenham::bresenham_line(a.x, 0, a.z, b.x, 0, b.z)) {
			const std::pair<int, int> cell{std::get<0>(point), std::get<2>(point)};
			if (cells.empty() || cells.back() != cell)
				cells.push_back(cell);
		}
	}
	if (joined)
		cells = connected_blocks::four_connected_line(cells);
	auto bases = barrier_bases(editor, bridge_surface, cells);
	if (lift > 0 && !cells.empty()) {
		int lowest = std::numeric_limits<int>::max();
		for (const auto &[x, z] : cells)
			lowest = std::min(lowest, editor.get_absolute_y(x, 0, z));
		bases.assign(cells.size(), std::optional<int>(lowest));
	}
	for (std::size_t i = 0; i < cells.size(); ++i) {
		if (!bases[i])
			continue;
		const auto [x, z] = cells[i];
		const int base = *bases[i];
		if (hedge && !editor.surface_is_sealed(x, z) &&
				base == editor.get_absolute_y(x, 0, z))
			editor.set_block_absolute(block_definitions::GRASS_BLOCK, x, base, z,
					std::nullopt, std::nullopt);
		if (hedge)
			editor.register_hedge(x, z);
		for (int y = lift + 1; y <= lift + wall_height; ++y) {
			if (joined)
				connected_blocks::place_connected(
						editor, barrier_material, x, base + y, z);
			else
				editor.set_block_absolute(
						barrier_material, x, base + y, z, std::nullopt, std::nullopt);
		}
		if (capped)
			editor.set_block_absolute(block_definitions::STONE_BRICK_SLAB, x,
					base + lift + wall_height + 1, z, std::nullopt, std::nullopt);
	}
}

namespace
{
enum class BarrierNodeRole
{
	Gate,
	Opening
};
enum class BarrierSpotKind
{
	Post,
	Gate,
	PathGap,
	RoadGap,
	Opening
};
struct BarrierSpot
{
	BarrierSpotKind kind;
	int x;
	int z;
};
struct BarrierPost
{
	int x;
	int z;
	std::optional<bool> gate;
};
struct BarrierMark
{
	std::size_t at;
	BarrierNodeRole role;
	std::size_t width;
};
struct BarrierTrace
{
	std::vector<std::pair<int, int>> cells;
	std::vector<std::pair<float, float>> directions;
	std::vector<BarrierMark> marks;
};
struct BarrierStyle
{
	Block material{block_definitions::COBBLESTONE_WALL};
	int height{2};
	bool crosses_roads{false};
	bool spaced{false};
};

BarrierStyle barrier_style(const tags_t &tags)
{
	BarrierStyle style;
	if (!tags.contains("barrier")) {
		style.height = 0;
		return style;
	}
	const auto type = tags.get("barrier");
	auto set = [&](const Block &block, int height) {
		style.material = block;
		style.height = height;
	};
	if (type == "bollard") {
		set(block_definitions::COBBLESTONE_WALL, 1);
		style.crosses_roads = true;
		style.spaced = true;
	} else if (type == "kerb" || type == "ditch") {
		style.height = 0;
	} else if (type == "hedge") {
		set(block_definitions::OAK_LEAVES, 2);
	} else if (type == "fence") {
		const auto fence_type = tags.get("fence_type");
		if (fence_type == "railing" || fence_type == "bars" || fence_type == "krest")
			set(block_definitions::STONE_BRICK_WALL, 1);
		else if (fence_type == "chain_link" || fence_type == "metal" ||
				 fence_type == "wire" || fence_type == "barbed_wire" ||
				 fence_type == "corrugated_metal" || fence_type == "electric" ||
				 fence_type == "metal_bars")
			set(block_definitions::STONE_BRICK_WALL, 2);
		else if (fence_type == "slatted" || fence_type == "paling")
			set(block_definitions::OAK_FENCE, 1);
		else if (fence_type == "wood" || fence_type == "split_rail" ||
				 fence_type == "panel" || fence_type == "pole")
			set(block_definitions::OAK_FENCE, 2);
		else if (fence_type == "concrete" || fence_type == "stone")
			set(block_definitions::STONE_BRICK_WALL, 2);
		else if (fence_type == "glass")
			set(block_definitions::GLASS, 1);
	} else if (type == "wall" || type == "city_wall") {
		set(block_definitions::STONE_BRICK_WALL, 3);
	} else if (type == "retaining_wall" || type == "guard_rail" || type == "handrail" ||
			   type == "cable_barrier") {
		set(block_definitions::STONE_BRICK_WALL, 1);
	} else if (type == "jersey_barrier") {
		set(block_definitions::LIGHT_GRAY_CONCRETE, 1);
		style.crosses_roads = true;
	} else if (type == "block") {
		set(block_definitions::STONE, 1);
		style.crosses_roads = true;
	}
	const auto material = tags.get("material");
	if (material == "brick")
		style.material = block_definitions::BRICK;
	else if (material == "concrete")
		style.material = block_definitions::LIGHT_GRAY_CONCRETE;
	else if (material == "metal")
		style.material = block_definitions::STONE_BRICK_WALL;
	else if (material == "wood")
		style.material = block_definitions::OAK_FENCE;
	return style;
}

std::optional<std::pair<BarrierNodeRole, std::size_t>> barrier_node_role(
		const ProcessedNode &node, const ProcessedWay &way, double scale)
{
	const auto type = node.tags.get("barrier");
	BarrierNodeRole role;
	std::size_t default_width = 1;
	if (type == "gate" || type == "swing_gate" || type == "sliding_gate" ||
			type == "wicket_gate" || type == "hampshire_gate" || type == "kissing_gate" ||
			type == "bump_gate") {
		role = BarrierNodeRole::Gate;
	} else if (type == "entrance" || type == "stile" || type == "horse_stile" ||
			   type == "turnstile" || type == "full-height_turnstile") {
		role = BarrierNodeRole::Opening;
	} else if (type == "lift_gate" || type == "cycle_barrier" ||
			   type == "motorcycle_barrier" || type == "cattle_grid" ||
			   type == "border_control" || type == "toll_booth" || type == "sally_port" ||
			   type == "bollard" || type == "chain" || type == "rope" ||
			   type == "height_restrictor") {
		role = BarrierNodeRole::Opening;
		default_width = 3;
	} else if (type.empty() && node.tags.contains("entrance")) {
		role = BarrierNodeRole::Opening;
	} else {
		return std::nullopt;
	}
	std::size_t width = default_width;
	if (const auto metres = highways::parse_width_tag_m(node.tags))
		width = static_cast<std::size_t>(
				std::clamp(std::round(*metres * scale), 1.0, 5.0));
	(void)way;
	return std::pair{role, width};
}

BarrierTrace trace_barrier(const ProcessedWay &way, double scale, bool with_marks)
{
	BarrierTrace trace;
	auto mark_node = [&](const ProcessedNode &node, std::size_t at) {
		if (!with_marks || node.tags.get("barrier") == way.tags.get("barrier"))
			return;
		if (const auto role = barrier_node_role(node, way, scale))
			trace.marks.push_back({at, role->first, role->second});
	};
	const auto segments = way.nodes.size() > 0 ? way.nodes.size() - 1 : 0;
	for (std::size_t i = 0; i < segments; ++i) {
		const auto &a = way.nodes[i];
		const auto &b = way.nodes[i + 1];
		const float dx = static_cast<float>(b.x - a.x);
		const float dz = static_cast<float>(b.z - a.z);
		const float length = std::hypot(dx, dz);
		const std::pair<float, float> direction =
				length > 0 ? std::pair{dx / length, dz / length} : std::pair{1.0f, 0.0f};
		const auto points = bresenham::bresenham_line(a.x, 0, a.z, b.x, 0, b.z);
		for (std::size_t k = 0; k < points.size(); ++k) {
			const auto &[x, y, z] = points[k];
			(void)y;
			if (trace.cells.empty() || trace.cells.back() != std::pair{x, z}) {
				trace.cells.emplace_back(x, z);
				trace.directions.push_back(direction);
			}
			if (k == 0)
				mark_node(a, trace.cells.size() - 1);
		}
		if (i + 1 == segments && !trace.cells.empty())
			mark_node(b, trace.cells.size() - 1);
	}
	return trace;
}

std::pair<std::size_t, std::size_t> barrier_span(
		std::size_t at, std::size_t width, std::size_t length)
{
	return {at > (width - 1) / 2 ? at - (width - 1) / 2 : 0,
			std::min(at + width / 2, length - 1)};
}

void orient_barrier_gates(std::vector<BarrierPost> &run)
{
	for (std::size_t i = 0; i < run.size(); ++i) {
		if (!run[i].gate)
			continue;
		const int x = run[i].x, z = run[i].z;
		bool along_x = false, along_z = false;
		if (i > 0) {
			along_x |= run[i - 1].z == z;
			along_z |= run[i - 1].x == x;
		}
		if (i + 1 < run.size()) {
			along_x |= run[i + 1].z == z;
			along_z |= run[i + 1].x == x;
		}
		if (along_x != along_z)
			run[i].gate = along_x;
	}
}

std::vector<std::vector<BarrierPost>> lay_out_barrier(const BarrierTrace &trace,
		std::vector<BarrierSpot> spots, bool spaced, bool joined,
		const std::function<unsigned char(int, int)> &cost)
{
	constexpr std::size_t max_gate_span = 5;
	for (const auto &mark : trace.marks) {
		if (mark.at >= spots.size())
			continue;
		auto &spot = spots[mark.at];
		if (mark.role == BarrierNodeRole::Gate && spot.kind == BarrierSpotKind::PathGap) {
			std::size_t lo = mark.at, hi = mark.at;
			while (lo > 0 && spots[lo - 1].kind == BarrierSpotKind::PathGap)
				--lo;
			while (hi + 1 < spots.size() &&
					spots[hi + 1].kind == BarrierSpotKind::PathGap)
				++hi;
			if (hi - lo + 1 <= max_gate_span)
				for (std::size_t i = lo; i <= hi; ++i)
					spots[i].kind = BarrierSpotKind::Gate;
		} else if (mark.role == BarrierNodeRole::Gate &&
				   spot.kind == BarrierSpotKind::Post) {
			const auto [lo, hi] = barrier_span(mark.at, mark.width, spots.size());
			for (std::size_t i = lo; i <= hi; ++i)
				if (spots[i].kind == BarrierSpotKind::Post)
					spots[i].kind = BarrierSpotKind::Gate;
		} else if (mark.role == BarrierNodeRole::Opening) {
			const auto [lo, hi] = barrier_span(mark.at, mark.width, spots.size());
			for (std::size_t i = lo; i <= hi; ++i)
				if (spots[i].kind == BarrierSpotKind::Post ||
						spots[i].kind == BarrierSpotKind::Gate)
					spots[i].kind = BarrierSpotKind::Opening;
		}
	}
	std::vector<std::vector<BarrierPost>> runs;
	std::vector<BarrierPost> run;
	for (std::size_t i = 0; i < spots.size(); ++i) {
		auto &spot = spots[i];
		if (spot.kind != BarrierSpotKind::Post && spot.kind != BarrierSpotKind::Gate) {
			if (!run.empty())
				runs.push_back(std::exchange(run, {}));
			continue;
		}
		const auto gate =
				spot.kind == BarrierSpotKind::Gate
						? std::optional<bool>(std::abs(trace.directions[i].first) >=
											  std::abs(trace.directions[i].second))
						: std::nullopt;
		if (!run.empty()) {
			auto &previous = run.back();
			if (previous.x == spot.x && previous.z == spot.z) {
				if (!previous.gate)
					previous.gate = gate;
				continue;
			}
			const int dx = spot.x - previous.x, dz = spot.z - previous.z;
			if (std::max(std::abs(dx), std::abs(dz)) > 1) {
				const auto line3 = bresenham::bresenham_line(
						previous.x, 0, previous.z, spot.x, 0, spot.z);
				std::vector<std::pair<int, int>> line;
				for (const auto &[x, y, z] : line3) {
					(void)y;
					line.emplace_back(x, z);
				}
				if (joined)
					line = connected_blocks::four_connected_line(line);
				for (std::size_t j = 1; j + 1 < line.size(); ++j)
					if (cost(line[j].first, line[j].second) == 0)
						run.push_back({line[j].first, line[j].second, std::nullopt});
			} else if (joined && dx != 0 && dz != 0) {
				const auto c1 = std::pair{spot.x, previous.z};
				const auto c2 = std::pair{previous.x, spot.z};
				const auto corner =
						cost(c1.first, c1.second) <= cost(c2.first, c2.second) ? c1 : c2;
				// Preserve the wall's corner even when both candidates are costly;
				// cost selects the clearer side, rather than deleting the corner.
				run.push_back({corner.first, corner.second,
						previous.gate ? previous.gate : gate});
			}
		}
		run.push_back({spot.x, spot.z, gate});
	}
	if (!run.empty())
		runs.push_back(std::move(run));
	for (auto &r : runs) {
		orient_barrier_gates(r);
		if (spaced) {
			std::vector<BarrierPost> every_other;
			// Match Rust's retain counter: increment first, then keep odd indexes.
			for (std::size_t i = 1; i < r.size(); i += 2)
				every_other.push_back(r[i]);
			r = std::move(every_other);
		}
	}
	return runs;
}

void place_barrier_gate(WorldEditor &editor, int x, int y, int z, bool along_x)
{
	connected_blocks::place_gate(
			editor, block_definitions::OAK_FENCE_GATE, x, y, z, along_x);
}
} // namespace

void generate_barriers(world_editor::WorldEditor &editor, const ProcessedElement &element,
		const bridges::BridgeSurfaceMap &bridge_surface, const RoadClearance &clearance,
		const CoordinateBitmap &building_footprints)
{
	if (!element.is_way())
		return;
	const auto style = barrier_style(element.tags());
	if (style.height == 0)
		return;
	const auto &way = element.as_way();
	const double scale = editor.scale();
	int lift = 0;
	if (const auto it = element.tags().find("min_height"); it != element.tags().end())
		if (const auto metres = parse_min_height(it->second))
			lift = rust_round_i32(std::max(0.0, *metres * scale));
	const double height_scale = lift > 0 ? scale : 1.0;
	int height = style.height;
	if (const auto it = element.tags().find("height"); it != element.tags().end()) {
		try {
			std::size_t used = 0;
			const double metres = std::stod(it->second, &used);
			if (used == it->second.size())
				height = std::max(1, rust_round_i32(metres * height_scale) - lift);
		} catch (...) {
		}
	}
	const bool hedge = style.material == block_definitions::OAK_LEAVES;
	const bool joined =
			connected_blocks::is_connectable(editor, style.material) && !style.spaced;
	const bool capped =
			height > 1 && !hedge && style.material != block_definitions::OAK_FENCE;
	const bool grounded = lift == 0;
	const auto trace = trace_barrier(way, scale, grounded);
	std::vector<bool> on_deck(trace.cells.size(), false);
	if (grounded && !bridge_surface.is_empty()) {
		const auto ground = [&](const std::pair<int, int> &p) {
			return editor.get_absolute_y(p.first, 0, p.second);
		};
		const auto deck = [&](const std::pair<int, int> &p) {
			return bridge_surface.nearby_deck_y(
					p.first, p.second, BRIDGE_BARRIER_NEARBY_RADIUS);
		};
		std::size_t i = 0;
		while (i < trace.cells.size()) {
			if (!deck(trace.cells[i])) {
				++i;
				continue;
			}
			const auto begin = i;
			while (i < trace.cells.size() && deck(trace.cells[i]))
				++i;
			const auto at_grade = [&](std::size_t inside, std::size_t outside) {
				const auto y = deck(trace.cells[inside]);
				return y && *y - ground(trace.cells[outside]) <= 2;
			};
			if ((begin == 0 && i == trace.cells.size()) ||
					(begin > 0 && at_grade(begin, begin - 1)) ||
					(i < trace.cells.size() && at_grade(i - 1, i)))
				std::fill(on_deck.begin() + begin, on_deck.begin() + i, true);
		}
	}
	std::vector<BarrierSpot> spots;
	spots.reserve(trace.cells.size());
	for (std::size_t i = 0; i < trace.cells.size(); ++i) {
		const auto [x, z] = trace.cells[i];
		if (!grounded || on_deck[i]) {
			spots.push_back({BarrierSpotKind::Post, x, z});
			continue;
		}
		const auto placement = clearance.place(
				x, z, trace.directions[i], style.crosses_roads, building_footprints);
		const auto kind = placement.kind == RoadClearance::PlacementKind::Post
								  ? BarrierSpotKind::Post
						  : placement.kind == RoadClearance::PlacementKind::PathGap
								  ? BarrierSpotKind::PathGap
								  : BarrierSpotKind::RoadGap;
		spots.push_back({kind, placement.x, placement.z});
	}
	const auto cost = [&](int x, int z) -> unsigned char {
		if (grounded && building_footprints.contains(x, z))
			return 2;
		if (grounded && !style.crosses_roads && clearance.on_road(x, z))
			return 1;
		return 0;
	};
	const auto runs =
			lay_out_barrier(trace, std::move(spots), style.spaced, joined, cost);
	const bool floating = lift > 0;
	for (const auto &run : runs) {
		std::vector<std::optional<int>> bases;
		if (floating) {
			int lowest = std::numeric_limits<int>::max();
			for (const auto &post : run)
				lowest = std::min(lowest, editor.get_absolute_y(post.x, 0, post.z));
			bases.assign(run.size(), lowest);
		} else {
			std::vector<std::pair<int, int>> cells;
			for (const auto &post : run)
				cells.emplace_back(post.x, post.z);
			bases = barrier_bases(editor, bridge_surface, cells);
		}
		for (std::size_t i = 0; i < run.size(); ++i) {
			if (!bases[i] || run[i].gate)
				continue;
			const int x = run[i].x, z = run[i].z, base = *bases[i];
			if (hedge && !editor.surface_is_sealed(x, z) &&
					base == editor.get_absolute_y(x, 0, z))
				editor.set_block_absolute(block_definitions::GRASS_BLOCK, x, base, z);
			if (hedge)
				editor.register_hedge(x, z);
			for (int y = lift + 1; y <= lift + height; ++y) {
				if (joined)
					connected_blocks::place_connected(
							editor, style.material, x, base + y, z);
				else
					editor.set_block_absolute(style.material, x, base + y, z);
			}
			if (capped)
				editor.set_block_absolute(block_definitions::STONE_BRICK_SLAB, x,
						base + lift + height + 1, z);
		}
		for (std::size_t i = 0; i < run.size(); ++i)
			if (bases[i] && run[i].gate)
				place_barrier_gate(
						editor, run[i].x, *bases[i] + lift + 1, run[i].z, *run[i].gate);
	}
}

void generate_barriers(world_editor::WorldEditor &editor, const ProcessedElement &element)
{
	static const bridges::BridgeSurfaceMap empty_bridge_surface;
	static const RoadClearance empty_clearance;
	static const CoordinateBitmap empty_buildings;
	generate_barriers(
			editor, element, empty_bridge_surface, empty_clearance, empty_buildings);
}

void generate_barriers(world_editor::WorldEditor &editor, const ProcessedElement &element,
		const bridges::BridgeSurfaceMap &bridge_surface)
{
	static const RoadClearance empty_clearance;
	static const CoordinateBitmap empty_buildings;
	generate_barriers(editor, element, bridge_surface, empty_clearance, empty_buildings);
}

void place_barrier_node_block(world_editor::WorldEditor &editor,
		osm_parser::ProcessedNode const &node,
		const bridges::BridgeSurfaceMap &bridge_surface, block_definitions::Block block,
		int dy)
{
	const auto deck_y = bridge_surface.contains(node.x, node.z)
								? bridge_surface.supported_top(editor, node.x, node.z,
										  BRIDGE_BARRIER_NEARBY_RADIUS)
								: std::nullopt;
	if (deck_y) {
		editor.set_block_absolute(block, node.x, *deck_y + dy, node.z,
				std::optional<std::vector<block_definitions::Block>>(),
				std::optional<std::vector<block_definitions::Block>>());
	} else {
		editor.set_block(block, node.x, dy, node.z,
				std::optional<std::vector<block_definitions::Block>>(), std::nullopt);
	}
}

void generate_barrier_nodes(world_editor::WorldEditor &editor,
		osm_parser::ProcessedNode const &node,
		const bridges::BridgeSurfaceMap &bridge_surface)
{
	auto it = node.tags.find("barrier");
	if (it == node.tags.end()) {
		return;
	}
	const std::string &val = it->second;
	if (val == "bollard") {
		place_barrier_node_block(
				editor, node, bridge_surface, block_definitions::COBBLESTONE_WALL, 1);
	} else if (val == "stile" || val == "gate" || val == "swing_gate" ||
			   val == "lift_gate") {
		/* Future implementation for gates:
        editor.set_block(
            block_definitions::OAK_TRAPDOOR,
            node.x,
            1,
            node.z,
            std::optional<std::vector<block_definitions::Block>>(
                std::vector<block_definitions::Block>{
                    block_definitions::COBBLESTONE_WALL,
                    block_definitions::OAK_FENCE,
                    block_definitions::STONE_BRICK_WALL,
                    block_definitions::OAK_LEAVES,
                    block_definitions::STONE_BRICK_SLAB
                }
            ),
            std::nullopt
        );
        editor.set_block(
            block_definitions::AIR,
            node.x,
            2,
            node.z,
            std::optional<std::vector<block_definitions::Block>>(
                std::vector<block_definitions::Block>{
                    block_definitions::COBBLESTONE_WALL,
                    block_definitions::OAK_FENCE,
                    block_definitions::STONE_BRICK_WALL,
                    block_definitions::OAK_LEAVES,
                    block_definitions::STONE_BRICK_SLAB
                }
            ),
            std::nullopt
        );
        editor.set_block(
            block_definitions::AIR,
            node.x,
            3,
            node.z,
            std::optional<std::vector<block_definitions::Block>>(
                std::vector<block_definitions::Block>{
                    block_definitions::COBBLESTONE_WALL,
                    block_definitions::OAK_FENCE,
                    block_definitions::STONE_BRICK_WALL,
                    block_definitions::OAK_LEAVES,
                    block_definitions::STONE_BRICK_SLAB
                }
            ),
            std::nullopt
        );
        */
	} else if (val == "block") {
		place_barrier_node_block(
				editor, node, bridge_surface, block_definitions::STONE, 1);
	} else if (val == "entrance") {
		place_barrier_node_block(editor, node, bridge_surface, block_definitions::AIR, 1);
	}
}

void generate_barrier_nodes(
		world_editor::WorldEditor &editor, osm_parser::ProcessedNode const &node)
{
	static const bridges::BridgeSurfaceMap empty_bridge_surface;
	generate_barrier_nodes(editor, node, empty_bridge_surface);
}

}
}
