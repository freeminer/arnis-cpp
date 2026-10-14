#include "plane.h"

#include "../assets_root.h"
#include "../deterministic_rng.h"
#include "../models_3d/pipeline.h"
#include "schem_decoder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace arnis::structures::plane
{
namespace
{
std::shared_ptr<const SchemDocument> plane_asset(
		const WorldEditor &editor, bool gear_down, unsigned livery)
{
	if (livery < 1 || livery > 6)
		return {};
	const auto root = editor.get_schematic_asset_root().empty()
							  ? assets::path("structures")
							  : editor.get_schematic_asset_root();
	const auto path = root / "planes" /
					  (std::string("plane_gear_") + (gear_down ? "down_" : "up_") +
							  std::to_string(livery) + ".schem");
	try {
		return load_schem_cached(path);
	} catch (...) {
		return {};
	}
}

std::vector<std::shared_ptr<const SchemDocument>> available_plane_assets(
		const WorldEditor &editor, bool gear_down)
{
	std::vector<std::shared_ptr<const SchemDocument>> assets;
	for (unsigned i = 1; i <= 6; ++i)
		if (auto asset = plane_asset(editor, gear_down, i))
			assets.push_back(std::move(asset));
	return assets;
}

double yaw_for_direction(double direction_x, double direction_z)
{
	// Schematic model nose points along local -Z. This is the inverse yaw
	// convention used by place_schem_document_yaw and Rust's yaw_for_dir.
	return std::atan2(direction_x, -direction_z) * 180.0 / M_PI;
}
} // namespace

static int region_floor(int coordinate)
{
	return coordinate >= 0 ? coordinate / 512 : -1 - ((-coordinate - 1) / 512);
}
bool collinear(double a, double b, double tolerance)
{
	const double d = std::abs(a - b);
	return std::min(d, M_PI - d) < tolerance;
}

bool extract_aeroway_segment(const ProcessedElement &element, Segment &out)
{
	if (!element.is_way())
		return false;
	const auto &way = element.as_way();
	const auto aeroway = way.tags.get("aeroway");
	if (aeroway.empty() || (aeroway != "runway" && aeroway != "taxiway") ||
			way.tags.get("area") == std::optional<std::string>("yes") ||
			way.nodes.size() < 2 || way.nodes.front().id == way.nodes.back().id)
		return false;
	const double dx = double(way.nodes.back().x - way.nodes.front().x);
	const double dz = double(way.nodes.back().z - way.nodes.front().z);
	if (dx == 0.0 && dz == 0.0)
		return false;
	out.way_id = way.id;
	out.first_node = way.nodes.front().id;
	out.last_node = way.nodes.back().id;
	out.kind = aeroway == "runway" ? AerowayKind::Runway : AerowayKind::Taxiway;
	out.angle = std::fmod(std::atan2(dz, dx) + M_PI, M_PI);
	if (out.angle < 0)
		out.angle += M_PI;
	out.points.clear();
	for (const auto &node : way.nodes)
		out.points.emplace_back(double(node.x), double(node.z));
	return true;
}

bool extract_parking_stand(const ProcessedElement &element, Stand &out)
{
	if (!element.is_way())
		return false;
	const auto &way = element.as_way();
	if (way.tags.get("aeroway") != "parking_position" || way.tags.get("area") == "yes" ||
			way.nodes.size() < 2)
		return false;
	const auto &a = way.nodes[way.nodes.size() - 2];
	const auto &b = way.nodes.back();
	if (a.id == b.id)
		return false;
	const double dx = double(b.x - a.x), dz = double(b.z - a.z);
	const double length = std::hypot(dx, dz);
	if (length < 1.0)
		return false;
	out = {way.id, b.x, b.z, dx / length, dz / length};
	return true;
}

std::vector<Stand> collect_parking_stands(const std::vector<ProcessedElement> &elements)
{
	std::vector<Stand> stands;
	for (const auto &element : elements) {
		Stand stand;
		if (extract_parking_stand(element, stand))
			stands.push_back(stand);
	}
	return stands;
}

bool make_stand_placement(const Stand &stand, Placement &out)
{
	auto rng = element_rng(stand.way_id);
	if (!rng.random_bool(STAND_OCCUPANCY_PROBABILITY))
		return false;
	const double yaw = yaw_for_direction(stand.direction_x, stand.direction_z);
	// The mapped point is the nose-wheel position. The bundled plane's nose wheel
	// sits 6 blocks behind the nose tip, so its model centre is 14 blocks behind
	// the mapped point (half the 40-block hull minus that offset).
	const double back = PLANE_LENGTH_BLOCKS * 0.5 - NOSE_GEAR_OFFSET_BLOCKS;
	const int anchor_x =
			static_cast<int>(std::lround(stand.x - stand.direction_x * back));
	const int anchor_z =
			static_cast<int>(std::lround(stand.z - stand.direction_z * back));
	out = make_placement(
			stand.way_id, PlaneKind::Parked, anchor_x, anchor_z, yaw, 0.0, 0);
	return true;
}

std::vector<Segment> collect_aeroway_segments(
		const std::vector<ProcessedElement> &elements)
{
	std::vector<Segment> out;
	for (const auto &element : elements) {
		Segment segment;
		if (extract_aeroway_segment(element, segment))
			out.push_back(std::move(segment));
	}
	return out;
}

bool principal_geometry(const std::vector<std::pair<double, double>> &points,
		std::uint64_t rep_id, AerowayKind kind, AerowayStrip &out)
{
	if (points.size() < 2)
		return false;
	double cx = 0.0, cz = 0.0;
	for (const auto &[x, z] : points) {
		cx += x;
		cz += z;
	}
	cx /= points.size();
	cz /= points.size();
	double cxx = 0.0, cxz = 0.0, czz = 0.0;
	for (const auto &[x, z] : points) {
		const double dx = x - cx, dz = z - cz;
		cxx += dx * dx;
		cxz += dx * dz;
		czz += dz * dz;
	}
	const double theta = 0.5 * std::atan2(2.0 * cxz, cxx - czz);
	double s = std::sin(theta), c = std::cos(theta);
	auto extent = [&](double ax, double az) {
		double lo = HUGE_VAL, hi = -HUGE_VAL;
		for (const auto &[x, z] : points) {
			const double v = (x - cx) * ax + (z - cz) * az;
			lo = std::min(lo, v);
			hi = std::max(hi, v);
		}
		return std::pair{lo, hi};
	};
	auto along = extent(c, s), across = extent(-s, c);
	if (across.second - across.first > along.second - along.first) {
		const double previous_s = s;
		s = c;
		c = -previous_s;
		along = extent(c, s);
		across = extent(-s, c);
	}
	if (along.second <= along.first)
		return false;
	out = {rep_id, kind, cx, cz, c, s, along.second - along.first,
			across.second - across.first, along.first, along.second};
	return true;
}

bool eligible_strip(const AerowayStrip &strip, double scale)
{
	if (scale <= 0.0)
		return false;
	const double length_m = strip.length_blocks / scale;
	if (length_m > 8000.0)
		return false;
	if (strip.kind == AerowayKind::Runway)
		return strip.perpendicular_blocks / strip.length_blocks <= 0.5;
	return strip.perpendicular_blocks / strip.length_blocks <= 0.12;
}

Footprint plane_footprint(int anchor_x, int anchor_z, double yaw_degrees)
{
	(void)yaw_degrees;
	return {anchor_x - PLANE_REACH_BLOCKS, anchor_z - PLANE_REACH_BLOCKS,
			anchor_x + PLANE_REACH_BLOCKS, anchor_z + PLANE_REACH_BLOCKS};
}

Placement make_placement(std::uint64_t representative_id, PlaneKind kind, int anchor_x,
		int anchor_z, double yaw_degrees, double pitch_degrees, int elevation_blocks)
{
	Placement placement{representative_id, kind, anchor_x, anchor_z, yaw_degrees,
			pitch_degrees, elevation_blocks};
	placement.footprint = plane_footprint(anchor_x, anchor_z, yaw_degrees);
	return placement;
}

std::vector<std::pair<int, int>> deferred_region_keys(
		const std::vector<Placement> &placements)
{
	std::vector<std::pair<int, int>> keys;
	for (const auto &placement : placements)
		for (int rx = region_floor(placement.anchor_x - PLANE_REACH_BLOCKS);
				rx <= region_floor(placement.anchor_x + PLANE_REACH_BLOCKS); ++rx)
			for (int rz = region_floor(placement.anchor_z - PLANE_REACH_BLOCKS);
					rz <= region_floor(placement.anchor_z + PLANE_REACH_BLOCKS); ++rz)
				keys.emplace_back(rx, rz);
	std::sort(keys.begin(), keys.end());
	keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
	return keys;
}

void cap_placements(std::vector<Placement> &placements)
{
	if (placements.size() > MAX_PLANES)
		placements.resize(MAX_PLANES);
}

void sort_placements(std::vector<Placement> &placements)
{
	std::stable_sort(placements.begin(), placements.end(),
			[](const Placement &a, const Placement &b) {
				if (a.representative_id != b.representative_id)
					return a.representative_id < b.representative_id;
				if (a.anchor_x != b.anchor_x)
					return a.anchor_x < b.anchor_x;
				return a.anchor_z < b.anchor_z;
			});
}

bool placement_separated(
		const Placement &candidate, const std::vector<Placement> &accepted)
{
	if (candidate.kind != PlaneKind::Parked)
		return true;
	const double min_distance = MIN_PLANE_SEPARATION_BLOCKS * MIN_PLANE_SEPARATION_BLOCKS;
	for (const auto &other : accepted) {
		if (other.kind != PlaneKind::Parked)
			continue;
		const double dx = double(candidate.anchor_x - other.anchor_x);
		const double dz = double(candidate.anchor_z - other.anchor_z);
		if (dx * dx + dz * dz < min_distance)
			return false;
	}
	return true;
}

void thin_by_spacing(std::vector<Placement> &placements)
{
	const int cell = static_cast<int>(std::ceil(MIN_PLANE_SEPARATION_BLOCKS));
	const double min_sq = MIN_PLANE_SEPARATION_BLOCKS * MIN_PLANE_SEPARATION_BLOCKS;
	std::unordered_map<std::uint64_t, std::vector<std::pair<int, int>>> grid;
	std::vector<Placement> kept;
	kept.reserve(placements.size());
	auto floor_div = [cell](int value) {
		const int quotient = value / cell;
		return value % cell < 0 ? quotient - 1 : quotient;
	};
	for (const auto &candidate : placements) {
		if (candidate.kind == PlaneKind::Ascending) {
			kept.push_back(candidate);
			continue;
		}
		{
			const int cx = floor_div(candidate.anchor_x),
					  cz = floor_div(candidate.anchor_z);
			bool crowded = false;
			for (int dx = -1; dx <= 1 && !crowded; ++dx)
				for (int dz = -1; dz <= 1 && !crowded; ++dz) {
					const std::uint64_t key =
							(std::uint64_t(std::uint32_t(cx + dx)) << 32) |
							std::uint32_t(cz + dz);
					auto it = grid.find(key);
					if (it == grid.end())
						continue;
					for (const auto &[x, z] : it->second) {
						const double ddx = double(candidate.anchor_x - x);
						const double ddz = double(candidate.anchor_z - z);
						crowded |= ddx * ddx + ddz * ddz < min_sq;
					}
				}
			if (crowded)
				continue;
			grid[(std::uint64_t(std::uint32_t(cx)) << 32) | std::uint32_t(cz)]
					.emplace_back(candidate.anchor_x, candidate.anchor_z);
		}
		kept.push_back(candidate);
	}
	placements.swap(kept);
}

void finalize_placements(std::vector<Placement> &placements)
{
	thin_by_spacing(placements);
	cap_placements(placements);
}

void filter_and_finalize_strips(std::vector<AerowayStrip> &strips, double scale)
{
	strips.erase(std::remove_if(strips.begin(), strips.end(),
						 [scale](const AerowayStrip &strip) {
							 return !eligible_strip(strip, scale);
						 }),
			strips.end());
}

bool make_centerline_placement(const AerowayStrip &strip, double scale, Placement &out)
{
	if (!eligible_strip(strip, scale) ||
			strip.length_blocks / scale < PARKED_MIN_LENGTH_M)
		return false;
	const int x = static_cast<int>(std::lround(strip.centroid_x));
	const int z = static_cast<int>(std::lround(strip.centroid_z));
	const double yaw = yaw_for_direction(strip.direction_x, strip.direction_z);
	out = make_placement(strip.rep_id, PlaneKind::Parked, x, z, yaw, 0.0, 0);
	return true;
}

bool make_ascending_placement(const AerowayStrip &strip, double scale, Placement &out)
{
	if (strip.kind != AerowayKind::Runway || !eligible_strip(strip, scale) ||
			strip.length_blocks / scale < ASCENDING_MIN_LENGTH_M)
		return false;
	const double x = strip.centroid_x + strip.direction_x * strip.max_along;
	const double z = strip.centroid_z + strip.direction_z * strip.max_along;
	const double yaw = yaw_for_direction(strip.direction_x, strip.direction_z);
	out = make_placement(strip.rep_id, PlaneKind::Ascending,
			static_cast<int>(std::lround(x)), static_cast<int>(std::lround(z)), yaw,
			ASCENDING_PITCH_DEG,
			static_cast<int>(std::lround(PLANE_LENGTH_BLOCKS * 0.45 + 20.0 * scale)));
	return true;
}

std::vector<Placement> build_placements(const std::vector<AerowayStrip> &strips,
		const std::vector<Stand> &stands, double scale)
{
	std::vector<Placement> placements;
	for (const auto &strip : strips) {
		if (!eligible_strip(strip, scale))
			continue;
		const double length_m = strip.length_blocks / scale;
		const double yaw = yaw_for_direction(strip.direction_x, strip.direction_z);
		if (strip.kind == AerowayKind::Runway && length_m >= ASCENDING_MIN_LENGTH_M) {
			Placement ascending;
			if (make_ascending_placement(strip, scale, ascending))
				placements.push_back(std::move(ascending));
		}
		if (length_m < PARKED_MIN_LENGTH_M)
			continue;
		const double probability = strip.kind == AerowayKind::Runway
										   ? RUNWAY_PARK_PROBABILITY
										   : TAXIWAY_PARK_PROBABILITY;
		auto rng = element_rng(strip.rep_id);
		if (!rng.random_bool(probability))
			continue;
		const double lo = strip.min_along + PLANE_LENGTH_BLOCKS * 0.5;
		const double hi = strip.max_along - PLANE_LENGTH_BLOCKS * 0.5;
		if (hi <= lo)
			continue;
		const double along = rng.uniform_real(lo, hi);
		const int x = static_cast<int>(
				std::lround(strip.centroid_x + along * strip.direction_x));
		const int z = static_cast<int>(
				std::lround(strip.centroid_z + along * strip.direction_z));
		placements.push_back(
				make_placement(strip.rep_id, PlaneKind::Parked, x, z, yaw, 0.0, 0));
	}
	for (const auto &stand : stands) {
		Placement candidate;
		if (make_stand_placement(stand, candidate))
			placements.push_back(std::move(candidate));
	}
	finalize_placements(placements);
	return placements;
}

std::vector<Placement> prescan_placements(
		const std::vector<ProcessedElement> &elements, double scale)
{
	auto strips = collect_aeroway_strips(elements);
	filter_and_finalize_strips(strips, scale);
	return build_placements(strips, collect_parking_stands(elements), scale);
}

bool place_plane_placement(WorldEditor &editor, const Placement &placement)
{
	if (!editor.place_schematics())
		return false;
	const bool gear_down = placement.kind == PlaneKind::Parked;
	const auto fleet = available_plane_assets(editor, gear_down);
	if (fleet.empty())
		return false;
	auto rng = element_rng(placement.representative_id * 31 + 7);
	const auto livery = static_cast<std::size_t>(rng()) % fleet.size();
	const auto &document = fleet[livery];
	const int ground_y = models_3d::lowest_ground_in_bbox(editor,
			placement.footprint.min_x, placement.footprint.min_z,
			placement.footprint.max_x, placement.footprint.max_z);
	return place_schem_document_yaw(editor, *document, placement.anchor_x,
			ground_y + 1 + placement.elevation_blocks, placement.anchor_z,
			placement.yaw_degrees, placement.pitch_degrees);
}

std::size_t place_plane_placements(
		WorldEditor &editor, const std::vector<Placement> &placements)
{
	std::size_t placed = 0;
	for (const auto &placement : placements)
		if (place_plane_placement(editor, placement))
			++placed;
	return placed;
}

std::vector<AerowayStrip> merge_aeroways(const std::vector<Segment> &segments)
{
	std::vector<std::size_t> parent(segments.size());
	for (std::size_t i = 0; i < parent.size(); ++i)
		parent[i] = i;
	auto find_root = [&](std::size_t value) {
		std::size_t root = value;
		while (parent[root] != root)
			root = parent[root];
		while (parent[value] != value) {
			const auto next = parent[value];
			parent[value] = root;
			value = next;
		}
		return root;
	};
	auto unite = [&](std::size_t a, std::size_t b) {
		const auto ra = find_root(a), rb = find_root(b);
		if (ra != rb)
			parent[ra] = rb;
	};
	std::unordered_map<std::uint64_t, std::vector<std::size_t>> endpoint_map;
	for (std::size_t i = 0; i < segments.size(); ++i) {
		endpoint_map[segments[i].first_node].push_back(i);
		endpoint_map[segments[i].last_node].push_back(i);
	}
	for (const auto &[node, indices] : endpoint_map) {
		(void)node;
		for (std::size_t a = 0; a < indices.size(); ++a)
			for (std::size_t b = a + 1; b < indices.size(); ++b) {
				const auto ia = indices[a], ib = indices[b];
				if (segments[ia].kind == segments[ib].kind &&
						collinear(segments[ia].angle, segments[ib].angle))
					unite(ia, ib);
			}
	}
	std::vector<std::vector<std::size_t>> groups(segments.size());
	for (std::size_t i = 0; i < segments.size(); ++i)
		groups[find_root(i)].push_back(i);
	std::vector<AerowayStrip> result;
	for (const auto &group : groups) {
		if (group.empty())
			continue;
		std::vector<std::pair<double, double>> points;
		std::uint64_t representative = std::numeric_limits<std::uint64_t>::max();
		for (const auto i : group) {
			representative = std::min(representative, segments[i].way_id);
			points.insert(
					points.end(), segments[i].points.begin(), segments[i].points.end());
		}
		AerowayStrip strip;
		if (principal_geometry(
					points, representative, segments[group.front()].kind, strip))
			result.push_back(strip);
	}
	return result;
}

std::vector<AerowayStrip> collect_aeroway_strips(
		const std::vector<ProcessedElement> &elements)
{
	return merge_aeroways(collect_aeroway_segments(elements));
}
}
