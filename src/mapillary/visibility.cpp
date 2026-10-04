#include "visibility.h"

#include "imgops.h"
#include "rectify.h"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace arnis::mapillary
{
namespace
{
using XY = std::array<double, 2>;

template <typename F>
void for_each_edge(const std::vector<XY> &ring, F &&fn)
{
	for (std::size_t i = 0; i < ring.size(); ++i)
		fn(ring[i], ring[(i + 1) % ring.size()]);
}

bool ring_contains(const std::vector<XY> &ring, const XY &point)
{
	if (ring.size() < 3)
		return false;
	bool inside = false;
	for_each_edge(ring, [&](const XY &a, const XY &b) {
		if ((a[1] > point[1]) != (b[1] > point[1])) {
			const auto x = (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1]) + a[0];
			if (point[0] < x)
				inside = !inside;
		}
	});
	return inside;
}

double point_segment_distance(const XY &point, const XY &a, const XY &b)
{
	const XY ab{b[0] - a[0], b[1] - a[1]};
	const auto length_squared = ab[0] * ab[0] + ab[1] * ab[1];
	const auto t =
			length_squared < 1e-24
					? 0.0
					: std::clamp(((point[0] - a[0]) * ab[0] + (point[1] - a[1]) * ab[1]) /
										 length_squared,
							  0.0, 1.0);
	const XY nearest{a[0] + t * ab[0], a[1] + t * ab[1]};
	return std::hypot(point[0] - nearest[0], point[1] - nearest[1]);
}

std::array<double, 4> expanded_bbox(const std::array<double, 4> &box, double by)
{
	return {box[0] - by, box[1] - by, box[2] + by, box[3] + by};
}

bool bbox_meets_segment(const std::array<double, 4> &box, const XY &a, const XY &b)
{
	return std::min(a[0], b[0]) <= box[2] && std::max(a[0], b[0]) >= box[0] &&
		   std::min(a[1], b[1]) <= box[3] && std::max(a[1], b[1]) >= box[1];
}

bool segments_cross(const XY &p1, const XY &p2, const XY &q1, const XY &q2)
{
	const auto orient = [](const XY &a, const XY &b, const XY &c) {
		return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
	};
	const auto on = [](const XY &a, const XY &b, const XY &c) {
		return c[0] <= std::max(a[0], b[0]) && c[0] >= std::min(a[0], b[0]) &&
			   c[1] <= std::max(a[1], b[1]) && c[1] >= std::min(a[1], b[1]);
	};
	const auto d1 = orient(p1, p2, q1), d2 = orient(p1, p2, q2);
	const auto d3 = orient(q1, q2, p1), d4 = orient(q1, q2, p2);
	if (((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0)) && d1 != 0.0 &&
			d2 != 0.0 && d3 != 0.0 && d4 != 0.0)
		return true;
	return (d1 == 0.0 && on(p1, p2, q1)) || (d2 == 0.0 && on(p1, p2, q2)) ||
		   (d3 == 0.0 && on(q1, q2, p1)) || (d4 == 0.0 && on(q1, q2, p2));
}

std::vector<std::size_t> own_indices_geometric(
		const Wall &wall, const Footprints &footprints)
{
	const auto middle = wall.midpoint();
	const XY probe{middle[0] - 0.75 * wall.normal[0], middle[1] - 0.75 * wall.normal[1]};
	const std::array<double, 4> segment_box{std::min(wall.a[0], wall.b[0]) - 0.5,
			std::min(wall.a[1], wall.b[1]) - 0.5, std::max(wall.a[0], wall.b[0]) + 0.5,
			std::max(wall.a[1], wall.b[1]) + 0.5};
	std::vector<std::size_t> result;
	for (std::size_t i = 0; i < footprints.polys.size(); ++i) {
		const auto &poly = footprints.polys[i];
		if (segment_box[0] > poly.bbox[2] || segment_box[2] < poly.bbox[0] ||
				segment_box[1] > poly.bbox[3] || segment_box[3] < poly.bbox[1])
			continue;
		if (poly.contains(probe) || (poly.boundary_distance(wall.a) < 0.05 &&
											poly.boundary_distance(wall.b) < 0.05))
			result.push_back(i);
	}
	return result;
}

double chord_sag(const Wall &wall, const Footprints &footprints,
		const std::vector<std::size_t> &own)
{
	double sag = 0.0;
	for (const auto i : own) {
		if (footprints.polys[i].rings.empty())
			continue;
		for (const auto &point : footprints.polys[i].rings.front()) {
			const auto local = wall.sh_of({point[0], point[1], 0.0}, 0.0);
			if (local[0] >= -0.05 && local[0] <= wall.length + 0.05 && local[2] > 0.0 &&
					local[2] < 2.0)
				sag = std::max(sag, local[2]);
		}
	}
	return sag;
}

double z_base_of(const Camera &camera)
{
	return std::isfinite(camera.ground_z) ? camera.ground_z
										  : camera.centre[2] - camera.cam_height_m;
}

struct FovResult
{
	bool ok{};
	double fraction{};
	bool midpoint_on_image{};
	XY interval{};
};

FovResult perspective_fov(const Wall &wall, const Camera &camera, double z_base,
		double top_height, const Params &params)
{
	const auto margin = params.persp_margin;
	const pose::Projector projector(camera);
	std::array<bool, fov_grid[0]> on_columns{};
	std::size_t on_count = 0;
	bool midpoint_on_image = false;
	for (std::size_t column = 0; column < fov_grid[0]; ++column) {
		const auto s = wall.length * static_cast<double>(column) /
					   static_cast<double>(fov_grid[0] - 1);
		for (std::size_t row = 0; row < fov_grid[1]; ++row) {
			const auto height = top_height * static_cast<double>(row) /
								static_cast<double>(fov_grid[1] - 1);
			const auto projected =
					projector.project(wall.point(s, height, z_base, params.eps_m));
			const bool on = projected.valid() && projected.u >= margin &&
							projected.u <= 1.0 - margin && projected.v >= margin &&
							projected.v <= 1.0 - margin;
			if (on) {
				++on_count;
				on_columns[column] = true;
				if (column == fov_grid[0] / 2)
					midpoint_on_image = true;
			}
		}
	}
	double fraction = static_cast<double>(on_count) /
					  static_cast<double>(fov_grid[0] * fov_grid[1]);
	std::optional<std::size_t> first, last;
	for (std::size_t i = 0; i < on_columns.size(); ++i)
		if (on_columns[i]) {
			if (!first)
				first = i;
			last = i;
		}
	XY interval{};
	if (first && last) {
		const auto half = 0.5 * wall.length / static_cast<double>(fov_grid[0] - 1);
		const auto s_of = [&](std::size_t i) {
			return wall.length * static_cast<double>(i) /
				   static_cast<double>(fov_grid[0] - 1);
		};
		interval = {std::max(0.0, s_of(*first) - half),
				std::min(wall.length, s_of(*last) + half)};
	}
	if (params.persp_landscape_only && camera.height > camera.width) {
		fraction = 0.0;
		midpoint_on_image = false;
	}
	return {fraction >= params.fov_min_on_image && midpoint_on_image, fraction,
			midpoint_on_image, interval};
}

std::string fixed(double value, int precision)
{
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(precision) << value;
	return stream.str();
}

} // namespace

Poly::Poly(const std::vector<XY> &exterior, const std::vector<std::vector<XY>> &holes)
{
	rings.push_back(exterior);
	rings.insert(rings.end(), holes.begin(), holes.end());
	bbox = {std::numeric_limits<double>::infinity(),
			std::numeric_limits<double>::infinity(),
			-std::numeric_limits<double>::infinity(),
			-std::numeric_limits<double>::infinity()};
	for (const auto &point : exterior) {
		bbox[0] = std::min(bbox[0], point[0]);
		bbox[1] = std::min(bbox[1], point[1]);
		bbox[2] = std::max(bbox[2], point[0]);
		bbox[3] = std::max(bbox[3], point[1]);
	}
}

bool Poly::contains(const XY &point) const
{
	if (rings.empty() || point[0] < bbox[0] || point[0] > bbox[2] || point[1] < bbox[1] ||
			point[1] > bbox[3] || !ring_contains(rings[0], point))
		return false;
	for (std::size_t i = 1; i < rings.size(); ++i)
		if (ring_contains(rings[i], point))
			return false;
	return true;
}

double Poly::boundary_distance(const XY &point) const
{
	double best = std::numeric_limits<double>::infinity();
	for (const auto &ring : rings)
		if (!ring.empty())
			for_each_edge(ring, [&](const XY &a, const XY &b) {
				best = std::min(best, point_segment_distance(point, a, b));
			});
	return best;
}

bool Poly::intersects_segment(const XY &a, const XY &b) const
{
	if (!bbox_meets_segment(bbox, a, b))
		return false;
	for (const auto &ring : rings) {
		if (ring.empty())
			continue;
		bool intersects = false;
		for_each_edge(ring, [&](const XY &p, const XY &q) {
			intersects = intersects || segments_cross(a, b, p, q);
		});
		if (intersects)
			return true;
	}
	return contains(a);
}

bool Poly::eroded_meets_segment(const XY &a, const XY &b, double radius) const
{
	if (radius <= 0.0)
		return intersects_segment(a, b);
	const auto length = std::hypot(b[0] - a[0], b[1] - a[1]);
	if (length < 1e-12)
		return contains(a) && boundary_distance(a) >= radius;
	if (!bbox_meets_segment(expanded_bbox(bbox, -radius), a, b))
		return false;
	const XY direction{(b[0] - a[0]) / length, (b[1] - a[1]) / length};
	for (double t = 0.0; t <= length;) {
		const XY point{a[0] + t * direction[0], a[1] + t * direction[1]};
		const auto distance = boundary_distance(point);
		if (contains(point)) {
			if (distance >= radius)
				return true;
			t += std::max(radius - distance, 1e-4);
		} else {
			t += std::max(distance, 1e-4);
		}
	}
	return false;
}

Footprints::Footprints(const std::vector<Building> &buildings)
{
	keys.reserve(buildings.size());
	polys.reserve(buildings.size());
	for (const auto &building : buildings) {
		keys.push_back(building.key);
		polys.emplace_back(building.ring, building.holes);
	}
}

std::vector<std::size_t> Footprints::indices_of(const std::string &key) const
{
	std::vector<std::size_t> result;
	for (std::size_t i = 0; i < keys.size(); ++i)
		if (keys[i] == key)
			result.push_back(i);
	return result;
}

std::vector<std::size_t> Footprints::containing(const XY &point) const
{
	std::vector<std::size_t> result;
	for (std::size_t i = 0; i < polys.size(); ++i)
		if (polys[i].contains(point))
			result.push_back(i);
	return result;
}

LineOfSight line_of_sight(const Wall &wall, const XY &camera_xy,
		const Footprints &footprints, const std::optional<std::string> &own_key,
		const Params &params)
{
	const auto own = own_key ? footprints.indices_of(*own_key)
							 : own_indices_geometric(wall, footprints);
	std::vector<std::pair<std::size_t, double>> lenient;
	for (const auto index : footprints.containing(camera_xy)) {
		const auto depth = footprints.polys[index].boundary_distance(camera_xy);
		if (depth > los_inside_tol_m)
			return {0.0, {0.0, 0.0}};
		lenient.emplace_back(index, los_inside_tol_m + 0.05);
	}
	const auto shrink_own =
			std::max(own_shrink_m, chord_sag(wall, footprints, own) + 0.2);
	const auto sample_count = std::max<std::size_t>(params.los_samples, std::size_t{1});
	const auto tangent = wall.tangent();
	std::vector<std::pair<double, bool>> visible;
	visible.reserve(sample_count);
	for (std::size_t sample = 0; sample < sample_count; ++sample) {
		const auto s = wall.length * (static_cast<double>(sample) + 0.5) /
					   static_cast<double>(sample_count);
		const XY point{wall.a[0] + s * tangent[0] + sample_front_m * wall.normal[0],
				wall.a[1] + s * tangent[1] + sample_front_m * wall.normal[1]};
		bool blocked = false;
		for (std::size_t i = 0; i < footprints.polys.size() && !blocked; ++i) {
			const auto lenient_it = std::find_if(lenient.begin(), lenient.end(),
					[&](const auto &entry) { return entry.first == i; });
			if (lenient_it != lenient.end())
				blocked = footprints.polys[i].eroded_meets_segment(
						camera_xy, point, lenient_it->second);
			else if (std::find(own.begin(), own.end(), i) != own.end())
				blocked = footprints.polys[i].eroded_meets_segment(
						camera_xy, point, shrink_own);
			else
				blocked = footprints.polys[i].intersects_segment(camera_xy, point);
		}
		visible.emplace_back(s, !blocked);
	}
	std::size_t seen_count = 0;
	double lo = std::numeric_limits<double>::infinity();
	double hi = -std::numeric_limits<double>::infinity();
	for (const auto &[s, seen] : visible)
		if (seen) {
			++seen_count;
			lo = std::min(lo, s);
			hi = std::max(hi, s);
		}
	if (!seen_count)
		return {0.0, {0.0, 0.0}};
	const auto half = 0.5 * wall.length / static_cast<double>(sample_count);
	return {static_cast<double>(seen_count) / static_cast<double>(sample_count),
			{std::max(0.0, lo - half), std::min(wall.length, hi + half)}};
}

double geometric_score(double distance_m, double incidence_deg, double angwidth_deg)
{
	return std::cos(incidence_deg * 0.017453292519943295769) *
		   std::clamp(1.0 - std::abs(distance_m - 12.0) / 30.0, 0.2, 1.0) *
		   std::min(1.0, angwidth_deg / 40.0);
}

double wall_height_m(const Wall &wall, const Params &params)
{
	return wall.height_osm && *wall.height_osm != 0.0 ? *wall.height_osm
													  : params.default_height_m;
}

std::vector<ViewCandidate> geometric_candidates(const Wall &wall,
		const std::map<std::string, Camera> &cameras, const Params &params,
		const Footprints *footprints)
{
	std::vector<ViewCandidate> result;
	const auto mid = wall.midpoint();
	const auto top_height = wall_height_m(wall, params);
	for (const auto &[pano_id, camera] : cameras) {
		const XY camera_xy{camera.centre[0], camera.centre[1]};
		const auto outward = (camera_xy[0] - wall.a[0]) * wall.normal[0] +
							 (camera_xy[1] - wall.a[1]) * wall.normal[1];
		if (outward <= 0.0)
			continue;
		const XY to_mid{camera_xy[0] - mid[0], camera_xy[1] - mid[1]};
		const auto distance = std::hypot(to_mid[0], to_mid[1]);
		if (distance < 1e-6 || distance > params.far_dist_m)
			continue;
		const auto cosine = std::clamp(
				(to_mid[0] * wall.normal[0] + to_mid[1] * wall.normal[1]) / distance,
				-1.0, 1.0);
		const auto incidence = std::acos(cosine) * 57.295779513082320876;
		const XY va{wall.a[0] - camera_xy[0], wall.a[1] - camera_xy[1]};
		const XY vb{wall.b[0] - camera_xy[0], wall.b[1] - camera_xy[1]};
		const auto na = std::hypot(va[0], va[1]), nb = std::hypot(vb[0], vb[1]);
		const auto angwidth = std::acos(std::clamp((va[0] * vb[0] + va[1] * vb[1]) /
														   std::max(1e-9, na * nb),
									  -1.0, 1.0)) *
							  57.295779513082320876;
		std::vector<Gate> gates{{GateName::Outward, true, outward},
				{GateName::Near, distance >= params.view_dist[0], distance},
				{GateName::Far, distance <= params.view_dist[1], distance},
				{GateName::Incidence, incidence <= params.max_incidence_deg, incidence},
				{GateName::Angwidth, angwidth >= params.min_angwidth_deg, angwidth}};
		const auto z_base = z_base_of(camera);
		const pose::Projector projector(camera);
		double bottom_v_max = -std::numeric_limits<double>::infinity();
		double top_v_min = std::numeric_limits<double>::infinity();
		for (const auto s : {0.0, 0.5 * wall.length, wall.length}) {
			const auto bottom =
					projector.project(wall.point(s, 0.0, z_base, params.eps_m)).v;
			const auto top =
					projector.project(wall.point(s, top_height, z_base, params.eps_m)).v;
			bottom_v_max = std::max(bottom_v_max, std::isfinite(bottom) ? bottom : 9.0);
			top_v_min = std::min(top_v_min, std::isfinite(top) ? top : -9.0);
		}
		bool fov_ok = false, midpoint_on_image = false;
		double fov_fraction = 0.0;
		std::optional<XY> s_on_image;
		const bool nadir_ok_spherical =
				camera.is_spherical() && bottom_v_max <= params.v_range[1];
		const bool zenith_ok_spherical =
				camera.is_spherical() && top_v_min >= params.v_range[0];
		if (camera.is_spherical()) {
			gates.push_back({GateName::Nadir, nadir_ok_spherical, bottom_v_max});
			gates.push_back({GateName::Zenith, zenith_ok_spherical, top_v_min});
		} else {
			const auto fov = perspective_fov(wall, camera, z_base, top_height, params);
			fov_ok = fov.ok;
			fov_fraction = fov.fraction;
			midpoint_on_image = fov.midpoint_on_image;
			s_on_image = fov.interval;
			gates.push_back({GateName::Fov, fov.ok, fov.fraction});
			gates.push_back({GateName::Nadir, fov.ok, fov.fraction});
			gates.push_back({GateName::Zenith, true, fov.fraction});
		}
		const bool nadir_ok = camera.is_spherical() ? nadir_ok_spherical : fov_ok;
		const bool zenith_ok = camera.is_spherical() ? zenith_ok_spherical : true;
		std::optional<std::string> reason;
		if (distance < params.view_dist[0])
			reason = "NEAR " + fixed(distance, 1) + " m";
		else if (distance > params.view_dist[1])
			reason = "FAR " + fixed(distance, 0) + " m";
		else if (incidence > params.max_incidence_deg)
			reason = "incidence " + fixed(incidence, 0) + " deg";
		else if (angwidth < params.min_angwidth_deg)
			reason = "angwidth " + fixed(angwidth, 0) + " deg";
		else if (!nadir_ok) {
			if (camera.is_spherical())
				reason = "nadir v " + fixed(bottom_v_max, 2);
			else if (fov_fraction >= params.fov_min_on_image && !midpoint_on_image)
				reason = "FOV midpoint off image (" + fixed(100.0 * fov_fraction, 0) +
						 " % on image)";
			else
				reason = "FOV " + fixed(100.0 * fov_fraction, 0) + " % on image";
		} else if (!zenith_ok)
			reason = "zenith v " + fixed(top_v_min, 2);

		ViewCandidate candidate;
		candidate.wall_key = wall.key;
		candidate.pano_id = pano_id;
		candidate.dist_m = distance;
		candidate.incidence_deg = incidence;
		candidate.angwidth_deg = angwidth;
		candidate.g_score = geometric_score(distance, incidence, angwidth);
		candidate.gates = std::move(gates);
		candidate.rejected_reason = std::move(reason);
		if (!candidate.rejected_reason && footprints) {
			const auto los = line_of_sight(
					wall, camera_xy, *footprints, wall.building_key, params);
			candidate.visible_frac = los.visible_fraction;
			candidate.s_vis = los.visible_s;
			candidate.set_gate(GateName::Los,
					los.visible_fraction >= params.los_min_visible, los.visible_fraction);
			if (los.visible_fraction < params.los_min_visible)
				candidate.rejected_reason =
						"LOS " + fixed(100.0 * los.visible_fraction, 0) + " %";
		} else if (!candidate.rejected_reason) {
			candidate.visible_frac = 1.0;
			candidate.s_vis = {0.0, wall.length};
		}
		if (s_on_image && !candidate.rejected_reason) {
			candidate.s_vis[0] = std::max(candidate.s_vis[0], (*s_on_image)[0]);
			candidate.s_vis[1] = std::min(candidate.s_vis[1], (*s_on_image)[1]);
		}
		result.push_back(std::move(candidate));
	}
	return result;
}

std::map<std::string, std::vector<ViewCandidate>> mark_reachable(std::vector<Wall> &walls,
		const std::map<std::string, Camera> &cameras, const Footprints &footprints,
		const Params &params, bool with_keys)
{
	std::map<std::string, std::vector<ViewCandidate>> result;
	for (auto &wall : walls) {
		auto candidates = geometric_candidates(
				wall, cameras, params, with_keys ? &footprints : nullptr);
		// The keyless entry point is used only before building keys exist; still
		// run the geometric own-footprint resolver for its LOS check.
		if (!with_keys)
			for (auto &candidate : candidates) {
				if (candidate.rejected_reason)
					continue;
				const auto camera = cameras.find(candidate.pano_id);
				if (camera == cameras.end())
					continue;
				const auto los = line_of_sight(wall,
						{camera->second.centre[0], camera->second.centre[1]}, footprints,
						std::nullopt, params);
				candidate.visible_frac = los.visible_fraction;
				candidate.s_vis = los.visible_s;
				candidate.set_gate(GateName::Los,
						los.visible_fraction >= params.los_min_visible,
						los.visible_fraction);
				if (los.visible_fraction < params.los_min_visible)
					candidate.rejected_reason =
							"LOS " + fixed(100.0 * los.visible_fraction, 0) + " %";
			}
		const bool geometric_ok = std::any_of(
				candidates.begin(), candidates.end(), [](const ViewCandidate &candidate) {
					return !candidate.rejected_reason ||
						   candidate.rejected_reason->starts_with("LOS");
				});
		const bool los_ok = std::any_of(
				candidates.begin(), candidates.end(), [](const ViewCandidate &candidate) {
					return !candidate.rejected_reason;
				});
		wall.reachable = los_ok;
		wall.unreachable_reason = los_ok		 ? ""
								  : geometric_ok ? "no_line_of_sight"
												 : "no_camera_in_front";
		result.emplace(wall.key, std::move(candidates));
	}
	return result;
}

namespace
{
double positive_mod(double value, double modulus)
{
	const auto result = std::fmod(value, modulus);
	return result < 0.0 ? result + modulus : result;
}

std::uint8_t rgb_to_gray(const projection::Rgb &pixel)
{
	return static_cast<std::uint8_t>(
			(std::uint32_t(pixel[0]) * 9798 + std::uint32_t(pixel[1]) * 19235 +
					std::uint32_t(pixel[2]) * 3735 + 16384) >>
			15);
}

std::size_t reflect101(std::int64_t index, std::size_t length)
{
	if (length == 1)
		return 0;
	const auto n = static_cast<std::int64_t>(length);
	const auto period = 2 * (n - 1);
	auto reflected = index % period;
	if (reflected < 0)
		reflected += period;
	if (reflected >= n)
		reflected = period - reflected;
	return static_cast<std::size_t>(reflected);
}

std::vector<double> laplacian(
		const std::vector<double> &gray, std::size_t width, std::size_t height)
{
	std::vector<double> result(width * height);
	for (std::size_t y = 0; y < height; ++y)
		for (std::size_t x = 0; x < width; ++x) {
			const auto centre = gray[y * width + x];
			const auto up =
					gray[reflect101(static_cast<std::int64_t>(y) - 1, height) * width +
							x];
			const auto down =
					gray[reflect101(static_cast<std::int64_t>(y) + 1, height) * width +
							x];
			const auto left =
					gray[y * width + reflect101(static_cast<std::int64_t>(x) - 1, width)];
			const auto right =
					gray[y * width + reflect101(static_cast<std::int64_t>(x) + 1, width)];
			result[y * width + x] = up + down + left + right - 4.0 * centre;
		}
	return result;
}

std::string format_fixed(double value, int precision)
{
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(precision) << value;
	return stream.str();
}

double candidate_gate_value(
		const ViewCandidate &candidate, GateName name, double fallback)
{
	const auto gate = candidate.gate(name);
	return gate ? gate->value : fallback;
}

bool camera_is_spherical(
		const std::map<std::string, Camera> &cameras, const std::string &pano_id)
{
	const auto camera = cameras.find(pano_id);
	return camera == cameras.end() || camera->second.is_spherical();
}
} // namespace

double solar_elevation_deg(std::int64_t captured_at_ms, double longitude, double latitude)
{
	const auto n = static_cast<double>(captured_at_ms) / 86400000.0 - 10957.5;
	const auto l = positive_mod(280.460 + 0.9856474 * n, 360.0);
	const auto g = positive_mod(357.528 + 0.9856003 * n, 360.0) * 0.017453292519943295769;
	const auto lambda = (l + 1.915 * std::sin(g) + 0.020 * std::sin(2.0 * g)) *
						0.017453292519943295769;
	const auto epsilon = (23.439 - 0.0000004 * n) * 0.017453292519943295769;
	const auto right_ascension =
			std::atan2(std::cos(epsilon) * std::sin(lambda), std::cos(lambda));
	const auto declination = std::asin(std::sin(epsilon) * std::sin(lambda));
	const auto gmst_hours = positive_mod(18.697374558 + 24.06570982441908 * n, 24.0);
	const auto hour_angle =
			positive_mod(gmst_hours * 15.0 + longitude, 360.0) * 0.017453292519943295769 -
			right_ascension;
	const auto lat = latitude * 0.017453292519943295769;
	return std::asin(std::sin(lat) * std::sin(declination) +
					 std::cos(lat) * std::cos(declination) * std::cos(hour_angle)) *
		   57.295779513082320876;
}

projection::Image remap_bilinear(const projection::Image &source,
		const std::vector<double> &map_u, const std::vector<double> &map_v,
		std::size_t width, std::size_t rows, bool wrap_columns)
{
	projection::Image result;
	result.width = static_cast<unsigned>(width);
	result.height = static_cast<unsigned>(rows);
	result.pixels.resize(width * rows);
	const auto source_width = static_cast<std::int64_t>(source.width);
	const auto source_height = static_cast<std::int64_t>(source.height);
	for (std::size_t row = 0; row < rows; ++row)
		for (std::size_t column = 0; column < width; ++column) {
			const auto index = row * width + column;
			if (index >= map_u.size() || index >= map_v.size() || !source_width ||
					!source_height)
				continue;
			const auto u = map_u[index], v = map_v[index];
			const auto x_floor = std::floor(u), y_floor = std::floor(v);
			const auto fx = u - x_floor, fy = v - y_floor;
			const auto x0 = static_cast<std::int64_t>(x_floor);
			const auto y0 = static_cast<std::int64_t>(y_floor);
			std::array<double, 3> value{};
			for (int dy = 0; dy <= 1; ++dy)
				for (int dx = 0; dx <= 1; ++dx) {
					const auto weight = (dx ? fx : 1.0 - fx) * (dy ? fy : 1.0 - fy);
					if (weight == 0.0)
						continue;
					auto x = x0 + dx;
					const auto y = y0 + dy;
					if (wrap_columns) {
						x %= source_width;
						if (x < 0)
							x += source_width;
					} else if (x < 0 || x >= source_width || y < 0 ||
							   y >= source_height) {
						continue;
					}
					if (x < 0 || x >= source_width || y < 0 || y >= source_height)
						continue;
					const auto &pixel =
							source.pixels[static_cast<std::size_t>(y) * source.width +
										  static_cast<std::size_t>(x)];
					for (std::size_t channel = 0; channel < 3; ++channel)
						value[channel] += weight * pixel[channel];
				}
			for (std::size_t channel = 0; channel < 3; ++channel)
				result.pixels[index][channel] = static_cast<std::uint8_t>(
						std::clamp(std::round(value[channel]), 0.0, 255.0));
		}
	return result;
}

Preview preview_crop(const Wall &wall, const Camera &camera, double z_base,
		const projection::Image &image, const Params &params,
		const PreviewOptions &options)
{
	const auto image_width = static_cast<double>(image.width);
	const auto image_height = static_cast<double>(image.height);
	double s0 = options.s_range ? (*options.s_range)[0] : 0.0;
	double s1 = options.s_range ? (*options.s_range)[1] : wall.length;
	if (s1 - s0 < 0.5) {
		s0 = 0.0;
		s1 = std::max(wall.length, 0.5);
	}
	const auto h_top = options.h_top.value_or(1.15 * wall_height_m(wall, params));
	constexpr double h_bottom = -0.5;
	const auto width = std::max<std::size_t>(options.width, 1);
	const auto ppm = static_cast<double>(width) / (s1 - s0);
	const auto rounded_rows = static_cast<std::size_t>(
			std::max(8.0, imgops::round_half_even((h_top - h_bottom) * ppm)));
	const auto rows = std::max<std::size_t>(1, std::min(options.max_rows, rounded_rows));
	const auto ppm_v = static_cast<double>(rows) / (h_top - h_bottom);
	const pose::Projector projector(camera);
	const bool spherical = camera.is_spherical();
	std::vector<double> map_u(rows * width), map_v(rows * width), ray(rows * width);
	std::vector<bool> outside(rows * width);
	for (std::size_t row = 0; row < rows; ++row) {
		const auto height = h_top - (static_cast<double>(row) + 0.5) / ppm_v;
		for (std::size_t column = 0; column < width; ++column) {
			const auto s = s0 + (static_cast<double>(column) + 0.5) / ppm;
			const auto projected =
					projector.project(wall.point(s, height, z_base, params.eps_m));
			const auto index = row * width + column;
			ray[index] = projected.length_m;
			if (spherical && image_width > 0.0 && image_height > 0.0) {
				map_u[index] = std::fmod(projected.u * image_width - 0.5, image_width);
				if (map_u[index] < 0.0)
					map_u[index] += image_width;
				map_v[index] = std::clamp(
						projected.v * image_height - 0.5, 0.0, image_height - 1.0);
			} else if (projected.inside_image()) {
				map_u[index] = projected.u * image_width - 0.5;
				map_v[index] = projected.v * image_height - 0.5;
			} else {
				map_u[index] = -1e6;
				map_v[index] = -1e6;
				outside[index] = true;
			}
		}
	}
	auto rgb = remap_bilinear(image, map_u, map_v, width, rows, spherical);
	std::vector<std::uint8_t> occlusion(rows * width);
	for (std::size_t index = 0; index < rows * width; ++index) {
		const auto v = image_height > 0.0 ? (map_v[index] + 0.5) / image_height : 0.0;
		if (spherical) {
			if (v > params.v_range[1])
				occlusion[index] |= OCC_NADIR;
			if (v < params.v_range[0])
				occlusion[index] |= OCC_ZENITH;
		} else if (outside[index]) {
			occlusion[index] |= OCC_OUTSIDE;
		}
	}
	if (options.depth && options.depth->width && options.depth->height &&
			image_width > 0.0 && image_height > 0.0) {
		for (std::size_t index = 0; index < rows * width; ++index) {
			if (outside[index])
				continue;
			const auto u = (map_u[index] + 0.5) / image_width;
			auto v = (map_v[index] + 0.5) / image_height;
			if (!spherical)
				v = std::clamp(v, 0.0, 1.0);
			const auto column = static_cast<unsigned>(std::clamp<std::int64_t>(
					static_cast<std::int64_t>(u * options.depth->width), 0,
					static_cast<std::int64_t>(options.depth->width) - 1));
			const auto row = static_cast<unsigned>(std::clamp<std::int64_t>(
					static_cast<std::int64_t>(v * options.depth->height), 0,
					static_cast<std::int64_t>(options.depth->height) - 1));
			const auto depth = options.depth->at(column, row);
			if (std::isfinite(depth) && depth < ray[index] - 1.5f)
				occlusion[index] |= OCC_CLOUD;
		}
	}
	if (options.s_visible)
		for (std::size_t column = 0; column < width; ++column) {
			const auto s = s0 + (static_cast<double>(column) + 0.5) / ppm;
			if (s < (*options.s_visible)[0] || s > (*options.s_visible)[1])
				for (std::size_t row = 0; row < rows; ++row)
					occlusion[row * width + column] |= OCC_FOOTPRINT;
		}
	const auto vegetation = rectify::vegetation_mask(rgb, ppm);
	for (std::size_t index = 0; index < vegetation.size(); ++index)
		if (vegetation[index])
			occlusion[index] |= OCC_SEG;
	const auto local_camera = wall.sh_of(camera.centre, z_base);
	return {std::move(rgb), std::move(occlusion), ppm, ppm_v, s0, s1, h_top, h_bottom,
			(local_camera[0] - s0) * ppm, (h_top - local_camera[1]) * ppm_v};
}

std::vector<Gate> image_gates(const projection::Image &image,
		const std::vector<std::uint8_t> &occlusion, const PanoMeta &meta,
		std::optional<double> longitude, std::optional<double> latitude)
{
	const auto width = static_cast<std::size_t>(image.width);
	const auto height = static_cast<std::size_t>(image.height);
	const auto count = width * height;
	if (!width || !height || image.pixels.size() < count || occlusion.size() < count)
		return {{GateName::Blur, false, 0.0}, {GateName::Exposure, false, 0.0},
				{GateName::Clipped, false, 1.0}, {GateName::Night, false, -90.0},
				{GateName::Quality, meta.quality >= quality_min, meta.quality},
				{GateName::Occlusion, false, 1.0}};
	std::vector<double> gray(count);
	std::vector<bool> clear(count);
	for (std::size_t i = 0; i < count; ++i) {
		gray[i] = rgb_to_gray(image.pixels[i]);
		clear[i] = occlusion[i] == 0;
	}
	const auto clear_count =
			static_cast<std::size_t>(std::count(clear.begin(), clear.end(), true));
	if (static_cast<double>(clear_count) / static_cast<double>(count) < 0.2)
		for (std::size_t i = 0; i < count; ++i)
			clear[i] = (occlusion[i] & OCC_OUTSIDE) == 0;
	const bool any_clear =
			std::any_of(clear.begin(), clear.end(), [](bool value) { return value; });
	const auto lap = laplacian(gray, width, height);
	double blur = 0.0, mean_l = 0.0, clipped = 1.0;
	if (any_clear) {
		double lap_sum = 0.0, gray_sum = 0.0;
		std::size_t used = 0, clipped_count = 0;
		for (std::size_t i = 0; i < count; ++i)
			if (clear[i]) {
				lap_sum += lap[i];
				gray_sum += gray[i];
				clipped_count += gray[i] <= 5.0 || gray[i] >= 250.0;
				++used;
			}
		mean_l = gray_sum / static_cast<double>(used);
		double variance_sum = 0.0;
		for (std::size_t i = 0; i < count; ++i)
			if (clear[i]) {
				const auto delta = lap[i] - lap_sum / static_cast<double>(used);
				variance_sum += delta * delta;
			}
		blur = variance_sum / static_cast<double>(used);
		clipped = static_cast<double>(clipped_count) / static_cast<double>(used);
	}
	const auto elevation = meta.captured_at != 0 ? solar_elevation_deg(meta.captured_at,
														   longitude.value_or(meta.lon),
														   latitude.value_or(meta.lat))
												 : 90.0;
	const bool night = elevation < night_elevation_deg || mean_l < night_l;
	std::size_t occluded = 0;
	for (std::size_t i = 0; i < count; ++i)
		occluded += (occlusion[i] & occluder_bits) != 0;
	const auto fraction = static_cast<double>(occluded) / static_cast<double>(count);
	return {{GateName::Blur, blur >= blur_min, blur},
			{GateName::Exposure, mean_l >= 35.0 && mean_l <= 220.0 && clipped < clip_max,
					mean_l},
			{GateName::Clipped, clipped < clip_max, clipped},
			{GateName::Night, !night, elevation},
			{GateName::Quality, meta.quality >= quality_min, meta.quality},
			{GateName::Occlusion, fraction <= occ_max, fraction}};
}

void apply_image_gates(ViewCandidate &candidate, const std::vector<Gate> &gates)
{
	for (const auto &gate : gates)
		candidate.set_gate(gate.name, gate.passed, gate.value);
	const auto find_gate = [&](GateName name, Gate fallback) {
		const auto it = std::find_if(gates.begin(), gates.end(),
				[&](const Gate &gate) { return gate.name == name; });
		return it == gates.end() ? fallback : *it;
	};
	const auto occ = find_gate(GateName::Occlusion, {GateName::Occlusion, true, 0.0});
	const auto blur = find_gate(GateName::Blur, {GateName::Blur, true, 0.0});
	candidate.f_occ = occ.value;
	candidate.blur = blur.value;
	if (candidate.rejected_reason)
		return;
	const auto night = find_gate(GateName::Night, {GateName::Night, true, 90.0});
	const auto exposure = find_gate(GateName::Exposure, {GateName::Exposure, true, 0.0});
	const auto clipped = find_gate(GateName::Clipped, {GateName::Clipped, true, 0.0});
	const auto quality = find_gate(GateName::Quality, {GateName::Quality, true, 1.0});
	if (!blur.passed)
		candidate.rejected_reason = "blur " + format_fixed(blur.value, 0);
	else if (!night.passed)
		candidate.rejected_reason = night.value < night_elevation_deg
											? "night"
											: "dark L " + format_fixed(exposure.value, 0);
	else if (!exposure.passed)
		candidate.rejected_reason = "exposure L " + format_fixed(exposure.value, 0) +
									" clip " + format_fixed(100.0 * clipped.value, 0) +
									" %";
	else if (!quality.passed)
		candidate.rejected_reason = "quality " + format_fixed(quality.value, 2);
	else if (!occ.passed)
		candidate.rejected_reason =
				"occluded " + format_fixed(100.0 * occ.value, 0) + " %";
}

std::vector<ViewCandidate> select_views(std::vector<ViewCandidate> &candidates,
		const std::map<std::string, Camera> &cameras, const Params &params,
		std::size_t max_views)
{
	std::vector<std::size_t> passing;
	for (std::size_t i = 0; i < candidates.size(); ++i)
		if (!candidates[i].rejected_reason)
			passing.push_back(i);
	std::vector<double> blurs;
	for (const auto index : passing)
		if (candidates[index].blur > 0.0)
			blurs.push_back(candidates[index].blur);
	const auto median_blur = blurs.empty() ? 0.0 : imgops::median(std::move(blurs));
	for (const auto index : passing) {
		auto &candidate = candidates[index];
		double blur_factor = 1.0;
		if (median_blur > 0.0 && candidate.blur > 0.0) {
			const auto relative = candidate.blur / median_blur;
			candidate.set_gate(GateName::BlurRel, relative >= blur_rel, relative);
			if (relative < blur_rel) {
				candidate.rejected_reason = "blur " + format_fixed(candidate.blur, 0) +
											" < 0.35 x median " +
											format_fixed(median_blur, 0);
				continue;
			}
			blur_factor = std::clamp(relative, blur_rel, 1.0);
		}
		const auto quality = candidate_gate_value(candidate, GateName::Quality, 1.0);
		const auto camera = cameras.find(candidate.pano_id);
		const auto pose_factor =
				camera == cameras.end() ? 1.0 : camera->second.pose_factor;
		const auto is_spherical =
				camera == cameras.end() || camera->second.is_spherical();
		const auto camera_factor = is_spherical || params.perspective_penalty <= 0.0
										   ? 1.0
										   : params.perspective_penalty;
		candidate.score = candidate.g_score * quality * (1.0 - candidate.f_occ) *
						  pose_factor * blur_factor * camera_factor;
	}
	passing.erase(std::remove_if(passing.begin(), passing.end(),
						  [&](std::size_t index) {
							  return candidates[index].rejected_reason.has_value();
						  }),
			passing.end());
	const bool panorama_passes =
			std::any_of(passing.begin(), passing.end(), [&](std::size_t index) {
				return camera_is_spherical(cameras, candidates[index].pano_id);
			});
	if (params.perspective_penalty <= 0.0 && panorama_passes) {
		passing.erase(std::remove_if(passing.begin(), passing.end(),
							  [&](std::size_t index) {
								  const bool keep = camera_is_spherical(
										  cameras, candidates[index].pano_id);
								  if (!keep) {
									  candidates[index].score = 0.0;
									  candidates[index].rejected_reason =
											  "perspective view, panorama available";
								  }
								  return !keep;
							  }),
				passing.end());
	}
	std::stable_sort(passing.begin(), passing.end(), [&](std::size_t a, std::size_t b) {
		if (candidates[a].score != candidates[b].score)
			return candidates[a].score > candidates[b].score;
		return candidates[a].pano_id < candidates[b].pano_id;
	});
	if (passing.empty())
		return {};
	std::vector<std::size_t> chosen{passing.front()};
	std::vector<std::size_t> rest(passing.begin() + 1, passing.end());
	const auto first_camera = cameras.find(candidates[passing.front()].pano_id);
	if (first_camera != cameras.end() && max_views >= 2) {
		const auto far = std::find_if(rest.begin(), rest.end(), [&](std::size_t index) {
			const auto camera = cameras.find(candidates[index].pano_id);
			return camera != cameras.end() &&
				   std::hypot(camera->second.centre[0] - first_camera->second.centre[0],
						   camera->second.centre[1] - first_camera->second.centre[1]) >=
						   min_position_sep_m;
		});
		if (far != rest.end()) {
			chosen.push_back(*far);
			rest.erase(far);
		}
	}
	for (const auto index : rest) {
		if (chosen.size() >= max_views)
			break;
		chosen.push_back(index);
	}
	std::vector<std::array<double, 2>> unique_positions;
	for (const auto index : chosen) {
		const auto camera = cameras.find(candidates[index].pano_id);
		if (camera == cameras.end())
			continue;
		const std::array<double, 2> point{
				camera->second.centre[0], camera->second.centre[1]};
		const bool unique = std::all_of(
				unique_positions.begin(), unique_positions.end(), [&](const auto &other) {
					return std::hypot(point[0] - other[0], point[1] - other[1]) >=
						   min_position_sep_m;
				});
		if (unique)
			unique_positions.push_back(point);
	}
	std::vector<ViewCandidate> result;
	result.reserve(chosen.size());
	for (const auto index : chosen) {
		candidates[index].set_gate(GateName::Positions, unique_positions.size() >= 2,
				double(unique_positions.size()));
		result.push_back(candidates[index]);
	}
	return result;
}

} // namespace arnis::mapillary
