#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
namespace arnis::mapillary
{
inline constexpr double EARTH_RADIUS_M = 6371000.0;
inline constexpr std::uint8_t OCC_FOOTPRINT = 1;
inline constexpr std::uint8_t OCC_CLOUD = 2;
inline constexpr std::uint8_t OCC_NADIR = 4;
inline constexpr std::uint8_t OCC_ZENITH = 8;
inline constexpr std::uint8_t OCC_SEG = 16;
inline constexpr std::uint8_t OCC_SKY = 32;
inline constexpr std::uint8_t OCC_OUTSIDE = 64;
inline constexpr std::uint8_t CLS_WALL = 255;
inline constexpr std::uint8_t CLS_WINDOW = 192;
inline constexpr std::uint8_t CLS_DOOR = 128;
inline constexpr std::uint8_t CLS_UNKNOWN = 64;
inline constexpr std::uint8_t CLS_NODATA = 0;

enum class CameraModel
{
	Spherical,
	Perspective,
	Brown,
	Fisheye
};
inline CameraModel parse_camera_model(const std::string &value)
{
	if (value == "equirectangular" || value == "spherical")
		return CameraModel::Spherical;
	if (value == "perspective")
		return CameraModel::Perspective;
	if (value == "brown")
		return CameraModel::Brown;
	if (value == "fisheye")
		return CameraModel::Fisheye;
	return CameraModel::Spherical;
}
inline bool is_spherical(CameraModel model)
{
	return model == CameraModel::Spherical;
}
inline const char *camera_model_name(CameraModel model)
{
	switch (model) {
	case CameraModel::Spherical:
		return "spherical";
	case CameraModel::Perspective:
		return "perspective";
	case CameraModel::Brown:
		return "brown";
	case CameraModel::Fisheye:
		return "fisheye";
	}
	return "spherical";
}

enum class PoseSource
{
	Sfm,
	Sequence,
	Autolevel,
	Heading
};
inline PoseSource parse_pose_source(const std::string &value)
{
	if (value == "sfm")
		return PoseSource::Sfm;
	if (value == "sequence")
		return PoseSource::Sequence;
	if (value == "autolevel")
		return PoseSource::Autolevel;
	return PoseSource::Heading;
}
inline const char *pose_source_name(PoseSource source)
{
	switch (source) {
	case PoseSource::Sfm:
		return "sfm";
	case PoseSource::Sequence:
		return "sequence";
	case PoseSource::Autolevel:
		return "autolevel";
	case PoseSource::Heading:
		return "heading";
	}
	return "heading";
}
inline double pose_factor(PoseSource source)
{
	switch (source) {
	case PoseSource::Sfm:
		return 1.0;
	case PoseSource::Sequence:
		return 0.8;
	case PoseSource::Autolevel:
		return 0.65;
	case PoseSource::Heading:
		return 0.4;
	}
	return 0.4;
}

enum class RegSource
{
	Local,
	Global,
	Unregistered,
	NoCluster
};
inline RegSource parse_reg_source(const std::string &value)
{
	if (value == "local")
		return RegSource::Local;
	if (value == "global")
		return RegSource::Global;
	if (value == "none")
		return RegSource::Unregistered;
	return RegSource::NoCluster;
}
inline const char *reg_source_name(RegSource source)
{
	switch (source) {
	case RegSource::Local:
		return "local";
	case RegSource::Global:
		return "global";
	case RegSource::Unregistered:
		return "none";
	case RegSource::NoCluster:
		return "no_cluster";
	}
	return "no_cluster";
}
enum class PlaneSource
{
	Cloud,
	OsmRegistered,
	OsmRaw
};
inline PlaneSource parse_plane_source(const std::string &value)
{
	if (value == "cloud")
		return PlaneSource::Cloud;
	if (value == "osm-registered")
		return PlaneSource::OsmRegistered;
	return PlaneSource::OsmRaw;
}
inline const char *plane_source_name(PlaneSource source)
{
	if (source == PlaneSource::Cloud)
		return "cloud";
	if (source == PlaneSource::OsmRegistered)
		return "osm-registered";
	return "osm-raw";
}
enum class GroundSource
{
	Cloud,
	Sequence,
	Default
};
inline GroundSource parse_ground_source(const std::string &value)
{
	if (value == "cloud")
		return GroundSource::Cloud;
	if (value == "sequence")
		return GroundSource::Sequence;
	return GroundSource::Default;
}
inline const char *ground_source_name(GroundSource source)
{
	if (source == GroundSource::Cloud)
		return "cloud";
	if (source == GroundSource::Sequence)
		return "sequence";
	return "default";
}
enum class HeightSource
{
	Tag,
	Levels,
	Default
};
inline HeightSource parse_height_source(const std::string &value)
{
	if (value == "tag")
		return HeightSource::Tag;
	if (value == "levels")
		return HeightSource::Levels;
	return HeightSource::Default;
}
inline const char *height_source_name(HeightSource source)
{
	if (source == HeightSource::Tag)
		return "tag";
	if (source == HeightSource::Levels)
		return "levels";
	return "default";
}
enum class GeometrySource
{
	Computed,
	Gps
};
inline GeometrySource parse_geometry_source(const std::string &value)
{
	return value == "gps" ? GeometrySource::Gps : GeometrySource::Computed;
}
inline const char *geometry_source_name(GeometrySource source)
{
	return source == GeometrySource::Gps ? "gps" : "computed";
}
enum class OsmKind
{
	Way,
	Relation
};
inline OsmKind parse_osm_kind(const std::string &value)
{
	return value == "relation" ? OsmKind::Relation : OsmKind::Way;
}
inline const char *osm_kind_name(OsmKind kind)
{
	return kind == OsmKind::Relation ? "relation" : "way";
}
enum class Tier
{
	A,
	B,
	C,
	D
};
enum class GateName
{
	Outward,
	Near,
	Far,
	Incidence,
	Angwidth,
	Fov,
	Nadir,
	Zenith,
	Los,
	Blur,
	BlurRel,
	Night,
	Exposure,
	Clipped,
	Quality,
	Occlusion,
	Positions
};
inline const char *gate_name(GateName gate)
{
	switch (gate) {
	case GateName::Outward:
		return "outward";
	case GateName::Near:
		return "near";
	case GateName::Far:
		return "far";
	case GateName::Incidence:
		return "incidence";
	case GateName::Angwidth:
		return "angwidth";
	case GateName::Fov:
		return "fov";
	case GateName::Nadir:
		return "nadir";
	case GateName::Zenith:
		return "zenith";
	case GateName::Los:
		return "los";
	case GateName::Blur:
		return "blur";
	case GateName::BlurRel:
		return "blur_rel";
	case GateName::Night:
		return "night";
	case GateName::Exposure:
		return "exposure";
	case GateName::Clipped:
		return "clipped";
	case GateName::Quality:
		return "quality";
	case GateName::Occlusion:
		return "occlusion";
	case GateName::Positions:
		return "positions";
	}
	return "outward";
}
inline GateName parse_gate_name(const std::string &value)
{
	const std::array<std::pair<const char *, GateName>, 17> names{
			{{"outward", GateName::Outward}, {"near", GateName::Near},
					{"far", GateName::Far}, {"incidence", GateName::Incidence},
					{"angwidth", GateName::Angwidth}, {"fov", GateName::Fov},
					{"nadir", GateName::Nadir}, {"zenith", GateName::Zenith},
					{"los", GateName::Los}, {"blur", GateName::Blur},
					{"blur_rel", GateName::BlurRel}, {"night", GateName::Night},
					{"exposure", GateName::Exposure}, {"clipped", GateName::Clipped},
					{"quality", GateName::Quality}, {"occlusion", GateName::Occlusion},
					{"positions", GateName::Positions}}};
	for (const auto &[name, gate] : names)
		if (value == name)
			return gate;
	return GateName::Outward;
}
inline const char *tier_name(Tier tier)
{
	switch (tier) {
	case Tier::A:
		return "A";
	case Tier::B:
		return "B";
	case Tier::C:
		return "C";
	case Tier::D:
		return "D";
	}
	return "D";
}
inline Tier parse_tier(const std::string &value)
{
	if (value == "A")
		return Tier::A;
	if (value == "B")
		return Tier::B;
	if (value == "C")
		return Tier::C;
	return Tier::D;
}
inline bool tier_has_texture(Tier tier)
{
	return tier == Tier::A;
}
inline bool tier_has_blocks(Tier tier)
{
	return tier == Tier::A || tier == Tier::B;
}

// Field-for-field counterpart of Rust's mapillary::Params.  Keeping these in
// one value object is important: geometry, visibility, rectification and tier
// selection must use the same thresholds for a generation and for cache keys.
struct Params
{
	double min_wall_m{3.0}, split_wall_m{30.0}, merge_deg{4.0};
	double min_ring_area_m2{4.0}, metres_per_level{3.0}, default_height_m{9.0};
	double eps_m{0.05}, pano_margin_m{45.0}, osm_margin_m{60.0};
	std::array<double, 2> view_dist{4.0, 35.0};
	double far_dist_m{45.0}, max_incidence_deg{60.0}, min_angwidth_deg{12.0};
	std::uint32_t los_samples{9};
	double los_min_visible{0.6};
	std::array<double, 2> v_range{0.05, 0.78};
	double fov_min_on_image{0.35}, persp_margin{0.06};
	bool persp_landscape_only{true};
	double perspective_penalty{1.0};
	std::vector<CameraModel> camera_types{
			CameraModel::Spherical, CameraModel::Perspective, CameraModel::Brown};
	double seq_window_s{60.0};
	std::uint32_t seq_min_panos{3};
	double roll_max_deg{20.0}, roll_sd_floor_deg{1.5}, roll_sd_mult{3.0};
	std::uint32_t autolevel_size{768}, autolevel_min_lines{15};
	double autolevel_max_residual_deg{1.5}, autolevel_line_deg{25.0};
	std::array<double, 2> ground_r{2.5, 8.0};
	std::uint32_t ground_min_pts{30};
	double ground_pct{5.0};
	std::array<double, 2> cam_height_range{1.2, 4.5};
	double rig_height_default_m{2.5};
	std::array<double, 2> persp_cam_height_range{0.8, 2.6};
	double persp_height_default_m{1.4};
	double foot_search_m{3.0};
	std::uint32_t foot_min_pts{20};
	double foot_pct{5.0}, depth_radius_m{60.0};
	std::array<std::uint32_t, 2> depth_size{720, 360};
	double shot_time_tol_s{1.0}, metric_tol{0.02};
	double reg_radius_m{40.0}, reg_check_radius_m{25.0}, reg_coarse_m{12.0};
	double reg_fine_m{1.5}, reg_step_fine{0.25}, reg_min_inliers{0.5};
	double reg_uniqueness{1.2}, reg_theta_deg{3.0}, reg_theta_step{0.5};
	std::array<double, 2> reg_band{3.0, 22.0};
	double plane_band_m{0.25}, plane_angle_step_deg{0.02}, plane_search_m{3.0};
	double plane_max_angle_deg{10.0};
	std::array<double, 2> plane_z_range{1.0, 30.0};
	std::array<double, 2> loose_ppm{6.0, 20.0};
	std::uint32_t tex_ppb{8};
	std::array<double, 2> top_margin{2.2, 12.0};
	double top_margin_default_m{30.0};
	double lean_max_deg{6.0}, lean_max_keystone{0.3};
	std::uint32_t lean_min_lines{15};
	double lean_min_inliers{0.6}, plane_gate_deg_per_m{0.35};
	std::array<double, 2> roof_ratio{0.5, 1.7};
	double roof_spread_m{2.0}, extent_window_m{3.0}, extent_lambda{1.0};
	double phase_range_m{0.5}, phase_step_m{0.125};
	double dark_thr{0.12}, window_darkfrac{0.4}, door_band_m{3.0};
	double unknown_nodata{0.3};
	std::uint32_t palette_k{3};
	double palette_merge{0.06};
	double tier_a{0.6}, tier_b{0.38}, tier_c{0.2};
	double tier_a_max_unknown{0.4}, tier_b_max_unknown{0.8};

	bool admits(CameraModel model) const
	{
		if (model == CameraModel::Spherical)
			model = CameraModel::Spherical;
		return std::find(camera_types.begin(), camera_types.end(), model) !=
			   camera_types.end();
	}
	double height_prior(CameraModel model) const
	{
		return model == CameraModel::Spherical ? rig_height_default_m
											   : persp_height_default_m;
	}
	std::string digest() const
	{
		std::ostringstream stream;
		stream << std::setprecision(17);
		auto d = [&](double value) { stream << value << ';'; };
		auto u = [&](auto value) { stream << value << ';'; };
		auto pair = [&](const auto &value) {
			d(value[0]);
			d(value[1]);
		};
		d(min_wall_m);
		d(split_wall_m);
		d(merge_deg);
		d(min_ring_area_m2);
		d(metres_per_level);
		d(default_height_m);
		d(eps_m);
		d(pano_margin_m);
		d(osm_margin_m);
		pair(view_dist);
		d(far_dist_m);
		d(max_incidence_deg);
		d(min_angwidth_deg);
		u(los_samples);
		d(los_min_visible);
		pair(v_range);
		d(fov_min_on_image);
		d(persp_margin);
		u(persp_landscape_only);
		d(perspective_penalty);
		for (const auto model : camera_types)
			u(static_cast<int>(model));
		d(seq_window_s);
		u(seq_min_panos);
		d(roll_max_deg);
		d(roll_sd_floor_deg);
		d(roll_sd_mult);
		u(autolevel_size);
		u(autolevel_min_lines);
		d(autolevel_max_residual_deg);
		d(autolevel_line_deg);
		pair(ground_r);
		u(ground_min_pts);
		d(ground_pct);
		pair(cam_height_range);
		d(rig_height_default_m);
		pair(persp_cam_height_range);
		d(persp_height_default_m);
		d(foot_search_m);
		u(foot_min_pts);
		d(foot_pct);
		d(depth_radius_m);
		u(depth_size[0]);
		u(depth_size[1]);
		d(shot_time_tol_s);
		d(metric_tol);
		d(reg_radius_m);
		d(reg_check_radius_m);
		d(reg_coarse_m);
		d(reg_fine_m);
		d(reg_step_fine);
		d(reg_min_inliers);
		d(reg_uniqueness);
		d(reg_theta_deg);
		d(reg_theta_step);
		pair(reg_band);
		d(plane_band_m);
		d(plane_angle_step_deg);
		d(plane_search_m);
		d(plane_max_angle_deg);
		pair(plane_z_range);
		pair(loose_ppm);
		u(tex_ppb);
		pair(top_margin);
		d(top_margin_default_m);
		d(lean_max_deg);
		d(lean_max_keystone);
		u(lean_min_lines);
		d(lean_min_inliers);
		d(plane_gate_deg_per_m);
		pair(roof_ratio);
		d(roof_spread_m);
		d(extent_window_m);
		d(extent_lambda);
		d(phase_range_m);
		d(phase_step_m);
		d(dark_thr);
		d(window_darkfrac);
		d(door_band_m);
		d(unknown_nodata);
		u(palette_k);
		d(palette_merge);
		d(tier_a);
		d(tier_b);
		d(tier_c);
		d(tier_a_max_unknown);
		d(tier_b_max_unknown);
		std::uint64_t hash = 14695981039346656037ull;
		for (const unsigned char byte : stream.str()) {
			hash ^= byte;
			hash *= 1099511628211ull;
		}
		std::ostringstream result;
		result << std::hex << std::setw(16) << std::setfill('0') << hash;
		return result.str();
	}
};

struct Frame
{
	double lon0{}, lat0{}, kx{}, ky{};
	Frame() = default;
	Frame(double longitude, double latitude) : lon0(longitude), lat0(latitude)
	{
		constexpr double pi = 3.14159265358979323846;
		ky = pi / 180.0 * EARTH_RADIUS_M;
		kx = ky * std::cos(latitude * pi / 180.0);
	}
	std::array<double, 2> to_enu(double longitude, double latitude) const
	{
		return {(longitude - lon0) * kx, (latitude - lat0) * ky};
	}
	std::array<double, 2> to_lonlat(const std::array<double, 2> &xy) const
	{
		return {xy[0] / kx + lon0, xy[1] / ky + lat0};
	}
};
struct BBox
{
	double min_lat{}, min_lon{}, max_lat{}, max_lon{};
	std::array<double, 2> centre() const
	{
		return {0.5 * (min_lon + max_lon), 0.5 * (min_lat + max_lat)};
	}
};
inline std::string building_key(OsmKind kind, std::int64_t id)
{
	return std::string(kind == OsmKind::Way ? "w" : "r") + std::to_string(id);
}
inline std::string wall_key(const std::string &building, std::size_t index,
		std::size_t piece, std::size_t pieces)
{
	return building + "_" + std::to_string(index) +
		   (pieces > 1 ? "p" + std::to_string(piece) : "");
}
inline std::string view_key(const std::string &wall, const std::string &pano)
{
	return wall + "__" + pano;
}
inline std::optional<std::pair<std::string, std::string>> split_view_key(
		const std::string &key)
{
	const auto separator = key.rfind("__");
	if (separator == std::string::npos)
		return std::nullopt;
	return std::make_pair(key.substr(0, separator), key.substr(separator + 2));
}
struct PanoMeta
{
	std::string id;
	double lon{}, lat{}, alt{}, compass{};
	std::optional<std::array<double, 3>> rotation;
	std::optional<double> atomic_scale;
	std::int64_t captured_at{};
	std::string sequence;
	double quality{};
	std::uint32_t width{}, height{};
	std::optional<std::string> cluster_id;
	GeometrySource geometry_source{GeometrySource::Computed};
	CameraModel camera_type{CameraModel::Spherical};
	std::vector<double> camera_params;
	bool is_spherical() const { return camera_type == CameraModel::Spherical; }
	bool valid() const
	{
		return !id.empty() && std::isfinite(lon) && std::isfinite(lat) &&
			   std::isfinite(alt) && std::isfinite(compass);
	}
};
inline bool is_spherical_camera(CameraModel model)
{
	return model == CameraModel::Spherical;
}
struct RegResult
{
	double dx{}, dy{}, theta_deg{};
	double inliers_before{}, inliers_after{}, ambiguity_ratio{}, radius_agreement_m{};
	std::size_t n_points{};
	bool accepted{}, scale_suspect{};
	RegSource source{RegSource::NoCluster};
	std::array<double, 3> shift() const { return {dx, dy, theta_deg}; }
	bool valid() const
	{
		return std::isfinite(dx) && std::isfinite(dy) && std::isfinite(theta_deg) &&
			   n_points > 0;
	}
};
struct WallEdge
{
	std::size_t edge_idx{};
	std::int64_t node_a{}, node_b{};
	double s0{}, s1{};
};
struct Building
{
	std::string key;
	std::int64_t osm_id{};
	OsmKind kind{OsmKind::Way};
	std::vector<std::array<double, 2>> ring;
	std::vector<std::vector<std::array<double, 2>>> holes;
	std::vector<std::int64_t> node_ids;
	std::map<std::string, std::string> tags;
	std::optional<double> height_osm;
	HeightSource height_source{HeightSource::Default};
	double min_height{};
	bool target{};
	std::vector<std::int64_t> member_ways;
	std::array<double, 2> centroid() const
	{
		std::array<double, 2> result{};
		if (ring.empty())
			return result;
		for (const auto &point : ring) {
			result[0] += point[0];
			result[1] += point[1];
		}
		result[0] /= ring.size();
		result[1] /= ring.size();
		return result;
	}
	double height_m(double default_height_m = 9.0) const
	{
		return height_osm.value_or(default_height_m);
	}
	double height_m(const Params &params) const
	{
		return height_m(params.default_height_m);
	}
	bool valid() const { return !key.empty() && ring.size() >= 3; }
};
struct Wall
{
	std::string key, building_key;
	std::size_t index{};
	std::int64_t node_a{}, node_b{};
	std::array<double, 2> a{}, b{}, normal{};
	double length{};
	std::vector<std::size_t> merged_indices;
	std::size_t piece{}, pieces{1};
	std::optional<double> height_osm;
	HeightSource height_source{HeightSource::Default};
	bool reachable{true};
	std::string unreachable_reason;
	std::vector<WallEdge> edges;
	double s_offset{};
	std::array<double, 2> tangent() const
	{
		const double dx = b[0] - a[0], dz = b[1] - a[1];
		const double length = std::max(1e-12, std::hypot(dx, dz));
		return {dx / length, dz / length};
	}
	std::array<double, 2> midpoint() const
	{
		return {0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])};
	}
	std::array<double, 3> point(
			double s, double height, double z_base, double epsilon = 0.0) const
	{
		const auto direction = tangent();
		return {a[0] + s * direction[0] + epsilon * normal[0],
				a[1] + s * direction[1] + epsilon * normal[1], z_base + height};
	}
	std::array<double, 3> sh_of(
			const std::array<double, 3> &position, double z_base) const
	{
		const auto direction = tangent();
		const double dx = position[0] - a[0], dz = position[1] - a[1];
		return {dx * direction[0] + dz * direction[1], position[2] - z_base,
				dx * normal[0] + dz * normal[1]};
	}
	double height_m(double default_height_m = 9.0) const
	{
		return height_osm.value_or(default_height_m);
	}
	double height_m(const Params &params) const
	{
		return height_m(params.default_height_m);
	}
	bool valid() const
	{
		return !key.empty() && std::isfinite(length) && length > 0.0 &&
			   std::isfinite(normal[0]) && std::isfinite(normal[1]);
	}
};
struct Gate
{
	GateName name{GateName::Outward};
	bool passed{};
	double value{};
};
struct ViewCandidate
{
	std::string wall_key, pano_id;
	double dist_m{}, incidence_deg{}, angwidth_deg{}, visible_frac{};
	std::array<double, 2> s_vis{};
	double g_score{}, f_occ{}, blur{}, score{};
	std::vector<Gate> gates;
	std::optional<std::string> rejected_reason;
	bool accepted() const { return !rejected_reason.has_value(); }
	bool valid() const
	{
		return !wall_key.empty() && !pano_id.empty() && std::isfinite(dist_m) &&
			   dist_m >= 0.0 && std::isfinite(score);
	}
	std::optional<Gate> gate(GateName name) const
	{
		for (const auto &entry : gates)
			if (entry.name == name)
				return entry;
		return std::nullopt;
	}
	void set_gate(GateName name, bool passed, double value)
	{
		const Gate replacement{name, passed, value};
		for (auto &entry : gates) {
			if (entry.name == name) {
				entry = replacement;
				return;
			}
		}
		gates.push_back(replacement);
	}
};
struct PlaneFit
{
	std::string wall_key;
	std::array<double, 2> normal{};
	double d{};
	PlaneSource source{PlaneSource::OsmRaw};
	std::size_t n_inliers{};
	double points_per_m{}, rms_m{}, angle_vs_osm_deg{}, offset_vs_osm_m{};
	std::optional<double> z_top98;
	bool z_continuous{};
	double ambiguity{};
	std::optional<std::array<double, 2>> a_ref, b_ref;
	std::optional<std::string> pano_id, cluster_id;
	bool valid() const
	{
		return !wall_key.empty() && std::isfinite(normal[0]) &&
			   std::isfinite(normal[1]) && std::isfinite(d) && n_inliers > 0;
	}
};
struct WallDecision
{
	std::string wall_key, source_a, source_b, height_source;
	double s_left{}, s_right{}, h_used{}, bimodality{};
	std::int32_t trimmed_a{}, trimmed_b{};
	std::optional<double> h_sky, h_cloud, h_osm;
	std::array<double, 2> phase{};
	std::vector<std::string> flags;
	bool valid() const
	{
		return !wall_key.empty() && std::isfinite(s_left) && std::isfinite(s_right) &&
			   std::isfinite(h_used) && s_right >= s_left && h_used >= 0.0;
	}
};
struct WallProduct
{
	std::string wall_key, building_key;
	std::int64_t node_a{}, node_b{};
	std::size_t piece{}, pieces{1};
	std::vector<WallEdge> edges;
	double col0_m{};
	std::uint32_t cols{}, rows{};
	std::vector<std::array<std::uint8_t, 3>> rgb;
	std::vector<std::uint8_t> classes;
	std::vector<bool> observed;
	std::vector<std::array<std::uint8_t, 3>> bands;
	// Row-major RGBA texture at eight pixels per metre.
	std::vector<std::uint8_t> texture_rgba;
	std::uint32_t texture_width{}, texture_height{};
	Tier tier{Tier::D};
	double confidence{}, height_used_m{}, unknown_share{};
	std::vector<std::string> views, flags;
	std::size_t cell_count() const { return std::size_t(cols) * rows; }
	bool valid() const
	{
		const auto cells = cell_count();
		return !wall_key.empty() && cols > 0 && rows > 0 && rgb.size() == cells &&
			   classes.size() == cells && observed.size() == cells &&
			   (texture_rgba.empty() ||
					   texture_rgba.size() ==
							   std::size_t(texture_width) * texture_height * 4);
	}
};
struct Camera
{
	std::string pano_id;
	CameraModel model{CameraModel::Spherical};
	std::array<double, 3> centre{};
	std::array<std::array<double, 3>, 3> axes{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
	double width{0}, height{0}, focal{0};
	// OpenSfM camera parameters.  Perspective uses [f, k1, k2], Brown uses
	// [fx, fy, cx, cy, k1, k2, p1, p2, k3].
	std::vector<double> parameters;
	PoseSource pose_source{PoseSource::Heading};
	double roll_deg{}, pitch_deg{}, ground_z{}, cam_height_m{};
	GroundSource ground_source{GroundSource::Default};
	std::optional<std::string> cluster_id, shot_id;
	std::optional<RegResult> registration;
	double compass_deg{}, pose_factor{0.4};
	bool is_spherical() const { return model == CameraModel::Spherical; }
};
inline bool finite(const std::array<double, 3> &v)
{
	return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
inline std::optional<std::array<double, 2>> project(
		const Camera &c, const std::array<double, 3> &p)
{
	if (!finite(p) || c.width <= 0 || c.height <= 0)
		return {};
	std::array<double, 3> d{p[0] - c.centre[0], p[1] - c.centre[1], p[2] - c.centre[2]};
	double x = d[0] * c.axes[0][0] + d[1] * c.axes[0][1] + d[2] * c.axes[0][2],
		   y = d[0] * c.axes[1][0] + d[1] * c.axes[1][1] + d[2] * c.axes[1][2],
		   z = d[0] * c.axes[2][0] + d[1] * c.axes[2][1] + d[2] * c.axes[2][2];
	if (c.model == CameraModel::Spherical) {
		constexpr double pi = 3.14159265358979323846;
		double n = std::hypot(x, y, z);
		if (n <= 1e-12)
			return {};
		return std::array<double, 2>{(std::atan2(x, z) / (2 * pi) + .5) * c.width,
				(.5 - std::asin(y / n) / pi) * c.height};
	}
	if (z <= 1e-9 || c.focal <= 0)
		return {};
	return std::array<double, 2>{
			c.width * .5 + c.focal * x / z, c.height * .5 + c.focal * y / z};
}
}
