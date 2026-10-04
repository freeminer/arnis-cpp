#pragma once

#include "types.h"
#include "project.h"
#include "rectify.h"
#include "imgops.h"

#include <array>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace arnis::mapillary::refine
{

inline constexpr double profile_ppm = 10.0;
inline constexpr const char *shear_none = "SHEAR_NONE";
inline constexpr const char *shear_ok = "SHEAR_OK";
inline constexpr const char *shear_rejected = "SHEAR_REJECTED";
inline constexpr const char *on_plane = "ON_PLANE";
inline constexpr const char *plane_mismatch = "PLANE_MISMATCH";
inline constexpr const char *unverified = "UNVERIFIED";
inline constexpr const char *roof_sky = "ROOF_SKY";
inline constexpr const char *roof_edge = "ROOF_EDGE";
inline constexpr const char *roof_osm = "ROOF_OSM";
inline constexpr const char *roof_behind = "ROOF_BEHIND";
inline constexpr const char *roof_unconfirmed = "ROOF_UNCONFIRMED";
inline constexpr const char *roof_nodata = "ROOF_NODATA";
inline constexpr const char *ground_edge = "GROUND_EDGE";
inline constexpr const char *ground_cloud = "GROUND_CLOUD";
inline constexpr const char *ground_default = "GROUND_DEFAULT";

struct Peak
{
	double x_m{};
	double energy{};
	double vlen_m{};
};

struct LineSegment
{
	double x0{}, y0{}, x1{}, y1{}, length{}, angle_deg{};
	double mid_x() const { return 0.5 * (x0 + x1); }
	double mid_y() const { return 0.5 * (y0 + y1); }
};

struct LeanStats
{
	std::size_t n{};
	double inliers{}, rms_deg{}, x_spread_m{};
};

struct RefineDetails
{
	std::size_t n_vertical{}, n_horizontal{}, lean_n{};
	double lean_inliers{}, lean_rms_deg{}, lean_x_spread_m{}, c0_after_deg{};
	double grad_c0_deg{}, grad_c1_deg_per_m{}, grad_support{};
	std::size_t plane_gate_n{};
	std::string sky_mode;
	std::optional<double> h_sky_raw, h_cloud;
	std::array<double, 2> s_range{};
	std::array<double, 4> box_px{};
	double l_fit{}, h_osm{};
};

struct ViewInputs
{
	const Wall *wall{};
	std::string pano_id;
	double fitted_length{};
	std::optional<std::array<double, 2>> visible_s;
	std::optional<double> cloud_height;
	GroundSource ground_source{GroundSource::Default};
};

/// Per-image evidence used by wall-level height and roofline aggregation.
struct Refinement
{
	std::string view_key;
	double c0_deg{}, c1_deg_per_m{};
	std::string lean_flag{shear_none};
	std::array<std::array<double, 3>, 3> h_shear{
			{{{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}}}};
	double plane_slope{};
	std::string plane_flag{unverified};
	std::optional<double> h_sky;
	std::string roof_flag{roof_osm};
	std::vector<std::string> roof_flags;
	double roof_spread_m{}, ground_dz{};
	std::string ground_flag{ground_default};
	std::vector<float> profile;
	double profile_x0_m{};
	std::vector<Peak> peaks;
	double z_base{};
	static Refinement empty(const std::string &view_key, double z_base);
};

struct LeanResult
{
	double c0_deg{}, c1_deg_per_m{};
	std::string flag{shear_none};
	std::array<std::array<double, 3>, 3> homography{
			{{{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}}}};
	LeanStats stats;
};

struct PlaneGateResult
{
	double slope{};
	std::string flag{unverified};
	std::size_t lines{};
};

struct ViewRefinement
{
	Refinement refinement;
	RefineDetails details;
};

struct Roofline
{
	std::optional<double> height;
	std::string flag{roof_osm};
	double spread_m{};
	std::vector<std::string> flags;
};

struct ViewEvidence
{
	const Refinement *refinement{};
	double score{};
	double ds_m{};
	std::optional<double> h_cloud;
	struct CropEvidence
	{
		const projection::Image *rgb{};
		const std::vector<std::uint8_t> *occlusion{};
		double ppm{};
		double s0_m{};
	};
	const CropEvidence *crop{};
};

struct Extent
{
	std::optional<double> s_l, s_r;
	std::string src_a{"osm"}, src_b{"osm"};
	std::vector<std::string> flags;
};

struct RoofVote
{
	std::optional<double> height;
	std::string source{roof_osm};
};

struct HeightDecision
{
	double height{};
	std::string source;
	std::vector<std::string> flags;
};

RoofVote roof_vote(const std::vector<ViewEvidence> &views);
LeanResult lean_keystone(const rectify::LooseCrop &crop,
		const std::vector<LineSegment> &verticals, const Params &params);
std::array<double, 3> lean_from_gradients(const std::vector<std::uint8_t> &gray,
		std::size_t width, std::size_t height, double x_foot, double ppm,
		const std::vector<std::uint8_t> *occlusion = nullptr);
std::tuple<imgops::Mask, const char *, std::vector<bool>> sky_mask(
		const projection::Image &image,
		std::optional<std::pair<double, double>> facade_rows = std::nullopt);
std::vector<double> row_energy(const projection::Image &image,
		const std::vector<std::uint8_t> *occlusion = nullptr,
		const std::vector<std::size_t> *columns = nullptr);
std::optional<std::size_t> strongest_row(const std::vector<double> &energy,
		std::ptrdiff_t first, std::ptrdiff_t last, double ratio,
		const std::vector<bool> *seen = nullptr);
double edge_y(const std::vector<double> &energy, std::size_t row);
std::pair<double, std::string> ground_row(
		const rectify::LooseCrop &crop, double ground_row_y, GroundSource source);
std::pair<std::vector<float>, std::vector<Peak>> column_profile(
		const rectify::LooseCrop &crop, const std::vector<LineSegment> &verticals,
		const std::array<double, 2> &row_bounds);
Roofline sky_roofline(const rectify::LooseCrop &crop, const imgops::Mask &sky,
		const std::array<double, 2> &s_range, std::optional<double> osm_height,
		bool height_source_default, std::optional<double> cloud_height,
		const Params &params);
PlaneGateResult plane_gate(const rectify::LooseCrop &crop,
		const std::vector<LineSegment> &horizontals,
		const std::array<double, 4> &pixel_bounds, const Params &params);
std::vector<LineSegment> detect_segments(const std::vector<std::uint8_t> &gray,
		std::size_t width, std::size_t height, double minimum_length_px);
std::pair<std::vector<LineSegment>, std::vector<LineSegment>> lsd_families(
		const std::vector<std::uint8_t> &gray, std::size_t width, std::size_t height,
		double pixels_per_metre, const std::vector<std::uint8_t> *occlusion = nullptr);
HeightDecision decide_height(std::optional<double> h_sky, std::optional<double> h_cloud,
		std::optional<double> h_osm, bool osm_default, const std::string &osm_source,
		const Params &params, bool roof_blind);
Extent extent_joint(const std::vector<Peak> &peaks, double x_a, double x_b,
		double osm_length, const ViewEvidence::CropEvidence *crop, bool fix_a, bool fix_b,
		const Params &params);
WallDecision decide_wall(const Wall &wall, double fitted_length, bool fix_a, bool fix_b,
		const std::vector<ViewEvidence> &views,
		const std::vector<std::string> &extra_flags, const Params &params);
std::pair<std::size_t, std::size_t> colour_trim(const projection::Image &texture,
		const std::vector<bool> &valid, std::size_t pixels_per_block,
		const std::array<double, 3> &wall_medoid_lab, std::size_t max_blocks,
		double delta_e);
std::array<double, 3> wall_medoid(
		const projection::Image &texture, const std::vector<bool> &valid);
std::array<double, 3> phase_search(const projection::Image &texture,
		const std::vector<bool> &valid, std::size_t pixels_per_block,
		const Params &params);
void finalise_wall(WallDecision &decision, const projection::Image &texture,
		const std::vector<bool> &valid, std::size_t pixels_per_block,
		std::optional<std::array<double, 3>> wall_medoid_lab, const Params &params);
std::array<double, 4> final_rect(const WallDecision &decision);
ViewRefinement refine_view(
		const rectify::LooseCrop &crop, const ViewInputs &input, const Params &params);

} // namespace arnis::mapillary::refine
