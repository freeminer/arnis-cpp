#pragma once

#include "geometry.h"
#include "project.h"
#include "pose.h"
#include "sfm.h"

#include <algorithm>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <array>
#include <vector>
namespace arnis::mapillary
{
inline constexpr double los_inside_tol_m = 1.0;
inline constexpr double own_shrink_m = 0.3;
inline constexpr double sample_front_m = 0.1;
inline constexpr std::array<std::size_t, 2> fov_grid{11, 5};
inline constexpr double blur_min = 40.0;
inline constexpr double blur_rel = 0.35;
inline constexpr double clip_max = 0.12;
inline constexpr double night_l = 45.0;
inline constexpr double night_elevation_deg = -3.0;
inline constexpr double quality_min = 0.5;
inline constexpr double occ_max = 0.55;
inline constexpr double min_position_sep_m = 3.0;
inline constexpr std::uint8_t occluder_bits = OCC_FOOTPRINT | OCC_CLOUD | OCC_SEG;

struct Poly
{
	std::vector<std::vector<std::array<double, 2>>> rings;
	std::array<double, 4> bbox{};
	Poly() = default;
	Poly(const std::vector<std::array<double, 2>> &exterior,
			const std::vector<std::vector<std::array<double, 2>>> &holes);
	bool contains(const std::array<double, 2> &point) const;
	double boundary_distance(const std::array<double, 2> &point) const;
	bool intersects_segment(
			const std::array<double, 2> &a, const std::array<double, 2> &b) const;
	bool eroded_meets_segment(const std::array<double, 2> &a,
			const std::array<double, 2> &b, double radius) const;
};

struct Footprints
{
	std::vector<std::string> keys;
	std::vector<Poly> polys;
	Footprints() = default;
	explicit Footprints(const std::vector<Building> &buildings);
	std::vector<std::size_t> indices_of(const std::string &key) const;
	std::vector<std::size_t> containing(const std::array<double, 2> &point) const;
};

struct LineOfSight
{
	double visible_fraction{};
	std::array<double, 2> visible_s{};
};

struct Preview
{
	projection::Image rgb;
	std::vector<std::uint8_t> occlusion;
	double ppm{}, ppm_v{}, s0{}, s1{}, h_top{}, h_bottom{}, x_foot{}, y_camera{};
};

struct PreviewOptions
{
	const sfm::DepthMap *depth{};
	std::optional<std::array<double, 2>> s_range, s_visible;
	std::optional<double> h_top;
	std::size_t width = 512, max_rows = 1024;
};

// Compact renderer-independent counterpart of Rust's wall visibility masks.
class Coverage
{
	std::size_t width_, height_;
	std::vector<unsigned char> covered_;

public:
	Coverage(std::size_t width, std::size_t height) :
			width_(width), height_(height), covered_(width * height)
	{
	}
	bool mark(double u, double v)
	{
		if (width_ == 0 || height_ == 0 || !std::isfinite(u) || !std::isfinite(v))
			return false;
		auto x = std::clamp<long>(std::lround(u), 0, long(width_ - 1));
		auto y = std::clamp<long>(std::lround(v), 0, long(height_ - 1));
		covered_[std::size_t(y) * width_ + std::size_t(x)] = 1;
		return true;
	}
	bool project_and_mark(const Camera &camera, const Wall &wall, double s, double h)
	{
		auto p = project_wall_point(camera, wall, s, h);
		return p && mark((*p)[0], (*p)[1]);
	}
	double fraction() const
	{
		if (covered_.empty())
			return 0;
		return double(std::count(
					   covered_.begin(), covered_.end(), static_cast<unsigned char>(1))) /
			   double(covered_.size());
	}
	bool usable(double minimum = .10) const { return fraction() >= minimum; }
};

double geometric_score(double distance_m, double incidence_deg, double angwidth_deg);
double wall_height_m(const Wall &wall, const Params &params);
LineOfSight line_of_sight(const Wall &wall, const std::array<double, 2> &camera_xy,
		const Footprints &footprints, const std::optional<std::string> &own_key,
		const Params &params);
std::vector<ViewCandidate> geometric_candidates(const Wall &wall,
		const std::map<std::string, Camera> &cameras, const Params &params,
		const Footprints *footprints = nullptr);
std::map<std::string, std::vector<ViewCandidate>> mark_reachable(std::vector<Wall> &walls,
		const std::map<std::string, Camera> &cameras, const Footprints &footprints,
		const Params &params, bool with_keys = true);
double solar_elevation_deg(
		std::int64_t captured_at_ms, double longitude, double latitude);
std::vector<Gate> image_gates(const projection::Image &crop_rgb,
		const std::vector<std::uint8_t> &occlusion, const PanoMeta &meta,
		std::optional<double> longitude = std::nullopt,
		std::optional<double> latitude = std::nullopt);
projection::Image remap_bilinear(const projection::Image &source,
		const std::vector<double> &map_u, const std::vector<double> &map_v,
		std::size_t width, std::size_t rows, bool wrap_columns);
Preview preview_crop(const Wall &wall, const Camera &camera, double z_base,
		const projection::Image &image, const Params &params,
		const PreviewOptions &options = {});
void apply_image_gates(ViewCandidate &candidate, const std::vector<Gate> &gates);
std::vector<ViewCandidate> select_views(std::vector<ViewCandidate> &candidates,
		const std::map<std::string, Camera> &cameras, const Params &params,
		std::size_t max_views);
}
