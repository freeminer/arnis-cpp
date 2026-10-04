#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include "project.h"
#include "pose.h"
#include "plane.h"
namespace arnis::mapillary::sfm
{
struct DepthMap;
}
namespace arnis::mapillary::rectify
{
inline constexpr double max_elevation_deg = 80, bottom_margin_m = 1.5,
						cloud_occlusion_margin_m = 1.5, los_inside_tolerance_m = 1,
						own_shrink_m = .3, sample_front_m = .1;
enum Occlusion : unsigned char
{
	Visible = 0,
	Outside = 1,
	Cloud = 2,
	Vegetation = 4,
	Sky = 8
};
struct Crop
{
	double s0{}, s1{}, z0{}, z1{}, pixels_per_metre{};
	unsigned width{}, height{};
};

/// Metric bounds of a wall crop, expanded for neighbours and roof discovery,
/// then clipped to the camera's maximum elevation cone.
inline std::array<double, 4> crop_extent(const ::arnis::mapillary::Wall &wall,
		const ::arnis::mapillary::PlaneFit &fit, double osm_height, const Params &params,
		const Camera *camera = nullptr, std::optional<double> base_z = std::nullopt,
		HeightSource height_source = HeightSource::Default)
{
	const auto fitted = plane::fitted_wall(wall, fit);
	const double margin = std::max(3.0, 0.3 * fitted.length);
	const double s_min = -margin, s_max = fitted.length + margin;
	const double height = osm_height != 0.0 ? osm_height : params.default_height_m;
	double h_top = std::max(params.top_margin[0] * height, height + params.top_margin[1]);
	if (height_source == HeightSource::Default)
		h_top = std::max(h_top, params.top_margin_default_m);
	const double h_bottom = -bottom_margin_m;
	if (camera && base_z) {
		const auto relative = fitted.sh_of(camera->centre, *base_z);
		const double ds = std::max({0.0, s_min - relative[0], relative[0] - s_max});
		const double horizontal_distance = std::hypot(relative[2], ds);
		const double cap = relative[1] +
						   std::max(horizontal_distance, 0.5) *
								   std::tan(max_elevation_deg * 0.017453292519943295769);
		h_top = std::max(h_bottom + 1.0, std::min(h_top, cap));
	}
	return {s_min, s_max, h_bottom, h_top};
}

inline double choose_ppm(double distance_m, unsigned image_width, const Params &params)
{
	const double distance = std::max(distance_m, 0.5);
	const double native =
			static_cast<double>(image_width) / (6.2831853071795864769 * distance);
	const double requested =
			std::clamp(0.9 * native, params.loose_ppm[0], params.loose_ppm[1]);
	return std::min(requested, std::max(native, params.loose_ppm[0]));
}

/// Rasterized loose facade crop consumed by the refinement stage.
struct LooseCrop
{
	std::string wall_key, pano_id;
	projection::Image rgb;
	std::vector<std::uint8_t> occl;
	double ppm{}, s0{}, h_bot{}, h_top{}, x_foot{}, y_cam{}, z_base{};
	std::string z_base_source;
	double s_to_x(double s) const { return (s - s0) * ppm; }
	double h_to_y(double h) const { return (h_top - h) * ppm; }
	double x_to_s(double x) const { return x / ppm + s0; }
	double y_to_h(double y) const { return h_top - y / ppm; }
};

struct View
{
	const Camera *camera{};
	const ::arnis::mapillary::PlaneFit *fit{};
	double z_base{};
	std::optional<double> distance_m;
	std::optional<std::array<double, 2>> visible_s;
};

struct RectView
{
	projection::Image rgb;
	std::vector<bool> valid;
};

RectView resample_rect(const ::arnis::mapillary::Wall &wall, const View &view,
		const projection::Image &image, const std::array<double, 4> &rectangle,
		const std::array<std::array<double, 3>, 3> *shear, unsigned pixels_per_block,
		const LooseCrop *crop, const Params &params);

std::vector<bool> vegetation_mask(
		const projection::Image &image, double pixels_per_metre);
std::vector<bool> footprint_columns(const ::arnis::mapillary::Wall &fitted_wall,
		const std::array<double, 3> &camera_centre,
		const std::vector<double> &wall_columns,
		const std::vector<::arnis::mapillary::Building> &buildings,
		const std::string &own_building_key, double setback_m);
LooseCrop loose_crop(const ::arnis::mapillary::Wall &wall, const View &view,
		const projection::Image &image, const sfm::DepthMap *depth,
		const std::vector<::arnis::mapillary::Building> &buildings, const Params &params);

inline Crop extent(double wall_length, double base_z, double top_z, double ppm,
		double bottom_margin = bottom_margin_m)
{
	Crop out;
	out.s1 = std::max(0., wall_length);
	out.z0 = base_z - bottom_margin;
	out.z1 = std::max(out.z0, top_z);
	out.pixels_per_metre = std::max(1., ppm);
	out.width = unsigned(std::ceil((out.s1 - out.s0) * out.pixels_per_metre));
	out.height = unsigned(std::ceil((out.z1 - out.z0) * out.pixels_per_metre));
	return out;
}
inline double choose_ppm(double distance_m, unsigned image_width, double requested,
		double min_ppm = 2, double max_ppm = 16)
{
	if (distance_m <= 0 || !image_width)
		return min_ppm;
	return std::clamp(std::min(requested, double(image_width) / (2 * distance_m)),
			min_ppm, max_ppm);
}
inline projection::Rgb bilinear(const projection::Image &image, double x, double y)
{
	projection::Rgb out{};
	if (!image.width || !image.height)
		return out;
	long x0 = long(std::floor(x)), y0 = long(std::floor(y));
	double tx = x - x0, ty = y - y0;
	auto at = [&](long px, long py) {
		px = std::clamp(px, 0L, long(image.width) - 1);
		py = std::clamp(py, 0L, long(image.height) - 1);
		return image.pixels[std::size_t(py) * image.width + px];
	};
	for (unsigned c = 0; c < 3; ++c) {
		double value =
				at(x0, y0)[c] * (1 - tx) * (1 - ty) + at(x0 + 1, y0)[c] * tx * (1 - ty) +
				at(x0, y0 + 1)[c] * (1 - tx) * ty + at(x0 + 1, y0 + 1)[c] * tx * ty;
		out[c] = std::uint8_t(std::clamp(std::round(value), 0., 255.));
	}
	return out;
}
inline std::vector<std::array<float, 2>> pixel_map(const pose::Projector &projector,
		const std::vector<std::array<double, 3>> &points)
{
	std::vector<std::array<float, 2>> result;
	result.reserve(points.size());
	for (const auto &point : points) {
		auto pixel = projector.pixel(point);
		result.push_back(pixel.value_or(std::array<float, 2>{-1, -1}));
	}
	return result;
}
inline bool usable(unsigned char flags)
{
	return flags == Visible;
}
} // namespace arnis::mapillary::rectify
