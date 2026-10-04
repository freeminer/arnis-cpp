#include "rectify.h"

#include "imgops.h"
#include "sfm.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace arnis::mapillary::rectify
{
namespace
{
using Matrix3 = std::array<std::array<double, 3>, 3>;
constexpr std::uint8_t occluded_bits =
		OCC_FOOTPRINT | OCC_CLOUD | OCC_NADIR | OCC_ZENITH | OCC_SEG | OCC_OUTSIDE;
constexpr std::array<double, 2> veg_hue_strong{112.0, 165.0};
constexpr std::array<double, 2> veg_hue_weak{108.0, 165.0};
constexpr std::array<double, 2> veg_hue_dark{115.0, 165.0};

Matrix3 inverse3(const Matrix3 &matrix)
{
	const auto &a = matrix;
	const Matrix3 cofactors{{{{a[1][1] * a[2][2] - a[1][2] * a[2][1],
									 a[0][2] * a[2][1] - a[0][1] * a[2][2],
									 a[0][1] * a[1][2] - a[0][2] * a[1][1]}},
			{{a[1][2] * a[2][0] - a[1][0] * a[2][2],
					a[0][0] * a[2][2] - a[0][2] * a[2][0],
					a[0][2] * a[1][0] - a[0][0] * a[1][2]}},
			{{a[1][0] * a[2][1] - a[1][1] * a[2][0],
					a[0][1] * a[2][0] - a[0][0] * a[2][1],
					a[0][0] * a[1][1] - a[0][1] * a[1][0]}}}};
	const double determinant = a[0][0] * cofactors[0][0] + a[0][1] * cofactors[1][0] +
							   a[0][2] * cofactors[2][0];
	const double factor = std::abs(determinant) < 1e-300 ? 0.0 : 1.0 / determinant;
	Matrix3 result{};
	for (std::size_t row = 0; row < 3; ++row)
		for (std::size_t column = 0; column < 3; ++column)
			result[row][column] = cofactors[row][column] * factor;
	return result;
}

std::array<double, 2> apply_h(const Matrix3 &matrix, double s, double h)
{
	double denominator = matrix[2][0] * s + matrix[2][1] * h + matrix[2][2];
	if (std::abs(denominator) < 1e-12)
		denominator = 1e-12;
	return {(matrix[0][0] * s + matrix[0][1] * h + matrix[0][2]) / denominator,
			(matrix[1][0] * s + matrix[1][1] * h + matrix[1][2]) / denominator};
}

std::int64_t wrap_index(std::int64_t index, std::int64_t length)
{
	if (length <= 0)
		return 0;
	index %= length;
	return index < 0 ? index + length : index;
}

projection::Image remap_bilinear(const projection::Image &source,
		const std::vector<float> &map_x, const std::vector<float> &map_y, unsigned width,
		unsigned height, bool wrap)
{
	projection::Image output{
			width, height, std::vector<projection::Rgb>(std::size_t(width) * height)};
	if (!width || !height || !source.width || !source.height ||
			source.pixels.size() < std::size_t(source.width) * source.height)
		return output;
	const auto source_width = static_cast<std::int64_t>(source.width);
	const auto source_height = static_cast<std::int64_t>(source.height);
	const auto fetch = [&](std::int64_t x, std::int64_t y,
							   std::size_t channel) -> double {
		if (wrap) {
			x = wrap_index(x, source_width);
			y = wrap_index(y, source_height);
		} else if (x < 0 || y < 0 || x >= source_width || y >= source_height) {
			return 0.0;
		}
		return source.pixels[static_cast<std::size_t>(y) * source.width +
							 static_cast<std::size_t>(x)][channel];
	};
	const std::size_t count = std::size_t(width) * height;
	for (std::size_t i = 0; i < count; ++i) {
		const auto fixed_x = static_cast<std::int64_t>(
				imgops::round_half_even(static_cast<double>(map_x[i]) * 32.0));
		const auto fixed_y = static_cast<std::int64_t>(
				imgops::round_half_even(static_cast<double>(map_y[i]) * 32.0));
		const auto x0 = (fixed_x >> 5);
		const auto y0 = (fixed_y >> 5);
		const double tx = static_cast<double>(fixed_x & 31) / 32.0;
		const double ty = static_cast<double>(fixed_y & 31) / 32.0;
		auto &pixel = output.pixels[i];
		for (std::size_t channel = 0; channel < 3; ++channel) {
			const double value = fetch(x0, y0, channel) * (1.0 - tx) * (1.0 - ty) +
								 fetch(x0 + 1, y0, channel) * tx * (1.0 - ty) +
								 fetch(x0, y0 + 1, channel) * (1.0 - tx) * ty +
								 fetch(x0 + 1, y0 + 1, channel) * tx * ty;
			pixel[channel] = static_cast<std::uint8_t>(
					std::clamp(std::floor(value + 0.5), 0.0, 255.0));
		}
	}
	return output;
}

std::vector<double> box_blur(const std::vector<double> &source, std::size_t width,
		std::size_t height, std::size_t kernel)
{
	if (!width || !height || source.size() < width * height || !kernel)
		return {};
	const auto radius = static_cast<std::ptrdiff_t>(kernel / 2);
	const auto reflect = [](std::ptrdiff_t index, std::size_t size) {
		if (size <= 1)
			return std::size_t{0};
		const auto period = 2 * static_cast<std::ptrdiff_t>(size - 1);
		index %= period;
		if (index < 0)
			index += period;
		if (index >= static_cast<std::ptrdiff_t>(size))
			index = period - index;
		return static_cast<std::size_t>(index);
	};
	std::vector<double> temp(width * height), output(width * height);
	for (std::size_t y = 0; y < height; ++y)
		for (std::size_t x = 0; x < width; ++x)
			for (std::ptrdiff_t offset = -radius; offset <= radius; ++offset)
				temp[y * width + x] +=
						source[y * width +
								reflect(static_cast<std::ptrdiff_t>(x) + offset, width)];
	const double normalizer = static_cast<double>(kernel) * kernel;
	for (std::size_t y = 0; y < height; ++y)
		for (std::size_t x = 0; x < width; ++x) {
			double sum = 0.0;
			for (std::ptrdiff_t offset = -radius; offset <= radius; ++offset)
				sum += temp[reflect(static_cast<std::ptrdiff_t>(y) + offset, height) *
									width +
							x];
			output[y * width + x] = sum / normalizer;
		}
	return output;
}

imgops::Mask dilate_ellipse(const imgops::Mask &mask, std::int64_t radius)
{
	imgops::Mask result(mask.w, mask.h);
	for (std::int64_t dy = -radius; dy <= radius; ++dy) {
		const auto span = static_cast<std::int64_t>(imgops::round_half_even(
				std::sqrt(std::max<double>(radius * radius - dy * dy, 0.0))));
		for (std::size_t y = 0; y < mask.h; ++y)
			for (std::size_t x = 0; x < mask.w; ++x) {
				for (std::int64_t dx = -span; dx <= span; ++dx)
					if (mask.get(static_cast<std::ptrdiff_t>(x) + dx,
								static_cast<std::ptrdiff_t>(y) + dy)) {
						result.set(x, y, true);
						break;
					}
			}
	}
	return result;
}

std::vector<std::uint8_t> remap_occlusion(const std::vector<float> &map_x,
		const std::vector<float> &map_y, std::size_t width, std::size_t height,
		unsigned image_width, unsigned image_height, const sfm::DepthMap *depth,
		const Params &params, bool spherical, const std::vector<float> &ray)
{
	const auto count = width * height;
	std::vector<std::uint8_t> result(count, 0);
	std::vector<bool> outside(count, false);
	for (std::size_t i = 0; i < count; ++i) {
		outside[i] = map_x[i] < -1.0f || map_y[i] < -1.0f;
		if (spherical) {
			const double v = (static_cast<double>(map_y[i]) + 0.5) / image_height;
			if (v > params.v_range[1])
				result[i] |= OCC_NADIR;
			if (v < params.v_range[0])
				result[i] |= OCC_ZENITH;
		}
	}
	if (!spherical &&
			std::any_of(outside.begin(), outside.end(), [](bool b) { return b; })) {
		auto grown =
				imgops::dilate(imgops::Mask::from_bits(width, height, outside), 5, 5);
		for (std::size_t i = 0; i < count; ++i)
			if (grown.bits[i])
				result[i] |= OCC_OUTSIDE;
	}
	if (depth && depth->width && depth->height && image_width && image_height) {
		for (std::size_t i = 0; i < count; ++i) {
			const double u = (static_cast<double>(map_x[i]) + 0.5) / image_width;
			const double v = std::clamp(
					(static_cast<double>(map_y[i]) + 0.5) / image_height, 0.0, 1.0);
			const auto column = static_cast<unsigned>(std::clamp<std::int64_t>(
					static_cast<std::int64_t>(u * depth->width), 0, depth->width - 1));
			const auto row = static_cast<unsigned>(std::clamp<std::int64_t>(
					static_cast<std::int64_t>(v * depth->height), 0, depth->height - 1));
			const float distance = depth->at(column, row);
			if (std::isfinite(distance) && distance < ray[i] - cloud_occlusion_margin_m &&
					!outside[i])
				result[i] |= OCC_CLOUD;
		}
	}
	return result;
}

double point_segment_distance(const std::array<double, 2> &point,
		const std::array<double, 2> &a, const std::array<double, 2> &b)
{
	const double vx = b[0] - a[0], vy = b[1] - a[1];
	const double length_squared = vx * vx + vy * vy;
	const double t =
			length_squared <= 0.0
					? 0.0
					: std::clamp(((point[0] - a[0]) * vx + (point[1] - a[1]) * vy) /
										 length_squared,
							  0.0, 1.0);
	return std::hypot(point[0] - (a[0] + t * vx), point[1] - (a[1] + t * vy));
}

bool ring_contains(const std::vector<std::array<double, 2>> &ring,
		const std::array<double, 2> &point)
{
	if (ring.size() < 3)
		return false;
	bool inside = false;
	std::size_t previous = ring.size() - 1;
	for (std::size_t i = 0; i < ring.size(); ++i) {
		const auto &a = ring[i];
		const auto &b = ring[previous];
		if ((a[1] > point[1]) != (b[1] > point[1])) {
			const double x = a[0] + (point[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
			if (point[0] < x)
				inside = !inside;
		}
		previous = i;
	}
	return inside;
}

bool polygon_contains(
		const ::arnis::mapillary::Building &building, const std::array<double, 2> &point)
{
	if (!ring_contains(building.ring, point))
		return false;
	return std::none_of(building.holes.begin(), building.holes.end(),
			[&](const auto &hole) { return ring_contains(hole, point); });
}

double boundary_distance(
		const ::arnis::mapillary::Building &building, const std::array<double, 2> &point)
{
	double best = std::numeric_limits<double>::infinity();
	const auto scan = [&](const auto &ring) {
		for (std::size_t i = 0; i < ring.size(); ++i)
			best = std::min(best,
					point_segment_distance(point, ring[i], ring[(i + 1) % ring.size()]));
	};
	scan(building.ring);
	for (const auto &hole : building.holes)
		scan(hole);
	return best;
}

double signed_distance(
		const ::arnis::mapillary::Building &building, const std::array<double, 2> &point)
{
	const double distance = boundary_distance(building, point);
	return polygon_contains(building, point) ? distance : -distance;
}

double orient(const std::array<double, 2> &a, const std::array<double, 2> &b,
		const std::array<double, 2> &c)
{
	return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

bool on_segment(const std::array<double, 2> &a, const std::array<double, 2> &p,
		const std::array<double, 2> &b)
{
	return p[0] <= std::max(a[0], b[0]) && p[0] >= std::min(a[0], b[0]) &&
		   p[1] <= std::max(a[1], b[1]) && p[1] >= std::min(a[1], b[1]);
}

bool segments_cross(const std::array<double, 2> &a, const std::array<double, 2> &b,
		const std::array<double, 2> &c, const std::array<double, 2> &d)
{
	const double o1 = orient(a, b, c), o2 = orient(a, b, d);
	const double o3 = orient(c, d, a), o4 = orient(c, d, b);
	if ((o1 > 0.0) != (o2 > 0.0) && (o3 > 0.0) != (o4 > 0.0) && o1 != 0.0 && o2 != 0.0)
		return true;
	return (o1 == 0.0 && on_segment(a, c, b)) || (o2 == 0.0 && on_segment(a, d, b)) ||
		   (o3 == 0.0 && on_segment(c, a, d)) || (o4 == 0.0 && on_segment(c, b, d));
}

bool polygon_intersects_segment(const ::arnis::mapillary::Building &building,
		const std::array<double, 2> &p, const std::array<double, 2> &q)
{
	const auto intersects_ring = [&](const auto &ring) {
		for (std::size_t i = 0; i < ring.size(); ++i)
			if (segments_cross(p, q, ring[i], ring[(i + 1) % ring.size()]))
				return true;
		return false;
	};
	if (intersects_ring(building.ring))
		return true;
	for (const auto &hole : building.holes)
		if (intersects_ring(hole))
			return true;
	return polygon_contains(building, p);
}

bool eroded_meets_segment(const ::arnis::mapillary::Building &building,
		const std::array<double, 2> &p, const std::array<double, 2> &q, double inset)
{
	if (inset <= 0.0)
		return polygon_intersects_segment(building, p, q);
	const double length = std::hypot(q[0] - p[0], q[1] - p[1]);
	if (length <= 0.0)
		return signed_distance(building, p) >= inset;
	const std::array<double, 2> direction{(q[0] - p[0]) / length, (q[1] - p[1]) / length};
	for (double t = 0.0; t <= length;) {
		const std::array<double, 2> point{
				p[0] + t * direction[0], p[1] + t * direction[1]};
		const double distance = signed_distance(building, point);
		if (distance >= inset)
			return true;
		t += std::max(inset - distance, 0.05);
	}
	return signed_distance(building, q) >= inset;
}

bool bbox_overlaps(const ::arnis::mapillary::Building &building,
		const std::array<double, 2> &lo, const std::array<double, 2> &hi)
{
	if (building.ring.empty())
		return false;
	double x0 = std::numeric_limits<double>::infinity();
	double y0 = x0;
	double x1 = -x0, y1 = -x0;
	for (const auto &point : building.ring) {
		x0 = std::min(x0, point[0]);
		y0 = std::min(y0, point[1]);
		x1 = std::max(x1, point[0]);
		y1 = std::max(y1, point[1]);
	}
	return lo[0] <= x1 && hi[0] >= x0 && lo[1] <= y1 && hi[1] >= y0;
}

std::size_t components8(const imgops::Mask &mask, std::vector<std::uint32_t> &labels,
		std::vector<std::size_t> &areas)
{
	const auto count = mask.w * mask.h;
	labels.assign(count, 0);
	areas.assign(1, 0);
	std::vector<std::size_t> stack;
	for (std::size_t start = 0; start < count; ++start) {
		if (!mask.bits[start] || labels[start])
			continue;
		const auto id = static_cast<std::uint32_t>(areas.size());
		std::size_t area = 0;
		labels[start] = id;
		stack.push_back(start);
		while (!stack.empty()) {
			const auto index = stack.back();
			stack.pop_back();
			++area;
			const auto x = static_cast<std::ptrdiff_t>(index % mask.w);
			const auto y = static_cast<std::ptrdiff_t>(index / mask.w);
			for (std::ptrdiff_t dy = -1; dy <= 1; ++dy)
				for (std::ptrdiff_t dx = -1; dx <= 1; ++dx) {
					if (!dx && !dy)
						continue;
					const auto nx = x + dx, ny = y + dy;
					if (nx < 0 || ny < 0 || nx >= static_cast<std::ptrdiff_t>(mask.w) ||
							ny >= static_cast<std::ptrdiff_t>(mask.h))
						continue;
					const auto next = static_cast<std::size_t>(ny) * mask.w + nx;
					if (mask.bits[next] && !labels[next]) {
						labels[next] = id;
						stack.push_back(next);
					}
				}
		}
		areas.push_back(area);
	}
	return areas.size() - 1;
}

std::vector<bool> vegetation_mask_impl(const projection::Image &image, double ppm)
{
	const auto width = static_cast<std::size_t>(image.width);
	const auto height = static_cast<std::size_t>(image.height);
	const auto count = width * height;
	if (!count || image.pixels.size() < count)
		return {};
	std::vector<std::uint8_t> classes(count, 0);
	std::vector<double> gray(count), weight(count), weighted_gray(count),
			weighted_square(count);
	for (std::size_t i = 0; i < count; ++i) {
		const auto &pixel = image.pixels[i];
		const auto lab = imgops::srgb_to_oklab(pixel);
		const double l = lab[0], a = lab[1], b = lab[2];
		const double chroma = std::hypot(a, b);
		const double hue =
				std::fmod(std::atan2(b, a) * 57.295779513082320876 + 360.0, 360.0);
		const auto accumulator = std::int64_t(pixel[0]) * 9798 +
								 std::int64_t(pixel[1]) * 19235 +
								 std::int64_t(pixel[2]) * 3735 + 16384;
		gray[i] = accumulator >> 15;
		if (a < -0.035 && hue > veg_hue_strong[0] && hue < veg_hue_strong[1] &&
				chroma > 0.05)
			classes[i] |= 1;
		if (a < -0.025 && hue > veg_hue_weak[0] && hue < veg_hue_weak[1] &&
				chroma > 0.035)
			classes[i] |= 2;
		if (a < -0.02 && hue > veg_hue_dark[0] && hue < veg_hue_dark[1] &&
				chroma > 0.02 && l < 0.42)
			classes[i] |= 4;
		weight[i] = classes[i] != 0 ? 1.0 : 0.0;
		weighted_gray[i] = gray[i] * weight[i];
		weighted_square[i] = gray[i] * gray[i] * weight[i];
	}
	const auto denominator = box_blur(weight, width, height, 7);
	const auto mean = box_blur(weighted_gray, width, height, 7);
	const auto square = box_blur(weighted_square, width, height, 7);
	imgops::Mask hit(width, height);
	for (std::size_t i = 0; i < count; ++i) {
		if (!classes[i])
			continue;
		const double d = std::max(denominator[i], 1e-3);
		const double average = mean[i] / d;
		const double deviation =
				std::sqrt(std::max(square[i] / d - average * average, 0.0));
		hit.bits[i] = ((classes[i] & 1) && deviation > 2.0) ||
					  ((classes[i] & 2) && deviation > 8.0) ||
					  ((classes[i] & 4) && deviation > 4.0);
	}
	std::vector<std::uint8_t> bytes(count);
	for (std::size_t i = 0; i < count; ++i)
		bytes[i] = hit.bits[i] ? 255 : 0;
	const auto filtered = imgops::median_filter_u8(bytes, width, height, 7);
	imgops::Mask kept(width, height);
	for (std::size_t i = 0; i < count; ++i)
		kept.bits[i] = filtered[i] > 0;
	std::vector<std::uint32_t> labels;
	std::vector<std::size_t> areas;
	components8(kept, labels, areas);
	const double minimum_area = std::max(9.0, 0.25 * ppm * ppm);
	for (std::size_t i = 0; i < count; ++i) {
		const auto label = labels[i];
		kept.bits[i] = label != 0 && static_cast<double>(areas[label]) >= minimum_area;
	}
	const auto radius = static_cast<std::int64_t>(imgops::round_half_even(0.3 * ppm));
	if (radius > 0)
		kept = dilate_ellipse(kept, radius);
	return kept.bits;
}
} // namespace

std::vector<bool> vegetation_mask(const projection::Image &image, double pixels_per_metre)
{
	return vegetation_mask_impl(image, pixels_per_metre);
}

std::vector<bool> footprint_columns(const ::arnis::mapillary::Wall &fitted,
		const std::array<double, 3> &camera_centre,
		const std::vector<double> &wall_columns,
		const std::vector<::arnis::mapillary::Building> &buildings,
		const std::string &own_building_key, double setback_m)
{
	std::vector<bool> blocked(wall_columns.size(), false);
	if (buildings.empty() || wall_columns.empty())
		return blocked;
	const std::array<double, 2> camera{camera_centre[0], camera_centre[1]};
	std::vector<std::size_t> own;
	for (std::size_t i = 0; i < buildings.size(); ++i)
		if (buildings[i].key == own_building_key)
			own.push_back(i);
	std::vector<std::pair<std::size_t, double>> lenient;
	for (std::size_t i = 0; i < buildings.size(); ++i) {
		if (!polygon_contains(buildings[i], camera))
			continue;
		if (boundary_distance(buildings[i], camera) > los_inside_tolerance_m)
			return std::vector<bool>(wall_columns.size(), true);
		lenient.emplace_back(i, los_inside_tolerance_m + 0.05);
	}
	double sag = 0.0;
	for (const auto index : own)
		for (const auto &vertex : buildings[index].ring) {
			const auto local = fitted.sh_of({vertex[0], vertex[1], 0.0}, 0.0);
			if (local[0] >= -0.05 && local[0] <= fitted.length + 0.05 && local[2] > 0.0 &&
					local[2] < 2.0)
				sag = std::max(sag, local[2]);
		}
	const double own_inset = std::max({own_shrink_m, sag + 0.2, setback_m + 0.2});
	const auto tangent = fitted.tangent();
	for (std::size_t k = 0; k < wall_columns.size(); ++k) {
		const double s = wall_columns[k];
		const std::array<double, 2> target{
				fitted.a[0] + s * tangent[0] + sample_front_m * fitted.normal[0],
				fitted.a[1] + s * tangent[1] + sample_front_m * fitted.normal[1]};
		const std::array<double, 2> lo{
				std::min(camera[0], target[0]), std::min(camera[1], target[1])};
		const std::array<double, 2> hi{
				std::max(camera[0], target[0]), std::max(camera[1], target[1])};
		for (std::size_t i = 0; i < buildings.size(); ++i) {
			const auto &building = buildings[i];
			if (!bbox_overlaps(building, lo, hi))
				continue;
			std::optional<double> inset;
			for (const auto &[index, value] : lenient)
				if (index == i) {
					inset = value;
					break;
				}
			if (!inset && std::find(own.begin(), own.end(), i) != own.end())
				inset = own_inset;
			const bool hit =
					inset ? eroded_meets_segment(building, camera, target, *inset)
						  : polygon_intersects_segment(building, camera, target);
			if (hit) {
				blocked[k] = true;
				break;
			}
		}
	}
	return blocked;
}

LooseCrop loose_crop(const ::arnis::mapillary::Wall &wall, const View &view,
		const projection::Image &image, const sfm::DepthMap *depth,
		const std::vector<::arnis::mapillary::Building> &buildings, const Params &params)
{
	LooseCrop result;
	if (!view.camera || !view.fit || !image.width || !image.height ||
			image.pixels.size() < std::size_t(image.width) * image.height)
		return result;
	const auto &camera = *view.camera;
	const double base_z = std::isfinite(view.z_base)
								  ? view.z_base
								  : camera.centre[2] - camera.cam_height_m;
	const auto fitted = plane::fitted_wall(wall, *view.fit);
	const double osm_height = wall.height_osm.value_or(params.default_height_m);
	const auto bounds = crop_extent(
			wall, *view.fit, osm_height, params, &camera, base_z, wall.height_source);
	const auto [s_min, s_max, h_bottom, h_top] = bounds;
	const auto midpoint = fitted.midpoint();
	const double distance = view.distance_m.value_or(
			std::hypot(camera.centre[0] - midpoint[0], camera.centre[1] - midpoint[1]));
	const double ppm = choose_ppm(distance, image.width, params);
	const auto width = static_cast<std::size_t>(
			std::max<std::int64_t>(imgops::round_half_even((s_max - s_min) * ppm), 1));
	const auto height = static_cast<std::size_t>(
			std::max<std::int64_t>(imgops::round_half_even((h_top - h_bottom) * ppm), 1));
	if (width > std::numeric_limits<unsigned>::max() ||
			height > std::numeric_limits<unsigned>::max() ||
			width > std::numeric_limits<std::size_t>::max() / height)
		return result;
	const auto count = width * height;
	pose::Projector projector(camera);
	std::vector<float> map_x(count, -1e6f), map_y(count, -1e6f), ray(count, 0.0f);
	for (std::size_t y = 0; y < height; ++y) {
		const double h = h_top - (static_cast<double>(y) + 0.5) / ppm;
		for (std::size_t x = 0; x < width; ++x) {
			const double s = s_min + (static_cast<double>(x) + 0.5) / ppm;
			const auto point = fitted.point(s, h, base_z, params.eps_m);
			const auto projected = projector.project(point);
			const auto index = y * width + x;
			ray[index] = static_cast<float>(projected.length_m);
			if (camera.is_spherical()) {
				map_x[index] = static_cast<float>(std::fmod(
						std::fmod(projected.u * image.width - 0.5, image.width) +
								image.width,
						image.width));
				map_y[index] =
						static_cast<float>(std::clamp(projected.v * image.height - 0.5,
								0.0, static_cast<double>(image.height - 1)));
			} else if (projected.inside_image()) {
				map_x[index] = static_cast<float>(projected.u * image.width - 0.5);
				map_y[index] = static_cast<float>(projected.v * image.height - 0.5);
			}
		}
	}
	const bool spherical = camera.is_spherical();
	result.rgb = remap_bilinear(image, map_x, map_y, static_cast<unsigned>(width),
			static_cast<unsigned>(height), spherical);
	result.occl = remap_occlusion(map_x, map_y, width, height, image.width, image.height,
			depth, params, spherical, ray);
	std::vector<double> columns(width);
	for (std::size_t x = 0; x < width; ++x)
		columns[x] = s_min + (static_cast<double>(x) + 0.5) / ppm;
	if (!buildings.empty()) {
		const double setback =
				std::max(view.fit->d - (view.fit->normal[0] * wall.midpoint()[0] +
											   view.fit->normal[1] * wall.midpoint()[1]),
						0.0);
		const auto blocked = footprint_columns(
				fitted, camera.centre, columns, buildings, wall.building_key, setback);
		for (std::size_t y = 0; y < height; ++y)
			for (std::size_t x = 0; x < width; ++x)
				if (blocked[x])
					result.occl[y * width + x] |= OCC_FOOTPRINT;
	} else if (view.visible_s) {
		for (std::size_t y = 0; y < height; ++y)
			for (std::size_t x = 0; x < width; ++x)
				if (columns[x] < (*view.visible_s)[0] ||
						columns[x] > (*view.visible_s)[1])
					result.occl[y * width + x] |= OCC_FOOTPRINT;
	}
	const auto vegetation = vegetation_mask(result.rgb, ppm);
	for (std::size_t i = 0; i < vegetation.size(); ++i)
		if (vegetation[i])
			result.occl[i] |= OCC_SEG;
	const auto camera_local = fitted.sh_of(camera.centre, base_z);
	result.wall_key = wall.key;
	result.pano_id = camera.pano_id;
	result.ppm = ppm;
	result.s0 = s_min;
	result.h_bot = h_bottom;
	result.h_top = h_top;
	result.x_foot = (camera_local[0] - s_min) * ppm;
	result.y_cam = (h_top - camera_local[1]) * ppm;
	result.z_base = base_z;
	result.z_base_source = "pano";
	return result;
}

RectView resample_rect(const ::arnis::mapillary::Wall &wall, const View &view,
		const projection::Image &image, const std::array<double, 4> &rectangle,
		const Matrix3 *shear, unsigned pixels_per_block, const LooseCrop *crop,
		const Params &params)
{
	RectView result;
	if (!view.camera || !view.fit || !pixels_per_block || !image.width || !image.height)
		return result;
	const auto [s_left, s_right, h_low, h_high] = rectangle;
	const auto columns = static_cast<std::size_t>(
			std::max<std::int64_t>(imgops::round_half_even(s_right - s_left), 1));
	const auto rows = static_cast<std::size_t>(
			std::max<std::int64_t>(imgops::round_half_even(h_high - h_low), 1));
	if (columns > std::numeric_limits<unsigned>::max() / pixels_per_block ||
			rows > std::numeric_limits<unsigned>::max() / pixels_per_block)
		return result;
	const auto width = static_cast<unsigned>(columns * pixels_per_block);
	const auto height = static_cast<unsigned>(rows * pixels_per_block);
	const auto count = std::size_t(width) * height;
	const auto fitted = plane::fitted_wall(wall, *view.fit);
	const auto inverse = shear ? std::optional<Matrix3>(inverse3(*shear)) : std::nullopt;
	std::vector<std::array<double, 2>> raw;
	std::vector<std::array<double, 3>> world;
	raw.reserve(count);
	world.reserve(count);
	for (unsigned y = 0; y < height; ++y) {
		const double h =
				h_high - (static_cast<double>(y) + 0.5) * (h_high - h_low) / height;
		for (unsigned x = 0; x < width; ++x) {
			const double s =
					s_left + (static_cast<double>(x) + 0.5) * (s_right - s_left) / width;
			const auto source =
					inverse ? apply_h(*inverse, s, h) : std::array<double, 2>{s, h};
			raw.push_back(source);
			world.push_back(
					fitted.point(source[0], source[1], view.z_base, params.eps_m));
		}
	}
	pose::Projector projector(*view.camera);
	std::vector<float> map_x(count, -1.0f), map_y(count, -1.0f);
	for (std::size_t i = 0; i < count; ++i)
		if (const auto pixel = projector.pixel(world[i])) {
			map_x[i] = (*pixel)[0];
			map_y[i] = (*pixel)[1];
		} else {
			// Rust writes a far-out map coordinate so all four constant-border
			// interpolation taps are black; -1 would blend the image edge in.
			map_x[i] = -1e6f;
			map_y[i] = -1e6f;
		}
	result.rgb = remap_bilinear(
			image, map_x, map_y, width, height, view.camera->is_spherical());
	result.valid.assign(count, false);
	for (std::size_t i = 0; i < count; ++i) {
		if (view.camera->is_spherical()) {
			const double v = (static_cast<double>(map_y[i]) + 0.5) / image.height;
			result.valid[i] = v >= params.v_range[0] && v <= params.v_range[1];
		} else {
			result.valid[i] = map_x[i] > -1.0f && map_y[i] > -1.0f;
		}
	}
	if (crop && crop->rgb.width && crop->rgb.height &&
			crop->occl.size() >= std::size_t(crop->rgb.width) * crop->rgb.height) {
		for (std::size_t i = 0; i < count; ++i) {
			if (!result.valid[i])
				continue;
			const double x = crop->s_to_x(raw[i][0]);
			const double y = crop->h_to_y(raw[i][1]);
			const bool inside =
					x >= 0.0 && x < crop->rgb.width && y >= 0.0 && y < crop->rgb.height;
			const auto column = static_cast<std::size_t>(
					std::clamp(static_cast<std::int64_t>(std::floor(x)), std::int64_t{0},
							static_cast<std::int64_t>(crop->rgb.width) - 1));
			const auto row = static_cast<std::size_t>(
					std::clamp(static_cast<std::int64_t>(std::floor(y)), std::int64_t{0},
							static_cast<std::int64_t>(crop->rgb.height) - 1));
			const auto bits = crop->occl[row * crop->rgb.width + column];
			constexpr std::uint8_t occluded = OCC_FOOTPRINT | OCC_CLOUD | OCC_NADIR |
											  OCC_ZENITH | OCC_SEG | OCC_OUTSIDE;
			result.valid[i] = inside && (bits & occluded) == 0;
		}
	}
	return result;
}
} // namespace arnis::mapillary::rectify
