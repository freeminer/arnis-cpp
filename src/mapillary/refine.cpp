#include "refine.h"

#include "refine_stats.h"
#include "imgops.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>

namespace arnis::mapillary::refine
{
namespace
{
constexpr double peak_min_energy = 3.0;
constexpr double peak_min_vertical_length_m = 1.5;
constexpr double peak_min_separation_m = 0.5;
constexpr double profile_sigma_m = 0.2;
constexpr double window_limit_m = 0.3;
constexpr double huber_degrees = 1.0;
constexpr double lean_inlier_degrees = 1.5;
constexpr std::size_t irls_iterations = 10;
constexpr double minimum_lean_x_spread_m = 1.5;
constexpr double gradient_max_degrees = 12.0;
constexpr double gradient_bin_degrees = 0.25;
constexpr double lsd_scale = 0.8;
constexpr double lsd_sigma_scale = 0.6;
constexpr double lsd_quant = 2.0;
constexpr double lsd_angle_threshold = 22.5;
constexpr double lsd_density_threshold = 0.7;
constexpr std::size_t lsd_bins = 1024;
constexpr double not_defined_angle = -1024.0;
constexpr double minimum_segment_metres = 0.6;
constexpr double vertical_family_degrees = 10.0;
constexpr double horizontal_family_degrees = 15.0;
constexpr double horizontal_minimum_metres = 1.0;
constexpr float sky_sobel_max = 25.0f;
constexpr std::array<int, 2> sky_hue_range{90, 135};
constexpr double sky_top_fraction = 0.15;
constexpr double sky_rule_min = 0.05;
constexpr double sky_rule_max = 0.60;
constexpr double edge_energy_floor = 3.0;
constexpr double ground_window_metres = 0.75;
constexpr double ground_edge_ratio = 3.0;
constexpr double roof_min_reach = 0.5;
constexpr double roof_spread_fraction = 0.15;
constexpr std::array<double, 2> roof_edge_window{0.8, 1.6};
constexpr double roof_edge_ratio = 2.0;
constexpr std::array<double, 2> roof_default_range{3.0, 80.0};
constexpr double roof_behind_metres = 2.0;
constexpr double roof_nodata_band_metres = 1.0;
constexpr double roof_nodata_max = 0.5;
constexpr double colour_max_delta_e = 12.0;
constexpr double rhythm_min_peak = 0.3;
constexpr double rhythm_max_diff = 0.2;
constexpr std::array<double, 2> rhythm_range_m{0.8, 6.0};
constexpr std::uint8_t occluded_bits =
		OCC_FOOTPRINT | OCC_CLOUD | OCC_NADIR | OCC_ZENITH | OCC_SEG | OCC_OUTSIDE;
constexpr std::array<std::array<double, 3>, 3> identity3{
		{{{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}}}};

double interp(double x, const std::vector<double> &xs, const std::vector<double> &ys)
{
	if (xs.empty())
		return std::numeric_limits<double>::quiet_NaN();
	if (x <= xs.front())
		return ys.front();
	if (x >= xs.back())
		return ys.back();
	std::size_t lo = 0, hi = xs.size() - 1;
	while (hi - lo > 1) {
		const auto mid = (lo + hi) / 2;
		if (xs[mid] <= x)
			lo = mid;
		else
			hi = mid;
	}
	const double t = (x - xs[lo]) / std::max(xs[hi] - xs[lo], 1e-300);
	return ys[lo] + t * (ys[hi] - ys[lo]);
}

double weighted_mean(
		const std::vector<double> &values, const std::vector<double> &weights)
{
	const double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
	if (sum <= 0.0) {
		if (values.empty())
			return 0.0;
		return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
	}
	double weighted = 0.0;
	for (std::size_t i = 0; i < values.size(); ++i)
		weighted += values[i] * weights[i];
	return weighted / sum;
}

struct IrlsLine
{
	double c0{}, c1{};
	std::vector<double> residuals;
};

struct RegionPoint
{
	int x{}, y{};
	double angle{}, gradient{};
};

struct LsdRectangle
{
	double x1{}, y1{}, x2{}, y2{}, width{}, x{}, y{}, theta{}, dx{}, dy{}, precision{},
			p{};
};

struct AngleField
{
	std::size_t width{}, height{};
	std::vector<double> angles, modgrad;
	bool aligned(int x, int y, double theta, double precision) const
	{
		if (x < 0 || y < 0 || static_cast<std::size_t>(x) >= width ||
				static_cast<std::size_t>(y) >= height)
			return false;
		const double angle = angles[static_cast<std::size_t>(y) * width + x];
		if (angle == not_defined_angle)
			return false;
		double difference = std::abs(theta - angle);
		if (difference > 1.5 * 3.14159265358979323846)
			difference = std::abs(difference - 2.0 * 3.14159265358979323846);
		return difference <= precision;
	}
};

IrlsLine irls_line(const std::vector<double> &x, const std::vector<double> &y,
		const std::vector<double> &initial_weights, bool fit_slope)
{
	std::vector<double> weights = initial_weights;
	double c0 = std::accumulate(weights.begin(), weights.end(), 0.0) > 0.0
						? weighted_mean(y, weights)
						: 0.0;
	double c1 = 0.0;
	for (std::size_t iteration = 0; iteration < irls_iterations; ++iteration) {
		if (fit_slope) {
			double a00 = 0.0, a01 = 0.0, a11 = 0.0, b0 = 0.0, b1 = 0.0;
			for (std::size_t i = 0; i < x.size(); ++i) {
				a00 += weights[i];
				a01 += weights[i] * x[i];
				a11 += weights[i] * x[i] * x[i];
				b0 += weights[i] * y[i];
				b1 += weights[i] * x[i] * y[i];
			}
			a00 += 1e-9;
			a11 += 1e-9;
			const double determinant = a00 * a11 - a01 * a01;
			if (std::abs(determinant) < 1e-300)
				break;
			c0 = (b0 * a11 - b1 * a01) / determinant;
			c1 = (a00 * b1 - a01 * b0) / determinant;
		} else {
			const double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
			c0 = 0.0;
			for (std::size_t i = 0; i < y.size(); ++i)
				c0 += weights[i] * y[i];
			c0 /= std::max(sum, 1e-12);
		}
		for (std::size_t i = 0; i < x.size(); ++i) {
			const double residual = y[i] - (c0 + c1 * x[i]);
			const double huber =
					std::abs(residual) <= huber_degrees
							? 1.0
							: huber_degrees / std::max(std::abs(residual), 1e-9);
			weights[i] = initial_weights[i] * huber;
		}
	}
	IrlsLine result{c0, c1, std::vector<double>(x.size())};
	for (std::size_t i = 0; i < x.size(); ++i)
		result.residuals[i] = y[i] - (c0 + c1 * x[i]);
	return result;
}

std::vector<double> convolve_same(
		const std::vector<double> &values, const std::vector<double> &kernel)
{
	std::vector<double> output(values.size(), 0.0);
	if (kernel.empty())
		return output;
	const auto offset = static_cast<std::ptrdiff_t>((kernel.size() - 1) / 2);
	for (std::size_t i = 0; i < values.size(); ++i)
		for (std::size_t j = 0; j < kernel.size(); ++j) {
			const auto index = static_cast<std::ptrdiff_t>(i) + offset -
							   static_cast<std::ptrdiff_t>(j);
			if (index >= 0 && static_cast<std::size_t>(index) < values.size())
				output[i] += values[static_cast<std::size_t>(index)] * kernel[j];
		}
	return output;
}

std::size_t argmax_first(const std::vector<double> &values)
{
	double best = -std::numeric_limits<double>::infinity();
	std::size_t index = 0;
	for (std::size_t i = 0; i < values.size(); ++i)
		if (values[i] > best) {
			best = values[i];
			index = i;
		}
	return index;
}

std::size_t reflect101(std::ptrdiff_t i, std::size_t length)
{
	if (length <= 1)
		return 0;
	const auto n = static_cast<std::ptrdiff_t>(length);
	while (i < 0 || i >= n) {
		if (i < 0)
			i = -i;
		else
			i = 2 * n - 2 - i;
	}
	return static_cast<std::size_t>(i);
}

std::vector<std::uint8_t> to_gray(const projection::Image &image)
{
	std::vector<std::uint8_t> result;
	result.reserve(image.pixels.size());
	for (const auto &[r, g, b] : image.pixels)
		result.push_back(static_cast<std::uint8_t>(
				(std::uint32_t(r) * 4899 + std::uint32_t(g) * 9617 +
						std::uint32_t(b) * 1868 + 8192) >>
				14));
	return result;
}

std::vector<float> sobel(const std::vector<float> &source, std::size_t width,
		std::size_t height, bool along_x)
{
	if (!width || !height || source.size() < width * height)
		return {};
	constexpr std::array<float, 3> derivative{-1.0f, 0.0f, 1.0f};
	constexpr std::array<float, 3> smooth{1.0f, 2.0f, 1.0f};
	const auto &kx = along_x ? derivative : smooth;
	const auto &ky = along_x ? smooth : derivative;
	std::vector<float> temp(width * height), output(width * height);
	for (std::size_t y = 0; y < height; ++y)
		for (std::size_t x = 0; x < width; ++x) {
			float acc = 0.0f;
			for (std::size_t t = 0; t < 3; ++t)
				acc += source[y * width +
							   reflect101(static_cast<std::ptrdiff_t>(x) +
												  static_cast<std::ptrdiff_t>(t) - 1,
									   width)] *
					   kx[t];
			temp[y * width + x] = acc;
		}
	for (std::size_t y = 0; y < height; ++y)
		for (std::size_t x = 0; x < width; ++x) {
			float acc = 0.0f;
			for (std::size_t t = 0; t < 3; ++t)
				acc += temp[reflect101(static_cast<std::ptrdiff_t>(y) +
											   static_cast<std::ptrdiff_t>(t) - 1,
									height) *
									   width +
							   x] *
					   ky[t];
			output[y * width + x] = acc;
		}
	return output;
}

std::vector<double> gaussian_kernel(std::size_t size, double sigma)
{
	static const std::array<std::vector<double>, 4> small{
			{{1.0}, {0.25, 0.5, 0.25}, {0.0625, 0.25, 0.375, 0.25, 0.0625},
					{0.03125, 0.109375, 0.21875, 0.28125, 0.21875, 0.109375, 0.03125}}};
	if (sigma <= 0.0 && size <= 7 && size % 2 == 1)
		return small[size / 2];
	if (sigma <= 0.0)
		sigma = 0.3 * ((static_cast<double>(size) - 1.0) * 0.5 - 1.0) + 0.8;
	const double scale = -0.5 / (sigma * sigma);
	const double center = (static_cast<double>(size) - 1.0) * 0.5;
	std::vector<double> kernel(size);
	for (std::size_t i = 0; i < size; ++i) {
		const double x = static_cast<double>(i) - center;
		kernel[i] = std::exp(scale * x * x);
	}
	const double sum = std::accumulate(kernel.begin(), kernel.end(), 0.0);
	for (auto &value : kernel)
		value /= sum;
	return kernel;
}

std::size_t gaussian_ksize(double sigma)
{
	const auto n = static_cast<std::int64_t>(imgops::round_half_even(sigma * 8.0 + 1.0));
	return static_cast<std::size_t>(std::max<std::int64_t>(n | 1, 1));
}

std::vector<float> gaussian_blur(const std::vector<float> &source, std::size_t width,
		std::size_t height, std::size_t size, double sigma)
{
	if (!width || !height || source.size() < width * height || !size)
		return {};
	const auto kernel = gaussian_kernel(size, sigma);
	const auto radius = static_cast<std::ptrdiff_t>(size / 2);
	std::vector<float> temp(width * height), output(width * height);
	for (std::size_t y = 0; y < height; ++y)
		for (std::size_t x = 0; x < width; ++x) {
			double sum = 0.0;
			for (std::size_t t = 0; t < kernel.size(); ++t)
				sum += source[y * width +
							   reflect101(static_cast<std::ptrdiff_t>(x) +
												  static_cast<std::ptrdiff_t>(t) - radius,
									   width)] *
					   kernel[t];
			temp[y * width + x] = static_cast<float>(sum);
		}
	for (std::size_t y = 0; y < height; ++y)
		for (std::size_t x = 0; x < width; ++x) {
			double sum = 0.0;
			for (std::size_t t = 0; t < kernel.size(); ++t)
				sum += temp[reflect101(static_cast<std::ptrdiff_t>(y) +
											   static_cast<std::ptrdiff_t>(t) - radius,
									height) *
									   width +
							   x] *
					   kernel[t];
			output[y * width + x] = static_cast<float>(sum);
		}
	return output;
}

std::vector<double> gaussian_blur_1d(const std::vector<double> &source, double sigma)
{
	if (sigma <= 0.0 || source.size() < 2)
		return source;
	const auto size = gaussian_ksize(sigma);
	const auto kernel = gaussian_kernel(size, sigma);
	const auto radius = static_cast<std::ptrdiff_t>(size / 2);
	std::vector<double> result(source.size());
	for (std::size_t x = 0; x < source.size(); ++x)
		for (std::size_t t = 0; t < kernel.size(); ++t)
			result[x] +=
					source[reflect101(static_cast<std::ptrdiff_t>(x) +
											  static_cast<std::ptrdiff_t>(t) - radius,
							source.size())] *
					kernel[t];
	return result;
}

std::vector<std::array<std::uint8_t, 3>> rgb_to_hsv(const projection::Image &image)
{
	constexpr int shift = 12;
	std::array<std::int64_t, 256> sdiv{}, hdiv{};
	for (std::size_t i = 1; i < 256; ++i) {
		sdiv[i] = static_cast<std::int64_t>(std::llround((255LL << shift) / double(i)));
		hdiv[i] = static_cast<std::int64_t>(std::llround((180LL << shift) / (6.0 * i)));
	}
	std::vector<std::array<std::uint8_t, 3>> output;
	output.reserve(image.pixels.size());
	for (const auto &[r8, g8, b8] : image.pixels) {
		const auto r = static_cast<std::int64_t>(r8);
		const auto g = static_cast<std::int64_t>(g8);
		const auto b = static_cast<std::int64_t>(b8);
		const auto value = std::max({r, g, b});
		const auto minimum = std::min({r, g, b});
		const auto diff = value - minimum;
		const auto saturation = (diff * sdiv[value] + (1 << (shift - 1))) >> shift;
		auto hue = value == r ? g - b : value == g ? b - r + 2 * diff : r - g + 4 * diff;
		hue = (hue * hdiv[diff] + (1 << (shift - 1))) >> shift;
		if (hue < 0)
			hue += 180;
		output.push_back({static_cast<std::uint8_t>(
								  std::clamp<std::int64_t>(hue, 0, 255)),
				static_cast<std::uint8_t>(std::clamp<std::int64_t>(saturation, 0, 255)),
				static_cast<std::uint8_t>(value)});
	}
	return output;
}

double otsu_threshold(const std::array<std::size_t, 256> &histogram)
{
	const auto total =
			std::accumulate(histogram.begin(), histogram.end(), std::size_t{0});
	if (!total)
		return 0.0;
	const double inv = 1.0 / total;
	double mean = 0.0;
	for (std::size_t i = 0; i < histogram.size(); ++i)
		mean += i * (histogram[i] * inv);
	double q1 = 0.0, mean1_acc = 0.0, max_sigma = 0.0, max_value = 0.0;
	for (std::size_t i = 0; i < histogram.size(); ++i) {
		const double probability = histogram[i] * inv;
		mean1_acc += i * probability;
		q1 += probability;
		const double q2 = 1.0 - q1;
		if (q1 < std::numeric_limits<double>::epsilon() ||
				q2 < std::numeric_limits<double>::epsilon())
			continue;
		const double mean1 = mean1_acc / q1;
		const double mean2 = (mean - q1 * mean1) / q2;
		const double sigma = q1 * q2 * (mean1 - mean2) * (mean1 - mean2);
		if (sigma > max_sigma) {
			max_sigma = sigma;
			max_value = i;
		}
	}
	return max_value;
}

std::array<std::size_t, 256> histogram(const std::vector<std::uint8_t> &values)
{
	std::array<std::size_t, 256> result{};
	for (const auto value : values)
		++result[value];
	return result;
}

std::vector<std::uint8_t> resize_bilinear(const std::vector<std::uint8_t> &source,
		std::size_t width, std::size_t height, std::size_t out_width,
		std::size_t out_height, double scale)
{
	if (!width || !height || !out_width || !out_height || source.size() < width * height)
		return {};
	std::vector<std::uint8_t> output(out_width * out_height);
	for (std::size_t y = 0; y < out_height; ++y) {
		const double fy = std::max((static_cast<double>(y) + 0.5) * scale - 0.5, 0.0);
		const auto y0 = std::min(static_cast<std::size_t>(std::floor(fy)), height - 1);
		const auto y1 = std::min(y0 + 1, height - 1);
		const double ty = fy - y0;
		for (std::size_t x = 0; x < out_width; ++x) {
			const double fx = std::max((static_cast<double>(x) + 0.5) * scale - 0.5, 0.0);
			const auto x0 = std::min(static_cast<std::size_t>(std::floor(fx)), width - 1);
			const auto x1 = std::min(x0 + 1, width - 1);
			const double tx = fx - x0;
			const double top =
					source[y0 * width + x0] * (1.0 - tx) + source[y0 * width + x1] * tx;
			const double bottom =
					source[y1 * width + x0] * (1.0 - tx) + source[y1 * width + x1] * tx;
			output[y * out_width + x] = static_cast<std::uint8_t>(
					std::clamp(std::round(top * (1.0 - ty) + bottom * ty), 0.0, 255.0));
		}
	}
	return output;
}

std::array<double, 3> multiply3(
		const std::array<std::array<double, 3>, 3> &a, const std::array<double, 3> &b)
{
	std::array<double, 3> out{};
	for (std::size_t i = 0; i < 3; ++i)
		for (std::size_t j = 0; j < 3; ++j)
			out[i] += a[i][j] * b[j];
	return out;
}

std::array<std::array<double, 3>, 3> inverse3(
		const std::array<std::array<double, 3>, 3> &m)
{
	const double determinant = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
							   m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
							   m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
	const double d = std::abs(determinant) < 1e-300 ? 1e-300 : determinant;
	std::array<std::array<double, 3>, 3> out{};
	out[0] = {(m[1][1] * m[2][2] - m[1][2] * m[2][1]) / d,
			(m[0][2] * m[2][1] - m[0][1] * m[2][2]) / d,
			(m[0][1] * m[1][2] - m[0][2] * m[1][1]) / d};
	out[1] = {(m[1][2] * m[2][0] - m[1][0] * m[2][2]) / d,
			(m[0][0] * m[2][2] - m[0][2] * m[2][0]) / d,
			(m[0][2] * m[1][0] - m[0][0] * m[1][2]) / d};
	out[2] = {(m[1][0] * m[2][1] - m[1][1] * m[2][0]) / d,
			(m[0][1] * m[2][0] - m[0][0] * m[2][1]) / d,
			(m[0][0] * m[1][1] - m[0][1] * m[1][0]) / d};
	return out;
}

std::array<std::array<double, 3>, 3> perspective_transform(
		const std::array<std::array<double, 2>, 4> &src,
		const std::array<std::array<double, 2>, 4> &dst)
{
	std::array<std::array<double, 9>, 8> a{};
	for (std::size_t i = 0; i < 4; ++i) {
		const auto [x, y] = src[i];
		const auto [u, v] = dst[i];
		a[i] = {x, y, 1.0, 0.0, 0.0, 0.0, -x * u, -y * u, u};
		a[i + 4] = {0.0, 0.0, 0.0, x, y, 1.0, -x * v, -y * v, v};
	}
	for (std::size_t col = 0; col < 8; ++col) {
		std::size_t pivot = col;
		for (std::size_t row = col + 1; row < 8; ++row)
			if (std::abs(a[row][col]) > std::abs(a[pivot][col]))
				pivot = row;
		std::swap(a[col], a[pivot]);
		const double divisor = a[col][col];
		if (std::abs(divisor) < 1e-300)
			return identity3;
		for (std::size_t c = col; c < 9; ++c)
			a[col][c] /= divisor;
		for (std::size_t row = 0; row < 8; ++row) {
			if (row == col)
				continue;
			const double factor = a[row][col];
			if (factor == 0.0)
				continue;
			for (std::size_t c = col; c < 9; ++c)
				a[row][c] -= factor * a[col][c];
		}
	}
	return {{{a[0][8], a[1][8], a[2][8]}, {a[3][8], a[4][8], a[5][8]},
			{a[6][8], a[7][8], 1.0}}};
}

std::array<double, 2> apply_h(
		const std::array<std::array<double, 3>, 3> &h, double x, double y)
{
	double w = h[2][0] * x + h[2][1] * y + h[2][2];
	if (std::abs(w) < 1e-12)
		w = 1e-12;
	return {(h[0][0] * x + h[0][1] * y + h[0][2]) / w,
			(h[1][0] * x + h[1][1] * y + h[1][2]) / w};
}

std::array<std::array<double, 3>, 3> multiply_matrix(
		const std::array<std::array<double, 3>, 3> &a,
		const std::array<std::array<double, 3>, 3> &b)
{
	std::array<std::array<double, 3>, 3> output{};
	for (std::size_t i = 0; i < 3; ++i)
		for (std::size_t j = 0; j < 3; ++j)
			for (std::size_t k = 0; k < 3; ++k)
				output[i][j] += a[i][k] * b[k][j];
	return output;
}

std::array<std::array<double, 3>, 3> shear_homography(
		const rectify::LooseCrop &crop, double c0_degrees, double c1_degrees_per_metre)
{
	const double s_camera = crop.x_to_s(crop.x_foot);
	const double h_camera = crop.y_to_h(crop.y_cam);
	const double s_a = crop.s0, s_b = crop.x_to_s(crop.rgb.width);
	const std::array<std::array<double, 2>, 4> source{{{{s_a, crop.h_bot}},
			{{s_b, crop.h_bot}}, {{s_b, crop.h_top}}, {{s_a, crop.h_top}}}};
	auto destination = source;
	for (std::size_t i = 0; i < source.size(); ++i) {
		const auto [s, h] = source[i];
		const double lean = (c0_degrees + c1_degrees_per_metre * (s - s_camera)) *
							(3.14159265358979323846 / 180.0);
		destination[i][0] = s - std::tan(lean) * (h - h_camera);
	}
	return perspective_transform(source, destination);
}

std::pair<projection::Image, std::vector<std::uint8_t>> warp_crop(
		const rectify::LooseCrop &crop,
		const std::array<std::array<double, 3>, 3> &metric_homography)
{
	const std::array<std::array<double, 3>, 3> to_pixel{
			{{{crop.ppm, 0.0, -crop.s0 * crop.ppm}},
					{{0.0, -crop.ppm, crop.h_top * crop.ppm}}, {{0.0, 0.0, 1.0}}}};
	const auto pixel_h = multiply_matrix(
			multiply_matrix(to_pixel, metric_homography), inverse3(to_pixel));
	const auto inverse = inverse3(pixel_h);
	const auto width = crop.rgb.width, height = crop.rgb.height;
	projection::Image warped;
	warped.width = width;
	warped.height = height;
	warped.pixels.resize(std::size_t(width) * height);
	std::vector<std::uint8_t> occlusion(std::size_t(width) * height);
	if (!width || !height || crop.rgb.pixels.size() < std::size_t(width) * height)
		return {std::move(warped), std::move(occlusion)};
	for (unsigned y = 0; y < height; ++y)
		for (unsigned x = 0; x < width; ++x) {
			const auto [sx, sy] = apply_h(inverse, x, y);
			const double cx = std::clamp(sx, 0.0, static_cast<double>(width - 1));
			const double cy = std::clamp(sy, 0.0, static_cast<double>(height - 1));
			const auto x0 = static_cast<unsigned>(std::floor(cx));
			const auto y0 = static_cast<unsigned>(std::floor(cy));
			const auto x1 = std::min(x0 + 1, width - 1);
			const auto y1 = std::min(y0 + 1, height - 1);
			const double tx = cx - x0, ty = cy - y0;
			projection::Rgb pixel{};
			for (std::size_t channel = 0; channel < 3; ++channel) {
				const double top =
						crop.rgb.pixels[std::size_t(y0) * width + x0][channel] *
								(1.0 - tx) +
						crop.rgb.pixels[std::size_t(y0) * width + x1][channel] * tx;
				const double bottom =
						crop.rgb.pixels[std::size_t(y1) * width + x0][channel] *
								(1.0 - tx) +
						crop.rgb.pixels[std::size_t(y1) * width + x1][channel] * tx;
				pixel[channel] = static_cast<std::uint8_t>(std::clamp(
						std::round(top * (1.0 - ty) + bottom * ty), 0.0, 255.0));
			}
			const auto index = std::size_t(y) * width + x;
			warped.pixels[index] = pixel;
			const auto nx = static_cast<unsigned>(
					std::clamp(std::round(cx), 0.0, static_cast<double>(width - 1)));
			const auto ny = static_cast<unsigned>(
					std::clamp(std::round(cy), 0.0, static_cast<double>(height - 1)));
			const auto nearest_index = std::size_t(ny) * width + nx;
			if (nearest_index < crop.occl.size())
				occlusion[index] = crop.occl[nearest_index];
		}
	return {std::move(warped), std::move(occlusion)};
}

double srgb_linear(double value)
{
	return value <= 0.04045 ? value / 12.92 : std::pow((value + 0.055) / 1.055, 2.4);
}

double lab_curve(double value)
{
	return value > 0.008856 ? std::cbrt(value) : 7.787 * value + 16.0 / 116.0;
}

std::array<double, 3> rgb_to_lab(const projection::Rgb &pixel)
{
	const double r = srgb_linear(pixel[0] / 255.0);
	const double g = srgb_linear(pixel[1] / 255.0);
	const double b = srgb_linear(pixel[2] / 255.0);
	const double x = (0.412453 * r + 0.357580 * g + 0.180423 * b) / 0.950456;
	const double y = 0.212671 * r + 0.715160 * g + 0.072169 * b;
	const double z = (0.019334 * r + 0.119193 * g + 0.950227 * b) / 1.088754;
	const double fx = lab_curve(x), fy = lab_curve(y), fz = lab_curve(z);
	const double l = y > 0.008856 ? 116.0 * fy - 16.0 : 903.3 * y;
	const double a = 500.0 * (fx - fy);
	const double bb = 200.0 * (fy - fz);
	const auto quantize = [](double v) {
		return static_cast<double>(std::clamp(imgops::round_half_even(v), 0.0, 255.0));
	};
	return {quantize(l * 255.0 / 100.0) * 100.0 / 255.0, quantize(a + 128.0) - 128.0,
			quantize(bb + 128.0) - 128.0};
}

std::vector<std::array<double, 3>> column_lab(const projection::Image &image,
		const std::vector<std::uint8_t> *occlusion, std::size_t x0, std::size_t x1)
{
	const auto width = static_cast<std::size_t>(image.width);
	const auto height = static_cast<std::size_t>(image.height);
	if (x1 <= x0 || x1 > width || image.pixels.size() < width * height)
		return {};
	const auto columns = x1 - x0;
	std::vector<std::array<double, 3>> result(columns, {0.0, 0.0, 0.0});
	std::vector<double> counts(columns, 0.0);
	for (std::size_t y = 0; y < height; ++y) {
		for (std::size_t col = 0; col < columns; ++col) {
			const auto index = y * width + x0 + col;
			if (occlusion && index < occlusion->size() &&
					((*occlusion)[index] & occluded_bits) != 0)
				continue;
			const auto lab = rgb_to_lab(image.pixels[index]);
			for (std::size_t c = 0; c < 3; ++c)
				result[col][c] += lab[c];
			counts[col] += 1.0;
		}
	}
	for (std::size_t col = 0; col < columns; ++col)
		for (auto &channel : result[col])
			channel /= std::max(counts[col], 1e-6);
	std::vector<std::size_t> good;
	for (std::size_t col = 0; col < columns; ++col)
		if (counts[col] >= 1.0)
			good.push_back(col);
	if (!good.empty() && good.size() < columns) {
		for (std::size_t col = 0; col < columns; ++col) {
			if (counts[col] >= 1.0)
				continue;
			auto hi = std::lower_bound(good.begin(), good.end(), col);
			if (hi == good.begin()) {
				result[col] = result[*hi];
			} else if (hi == good.end()) {
				result[col] = result[good.back()];
			} else {
				const auto right = *hi, left = *(hi - 1);
				const double t = static_cast<double>(col - left) / (right - left);
				for (std::size_t c = 0; c < 3; ++c)
					result[col][c] =
							result[left][c] + t * (result[right][c] - result[left][c]);
			}
		}
	}
	return result;
}

std::array<double, 3> median_lab(const std::vector<std::array<double, 3>> &samples,
		std::size_t begin, std::size_t end)
{
	std::array<double, 3> result{};
	for (std::size_t channel = 0; channel < 3; ++channel) {
		std::vector<double> values;
		for (std::size_t i = begin; i < end; ++i)
			values.push_back(samples[i][channel]);
		result[channel] = imgops::median(std::move(values));
	}
	return result;
}

double lab_distance(const std::array<double, 3> &a, const std::array<double, 3> &b)
{
	const double dl = a[0] - b[0], da = a[1] - b[1], db = a[2] - b[2];
	return std::sqrt(dl * dl + da * da + db * db);
}

std::optional<double> autocorrelation_period(
		const std::vector<double> &signal, double pixels_per_metre)
{
	const auto n = signal.size();
	if (n < 4)
		return std::nullopt;
	const double mean = std::accumulate(signal.begin(), signal.end(), 0.0) / n;
	std::vector<double> centered(n);
	std::transform(signal.begin(), signal.end(), centered.begin(),
			[&](double value) { return value - mean; });
	double variance = 0.0;
	for (const double value : centered)
		variance += value * value;
	variance /= n;
	if (std::sqrt(variance) < 1e-6)
		return std::nullopt;
	std::vector<double> autocorrelation(n, 0.0);
	for (std::size_t lag = 0; lag < n; ++lag)
		for (std::size_t i = 0; i < n - lag; ++i)
			autocorrelation[lag] += centered[i] * centered[i + lag];
	const double a0 = std::max(autocorrelation[0], 1e-9);
	for (auto &value : autocorrelation)
		value /= a0;
	const auto lo = static_cast<std::size_t>(rhythm_range_m[0] * pixels_per_metre);
	const auto hi = std::min(
			static_cast<std::size_t>(rhythm_range_m[1] * pixels_per_metre), n - 1);
	if (hi <= lo + 2)
		return std::nullopt;
	std::size_t i = lo + 1;
	while (i < hi) {
		if (static_cast<float>(autocorrelation[i - 1]) <
				static_cast<float>(autocorrelation[i])) {
			std::size_t j = i;
			while (j + 1 < hi && static_cast<float>(autocorrelation[j + 1]) ==
										 static_cast<float>(autocorrelation[i]))
				++j;
			if (j + 1 < hi && static_cast<float>(autocorrelation[j + 1]) <
									  static_cast<float>(autocorrelation[i])) {
				const auto peak = (i + j) / 2;
				if (static_cast<float>(autocorrelation[peak]) >= rhythm_min_peak)
					return static_cast<double>(peak) / pixels_per_metre;
			}
			i = j + 1;
		} else {
			++i;
		}
	}
	return std::nullopt;
}

std::pair<bool, double> half_consistency(const projection::Image &image,
		const std::vector<std::uint8_t> *occlusion, std::ptrdiff_t c0, std::ptrdiff_t c1,
		double ppm)
{
	const auto width = static_cast<std::ptrdiff_t>(image.width);
	c0 = std::max<std::ptrdiff_t>(0, c0);
	c1 = std::min(width, std::max<std::ptrdiff_t>(0, c1));
	if (c1 <= c0 || static_cast<double>(c1 - c0) < 4.0 * ppm)
		return {false, 0.0};
	const auto lab = column_lab(
			image, occlusion, static_cast<std::size_t>(c0), static_cast<std::size_t>(c1));
	if (lab.size() < 2)
		return {false, 0.0};
	const auto middle = lab.size() / 2;
	const auto left = median_lab(lab, 0, middle);
	const auto right = median_lab(lab, middle, lab.size());
	const double delta_e = lab_distance(left, right);
	std::vector<double> left_l, right_l;
	left_l.reserve(middle);
	right_l.reserve(lab.size() - middle);
	for (std::size_t i = 0; i < middle; ++i)
		left_l.push_back(lab[i][0]);
	for (std::size_t i = middle; i < lab.size(); ++i)
		right_l.push_back(lab[i][0]);
	const auto left_period = autocorrelation_period(left_l, ppm);
	const auto right_period = autocorrelation_period(right_l, ppm);
	bool fires = delta_e > colour_max_delta_e;
	if (left_period && right_period &&
			std::abs(*left_period - *right_period) /
							std::max(*left_period, *right_period) >
					rhythm_max_diff)
		fires = true;
	return {fires, delta_e};
}

std::size_t strongest_peak(const std::vector<const Peak *> &peaks)
{
	std::size_t best = 0;
	for (std::size_t i = 1; i < peaks.size(); ++i)
		if (peaks[i]->energy > peaks[best]->energy)
			best = i;
	return best;
}

double median_values(std::vector<double> &values)
{
	return imgops::median(std::move(values));
}

std::array<double, 3> median_oklab(const std::vector<imgops::Lab> &samples)
{
	std::array<double, 3> result{};
	for (std::size_t channel = 0; channel < result.size(); ++channel) {
		std::vector<double> values;
		values.reserve(samples.size());
		for (const auto &sample : samples)
			values.push_back(sample[channel]);
		result[channel] = median_values(values);
	}
	return result;
}

std::vector<double> median_filter_f64(const std::vector<double> &source,
		std::size_t width, std::size_t height, std::size_t kernel)
{
	if (!width || !height || source.size() < width * height || !kernel)
		return {};
	const auto radius = static_cast<std::ptrdiff_t>(kernel / 2);
	std::vector<double> output(width * height);
	std::vector<double> window;
	window.reserve(kernel * kernel);
	for (std::size_t y = 0; y < height; ++y)
		for (std::size_t x = 0; x < width; ++x) {
			window.clear();
			for (std::ptrdiff_t dy = -radius; dy <= radius; ++dy) {
				const auto sy =
						std::clamp<std::ptrdiff_t>(static_cast<std::ptrdiff_t>(y) + dy, 0,
								static_cast<std::ptrdiff_t>(height) - 1);
				for (std::ptrdiff_t dx = -radius; dx <= radius; ++dx) {
					const auto sx = std::clamp<std::ptrdiff_t>(
							static_cast<std::ptrdiff_t>(x) + dx, 0,
							static_cast<std::ptrdiff_t>(width) - 1);
					window.push_back(source[static_cast<std::size_t>(sy) * width +
											static_cast<std::size_t>(sx)]);
				}
			}
			std::sort(window.begin(), window.end(), [](double a, double b) {
				const auto key = [](double v) {
					const auto bits = std::bit_cast<std::uint64_t>(v);
					return (bits >> 63) ? ~bits : (bits ^ (std::uint64_t{1} << 63));
				};
				return key(a) < key(b);
			});
			output[y * width + x] = window[window.size() / 2];
		}
	return output;
}

double bimodality(const std::vector<double> &lightness, const std::vector<double> &blue,
		const std::vector<bool> &valid, std::size_t width, std::size_t height,
		std::size_t pixels_per_block, std::size_t origin_x, std::size_t origin_y)
{
	if (!pixels_per_block || height <= origin_y || width <= origin_x)
		return 0.0;
	const auto blocks_y = (height - origin_y) / pixels_per_block;
	const auto blocks_x = (width - origin_x) / pixels_per_block;
	if (!blocks_y || !blocks_x)
		return 0.0;
	std::vector<double> medians(
			blocks_y * blocks_x, std::numeric_limits<double>::quiet_NaN());
	std::vector<double> shares(blocks_y * blocks_x, 0.0);
	for (std::size_t by = 0; by < blocks_y; ++by)
		for (std::size_t bx = 0; bx < blocks_x; ++bx) {
			std::vector<double> values;
			values.reserve(pixels_per_block * pixels_per_block);
			for (std::size_t y = 0; y < pixels_per_block; ++y)
				for (std::size_t x = 0; x < pixels_per_block; ++x) {
					const auto index = (origin_y + by * pixels_per_block + y) * width +
									   origin_x + bx * pixels_per_block + x;
					if (index < valid.size() && valid[index] && index < lightness.size())
						values.push_back(lightness[index]);
				}
			const auto index = by * blocks_x + bx;
			shares[index] = static_cast<double>(values.size()) /
							(static_cast<double>(pixels_per_block) * pixels_per_block);
			if (!values.empty())
				medians[index] = median_values(values);
		}
	std::vector<bool> observed(shares.size());
	for (std::size_t i = 0; i < shares.size(); ++i)
		observed[i] = shares[i] >= 0.3;
	if (std::none_of(observed.begin(), observed.end(), [](bool v) { return v; }))
		return 0.0;
	std::vector<double> seen;
	for (std::size_t i = 0; i < medians.size(); ++i)
		if (observed[i] && std::isfinite(medians[i]))
			seen.push_back(medians[i]);
	const double fill = seen.empty() ? 0.0 : median_values(seen);
	for (auto &value : medians)
		if (!std::isfinite(value))
			value = fill;
	const auto reference = median_filter_f64(medians, blocks_x, blocks_y, 5);
	double acc = 0.0;
	std::size_t count = 0;
	for (std::size_t by = 0; by < blocks_y; ++by)
		for (std::size_t bx = 0; bx < blocks_x; ++bx) {
			const auto block = by * blocks_x + bx;
			if (!observed[block])
				continue;
			const double lr = reference[block];
			const double threshold = 0.12 * std::clamp(lr / 0.55, 0.5, 1.0);
			std::size_t hits = 0, total = 0;
			for (std::size_t y = 0; y < pixels_per_block; ++y)
				for (std::size_t x = 0; x < pixels_per_block; ++x) {
					const auto index = (origin_y + by * pixels_per_block + y) * width +
									   origin_x + bx * pixels_per_block + x;
					if (index >= valid.size() || !valid[index] ||
							index >= lightness.size() || index >= blue.size())
						continue;
					++total;
					const bool dark = lightness[index] < lr - threshold;
					const bool glass =
							blue[index] < -0.04 && lightness[index] > lr + 0.05;
					if (dark || glass)
						++hits;
				}
			const double dark_share =
					static_cast<double>(hits) / std::max<std::size_t>(total, 1);
			acc += std::abs(dark_share - 0.5);
			++count;
		}
	return count ? acc / count * 2.0 : 0.0;
}

double round6(double value)
{
	return imgops::round_half_even(value * 1e6) / 1e6;
}

double angle_diff_signed(double a, double b)
{
	double difference = a - b;
	while (difference <= -3.14159265358979323846)
		difference += 2.0 * 3.14159265358979323846;
	while (difference > 3.14159265358979323846)
		difference -= 2.0 * 3.14159265358979323846;
	return difference;
}

std::pair<AngleField, std::vector<std::pair<std::size_t, std::size_t>>> ll_angle(
		const std::vector<std::uint8_t> &image, std::size_t width, std::size_t height,
		double threshold)
{
	AngleField field;
	field.width = width;
	field.height = height;
	field.angles.assign(width * height, not_defined_angle);
	field.modgrad.assign(width * height, 0.0);
	double max_gradient = -1.0;
	for (std::size_t y = 0; y + 1 < height; ++y)
		for (std::size_t x = 0; x + 1 < width; ++x) {
			const int a = image[y * width + x];
			const int b = image[y * width + x + 1];
			const int c = image[(y + 1) * width + x];
			const int d = image[(y + 1) * width + x + 1];
			const double da = d - a, bc = b - c;
			const double gx = da + bc, gy = da - bc;
			const double norm = std::sqrt((gx * gx + gy * gy) / 4.0);
			const auto index = y * width + x;
			field.modgrad[index] = norm;
			if (norm > threshold) {
				field.angles[index] =
						std::fmod(std::atan2(gx, -gy) + 2.0 * 3.14159265358979323846,
								2.0 * 3.14159265358979323846);
				if (norm > max_gradient)
					max_gradient = norm;
			}
		}
	std::vector<std::vector<std::pair<std::size_t, std::size_t>>> bins(lsd_bins);
	const double coefficient =
			max_gradient > 0.0 ? static_cast<double>(lsd_bins - 1) / max_gradient : 0.0;
	for (std::size_t y = 0; y + 1 < height; ++y)
		for (std::size_t x = 0; x + 1 < width; ++x) {
			const auto index = y * width + x;
			const auto bin =
					std::min(static_cast<std::size_t>(field.modgrad[index] * coefficient),
							lsd_bins - 1);
			bins[bin].emplace_back(x, y);
		}
	std::vector<std::pair<std::size_t, std::size_t>> ordered;
	ordered.reserve(width * height);
	for (auto bin = bins.rbegin(); bin != bins.rend(); ++bin)
		ordered.insert(ordered.end(), bin->begin(), bin->end());
	return {std::move(field), std::move(ordered)};
}

double region_grow(const AngleField &field, std::vector<bool> &used,
		std::pair<std::size_t, std::size_t> seed, double precision,
		std::vector<RegionPoint> &region)
{
	region.clear();
	const auto [sx, sy] = seed;
	double angle = field.angles[sy * field.width + sx];
	region.push_back({static_cast<int>(sx), static_cast<int>(sy), angle,
			field.modgrad[sy * field.width + sx]});
	used[sy * field.width + sx] = true;
	double sum_dx = std::cos(angle), sum_dy = std::sin(angle);
	std::size_t i = 0;
	while (i < region.size()) {
		const auto point = region[i];
		const int x_min = std::max(point.x - 1, 0);
		const int x_max = std::min(point.x + 1, static_cast<int>(field.width) - 1);
		const int y_min = std::max(point.y - 1, 0);
		const int y_max = std::min(point.y + 1, static_cast<int>(field.height) - 1);
		for (int y = y_min; y <= y_max; ++y)
			for (int x = x_min; x <= x_max; ++x) {
				const auto index = static_cast<std::size_t>(y) * field.width + x;
				if (used[index] || !field.aligned(x, y, angle, precision))
					continue;
				used[index] = true;
				const double next_angle = field.angles[index];
				region.push_back({x, y, next_angle, field.modgrad[index]});
				sum_dx += std::cos(next_angle);
				sum_dy += std::sin(next_angle);
				angle = std::fmod(
						std::atan2(sum_dy, sum_dx) + 2.0 * 3.14159265358979323846,
						2.0 * 3.14159265358979323846);
			}
		++i;
	}
	return angle;
}

double region_theta(const std::vector<RegionPoint> &region, double x, double y,
		double region_angle, double precision)
{
	double ixx = 0.0, iyy = 0.0, ixy = 0.0;
	for (const auto &point : region) {
		const double dx = point.x - x, dy = point.y - y;
		ixx += dy * dy * point.gradient;
		iyy += dx * dx * point.gradient;
		ixy -= dx * dy * point.gradient;
	}
	const double lambda =
			0.5 * (ixx + iyy - std::sqrt((ixx - iyy) * (ixx - iyy) + 4.0 * ixy * ixy));
	double theta = std::abs(ixx) > std::abs(iyy) ? std::atan2(lambda - ixx, ixy)
												 : std::atan2(ixy, lambda - iyy);
	theta = std::fmod(theta + 2.0 * 3.14159265358979323846, 2.0 * 3.14159265358979323846);
	if (std::abs(angle_diff_signed(theta, region_angle)) > precision)
		theta += 3.14159265358979323846;
	return theta;
}

LsdRectangle region_to_rectangle(const std::vector<RegionPoint> &region,
		double region_angle, double precision, double p)
{
	double x = 0.0, y = 0.0, sum = 0.0;
	for (const auto &point : region) {
		x += point.x * point.gradient;
		y += point.y * point.gradient;
		sum += point.gradient;
	}
	if (sum <= 0.0)
		return {};
	x /= sum;
	y /= sum;
	const double theta = region_theta(region, x, y, region_angle, precision);
	const double dx = std::cos(theta), dy = std::sin(theta);
	double l_min = 0.0, l_max = 0.0, w_min = 0.0, w_max = 0.0;
	for (const auto &point : region) {
		const double rdx = point.x - x, rdy = point.y - y;
		const double along = rdx * dx + rdy * dy;
		const double across = -rdx * dy + rdy * dx;
		if (along > l_max)
			l_max = along;
		else if (along < l_min)
			l_min = along;
		if (across > w_max)
			w_max = across;
		else if (across < w_min)
			w_min = across;
	}
	return {x + l_min * dx, y + l_min * dy, x + l_max * dx, y + l_max * dy,
			std::max(w_max - w_min, 1.0), x, y, theta, dx, dy, precision, p};
}

double point_distance(double x0, double y0, double x1, double y1)
{
	return std::hypot(x1 - x0, y1 - y0);
}

bool refine_lsd_region(const AngleField &field, std::vector<bool> &used,
		std::vector<RegionPoint> &region, double region_angle, double precision, double p,
		LsdRectangle &rectangle)
{
	double density = region.size() / std::max(point_distance(rectangle.x1, rectangle.y1,
													  rectangle.x2, rectangle.y2) *
													  rectangle.width,
											 1e-12);
	if (density >= lsd_density_threshold)
		return true;
	if (region.empty())
		return false;
	const double xc = region[0].x, yc = region[0].y;
	const double seed_angle = region[0].angle;
	double sum = 0.0, squared_sum = 0.0;
	std::size_t count = 0;
	for (const auto &point : region) {
		used[static_cast<std::size_t>(point.y) * field.width + point.x] = false;
		if (point_distance(xc, yc, point.x, point.y) < rectangle.width) {
			const double difference = angle_diff_signed(point.angle, seed_angle);
			sum += difference;
			squared_sum += difference * difference;
			++count;
		}
	}
	if (!count)
		return false;
	const double mean_angle = sum / count;
	const double tau = 2.0 * std::sqrt((squared_sum - 2.0 * mean_angle * sum) / count +
									   mean_angle * mean_angle);
	const auto seed = std::pair{
			static_cast<std::size_t>(region[0].x), static_cast<std::size_t>(region[0].y)};
	const double angle = region_grow(field, used, seed, tau, region);
	if (region.size() < 2)
		return false;
	rectangle = region_to_rectangle(region, angle, precision, p);
	density = region.size() / std::max(point_distance(rectangle.x1, rectangle.y1,
											   rectangle.x2, rectangle.y2) *
											   rectangle.width,
									  1e-12);
	if (density >= lsd_density_threshold)
		return true;
	double radius_squared =
			std::max(std::pow(point_distance(xc, yc, rectangle.x1, rectangle.y1), 2),
					std::pow(point_distance(xc, yc, rectangle.x2, rectangle.y2), 2));
	while (density < lsd_density_threshold) {
		radius_squared *= 0.75 * 0.75;
		std::size_t i = 0;
		while (i < region.size()) {
			const double dx = region[i].x - xc, dy = region[i].y - yc;
			if (dx * dx + dy * dy > radius_squared) {
				used[static_cast<std::size_t>(region[i].y) * field.width + region[i].x] =
						false;
				region[i] = region.back();
				region.pop_back();
			} else {
				++i;
			}
		}
		if (region.size() < 2)
			return false;
		rectangle = region_to_rectangle(region, region_angle, precision, p);
		density = region.size() / std::max(point_distance(rectangle.x1, rectangle.y1,
												   rectangle.x2, rectangle.y2) *
												   rectangle.width,
										  1e-12);
	}
	return true;
}

std::pair<std::vector<float>, std::vector<float>> colour_gradient(
		const projection::Image &image)
{
	const auto width = static_cast<std::size_t>(image.width);
	const auto height = static_cast<std::size_t>(image.height);
	const auto count = width * height;
	std::vector<float> magnitude(count, 0.0f), vertical(count, 0.0f);
	if (!width || !height || image.pixels.size() < count)
		return {std::move(magnitude), std::move(vertical)};
	for (std::size_t channel = 0; channel < 3; ++channel) {
		std::vector<float> plane(count);
		for (std::size_t i = 0; i < count; ++i)
			plane[i] = image.pixels[i][channel];
		const auto gx = sobel(plane, width, height, true);
		const auto gy = sobel(plane, width, height, false);
		for (std::size_t i = 0; i < count; ++i) {
			magnitude[i] = std::max(magnitude[i], std::hypot(gx[i], gy[i]));
			vertical[i] = std::max(vertical[i], std::abs(gy[i]));
		}
	}
	return {std::move(magnitude), std::move(vertical)};
}

std::vector<bool> sky_rule(const std::vector<std::array<std::uint8_t, 3>> &hsv)
{
	std::vector<bool> result;
	result.reserve(hsv.size());
	for (const auto &pixel : hsv) {
		const int hue = pixel[0], saturation = pixel[1], value = pixel[2];
		result.push_back(
				(hue >= sky_hue_range[0] && hue <= sky_hue_range[1] && saturation > 30) ||
				(value > 150 && saturation < 60));
	}
	return result;
}

std::vector<bool> otsu_sky_side(const std::vector<std::uint8_t> &values,
		std::size_t width, std::size_t height, std::size_t region_rows)
{
	const auto count = width * height;
	if (!width || !height || values.size() < count)
		return std::vector<bool>(count, false);
	region_rows = std::clamp<std::size_t>(region_rows, 1, height);
	std::array<std::size_t, 256> region_histogram{};
	for (std::size_t i = 0; i < region_rows * width; ++i)
		++region_histogram[values[i]];
	const double threshold = otsu_threshold(region_histogram);
	std::vector<bool> high(count);
	for (std::size_t i = 0; i < count; ++i)
		high[i] = values[i] > threshold;
	const double top_share =
			static_cast<double>(std::count(high.begin(), high.begin() + width, true)) /
			width;
	std::vector<bool> side = high;
	if (top_share < 0.5)
		for (std::size_t i = 0; i < side.size(); ++i)
			side[i] = !side[i];
	std::vector<std::uint8_t> selected;
	selected.reserve(region_rows * width);
	for (std::size_t i = 0; i < region_rows * width; ++i)
		if (side[i])
			selected.push_back(values[i]);
	if (selected.size() < 100 ||
			*std::min_element(selected.begin(), selected.end()) ==
					*std::max_element(selected.begin(), selected.end()))
		return side;
	std::array<std::size_t, 256> selected_histogram{};
	for (const auto value : selected)
		++selected_histogram[value];
	const double threshold2 = otsu_threshold(selected_histogram);
	std::size_t high2_count = 0;
	double mean_high = 0.0, mean_low = 0.0;
	std::size_t low_count = 0;
	for (const auto value : selected) {
		const bool bit = value > threshold2;
		if (bit) {
			++high2_count;
			mean_high += value;
		} else {
			++low_count;
			mean_low += value;
		}
	}
	const double fraction = static_cast<double>(high2_count) / selected.size();
	if (fraction < 0.15 || fraction > 0.85 || !high2_count || !low_count)
		return side;
	mean_high /= high2_count;
	mean_low /= low_count;
	double mean_all = 0.0;
	for (const auto value : selected)
		mean_all += value;
	mean_all /= selected.size();
	double variance = 0.0;
	for (const auto value : selected) {
		const double delta = value - mean_all;
		variance += delta * delta;
	}
	variance /= selected.size();
	const double separation = fraction * (1.0 - fraction) * (mean_high - mean_low) *
							  (mean_high - mean_low) / std::max(variance, 1e-9);
	if (std::abs(mean_high - mean_low) <= 25.0 || separation <= 0.8)
		return side;
	std::size_t row0_high = 0, row0_low = 0;
	for (std::size_t x = 0; x < width; ++x) {
		if (!side[x])
			continue;
		if (values[x] > threshold2)
			++row0_high;
		else
			++row0_low;
	}
	const bool keep_high = row0_high >= row0_low;
	for (std::size_t i = 0; i < count; ++i)
		side[i] = side[i] && ((values[i] > threshold2) == keep_high);
	return side;
}

std::vector<std::array<double, 3>> column_lab(const projection::Image &image,
		const std::vector<std::uint8_t> *occlusion, std::size_t x0, std::size_t x1,
		std::array<double, 2> rows)
{
	const auto width = static_cast<std::size_t>(image.width);
	const auto height = static_cast<std::size_t>(image.height);
	if (x1 <= x0 || x1 > width || image.pixels.size() < width * height)
		return {};
	std::size_t first = static_cast<std::size_t>(std::max(rows[0], 0.0));
	std::size_t last =
			rows[1] < 0.0 ? 0 : std::min(static_cast<std::size_t>(rows[1]), height);
	if (last <= first) {
		first = 0;
		last = height;
	}
	const auto columns = x1 - x0;
	std::vector<std::array<double, 3>> result(columns, {0.0, 0.0, 0.0});
	std::vector<double> counts(columns, 0.0);
	for (std::size_t y = first; y < last; ++y)
		for (std::size_t col = 0; col < columns; ++col) {
			const auto index = y * width + x0 + col;
			if (occlusion && index < occlusion->size() &&
					((*occlusion)[index] & occluded_bits) != 0)
				continue;
			const auto lab = rgb_to_lab(image.pixels[index]);
			for (std::size_t channel = 0; channel < 3; ++channel)
				result[col][channel] += lab[channel];
			counts[col] += 1.0;
		}
	for (std::size_t col = 0; col < columns; ++col)
		for (auto &channel : result[col])
			channel /= std::max(counts[col], 1e-6);
	std::vector<std::size_t> valid;
	for (std::size_t col = 0; col < columns; ++col)
		if (counts[col] >= 1.0)
			valid.push_back(col);
	if (!valid.empty() && valid.size() < columns)
		for (std::size_t col = 0; col < columns; ++col) {
			if (counts[col] >= 1.0)
				continue;
			auto right = std::lower_bound(valid.begin(), valid.end(), col);
			if (right == valid.begin()) {
				result[col] = result[*right];
			} else if (right == valid.end()) {
				result[col] = result[valid.back()];
			} else {
				const auto left_index = *(right - 1), right_index = *right;
				const double t = static_cast<double>(col - left_index) /
								 (right_index - left_index);
				for (std::size_t channel = 0; channel < 3; ++channel)
					result[col][channel] = result[left_index][channel] +
										   t * (result[right_index][channel] -
													   result[left_index][channel]);
			}
		}
	return result;
}

std::vector<double> gradient_1d(const std::vector<double> &values)
{
	if (values.size() < 2)
		return std::vector<double>(values.size(), 0.0);
	std::vector<double> result(values.size());
	result.front() = values[1] - values[0];
	result.back() = values.back() - values[values.size() - 2];
	for (std::size_t i = 1; i + 1 < values.size(); ++i)
		result[i] = 0.5 * (values[i + 1] - values[i - 1]);
	return result;
}

std::vector<std::size_t> find_peaks(const std::vector<float> &values,
		double minimum_height, std::size_t minimum_distance)
{
	std::vector<std::size_t> peaks;
	std::size_t i = 1;
	while (i + 1 < values.size()) {
		if (values[i - 1] < values[i]) {
			std::size_t end = i;
			while (end + 1 < values.size() && values[end + 1] == values[i])
				++end;
			if (end + 1 < values.size() && values[end + 1] < values[i])
				peaks.push_back((i + end) / 2);
			i = end + 1;
		} else {
			++i;
		}
	}
	peaks.erase(std::remove_if(peaks.begin(), peaks.end(),
						[&](std::size_t peak) { return values[peak] < minimum_height; }),
			peaks.end());
	if (minimum_distance <= 1 || peaks.empty())
		return peaks;
	std::vector<std::size_t> order(peaks.size());
	std::iota(order.begin(), order.end(), 0);
	std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
		return std::pair{values[peaks[a]], a} > std::pair{values[peaks[b]], b};
	});
	std::vector<bool> keep(peaks.size(), true);
	for (const auto selected : order) {
		if (!keep[selected])
			continue;
		for (std::ptrdiff_t other = static_cast<std::ptrdiff_t>(selected) - 1;
				other >= 0 && peaks[selected] - peaks[other] < minimum_distance; --other)
			keep[static_cast<std::size_t>(other)] = false;
		for (std::size_t other = selected + 1;
				other < peaks.size() && peaks[other] - peaks[selected] < minimum_distance;
				++other)
			keep[other] = false;
	}
	std::vector<std::size_t> result;
	for (std::size_t n = 0; n < peaks.size(); ++n)
		if (keep[n])
			result.push_back(peaks[n]);
	return result;
}
} // namespace

std::tuple<imgops::Mask, const char *, std::vector<bool>> sky_mask(
		const projection::Image &image,
		std::optional<std::pair<double, double>> facade_rows)
{
	const auto width = static_cast<std::size_t>(image.width);
	const auto height = static_cast<std::size_t>(image.height);
	const auto count = width * height;
	if (!width || !height || image.pixels.size() < count)
		return {imgops::Mask{}, "rule", {}};
	auto [magnitude, unused_vertical] = colour_gradient(image);
	(void)unused_vertical;
	const auto smooth = gaussian_blur(magnitude, width, height, 5, 0.0);
	std::vector<bool> flat(count);
	for (std::size_t i = 0; i < count; ++i)
		flat[i] = smooth[i] < sky_sobel_max;
	const auto hsv = rgb_to_hsv(image);
	auto colour = sky_rule(hsv);
	std::vector<bool> candidates(count);
	for (std::size_t i = 0; i < count; ++i)
		candidates[i] = flat[i] && colour[i];
	const auto top_rows = std::max<std::size_t>(
			static_cast<std::size_t>(imgops::round_half_even(sky_top_fraction * height)),
			1);
	const auto top_count = std::min(top_rows * width, count);
	const double top_fraction = static_cast<double>(std::count(candidates.begin(),
										candidates.begin() + top_count, true)) /
								(top_rows * width);
	bool over = false;
	std::size_t row_limit = height / 2;
	if (facade_rows) {
		const auto first = static_cast<std::size_t>(
				std::clamp(std::min(facade_rows->first, facade_rows->second), 0.0,
						static_cast<double>(height)));
		const auto last = static_cast<std::size_t>(
				std::clamp(std::max(facade_rows->first, facade_rows->second), 0.0,
						static_cast<double>(height)));
		if (last > first) {
			const auto begin = colour.begin() + first * width;
			const auto end = colour.begin() + last * width;
			over = static_cast<double>(std::count(begin, end, true)) /
						   ((last - first) * width) >
				   sky_rule_max;
			row_limit = last;
		}
	}
	const char *mode = "rule";
	if (top_fraction < sky_rule_min || over) {
		std::vector<std::uint8_t> value_plane(count);
		for (std::size_t i = 0; i < count; ++i)
			value_plane[i] = hsv[i][2];
		const auto rows = std::min(std::max(row_limit, top_rows + 1), height);
		colour = otsu_sky_side(value_plane, width, height, rows);
		for (std::size_t i = 0; i < count; ++i)
			candidates[i] = flat[i] && colour[i];
		mode = "otsu";
	}
	auto mask = imgops::Mask::from_bits(width, height, std::move(candidates));
	const auto labels = imgops::connected_components(mask);
	std::vector<std::uint32_t> top_labels;
	for (std::size_t x = 0; x < width; ++x) {
		if (!mask.at(x, 0))
			continue;
		const auto label = labels.at(x, 0);
		if (label)
			top_labels.push_back(label);
	}
	std::sort(top_labels.begin(), top_labels.end());
	top_labels.erase(std::unique(top_labels.begin(), top_labels.end()), top_labels.end());
	std::vector<bool> connected;
	connected.reserve(labels.labels.size());
	for (const auto label : labels.labels)
		connected.push_back(
				std::binary_search(top_labels.begin(), top_labels.end(), label));
	return {imgops::Mask::from_bits(width, height, std::move(connected)), mode,
			std::move(colour)};
}

std::vector<double> row_energy(const projection::Image &image,
		const std::vector<std::uint8_t> *occlusion,
		const std::vector<std::size_t> *columns)
{
	const auto width = static_cast<std::size_t>(image.width);
	const auto height = static_cast<std::size_t>(image.height);
	if (!width || !height || image.pixels.size() < width * height)
		return {};
	const auto gradients = colour_gradient(image).second;
	std::vector<std::size_t> all;
	if (!columns) {
		all.resize(width);
		std::iota(all.begin(), all.end(), 0);
		columns = &all;
	}
	std::vector<double> profile(height, 0.0);
	for (std::size_t y = 0; y < height; ++y) {
		double numerator = 0.0;
		std::size_t denominator = 0;
		for (const auto column : *columns) {
			if (column >= width)
				continue;
			const auto index = y * width + column;
			if (occlusion && index < occlusion->size() &&
					((*occlusion)[index] & occluded_bits) != 0)
				continue;
			numerator += gradients[index] / 8.0;
			++denominator;
		}
		profile[y] = numerator / (occlusion ? std::max<std::size_t>(denominator, 1)
											: std::max<std::size_t>(columns->size(), 1));
	}
	return convolve_same(profile, {0.25, 0.5, 0.25});
}

std::optional<std::size_t> strongest_row(const std::vector<double> &energy,
		std::ptrdiff_t first, std::ptrdiff_t last, double ratio,
		const std::vector<bool> *seen)
{
	const auto begin = static_cast<std::size_t>(std::max<std::ptrdiff_t>(first, 0));
	const auto end = std::min<std::size_t>(
			static_cast<std::size_t>(std::max<std::ptrdiff_t>(last, 0)), energy.size());
	if (end <= begin)
		return std::nullopt;
	const auto best = begin + argmax_first(std::vector<double>(
									  energy.begin() + begin, energy.begin() + end));
	std::vector<double> kept;
	if (seen && std::any_of(seen->begin(), seen->end(), [](bool bit) { return bit; })) {
		const auto n = std::min(energy.size(), seen->size());
		for (std::size_t i = 0; i < n; ++i)
			if ((*seen)[i])
				kept.push_back(energy[i]);
	} else {
		kept = energy;
	}
	const double reference =
			std::max(ratio * imgops::median(std::move(kept)), edge_energy_floor);
	return energy[best] >= reference ? std::optional<std::size_t>(best) : std::nullopt;
}

double edge_y(const std::vector<double> &energy, std::size_t row)
{
	if (energy.empty())
		return 0.5;
	const auto first = row == 0 ? 0 : row - 1;
	const auto last = std::min(row + 2, energy.size());
	double sum = 0.0, weighted = 0.0;
	for (std::size_t i = first; i < last; ++i) {
		sum += energy[i];
		weighted += energy[i] * (static_cast<double>(i) + 0.5);
	}
	return sum <= 0.0 ? static_cast<double>(row) + 0.5 : weighted / sum;
}

std::pair<double, std::string> ground_row(
		const rectify::LooseCrop &crop, double ground_row_y, GroundSource source)
{
	const auto energy = row_energy(crop.rgb, &crop.occl);
	const auto window = static_cast<std::ptrdiff_t>(
			imgops::round_half_even(ground_window_metres * crop.ppm));
	const auto row = static_cast<std::ptrdiff_t>(imgops::round_half_even(ground_row_y));
	if (const auto best = strongest_row(
				energy, row - window, row + window + 1, ground_edge_ratio))
		return {crop.y_to_h(edge_y(energy, *best)), ground_edge};
	return {0.0, source == GroundSource::Cloud ? ground_cloud : ground_default};
}

std::pair<std::vector<float>, std::vector<Peak>> column_profile(
		const rectify::LooseCrop &crop, const std::vector<LineSegment> &verticals,
		const std::array<double, 2> &row_bounds)
{
	const auto width = static_cast<std::size_t>(crop.rgb.width);
	if (!width || crop.ppm <= 0.0)
		return {};
	const double s_a = crop.s0, s_b = crop.x_to_s(width);
	const auto count = std::max<std::size_t>(
			static_cast<std::size_t>(imgops::round_half_even((s_b - s_a) * profile_ppm)),
			2);
	std::vector<double> positions(count), line_energy(count, 0.0);
	for (std::size_t i = 0; i < count; ++i)
		positions[i] = s_a + (static_cast<double>(i) + 0.5) / profile_ppm;
	const double row_low = std::min(row_bounds[0], row_bounds[1]);
	const double row_high = std::max(row_bounds[0], row_bounds[1]);
	for (const auto &segment : verticals) {
		if (segment.mid_y() < row_low || segment.mid_y() > row_high)
			continue;
		const double s_mid = crop.x_to_s(segment.mid_x());
		const auto index = static_cast<std::size_t>(
				std::clamp(imgops::round_half_even((s_mid - s_a) * profile_ppm - 0.5),
						0.0, static_cast<double>(count - 1)));
		line_energy[index] += segment.length / crop.ppm;
	}
	const double sigma = profile_sigma_m * profile_ppm;
	line_energy = gaussian_blur_1d(line_energy, sigma);
	const auto labs = column_lab(crop.rgb, &crop.occl, 0, width, row_bounds);
	std::vector<double> source_x(width);
	for (std::size_t x = 0; x < width; ++x)
		source_x[x] = crop.x_to_s(static_cast<double>(x) + 0.5);
	std::array<std::vector<double>, 3> resampled;
	for (std::size_t channel = 0; channel < 3; ++channel) {
		std::vector<double> source_lab;
		source_lab.reserve(labs.size());
		for (const auto &lab : labs)
			source_lab.push_back(lab[channel]);
		resampled[channel].reserve(count);
		for (const auto position : positions)
			resampled[channel].push_back(interp(position, source_x, source_lab));
		resampled[channel] = gaussian_blur_1d(resampled[channel], sigma);
	}
	std::vector<double> colour_energy(count, 0.0);
	for (const auto &channel : resampled) {
		const auto gradient = gradient_1d(channel);
		for (std::size_t i = 0; i < count; ++i)
			colour_energy[i] += std::pow(gradient[i] * profile_ppm, 2);
	}
	for (auto &value : colour_energy)
		value = std::sqrt(value);
	const double p99 = std::max(imgops::percentile(colour_energy, 99.0), 1e-6);
	const double maximum = *std::max_element(line_energy.begin(), line_energy.end());
	const double normalizer = std::max({imgops::median(line_energy), 0.1 * maximum, 0.3});
	std::vector<float> profile(count);
	for (std::size_t i = 0; i < count; ++i)
		profile[i] =
				static_cast<float>(line_energy[i] / normalizer +
								   2.0 * std::clamp(colour_energy[i] / p99, 0.0, 1.0));
	const auto min_separation = std::max<std::size_t>(
			static_cast<std::size_t>(peak_min_separation_m * profile_ppm), 1);
	const auto half_window =
			static_cast<std::size_t>(imgops::round_half_even(2.0 * sigma));
	std::vector<Peak> peaks;
	for (const auto index : find_peaks(profile, peak_min_energy, min_separation)) {
		const auto first = index > half_window ? index - half_window : 0;
		const auto last = std::min(index + half_window + 1, count);
		const double vertical_length = std::accumulate(
				line_energy.begin() + first, line_energy.begin() + last, 0.0);
		peaks.push_back({positions[index], profile[index], vertical_length});
	}
	return {std::move(profile), std::move(peaks)};
}

namespace
{
bool nodata_column(const rectify::LooseCrop &crop, std::size_t column, std::size_t last)
{
	const auto width = static_cast<std::size_t>(crop.rgb.width);
	const auto height = static_cast<std::size_t>(crop.rgb.height);
	if (column >= width || crop.ppm <= 0.0)
		return false;
	const auto band = std::max<std::size_t>(
			static_cast<std::size_t>(
					imgops::round_half_even(roof_nodata_band_metres * crop.ppm)),
			1);
	const auto end = std::min(last + 1, height);
	const auto first = end > band ? end - band : 0;
	if (end <= first)
		return false;
	std::size_t outside = 0;
	for (std::size_t y = first; y < end; ++y) {
		const auto index = y * width + column;
		if (index < crop.occl.size() && (crop.occl[index] & OCC_OUTSIDE) != 0)
			++outside;
	}
	return static_cast<double>(outside) / (end - first) > roof_nodata_max;
}

std::array<double, 3> median_lab(const std::vector<std::array<double, 3>> &samples)
{
	std::array<double, 3> result{};
	for (std::size_t channel = 0; channel < 3; ++channel) {
		std::vector<double> values;
		values.reserve(samples.size());
		for (const auto &sample : samples)
			values.push_back(sample[channel]);
		result[channel] = imgops::median(std::move(values));
	}
	return result;
}

std::optional<double> roof_edge_below(const rectify::LooseCrop &crop,
		const std::vector<double> &energy, const std::vector<std::size_t> &columns,
		double sky_height)
{
	const double low_height = std::max(0.4 * sky_height, 3.0);
	const double high_height = sky_height - 0.75;
	if (high_height - low_height < 1.0 || columns.empty())
		return std::nullopt;
	const auto first = std::min<std::size_t>(
			static_cast<std::size_t>(std::max<std::ptrdiff_t>(
					static_cast<std::ptrdiff_t>(crop.h_to_y(high_height)), 0)),
			energy.size());
	const auto last = std::min<std::size_t>(
			static_cast<std::size_t>(std::max<std::ptrdiff_t>(
					static_cast<std::ptrdiff_t>(crop.h_to_y(low_height)), 0)),
			energy.size());
	if (last < first + 3)
		return std::nullopt;
	const auto band_begin = energy.begin() + first;
	const auto band_end = energy.begin() + last;
	const auto best = first + argmax_first(std::vector<double>(band_begin, band_end));
	const std::vector<double> band(band_begin, band_end);
	if (energy[best] <
			std::max(roof_edge_ratio * imgops::median(band), edge_energy_floor))
		return std::nullopt;
	const double edge_height = crop.y_to_h(edge_y(energy, best));
	const auto pixel_radius =
			static_cast<std::ptrdiff_t>(imgops::round_half_even(crop.ppm));
	const auto width = static_cast<std::size_t>(crop.rgb.width);
	projection::Image subset;
	subset.width = static_cast<unsigned>(std::max<std::size_t>(columns.size(), 1));
	subset.height = crop.rgb.height;
	subset.pixels.resize(std::size_t(subset.width) * subset.height);
	std::vector<std::uint8_t> subset_occlusion(std::size_t(subset.width) * subset.height);
	for (std::size_t y = 0; y < subset.height; ++y)
		for (std::size_t c = 0; c < columns.size(); ++c) {
			if (columns[c] >= width)
				continue;
			const auto source_index = y * width + columns[c];
			const auto target_index = y * subset.width + c;
			if (source_index < crop.rgb.pixels.size())
				subset.pixels[target_index] = crop.rgb.pixels[source_index];
			if (source_index < crop.occl.size())
				subset_occlusion[target_index] = crop.occl[source_index];
		}
	auto above = column_lab(subset, &subset_occlusion, 0, columns.size(),
			{static_cast<double>(static_cast<std::ptrdiff_t>(best) - pixel_radius - 2),
					static_cast<double>(static_cast<std::ptrdiff_t>(best) - 2)});
	auto below = column_lab(subset, &subset_occlusion, 0, columns.size(),
			{static_cast<double>(best + 3),
					static_cast<double>(
							static_cast<std::ptrdiff_t>(best) + pixel_radius + 3)});
	return lab_distance(median_lab(above), median_lab(below)) > colour_max_delta_e
				   ? std::optional<double>(edge_height)
				   : std::nullopt;
}
} // namespace

Roofline sky_roofline(const rectify::LooseCrop &crop, const imgops::Mask &sky,
		const std::array<double, 2> &s_range, std::optional<double> osm_height,
		bool height_source_default, std::optional<double> cloud_height,
		const Params &params)
{
	const auto width = static_cast<std::size_t>(crop.rgb.width);
	const auto height = static_cast<std::size_t>(crop.rgb.height);
	std::vector<std::size_t> columns;
	for (std::size_t x = 0; x < width; ++x) {
		const double s = crop.x_to_s(static_cast<double>(x) + 0.5);
		if (s >= s_range[0] && s <= s_range[1])
			columns.push_back(x);
	}
	if (columns.empty()) {
		columns.resize(width);
		std::iota(columns.begin(), columns.end(), 0);
	}
	const auto vertical_gradient = colour_gradient(crop.rgb).second;
	std::vector<double> valid_heights, dropped_heights;
	std::size_t reached = 0;
	for (const auto column : columns) {
		if (column >= sky.w || !sky.h)
			continue;
		std::optional<std::size_t> last_sky;
		for (std::size_t y = std::min(height, sky.h); y-- > 0;)
			if (sky.at(column, y)) {
				last_sky = y;
				break;
			}
		if (!last_sky)
			continue;
		++reached;
		const auto first = *last_sky > 0 ? *last_sky - 1 : 0;
		const auto last = std::min(*last_sky + 6, height);
		std::optional<std::size_t> best;
		float best_value = -std::numeric_limits<float>::infinity();
		for (std::size_t y = first; y < last; ++y) {
			const float value = vertical_gradient[y * width + column];
			if (value > best_value) {
				best_value = value;
				best = y;
			}
		}
		double y_edge = static_cast<double>(*last_sky) + 1.0;
		if (best && best_value >= 16.0f) {
			std::vector<double> column_energy(height);
			for (std::size_t y = 0; y < height; ++y)
				column_energy[y] = vertical_gradient[y * width + column];
			y_edge = edge_y(column_energy, *best);
		}
		y_edge = std::clamp(y_edge, static_cast<double>(*last_sky),
				static_cast<double>(*last_sky) + 6.0);
		if (nodata_column(crop, column, *last_sky))
			dropped_heights.push_back(crop.y_to_h(y_edge));
		else
			valid_heights.push_back(crop.y_to_h(y_edge));
	}
	const bool blind = reached > 0 && dropped_heights.size() == reached;
	const double reach =
			columns.empty() ? 0.0
							: static_cast<double>(valid_heights.size()) / columns.size();
	Roofline result;
	if (blind)
		result.flags.push_back(roof_nodata);
	const double default_height =
			osm_height && *osm_height != 0.0 ? *osm_height : params.default_height_m;
	const bool is_default = height_source_default || !osm_height || *osm_height == 0.0;
	std::optional<double> sky_height;
	if (!valid_heights.empty()) {
		sky_height = imgops::percentile(valid_heights, 30.0);
		const double p90 = imgops::percentile(valid_heights, 90.0);
		const double p10 = imgops::percentile(valid_heights, 10.0);
		result.spread_m = p90 - p10;
	}
	const auto energy = row_energy(crop.rgb, &crop.occl, &columns);
	bool accepted = false;
	if (sky_height && reach >= roof_min_reach &&
			result.spread_m <
					std::max(params.roof_spread_m, roof_spread_fraction * *sky_height)) {
		const double low = is_default ? roof_default_range[0]
									  : params.roof_ratio[0] * default_height;
		const double high = is_default ? roof_default_range[1]
									   : params.roof_ratio[1] * default_height;
		accepted = *sky_height >= low && *sky_height <= high;
	}
	std::optional<double> behind_height = sky_height;
	if (!behind_height && blind)
		behind_height = imgops::percentile(dropped_heights, 30.0);
	if (behind_height && cloud_height &&
			*behind_height > *cloud_height + roof_behind_metres)
		result.flags.push_back(roof_behind);
	if (accepted && is_default) {
		const bool cloud_matches =
				cloud_height &&
				std::abs(*sky_height - *cloud_height) <= roof_behind_metres;
		if (!cloud_matches) {
			if (const auto edge = roof_edge_below(crop, energy, columns, *sky_height)) {
				for (const auto *flag : {roof_behind, roof_unconfirmed})
					if (std::find(result.flags.begin(), result.flags.end(), flag) ==
							result.flags.end())
						result.flags.emplace_back(flag);
				result.height = *edge;
				result.flag = roof_edge;
				return result;
			}
			if (!cloud_height)
				result.flags.push_back(roof_unconfirmed);
		}
	}
	if (accepted) {
		result.height = sky_height;
		result.flag = roof_sky;
		return result;
	}
	const auto high_row = static_cast<std::ptrdiff_t>(
			crop.h_to_y(roof_edge_window[1] * default_height));
	const auto low_row = static_cast<std::ptrdiff_t>(
			crop.h_to_y(roof_edge_window[0] * default_height));
	std::vector<bool> seen(height, true);
	for (std::size_t y = 0; y < height; ++y) {
		const auto outside =
				std::count_if(columns.begin(), columns.end(), [&](std::size_t x) {
					const auto index = y * width + x;
					return index < crop.occl.size() &&
						   (crop.occl[index] & OCC_OUTSIDE) != 0;
				});
		seen[y] = columns.empty() ||
				  static_cast<double>(outside) / columns.size() <= roof_nodata_max;
	}
	if (const auto best = strongest_row(
				energy, high_row, low_row + 1, roof_edge_ratio, &seen)) {
		result.height = crop.y_to_h(edge_y(energy, *best));
		result.flag = roof_edge;
		return result;
	}
	result.flag = roof_osm;
	return result;
}

namespace
{
rectify::LooseCrop corrected_crop(const rectify::LooseCrop &crop,
		const std::array<std::array<double, 3>, 3> &homography)
{
	auto [rgb, occlusion] = warp_crop(crop, homography);
	auto result = crop;
	result.rgb = std::move(rgb);
	result.occl = std::move(occlusion);
	return result;
}
} // namespace

ViewRefinement refine_view(
		const rectify::LooseCrop &crop, const ViewInputs &input, const Params &params)
{
	ViewRefinement output;
	if (!input.wall) {
		output.refinement = Refinement::empty({}, crop.z_base);
		return output;
	}
	const auto &wall = *input.wall;
	const double fitted_length = input.fitted_length;
	const double osm_height = wall.height_osm && *wall.height_osm != 0.0
									  ? *wall.height_osm
									  : params.default_height_m;
	const auto width = static_cast<std::size_t>(crop.rgb.width);
	const auto height = static_cast<std::size_t>(crop.rgb.height);
	std::vector<std::uint8_t> gray;
	gray.reserve(crop.rgb.pixels.size());
	for (const auto &pixel : crop.rgb.pixels)
		gray.push_back(static_cast<std::uint8_t>(
				(std::uint32_t(pixel[0]) * 4899 + std::uint32_t(pixel[1]) * 9617 +
						std::uint32_t(pixel[2]) * 1868 + 8192) >>
				14));
	auto [verticals_initial, horizontals_initial] =
			lsd_families(gray, width, height, crop.ppm, &crop.occl);
	auto verticals = std::move(verticals_initial);
	auto horizontals = std::move(horizontals_initial);
	const auto lean = lean_keystone(crop, verticals, params);
	const auto gradient_lean =
			lean_from_gradients(gray, width, height, crop.x_foot, crop.ppm, &crop.occl);
	double c0_after = lean.c0_deg;
	std::optional<rectify::LooseCrop> warped;
	const rectify::LooseCrop *working_crop = &crop;
	if (lean.flag == shear_ok) {
		warped = corrected_crop(crop, lean.homography);
		gray.clear();
		gray.reserve(warped->rgb.pixels.size());
		for (const auto &pixel : warped->rgb.pixels)
			gray.push_back(static_cast<std::uint8_t>(
					(std::uint32_t(pixel[0]) * 4899 + std::uint32_t(pixel[1]) * 9617 +
							std::uint32_t(pixel[2]) * 1868 + 8192) >>
					14));
		auto families = lsd_families(gray, width, height, warped->ppm, &warped->occl);
		verticals = std::move(families.first);
		horizontals = std::move(families.second);
		c0_after = lean_keystone(*warped, verticals, params).c0_deg;
		working_crop = &*warped;
	}
	const auto &view_crop = *working_crop;
	const std::array<double, 4> bbox{view_crop.s_to_x(0.0),
			view_crop.s_to_x(fitted_length), view_crop.h_to_y(osm_height),
			view_crop.h_to_y(0.0)};
	const auto plane = plane_gate(view_crop, horizontals, bbox, params);
	const auto [ground_dz, ground_flag] =
			ground_row(view_crop, view_crop.h_to_y(0.0), input.ground_source);
	const auto facade_rows = std::pair{
			view_crop.h_to_y(0.8 * osm_height), view_crop.h_to_y(0.3 * osm_height)};
	auto [sky, sky_mode, sky_colour] = sky_mask(view_crop.rgb, facade_rows);
	(void)sky_colour;
	const std::array<double, 2> s_range =
			input.visible_s.value_or(std::array<double, 2>{0.0, fitted_length});
	const auto roof = sky_roofline(view_crop, sky, s_range, wall.height_osm,
			wall.height_source == HeightSource::Default, input.cloud_height, params);
	const std::optional<double> sky_adjusted =
			roof.height ? std::optional<double>(*roof.height - ground_dz) : std::nullopt;
	const std::optional<double> cloud_adjusted =
			input.cloud_height ? std::optional<double>(*input.cloud_height - ground_dz)
							   : std::nullopt;
	const double guessed_height =
			roof.height && roof.flag == roof_sky ? *roof.height : osm_height + ground_dz;
	const std::array<double, 2> profile_rows{
			view_crop.h_to_y(guessed_height - 0.3), view_crop.h_to_y(ground_dz + 0.3)};
	auto [profile, peaks] = column_profile(view_crop, verticals, profile_rows);
	const auto &stats = lean.stats;
	output.details.n_vertical = verticals.size();
	output.details.n_horizontal = horizontals.size();
	output.details.lean_n = stats.n;
	output.details.lean_inliers = stats.inliers;
	output.details.lean_rms_deg = stats.rms_deg;
	output.details.lean_x_spread_m = stats.x_spread_m;
	output.details.c0_after_deg = c0_after;
	output.details.grad_c0_deg = gradient_lean[0];
	output.details.grad_c1_deg_per_m = gradient_lean[1];
	output.details.grad_support = gradient_lean[2];
	output.details.plane_gate_n = plane.lines;
	output.details.sky_mode = sky_mode;
	output.details.h_sky_raw = roof.height;
	output.details.h_cloud = cloud_adjusted;
	output.details.s_range = s_range;
	output.details.box_px = bbox;
	output.details.l_fit = fitted_length;
	output.details.h_osm = osm_height;

	auto &refinement = output.refinement;
	refinement.view_key = view_key(wall.key, input.pano_id);
	refinement.c0_deg = lean.c0_deg;
	refinement.c1_deg_per_m = lean.c1_deg_per_m;
	refinement.lean_flag = lean.flag;
	refinement.h_shear = lean.homography;
	refinement.plane_slope = plane.slope;
	refinement.plane_flag = plane.flag;
	refinement.h_sky = sky_adjusted;
	refinement.roof_flag = roof.flag;
	refinement.roof_flags = roof.flags;
	refinement.roof_spread_m = roof.spread_m;
	refinement.ground_dz = ground_dz;
	refinement.ground_flag = ground_flag;
	refinement.profile = std::move(profile);
	refinement.profile_x0_m = view_crop.s0 + 0.5 / profile_ppm;
	refinement.peaks = std::move(peaks);
	refinement.z_base = crop.z_base + ground_dz;
	return output;
}

std::vector<LineSegment> detect_segments(const std::vector<std::uint8_t> &gray,
		std::size_t width, std::size_t height, double minimum_length_px)
{
	if (width < 4 || height < 4 || gray.size() < width * height)
		return {};

	// Match the Rust LSD preprocessing: blur at full resolution, then downscale
	// before estimating level-line angles.
	const double sigma = lsd_sigma_scale / lsd_scale;
	const auto half_size = static_cast<std::size_t>(
			std::ceil(sigma * std::sqrt(2.0 * 3.0 * std::log(10.0))));
	const auto kernel_size = 1 + 2 * half_size;
	std::vector<float> source(width * height);
	std::transform(gray.begin(), gray.begin() + width * height, source.begin(),
			[](std::uint8_t value) { return static_cast<float>(value); });
	const auto blurred = gaussian_blur(source, width, height, kernel_size, sigma);
	const auto small_width = static_cast<std::size_t>(
			imgops::round_half_even(static_cast<double>(width) * lsd_scale));
	const auto small_height = static_cast<std::size_t>(
			imgops::round_half_even(static_cast<double>(height) * lsd_scale));
	if (small_width < 4 || small_height < 4)
		return {};
	std::vector<std::uint8_t> blurred_bytes(blurred.size());
	std::transform(
			blurred.begin(), blurred.end(), blurred_bytes.begin(), [](float value) {
				return static_cast<std::uint8_t>(
						std::clamp(std::round(value), 0.0f, 255.0f));
			});
	const auto small = resize_bilinear(
			blurred_bytes, width, height, small_width, small_height, 1.0 / lsd_scale);

	constexpr double pi = 3.14159265358979323846;
	const double precision = pi * lsd_angle_threshold / 180.0;
	const double probability = lsd_angle_threshold / 180.0;
	const double threshold = lsd_quant / std::sin(precision);
	auto [field, ordered] = ll_angle(small, small_width, small_height, threshold);
	const double log_nt =
			5.0 * ((std::log10(small_width) + std::log10(small_height)) / 2.0) +
			std::log10(11.0);
	const auto minimum_region_size =
			static_cast<std::size_t>(-log_nt / std::log10(probability));
	std::vector<bool> used(small_width * small_height, false);
	std::vector<RegionPoint> region;
	std::vector<LineSegment> result;
	for (const auto &[x, y] : ordered) {
		const auto index = y * small_width + x;
		if (used[index] || field.angles[index] == not_defined_angle)
			continue;
		const double angle = region_grow(field, used, {x, y}, precision, region);
		if (region.size() < minimum_region_size)
			continue;
		auto rectangle = region_to_rectangle(region, angle, precision, probability);
		if (!refine_lsd_region(
					field, used, region, angle, precision, probability, rectangle))
			continue;
		const double x0 = (rectangle.x1 + 0.5) / lsd_scale;
		const double y0 = (rectangle.y1 + 0.5) / lsd_scale;
		const double x1 = (rectangle.x2 + 0.5) / lsd_scale;
		const double y1 = (rectangle.y2 + 0.5) / lsd_scale;
		const double length = point_distance(x0, y0, x1, y1);
		if (length >= minimum_length_px)
			result.push_back({x0, y0, x1, y1, length, 0.0});
	}
	return result;
}

std::pair<std::vector<LineSegment>, std::vector<LineSegment>> lsd_families(
		const std::vector<std::uint8_t> &gray, std::size_t width, std::size_t height,
		double pixels_per_metre, const std::vector<std::uint8_t> *occlusion)
{
	auto segments = detect_segments(gray, width, height, 0.0);
	std::vector<LineSegment> verticals, horizontals;
	for (const auto &segment : segments) {
		if (occlusion && width && height) {
			const auto x = static_cast<std::size_t>(
					std::clamp<std::int64_t>(static_cast<std::int64_t>(segment.mid_x()),
							0, static_cast<std::int64_t>(width - 1)));
			const auto y = static_cast<std::size_t>(
					std::clamp<std::int64_t>(static_cast<std::int64_t>(segment.mid_y()),
							0, static_cast<std::int64_t>(height - 1)));
			const auto index = y * width + x;
			if (index < occlusion->size() && ((*occlusion)[index] & occluded_bits) != 0)
				continue;
		}

		// Vertical segments are oriented from bottom to top.
		const double vx0 = segment.y0 < segment.y1 ? segment.x1 : segment.x0;
		const double vy0 = segment.y0 < segment.y1 ? segment.y1 : segment.y0;
		const double vx1 = segment.y0 < segment.y1 ? segment.x0 : segment.x1;
		const double vy1 = segment.y0 < segment.y1 ? segment.y0 : segment.y1;
		const double lean =
				std::atan2(vx1 - vx0, vy0 - vy1) * 180.0 / 3.14159265358979323846;
		if (std::abs(lean) < vertical_family_degrees &&
				segment.length >= minimum_segment_metres * pixels_per_metre)
			verticals.push_back({vx0, vy0, vx1, vy1, segment.length, lean});

		// Horizontal segments are oriented from left to right.
		const bool reverse = segment.x0 > segment.x1;
		const double hx0 = reverse ? segment.x1 : segment.x0;
		const double hy0 = reverse ? segment.y1 : segment.y0;
		const double hx1 = reverse ? segment.x0 : segment.x1;
		const double hy1 = reverse ? segment.y0 : segment.y1;
		const double tilt =
				std::atan2(hy0 - hy1, hx1 - hx0) * 180.0 / 3.14159265358979323846;
		if (std::abs(tilt) < horizontal_family_degrees &&
				segment.length >= horizontal_minimum_metres * pixels_per_metre)
			horizontals.push_back({hx0, hy0, hx1, hy1, segment.length, tilt});
	}
	return {std::move(verticals), std::move(horizontals)};
}

Refinement Refinement::empty(const std::string &key, double base_z)
{
	Refinement result;
	result.view_key = key;
	result.z_base = base_z;
	return result;
}

LeanResult lean_keystone(const rectify::LooseCrop &crop,
		const std::vector<LineSegment> &verticals, const Params &params)
{
	LeanResult result;
	result.stats.n = verticals.size();
	if (verticals.size() < 2)
		return result;
	std::vector<double> x, angle, weights;
	x.reserve(verticals.size());
	angle.reserve(verticals.size());
	weights.reserve(verticals.size());
	for (const auto &segment : verticals) {
		x.push_back((segment.mid_x() - crop.x_foot) / crop.ppm);
		angle.push_back(segment.angle_deg);
		weights.push_back(segment.length);
	}
	const double mean_x = weighted_mean(x, weights);
	std::vector<double> deviations;
	deviations.reserve(x.size());
	for (const double value : x)
		deviations.push_back((value - mean_x) * (value - mean_x));
	result.stats.x_spread_m = std::sqrt(weighted_mean(deviations, weights));
	const auto fit = irls_line(
			x, angle, weights, result.stats.x_spread_m >= minimum_lean_x_spread_m);
	result.c0_deg = fit.c0;
	result.c1_deg_per_m = fit.c1;
	result.stats.inliers =
			static_cast<double>(std::count_if(fit.residuals.begin(), fit.residuals.end(),
					[](double residual) {
						return std::abs(residual) <= lean_inlier_degrees;
					})) /
			verticals.size();
	std::vector<double> squared;
	squared.reserve(fit.residuals.size());
	for (const double residual : fit.residuals)
		squared.push_back(residual * residual);
	result.stats.rms_deg = std::sqrt(weighted_mean(squared, weights));
	if (verticals.size() < params.lean_min_lines)
		return result;
	const bool accepted = result.stats.inliers >= params.lean_min_inliers &&
						  std::abs(result.c0_deg) <= params.lean_max_deg &&
						  std::abs(result.c1_deg_per_m) <= params.lean_max_keystone;
	if (!accepted) {
		result.flag = shear_rejected;
		return result;
	}
	result.flag = shear_ok;
	result.homography = shear_homography(crop, result.c0_deg, result.c1_deg_per_m);
	return result;
}

std::array<double, 3> lean_from_gradients(const std::vector<std::uint8_t> &gray,
		std::size_t width, std::size_t height, double x_foot, double ppm,
		const std::vector<std::uint8_t> *occlusion)
{
	if (!width || !height || gray.size() < width * height)
		return {0.0, 0.0, 0.0};
	std::vector<float> source(gray.begin(), gray.begin() + width * height);
	const auto blurred = gaussian_blur(source, width, height, gaussian_ksize(1.0), 1.0);
	const auto gx = sobel(blurred, width, height, true);
	const auto gy = sobel(blurred, width, height, false);
	std::vector<double> magnitude(width * height), theta(width * height);
	for (std::size_t i = 0; i < magnitude.size(); ++i) {
		magnitude[i] = std::hypot(static_cast<double>(gx[i]), static_cast<double>(gy[i]));
		const double sign = gx[i] >= 0.0f ? 1.0 : -1.0;
		theta[i] = std::atan2(static_cast<double>(gy[i]) * sign,
						   std::abs(static_cast<double>(gx[i]))) *
				   (180.0 / 3.14159265358979323846);
	}
	auto sorted = magnitude;
	const double threshold = std::max(imgops::percentile(std::move(sorted), 85.0), 20.0);
	std::vector<double> xs, angles, weights;
	for (std::size_t y = 0; y < height; ++y)
		for (std::size_t x = 0; x < width; ++x) {
			const auto i = y * width + x;
			if (magnitude[i] <= threshold || std::abs(theta[i]) > gradient_max_degrees)
				continue;
			if (occlusion && i < occlusion->size() &&
					((*occlusion)[i] & occluded_bits) != 0)
				continue;
			xs.push_back(static_cast<double>(x));
			angles.push_back(theta[i]);
			weights.push_back(magnitude[i]);
		}
	const double support = xs.size();
	if (support < 50.0)
		return {0.0, 0.0, support};
	const auto bins = static_cast<std::size_t>(
			std::clamp(imgops::round_half_even(width / (3.0 * ppm)), 3.0, 12.0));
	const auto angle_bins = static_cast<std::size_t>(
			imgops::round_half_even((2.0 * gradient_max_degrees) / gradient_bin_degrees));
	std::vector<double> band_x, band_theta, band_weight;
	for (std::size_t band = 0; band < bins; ++band) {
		const double lo = static_cast<double>(width) * band / bins;
		const double hi = static_cast<double>(width) * (band + 1) / bins;
		std::vector<std::size_t> selected;
		for (std::size_t i = 0; i < xs.size(); ++i)
			if (xs[i] >= lo && xs[i] < hi)
				selected.push_back(i);
		if (selected.size() < 50)
			continue;
		std::vector<double> histogram(angle_bins, 0.0);
		for (const auto i : selected) {
			const double bin =
					std::floor((angles[i] + gradient_max_degrees) / gradient_bin_degrees);
			if (bin >= 0.0 && static_cast<std::size_t>(bin) < angle_bins)
				histogram[static_cast<std::size_t>(bin)] += weights[i];
		}
		const auto smoothed = convolve_same(histogram, {0.25, 0.5, 0.25});
		const auto best_bin = argmax_first(smoothed);
		const double centre = -gradient_max_degrees + best_bin * gradient_bin_degrees +
							  0.5 * gradient_bin_degrees;
		std::vector<double> near_x, near_angles, near_weights;
		for (const auto i : selected)
			if (std::abs(angles[i] - centre) <= 1.0) {
				near_x.push_back(xs[i]);
				near_angles.push_back(angles[i]);
				near_weights.push_back(weights[i]);
			}
		if (near_x.size() < 20)
			continue;
		band_theta.push_back(weighted_mean(near_angles, near_weights));
		band_x.push_back(weighted_mean(near_x, near_weights));
		band_weight.push_back(
				std::accumulate(near_weights.begin(), near_weights.end(), 0.0));
	}
	if (band_theta.empty())
		return {0.0, 0.0, support};
	std::vector<double> u;
	u.reserve(band_x.size());
	for (const double x : band_x)
		u.push_back((x - x_foot) / ppm);
	double spread = 0.0;
	if (u.size() > 1) {
		const double mean_u = weighted_mean(u, band_weight);
		std::vector<double> deviations;
		for (const double value : u)
			deviations.push_back((value - mean_u) * (value - mean_u));
		spread = std::sqrt(weighted_mean(deviations, band_weight));
	}
	const auto fit = irls_line(u, band_theta, band_weight,
			u.size() >= 3 && spread >= minimum_lean_x_spread_m);
	return {fit.c0, fit.c1, support};
}

PlaneGateResult plane_gate(const rectify::LooseCrop &crop,
		const std::vector<LineSegment> &horizontals,
		const std::array<double, 4> &pixel_bounds, const Params &params)
{
	if (horizontals.empty())
		return {};
	const auto [x0, x1, y0, y1] = pixel_bounds;
	const double width = x1 - x0, height = y1 - y0;
	std::vector<const LineSegment *> kept;
	for (const auto &line : horizontals) {
		const double x = line.mid_x(), y = line.mid_y();
		if (x >= x0 + 0.1 * width && x <= x1 - 0.1 * width && y >= y0 + 0.1 * height &&
				y <= y1 - 0.1 * height && std::abs(y - crop.y_cam) >= crop.ppm &&
				line.length >= crop.ppm)
			kept.push_back(&line);
	}
	PlaneGateResult result;
	result.lines = kept.size();
	if (kept.empty())
		return result;
	double total_metres = 0.0;
	std::vector<double> line_heights, tilt, base, weight;
	for (const auto *line : kept) {
		total_metres += line->length;
		line_heights.push_back((crop.y_cam - line->mid_y()) / crop.ppm);
		tilt.push_back(line->angle_deg);
		base.push_back(line->length);
		weight.push_back(line->length);
	}
	total_metres /= crop.ppm;
	for (std::size_t iteration = 0; iteration < irls_iterations; ++iteration) {
		double numerator = 0.0, denominator = 0.0;
		for (std::size_t i = 0; i < kept.size(); ++i) {
			numerator += weight[i] * tilt[i] * line_heights[i];
			denominator += weight[i] * line_heights[i] * line_heights[i];
		}
		result.slope = numerator / std::max(denominator, 1e-9);
		for (std::size_t i = 0; i < kept.size(); ++i) {
			const double residual = tilt[i] - result.slope * line_heights[i];
			const double huber =
					std::abs(residual) <= huber_degrees
							? 1.0
							: huber_degrees / std::max(std::abs(residual), 1e-9);
			weight[i] = base[i] * huber;
		}
	}
	if (kept.size() < 8 || total_metres < 15.0)
		return result;
	result.flag = std::abs(result.slope) <= params.plane_gate_deg_per_m ? on_plane
																		: plane_mismatch;
	return result;
}

RoofVote roof_vote(const std::vector<ViewEvidence> &views)
{
	std::array<std::vector<std::pair<double, double>>, 3> ranked;
	for (const auto &view : views) {
		if (!view.refinement || !view.refinement->h_sky ||
				!std::isfinite(*view.refinement->h_sky))
			continue;
		const auto &evidence = *view.refinement;
		const bool sky = evidence.roof_flag == roof_sky;
		if (!sky && evidence.roof_flag != roof_edge)
			continue;
		const bool unconfirmed =
				sky && std::find(evidence.roof_flags.begin(), evidence.roof_flags.end(),
							   roof_unconfirmed) != evidence.roof_flags.end();
		const std::size_t rank = unconfirmed ? 0 : sky ? 2 : 1;
		const double weight = std::isnan(view.score) ? 0.05 : std::max(view.score, 0.05);
		ranked[rank].emplace_back(*evidence.h_sky, weight);
	}
	for (const std::size_t rank : {2U, 1U, 0U}) {
		const auto &group = ranked[rank];
		if (group.empty())
			continue;
		std::vector<double> values, weights;
		values.reserve(group.size());
		weights.reserve(group.size());
		for (const auto &[height, weight] : group) {
			values.push_back(height);
			weights.push_back(weight);
		}
		return {weighted_median(values, weights), rank == 1 ? roof_edge : roof_sky};
	}
	return {};
}

HeightDecision decide_height(std::optional<double> h_sky, std::optional<double> h_cloud,
		std::optional<double> h_osm, bool osm_default, const std::string &osm_source,
		const Params &params, bool roof_blind)
{
	if (h_osm && *h_osm == 0.0)
		h_osm.reset();
	const bool has_osm = h_osm.has_value();
	const bool use_default = osm_default || !has_osm;
	const double osm_height = h_osm.value_or(params.default_height_m);
	const std::string osm_label = use_default ? "default" : osm_source;
	if (h_sky && h_cloud) {
		if (std::abs(*h_sky - *h_cloud) <= 2.0)
			return {*h_sky, "sky+cloud", {}};
		if (!use_default)
			return {osm_height, osm_label, {"HEIGHT_CONFLICT"}};
		return {*h_cloud, "cloud", {"HEIGHT_CONFLICT"}};
	}
	if (h_sky)
		return {*h_sky, "sky", {}};
	if (h_cloud) {
		if (roof_blind && !use_default)
			return {osm_height, osm_label, {}};
		if (use_default || std::abs(*h_cloud - osm_height) > 2.5)
			return {*h_cloud, "cloud", {}};
		return {osm_height, osm_label, {}};
	}
	return {osm_height, osm_label, {}};
}

Extent extent_joint(const std::vector<Peak> &peaks, double x_a, double x_b,
		double osm_length, const ViewEvidence::CropEvidence *crop, bool fix_a, bool fix_b,
		const Params &params)
{
	const double window = params.extent_window_m;
	const double lambda = std::max(params.extent_lambda, 1e-6);
	auto candidates = [&](double end) {
		std::vector<const Peak *> result;
		for (const auto &peak : peaks)
			if (std::abs(peak.x_m - end) <= window && peak.energy >= peak_min_energy &&
					peak.vlen_m >= peak_min_vertical_length_m &&
					std::abs(peak.x_m - (end - window)) >= window_limit_m &&
					std::abs(peak.x_m - (end + window)) >= window_limit_m)
				result.push_back(&peak);
		return result;
	};
	const auto ca = fix_a ? std::vector<const Peak *>{} : candidates(x_a);
	const auto cb = fix_b ? std::vector<const Peak *>{} : candidates(x_b);
	Extent result;
	if (!ca.empty() && !cb.empty()) {
		double best_score = -std::numeric_limits<double>::infinity();
		for (const Peak *left : ca)
			for (const Peak *right : cb) {
				if (right->x_m - left->x_m < 1.0)
					continue;
				const double score =
						left->energy + right->energy -
						std::abs((right->x_m - left->x_m) - osm_length) / lambda;
				if (score > best_score) {
					best_score = score;
					result.s_l = left->x_m;
					result.s_r = right->x_m;
				}
			}
		if (result.s_l && result.s_r)
			result.src_a = result.src_b = "edge-joint";
	}
	if (!result.s_l && !result.s_r) {
		if (!ca.empty()) {
			const double left = ca[strongest_peak(ca)]->x_m;
			result.s_l = left;
			result.src_a = "edge-joint";
			if (!fix_b) {
				result.s_r = x_b + (left - x_a);
				result.src_b = "edge-shift";
			}
		} else if (!cb.empty()) {
			const double right = cb[strongest_peak(cb)]->x_m;
			result.s_r = right;
			result.src_b = "edge-joint";
			if (!fix_a) {
				result.s_l = x_a + (right - x_b);
				result.src_a = "edge-shift";
			}
		}
	}
	if (!result.s_l && !result.s_r)
		return result;
	const double left = result.s_l.value_or(x_a);
	const double right = result.s_r.value_or(x_b);
	if (crop && crop->rgb) {
		const auto c0 = static_cast<std::ptrdiff_t>(
				imgops::round_half_even((left - crop->s0_m) * crop->ppm));
		const auto c1 = static_cast<std::ptrdiff_t>(
				imgops::round_half_even((right - crop->s0_m) * crop->ppm));
		const auto [fires, delta_e] =
				half_consistency(*crop->rgb, crop->occlusion, c0, c1, crop->ppm);
		if (fires) {
			std::vector<const Peak *> interior;
			for (const auto &peak : peaks)
				if (peak.x_m > left + 1.0 && peak.x_m < right - 1.0 &&
						peak.energy >= peak_min_energy &&
						peak.vlen_m >= peak_min_vertical_length_m)
					interior.push_back(&peak);
			if (!interior.empty()) {
				std::ostringstream flag;
				flag << "PARTY_WALL_INSIDE " << std::fixed << std::setprecision(1)
					 << interior[strongest_peak(interior)]->x_m;
				result.flags.push_back(flag.str());
			} else {
				std::ostringstream flag;
				flag << "EXTENT_INCONSISTENT dE " << std::fixed << std::setprecision(1)
					 << delta_e;
				result.flags.push_back(flag.str());
				result.s_l.reset();
				result.s_r.reset();
				result.src_a = result.src_b = "osm";
			}
		}
	}
	return result;
}

WallDecision decide_wall(const Wall &wall, double fitted_length, bool fix_a, bool fix_b,
		const std::vector<ViewEvidence> &views,
		const std::vector<std::string> &extra_flags, const Params &params)
{
	std::vector<double> ends_a, ends_b;
	std::vector<std::string> sources_a, sources_b;
	std::vector<std::string> flags;
	for (const auto &view : views) {
		if (!view.refinement)
			continue;
		std::vector<Peak> shifted;
		shifted.reserve(view.refinement->peaks.size());
		for (auto peak : view.refinement->peaks) {
			peak.x_m -= view.ds_m;
			shifted.push_back(peak);
		}
		ViewEvidence::CropEvidence adjusted_crop;
		const ViewEvidence::CropEvidence *crop = nullptr;
		if (view.crop) {
			adjusted_crop = *view.crop;
			adjusted_crop.s0_m -= view.ds_m;
			crop = &adjusted_crop;
		}
		const auto extent = extent_joint(
				shifted, 0.0, fitted_length, wall.length, crop, fix_a, fix_b, params);
		flags.insert(flags.end(), extent.flags.begin(), extent.flags.end());
		if (extent.s_l) {
			ends_a.push_back(*extent.s_l);
			sources_a.push_back(extent.src_a);
		}
		if (extent.s_r) {
			ends_b.push_back(*extent.s_r);
			sources_b.push_back(extent.src_b);
		}
	}
	auto majority = [](const std::vector<std::string> &values) {
		std::string best;
		std::size_t best_count = 0;
		for (const auto &value : values) {
			const auto count = static_cast<std::size_t>(
					std::count(values.begin(), values.end(), value));
			if (count > best_count) {
				best_count = count;
				best = value;
			}
		}
		return best;
	};
	WallDecision decision;
	decision.wall_key = wall.key;
	if (fix_a) {
		decision.s_left = 0.0;
		decision.source_a = "cloud-corner";
	} else if (!ends_a.empty()) {
		decision.s_left = imgops::median(std::move(ends_a));
		decision.source_a = majority(sources_a);
	} else {
		decision.s_left = 0.0;
		decision.source_a = "osm";
	}
	if (fix_b) {
		decision.s_right = fitted_length;
		decision.source_b = "cloud-corner";
	} else if (!ends_b.empty()) {
		decision.s_right = imgops::median(std::move(ends_b));
		decision.source_b = majority(sources_b);
	} else {
		decision.s_right = fitted_length;
		decision.source_b = "osm";
	}
	if (decision.s_right - decision.s_left < std::max(1.0, 0.5 * fitted_length)) {
		decision.s_left = 0.0;
		decision.s_right = fitted_length;
		decision.source_a = decision.source_b = "osm";
		flags.emplace_back("EXTENT_DEGENERATE");
	}
	const auto roof = roof_vote(views);
	decision.h_sky = roof.height;
	std::vector<double> clouds;
	for (const auto &view : views)
		if (view.h_cloud && std::isfinite(*view.h_cloud))
			clouds.push_back(*view.h_cloud);
	if (!clouds.empty())
		decision.h_cloud = imgops::median(std::move(clouds));
	decision.h_osm = wall.height_osm;
	if (decision.h_osm && *decision.h_osm == 0.0)
		decision.h_osm.reset();
	const bool blind = std::any_of(views.begin(), views.end(), [](const auto &view) {
		if (!view.refinement)
			return false;
		const auto &view_flags = view.refinement->roof_flags;
		return std::find(view_flags.begin(), view_flags.end(), roof_nodata) !=
					   view_flags.end() &&
			   std::find(view_flags.begin(), view_flags.end(), roof_behind) !=
					   view_flags.end();
	});
	const auto height = decide_height(decision.h_sky, decision.h_cloud, decision.h_osm,
			wall.height_source == HeightSource::Default,
			height_source_name(wall.height_source), params, blind);
	decision.h_used = height.height;
	decision.height_source = height.source;
	flags.insert(flags.end(), height.flags.begin(), height.flags.end());
	flags.push_back(roof.source);
	if (!views.empty() && views.front().refinement) {
		flags.push_back(views.front().refinement->lean_flag);
		flags.push_back(views.front().refinement->ground_flag);
		const auto mismatches = static_cast<std::size_t>(
				std::count_if(views.begin(), views.end(), [](const auto &view) {
					return view.refinement &&
						   view.refinement->plane_flag == plane_mismatch;
				}));
		if (mismatches > 0 && mismatches * 2 >= views.size())
			flags.emplace_back(plane_mismatch);
		else
			flags.push_back(views.front().refinement->plane_flag);
	}
	flags.insert(flags.end(), extra_flags.begin(), extra_flags.end());
	for (const auto &flag : flags)
		if (std::find(decision.flags.begin(), decision.flags.end(), flag) ==
				decision.flags.end())
			decision.flags.push_back(flag);
	return decision;
}

std::pair<std::size_t, std::size_t> colour_trim(const projection::Image &texture,
		const std::vector<bool> &valid, std::size_t pixels_per_block,
		const std::array<double, 3> &wall_colour, std::size_t max_blocks, double delta_e)
{
	if (!pixels_per_block || !texture.width || !texture.height ||
			texture.pixels.size() < std::size_t(texture.width) * texture.height)
		return {0, 0};
	const auto width = static_cast<std::size_t>(texture.width);
	const auto height = static_cast<std::size_t>(texture.height);
	const auto columns = width / pixels_per_block;
	if (columns < 2 * max_blocks + 1)
		return {0, 0};
	std::vector<std::optional<std::array<double, 3>>> medians;
	std::vector<double> valid_shares;
	medians.reserve(columns);
	valid_shares.reserve(columns);
	for (std::size_t block_x = 0; block_x < columns; ++block_x) {
		std::vector<imgops::Lab> samples;
		std::size_t total = 0;
		for (std::size_t y = 0; y < height; ++y)
			for (std::size_t x = block_x * pixels_per_block;
					x < (block_x + 1) * pixels_per_block; ++x) {
				const auto index = y * width + x;
				++total;
				if (index < valid.size() && valid[index])
					samples.push_back(imgops::srgb_to_oklab(texture.pixels[index]));
			}
		valid_shares.push_back(total ? static_cast<double>(samples.size()) / total : 0.0);
		if (samples.empty())
			medians.emplace_back(std::nullopt);
		else
			medians.emplace_back(median_oklab(samples));
	}
	auto count_from = [&](bool reverse) {
		std::size_t count = 0;
		for (std::size_t offset = 0; offset < columns; ++offset) {
			const auto column = reverse ? columns - 1 - offset : offset;
			if (count >= max_blocks || valid_shares[column] < 0.2)
				break;
			if (medians[column] &&
					imgops::oklab_distance(*medians[column], wall_colour) > delta_e)
				++count;
			else
				break;
		}
		return count;
	};
	return {count_from(false), count_from(true)};
}

std::array<double, 3> wall_medoid(
		const projection::Image &texture, const std::vector<bool> &valid)
{
	const auto width = static_cast<std::size_t>(texture.width);
	const auto height = static_cast<std::size_t>(texture.height);
	const auto x0 = static_cast<std::size_t>(0.2 * width);
	const auto x1 =
			std::min(width, std::max(x0 + 1, static_cast<std::size_t>(0.8 * width)));
	std::vector<imgops::Lab> samples;
	if (texture.pixels.size() >= width * height)
		for (std::size_t y = 0; y < height; ++y)
			for (std::size_t x = x0; x < x1; ++x) {
				const auto index = y * width + x;
				if (index < valid.size() && valid[index])
					samples.push_back(imgops::srgb_to_oklab(texture.pixels[index]));
			}
	return samples.empty() ? std::array<double, 3>{0.6, 0.0, 0.0} : median_oklab(samples);
}

std::array<double, 3> phase_search(const projection::Image &texture,
		const std::vector<bool> &valid, std::size_t pixels_per_block,
		const Params &params)
{
	const auto width = static_cast<std::size_t>(texture.width);
	const auto height = static_cast<std::size_t>(texture.height);
	if (!pixels_per_block || !width || !height || texture.pixels.size() < width * height)
		return {0.0, 0.0, 0.0};
	std::vector<double> lightness, blue;
	lightness.reserve(width * height);
	blue.reserve(width * height);
	for (const auto &pixel : texture.pixels) {
		const auto lab = imgops::srgb_to_oklab(pixel);
		lightness.push_back(lab[0]);
		blue.push_back(lab[2]);
	}
	const auto k = static_cast<int>(
			imgops::round_half_even(params.phase_range_m / params.phase_step_m));
	std::vector<double> shifts;
	for (int i = -k; i <= k; ++i)
		shifts.push_back(i * params.phase_step_m);
	std::array<double, 3> best{0.0, 0.0, -1.0};
	for (const double dy : shifts)
		for (const double dx : shifts) {
			auto offset = [&](double d) {
				const auto pixel_shift = static_cast<std::int64_t>(
						imgops::round_half_even(d * pixels_per_block));
				const auto mod =
						pixel_shift % static_cast<std::int64_t>(pixels_per_block);
				return static_cast<std::size_t>(
						(mod + static_cast<std::int64_t>(pixels_per_block)) %
						static_cast<std::int64_t>(pixels_per_block));
			};
			const double score = bimodality(lightness, blue, valid, width, height,
					pixels_per_block, offset(dx), offset(dy));
			const auto candidate =
					std::pair{round6(score), -(std::abs(dx) + std::abs(dy))};
			const auto incumbent =
					std::pair{round6(best[2]), -(std::abs(best[0]) + std::abs(best[1]))};
			if (candidate > incumbent)
				best = {dx, dy, score};
		}
	return best;
}

void finalise_wall(WallDecision &decision, const projection::Image &texture,
		const std::vector<bool> &valid, std::size_t pixels_per_block,
		std::optional<std::array<double, 3>> medoid, const Params &params)
{
	if (!medoid)
		medoid = wall_medoid(texture, valid);
	auto [trim_a, trim_b] =
			colour_trim(texture, valid, pixels_per_block, *medoid, 2, 0.08);
	if (decision.source_a != "osm")
		trim_a = 0;
	if (decision.source_b != "osm")
		trim_b = 0;
	const double raw_columns =
			imgops::round_half_even(decision.s_right - decision.s_left);
	const auto columns = raw_columns > 0.0 ? static_cast<std::size_t>(raw_columns) : 0;
	if (trim_a + trim_b + 1 >= columns)
		trim_a = trim_b = 0;
	decision.trimmed_a = static_cast<std::int32_t>(trim_a);
	decision.trimmed_b = static_cast<std::int32_t>(trim_b);
	decision.s_left += trim_a;
	decision.s_right -= trim_b;
	const auto width = static_cast<std::size_t>(texture.width);
	const auto height = static_cast<std::size_t>(texture.height);
	const auto x0 = trim_a * pixels_per_block;
	const auto x1 =
			width > trim_b * pixels_per_block ? width - trim_b * pixels_per_block : 0;
	projection::Image cropped;
	std::vector<bool> cropped_valid;
	const projection::Image *phase_texture = &texture;
	const std::vector<bool> *phase_valid = &valid;
	if (x0 != 0 || x1 != width) {
		cropped.width = static_cast<unsigned>(x1 - x0);
		cropped.height = static_cast<unsigned>(height);
		cropped.pixels.reserve((x1 - x0) * height);
		cropped_valid.reserve((x1 - x0) * height);
		for (std::size_t y = 0; y < height; ++y)
			for (std::size_t x = x0; x < x1; ++x) {
				const auto index = y * width + x;
				cropped.pixels.push_back(texture.pixels[index]);
				cropped_valid.push_back(index < valid.size() && valid[index]);
			}
		phase_texture = &cropped;
		phase_valid = &cropped_valid;
	}
	const auto phase =
			phase_search(*phase_texture, *phase_valid, pixels_per_block, params);
	decision.phase = {phase[0], phase[1]};
	decision.bimodality = phase[2];
	if (trim_a || trim_b)
		decision.flags.push_back(
				"TRIMMED " + std::to_string(trim_a) + "/" + std::to_string(trim_b));
}

std::array<double, 4> final_rect(const WallDecision &decision)
{
	return {decision.s_left, decision.s_right, 0.0, decision.h_used};
}

} // namespace arnis::mapillary::refine
