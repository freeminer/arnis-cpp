#pragma once

#include "project.h"
#include "imgops.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace arnis::mapillary
{
struct Params;
}

namespace arnis::mapillary::fuse
{
inline constexpr double border_falloff_m = 1.0;
inline constexpr double border_floor = .15;
inline constexpr double min_weight_sum = .25;
inline constexpr double min_phase_response = .03;
inline constexpr std::size_t min_overlap_px = 256;
inline constexpr double agreement_max_m = .75;
inline constexpr double max_shift_m = 1.5;
inline constexpr double hole_max_m = 1.5;
inline constexpr int inpaint_radius_px = 3;
inline constexpr double pi = 3.141592653589793238462643383279502884;
inline constexpr std::uint8_t telea_known = 0;
inline constexpr std::uint8_t telea_band = 1;
inline constexpr std::uint8_t telea_inside = 2;
inline constexpr std::uint8_t telea_change = 3;

enum class FuseMode
{
	Fused,
	BestView,
	Single
};
inline const char *fuse_mode_name(FuseMode mode)
{
	switch (mode) {
	case FuseMode::Fused:
		return "fused";
	case FuseMode::BestView:
		return "best_view";
	case FuseMode::Single:
		return "single";
	}
	return "single";
}

struct FusedTexture
{
	std::string wall_key;
	projection::Image rgb;
	std::vector<bool> valid;
	double ppm = 0.0;
	std::vector<std::array<double, 2>> shifts;
	double agreement_m = 0.0;
	FuseMode mode = FuseMode::Single;
	std::size_t best_index = 0;
	double hole_fraction = 0.0;
	std::vector<bool> filled;
	std::vector<std::string> flags;
};

inline std::size_t optimal_dft_size(std::size_t n)
{
	for (std::size_t m = std::max<std::size_t>(n, 1);; ++m) {
		std::size_t remainder = m;
		for (const std::size_t factor : {2, 3, 5})
			while (remainder % factor == 0)
				remainder /= factor;
		if (remainder == 1)
			return m;
	}
}

inline std::size_t smallest_factor(std::size_t n)
{
	for (std::size_t factor = 2; factor * factor <= n; ++factor)
		if (n % factor == 0)
			return factor;
	return n;
}

inline std::vector<std::complex<double>> dft_1d(
		const std::vector<std::complex<double>> &input, double sign,
		const std::vector<std::complex<double>> &roots)
{
	const std::size_t n = input.size();
	if (n <= 1)
		return input;
	const std::size_t radix = smallest_factor(n), sub_size = n / radix;
	const std::size_t step = roots.size() / n;
	std::vector<std::vector<std::complex<double>>> subtransforms;
	if (radix != n) {
		subtransforms.reserve(radix);
		for (std::size_t r = 0; r < radix; ++r) {
			std::vector<std::complex<double>> sub(sub_size);
			for (std::size_t k = 0; k < sub_size; ++k)
				sub[k] = input[k * radix + r];
			subtransforms.push_back(dft_1d(sub, sign, roots));
		}
	}
	std::vector<std::complex<double>> out(n);
	if (radix == n) {
		for (std::size_t k = 0; k < n; ++k)
			for (std::size_t j = 0; j < n; ++j)
				out[k] += input[j] * roots[(j * k * step) % roots.size()];
		return out;
	}
	for (std::size_t k = 0; k < n; ++k)
		for (std::size_t r = 0; r < radix; ++r)
			out[k] +=
					subtransforms[r][k % sub_size] * roots[(r * k * step) % roots.size()];
	return out;
}

inline std::vector<std::complex<double>> roots_of_unity(std::size_t n, double sign)
{
	std::vector<std::complex<double>> roots(n);
	for (std::size_t k = 0; k < n; ++k) {
		const double angle = sign * 2.0 * pi * static_cast<double>(k) / n;
		roots[k] = {std::cos(angle), std::sin(angle)};
	}
	return roots;
}

inline void dft_2d(std::vector<std::complex<double>> &data, std::size_t width,
		std::size_t height, double sign)
{
	const auto roots_x = roots_of_unity(width, sign);
	const auto roots_y = roots_of_unity(height, sign);
	for (std::size_t y = 0; y < height; ++y) {
		std::vector<std::complex<double>> row(
				data.begin() + y * width, data.begin() + (y + 1) * width);
		const auto transformed = dft_1d(row, sign, roots_x);
		std::copy(transformed.begin(), transformed.end(), data.begin() + y * width);
	}
	for (std::size_t x = 0; x < width; ++x) {
		std::vector<std::complex<double>> column(height);
		for (std::size_t y = 0; y < height; ++y)
			column[y] = data[y * width + x];
		const auto transformed = dft_1d(column, sign, roots_y);
		for (std::size_t y = 0; y < height; ++y)
			data[y * width + x] = transformed[y];
	}
}

inline std::vector<double> hanning_window(std::size_t width, std::size_t height)
{
	std::vector<double> out(width * height);
	for (std::size_t y = 0; y < height; ++y) {
		const double wy =
				height <= 1 ? 0.0 : 0.5 * (1.0 - std::cos(2.0 * pi * y / (height - 1)));
		for (std::size_t x = 0; x < width; ++x) {
			const double wx =
					width <= 1 ? 0.0 : 0.5 * (1.0 - std::cos(2.0 * pi * x / (width - 1)));
			out[y * width + x] = std::sqrt(std::max(wy * wx, 0.0));
		}
	}
	return out;
}

inline std::array<double, 3> phase_correlate(const std::vector<double> &a,
		const std::vector<double> &b, std::size_t width, std::size_t height)
{
	if (!width || !height || width > std::numeric_limits<std::size_t>::max() / height ||
			width * height < 16 || a.size() < width * height || b.size() < width * height)
		return {0.0, 0.0, 0.0};
	if (*std::max_element(a.begin(), a.begin() + width * height) <= 0.0 ||
			*std::max_element(b.begin(), b.begin() + width * height) <= 0.0)
		return {0.0, 0.0, 0.0};
	const std::size_t padded_width = optimal_dft_size(width);
	const std::size_t padded_height = optimal_dft_size(height);
	const auto window = hanning_window(width, height);
	using Complex = std::complex<double>;
	std::vector<Complex> fa(padded_width * padded_height),
			fb(padded_width * padded_height);
	for (std::size_t y = 0; y < height; ++y)
		for (std::size_t x = 0; x < width; ++x) {
			const auto source = y * width + x;
			const auto target = y * padded_width + x;
			fa[target] = a[source] * window[source];
			fb[target] = b[source] * window[source];
		}
	dft_2d(fa, padded_width, padded_height, -1.0);
	dft_2d(fb, padded_width, padded_height, -1.0);
	std::vector<Complex> cross(padded_width * padded_height);
	for (std::size_t i = 0; i < cross.size(); ++i) {
		const auto product = fa[i] * std::conj(fb[i]);
		const double magnitude = std::abs(product);
		cross[i] =
				product * (magnitude / (magnitude * magnitude +
											   std::numeric_limits<double>::epsilon()));
	}
	dft_2d(cross, padded_width, padded_height, 1.0);
	std::vector<double> surface(cross.size());
	for (std::size_t y = 0; y < padded_height; ++y)
		for (std::size_t x = 0; x < padded_width; ++x)
			surface[((y + padded_height / 2) % padded_height) * padded_width +
					(x + padded_width / 2) % padded_width] =
					cross[y * padded_width + x].real();
	const auto peak = static_cast<std::size_t>(
			std::max_element(surface.begin(), surface.end()) - surface.begin());
	const auto peak_x = static_cast<std::ptrdiff_t>(peak % padded_width);
	const auto peak_y = static_cast<std::ptrdiff_t>(peak / padded_width);
	const auto min_x = std::max<std::ptrdiff_t>(0, peak_x - 2);
	const auto max_x = std::min<std::ptrdiff_t>(padded_width - 1, peak_x + 2);
	const auto min_y = std::max<std::ptrdiff_t>(0, peak_y - 2);
	const auto max_y = std::min<std::ptrdiff_t>(padded_height - 1, peak_y + 2);
	double weighted_x = 0.0, weighted_y = 0.0, sum = 0.0;
	for (auto y = min_y; y <= max_y; ++y)
		for (auto x = min_x; x <= max_x; ++x) {
			const double value = surface[static_cast<std::size_t>(y) * padded_width + x];
			weighted_x += static_cast<double>(x) * value;
			weighted_y += static_cast<double>(y) * value;
			sum += value;
		}
	const double response = sum / static_cast<double>(padded_width * padded_height);
	const double denominator = sum + std::numeric_limits<double>::epsilon();
	return {padded_width * 0.5 - weighted_x / denominator,
			padded_height * 0.5 - weighted_y / denominator, response};
}

struct ViewTexture
{
	std::string pano_id;
	projection::Image rgb;
	std::vector<bool> valid;
	double score = 0.0;
};

struct Alignment
{
	std::vector<projection::Image> images;
	std::vector<std::vector<bool>> masks;
	std::vector<std::array<double, 2>> shifts;
	double agreement_m = 0.0;
	std::vector<bool> usable;
};

inline std::vector<double> gradient_magnitude(
		const projection::Image &image, const std::vector<bool> *valid);
inline projection::Image shift_image(
		const projection::Image &image, double dx, double dy);
inline std::vector<bool> shift_mask(const std::vector<bool> &mask, std::size_t w,
		std::size_t h, double dx, double dy);

struct HoleFill
{
	projection::Image rgb;
	std::vector<bool> valid;
	double fraction = 0.0;
	std::vector<bool> filled;
};

HoleFill fill_holes(const projection::Image &rgb, const std::vector<bool> &valid,
		unsigned pixels_per_block, double maximum_hole_m);
FusedTexture fuse(const std::string &wall_key, const std::vector<ViewTexture> &views,
		unsigned pixels_per_block, const ::arnis::mapillary::Params &params);

inline Alignment align_views(const std::vector<ViewTexture> &views,
		unsigned pixels_per_block, double maximum_shift_m, std::size_t reference_index)
{
	Alignment result;
	if (views.empty() || reference_index >= views.size() || !pixels_per_block)
		return result;
	const auto width = views[reference_index].rgb.width;
	const auto height = views[reference_index].rgb.height;
	const auto count = std::size_t(width) * height;
	if (!width || !height || views[reference_index].valid.size() != count ||
			views[reference_index].rgb.pixels.size() < count)
		return result;
	const auto &reference_valid = views[reference_index].valid;
	const auto reference_gradient =
			gradient_magnitude(views[reference_index].rgb, &reference_valid);
	std::vector<double> residuals;
	std::size_t correlated = 0;
	result.images.reserve(views.size());
	result.masks.reserve(views.size());
	result.shifts.reserve(views.size());
	result.usable.assign(views.size(), true);
	for (std::size_t i = 0; i < views.size(); ++i) {
		const auto &view = views[i];
		if (view.rgb.width != width || view.rgb.height != height ||
				view.rgb.pixels.size() < count || view.valid.size() != count) {
			result.images.push_back(view.rgb);
			result.masks.push_back(view.valid);
			result.shifts.push_back({0.0, 0.0});
			result.usable[i] = false;
			continue;
		}
		if (i == reference_index) {
			result.images.push_back(view.rgb);
			result.masks.push_back(view.valid);
			result.shifts.push_back({0.0, 0.0});
			continue;
		}
		std::vector<bool> overlap(count);
		std::size_t overlap_count = 0;
		for (std::size_t k = 0; k < count; ++k) {
			overlap[k] = reference_valid[k] && view.valid[k];
			overlap_count += overlap[k] ? 1 : 0;
		}
		if (overlap_count < min_overlap_px) {
			result.images.push_back(view.rgb);
			result.masks.push_back(view.valid);
			result.shifts.push_back({0.0, 0.0});
			continue;
		}
		const auto gradient = gradient_magnitude(view.rgb, &view.valid);
		std::vector<double> a(count), b(count);
		for (std::size_t k = 0; k < count; ++k)
			if (overlap[k]) {
				a[k] = reference_gradient[k];
				b[k] = gradient[k];
			}
		const auto correlation = phase_correlate(a, b, width, height);
		const double dx = correlation[0], dy = correlation[1];
		const double du = dx / pixels_per_block, dv = dy / pixels_per_block;
		const double shift_m = std::hypot(du, dv);
		++correlated;
		if (correlation[2] < min_phase_response)
			result.usable[i] = false;
		else
			residuals.push_back(shift_m);
		if (result.usable[i] && shift_m <= maximum_shift_m) {
			result.images.push_back(shift_image(view.rgb, -dx, -dy));
			result.masks.push_back(shift_mask(view.valid, width, height, -dx, -dy));
		} else {
			result.images.push_back(view.rgb);
			result.masks.push_back(view.valid);
		}
		result.shifts.push_back({du, dv});
	}
	if (!residuals.empty())
		result.agreement_m = imgops::median(std::move(residuals));
	else if (correlated)
		result.agreement_m = maximum_shift_m;
	return result;
}

inline std::size_t reflect101(long value, std::size_t size)
{
	if (size <= 1)
		return 0;
	long period = 2 * long(size - 1), index = value % period;
	if (index < 0)
		index += period;
	return std::size_t(index >= long(size) ? period - index : index);
}
inline double grey(const projection::Rgb &pixel)
{
	const auto accumulator = std::int64_t(pixel[0]) * 9798 +
							 std::int64_t(pixel[1]) * 19235 +
							 std::int64_t(pixel[2]) * 3735 + 16384;
	return static_cast<double>(accumulator >> 15);
}
inline std::vector<double> gradient_magnitude(
		const projection::Image &image, const std::vector<bool> *valid = nullptr)
{
	std::size_t w = image.width, h = image.height;
	std::vector<double> result(w * h);
	if (image.pixels.size() < w * h)
		return result;
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x) {
			double gx = 0, gy = 0;
			for (long dy = -1; dy <= 1; ++dy)
				for (long dx = -1; dx <= 1; ++dx) {
					double value = grey(image.pixels[reflect101(long(y) + dy, h) * w +
													 reflect101(long(x) + dx, w)]);
					int sx = dx == 0 ? 0 : (dx < 0 ? -1 : 1),
						sy = dy == 0 ? 0 : (dy < 0 ? -1 : 1);
					int wx = dy == 0 ? 2 : 1, wy = dx == 0 ? 2 : 1;
					gx += sx * wx * value;
					gy += sy * wy * value;
				}
			result[y * w + x] = std::hypot(gx, gy);
		}
	if (valid && valid->size() == result.size()) {
		// Mask edges must not become artificial high-gradient edges during
		// phase correlation; Rust erodes valid coverage by 3x3 here.
		const auto interior = imgops::erode(imgops::Mask::from_bits(w, h, *valid), 3, 3);
		for (std::size_t i = 0; i < result.size(); ++i)
			if (!interior.bits[i])
				result[i] = 0;
	}
	return result;
}
inline std::vector<bool> shift_mask(
		const std::vector<bool> &mask, std::size_t w, std::size_t h, double dx, double dy)
{
	if (mask.size() != w * h)
		return {};
	if (std::abs(dx) < 1e-9 && std::abs(dy) < 1e-9)
		return mask;
	const auto rounded_shift = [](double d) {
		const auto fixed =
				static_cast<std::int64_t>(imgops::round_half_even(-d * 1024.0));
		const auto value = fixed + 512;
		const auto euclidean = value >= 0 ? value / 1024 : -((-value + 1023) / 1024);
		return -euclidean;
	};
	const auto qx = rounded_shift(dx), qy = rounded_shift(dy);
	std::vector<bool> out(w * h, false);
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x) {
			const auto sx = static_cast<std::int64_t>(x) - qx;
			const auto sy = static_cast<std::int64_t>(y) - qy;
			if (sx >= 0 && sy >= 0 && sx < static_cast<std::int64_t>(w) &&
					sy < static_cast<std::int64_t>(h))
				out[y * w + x] = mask[static_cast<std::size_t>(sy) * w +
									  static_cast<std::size_t>(sx)];
		}
	return out;
}
inline std::vector<double> distance_to_false(
		const std::vector<bool> &mask, std::size_t w, std::size_t h)
{
	if (!w || !h || mask.size() != w * h)
		return {};
	const double inf = std::numeric_limits<double>::infinity();
	std::vector<double> field(w * h);
	for (std::size_t i = 0; i < field.size(); ++i)
		field[i] = mask[i] ? inf : 0.0;
	auto transform = [inf](const std::vector<double> &f) {
		const std::size_t n = f.size();
		std::vector<double> out(n, 0.0);
		if (!n)
			return out;
		std::vector<std::size_t> sites(n);
		std::vector<double> boundary(n + 1);
		std::size_t k = 0;
		sites[0] = 0;
		boundary[0] = -inf;
		boundary[1] = inf;
		for (std::size_t q = 1; q < n; ++q) {
			if (!std::isfinite(f[q]))
				continue;
			double s;
			for (;;) {
				const auto p = sites[k];
				s = !std::isfinite(f[p]) ? -inf
										 : ((f[q] + double(q) * double(q)) -
												   (f[p] + double(p) * double(p))) /
												   (2.0 * double(q) - 2.0 * double(p));
				if (s > boundary[k] || k == 0)
					break;
				--k;
			}
			if (s <= boundary[k]) {
				sites[0] = q;
				boundary[0] = -inf;
				boundary[1] = inf;
				k = 0;
			} else {
				++k;
				sites[k] = q;
				boundary[k] = s;
				boundary[k + 1] = inf;
			}
		}
		k = 0;
		for (std::size_t q = 0; q < n; ++q) {
			while (boundary[k + 1] < double(q))
				++k;
			const auto p = sites[k];
			const double d = double(q) - double(p);
			out[q] = std::isfinite(f[p]) ? d * d + f[p] : inf;
		}
		return out;
	};
	for (std::size_t x = 0; x < w; ++x) {
		std::vector<double> line(h);
		for (std::size_t y = 0; y < h; ++y)
			line[y] = field[y * w + x];
		const auto d = transform(line);
		for (std::size_t y = 0; y < h; ++y)
			field[y * w + x] = d[y];
	}
	for (std::size_t y = 0; y < h; ++y) {
		std::vector<double> line(w);
		for (std::size_t x = 0; x < w; ++x)
			line[x] = field[y * w + x];
		const auto d = transform(line);
		for (std::size_t x = 0; x < w; ++x)
			field[y * w + x] = std::sqrt(std::max(d[x], 0.0));
	}
	return field;
}
inline projection::Image shift_image(const projection::Image &image, double dx, double dy)
{
	projection::Image out{
			image.width, image.height, std::vector<projection::Rgb>(image.pixels.size())};
	const auto quantised_shift = [](double d) {
		const auto fixed =
				static_cast<std::int64_t>(imgops::round_half_even(-d * 1024.0));
		const auto value = fixed + 16;
		const auto rounded = value >= 0 ? value / 32 : -((-value + 31) / 32);
		return static_cast<double>(rounded) / 32.0;
	};
	const double qx = quantised_shift(dx), qy = quantised_shift(dy);
	for (unsigned y = 0; y < image.height; ++y)
		for (unsigned x = 0; x < image.width; ++x) {
			double sx = x + qx, sy = y + qy;
			long x0 = long(std::floor(sx)), y0 = long(std::floor(sy));
			double tx = sx - x0, ty = sy - y0;
			projection::Rgb pixel{};
			for (unsigned c = 0; c < 3; ++c) {
				auto at = [&](long px, long py) {
					return px < 0 || py < 0 || px >= long(image.width) ||
										   py >= long(image.height)
								   ? 0.0
								   : image.pixels[std::size_t(py) * image.width + px][c];
				};
				pixel[c] = std::uint8_t(std::clamp(
						imgops::round_half_even(at(x0, y0) * (1 - tx) * (1 - ty) +
												at(x0 + 1, y0) * tx * (1 - ty) +
												at(x0, y0 + 1) * (1 - tx) * ty +
												at(x0 + 1, y0 + 1) * tx * ty),
						0.0, 255.0));
			}
			out.pixels[std::size_t(y) * image.width + x] = pixel;
		}
	return out;
}
inline std::vector<double> border_weight(
		const std::vector<bool> &valid, unsigned w, unsigned h, double pixels_per_metre)
{
	std::vector<double> result(std::size_t(w) * h);
	if (!w || !h || valid.size() != result.size())
		return result;
	// Pad by invalid pixels so the raster boundary is treated exactly like an
	// interior hole, then measure Euclidean distance to the nearest invalid cell.
	const auto padded_width = std::size_t(w) + 2;
	const auto padded_height = std::size_t(h) + 2;
	std::vector<bool> padded(padded_width * padded_height, false);
	for (unsigned y = 0; y < h; ++y)
		for (unsigned x = 0; x < w; ++x)
			padded[(std::size_t(y) + 1) * padded_width + x + 1] =
					valid[std::size_t(y) * w + x];
	const auto distance = distance_to_false(padded, padded_width, padded_height);
	for (unsigned y = 0; y < h; ++y)
		for (unsigned x = 0; x < w; ++x) {
			std::size_t i = std::size_t(y) * w + x;
			if (!valid[i])
				continue;
			double t =
					std::clamp(distance[(std::size_t(y) + 1) * padded_width + x + 1] /
									   std::max(1.0, border_falloff_m * pixels_per_metre),
							0.0, 1.0);
			result[i] =
					border_floor +
					(1 - border_floor) * (.5 - .5 * std::cos(3.1415926535897932385 * t));
		}
	return result;
}
inline projection::Image match_exposure(const projection::Image &reference,
		const projection::Image &other, const std::vector<bool> &overlap)
{
	const std::size_t count = std::size_t(other.width) * other.height;
	if (reference.width != other.width || reference.height != other.height ||
			reference.pixels.size() < count || other.pixels.size() < count ||
			overlap.size() < count ||
			std::count(overlap.begin(), overlap.begin() + count, true) < 64)
		return other;
	std::vector<imgops::Lab> ref_lab(count), other_lab(count);
	imgops::Lab mean_ref{}, mean_other{};
	std::size_t n = 0;
	for (std::size_t i = 0; i < count; ++i) {
		ref_lab[i] = imgops::srgb_to_oklab(reference.pixels[i]);
		other_lab[i] = imgops::srgb_to_oklab(other.pixels[i]);
		if (!overlap[i])
			continue;
		++n;
		for (std::size_t c = 0; c < 3; ++c) {
			mean_ref[c] += ref_lab[i][c];
			mean_other[c] += other_lab[i][c];
		}
	}
	for (std::size_t c = 0; c < 3; ++c) {
		mean_ref[c] /= static_cast<double>(n);
		mean_other[c] /= static_cast<double>(n);
	}
	double variance_ref = 0.0, variance_other = 0.0;
	for (std::size_t i = 0; i < count; ++i) {
		if (!overlap[i])
			continue;
		variance_ref += std::pow(ref_lab[i][0] - mean_ref[0], 2);
		variance_other += std::pow(other_lab[i][0] - mean_other[0], 2);
	}
	const double sd_ref = std::sqrt(variance_ref / static_cast<double>(n));
	const double sd_other = std::sqrt(variance_other / static_cast<double>(n));
	const double gain = sd_ref > 1e-4 && sd_other > 1e-4
								? std::clamp(sd_ref / sd_other, 0.5, 2.0)
								: 1.0;
	projection::Image result{
			other.width, other.height, std::vector<projection::Rgb>(count)};
	for (std::size_t i = 0; i < count; ++i) {
		const imgops::Lab adjusted{(other_lab[i][0] - mean_other[0]) * gain + mean_ref[0],
				other_lab[i][1] - mean_other[1] + mean_ref[1],
				other_lab[i][2] - mean_other[2] + mean_ref[2]};
		result.pixels[i] = imgops::oklab_to_rgb8(adjusted);
	}
	return result;
}
inline std::pair<projection::Image, std::vector<bool>> weighted_median(
		const std::vector<projection::Image> &images,
		const std::vector<std::vector<double>> &weights, double minimum_weight)
{
	if (images.empty())
		return {};
	auto result = images.front();
	std::vector<bool> valid(result.pixels.size());
	for (std::size_t i = 0; i < result.pixels.size(); ++i) {
		double total = 0;
		double best = 0.0;
		for (const auto &weight : weights)
			if (i < weight.size()) {
				total += weight[i];
				best = std::max(best, weight[i]);
			}
		const double floor = best > 0.0 ? std::min(minimum_weight, minimum_weight * best)
										: minimum_weight;
		valid[i] = total > 0.0 && total >= floor;
		if (!valid[i]) {
			result.pixels[i] = {};
			continue;
		}
		for (unsigned c = 0; c < 3; ++c) {
			std::vector<std::pair<double, double>> entries;
			for (std::size_t v = 0; v < images.size(); ++v)
				entries.emplace_back(images[v].pixels[i][c],
						i < weights[v].size() ? weights[v][i] : 0);
			std::sort(entries.begin(), entries.end());
			double sum = 0;
			for (const auto &[value, weight] : entries) {
				sum += weight;
				if (sum >= total * .5) {
					result.pixels[i][c] = static_cast<std::uint8_t>(
							std::clamp(imgops::round_half_even(value), 0.0, 255.0));
					break;
				}
			}
		}
	}
	return {std::move(result), std::move(valid)};
}
} // namespace arnis::mapillary::fuse
