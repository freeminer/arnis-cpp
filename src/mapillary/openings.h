#pragma once
#include "imgops.h"
#include "project.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
namespace arnis::mapillary::openings
{
inline constexpr std::size_t pixels_per_metre = 8;
inline constexpr std::size_t subcell_pixels = 4;
inline constexpr std::size_t reference_window_subcells = 7;
inline constexpr double dark_threshold = .12;
inline constexpr double chroma_threshold = .045;
inline constexpr double min_width_m = .5, min_height_m = .4, min_fill = .45;
inline constexpr double door_min_width_m = .8, door_max_width_m = 2.2;
inline constexpr double door_min_height_m = 1.5, shop_min_height_m = 1.2;
inline constexpr double shop_max_height_m = 5.0, plinth_max_height_m = 2.0;
inline constexpr double base_tolerance_m = 1.5;
inline constexpr double large_width_m = 4.5, large_height_m = 3.5;
inline constexpr double split_width_m = 2.5, split_height_m = 2.8;
inline constexpr double narrow_width_m = 1.8, deep_glass = .32;
inline constexpr double faint_contrast = .10, shadow_chroma = .025;
inline constexpr double shadow_contrast = .22, lively_lightness_std = .07;
inline constexpr double floor_gap_m = 1.2, bay_gap_m = .5;
inline constexpr double unknown_valid = .7, minimum_evidence = .12;
inline constexpr double rhythm_tolerance_m = .35;
inline constexpr std::size_t rhythm_min_windows = 4;
inline constexpr double rhythm_min_support = .6, extend_min_support = .5;
inline constexpr double extend_evidence = .03;
inline constexpr bool template_pass = false;
inline constexpr double template_score = .70;
inline constexpr std::size_t PPM = pixels_per_metre;
inline constexpr bool TEMPLATE_PASS = template_pass;
inline constexpr double TM_SCORE = template_score;
inline constexpr unsigned char cls_wall = 255, cls_window = 192, cls_door = 128,
							   cls_unknown = 64, cls_nodata = 0;
enum class Kind
{
	Window,
	Door,
	Shop,
	Rejected
};
inline const char *kind_name(Kind kind)
{
	switch (kind) {
	case Kind::Window:
		return "window";
	case Kind::Door:
		return "door";
	case Kind::Shop:
		return "shop";
	case Kind::Rejected:
		return "rejected";
	}
	return "rejected";
}
inline Kind parse_kind(const std::string &value)
{
	if (value == "window")
		return Kind::Window;
	if (value == "door")
		return Kind::Door;
	if (value == "shop")
		return Kind::Shop;
	return Kind::Rejected;
}
struct Rect
{
	Kind kind{Kind::Rejected};
	double x0{}, y0{}, x1{}, y1{};
	const char *reason{};
	double contrast{}, chroma{}, std_l{}, fill{1};
	std::size_t columns{}, rows{};
	std::size_t n_columns{}, n_rows{};
	double width_m() const { return (x1 - x0) / pixels_per_metre; }
	double height_m() const { return (y1 - y0) / pixels_per_metre; }
	bool valid() const
	{
		return x1 > x0 && y1 > y0 && std::isfinite(x0) && std::isfinite(y0) &&
			   std::isfinite(x1) && std::isfinite(y1);
	}
};
struct Rhythm
{
	double pitch{}, phase{}, support{};
	std::size_t n_windows{};
	bool accepted{};
	std::vector<double> lines(double width_px) const
	{
		std::vector<double> result;
		if (!accepted || pitch <= 0)
			return result;
		const double limit = width_px / pixels_per_metre;
		for (double x = phase; x < limit + pitch; x += pitch)
			if (x >= 0 && x <= limit)
				result.push_back(x * pixels_per_metre);
		return result;
	}
};
struct Texture
{
	projection::Image image;
	std::vector<bool> valid;
	bool consistent() const
	{
		return image.pixels.size() == std::size_t(image.width) * image.height &&
			   valid.size() == image.pixels.size();
	}
};
struct Result
{
	std::size_t rows{}, columns{};
	std::vector<unsigned char> classes;
	std::vector<projection::Rgb> colours;
	std::vector<double> evidence;
	std::vector<Rect> rectangles;
	std::vector<bool> mask;
	std::vector<std::pair<double, double>> floors;
	Rhythm rhythm;
	bool consistent() const
	{
		const auto cells = rows * columns;
		return colours.size() == cells && classes.size() == cells &&
			   evidence.size() == cells && mask.size() == cells;
	}
};

// Rust's 0.5 m sub-cell wall reference, which prevents windows from lowering
// their own comparison colour. The returned planes are NaN only when no
// usable pixels exist anywhere in the corresponding neighbourhood.
inline std::tuple<std::vector<double>, std::vector<double>, std::vector<double>,
		std::size_t, std::size_t>
wall_reference(const std::vector<imgops::Lab> &lab, const std::vector<bool> &valid,
		std::size_t width, std::size_t height)
{
	const std::size_t sub_rows = height / subcell_pixels;
	const std::size_t sub_cols = width / subcell_pixels;
	const auto sub_count = sub_rows * sub_cols;
	std::vector<imgops::Lab> medians(
			sub_count, {std::numeric_limits<double>::quiet_NaN(),
							   std::numeric_limits<double>::quiet_NaN(),
							   std::numeric_limits<double>::quiet_NaN()});
	std::vector<double> values;
	values.reserve(subcell_pixels * subcell_pixels);
	std::vector<imgops::Lab> samples;
	samples.reserve(subcell_pixels * subcell_pixels);
	for (std::size_t sy = 0; sy < sub_rows; ++sy)
		for (std::size_t sx = 0; sx < sub_cols; ++sx) {
			imgops::Lab median{};
			samples.clear();
			for (std::size_t y = sy * subcell_pixels; y < (sy + 1) * subcell_pixels; ++y)
				for (std::size_t x = sx * subcell_pixels; x < (sx + 1) * subcell_pixels;
						++x) {
					const auto i = y * width + x;
					if (i < valid.size() && valid[i])
						samples.push_back(lab[i]);
				}
			if (samples.empty())
				continue;
			for (std::size_t channel = 0; channel < median.size(); ++channel) {
				values.clear();
				for (const auto &sample : samples)
					values.push_back(sample[channel]);
				median[channel] = imgops::median(values);
			}
			medians[sy * sub_cols + sx] = median;
		}

	const auto nan = std::numeric_limits<double>::quiet_NaN();
	std::vector<double> l_ref(sub_count, nan), a_ref(sub_count, nan),
			b_ref(sub_count, nan);
	const auto radius = static_cast<std::ptrdiff_t>(reference_window_subcells / 2);
	std::vector<double> l_window, a_window, b_window, scratch;
	const auto window_size = reference_window_subcells * reference_window_subcells;
	l_window.reserve(window_size);
	a_window.reserve(window_size);
	b_window.reserve(window_size);
	scratch.reserve(window_size);
	std::vector<double> a_wall, b_wall;
	for (std::size_t sy = 0; sy < sub_rows; ++sy)
		for (std::size_t sx = 0; sx < sub_cols; ++sx) {
			l_window.clear();
			a_window.clear();
			b_window.clear();
			for (std::ptrdiff_t dy = -radius; dy <= radius; ++dy)
				for (std::ptrdiff_t dx = -radius; dx <= radius; ++dx) {
					const auto y = static_cast<std::ptrdiff_t>(sy) + dy;
					const auto x = static_cast<std::ptrdiff_t>(sx) + dx;
					if (x < 0 || y < 0 || x >= static_cast<std::ptrdiff_t>(sub_cols) ||
							y >= static_cast<std::ptrdiff_t>(sub_rows))
						continue;
					const auto &pixel = medians[static_cast<std::size_t>(y) * sub_cols +
												static_cast<std::size_t>(x)];
					if (!std::isfinite(pixel[0]))
						continue;
					l_window.push_back(pixel[0]);
					a_window.push_back(pixel[1]);
					b_window.push_back(pixel[2]);
				}
			if (l_window.empty())
				continue;
			scratch = l_window;
			const double lq = imgops::percentile(std::move(scratch), 75.0);
			const auto i = sy * sub_cols + sx;
			l_ref[i] = lq;
			a_wall.clear();
			b_wall.clear();
			for (std::size_t j = 0; j < l_window.size(); ++j)
				if (l_window[j] >= lq - .08) {
					a_wall.push_back(a_window[j]);
					b_wall.push_back(b_window[j]);
				}
			if (!a_wall.empty()) {
				a_ref[i] = imgops::median(a_wall);
				b_ref[i] = imgops::median(b_wall);
			}
		}

	std::vector<double> global_l, global_a, global_b;
	for (const auto &pixel : medians)
		if (std::isfinite(pixel[0])) {
			global_l.push_back(pixel[0]);
			global_a.push_back(pixel[1]);
			global_b.push_back(pixel[2]);
		}
	const double fallback_l = global_l.empty() ? .6 : imgops::percentile(global_l, 75.0);
	const double fallback_a = global_a.empty() ? 0.0 : imgops::median(global_a);
	const double fallback_b = global_b.empty() ? 0.0 : imgops::median(global_b);
	for (std::size_t i = 0; i < sub_count; ++i) {
		if (!std::isfinite(l_ref[i]))
			l_ref[i] = fallback_l;
		if (!std::isfinite(a_ref[i]))
			a_ref[i] = fallback_a;
		if (!std::isfinite(b_ref[i]))
			b_ref[i] = fallback_b;
	}
	return {std::move(l_ref), std::move(a_ref), std::move(b_ref), sub_rows, sub_cols};
}
inline bool opening_candidate(const projection::Rgb &pixel, const projection::Rgb &wall)
{
	int light =
				[](const projection::Rgb &p) {
					return (299 * p[0] + 587 * p[1] + 114 * p[2]) / 1000;
				}(pixel),
		reference = [](const projection::Rgb &p) {
			return (299 * p[0] + 587 * p[1] + 114 * p[2]) / 1000;
		}(wall);
	const double local_lightness = reference / 255.0;
	return light < reference * (1.0 - dark_threshold * local_lightness) ||
		   std::abs(int(pixel[2]) - int(wall[2])) > chroma_threshold * 255.0;
}
inline std::vector<Rect> components(
		const std::vector<bool> &mask, std::size_t width, std::size_t height)
{
	std::vector<Rect> out;
	std::vector<bool> seen(mask.size());
	for (std::size_t start = 0; start < mask.size(); ++start)
		if (mask[start] && !seen[start]) {
			std::queue<std::size_t> todo;
			todo.push(start);
			seen[start] = true;
			std::size_t x0 = width, y0 = height, x1 = 0, y1 = 0, n = 0;
			while (!todo.empty()) {
				auto i = todo.front();
				todo.pop();
				auto x = i % width, y = i / width;
				x0 = std::min(x0, x);
				y0 = std::min(y0, y);
				x1 = std::max(x1, x);
				y1 = std::max(y1, y);
				++n;
				for (int dy = -1; dy <= 1; ++dy)
					for (int dx = -1; dx <= 1; ++dx) {
						long nx = long(x) + dx, ny = long(y) + dy;
						if (nx >= 0 && ny >= 0 && nx < long(width) && ny < long(height)) {
							auto ni = std::size_t(ny) * width + nx;
							if (mask[ni] && !seen[ni]) {
								seen[ni] = true;
								todo.push(ni);
							}
						}
					}
			}
			Rect rect;
			rect.kind = Kind::Rejected;
			rect.x0 = x0;
			rect.y0 = y0;
			rect.x1 = x1 + 1;
			rect.y1 = y1 + 1;
			rect.fill = double(n) / ((x1 - x0 + 1) * (y1 - y0 + 1));
			rect.columns = x1 - x0 + 1;
			rect.rows = y1 - y0 + 1;
			out.push_back(rect);
		}
	return out;
}

inline void clear_long_runs(imgops::Mask &mask, std::size_t max_h, std::size_t max_v)
{
	for (std::size_t y = 0; y < mask.h; ++y)
		for (std::size_t x = 0; x < mask.w;) {
			if (!mask.at(x, y)) {
				++x;
				continue;
			}
			std::size_t end = x + 1;
			while (end < mask.w && mask.at(end, y))
				++end;
			if (end - x > max_h)
				for (auto xx = x; xx < end; ++xx)
					mask.set(xx, y, false);
			x = end;
		}
	for (std::size_t x = 0; x < mask.w; ++x)
		for (std::size_t y = 0; y < mask.h;) {
			if (!mask.at(x, y)) {
				++y;
				continue;
			}
			std::size_t end = y + 1;
			while (end < mask.h && mask.at(x, end))
				++end;
			if (end - y > max_v)
				for (auto yy = y; yy < end; ++yy)
					mask.set(x, yy, false);
			y = end;
		}
}

struct BlobSums
{
	double count{}, dl{}, l{}, l2{}, chroma{}, a{}, b{};
};

inline std::vector<Rect> split_blob(const imgops::Labels &labels, std::uint32_t id,
		std::size_t x, std::size_t y, std::size_t bw, std::size_t bh,
		const std::vector<double> &dl, const std::vector<double> &dchroma,
		std::size_t width, bool parent_ragged)
{
	imgops::Mask cut(bw, bh);
	for (std::size_t yy = 0; yy < bh; ++yy)
		for (std::size_t xx = 0; xx < bw; ++xx)
			cut.set(xx, yy, labels.at(x + xx, y + yy) == id);
	if (bw > 2 * pixels_per_metre) {
		std::vector<bool> band(bh);
		for (std::size_t yy = 0; yy < bh; ++yy) {
			std::size_t occupied = 0;
			for (std::size_t xx = 0; xx < bw; ++xx)
				occupied += cut.at(xx, yy);
			band[yy] = occupied >= .85 * bw;
		}
		if (std::any_of(band.begin(), band.end(), [](bool v) { return v; }) &&
				!std::all_of(band.begin(), band.end(), [](bool v) { return v; }))
			for (std::size_t yy = 0; yy < bh; ++yy)
				if (band[yy])
					for (std::size_t xx = 0; xx < bw; ++xx)
						cut.set(xx, yy, false);
	}
	std::vector<std::size_t> occupied(bw);
	for (std::size_t xx = 0; xx < bw; ++xx)
		for (std::size_t yy = 0; yy < bh; ++yy)
			occupied[xx] += cut.at(xx, yy);
	const auto max_occupied = *std::max_element(occupied.begin(), occupied.end());
	std::vector<bool> bridge(bw);
	for (std::size_t xx = 0; xx < bw; ++xx)
		bridge[xx] = occupied[xx] < .4 * max_occupied;
	if (max_occupied &&
			std::any_of(bridge.begin(), bridge.end(), [](bool v) { return v; }) &&
			!std::all_of(bridge.begin(), bridge.end(), [](bool v) { return v; }))
		for (std::size_t xx = 0; xx < bw; ++xx)
			if (bridge[xx])
				for (std::size_t yy = 0; yy < bh; ++yy)
					cut.set(xx, yy, false);
	auto opened = imgops::open(cut, 5, 5);
	clear_long_runs(opened, static_cast<std::size_t>(split_width_m * pixels_per_metre),
			static_cast<std::size_t>(large_height_m * pixels_per_metre));
	const auto parts = imgops::connected_components(opened);
	std::vector<Rect> result;
	const double min_part_fill = parent_ragged ? .6 : min_fill;
	for (std::size_t j = 1; j < parts.count(); ++j) {
		const auto &c = parts.components[j];
		const double wm = double(c.w) / pixels_per_metre,
					 hm = double(c.h) / pixels_per_metre;
		const double max_h = wm <= narrow_width_m ? large_height_m : split_height_m;
		const double fill = double(c.area) / std::max<std::size_t>(1, c.w * c.h);
		if (wm < min_width_m || hm < min_height_m || wm > split_width_m || hm > max_h ||
				fill < min_part_fill)
			continue;
		double contrast = 0, chroma = 0;
		std::size_t n = 0;
		for (std::size_t yy = 0; yy < bh; ++yy)
			for (std::size_t xx = 0; xx < bw; ++xx)
				if (parts.at(xx, yy) == j) {
					contrast += dl[(y + yy) * width + x + xx];
					chroma += dchroma[(y + yy) * width + x + xx];
					++n;
				}
		if (!n)
			continue;
		contrast /= n;
		chroma /= n;
		if (contrast < faint_contrast && chroma < .04)
			continue;
		Rect r;
		r.kind = Kind::Window;
		r.x0 = x + c.x;
		r.y0 = y + c.y;
		r.x1 = x + c.x + c.w;
		r.y1 = y + c.y + c.h;
		r.reason = "split";
		r.contrast = contrast;
		r.chroma = chroma;
		r.fill = fill;
		result.push_back(r);
	}
	return result.size() >= 2 ? result : std::vector<Rect>{};
}

inline std::vector<Rect> find_rects(const imgops::Mask &opening,
		const std::vector<double> &dl, const std::vector<double> &l,
		const std::vector<double> &a, const std::vector<double> &b,
		const std::vector<double> &dchroma)
{
	const auto width = opening.w, height = opening.h;
	const auto labels = imgops::connected_components(opening);
	if (labels.count() <= 1)
		return {};
	std::vector<BlobSums> sums(labels.count());
	for (std::size_t i = 0; i < width * height; ++i) {
		auto &s = sums[labels.labels[i]];
		const auto finite = [](double v) { return std::isfinite(v) ? v : 0.; };
		++s.count;
		s.dl += finite(dl[i]);
		s.l += finite(l[i]);
		s.l2 += finite(l[i] * l[i]);
		s.chroma += finite(dchroma[i]);
		s.a += finite(a[i]);
		s.b += finite(b[i]);
	}
	std::vector<Rect> rects;
	const double wall_w = double(width) / pixels_per_metre;
	for (std::size_t id = 1; id < labels.count(); ++id) {
		const auto &c = labels.components[id];
		const auto &s = sums[id];
		const double n = std::max(1., s.count), contrast = s.dl / n,
					 chroma = s.chroma / n;
		const double mean_l = s.l / n,
					 std_l = std::sqrt(std::max(0., s.l2 / n - mean_l * mean_l));
		const double mean_a = s.a / n, mean_b = s.b / n;
		const double wm = double(c.w) / pixels_per_metre,
					 hm = double(c.h) / pixels_per_metre;
		const double fill = double(c.area) / std::max<std::size_t>(1, c.w * c.h);
		const bool base = c.y + c.h >= height - base_tolerance_m * pixels_per_metre;
		const bool bottom = c.y + c.h >= height - .6 * pixels_per_metre;
		Rect r;
		r.x0 = c.x;
		r.y0 = c.y;
		r.x1 = c.x + c.w;
		r.y1 = c.y + c.h;
		r.contrast = contrast;
		r.chroma = chroma;
		r.std_l = std_l;
		r.fill = fill;
		if (c.y <= .3 * pixels_per_metre && mean_l > .7 && mean_b < -.04 && !base)
			r.reason = "sky";
		else if (mean_a < -.02 && mean_b > .02 && (wm > 2 || hm > 2))
			r.reason = "tree";
		else if (base) {
			const bool lively = std_l >= lively_lightness_std || chroma >= .04;
			if (wm >= 3 && hm > shop_max_height_m)
				r.reason = "tall";
			else if (wm >= 3 && hm >= shop_min_height_m) {
				const auto rows_above = std::max<std::size_t>(
						1, c.h > pixels_per_metre ? c.h - pixels_per_metre : 1);
				double sl = 0, sl2 = 0, sc = 0;
				std::size_t count = 0;
				for (std::size_t yy = 0; yy < std::min(rows_above, c.h); ++yy)
					for (std::size_t xx = 0; xx < c.w; ++xx)
						if (labels.at(c.x + xx, c.y + yy) == id) {
							auto p = (c.y + yy) * width + c.x + xx;
							sl += l[p];
							sl2 += l[p] * l[p];
							sc += dchroma[p];
							++count;
						}
				const double ml = count ? sl / count : mean_l;
				const double su =
						count ? std::sqrt(std::max(0., sl2 / count - ml * ml)) : std_l;
				const double cu = count ? sc / count : chroma;
				const bool lively_up = su >= lively_lightness_std || cu >= .04;
				if (fill >= .3 && lively_up && contrast >= faint_contrast)
					r.kind = Kind::Shop;
				else if (hm <= plinth_max_height_m && !lively_up)
					r.reason = "plinth";
				else if (fill < min_fill)
					r.reason = "ragged";
				else if (hm > plinth_max_height_m)
					r.kind = Kind::Shop;
				else
					r.reason = "plinth";
			} else if (wm < min_width_m || hm < min_height_m)
				r.reason = "small";
			else if (fill < min_fill)
				r.reason = "ragged";
			else if (wm >= door_min_width_m && wm <= door_max_width_m &&
					 hm >= door_min_height_m && c.x >= .3 * pixels_per_metre &&
					 c.x + c.w <= width - .3 * pixels_per_metre && (bottom || hm >= 2))
				r.kind = Kind::Door;
			else if (wm > door_max_width_m && hm >= shop_min_height_m && lively &&
					 contrast >= faint_contrast)
				r.kind = Kind::Shop;
			else if (wm > door_max_width_m && hm >= shop_min_height_m) {
				if (hm > plinth_max_height_m)
					r.kind = Kind::Shop;
				else
					r.reason = "plinth";
			} else if (!bottom && hm >= .8 && wm <= 2 && fill >= .6 &&
					   contrast >= faint_contrast)
				r.kind = Kind::Window;
			else
				r.reason = "base strip";
		} else if (wm < min_width_m || hm < min_height_m)
			r.reason = "small";
		else if (hm < .8 && wm > 2)
			r.reason = "strip";
		else if (c.y < .5 * pixels_per_metre && wm > 2.5 && hm < 1.3)
			r.reason = "eave";
		else if (wm <= narrow_width_m && hm <= 3.4 && fill >= .6 &&
				 (std_l >= .04 || chroma >= .03))
			r.kind = Kind::Window;
		else {
			if (wm > split_width_m || hm > split_height_m || fill < min_fill) {
				auto parts = split_blob(labels, id, c.x, c.y, c.w, c.h, dl, dchroma,
						width, fill < min_fill);
				if (!parts.empty()) {
					rects.insert(rects.end(), parts.begin(), parts.end());
					continue;
				}
			}
			const bool glassy =
					fill >= .6 && hm <= large_height_m &&
					(std_l >= lively_lightness_std + .01 || contrast >= deep_glass);
			if (fill < min_fill)
				r.reason = "ragged";
			else if ((wm > split_width_m || hm > split_height_m) && !glassy)
				r.reason =
						wm <= std::max(large_width_m, .7 * wall_w) && hm <= large_height_m
								? "band"
								: "too big";
			else if (contrast < faint_contrast && chroma < .04)
				r.reason = "faint";
			else if (chroma < shadow_chroma && contrast < shadow_contrast &&
					 std_l < lively_lightness_std && (wm > 2.5 || hm > 2.5))
				r.reason = "shadow";
			else
				r.kind = Kind::Window;
		}
		rects.push_back(r);
	}
	std::vector<std::size_t> wide;
	for (std::size_t i = 0; i < rects.size(); ++i)
		if (rects[i].kind == Kind::Window &&
				rects[i].width_m() > std::max(large_width_m, .7 * wall_w))
			wide.push_back(i);
	if (wide.size() == 1) {
		rects[wide.front()].kind = Kind::Rejected;
		rects[wide.front()].reason = "parapet";
	}
	return rects;
}

inline double cx(const Rect &r)
{
	return .5 * (r.x0 + r.x1);
}
inline double cy(const Rect &r)
{
	return .5 * (r.y0 + r.y1);
}
inline std::vector<std::vector<std::size_t>> floors_of(const std::vector<Rect> &rects)
{
	std::vector<std::size_t> windows;
	for (std::size_t i = 0; i < rects.size(); ++i)
		if (rects[i].kind == Kind::Window)
			windows.push_back(i);
	std::sort(windows.begin(), windows.end(),
			[&](auto a, auto b) { return cy(rects[a]) < cy(rects[b]); });
	std::vector<std::vector<std::size_t>> groups;
	for (auto i : windows) {
		bool joins = false;
		if (!groups.empty()) {
			double sum = 0;
			for (auto j : groups.back())
				sum += cy(rects[j]);
			joins = std::abs(cy(rects[i]) - sum / groups.back().size()) <=
					floor_gap_m * pixels_per_metre;
		}
		if (joins)
			groups.back().push_back(i);
		else
			groups.push_back({i});
	}
	return groups;
}

inline std::optional<double> mask_mean(
		const imgops::Mask &mask, double x0, double y0, double x1, double y1)
{
	const auto ys =
			static_cast<std::size_t>(std::clamp(std::trunc(y0), 0.0, double(mask.h)));
	const auto ye =
			static_cast<std::size_t>(std::clamp(std::trunc(y1), 0.0, double(mask.h)));
	const auto xs =
			static_cast<std::size_t>(std::clamp(std::trunc(x0), 0.0, double(mask.w)));
	const auto xe =
			static_cast<std::size_t>(std::clamp(std::trunc(x1), 0.0, double(mask.w)));
	if (ye <= ys || xe <= xs)
		return std::nullopt;
	std::size_t count = 0;
	for (auto y = ys; y < ye; ++y)
		for (auto x = xs; x < xe; ++x)
			count += mask.at(x, y);
	return double(count) / ((ye - ys) * (xe - xs));
}

inline std::vector<std::vector<std::size_t>> regularise_floors(
		std::vector<Rect> &rects, const imgops::Mask &soft)
{
	auto groups = floors_of(rects);
	for (const auto &group : groups) {
		if (group.size() < 2) {
			const auto i = group.front();
			if (rects[i].width_m() * rects[i].height_m() < 1.0 &&
					rects[i].contrast < deep_glass) {
				rects[i].kind = Kind::Rejected;
				rects[i].reason = "lone";
			}
			continue;
		}
		std::vector<double> tops, heights, widths;
		for (auto i : group) {
			tops.push_back(rects[i].y0);
			heights.push_back(rects[i].y1 - rects[i].y0);
			widths.push_back(rects[i].x1 - rects[i].x0);
		}
		const auto top = imgops::median(tops), height = imgops::median(heights),
				   width = imgops::median(widths);
		for (auto i : group) {
			auto &r = rects[i];
			const auto y0 = r.y0, y1 = r.y1, x0 = r.x0, x1 = r.x1;
			if (std::abs(y0 - top) <= .5 * pixels_per_metre &&
					std::abs((y1 - y0) - height) <= .6 * pixels_per_metre) {
				r.y0 = top;
				r.y1 = top + height;
			} else if (height >= 1.2 * pixels_per_metre &&
					   std::abs(y0 - top) <= .5 * pixels_per_metre &&
					   y1 - y0 < .6 * height) {
				if (auto mean = mask_mean(soft, x0, top, x1, top + height);
						mean && *mean >= minimum_evidence) {
					r.y0 = top;
					r.y1 = top + height;
				}
			}
			if (std::abs((x1 - x0) - width) <= .5 * pixels_per_metre) {
				const auto center = cx(r);
				r.x0 = center - .5 * width;
				r.x1 = center + .5 * width;
			}
		}
	}
	groups.erase(std::remove_if(groups.begin(), groups.end(),
						 [&](const auto &g) {
							 return std::none_of(g.begin(), g.end(), [&](auto i) {
								 return rects[i].kind == Kind::Window;
							 });
						 }),
			groups.end());
	return groups;
}

inline void regularise_columns(std::vector<Rect> &rects)
{
	std::vector<std::size_t> windows;
	for (std::size_t i = 0; i < rects.size(); ++i)
		if (rects[i].kind == Kind::Window)
			windows.push_back(i);
	std::sort(windows.begin(), windows.end(),
			[&](auto a, auto b) { return cx(rects[a]) < cx(rects[b]); });
	std::vector<std::vector<std::size_t>> bays;
	for (auto i : windows) {
		bool joins = false;
		if (!bays.empty()) {
			std::vector<double> centers;
			for (auto j : bays.back())
				centers.push_back(cx(rects[j]));
			joins = std::abs(cx(rects[i]) - imgops::median(centers)) <=
					bay_gap_m * pixels_per_metre;
		}
		if (joins)
			bays.back().push_back(i);
		else
			bays.push_back({i});
	}
	for (const auto &bay : bays) {
		if (bay.size() < 2)
			continue;
		std::vector<double> centers;
		for (auto i : bay)
			centers.push_back(cx(rects[i]));
		const auto center = imgops::median(centers);
		for (auto i : bay) {
			const auto half = .5 * (rects[i].x1 - rects[i].x0);
			rects[i].x0 = center - half;
			rects[i].x1 = center + half;
		}
	}
}

inline std::vector<std::size_t> upper_floors(
		const std::vector<std::vector<std::size_t>> &groups,
		const std::vector<Rect> &rects, std::size_t height)
{
	std::vector<std::size_t> result;
	for (std::size_t k = 0; k < groups.size(); ++k) {
		std::vector<double> bottoms;
		for (auto i : groups[k])
			bottoms.push_back(rects[i].y1);
		if (!bottoms.empty() &&
				imgops::median(bottoms) < double(height) - 2 * pixels_per_metre)
			result.push_back(k);
	}
	return result;
}

inline Rhythm fit_rhythm(const std::vector<std::vector<std::size_t>> &groups,
		const std::vector<Rect> &rects, std::size_t height)
{
	Rhythm rhythm;
	const auto floors = upper_floors(groups, rects, height);
	std::vector<std::size_t> windows;
	for (auto gi : floors)
		for (auto i : groups[gi])
			if (rects[i].kind == Kind::Window)
				windows.push_back(i);
	rhythm.n_windows = windows.size();
	if (windows.size() < rhythm_min_windows)
		return rhythm;
	std::vector<double> gaps;
	for (auto gi : floors) {
		std::vector<double> centers;
		for (auto i : groups[gi])
			if (rects[i].kind == Kind::Window)
				centers.push_back(cx(rects[i]) / pixels_per_metre);
		std::sort(centers.begin(), centers.end());
		for (std::size_t i = 1; i < centers.size(); ++i) {
			const auto d = centers[i] - centers[i - 1];
			if (d >= 1.2 && d <= 8.0)
				gaps.push_back(d);
		}
	}
	if (gaps.size() < 2)
		return rhythm;
	std::sort(gaps.begin(), gaps.end());
	std::vector<std::pair<std::size_t, double>> clusters;
	for (std::size_t i = 0; i < gaps.size();) {
		std::size_t j = i;
		while (j + 1 < gaps.size() && gaps[j + 1] - gaps[i] <= .5)
			++j;
		std::vector<double> cluster(gaps.begin() + i, gaps.begin() + j + 1);
		clusters.emplace_back(j - i + 1, imgops::median(cluster));
		i = j + 1;
	}
	std::sort(clusters.begin(), clusters.end(), [](const auto &a, const auto &b) {
		return a.first != b.first ? a.first > b.first : a.second > b.second;
	});
	std::vector<double> candidates;
	for (std::size_t i = 0; i < std::min<std::size_t>(3, clusters.size()); ++i)
		if (clusters[i].first >= 2)
			candidates.push_back(clusters[i].second);
	if (candidates.empty())
		candidates.push_back(clusters.front().second);
	std::vector<double> centers;
	for (auto i : windows)
		centers.push_back(cx(rects[i]) / pixels_per_metre);
	double best_score = -std::numeric_limits<double>::infinity();
	std::size_t best_hits = 0;
	for (auto pitch : candidates) {
		const auto chance = windows.size() * std::min(1., 2 * rhythm_tolerance_m / pitch);
		const auto count = static_cast<std::size_t>(std::ceil(pitch / .05));
		for (std::size_t j = 0; j < count; ++j) {
			const double phase = j * .05;
			std::size_t hits = 0;
			for (auto x : centers) {
				auto m = std::fmod(x - phase + .5 * pitch, pitch);
				if (m < 0)
					m += pitch;
				m -= .5 * pitch;
				hits += std::abs(m) <= rhythm_tolerance_m;
			}
			const auto score = double(hits) - chance;
			if (score > best_score) {
				best_score = score;
				best_hits = hits;
				rhythm.pitch = pitch;
				rhythm.phase = phase;
			}
		}
	}
	rhythm.support = double(best_hits) / windows.size();
	rhythm.accepted = rhythm.support >= rhythm_min_support;
	return rhythm;
}

inline std::size_t apply_rhythm(std::vector<Rect> &rects,
		std::vector<std::vector<std::size_t>> &groups, const Rhythm &rhythm,
		const imgops::Mask &soft, std::size_t height, std::size_t width)
{
	if (!rhythm.accepted)
		return 0;
	const auto lines = rhythm.lines(width);
	if (lines.size() < 2)
		return 0;
	const auto upper = upper_floors(groups, rects, height);
	std::vector<double> all_centers;
	for (auto gi : upper)
		for (auto i : groups[gi])
			if (rects[i].kind == Kind::Window)
				all_centers.push_back(cx(rects[i]));
	if (all_centers.empty())
		return 0;
	const auto bounds = std::minmax_element(all_centers.begin(), all_centers.end());
	const auto lo_span = *bounds.first - .5 * rhythm.pitch * pixels_per_metre;
	const auto hi_span = *bounds.second + .5 * rhythm.pitch * pixels_per_metre;
	std::vector<bool> in_span;
	for (auto x : lines)
		in_span.push_back(lo_span <= x && x <= hi_span);
	struct Floor
	{
		std::size_t group;
		std::vector<std::size_t> windows;
		std::vector<bool> taken;
	};
	std::vector<Floor> floors;
	std::vector<std::size_t> bays_any(lines.size());
	for (auto gi : upper) {
		Floor floor{gi, {}, std::vector<bool>(lines.size())};
		for (auto i : groups[gi])
			if (rects[i].kind == Kind::Window)
				floor.windows.push_back(i);
		for (auto i : floor.windows) {
			const auto center = cx(rects[i]);
			std::size_t nearest = 0;
			double distance = std::abs(center - lines[0]);
			for (std::size_t k = 1; k < lines.size(); ++k)
				if (auto d = std::abs(center - lines[k]); d < distance) {
					distance = d;
					nearest = k;
				}
			if (distance <= rhythm_tolerance_m * pixels_per_metre) {
				const auto half = .5 * (rects[i].x1 - rects[i].x0);
				rects[i].x0 = lines[nearest] - half;
				rects[i].x1 = lines[nearest] + half;
				floor.taken[nearest] = true;
			}
		}
		for (std::size_t k = 0; k < lines.size(); ++k)
			if (floor.taken[k])
				++bays_any[k];
		floors.push_back(std::move(floor));
	}
	std::size_t added = 0;
	for (auto &floor : floors) {
		const auto count = static_cast<std::size_t>(
				std::count(floor.taken.begin(), floor.taken.end(), true));
		if (floor.windows.size() < 3 || count < 2)
			continue;
		std::vector<double> widths, heights, tops;
		for (auto i : floor.windows) {
			widths.push_back(rects[i].x1 - rects[i].x0);
			heights.push_back(rects[i].y1 - rects[i].y0);
			tops.push_back(rects[i].y0);
		}
		const auto ww = imgops::median(widths), hh = imgops::median(heights),
				   top = imgops::median(tops);
		auto first = std::find(floor.taken.begin(), floor.taken.end(), true);
		auto last = std::find(floor.taken.rbegin(), floor.taken.rend(), true);
		const auto lo = static_cast<std::size_t>(first - floor.taken.begin());
		const auto hi = floor.taken.size() - 1 -
						static_cast<std::size_t>(last - floor.taken.rbegin());
		if (double(count) / (hi - lo + 1) < extend_min_support)
			continue;
		for (std::size_t k = 0; k < lines.size(); ++k) {
			if (floor.taken[k] || !in_span[k])
				continue;
			const auto x0 = lines[k] - .5 * ww, x1 = lines[k] + .5 * ww;
			if (x0 < .3 * pixels_per_metre || x1 > width - .3 * pixels_per_metre)
				continue;
			const bool overlap =
					std::any_of(rects.begin(), rects.end(), [&](const auto &r) {
						return r.kind != Kind::Rejected && r.x0 < x1 && r.x1 > x0 &&
							   r.y0 < top + hh && r.y1 > top;
					});
			if (overlap)
				continue;
			double need;
			if (lo <= k && k <= hi)
				need = extend_evidence;
			else if (bays_any[k] >= 1)
				need = .06;
			else
				continue;
			const auto evidence = mask_mean(soft, x0, top, x1, top + hh);
			if (!evidence || *evidence < need)
				continue;
			Rect r;
			r.kind = Kind::Window;
			r.x0 = x0;
			r.y0 = top;
			r.x1 = x1;
			r.y1 = top + hh;
			r.reason = "rhythm";
			rects.push_back(r);
			groups[floor.group].push_back(rects.size() - 1);
			++added;
		}
	}
	return added;
}

inline std::size_t cell_count(double metres)
{
	return static_cast<std::size_t>(std::max(1.0, std::floor(metres + .3)));
}
inline std::pair<std::size_t, std::size_t> cell_span(
		double lo, double hi, int origin, std::size_t cells, std::size_t requested)
{
	if (!cells)
		return {0, 0};
	const auto count = std::clamp<std::size_t>(requested, 1, cells);
	const auto center =
			std::floor(((lo + hi) * .5 - origin) / pixels_per_metre - count * .5 + .5);
	const auto start =
			static_cast<std::size_t>(std::clamp(center, 0.0, double(cells - count)));
	return {start, start + count};
}
inline void assign_cells(std::vector<Rect> &rects,
		const std::vector<std::vector<std::size_t>> &groups, double pitch_m)
{
	const auto max_cols =
			pitch_m >= 2 ? std::max<std::size_t>(1, cell_count(pitch_m) - 1) : 0;
	for (auto &r : rects) {
		r.n_columns = cell_count(r.width_m());
		r.n_rows = cell_count(r.height_m());
	}
	for (const auto &group : groups) {
		std::vector<std::size_t> windows;
		for (auto i : group)
			if (rects[i].kind == Kind::Window)
				windows.push_back(i);
		if (windows.size() < 2)
			continue;
		std::vector<double> widths, heights;
		for (auto i : windows) {
			widths.push_back(rects[i].width_m());
			heights.push_back(rects[i].height_m());
		}
		const auto wm = imgops::median(widths), hm = imgops::median(heights);
		for (auto i : windows) {
			if (std::abs(rects[i].width_m() - wm) <= 1)
				rects[i].n_columns = cell_count(wm);
			if (std::abs(rects[i].height_m() - hm) <= 1)
				rects[i].n_rows = cell_count(hm);
		}
	}
	if (max_cols)
		for (auto &r : rects)
			if (r.kind == Kind::Window)
				r.n_columns = std::min(r.n_columns, max_cols);
}
inline Result classify_grid(const std::vector<projection::Rgb> &colours,
		std::size_t columns, std::size_t rows)
{
	Result out;
	out.columns = columns;
	out.rows = rows;
	out.colours = colours;
	out.classes.assign(colours.size(), cls_wall);
	out.evidence.assign(colours.size(), 0);
	out.mask.assign(colours.size(), false);
	if (colours.size() != columns * rows)
		return out;
	for (std::size_t r = 0; r < rows; ++r)
		for (std::size_t c = 0; c < columns; ++c) {
			std::array<unsigned, 3> sum{};
			std::size_t n = 0;
			for (long y = long(r) - 1; y <= long(r) + 1; ++y)
				for (long x = long(c) - 1; x <= long(c) + 1; ++x)
					if (x >= 0 && y >= 0 && x < long(columns) && y < long(rows)) {
						auto i = std::size_t(y) * columns + x;
						for (unsigned k = 0; k < 3; ++k)
							sum[k] += colours[i][k];
						++n;
					}
			projection::Rgb wall{std::uint8_t(sum[0] / n), std::uint8_t(sum[1] / n),
					std::uint8_t(sum[2] / n)};
			auto i = r * columns + c;
			if (opening_candidate(colours[i], wall)) {
				out.mask[i] = true;
				out.evidence[i] = 1;
				out.classes[i] = cls_window;
			}
		}
	out.rectangles = components(out.mask, columns, rows);
	for (auto &rect : out.rectangles) {
		if (rect.width_m() < min_width_m || rect.height_m() < min_height_m ||
				rect.fill < min_fill) {
			rect.kind = Kind::Rejected;
			rect.reason = "small";
			continue;
		}
		const bool at_base =
				rect.y1 >= double(rows) - base_tolerance_m * pixels_per_metre;
		if (at_base && rect.height_m() >= door_min_height_m &&
				rect.width_m() >= door_min_width_m && rect.width_m() <= door_max_width_m)
			rect.kind = Kind::Door;
		else if (at_base && rect.height_m() >= shop_min_height_m &&
				 rect.height_m() <= shop_max_height_m &&
				 rect.width_m() > door_max_width_m)
			rect.kind = Kind::Shop;
		else
			rect.kind = Kind::Window;
		const auto cls = rect.kind == Kind::Door ? cls_door : cls_window;
		if (rect.kind != Kind::Rejected)
			for (std::size_t y = std::size_t(rect.y0);
					y < std::size_t(rect.y1) && y < rows; ++y)
				for (std::size_t x = std::size_t(rect.x0);
						x < std::size_t(rect.x1) && x < columns; ++x)
					if (out.mask[y * columns + x])
						out.classes[y * columns + x] = cls;
	}
	return out;
}

// Texture-aware opening classification shared by facade pipeline clients. It
// ports the Rust local OkLab reference, vegetation guard, morphology and
// pixel-to-block aggregation. The later Rust floor/rhythm regularizers can
// refine these pixel components without changing this texture contract.
inline Result classify_texture(const Texture &texture, std::size_t columns,
		std::size_t rows, std::pair<int, int> origin_px = {0, 0})
{
	Result out;
	out.columns = columns;
	out.rows = rows;
	const auto width = static_cast<std::size_t>(texture.image.width);
	const auto height = static_cast<std::size_t>(texture.image.height);
	if (!columns || !rows || !texture.consistent() || width < subcell_pixels ||
			height < subcell_pixels)
		return out;

	const auto pixel_count = width * height;
	std::vector<imgops::Lab> lab(pixel_count);
	std::vector<bool> vegetation(pixel_count), reference_valid(pixel_count);
	for (std::size_t i = 0; i < pixel_count; ++i) {
		lab[i] = imgops::srgb_to_oklab(texture.image.pixels[i]);
		vegetation[i] =
				texture.valid[i] && lab[i][1] < -.03 && lab[i][2] > .03 && lab[i][0] < .6;
		reference_valid[i] = texture.valid[i] && !vegetation[i];
	}
	const auto [l_ref, a_ref, b_ref, sub_rows, sub_cols] =
			wall_reference(lab, reference_valid, width, height);
	if (!sub_rows || !sub_cols)
		return out;
	auto sub_at = [&](std::size_t x, std::size_t y) {
		return std::min(y / subcell_pixels, sub_rows - 1) * sub_cols +
			   std::min(x / subcell_pixels, sub_cols - 1);
	};
	imgops::Mask opening(width, height), soft(width, height);
	std::vector<double> l(pixel_count), a(pixel_count), b(pixel_count), dl(pixel_count),
			dchroma(pixel_count);
	for (std::size_t y = 0; y < height; ++y)
		for (std::size_t x = 0; x < width; ++x) {
			const auto i = y * width + x;
			const auto s = sub_at(x, y);
			l[i] = lab[i][0];
			a[i] = lab[i][1];
			b[i] = lab[i][2];
			const double threshold = dark_threshold * std::clamp(l_ref[s] / .55, .5, 1.0);
			const double dark_delta = l_ref[s] - lab[i][0];
			const double chroma_delta =
					std::hypot(lab[i][1] - a_ref[s], lab[i][2] - b_ref[s]);
			dl[i] = dark_delta;
			dchroma[i] = chroma_delta;
			const bool live = texture.valid[i] && !vegetation[i];
			const bool sky = lab[i][0] > .75 && lab[i][2] < -.06;
			opening.set(x, y,
					live && (dark_delta > threshold ||
									(chroma_delta > chroma_threshold &&
											dark_delta > -.08 && !sky)));
			soft.set(x, y, live && (dark_delta > .5 * threshold || chroma_delta > .03));
		}
	const auto cleaned = imgops::close(imgops::open(opening, 3, 3), 3, 3);
	out.mask.assign(rows * columns, false);
	out.classes.assign(rows * columns, cls_wall);
	out.colours.assign(rows * columns, {});
	out.evidence.assign(rows * columns, 0.0);
	out.rectangles = find_rects(cleaned, dl, l, a, b, dchroma);
	auto groups = regularise_floors(out.rectangles, soft);
	out.rhythm = fit_rhythm(groups, out.rectangles, height);
	if (out.rhythm.accepted)
		apply_rhythm(out.rectangles, groups, out.rhythm, soft, height, width);
	else
		regularise_columns(out.rectangles);
	assign_cells(out.rectangles, groups, out.rhythm.accepted ? out.rhythm.pitch : 0.0);
	for (const auto &group : groups) {
		std::vector<double> tops, bottoms;
		for (auto i : group) {
			tops.push_back(out.rectangles[i].y0);
			bottoms.push_back(out.rectangles[i].y1);
		}
		if (!tops.empty())
			out.floors.emplace_back(imgops::median(tops), imgops::median(bottoms));
	}

	const auto [origin_x, origin_y] = origin_px;
	for (const auto &group : groups) {
		std::vector<std::size_t> windows;
		for (auto i : group)
			if (out.rectangles[i].kind == Kind::Window)
				windows.push_back(i);
		std::sort(windows.begin(), windows.end(), [&](auto a, auto b) {
			return out.rectangles[a].x0 < out.rectangles[b].x0;
		});
		std::vector<std::pair<std::size_t, std::size_t>> spans;
		for (auto i : windows)
			spans.push_back(cell_span(out.rectangles[i].x0, out.rectangles[i].x1,
					origin_x, columns, out.rectangles[i].n_columns));
		for (std::size_t i = 1; i < windows.size(); ++i) {
			const auto &previous = out.rectangles[windows[i - 1]];
			const auto &current = out.rectangles[windows[i]];
			const bool overlap = spans[i].first < spans[i - 1].second;
			const bool touch = spans[i].first == spans[i - 1].second &&
							   current.x0 - previous.x1 >= .3 * pixels_per_metre;
			if (overlap || touch) {
				const auto wider =
						out.rectangles[windows[i]].n_columns >=
										out.rectangles[windows[i - 1]].n_columns
								? i
								: i - 1;
				auto &r = out.rectangles[windows[wider]];
				if (r.n_columns > 1) {
					--r.n_columns;
					spans[wider] = cell_span(r.x0, r.x1, origin_x, columns, r.n_columns);
				}
			}
		}
		for (std::size_t i = 0; i < windows.size(); ++i) {
			const auto &r = out.rectangles[windows[i]];
			const auto [r0, r1] = cell_span(r.y0, r.y1, origin_y, rows, r.n_rows);
			for (auto row = r0; row < r1; ++row)
				for (auto col = spans[i].first; col < spans[i].second; ++col)
					out.classes[row * columns + col] = cls_window;
		}
	}
	for (const auto &rect : out.rectangles) {
		if (rect.kind == Kind::Door) {
			const auto [c0, c1] =
					cell_span(rect.x0, rect.x1, origin_x, columns, rect.n_columns);
			const auto count = static_cast<std::size_t>(std::clamp<long long>(
					static_cast<long long>(imgops::round_half_even(rect.height_m())), 1,
					static_cast<long long>(rows)));
			for (auto row = rows - count; row < rows; ++row)
				for (auto col = c0; col < c1; ++col)
					out.classes[row * columns + col] = cls_door;
		} else if (rect.kind == Kind::Shop) {
			const auto [c0, c1] =
					cell_span(rect.x0, rect.x1, origin_x, columns, rect.n_columns);
			const auto [r0, unused] =
					cell_span(rect.y0, rect.y1, origin_y, rows, rect.n_rows);
			(void)unused;
			for (auto row = r0; row < rows; ++row)
				for (auto col = c0; col < c1; ++col)
					if (out.classes[row * columns + col] != cls_door)
						out.classes[row * columns + col] = cls_window;
		}
	}

	std::vector<imgops::Lab> selected;
	std::vector<double> channel;
	std::vector<std::pair<std::size_t, bool>> pixels;
	selected.reserve(pixels_per_metre * pixels_per_metre);
	channel.reserve(pixels_per_metre * pixels_per_metre);
	pixels.reserve(pixels_per_metre * pixels_per_metre);
	for (std::size_t row = 0; row < rows; ++row)
		for (std::size_t col = 0; col < columns; ++col) {
			const auto cell = row * columns + col;
			const auto x0 = static_cast<std::int64_t>(origin_x) +
							static_cast<std::int64_t>(col * pixels_per_metre);
			const auto y0 = static_cast<std::int64_t>(origin_y) +
							static_cast<std::int64_t>(row * pixels_per_metre);
			pixels.clear();
			std::size_t soft_count = 0, vegetation_count = 0;
			for (std::size_t dy = 0; dy < pixels_per_metre; ++dy)
				for (std::size_t dx = 0; dx < pixels_per_metre; ++dx) {
					const auto x = x0 + static_cast<std::int64_t>(dx);
					const auto y = y0 + static_cast<std::int64_t>(dy);
					if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(width) ||
							y >= static_cast<std::int64_t>(height))
						continue;
					const auto i = static_cast<std::size_t>(y) * width +
								   static_cast<std::size_t>(x);
					if (!texture.valid[i])
						continue;
					soft_count += soft.at(
							static_cast<std::size_t>(x), static_cast<std::size_t>(y));
					vegetation_count += vegetation[i];
					pixels.emplace_back(i, cleaned.at(static_cast<std::size_t>(x),
												   static_cast<std::size_t>(y)));
				}
			if (pixels.empty()) {
				out.classes[cell] = cls_nodata;
				continue;
			}
			out.evidence[cell] = static_cast<double>(soft_count) / pixels.size();
			if (static_cast<double>(vegetation_count) > .5 * pixels.size() ||
					(static_cast<double>(pixels.size()) <
									unknown_valid * pixels_per_metre * pixels_per_metre &&
							out.classes[cell] == cls_wall))
				out.classes[cell] = cls_unknown;
			const bool want_open =
					out.classes[cell] == cls_window || out.classes[cell] == cls_door;
			std::size_t selected_count = 0;
			for (const auto &[i, is_open] : pixels)
				selected_count += is_open == want_open;
			selected.clear();
			for (const auto &[i, is_open] : pixels)
				if (!selected_count || is_open == want_open)
					selected.push_back(lab[i]);
			imgops::Lab colour{};
			for (std::size_t c = 0; c < colour.size(); ++c) {
				channel.clear();
				for (const auto &value : selected)
					channel.push_back(value[c]);
				colour[c] = imgops::median(channel);
			}
			out.colours[cell] = imgops::oklab_to_rgb8(colour);
			out.mask[cell] = std::any_of(pixels.begin(), pixels.end(),
					[&](const auto &pixel) { return pixel.second; });
		}
	return out;
}
} // namespace arnis::mapillary::openings
