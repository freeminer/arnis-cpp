#include "imgops.h"

#include <algorithm>
#include <cmath>
#include <deque>

namespace arnis::mapillary::imgops
{
namespace
{
std::array<double, 3> mul(const double m[3][3], std::array<double, 3> v)
{
	return {m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
			m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
			m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]};
}
double srgb_linear(double c)
{
	return c <= 0.04045 ? c / 12.92 : std::pow((c + 0.055) / 1.055, 2.4);
}
double linear_srgb(double c)
{
	c = std::clamp(c, 0.0, 1.0);
	return c <= 0.0031308 ? 12.92 * c : 1.055 * std::pow(c, 1.0 / 2.4) - .055;
}
}

Lab srgb_to_oklab(Rgb rgb)
{
	const double xyz[3][3] = {{0.4122214708, 0.5363325363, 0.0514459929},
			{0.2119034982, 0.6806995451, 0.1073969566},
			{0.0883024619, 0.2817188376, 0.6299787005}};
	const double lms[3][3] = {{0.8190224432, 0.3619062563, -0.1288737826},
			{0.0329836672, 0.9292868616, 0.0361446664},
			{0.0481771996, 0.2642395249, 0.6335478258}};
	const auto l = mul(xyz, {srgb_linear(rgb[0] / 255.0), srgb_linear(rgb[1] / 255.0),
									srgb_linear(rgb[2] / 255.0)});
	const auto m = mul(lms, l);
	const auto c = std::array<double, 3>{std::cbrt(std::max(0.0, m[0])),
			std::cbrt(std::max(0.0, m[1])), std::cbrt(std::max(0.0, m[2]))};
	return {0.2104542553 * c[0] + 0.7936177850 * c[1] - 0.0040720468 * c[2],
			1.9779984951 * c[0] - 2.4285922050 * c[1] + 0.4505937099 * c[2],
			0.0259040371 * c[0] + 0.7827717662 * c[1] - 0.8086757660 * c[2]};
}

Lab oklab_to_srgb(Lab lab)
{
	const double lms[3][3] = {{1.0, 0.3963377774, 0.2158037573},
			{1.0, -0.1055613458, -0.0638541728}, {1.0, -0.0894841775, -1.2914855480}};
	const double xyz[3][3] = {{4.0767419614, -3.3077121752, 0.2309700407},
			{-1.2684382023, 2.6097577853, -0.3413194659},
			{-0.0041960228, -0.7034187250, 1.7076147197}};
	const auto c = mul(lms, lab);
	const auto m = std::array<double, 3>{
			c[0] * c[0] * c[0], c[1] * c[1] * c[1], c[2] * c[2] * c[2]};
	const auto linear = mul(xyz, m);
	return {linear_srgb(linear[0]), linear_srgb(linear[1]), linear_srgb(linear[2])};
}

Rgb oklab_to_rgb8(Lab lab)
{
	const auto srgb = oklab_to_srgb(lab);
	Rgb out{};
	for (int i = 0; i < 3; ++i)
		out[i] = static_cast<std::uint8_t>(
				std::clamp(std::llround(srgb[i] * 255.0), 0LL, 255LL));
	return out;
}

double oklab_distance(Lab a, Lab b)
{
	const double dl = a[0] - b[0], da = a[1] - b[1], db = a[2] - b[2];
	return std::sqrt(dl * dl + da * da + db * db);
}

double round_half_even(double value)
{
	const double floor_value = std::floor(value), fraction = value - floor_value;
	if (fraction < .5)
		return floor_value;
	if (fraction > .5)
		return floor_value + 1.0;
	return std::fmod(floor_value, 2.0) == 0.0 ? floor_value : floor_value + 1.0;
}

bool Mask::get(std::ptrdiff_t x, std::ptrdiff_t y) const
{
	return x >= 0 && y >= 0 && static_cast<std::size_t>(x) < w &&
		   static_cast<std::size_t>(y) < h &&
		   at(static_cast<std::size_t>(x), static_cast<std::size_t>(y));
}
std::size_t Mask::count() const
{
	return std::count(bits.begin(), bits.end(), true);
}
bool Mask::any() const
{
	return std::any_of(bits.begin(), bits.end(), [](bool b) { return b; });
}
Mask erode(const Mask &m, std::size_t kw, std::size_t kh)
{
	Mask out(m.w, m.h);
	if (!kw || !kh)
		return out;
	const auto ax = kw / 2, ay = kh / 2;
	for (std::size_t y = 0; y < m.h; ++y)
		for (std::size_t x = 0; x < m.w; ++x) {
			bool keep = true;
			for (std::size_t dy = 0; dy < kh && keep; ++dy)
				for (std::size_t dx = 0; dx < kw; ++dx)
					if (const auto sx = std::ptrdiff_t(x + dx) - std::ptrdiff_t(ax),
							sy = std::ptrdiff_t(y + dy) - std::ptrdiff_t(ay);
							sx >= 0 && sy >= 0 && sx < std::ptrdiff_t(m.w) &&
							sy < std::ptrdiff_t(m.h) && !m.at(sx, sy)) {
						keep = false;
						break;
					}
			out.set(x, y, keep);
		}
	return out;
}
Mask dilate(const Mask &m, std::size_t kw, std::size_t kh)
{
	Mask out(m.w, m.h);
	if (!kw || !kh)
		return out;
	const auto ax = kw / 2, ay = kh / 2;
	for (std::size_t y = 0; y < m.h; ++y)
		for (std::size_t x = 0; x < m.w; ++x) {
			bool hit = false;
			for (std::size_t dy = 0; dy < kh && !hit; ++dy)
				for (std::size_t dx = 0; dx < kw; ++dx)
					if (m.get(std::ptrdiff_t(x + dx) - std::ptrdiff_t(ax),
								std::ptrdiff_t(y + dy) - std::ptrdiff_t(ay))) {
						hit = true;
						break;
					}
			out.set(x, y, hit);
		}
	return out;
}
Mask open(const Mask &m, std::size_t kw, std::size_t kh)
{
	return dilate(erode(m, kw, kh), kw, kh);
}
Mask close(const Mask &m, std::size_t kw, std::size_t kh)
{
	return erode(dilate(m, kw, kh), kw, kh);
}
Labels connected_components(const Mask &m)
{
	Labels out{m.w, m.h, std::vector<std::uint32_t>(m.w * m.h),
			{Component{0, 0, m.w, m.h, 0}}};
	std::uint32_t next = 0;
	const int dirs[4][2]{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
	for (std::size_t y = 0; y < m.h; ++y)
		for (std::size_t x = 0; x < m.w; ++x) {
			if (!m.at(x, y) || out.labels[y * m.w + x])
				continue;
			++next;
			std::deque<std::pair<std::size_t, std::size_t>> q{{x, y}};
			out.labels[y * m.w + x] = next;
			Component c{x, y, 1, 1, 0};
			while (!q.empty()) {
				auto [px, py] = q.front();
				q.pop_front();
				++c.area;
				c.x = std::min(c.x, px);
				c.y = std::min(c.y, py);
				c.w = std::max(c.w, px - c.x + 1);
				c.h = std::max(c.h, py - c.y + 1);
				for (auto &d : dirs) {
					auto nx = std::ptrdiff_t(px) + d[0], ny = std::ptrdiff_t(py) + d[1];
					if (nx >= 0 && ny >= 0 && nx < std::ptrdiff_t(m.w) &&
							ny < std::ptrdiff_t(m.h) && m.at(nx, ny) &&
							!out.labels[ny * m.w + nx]) {
						out.labels[ny * m.w + nx] = next;
						q.emplace_back(nx, ny);
					}
				}
			}
			out.components.push_back(c);
		}
	for (std::size_t i = 0; i < m.bits.size(); ++i)
		if (!m.bits[i])
			++out.components[0].area;
	return out;
}

std::vector<std::uint8_t> median_filter_u8(const std::vector<std::uint8_t> &src,
		std::size_t width, std::size_t height, std::size_t kernel)
{
	if (!kernel || !(kernel & 1) || src.size() != width * height)
		return {};
	std::vector<std::uint8_t> out(src.size()), window;
	window.reserve(kernel * kernel);
	const auto radius = static_cast<std::ptrdiff_t>(kernel / 2);
	for (std::size_t y = 0; y < height; ++y)
		for (std::size_t x = 0; x < width; ++x) {
			window.clear();
			for (std::ptrdiff_t dy = -radius; dy <= radius; ++dy)
				for (std::ptrdiff_t dx = -radius; dx <= radius; ++dx) {
					const auto sy = std::clamp<std::ptrdiff_t>(
							static_cast<std::ptrdiff_t>(y) + dy, 0, height - 1);
					const auto sx = std::clamp<std::ptrdiff_t>(
							static_cast<std::ptrdiff_t>(x) + dx, 0, width - 1);
					window.push_back(src[static_cast<std::size_t>(sy) * width + sx]);
				}
			std::sort(window.begin(), window.end());
			out[y * width + x] = window[window.size() / 2];
		}
	return out;
}

double median(std::vector<double> values)
{
	if (values.empty())
		return 0.0;
	std::sort(values.begin(), values.end());
	const auto n = values.size();
	return n & 1 ? values[n / 2] : .5 * (values[n / 2 - 1] + values[n / 2]);
}

double percentile(std::vector<double> values, double q)
{
	if (values.empty())
		return 0.0;
	std::sort(values.begin(), values.end());
	const double pos = std::clamp(q, 0.0, 100.0) / 100.0 * (values.size() - 1);
	const auto lo = static_cast<std::size_t>(std::floor(pos));
	if (lo + 1 >= values.size())
		return values.back();
	return values[lo] + (pos - lo) * (values[lo + 1] - values[lo]);
}
}
