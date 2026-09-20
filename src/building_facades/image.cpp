#include "image.h"

#include "../../../png_holder.h"
#include "../../../arnis_world_editor.h"
#include <algorithm>
#include <cmath>

namespace arnis::building_facades
{
const std::uint8_t *RgbImage::pixel(std::uint32_t x, std::uint32_t y) const
{
	if (x >= width || y >= height)
		return nullptr;
	return pixels.data() + (static_cast<std::size_t>(y) * width + x) * 3;
}

std::optional<RgbImage> load_image(const std::filesystem::path &directory,
		const Entry &entry, double pixels_per_metre)
{
	if (!(pixels_per_metre > 0.0) || entry.file.empty())
		return std::nullopt;
	try {
		const PngImage source((directory / entry.file).string());
		if (source.width() <= 0 || source.height() <= 0)
			return std::nullopt;
		const auto [w, h] = entry.scaled_size(pixels_per_metre);
		if (!w || !h)
			return std::nullopt;
		RgbImage out{w, h, std::vector<std::uint8_t>(std::size_t(w) * h * 3)};
		for (std::uint32_t y = 0; y < h; ++y)
			for (std::uint32_t x = 0; x < w; ++x) {
				const double fx = (x + .5) * source.width() / double(w) - .5;
				const double fy = (y + .5) * source.height() / double(h) - .5;
				const int sx = std::clamp(
						static_cast<int>(std::floor(fx)), 0, source.width() - 1);
				const int sy = std::clamp(
						static_cast<int>(std::floor(fy)), 0, source.height() - 1);
				const int sx1 = std::min(source.width() - 1, sx + 1);
				const int sy1 = std::min(source.height() - 1, sy + 1);
				const double tx = std::clamp(fx - std::floor(fx), 0.0, 1.0);
				const double ty = std::clamp(fy - std::floor(fy), 0.0, 1.0);
				const auto c00 = source.get_pixel(sx, sy);
				const auto c10 = source.get_pixel(sx1, sy);
				const auto c01 = source.get_pixel(sx, sy1);
				const auto c11 = source.get_pixel(sx1, sy1);
				if (!c00 || !c10 || !c01 || !c11)
					continue;
				const auto at = (std::size_t(y) * w + x) * 3;
				const auto blend = [&](auto channel) {
					const double a = channel(*c00) * (1.0 - tx) + channel(*c10) * tx;
					const double b = channel(*c01) * (1.0 - tx) + channel(*c11) * tx;
					return static_cast<std::uint8_t>(
							std::clamp(std::lround(a * (1.0 - ty) + b * ty), 0L, 255L));
				};
				out.pixels[at] = blend([](const auto &v) { return v.getRed(); });
				out.pixels[at + 1] = blend([](const auto &v) { return v.getGreen(); });
				out.pixels[at + 2] = blend([](const auto &v) { return v.getBlue(); });
			}
		return out;
	} catch (...) {
		return std::nullopt;
	}
}

std::optional<RgbImage> gather_region(const Fit &fit, const RgbImage &source, double x0_m,
		double x1_m, double y0_m, double y1_m)
{
	if (!source.valid() || fit.pixels_per_metre <= 0.0)
		return std::nullopt;
	const auto ppm = fit.pixels_per_metre;
	const auto ow = static_cast<std::uint32_t>(
			std::clamp(std::llround((x1_m - x0_m) * ppm), 0LL, 1LL << 15));
	const auto oh = static_cast<std::uint32_t>(
			std::clamp(std::llround((y1_m - y0_m) * ppm), 0LL, 1LL << 15));
	if (!ow || !oh)
		return std::nullopt;
	RgbImage out{ow, oh, std::vector<std::uint8_t>(std::size_t(ow) * oh * 3)};
	for (std::uint32_t y = 0; y < oh; ++y) {
		const auto sy = fit.source_y(y1_m - (y + .5) / ppm, source.height);
		for (std::uint32_t x = 0; x < ow; ++x) {
			const auto sx = fit.source_x(x0_m + (x + .5) / ppm, source.width);
			const auto *p = source.pixel(sx, sy);
			if (p)
				std::copy_n(p, 3, out.pixels.data() + (std::size_t(y) * ow + x) * 3);
		}
	}
	return out;
}

bool submit_panel(world_editor::WorldEditor &editor, int x, int y, int z,
		std::int8_t facing, const RgbImage &image)
{
	if (!image.valid())
		return false;
	return editor.place_facade_panel(
			x, y, z, facing, image.pixels, image.width, image.height);
}
}
