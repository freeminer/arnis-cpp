#include "image.h"

#include "../../../png_holder.h"
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
		const auto [w, h] = entry.scaled_size(pixels_per_metre);
		if (!w || !h)
			return std::nullopt;
		RgbImage out{w, h, std::vector<std::uint8_t>(std::size_t(w) * h * 3)};
		for (std::uint32_t y = 0; y < h; ++y)
			for (std::uint32_t x = 0; x < w; ++x) {
				const auto sx = std::min(source.width() - 1,
						static_cast<int>(
								std::floor((x + .5) * source.width() / double(w))));
				const auto sy = std::min(source.height() - 1,
						static_cast<int>(
								std::floor((y + .5) * source.height() / double(h))));
				const auto c = source.get_pixel(sx, sy);
				if (!c)
					continue;
				const auto at = (std::size_t(y) * w + x) * 3;
				out.pixels[at] = c->getRed();
				out.pixels[at + 1] = c->getGreen();
				out.pixels[at + 2] = c->getBlue();
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
}
