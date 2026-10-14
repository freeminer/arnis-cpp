#include "image.h"

#include "../../../arnis_world_editor.h"
#include "stb_image.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <numbers>
#include <vector>

namespace arnis::building_facades
{
namespace
{
struct SampleWeight
{
	int index;
	double weight;
};

double sinc(double x)
{
	if (std::abs(x) < 1e-12)
		return 1.0;
	const double pix = std::numbers::pi * x;
	return std::sin(pix) / pix;
}

double lanczos3(double x)
{
	x = std::abs(x);
	return x < 3.0 ? sinc(x) * sinc(x / 3.0) : 0.0;
}

std::vector<std::vector<SampleWeight>> lanczos_weights(int source_size, int target_size)
{
	std::vector<std::vector<SampleWeight>> result(static_cast<std::size_t>(target_size));
	if (source_size <= 0 || target_size <= 0)
		return result;
	const double ratio = static_cast<double>(source_size) / target_size;
	const double filter_scale = std::max(1.0, ratio);
	const double support = 3.0 * filter_scale;
	for (int target = 0; target < target_size; ++target) {
		const double center = (target + 0.5) * ratio;
		const int first = std::max(0, static_cast<int>(std::floor(center - support)));
		const int last = std::min(
				source_size - 1, static_cast<int>(std::ceil(center + support)) - 1);
		auto &weights = result[static_cast<std::size_t>(target)];
		weights.reserve(static_cast<std::size_t>(std::max(0, last - first + 1)));
		double sum = 0.0;
		for (int source = first; source <= last; ++source) {
			const double weight = lanczos3((source + 0.5 - center) / filter_scale);
			if (weight == 0.0)
				continue;
			weights.push_back({source, weight});
			sum += weight;
		}
		if (sum != 0.0)
			for (auto &weight : weights)
				weight.weight /= sum;
	}
	return result;
}

RgbImage resize_lanczos3(int source_width, int source_height,
		const std::uint8_t *source_pixels, std::uint32_t width, std::uint32_t height)
{
	RgbImage out{width, height,
			std::vector<std::uint8_t>(static_cast<std::size_t>(width) * height * 3)};
	if (source_width == static_cast<int>(width) &&
			source_height == static_cast<int>(height)) {
		std::copy_n(source_pixels, out.pixels.size(), out.pixels.data());
		return out;
	}

	const auto x_weights = lanczos_weights(source_width, static_cast<int>(width));
	const auto y_weights = lanczos_weights(source_height, static_cast<int>(height));
	std::vector<std::array<double, 3>> horizontal(
			static_cast<std::size_t>(source_height) * width);
	for (int y = 0; y < source_height; ++y)
		for (std::uint32_t x = 0; x < width; ++x) {
			std::array<double, 3> value{};
			for (const auto &sample : x_weights[x]) {
				const auto offset =
						(static_cast<std::size_t>(y) * source_width + sample.index) * 3;
				for (std::size_t channel = 0; channel < value.size(); ++channel)
					value[channel] += source_pixels[offset + channel] * sample.weight;
			}
			horizontal[static_cast<std::size_t>(y) * width + x] = value;
		}
	for (std::uint32_t y = 0; y < height; ++y)
		for (std::uint32_t x = 0; x < width; ++x) {
			std::array<double, 3> value{};
			for (const auto &sample : y_weights[y]) {
				const auto &pixel =
						horizontal[static_cast<std::size_t>(sample.index) * width + x];
				for (std::size_t channel = 0; channel < value.size(); ++channel)
					value[channel] += pixel[channel] * sample.weight;
			}
			const auto offset = (static_cast<std::size_t>(y) * width + x) * 3;
			for (std::size_t channel = 0; channel < value.size(); ++channel)
				out.pixels[offset + channel] = static_cast<std::uint8_t>(
						std::clamp(std::lround(value[channel]), 0L, 255L));
		}
	return out;
}
} // namespace

const std::uint8_t *RgbImage::pixel(std::uint32_t x, std::uint32_t y) const
{
	if (x >= width || y >= height)
		return nullptr;
	return pixels.data() + (static_cast<std::size_t>(y) * width + x) * 3;
}

std::optional<RgbImage> load_image(const std::filesystem::path &directory,
		const Entry &entry, double pixels_per_metre, std::string *error)
{
	if (!(pixels_per_metre > 0.0) || entry.file.empty()) {
		if (error)
			*error = "invalid image scale or empty file name";
		return std::nullopt;
	}
	try {
		int source_width = 0, source_height = 0, channels = 0;
		const auto path = (directory / entry.file).string();
		std::unique_ptr<stbi_uc, decltype(&stbi_image_free)> source(
				stbi_load(path.c_str(), &source_width, &source_height, &channels, 3),
				stbi_image_free);
		if (!source || source_width <= 0 || source_height <= 0) {
			if (error)
				*error = source ? "image has no pixels"
								: (stbi_failure_reason() ? stbi_failure_reason()
														 : "image decoder failed");
			return std::nullopt;
		}
		const auto [w, h] = entry.scaled_size(pixels_per_metre);
		if (!w || !h) {
			if (error)
				*error = "scaled image dimensions are empty";
			return std::nullopt;
		}
		return resize_lanczos3(source_width, source_height, source.get(), w, h);
	} catch (const std::exception &e) {
		if (error)
			*error = e.what();
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
	const std::size_t stride = std::size_t(ow) * 3;
	const std::size_t source_stride = std::size_t(source.width) * 3;
	// Rust's fitter builds these separable lookup tables once per panel. Repeated
	// storeys often address the same source row, so copy that completed row
	// rather than repeating a per-pixel gather.
	std::vector<std::size_t> columns;
	columns.reserve(ow);
	for (std::uint32_t x = 0; x < ow; ++x) {
		const auto sx =
				fit.source_x(x0_m + (static_cast<double>(x) + .5) / ppm, source.width);
		columns.push_back(std::size_t(sx) * 3);
	}
	std::vector<std::uint32_t> rows;
	rows.reserve(oh);
	for (std::uint32_t y = 0; y < oh; ++y)
		rows.push_back(
				fit.source_y(y1_m - (static_cast<double>(y) + .5) / ppm, source.height));
	std::vector<std::size_t> built(
			source.height, std::numeric_limits<std::size_t>::max());
	for (std::uint32_t y = 0; y < oh; ++y) {
		const std::size_t destination = std::size_t(y) * stride;
		const auto sy = rows[y];
		if (built[sy] != std::numeric_limits<std::size_t>::max()) {
			std::copy_n(out.pixels.data() + built[sy], stride,
					out.pixels.data() + destination);
			continue;
		}
		const auto source_row = std::size_t(sy) * source_stride;
		for (std::size_t x = 0; x < columns.size(); ++x) {
			const auto from = source_row + columns[x];
			std::copy_n(source.pixels.data() + from, 3,
					out.pixels.data() + destination + x * 3);
		}
		built[sy] = destination;
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
