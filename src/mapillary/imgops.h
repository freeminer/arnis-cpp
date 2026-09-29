#pragma once

#include <array>
#include <cstdint>
#include <vector>
#include <utility>

namespace arnis::mapillary::imgops
{
using Rgb = std::array<std::uint8_t, 3>;
using Lab = std::array<double, 3>;

Lab srgb_to_oklab(Rgb rgb);
Lab oklab_to_srgb(Lab lab);
Rgb oklab_to_rgb8(Lab lab);
double oklab_distance(Lab a, Lab b);
double round_half_even(double value);
double median(std::vector<double> values);
double percentile(std::vector<double> values, double q);
std::vector<std::uint8_t> median_filter_u8(const std::vector<std::uint8_t> &src,
		std::size_t width, std::size_t height, std::size_t kernel);

struct Mask
{
	std::size_t w = 0, h = 0;
	std::vector<bool> bits;
	Mask() = default;
	Mask(std::size_t width, std::size_t height) : w(width), h(height), bits(w * h) {}
	static Mask from_bits(std::size_t width, std::size_t height, std::vector<bool> values)
	{
		if (values.size() != width * height)
			return {};
		Mask out(width, height);
		out.bits = std::move(values);
		return out;
	}
	bool get(std::ptrdiff_t x, std::ptrdiff_t y) const;
	void set(std::size_t x, std::size_t y, bool value) { bits[y * w + x] = value; }
	bool at(std::size_t x, std::size_t y) const { return bits[y * w + x]; }
	std::size_t count() const;
	bool any() const;
};
struct Component
{
	std::size_t x, y, w, h, area;
};
struct Labels
{
	std::size_t w = 0, h = 0;
	std::vector<std::uint32_t> labels;
	std::vector<Component> components;
	std::uint32_t at(std::size_t x, std::size_t y) const { return labels[y * w + x]; }
	std::size_t count() const { return components.size(); }
};
Mask erode(const Mask &, std::size_t kw, std::size_t kh);
Mask dilate(const Mask &, std::size_t kw, std::size_t kh);
Mask open(const Mask &, std::size_t kw, std::size_t kh);
Mask close(const Mask &, std::size_t kw, std::size_t kh);
Labels connected_components(const Mask &);
}
