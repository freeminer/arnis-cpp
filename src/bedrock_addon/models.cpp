#include "models.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <png.h>

namespace arnis::bedrock_addon::models
{
namespace
{
constexpr std::uint32_t CELL = 8;
constexpr std::uint32_t PER_ROW = TEXTURE_WIDTH / CELL;
constexpr std::size_t FALLBACK = 7;
constexpr std::array<std::string_view, 34> BLOCKS{"minecraft:white_concrete",
		"minecraft:orange_concrete", "minecraft:magenta_concrete",
		"minecraft:light_blue_concrete", "minecraft:yellow_concrete",
		"minecraft:lime_concrete", "minecraft:pink_concrete", "minecraft:gray_concrete",
		"minecraft:light_gray_concrete", "minecraft:cyan_concrete",
		"minecraft:purple_concrete", "minecraft:blue_concrete",
		"minecraft:brown_concrete", "minecraft:green_concrete", "minecraft:red_concrete",
		"minecraft:black_concrete", "minecraft:white_terracotta",
		"minecraft:orange_terracotta", "minecraft:magenta_terracotta",
		"minecraft:light_blue_terracotta", "minecraft:yellow_terracotta",
		"minecraft:lime_terracotta", "minecraft:pink_terracotta",
		"minecraft:gray_terracotta", "minecraft:light_gray_terracotta",
		"minecraft:cyan_terracotta", "minecraft:purple_terracotta",
		"minecraft:blue_terracotta", "minecraft:brown_terracotta",
		"minecraft:green_terracotta", "minecraft:red_terracotta",
		"minecraft:black_terracotta", "minecraft:terracotta", "minecraft:tinted_glass"};
// These match map_renderer.rs; the blocks used by vehicle bodies have explicit map colours.
constexpr std::array<std::array<std::uint8_t, 3>, 34> COLOURS{{{207, 213, 214},
		{224, 97, 0}, {169, 48, 159}, {35, 137, 198}, {241, 175, 21}, {94, 169, 24},
		{214, 101, 143}, {55, 58, 62}, {125, 125, 115}, {21, 119, 136}, {100, 32, 156},
		{45, 47, 143}, {96, 60, 32}, {73, 91, 36}, {142, 33, 33}, {8, 10, 15},
		{210, 178, 161}, {162, 84, 38}, {149, 88, 109}, {113, 109, 138}, {186, 133, 35},
		{104, 118, 53}, {162, 78, 79}, {58, 42, 36}, {135, 107, 98}, {87, 91, 91},
		{118, 70, 86}, {74, 60, 91}, {77, 51, 36}, {76, 83, 42}, {143, 61, 47},
		{37, 23, 16}, {152, 94, 67}, {44, 38, 46}}};

enum class Shade : std::uint32_t
{
	Plain,
	Dim,
	Lit
};

std::size_t block_index(std::string_view block)
{
	const auto it = std::find(BLOCKS.begin(), BLOCKS.end(), block);
	return it == BLOCKS.end() ? FALLBACK : static_cast<std::size_t>(it - BLOCKS.begin());
}

std::array<std::uint8_t, 3> colour(std::size_t i)
{
	return COLOURS[i];
}

std::uint32_t cell(std::size_t block, Shade shade)
{
	return static_cast<std::uint32_t>(block) * 3 + static_cast<std::uint32_t>(shade);
}

double round3(float value)
{
	return std::round(static_cast<double>(value) * 1000.0) / 1000.0;
}

nlohmann::json cube(const Shape &shape, std::uint32_t texture_cell)
{
	const auto px = [](float value) { return round3(value * 16.0f); };
	const std::array<int, 2> uv{
			static_cast<int>((texture_cell % PER_ROW) * CELL + CELL / 4),
			static_cast<int>((texture_cell / PER_ROW) * CELL + CELL / 4)};
	const nlohmann::json face{{"uv", uv}, {"uv_size", {CELL / 2, CELL / 2}}};
	nlohmann::json result;
	result["origin"] = {px(shape.corner[0]), px(shape.corner[1]),
			px(-(shape.corner[2] + shape.size[2]))};
	result["size"] = {px(shape.size[0]), px(shape.size[1]), px(shape.size[2])};
	result["uv"] = {{"north", face}, {"south", face}, {"east", face}, {"west", face},
			{"up", face}, {"down", face}};
	if (shape.tilt != 0.0f) {
		result["pivot"] = {
				px(shape.corner[0]), px(shape.corner[1]), px(-shape.corner[2])};
		result["rotation"] = {
				round3(shape.tilt * 180.0f / std::numbers::pi_v<float>), 0.0, 0.0};
	}
	return result;
}

} // namespace

bool knows(std::string_view block) noexcept
{
	return std::find(BLOCKS.begin(), BLOCKS.end(), block) != BLOCKS.end();
}

std::vector<std::uint8_t> texture()
{
	std::vector<std::uint8_t> rgba(TEXTURE_WIDTH * TEXTURE_HEIGHT * 4, 0);
	for (std::size_t i = 0; i < BLOCKS.size(); ++i) {
		const auto rgb = colour(i);
		for (const Shade shade : {Shade::Plain, Shade::Dim, Shade::Lit}) {
			const auto c =
					static_cast<std::uint32_t>(i) * 3 + static_cast<std::uint32_t>(shade);
			const auto x0 = (c % PER_ROW) * CELL;
			const auto y0 = (c / PER_ROW) * CELL;
			std::array<std::uint8_t, 4> pixel{rgb[0], rgb[1], rgb[2], 255};
			if (shade == Shade::Dim)
				for (std::size_t channel = 0; channel < 3; ++channel)
					pixel[channel] = static_cast<std::uint8_t>(rgb[channel] * 9 / 16);
			if (shade == Shade::Lit)
				pixel[3] = 0;
			for (std::uint32_t y = y0; y < y0 + CELL; ++y)
				for (std::uint32_t x = x0; x < x0 + CELL; ++x)
					std::copy(pixel.begin(), pixel.end(),
							rgba.begin() +
									(static_cast<std::size_t>(y) * TEXTURE_WIDTH + x) *
											4);
		}
	}
	png_image image{};
	image.version = PNG_IMAGE_VERSION;
	image.width = TEXTURE_WIDTH;
	image.height = TEXTURE_HEIGHT;
	image.format = PNG_FORMAT_RGBA;
	png_alloc_size_t size = 0;
	if (!png_image_write_to_memory(&image, nullptr, &size, 0, rgba.data(), 0, nullptr))
		return {};
	std::vector<std::uint8_t> output(size);
	if (!png_image_write_to_memory(
				&image, output.data(), &size, 0, rgba.data(), 0, nullptr))
		return {};
	output.resize(size);
	return output;
}

nlohmann::json geometry(std::string_view name, const std::vector<Shape> &shapes)
{
	const auto append_bone =
			[&](std::string_view bone, Shade shade,
					std::optional<Lamp> lamp) -> std::optional<nlohmann::json> {
		nlohmann::json cubes = nlohmann::json::array();
		for (const auto &shape : shapes) {
			const bool selected = lamp ? shape.lamp == *lamp : shape.lamp == Lamp::Off;
			if (selected)
				cubes.push_back(cube(shape, cell(block_index(shape.block), shade)));
		}
		if (cubes.empty())
			return std::nullopt;
		return nlohmann::json{
				{"name", bone}, {"pivot", {0, 0, 0}}, {"cubes", std::move(cubes)}};
	};
	nlohmann::json bones = nlohmann::json::array();
	for (const auto &bone : {append_bone("body", Shade::Plain, Lamp::Off),
				 append_bone("lit", Shade::Lit, Lamp::On),
				 append_bone("brake_off", Shade::Dim, Lamp::Brake),
				 append_bone("lit_brake", Shade::Lit, Lamp::Brake)})
		if (bone)
			bones.push_back(*bone);
	float reach = 0.0f;
	float top = 0.0f;
	for (const auto &shape : shapes) {
		reach = std::max({reach, std::abs(shape.corner[0]),
				std::abs(shape.corner[0] + shape.size[0]), std::abs(shape.corner[2]),
				std::abs(shape.corner[2] + shape.size[2])});
		top = std::max(top, shape.corner[1] + shape.size[1]);
	}
	const float width = 2.0f * reach + 1.0f;
	const float height = top + 1.0f;
	nlohmann::json description{{"identifier", "geometry.arnis." + std::string(name)},
			{"texture_width", TEXTURE_WIDTH}, {"texture_height", TEXTURE_HEIGHT},
			{"visible_bounds_width", round3(std::ceil(width))},
			{"visible_bounds_height", round3(std::ceil(height))},
			{"visible_bounds_offset", {0.0, round3(height / 2.0f), 0.0}}};
	nlohmann::json geometry_entry{
			{"description", std::move(description)}, {"bones", std::move(bones)}};
	return {{"format_version", "1.12.0"},
			{"minecraft:geometry", nlohmann::json::array({std::move(geometry_entry)})}};
}

} // namespace arnis::bedrock_addon::models
