#include "render.h"
#include "pictograms.h"
#include "posters.h"
#include "templates.h"
#include "stb_image.h"
#include <fstream>
#include <iterator>
namespace arnis::decals
{
namespace
{
bool image(Canvas &canvas, const std::filesystem::path &path)
{
	std::ifstream file(path, std::ios::binary);
	if (!file)
		return false;
	std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(file)), {});
	int width = 0, height = 0, channels = 0;
	unsigned char *decoded = stbi_load_from_memory(
			bytes.data(), bytes.size(), &width, &height, &channels, 4);
	if (!decoded)
		return false;
	if (width == int(canvas.width) && height == int(canvas.height))
		canvas.blit_rgba(decoded, width, height);
	else {
		std::vector<unsigned char> resized(std::size_t(canvas.width) * canvas.height * 4);
		for (std::uint32_t y = 0; y < canvas.height; ++y)
			for (std::uint32_t x = 0; x < canvas.width; ++x) {
				const int sx =
						std::min(width - 1, int(std::uint64_t(x) * width / canvas.width));
				const int sy = std::min(
						height - 1, int(std::uint64_t(y) * height / canvas.height));
				std::copy_n(decoded + (std::size_t(sy) * width + sx) * 4, 4,
						resized.data() + (std::size_t(y) * canvas.width + x) * 4);
			}
		canvas.blit_rgba(resized.data(), canvas.width, canvas.height);
	}
	stbi_image_free(decoded);
	return true;
}
}
Canvas render(const DecalKey &key)
{
	const auto [cols, rows] = key.dims();
	Canvas canvas(cols, rows);
	std::visit(
			[&](const auto &value) {
				using Key = std::decay_t<decltype(value)>;
				if constexpr (std::is_same_v<Key, TextKey>)
					templates::text_sign(canvas, value.style, value.text);
				else if constexpr (std::is_same_v<Key, TrafficKey>)
					templates::traffic_sign(canvas, value.sign);
				else if constexpr (std::is_same_v<Key, SpeedLimitKey>)
					templates::speed_limit(canvas, value.value, value.mph, value.style);
				else if constexpr (std::is_same_v<Key, RouteShieldKey>)
					templates::route_shield(canvas, value.style, value.text);
				else if constexpr (std::is_same_v<Key, PictogramKey>) {
					if (const auto path = pictograms::asset(value.name);
							!path || !image(canvas, *path))
						templates::blank_plate(canvas, colors::GRAY);
				} else if constexpr (std::is_same_v<Key, PosterKey>) {
					if (!image(canvas, posters::billboard(value.variant)))
						templates::blank_plate(canvas, colors::LIGHT_GRAY);
					canvas.stroke_rect(
							0, 0, canvas.width, canvas.height, 3, colors::NEAR_BLACK);
				} else if constexpr (std::is_same_v<Key, ColumnPosterKey>) {
					if (!image(canvas, posters::column(value.variant)))
						templates::blank_plate(canvas, colors::LIGHT_GRAY);
					canvas.stroke_rect(
							0, 0, canvas.width, canvas.height, 3, colors::NEAR_BLACK);
				} else
					templates::blank_plate(canvas, colors::LIGHT_GRAY);
			},
			key);
	return canvas;
}
}
