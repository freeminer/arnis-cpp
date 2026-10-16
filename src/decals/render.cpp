#include "render.h"
#include "font.h"
#include "pictograms.h"
#include "posters.h"
#include "templates.h"
#include "../map_item_palette.h"
#include "stb_image.h"
#include <fstream>
#include <iterator>
namespace arnis::decals
{
namespace
{
void local_map(Canvas &canvas, int x, int z, const std::optional<PreviewRaster> &preview)
{
	const int w = static_cast<int>(canvas.width), h = static_cast<int>(canvas.height);
	const int span = LOCAL_MAP_HALF_SPAN * 2;
	if (preview && preview->image && preview->step > 0 && preview->image->width &&
			preview->image->height) {
		const auto &image = *preview->image;
		for (int j = 0; j < h; ++j)
			for (int i = 0; i < w; ++i) {
				const int wx = x - LOCAL_MAP_HALF_SPAN + i * span / w;
				const int wz = z - LOCAL_MAP_HALF_SPAN + j * span / h;
				const int px = (wx - preview->min_x) / static_cast<int>(preview->step);
				const int pz = (wz - preview->min_z) / static_cast<int>(preview->step);
				std::uint8_t color = colors::LIGHT_GRAY;
				if (px >= 0 && pz >= 0 && px < static_cast<int>(image.width) &&
						pz < static_cast<int>(image.height)) {
					const auto offset =
							(static_cast<std::size_t>(pz) * image.width + px) * 4;
					if (offset + 2 < image.pixels.size())
						color = map_palette::nearest_map_color(image.pixels[offset],
								image.pixels[offset + 1], image.pixels[offset + 2]);
				}
				canvas.set(i, j, color);
			}
	} else {
		canvas.fill(colors::LIGHT_GRAY);
	}

	canvas.stroke_rect(0, 0, w, h, 4, colors::NEAR_BLACK);
	const int cx = w / 2, cy = h / 2;
	canvas.disc(cx, cy, 11, colors::WHITE);
	canvas.disc(cx, cy, 8, colors::RED);
	canvas.polygon({{float(w - 26), 12.0f}, {float(w - 14), 40.0f},
						   {float(w - 26), 32.0f}, {float(w - 38), 40.0f}},
			colors::WHITE);
	canvas.polygon({{float(w - 26), 16.0f}, {float(w - 18), 36.0f},
						   {float(w - 26), 30.0f}, {float(w - 34), 36.0f}},
			colors::RED);
	font::Font::get(font::FontSize::S18)
			.draw_centered(canvas, w - 26, 42, "N", colors::WHITE, 1);
}

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
Canvas render(const DecalKey &key, const std::optional<PreviewRaster> &preview)
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
				} else if constexpr (std::is_same_v<Key, LocalMapKey>)
					local_map(canvas, value.x, value.z, preview);
				else
					templates::blank_plate(canvas, colors::LIGHT_GRAY);
			},
			key);
	return canvas;
}
}
