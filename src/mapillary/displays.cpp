#include "displays.h"
#include "../../../arnis_world_editor.h"
#include <algorithm>
#include <bit>
#include <cmath>
#include <nlohmann/json.hpp>
namespace arnis::mapillary::displays
{
std::optional<std::pair<double, double>> outward_normal(
		std::pair<int, int> d, int sx, int sz)
{
	double x = d.first, z = d.second, l = std::hypot(x, z), side = z * sx - x * sz;
	if (l > 1e-9 && side > 0)
		return {{z / l, -x / l}};
	if (l > 1e-9 && side < 0)
		return {{-z / l, x / l}};
	double sl = std::hypot(sx, sz);
	return sl < 1e-9 ? std::nullopt
					 : std::optional<std::pair<double, double>>{{sx / sl, sz / sl}};
}
std::pair<double, double> right_of(std::pair<double, double> n)
{
	return {n.second, -n.first};
}
double yaw_deg(std::pair<double, double> n)
{
	return std::atan2(-n.first, n.second) * 180.0 / M_PI;
}
std::array<float, 4> left_rotation(std::pair<double, double> n)
{
	double h = std::atan2(n.first, n.second) / 2;
	return {0, float(std::sin(h)), 0, float(std::cos(h))};
}
std::pair<double, double> facing_of(std::array<float, 4> q)
{
	double y = q[1], w = q[3];
	return {2 * y * w, w * w - y * y};
}
bool flip_crop(std::pair<int, int> d, std::pair<double, double> n)
{
	auto r = right_of(n);
	return d.first * r.first + d.second * r.second < 0;
}
double cell_step(std::pair<int, int> d)
{
	int m = std::max(std::abs(d.first), std::abs(d.second));
	return m ? std::hypot(d.first, d.second) / m : 1.;
}
std::int8_t facing_index(std::pair<double, double> n)
{
	if (std::abs(n.first) >= std::abs(n.second))
		return n.first >= 0 ? 0 : 2;
	return n.second >= 0 ? 3 : 1;
}
Quad quad_for(const std::vector<std::pair<int, int>> &cells, std::pair<double, double> n,
		double step, int base, int h)
{
	auto r = right_of(n);
	double lo = 1e300, hi = -1e300, tm = -1e300;
	for (auto [x, z] : cells) {
		double px = x + .5, pz = z + .5, s = px * r.first + pz * r.second;
		lo = std::min(lo, s);
		hi = std::max(hi, s);
		tm = std::max(tm, px * n.first + pz * n.second);
	}
	double sc = (lo + hi) / 2,
		   plane = tm + .5 * (std::abs(n.first) + std::abs(n.second)) + PUSH_OUT;
	return {sc * r.first + plane * n.first, double(base) + h / 2.,
			sc * r.second + plane * n.second, (hi - lo) + step, double(h),
			left_rotation(n)};
}

std::vector<std::pair<int, int>> cut(int length, int maximum)
{
	std::vector<std::pair<int, int>> out;
	if (length <= 0 || maximum <= 0)
		return out;
	for (int start = 0; start < length; start += maximum)
		out.emplace_back(start, std::min(length, start + maximum));
	return out;
}

std::vector<Piece> pieces(const std::vector<std::pair<int, int>> &cells,
		std::pair<double, double> normal, double step, int base_y, int total_height)
{
	std::vector<Piece> out;
	if (cells.empty() || total_height <= 0 || step <= 0)
		return out;
	const int max_cells = std::max(1, static_cast<int>(MAX_PANEL / step));
	for (const auto &[p0, p1] : cut(static_cast<int>(cells.size()), max_cells))
		for (const auto &[b0, b1] : cut(total_height, MAX_PANEL)) {
			std::vector<std::pair<int, int>> footprint(
					cells.begin() + p0, cells.begin() + p1);
			out.push_back({p0, p1, b0, b1,
					quad_for(footprint, normal, step, base_y + b0, b1 - b0)});
		}
	return out;
}

namespace
{
std::int64_t name_seed(const std::string &name)
{
	std::uint64_t hash = 0x0102'0304'0506'0708ULL;
	for (const unsigned char byte : name) {
		hash ^= byte;
		hash *= 0x0000'0100'0000'01b3ULL;
	}
	return std::bit_cast<std::int64_t>(hash);
}

nlohmann::json display_nbt(const std::string &name, const Quad &quad)
{
	const std::string model = "arnis:" + name;
	return {{"item", {{"id", "minecraft:stone"}, {"count", 1},
							 {"components", {{"minecraft:item_model", model}}}}},
			{"item_display", "fixed"},
			{"transformation",
					{{"left_rotation",
							 {quad.rot[0], quad.rot[1], quad.rot[2], quad.rot[3]}},
							{"right_rotation", {0.0f, 0.0f, 0.0f, 1.0f}},
							{"translation", {0.0f, 0.0f, 0.0f}},
							{"scale", {static_cast<float>(quad.w),
											  static_cast<float>(quad.h), 1.0f}}}},
			{"billboard", "fixed"}, {"view_range", 16.0f}, {"width", 0.0f},
			{"height", 0.0f}};
}

std::uint32_t tex_side(double blocks, std::uint32_t px)
{
	return std::max<std::uint32_t>(
				   1, static_cast<std::uint32_t>(std::llround(blocks * px / 16.0))) *
		   16;
}
}
Sprite Sprite::of(double width, double height, std::uint32_t px)
{
	Sprite out{tex_side(width, px), tex_side(height, px), 0, 0};
	const double exact_w = std::max(1.0, std::round(width * px));
	const double exact_h = std::max(1.0, std::round(height * px));
	const double fit =
			std::min({double(out.side_w) / exact_w, double(out.side_h) / exact_h, 1.0});
	out.used_w = std::clamp<std::uint32_t>(std::llround(exact_w * fit), 1, out.side_w);
	out.used_h = std::clamp<std::uint32_t>(std::llround(exact_h * fit), 1, out.side_h);
	return out;
}
std::pair<double, double> Sprite::uv_extent() const
{
	return {16.0 * used_w / side_w, 16.0 * used_h / side_h};
}

std::vector<std::uint8_t> lay_out_rgb(const std::vector<std::uint8_t> &rgb,
		std::uint32_t input_width, std::uint32_t input_height, Sprite sprite)
{
	if (input_width == 0 || input_height == 0 ||
			rgb.size() != std::size_t(input_width) * input_height * 3 ||
			sprite.side_w == 0 || sprite.side_h == 0 || sprite.used_w == 0 ||
			sprite.used_h == 0)
		return {};
	std::vector<std::uint8_t> out(std::size_t(sprite.side_w) * sprite.side_h * 3);
	for (std::uint32_t y = 0; y < sprite.used_h; ++y) {
		const auto sy = std::min(input_height - 1,
				static_cast<std::uint32_t>((std::uint64_t(y) * input_height) /
										   std::max<std::uint32_t>(1, sprite.used_h)));
		for (std::uint32_t x = 0; x < sprite.used_w; ++x) {
			const auto sx = std::min(
					input_width - 1, static_cast<std::uint32_t>(
											 (std::uint64_t(x) * input_width) /
											 std::max<std::uint32_t>(1, sprite.used_w)));
			std::copy_n(rgb.begin() + (std::size_t(sy) * input_width + sx) * 3, 3,
					out.begin() + (std::size_t(y) * sprite.side_w + x) * 3);
		}
		for (std::uint32_t x = sprite.used_w; x < sprite.side_w; ++x)
			std::copy_n(out.begin() +
								(std::size_t(y) * sprite.side_w + sprite.used_w - 1) * 3,
					3, out.begin() + (std::size_t(y) * sprite.side_w + x) * 3);
	}
	for (std::uint32_t y = sprite.used_h; y < sprite.side_h; ++y)
		std::copy_n(out.begin() + (std::size_t(sprite.used_h - 1) * sprite.side_w) * 3,
				sprite.side_w * 3, out.begin() + (std::size_t(y) * sprite.side_w) * 3);
	return out;
}

std::string item_definition_json(const std::string &name)
{
	nlohmann::json model = nlohmann::json::object();
	model["type"] = "minecraft:model";
	model["model"] = "arnis:item/" + name;
	nlohmann::json item = nlohmann::json::object();
	item["model"] = std::move(model);
	return item.dump();
}
std::string model_json_for(const std::string &texture_name, Sprite sprite)
{
	const auto [u, v] = sprite.uv_extent();
	const auto texture = "arnis:block/" + texture_name;
	const nlohmann::json face{{"uv", {0.0, 0.0, u, v}}, {"texture", "#0"}};
	nlohmann::json model = nlohmann::json::object();
	model["textures"] = {{"0", texture}, {"particle", texture}};
	nlohmann::json element = nlohmann::json::object();
	element["from"] = {0.0, 0.0, 7.9};
	element["to"] = {16.0, 16.0, 8.1};
	element["faces"] = {{"north", face}, {"south", face}};
	model["elements"] = nlohmann::json::array({std::move(element)});
	model["display"]["fixed"] = {
			{"rotation", {0, 0, 0}}, {"translation", {0, 0, 0}}, {"scale", {1, 1, 1}}};
	return model.dump();
}

bool Registry::collect(const std::string &name,
		const std::vector<std::pair<int, int>> &cells, std::pair<double, double> normal,
		double step, int base_y, int total_height, const std::vector<std::uint8_t> &rgb,
		std::uint32_t rgb_width, std::uint32_t rgb_height, std::uint32_t pixels_per_block,
		const std::function<bool(const Quad &, const Panel &)> &sink)
{
	if (!enabled_ || cells.empty() || name.empty())
		return false;
	++stats_.candidates;
	if (!names_.insert(name).second) {
		++stats_.dropped;
		return false;
	}
	const auto all = pieces(cells, normal, step, base_y, total_height);
	if (all.empty()) {
		++stats_.dropped;
		return false;
	}
	bool placed = false;
	for (const auto &piece : all) {
		const auto sprite = Sprite::of(piece.quad.w, piece.quad.h, pixels_per_block);
		Panel panel{name, piece.quad.w, piece.quad.h,
				lay_out_rgb(rgb, rgb_width, rgb_height, sprite), sprite.side_w,
				sprite.side_h};
		if (panel.pixels.empty() || !sink || !sink(piece.quad, panel))
			continue;
		placed = true;
		++stats_.displays;
	}
	if (placed)
		++stats_.placed;
	else
		++stats_.dropped;
	return placed;
}

bool Registry::collect_to_editor(arnis::world_editor::WorldEditor &editor,
		const std::string &name, const std::vector<std::pair<int, int>> &cells,
		std::pair<double, double> normal, double step, int base_y, int total_height,
		const std::vector<std::uint8_t> &rgb, std::uint32_t rgb_width,
		std::uint32_t rgb_height, std::uint32_t pixels_per_block)
{
	return collect(name, cells, normal, step, base_y, total_height, rgb, rgb_width,
			rgb_height, pixels_per_block, [&](const Quad &quad, const Panel &panel) {
				const auto x = static_cast<int>(std::llround(quad.cx));
				const auto y = static_cast<int>(std::llround(quad.cy));
				const auto z = static_cast<int>(std::llround(quad.cz));
				const auto facing = facing_index(normal);
				if (editor.has_entity_sink() &&
						editor.add_item_display(quad.cx, quad.cy, quad.cz,
								name_seed(name), display_nbt(name, quad))) {
					editor.record_facade_panel(x, y, z, facing, panel.pixels,
							panel.pixel_width, panel.pixel_height);
					return true;
				}
				return editor.place_facade_panel(x, y, z, facing, panel.pixels,
						panel.pixel_width, panel.pixel_height);
			});
}
}
