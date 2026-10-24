#include "structure_kit.h"

#include "../colors.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <nlohmann/json.hpp>
#include <tuple>

namespace arnis::structure_kit
{
namespace
{
using Colour = std::pair<Block, std::array<std::uint8_t, 3>>;
const Colour concrete[] = {
		{WHITE_CONCRETE, {207, 213, 214}},
		{ORANGE_CONCRETE, {224, 97, 0}},
		{MAGENTA_CONCRETE, {169, 48, 159}},
		{LIGHT_BLUE_CONCRETE, {35, 137, 198}},
		{YELLOW_CONCRETE, {241, 175, 21}},
		{LIME_CONCRETE, {94, 168, 24}},
		{PINK_CONCRETE, {213, 101, 142}},
		{GRAY_CONCRETE, {54, 57, 61}},
		{LIGHT_GRAY_CONCRETE, {125, 125, 115}},
		{CYAN_CONCRETE, {21, 119, 136}},
		{PURPLE_CONCRETE, {100, 31, 156}},
		{BLUE_CONCRETE, {44, 46, 143}},
		{BROWN_CONCRETE, {96, 59, 31}},
		{GREEN_CONCRETE, {73, 91, 36}},
		{RED_CONCRETE, {142, 32, 32}},
		{BLACK_CONCRETE, {8, 10, 15}},
};

std::array<double, 3> rotate(const std::array<double, 4> &q, const Point3 &v)
{
	const auto [x, y, z, w] = q;
	const std::array<double, 3> t{2.0 * (y * v[2] - z * v[1]),
			2.0 * (z * v[0] - x * v[2]), 2.0 * (x * v[1] - y * v[0])};
	return {v[0] + w * t[0] + y * t[2] - z * t[1], v[1] + w * t[1] + z * t[0] - x * t[2],
			v[2] + w * t[2] + x * t[1] - y * t[0]};
}

Block beam_block(std::uint8_t axis, Block block, Block vertical)
{
	if (axis == 1)
		return vertical;
	if (axis == 2 && block == CHAIN_X)
		return CHAIN_Z;
	return block;
}
} // namespace

Frame Frame::from_heading(double x, double z, double dx, double dz)
{
	const double length = std::hypot(dx, dz);
	return {x, z, length > 1e-9 ? dx / length : 1.0, length > 1e-9 ? dz / length : 0.0,
			1.0};
}

Frame Frame::flipped() const
{
	auto result = *this;
	result.side = -result.side;
	return result;
}

std::pair<double, double> Frame::v() const
{
	return {-uz * side, ux * side};
}

std::pair<double, double> Frame::world(double a, double b) const
{
	const auto [vx, vz] = v();
	return {ox + a * ux + b * vx, oz + a * uz + b * vz};
}

std::pair<double, double> Frame::local(double x, double z) const
{
	const auto [vx, vz] = v();
	const double dx = x - ox, dz = z - oz;
	return {dx * ux + dz * uz, dx * vx + dz * vz};
}

std::pair<int, int> Frame::cell(double a, double b) const
{
	const auto [x, z] = world(a, b);
	return {static_cast<int>(std::floor(x)), static_cast<int>(std::floor(z))};
}

std::vector<std::array<double, 4>> Frame::cells(
		double a0, double a1, double b0, double b1) const
{
	const std::array<std::pair<double, double>, 4> corners{
			world(a0, b0), world(a0, b1), world(a1, b0), world(a1, b1)};
	double min_x = std::numeric_limits<double>::max(), max_x = -min_x;
	double min_z = min_x, max_z = -min_x;
	for (const auto &[x, z] : corners) {
		min_x = std::min(min_x, x);
		max_x = std::max(max_x, x);
		min_z = std::min(min_z, z);
		max_z = std::max(max_z, z);
	}
	const int x0 = static_cast<int>(std::floor(min_x)) - 1;
	const int x1 = static_cast<int>(std::ceil(max_x)) + 1;
	const int z0 = static_cast<int>(std::floor(min_z)) - 1;
	const int z1 = static_cast<int>(std::ceil(max_z)) + 1;
	std::vector<std::array<double, 4>> out;
	for (int x = x0; x <= x1; ++x)
		for (int z = z0; z <= z1; ++z) {
			const auto [a, b] = local(x + 0.5, z + 0.5);
			if (a >= a0 && a <= a1 && b >= b0 && b <= b1)
				out.push_back({static_cast<double>(x), static_cast<double>(z), a, b});
		}
	return out;
}

std::pair<int, int> axis_of(double dx, double dz)
{
	return std::abs(dx) >= std::abs(dz) ? std::pair{dx < 0 ? -1 : 1, 0}
										: std::pair{0, dz < 0 ? -1 : 1};
}

const char *facing_name(std::pair<int, int> step)
{
	if (step == std::pair{0, -1})
		return "north";
	if (step == std::pair{0, 1})
		return "south";
	if (step == std::pair{1, 0})
		return "east";
	return "west";
}

BlockWithProperties with_prop(
		Block block, const std::string &key, const std::string &value)
{
	return {block, {{key, value}}};
}

BlockWithProperties with_props(
		Block block, const std::vector<std::pair<std::string, std::string>> &properties)
{
	BlockWithProperties::Properties values;
	for (const auto &[key, value] : properties)
		values[key] = value;
	return {block, std::move(values)};
}

BlockWithProperties stair_backed(Block block, std::pair<int, int> back)
{
	return with_props(block, {{"facing", facing_name(back)}, {"half", "bottom"}});
}

BlockWithProperties top_slab(Block block)
{
	return with_prop(block, "type", "top");
}

void place(WorldEditor &editor, const BlockWithProperties &block, int x, int y, int z)
{
	editor.set_block_with_properties_absolute(block, x, y, z, std::nullopt, std::nullopt);
}

std::optional<Block> concrete_for_colour(const std::string &text)
{
	auto rgb = color_text_to_rgb_tuple(text);
	if (!rgb)
		return std::nullopt;
	const auto [tl, ta, tb] =
			rgb_to_oklab(std::get<0>(*rgb), std::get<1>(*rgb), std::get<2>(*rgb));
	const double tc = std::hypot(ta, tb);
	double best = std::numeric_limits<double>::infinity();
	Block selected = WHITE_CONCRETE;
	for (const auto &[block, colour] : concrete) {
		const auto [l, a, b] = rgb_to_oklab(colour[0], colour[1], colour[2]);
		const double c = std::hypot(a, b), dl = 0.25 * (tl - l);
		double d;
		if (tc < 0.04) {
			d = (ta - a) * (ta - a) + (tb - b) * (tb - b);
		} else {
			const double ha = ta / tc * c, hb = tb / tc * c;
			d = 4.0 * ((ha - a) * (ha - a) + (hb - b) * (hb - b)) +
				0.64 * (tc - c) * (tc - c);
		}
		d += dl * dl;
		if (d < best) {
			best = d;
			selected = block;
		}
	}
	return selected;
}

std::uint64_t mix(std::uint64_t a, std::uint64_t b)
{
	std::uint64_t h = a ^ (b * 0x9E3779B97F4A7C15ULL);
	h ^= h >> 33;
	h *= 0xFF51AFD7ED558CCDULL;
	h ^= h >> 33;
	return h;
}

std::vector<std::pair<std::array<int, 3>, std::uint8_t>> line_cells(Point3 p, Point3 q)
{
	const Point3 d{q[0] - p[0], q[1] - p[1], q[2] - p[2]};
	const double length = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
	const int steps = std::max(1, static_cast<int>(std::ceil(length * 3.0)));
	const std::uint8_t horizontal = std::abs(d[0]) >= std::abs(d[2]) ? 0 : 2;
	std::vector<std::pair<std::array<int, 3>, std::uint8_t>> out;
	for (int i = 0; i <= steps; ++i) {
		const double t = static_cast<double>(i) / steps;
		std::array<int, 3> c{static_cast<int>(std::floor(p[0] + d[0] * t)),
				static_cast<int>(std::floor(p[1] + d[1] * t)),
				static_cast<int>(std::floor(p[2] + d[2] * t))};
		if (out.empty()) {
			out.emplace_back(c, horizontal);
			continue;
		}
		auto cur = out.back().first;
		if (cur == c)
			continue;
		for (std::uint8_t axis : {std::uint8_t{0}, std::uint8_t{2}, std::uint8_t{1}}) {
			if (cur[axis] == c[axis])
				continue;
			cur[axis] = c[axis];
			out.emplace_back(cur, axis);
		}
	}
	return out;
}

void beam(WorldEditor &editor, Point3 p, Point3 q, double thickness,
		const std::string &display_block, std::int64_t seed, Block block, Block vertical)
{
	// WorldEditor uses the stable generation-format values Java=0, Bedrock=1,
	// Luanti=2; keep structure placement independent of output backend headers.
	if (editor.format() == 0) {
		display_beam(editor, p, q, thickness, display_block, seed);
		return;
	}
	for (const auto &[cell, axis] : line_cells(p, q)) {
		if (axis == 1 && block == CHAIN_X) {
			editor.set_block_with_properties_absolute(with_prop(vertical, "axis", "y"),
					cell[0], cell[1], cell[2], std::nullopt, std::nullopt);
		} else {
			editor.set_block_absolute(beam_block(axis, block, vertical), cell[0], cell[1],
					cell[2], std::nullopt, std::nullopt);
		}
	}
}

void display_beam(WorldEditor &editor, Point3 p, Point3 q, double thickness,
		const std::string &display_block, std::int64_t seed)
{
	const Point3 d{q[0] - p[0], q[1] - p[1], q[2] - p[2]};
	const double length = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
	if (length < 1e-3)
		return;
	const Point3 u{d[0] / length, d[1] / length, d[2] / length};
	const double w = 1.0 + u[2];
	std::array<double, 4> quat{};
	if (w < 1e-6)
		quat = {0, 1, 0, 0};
	else {
		const double x = -u[1], y = u[0], n = std::sqrt(x * x + y * y + w * w);
		quat = {x / n, y / n, 0, w / n};
	}
	const Point3 size{thickness, thickness, length + thickness * 0.5};
	const auto r = rotate(quat, {size[0] / 2, size[1] / 2, size[2] / 2});
	auto floats = [](std::initializer_list<double> vals) {
		nlohmann::json result = nlohmann::json::array();
		for (double v : vals)
			result.push_back(v);
		return result;
	};
	nlohmann::json extra;
	extra["block_state"]["Name"] = display_block;
	extra["transformation"]["left_rotation"] =
			floats({quat[0], quat[1], quat[2], quat[3]});
	extra["transformation"]["right_rotation"] = floats({0, 0, 0, 1});
	extra["transformation"]["translation"] = floats({-r[0], -r[1], -r[2]});
	extra["transformation"]["scale"] = floats({size[0], size[1], size[2]});
	extra["view_range"] = 3.0;
	extra["Tags"] = {"arnis_rope"};
	editor.add_block_display((p[0] + q[0]) / 2, (p[1] + q[1]) / 2, (p[2] + q[2]) / 2,
			seed, std::move(extra));
}

} // namespace arnis::structure_kit
