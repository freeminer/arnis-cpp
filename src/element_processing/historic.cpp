#include "historic.h"
#include "../deterministic_rng.h"
#include "../strict_parse.h"
#include "signage.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <iterator>
#include <limits>
#include <optional>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace arnis
{
namespace historic
{

namespace
{
std::optional<std::string> artwork_form(const tags_t &tags)
{
	if (tags.get("historic") == "housefigure" || tags.get("covered") == "yes" ||
			tags.get("indoor") == "yes" || tags.get("location") == "indoor" ||
			tags.get("support") == "wall_mounted" || tags.get("support") == "in_wall" ||
			tags.get("support") == "roof" || tags.get("support") == "building")
		return std::nullopt;
	if (const auto level = tags.get("level"); !level.empty()) {
		const auto end = level.find(';');
		std::string_view level_value = std::string_view(level).substr(0, end);
		while (!level_value.empty() &&
				std::isspace(static_cast<unsigned char>(level_value.front())))
			level_value.remove_prefix(1);
		while (!level_value.empty() &&
				std::isspace(static_cast<unsigned char>(level_value.back())))
			level_value.remove_suffix(1);
		const auto parsed = strict_parse::f64(level_value);
		if (parsed && static_cast<float>(*parsed) != 0.0f)
			return std::nullopt;
	}
	std::string form;
	if (tags.get("tourism") == "artwork")
		form = tags.get("artwork_type");
	else if (tags.get("historic") == "memorial")
		form = tags.get("memorial");
	if (form == "statue" || form == "bust" || form == "sculpture")
		return form;
	return std::nullopt;
}

Block figure_material(const tags_t &tags, std::uint64_t roll)
{
	auto material = tags.get("material");
	std::transform(material.begin(), material.end(), material.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	if (material.find("gold") != std::string::npos ||
			material.find("gilded") != std::string::npos)
		return GOLD_BLOCK;
	if (material.find("marble") != std::string::npos ||
			material.find("limestone") != std::string::npos)
		return SMOOTH_QUARTZ;
	if (material.find("stone") != std::string::npos ||
			material.find("granite") != std::string::npos ||
			material.find("sandstone") != std::string::npos ||
			material.find("concrete") != std::string::npos)
		return SMOOTH_STONE;
	if (material.find("bronze") != std::string::npos ||
			material.find("copper") != std::string::npos ||
			material.find("metal") != std::string::npos)
		return roll % 3 == 0 ? WAXED_EXPOSED_COPPER : WAXED_OXIDIZED_COPPER;
	switch (roll % 10) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
		return WAXED_OXIDIZED_COPPER;
	case 5:
	case 6:
		return WAXED_EXPOSED_COPPER;
	case 7:
	case 8:
		return SMOOTH_STONE;
	default:
		return SMOOTH_QUARTZ;
	}
}

const char *figure_material_name(const Block &block)
{
	if (block.id() == GOLD_BLOCK.id())
		return "minecraft:gold_block";
	if (block.id() == SMOOTH_QUARTZ.id())
		return "minecraft:smooth_quartz";
	if (block.id() == SMOOTH_STONE.id())
		return "minecraft:smooth_stone";
	if (block.id() == WAXED_EXPOSED_COPPER.id())
		return "minecraft:waxed_weathered_copper";
	if (block.id() == IRON_BLOCK.id())
		return "minecraft:iron_block";
	if (block.id() == POLISHED_BLACKSTONE.id())
		return "minecraft:polished_blackstone";
	if (block.id() == WAXED_COPPER_BLOCK.id())
		return "minecraft:waxed_copper_block";
	return "minecraft:waxed_oxidized_copper";
}

float artwork_yaw(const tags_t &tags, std::uint64_t roll)
{
	static constexpr std::string_view compass[] = {"n", "nne", "ne", "ene", "e", "ese",
			"se", "sse", "s", "ssw", "sw", "wsw", "w", "wnw", "nw", "nnw"};
	auto direction = tags.get("direction");
	std::transform(direction.begin(), direction.end(), direction.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	while (!direction.empty() &&
			std::isspace(static_cast<unsigned char>(direction.front())))
		direction.erase(direction.begin());
	while (!direction.empty() &&
			std::isspace(static_cast<unsigned char>(direction.back())))
		direction.pop_back();
	for (std::size_t i = 0; i < std::size(compass); ++i)
		if (direction == compass[i])
			return static_cast<float>(std::fmod(i * 22.5 + 180.0, 360.0));
	if (const auto degrees = strict_parse::f64(direction); degrees) {
		const float mapped = static_cast<float>(*degrees);
		if (std::isfinite(mapped))
			return std::fmod(std::fmod(mapped + 180.0f, 360.0f) + 360.0f, 360.0f);
	}
	return static_cast<float>(((roll >> 4) % 4) * 90);
}

using Turn = std::array<float, 3>;
using Quat = std::array<double, 4>;
using Point3 = std::array<double, 3>;
constexpr double PI = 3.14159265358979323846;
struct LimbPose
{
	Turn upper{}, lower{};
};
enum class FigureHold
{
	Nothing,
	Staff,
	Sword,
	Book,
	Scroll
};
struct FigurePose
{
	Turn head{};
	LimbPose right_arm{}, left_arm{}, right_leg{}, left_leg{};
	bool robe = false;
	FigureHold holds = FigureHold::Nothing;
};

constexpr LimbPose limb(Turn upper, Turn lower)
{
	return {upper, lower};
}
constexpr LimbPose STRAIGHT_LIMB{};
constexpr std::array<FigurePose, 7> FIGURE_POSES{{
		{{4, -8, 0}, limb({-10, 0, -4}, {-100, 70, 0}), limb({-4, 0, 6}, {-8, 0, 0}),
				limb({0, 0, -2}, {0, 0, 0}), limb({-10, 0, 3}, {16, 0, 0}), false,
				FigureHold::Nothing},
		{{-12, 0, 0}, limb({-150, 0, -12}, {-15, 0, 0}), limb({0, 0, 35}, {0, 0, -90}),
				limb({-8, 0, -3}, {12, 0, 0}), limb({4, 0, 2}, {0, 0, 0}), false,
				FigureHold::Nothing},
		{{24, 0, 0}, limb({-15, 0, -5}, {-70, 35, 0}), limb({-15, 0, 5}, {-70, -35, 0}),
				STRAIGHT_LIMB, STRAIGHT_LIMB, true, FigureHold::Book},
		{{-6, 10, 0}, limb({-125, 0, -10}, {-10, 0, 0}), limb({0, 0, 8}, {-12, 0, 0}),
				limb({6, 0, -2}, {0, 0, 0}), limb({-20, 0, 2}, {22, 0, 0}), false,
				FigureHold::Sword},
		{{6, 0, 0}, limb({-8, 0, -8}, {-75, 0, 0}), limb({-8, 0, 6}, {-110, -30, 0}),
				STRAIGHT_LIMB, STRAIGHT_LIMB, true, FigureHold::Staff},
		{{0, -15, 0}, limb({-4, 0, -5}, {-30, 0, 0}), limb({-85, 14, 0}, {0, 0, 0}),
				limb({4, 0, -2}, {0, 0, 0}), limb({-12, 0, 3}, {18, 0, 0}), false,
				FigureHold::Nothing},
		{{8, 6, 0}, limb({-12, 0, -4}, {-80, 20, 0}), limb({-2, 0, 5}, {-6, 0, 0}),
				limb({-10, 0, -3}, {16, 0, 0}), limb({0, 0, 2}, {0, 0, 0}), false,
				FigureHold::Scroll},
}};
constexpr Quat NO_TURN{0, 0, 0, 1};

Quat multiply(Quat a, Quat b)
{
	return {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
			a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
			a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
			a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}

Quat turn_quat(Turn turn)
{
	const auto axis = [](float degrees, std::size_t i) {
		const auto half = double(degrees) * (PI / 360.0);
		Quat q{0, 0, 0, std::cos(half)};
		q[i] = std::sin(half);
		return q;
	};
	return multiply(multiply(axis(turn[2], 2), axis(turn[1], 1)), axis(turn[0], 0));
}

Point3 rotate(Quat q, Point3 v)
{
	const Quat p{v[0], v[1], v[2], 0};
	const Quat inverse{-q[0], -q[1], -q[2], q[3]};
	const auto r = multiply(multiply(q, p), inverse);
	return {r[0], r[1], r[2]};
}

Point3 add(Point3 a, Point3 b)
{
	return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

bool add_artwork_figure_displays(WorldEditor &editor, const ProcessedNode &node,
		const char *material, int relative_y, bool bust, bool sculpture, bool horse,
		bool rider, float yaw, std::uint64_t roll, double y_fraction = 0.5,
		double figure_scale = 1.35, int pose_override = -1,
		std::optional<int> absolute_y_override = std::nullopt)
{
	// Rust's statues module uses one deterministic block-display entity per articulated
	// part. This native backend version keeps the same metre-based body proportions and
	// Java block-display transform, while leaving the non-Java fallback block-built.
	const auto x = static_cast<double>(node.x) + 0.5;
	const auto z = static_cast<double>(node.z) + 0.5;
	const auto y = absolute_y_override
						   ? static_cast<double>(*absolute_y_override) + y_fraction
						   : static_cast<double>(
									 editor.get_absolute_y(node.x, relative_y, node.z)) +
									 y_fraction;
	auto seed = static_cast<std::int64_t>(node.id * std::uint64_t(64));
	const auto add_box = [&](Point3 pivot, Point3 offset, Point3 size, Quat rotation,
								 double scale) {
		const auto rotated_offset = rotate(
				rotation, {offset[0] * scale, offset[1] * scale, offset[2] * scale});
		const Point3 translation{pivot[0] * scale + rotated_offset[0],
				pivot[1] * scale + rotated_offset[1],
				pivot[2] * scale + rotated_offset[2]};
		nlohmann::json extra = {{"block_state", {{"Name", material}}},
				{"Rotation", {yaw, 0.0f}},
				{"transformation", {{"left_rotation", {rotation[0], rotation[1],
															  rotation[2], rotation[3]}},
										   {"right_rotation", {0.0f, 0.0f, 0.0f, 1.0f}},
										   {"translation", translation},
										   {"scale", {size[0] * scale, size[1] * scale,
															 size[2] * scale}}}},
				{"view_range", 3.0}};
		return editor.add_block_display(x, y, z, seed++, std::move(extra));
	};
	const auto add_part = [&](Point3 centre, Point3 size) {
		return add_box({0, 0, 0},
				{centre[0] - size[0] / 2, centre[1] - size[1] / 2,
						centre[2] - size[2] / 2},
				size, NO_TURN, 1.0);
	};
	const auto articulated_part = [&](Point3 pivot, Point3 offset, Point3 size,
										  Quat rotation) {
		return add_box(pivot, offset, size, rotation, figure_scale);
	};
	const auto articulated_segment = [&](Point3 joint, Point3 measures, Quat rotation) {
		const double lap = measures[1] * 0.4;
		return articulated_part(joint, {-measures[1] / 2, -measures[0], -measures[2] / 2},
				{measures[1], measures[0] + lap, measures[2]}, rotation);
	};
	const auto articulated_limb = [&](Point3 joint, LimbPose pose, Point3 upper_size,
										  Point3 lower_size) {
		const auto upper = turn_quat(pose.upper);
		const auto lower = multiply(upper, turn_quat(pose.lower));
		const auto mid = add(joint, rotate(upper, {0, -upper_size[0], 0}));
		const auto end = add(mid, rotate(lower, {0, -lower_size[0], 0}));
		articulated_segment(joint, upper_size, upper);
		articulated_segment(mid, lower_size, lower);
		return std::tuple{lower, mid, end};
	};
	constexpr double RIDER_SHOULDER_X = 0.22;
	constexpr Point3 RIDER_UPPER_ARM{0.29, 0.105, 0.105};
	constexpr Point3 RIDER_FOREARM{0.31, 0.095, 0.095};
	if (bust) {
		constexpr double scale = 1.6;
		bool placed = add_box(
				{0, 0, 0}, {-0.22, 0.0, -0.12}, {0.44, 0.20, 0.24}, NO_TURN, scale);
		placed |= add_box(
				{0, 0, 0}, {-0.16, 0.20, -0.10}, {0.32, 0.06, 0.20}, NO_TURN, scale);
		placed |= add_box(
				{0, 0.25, 0}, {-0.055, 0, -0.055}, {0.11, 0.12, 0.11}, NO_TURN, scale);
		const auto head_turn = turn_quat({4, 0, 0});
		placed |= add_box(
				{0, 0.33, 0}, {-0.105, 0, -0.11}, {0.21, 0.25, 0.23}, head_turn, scale);
		placed |= add_box(
				{0, 0.33, 0}, {-0.02, 0.06, 0.105}, {0.04, 0.07, 0.05}, head_turn, scale);
		return placed;
	}
	if (rider) {
		constexpr double seat = 1.5;
		bool placed = articulated_part(
				{0, seat, -0.05}, {-0.18, 0, -0.12}, {0.36, 0.58, 0.24}, NO_TURN);
		const auto raised_horse = ((roll >> 16) & 1u) == 0;
		// Horse barrel, four articulated legs, neck, head, ears, and tail.
		placed |= articulated_part(
				{0, 0, 0}, {-0.23, 0.95, -0.8}, {0.46, 0.56, 1.55}, NO_TURN);
		const LimbPose raised =
				raised_horse ? limb({-42, 0, 0}, {80, 0, 0}) : STRAIGHT_LIMB;
		constexpr Point3 horse_thigh{0.52, 0.14, 0.18};
		constexpr Point3 horse_shin{0.5, 0.1, 0.11};
		for (const auto &[pivot, leg] : {
					 std::pair{Point3{-0.13, 1.05, 0.6}, STRAIGHT_LIMB},
					 std::pair{Point3{0.13, 1.05, 0.6}, raised},
					 std::pair{Point3{-0.13, 1.05, -0.62}, limb({-5, 0, 0}, {0, 0, 0})},
					 std::pair{Point3{0.13, 1.05, -0.62}, limb({8, 0, 0}, {0, 0, 0})}})
			articulated_limb(pivot, leg, horse_thigh, horse_shin);
		const auto neck = turn_quat({32, 0, 0});
		const Point3 withers{0, 1.32, 0.6};
		placed |= articulated_part(withers, {-0.12, 0, -0.17}, {0.24, 0.72, 0.32}, neck);
		const auto poll = add(withers, rotate(neck, {0, 0.66, 0}));
		const auto head_turn = turn_quat({38, 0, 0});
		placed |=
				articulated_part(poll, {-0.1, -0.14, -0.12}, {0.2, 0.24, 0.6}, head_turn);
		for (const double side : {-1.0, 1.0})
			placed |= articulated_part(poll, {side * 0.06 - 0.025, 0.08, -0.1},
					{0.05, 0.13, 0.05}, head_turn);
		placed |= articulated_part({0, 1.42, -0.8}, {-0.06, -0.78, -0.06},
				{0.12, 0.8, 0.12}, turn_quat({22, 0, 0}));
		// Sitting rider, legs astride the barrel and one arm raised to command.
		const Point3 rider_head{0, seat + 0.56, -0.05};
		placed |= articulated_part(rider_head, {-0.105, 0, -0.11}, {0.21, 0.25, 0.23},
				turn_quat({-4, 0, 0}));
		for (const double side : {-1.0, 1.0}) {
			const Point3 hip{0.16 * side, seat + 0.1, -0.05};
			const LimbPose leg =
					limb({-55, 0, static_cast<float>(22 * side)}, {65, 0, 0});
			const auto ankle = std::get<2>(articulated_limb(
					hip, leg, Point3{0.45, 0.15, 0.16}, Point3{0.43, 0.12, 0.13}));
			placed |= articulated_part(
					ankle, {-0.055, -0.05, -0.07}, {0.11, 0.08, 0.26}, NO_TURN);
		}
		const bool commanding = ((roll >> 17) & 1u) == 0;
		const LimbPose right = commanding ? limb({-125, 0, -12}, {-10, 0, 0})
										  : limb({-70, 0, -10}, {-15, 0, 0});
		const LimbPose left = limb({-25, 0, 6}, {-55, -15, 0});
		Point3 grip{};
		Quat grip_rotation = NO_TURN;
		for (const auto &[side, arm] : {std::pair{-1.0, right}, std::pair{1.0, left}}) {
			const Point3 shoulder{RIDER_SHOULDER_X * side, seat + 0.52, -0.05};
			const auto arm_end =
					articulated_limb(shoulder, arm, RIDER_UPPER_ARM, RIDER_FOREARM);
			if (side < 0) {
				grip_rotation = std::get<0>(arm_end);
				grip = add(std::get<2>(arm_end), rotate(grip_rotation, {0, 0.05, 0}));
			}
		}
		if (commanding) {
			placed |= articulated_part(
					grip, {-0.018, -0.84, -0.04}, {0.036, 0.78, 0.08}, grip_rotation);
			placed |= articulated_part(
					grip, {-0.03, -0.08, -0.12}, {0.06, 0.035, 0.24}, grip_rotation);
		} else {
			placed |= articulated_part(
					grip, {-0.025, -0.3, -0.025}, {0.05, 0.36, 0.05}, grip_rotation);
		}
		return placed;
	}
	if (sculpture) {
		bool placed = false;
		if (horse) {
			const bool pawing = ((roll >> 16) & 1u) == 0;
			placed |= articulated_part(
					{0, 0, 0}, {-0.23, 0.95, -0.8}, {0.46, 0.56, 1.55}, NO_TURN);
			const LimbPose raised =
					pawing ? limb({-42, 0, 0}, {80, 0, 0}) : STRAIGHT_LIMB;
			constexpr Point3 horse_thigh{0.52, 0.14, 0.18};
			constexpr Point3 horse_shin{0.5, 0.1, 0.11};
			for (const auto &[pivot, leg] :
					{std::pair{Point3{-0.13, 1.05, 0.6}, STRAIGHT_LIMB},
							std::pair{Point3{0.13, 1.05, 0.6}, raised},
							std::pair{Point3{-0.13, 1.05, -0.62},
									limb({-5, 0, 0}, {0, 0, 0})},
							std::pair{Point3{0.13, 1.05, -0.62},
									limb({8, 0, 0}, {0, 0, 0})}})
				articulated_limb(pivot, leg, horse_thigh, horse_shin);
			const auto neck = turn_quat({32, 0, 0});
			const Point3 withers{0, 1.32, 0.6};
			placed |= articulated_part(
					withers, {-0.12, 0, -0.17}, {0.24, 0.72, 0.32}, neck);
			const auto poll = add(withers, rotate(neck, {0, 0.66, 0}));
			const auto head_turn = turn_quat({38, 0, 0});
			placed |= articulated_part(
					poll, {-0.1, -0.14, -0.12}, {0.2, 0.24, 0.6}, head_turn);
			for (const double side : {-1.0, 1.0})
				placed |= articulated_part(poll, {side * 0.06 - 0.025, 0.08, -0.1},
						{0.05, 0.13, 0.05}, head_turn);
			placed |= articulated_part({0, 1.42, -0.8}, {-0.06, -0.78, -0.06},
					{0.12, 0.8, 0.12}, turn_quat({22, 0, 0}));
			return placed;
		}
		if (((roll >> 20) & 1u) == 0) {
			for (int k = 0; k < 7; ++k)
				placed |= add_box({0, 0.06 + double(k) * 0.4, 0}, {-0.4, 0, -0.4},
						{0.8, 0.34, 0.8}, turn_quat({0, float(k) * 13.0f, 0}), 1.0);
		} else {
			double y = 0.0;
			const std::array<std::pair<Point3, Turn>, 4> stones{{
					{{1.1, 0.55, 0.8}, {0, 0, 3}},
					{{0.85, 0.5, 0.65}, {0, 37, -4}},
					{{0.66, 0.45, 0.52}, {3, -21, 0}},
					{{0.46, 0.4, 0.4}, {0, 52, 5}},
			}};
			for (const auto &[size, turn] : stones) {
				placed |= add_box({0, y, 0}, {-size[0] / 2, 0, -size[2] / 2}, size,
						turn_quat(turn), 1.0);
				y += size[1] - 0.04;
			}
		}
		return placed;
	}
	constexpr Point3 HIP{0.09, 0.94, 0.0};
	constexpr Point3 SHOULDER{0.22, 1.40, 0.0};
	constexpr double NECK = 1.44;
	constexpr Point3 THIGH{0.45, 0.15, 0.16}, SHIN{0.43, 0.12, 0.13};
	constexpr Point3 UPPER_ARM{0.29, 0.105, 0.105}, FOREARM{0.31, 0.095, 0.095};
	const double scale = figure_scale;
	const auto part = [&](Point3 pivot, Point3 offset, Point3 size, Quat q) {
		return add_box(pivot, offset, size, q, scale);
	};
	const auto segment = [&](Point3 joint, Point3 measures, Quat q) {
		const double lap = measures[1] * 0.4;
		return part(joint, {-measures[1] / 2, -measures[0], -measures[2] / 2},
				{measures[1], measures[0] + lap, measures[2]}, q);
	};
	const auto draw_limb = [&](Point3 joint, LimbPose pose, Point3 upper_size,
								   Point3 lower_size, bool visible) {
		const auto upper = turn_quat(pose.upper);
		const auto lower = multiply(upper, turn_quat(pose.lower));
		const auto mid = add(joint, rotate(upper, {0, -upper_size[0], 0}));
		const auto end = add(mid, rotate(lower, {0, -lower_size[0], 0}));
		if (visible) {
			segment(joint, upper_size, upper);
			segment(mid, lower_size, lower);
		}
		return std::tuple{lower, mid, end};
	};
	const auto pose_index =
			pose_override >= 0
					? static_cast<std::size_t>(pose_override)
					: static_cast<std::size_t>((roll >> 8) % FIGURE_POSES.size());
	const auto &pose = FIGURE_POSES[pose_index % FIGURE_POSES.size()];
	bool placed = false;
	for (const auto &[side, leg] :
			{std::pair{-1.0, pose.right_leg}, std::pair{1.0, pose.left_leg}}) {
		const Point3 hip{HIP[0] * side, HIP[1], HIP[2]};
		const auto ankle = std::get<2>(draw_limb(hip, leg, THIGH, SHIN, !pose.robe));
		const auto foot_turn = turn_quat({0, leg.upper[1], 0});
		const Point3 foot{ankle[0], std::max(0.0, ankle[1] - 0.06), ankle[2]};
		placed |= part(foot, {-0.055, 0, -0.07}, {0.11, 0.08, 0.27}, foot_turn);
	}
	if (pose.robe)
		placed |= part({0, 0, 0}, {-0.19, 0, -0.135}, {0.38, 1.46, 0.27}, NO_TURN);
	else
		placed |= part({0, 0, 0}, {-0.19, 0.5, -0.125}, {0.38, 0.96, 0.25}, NO_TURN);
	const auto head_turn = turn_quat(pose.head);
	placed |= part({0, NECK - 0.02, 0}, {-0.055, 0, -0.055}, {0.11, 0.12, 0.11}, NO_TURN);
	placed |=
			part({0, NECK + 0.08, 0}, {-0.105, 0, -0.11}, {0.21, 0.25, 0.23}, head_turn);
	placed |= part(
			{0, NECK + 0.08, 0}, {-0.02, 0.06, 0.105}, {0.04, 0.07, 0.05}, head_turn);
	std::array<Point3, 2> hands{};
	Quat right_forearm = NO_TURN;
	for (std::size_t i = 0; i < 2; ++i) {
		const double side = i == 0 ? -1.0 : 1.0;
		const auto arm = i == 0 ? pose.right_arm : pose.left_arm;
		const Point3 shoulder{SHOULDER[0] * side, SHOULDER[1], 0};
		const auto limb_end = draw_limb(shoulder, arm, UPPER_ARM, FOREARM, true);
		const auto lower = std::get<0>(limb_end);
		const auto end = std::get<2>(limb_end);
		hands[i] = add(end, rotate(lower, {0, 0.05, 0}));
		if (i == 0)
			right_forearm = lower;
	}
	const Point3 hand = hands[0];
	if (pose.holds == FigureHold::Staff) {
		placed |= part(hand, {-0.025, -hand[1], -0.025}, {0.05, 2.05, 0.05}, NO_TURN);
	} else if (pose.holds == FigureHold::Sword) {
		placed |= part(hand, {-0.018, -0.84, -0.04}, {0.036, 0.78, 0.08}, right_forearm);
		placed |= part(hand, {-0.03, -0.08, -0.12}, {0.06, 0.035, 0.24}, right_forearm);
	} else if (pose.holds == FigureHold::Book) {
		Point3 between{};
		for (std::size_t i = 0; i < 3; ++i)
			between[i] = (hands[0][i] + hands[1][i]) / 2;
		placed |= part(
				between, {-0.14, 0, -0.1}, {0.28, 0.035, 0.2}, turn_quat({-35, 0, 0}));
	} else if (pose.holds == FigureHold::Scroll) {
		placed |= part(hand, {-0.16, -0.035, -0.035}, {0.32, 0.07, 0.07}, right_forearm);
	}
	return placed;
}

std::optional<double> mapped_height(const tags_t &tags)
{
	auto value = tags.get("height");
	if (value.empty())
		return std::nullopt;
	const auto first = value.find_first_not_of(" \t\r\n");
	if (first == std::string::npos)
		return std::nullopt;
	const auto last = value.find_last_not_of(" \t\r\n");
	value = value.substr(first, last - first + 1);
	if (!value.empty() && value.back() == 'm') {
		value.pop_back();
		const auto trimmed = value.find_last_not_of(" \t\r\n");
		if (trimmed == std::string::npos)
			return std::nullopt;
		value.resize(trimmed + 1);
	}
	return strict_parse::f64(value);
}

bool names_a_horse(const tags_t &tags)
{
	static constexpr std::string_view keys[] = {
			"name", "name:en", "subject", "artwork_subject", "description"};
	static constexpr std::string_view words[] = {
			"horse", "pferd", "cheval", "caballo", "cavallo", "paard", "koń"};
	for (const auto key : keys) {
		auto value = tags.get(std::string(key));
		std::transform(value.begin(), value.end(), value.begin(),
				[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		// Rust's Unicode lowercase maps Polish capital Ń (C5 83) to ń (C5 84).
		// Keep the non-ASCII horse keyword case-insensitive without depending on a
		// process locale, which is often the C locale during map generation.
		for (std::size_t pos = 0;
				(pos = value.find("\xC5\x83", pos)) != std::string::npos; pos += 2)
			value.replace(pos, 2, "\xC5\x84");
		for (const auto word : words)
			if (value.find(word) != std::string::npos)
				return true;
	}
	return false;
}

int artwork_lowest_ground(const WorldEditor &editor, int x, int z, int radius)
{
	int ground = std::numeric_limits<int>::max();
	for (int dx = -radius; dx <= radius; ++dx)
		for (int dz = -radius; dz <= radius; ++dz)
			ground = std::min(ground, editor.get_ground_level(x + dx, z + dz));
	return ground;
}

bool artwork_is_plant(const WorldEditor &editor, const Block &block)
{
	const auto name = editor.block_name(block);
	if (!name)
		return false;
	const auto separator = name->find(':');
	const std::string_view short_name =
			separator == std::string::npos
					? std::string_view(*name)
					: std::string_view(*name).substr(separator + 1);
	for (const std::string_view part : {"leaves", "_log", "_wood", "_stem", "_hyphae",
				 "grass", "fern", "bush", "vine"})
		if (short_name.find(part) != std::string_view::npos)
			return true;
	const auto has_suffix = [&](std::string_view suffix) {
		return short_name.size() >= suffix.size() &&
			   short_name.compare(
					   short_name.size() - suffix.size(), suffix.size(), suffix) == 0;
	};
	return short_name == "dandelion" || short_name == "poppy" || short_name == "allium" ||
		   short_name == "cornflower" || short_name == "oxeye_daisy" ||
		   short_name == "azure_bluet" || has_suffix("_tulip") || has_suffix("_sapling");
}

bool artwork_can_place(const WorldEditor &editor, int x, int y, int z)
{
	const auto existing = editor.get_block_absolute(x, y, z);
	return !existing || artwork_is_plant(editor, *existing);
}

void artwork_put(WorldEditor &editor, const Block &block, int x, int y, int z)
{
	if (artwork_can_place(editor, x, y, z))
		editor.set_block_absolute(block, x, y, z, std::nullopt, std::nullopt);
}

void artwork_put(
		WorldEditor &editor, const BlockWithProperties &block, int x, int y, int z)
{
	if (artwork_can_place(editor, x, y, z))
		editor.set_block_with_properties_absolute(
				block, x, y, z, std::nullopt, std::vector<Block>{});
}

void artwork_stepped_square(WorldEditor &editor, int x, int z, int radius, int y,
		const Block &fill, const Block &stairs)
{
	for (int dx = -radius; dx <= radius; ++dx) {
		for (int dz = -radius; dz <= radius; ++dz) {
			const bool edge_x = std::abs(dx) == radius;
			const bool edge_z = std::abs(dz) == radius;
			if (!edge_x && !edge_z) {
				artwork_put(editor, fill, x + dx, y, z + dz);
				continue;
			}

			StairFacing facing;
			StairShape shape = StairShape::Straight;
			if (edge_x && !edge_z) {
				facing = dx < 0 ? StairFacing::East : StairFacing::West;
			} else {
				facing = dz < 0 ? StairFacing::South : StairFacing::North;
				if (edge_x)
					shape = dx * dz > 0 ? StairShape::OuterRight : StairShape::OuterLeft;
			}
			artwork_put(editor, create_stair_with_properties(stairs, facing, shape),
					x + dx, y, z + dz);
		}
	}
}

int artwork_pedestal(WorldEditor &editor, int x, int z, std::uint64_t roll, int radius)
{
	const Block *die = &POLISHED_ANDESITE;
	const Block *stairs = &POLISHED_ANDESITE_STAIRS;
	const Block *cap = &POLISHED_ANDESITE_SLAB;
	switch ((roll >> 12) & 3u) {
	case 0:
		die = &POLISHED_GRANITE;
		stairs = &POLISHED_GRANITE_STAIRS;
		cap = &POLISHED_GRANITE_SLAB;
		break;
	case 1:
		die = &POLISHED_DIORITE;
		stairs = &POLISHED_DIORITE_STAIRS;
		cap = &POLISHED_DIORITE_SLAB;
		break;
	default:
		break;
	}
	const int ground = artwork_lowest_ground(editor, x, z, radius);
	artwork_stepped_square(editor, x, z, radius, ground + 1, *die, *stairs);
	for (int dx = 1 - radius; dx < radius; ++dx) {
		for (int dz = 1 - radius; dz < radius; ++dz) {
			artwork_put(editor, *die, x + dx, ground + 2, z + dz);
			artwork_put(editor, *cap, x + dx, ground + 3, z + dz);
		}
	}
	return ground + 3;
}

void generate_column(WorldEditor &editor, const ProcessedNode &node, int requested_height,
		std::uint64_t roll, bool figure)
{
	const int x = node.x, z = node.z;
	int ground = std::numeric_limits<int>::max();
	for (int dx = -2; dx <= 2; ++dx)
		for (int dz = -2; dz <= 2; ++dz)
			ground = std::min(ground, editor.get_ground_level(x + dx, z + dz));
	const auto put = [&](const Block &block, int px, int py, int pz) {
		artwork_put(editor, block, px, py, pz);
	};
	artwork_stepped_square(editor, x, z, 2, ground + 1, STONE_BRICKS, STONE_BRICK_STAIRS);
	for (int y = ground + 2; y <= ground + 3; ++y)
		for (int dx = -1; dx <= 1; ++dx)
			for (int dz = -1; dz <= 1; ++dz)
				put(dx != 0 && dz != 0 ? CHISELED_STONE_BRICKS : POLISHED_ANDESITE,
						x + dx, y, z + dz);
	artwork_stepped_square(
			editor, x, z, 1, ground + 4, POLISHED_ANDESITE, POLISHED_ANDESITE_STAIRS);
	const int height = std::clamp(requested_height, 5, 30);
	const bool crowned = figure && editor.get_generation_format() == 0;
	const int capital = ground + (crowned ? std::max(height - 3, 5) : height);
	for (int y = ground + 5; y < capital; ++y)
		put(QUARTZ_PILLAR, x, y, z);
	put(CHISELED_QUARTZ_BLOCK, x, capital, z);
	if (crowned) {
		const bool explicit_material = node.tags.find("material") != node.tags.end();
		const Block crown_material = !explicit_material && roll % 2 != 0
											 ? GOLD_BLOCK
											 : figure_material(node.tags, roll >> 1);
		const int pose = std::array<int, 3>{1, 3, 4}[(roll >> 8) % 3];
		add_artwork_figure_displays(editor, node, figure_material_name(crown_material), 0,
				false, false, false, false, artwork_yaw(node.tags, roll), roll, 0.0, 1.7,
				pose, capital + 1);
	} else {
		put(SMOOTH_QUARTZ_SLAB, x, capital + 1, z);
	}
}
}

bool is_artwork_figure(const tags_t &tags)
{
	return artwork_form(tags).has_value();
}

void generate_artwork_figure(WorldEditor &editor, const ProcessedNode &node)
{
	const auto form = artwork_form(node.tags);
	if (!form)
		return;
	const int x = node.x, z = node.z;
	auto rng = element_rng(node.id);
	const auto roll = rng();
	const Block material = figure_material(node.tags, roll);
	if (*form == "statue" && node.tags.get("statue") != "equestrian") {
		if (const auto height = mapped_height(node.tags); height && *height >= 6.0) {
			generate_column(
					editor, node, static_cast<int>(std::lround(*height)), roll, true);
			return;
		}
	}
	if (*form == "bust") {
		const int ground = editor.get_ground_level(x, z);
		artwork_put(editor, POLISHED_ANDESITE, x, ground + 1, z);
		artwork_put(editor, ANDESITE_WALL, x, ground + 2, z);
		artwork_put(editor, POLISHED_ANDESITE_SLAB, x, ground + 3, z);
		if (editor.get_generation_format() == 0) {
			if (add_artwork_figure_displays(editor, node, figure_material_name(material),
						0, true, false, false, false, artwork_yaw(node.tags, roll), roll,
						0.5, 1.6, -1, ground + 3))
				return;
		}
		artwork_put(editor, material, x, ground + 3, z);
		return;
	}
	if (*form == "sculpture") {
		const bool horse = names_a_horse(node.tags);
		const int radius = horse ? 1 : 0;
		const Block sculpture_material = std::array<Block, 4>{WAXED_OXIDIZED_COPPER,
				IRON_BLOCK, POLISHED_BLACKSTONE, WAXED_COPPER_BLOCK}[(roll >> 16) % 4];
		const int ground = artwork_lowest_ground(editor, x, z, radius);
		for (int dx = -radius; dx <= radius; ++dx)
			for (int dz = -radius; dz <= radius; ++dz)
				artwork_put(editor, POLISHED_ANDESITE, x + dx, ground + 1, z + dz);
		if (editor.get_generation_format() == 0 &&
				add_artwork_figure_displays(editor, node,
						figure_material_name(sculpture_material), 0, false, true, horse,
						false, artwork_yaw(node.tags, roll), roll, 0.0, 1.0, -1,
						ground + 2))
			return;
		for (int y = 2; y <= 4; ++y)
			artwork_put(editor, material, x, ground + y, z);
		return;
	}
	const bool equestrian = node.tags.get("statue") == "equestrian";
	const int ground = artwork_lowest_ground(editor, x, z, equestrian ? 2 : 1);
	const int pedestal_top = artwork_pedestal(editor, x, z, roll, equestrian ? 2 : 1);
	if (editor.get_generation_format() == 0 &&
			add_artwork_figure_displays(editor, node, figure_material_name(material), 0,
					false, false, false, equestrian, artwork_yaw(node.tags, roll), roll,
					0.5, 1.35, -1, pedestal_top))
		return;
	for (int y = 3; y <= 5; ++y)
		artwork_put(editor, material, x, ground + y, z);
}

void generate_artwork_figure(WorldEditor &editor, const ProcessedWay &way)
{
	if (way.nodes.empty() || !is_artwork_figure(way.tags))
		return;
	const auto &first = way.nodes.front();
	const bool closed = way.nodes.size() > 1 && way.nodes.back().x == first.x &&
						way.nodes.back().z == first.z;
	const std::size_t count = way.nodes.size() - static_cast<std::size_t>(closed);
	if (count == 0)
		return;
	std::int64_t x_sum = 0;
	std::int64_t z_sum = 0;
	for (std::size_t i = 0; i < count; ++i) {
		x_sum += way.nodes[i].x;
		z_sum += way.nodes[i].z;
	}
	ProcessedNode centre{};
	centre.id = way.id;
	centre.tags = way.tags;
	centre.x = static_cast<int>(std::llround(double(x_sum) / double(count)));
	centre.z = static_cast<int>(std::llround(double(z_sum) / double(count)));
	generate_artwork_figure(editor, centre);
}

void generate_historic(WorldEditor &editor, const ProcessedNode &node)
{
	// Match Rust parse::<i32>().unwrap_or(0): malformed tags default to zero.
	for (const char *key : {"layer", "level"}) {
		if (const auto it = node.tags.find(key);
				it != node.tags.end() && strict_parse::i32_or_zero(it->second) < 0)
			return;
	}

	auto it_historic = node.tags.find("historic");
	if (it_historic != node.tags.end()) {
		const std::string &historic_type = it_historic->second;
		if (historic_type == "memorial") {
			generate_memorial(editor, node);
		} else if (historic_type == "monument") {
			generate_monument(editor, node);
		} else if (historic_type == "wayside_cross") {
			generate_wayside_cross(editor, node);
		}
	}
}

void generate_memorial(WorldEditor &editor, const ProcessedNode &node)
{
	int x = node.x;
	int z = node.z;

	// Get memorial subtype
	auto it_memorial = node.tags.find("memorial");
	std::string memorial_type =
			(it_memorial != node.tags.end()) ? it_memorial->second : "yes";

	if (memorial_type == "plaque") {
		// Prefer the Rust wall-mounted plaque; retain the small stand as fallback.
		if (signage::generate_plaque(editor, node))
			return;
		editor.set_block(STONE_BRICKS, x, 1, z, std::nullopt, std::nullopt);
		editor.set_block(STONE_BRICK_SLAB, x, 2, z, std::nullopt, std::nullopt);
		if (const auto key = signage::plaque_key(node.tags)) {
			const int y = editor.get_ground_level(x, z) + 1;
			for (const std::int8_t facing :
					{std::int8_t(2), std::int8_t(3), std::int8_t(4), std::int8_t(5)})
				editor.place_decal(x, y, z, facing, *key);
		}
	} else if (memorial_type == "statue" || memorial_type == "sculpture" ||
			   memorial_type == "bust") {
		// Rust only places a figure when statues::form accepts its tags (ground-level,
		// uncovered, and an actual statue/bust/sculpture). Keep this entry point aligned
		// for callers that dispatch through historic instead of the artwork pass.
		if (is_artwork_figure(node.tags))
			generate_artwork_figure(editor, node);
	} else if (memorial_type == "stone" || memorial_type == "stolperstein") {
		// Simple memorial stone embedded in ground
		Block stone_block = (memorial_type == "stolperstein") ? GOLD_BLOCK : STONE;
		editor.set_block(stone_block, x, 0, z, std::nullopt, std::nullopt);
	} else if (memorial_type == "cross" || memorial_type == "war_memorial") {
		// Memorial cross
		generate_cross(editor, x, z, 5);
	} else if (memorial_type == "obelisk") {
		// Tall pointed pillar with fixed height
		// Base layer at Y=1
		for (int dx = -1; dx <= 1; ++dx) {
			for (int dz = -1; dz <= 1; ++dz) {
				editor.set_block(
						STONE_BRICKS, x + dx, 1, z + dz, std::nullopt, std::nullopt);
			}
		}

		// Second base layer at Y=2
		for (int dx = -1; dx <= 1; ++dx) {
			for (int dz = -1; dz <= 1; ++dz) {
				editor.set_block(
						STONE_BRICKS, x + dx, 2, z + dz, std::nullopt, std::nullopt);
			}
		}
		// Stone brick slabs on the 4 corners at Y=3 (on top of corner blocks)
		editor.set_block(STONE_BRICK_SLAB, x - 1, 3, z - 1, std::nullopt, std::nullopt);
		editor.set_block(STONE_BRICK_SLAB, x + 1, 3, z - 1, std::nullopt, std::nullopt);
		editor.set_block(STONE_BRICK_SLAB, x - 1, 3, z + 1, std::nullopt, std::nullopt);
		editor.set_block(STONE_BRICK_SLAB, x + 1, 3, z + 1, std::nullopt, std::nullopt);

		// Main shaft, fixed height of 4 blocks (Y=3 to Y=6)
		for (int y = 3; y <= 6; ++y) {
			editor.set_block(SMOOTH_QUARTZ, x, y, z, std::nullopt, std::nullopt);
		}

		editor.set_block(STONE_BRICK_SLAB, x, 7, z, std::nullopt, std::nullopt);
	} else if (memorial_type == "stele") {
		// Upright stone slab
		// Base
		editor.set_block(STONE_BRICKS, x, 1, z, std::nullopt, std::nullopt);

		// Upright slab (using wall blocks for thin appearance)
		for (int y = 2; y <= 4; ++y) {
			editor.set_block(STONE_BRICK_WALL, x, y, z, std::nullopt, std::nullopt);
		}
		editor.set_block(STONE_BRICK_SLAB, x, 5, z, std::nullopt, std::nullopt);
	} else {
		// Default: simple stone pillar monument
		editor.set_block(STONE_BRICKS, x, 1, z, std::nullopt, std::nullopt);
		editor.set_block(STONE_BRICKS, x, 2, z, std::nullopt, std::nullopt);
		editor.set_block(CHISELED_STONE_BRICKS, x, 3, z, std::nullopt, std::nullopt);
		editor.set_block(STONE_BRICK_SLAB, x, 4, z, std::nullopt, std::nullopt);
	}
}

void generate_monument(WorldEditor &editor, const ProcessedNode &node)
{
	const int height = static_cast<int>(
			std::lround(std::clamp(mapped_height(node.tags).value_or(10.0), 5.0, 20.0)));
	const auto memorial = node.tags.get("memorial");
	const bool figure = memorial == "statue" || memorial == "column";
	auto rng = element_rng(node.id);
	generate_column(editor, node, height, rng(), figure);
}

void generate_wayside_cross(WorldEditor &editor, const ProcessedNode &node)
{
	int x = node.x;
	int z = node.z;

	// Simple roadside cross
	generate_cross(editor, x, z, 4);
}

void generate_cross(WorldEditor &editor, int x, int z, int height)
{
	// Base
	editor.set_block(STONE_BRICKS, x, 1, z, std::nullopt, std::nullopt);

	// Vertical beam
	for (int y = 2; y <= height; ++y) {
		editor.set_block(STONE_BRICK_WALL, x, y, z, std::nullopt, std::nullopt);
	}

	// Horizontal beam (cross arm) at approximately 2/3 height, but at least 2 and at most height-1
	int arm_y = std::max(2, std::min(height - 1, (height * 2 + 2) / 3));

	// Only place horizontal arms if height allows for them (height >= 3)
	if (height >= 3) {
		editor.set_block(STONE_BRICK_WALL, x - 1, arm_y, z, std::nullopt, std::nullopt);
		editor.set_block(STONE_BRICK_WALL, x + 1, arm_y, z, std::nullopt, std::nullopt);
	}
}

void generate_pyramid(WorldEditor &editor, const ProcessedWay &element, const Args &args,
		const FloodFillCache &flood_fill_cache)
{
	if (element.nodes.size() < 3) {
		return;
	}

	// Get the footprint via flood fill
	const auto footprint = flood_fill_cache.get_or_compute(element, args.timeout);

	if (footprint->empty()) {
		return;
	}

	// Determine base Y from terrain or ground level
	// Use the MINIMUM ground level so the pyramid sits on the lowest point
	// and doesn't float in areas with elevation differences
	int base_y = args.ground_level;
	if (args.terrain_enabled()) {
		int min_ground_level = args.ground_level;
		for (const auto &p : *footprint) {
			int ground_level = editor.get_ground_level(p.first, p.second);
			if (ground_level < min_ground_level) {
				min_ground_level = ground_level;
			}
		}
		base_y = min_ground_level;
	}

	// Bounding box of the footprint
	int min_x = (*footprint)[0].first;
	int max_x = (*footprint)[0].first;
	int min_z = (*footprint)[0].second;
	int max_z = (*footprint)[0].second;

	for (const auto &p : *footprint) {
		if (p.first < min_x)
			min_x = p.first;
		if (p.first > max_x)
			max_x = p.first;
		if (p.second < min_z)
			min_z = p.second;
		if (p.second > max_z)
			max_z = p.second;
	}

	double center_x = (min_x + max_x) / 2.0;
	double center_z = (min_z + max_z) / 2.0;

	// The pyramid height is half the shorter side of the bounding box (classic proportions)
	double width = max_x - min_x + 1;
	double length = max_z - min_z + 1;
	double half_base = std::min(width, length) / 2.0;
	// Height = half the shorter side (classic pyramid proportions).
	// Footprint is already in scaled Minecraft coordinates, so no extra scale factor needed.
	int pyramid_height = std::max(3, static_cast<int>(half_base));

	// Build the pyramid layer by layer.
	// For each layer, only place blocks whose Chebyshev distance from the
	// footprint centre is within the shrinking radius AND that were in the
	// original footprint.
	std::optional<int> last_placed_layer;
	for (int layer = 0; layer < pyramid_height; ++layer) {
		// The allowed radius shrinks linearly from half_base at layer 0 to 0
		double radius = half_base * (1.0 - static_cast<double>(layer) /
													static_cast<double>(pyramid_height));
		if (radius < 0.0) {
			break;
		}

		int y = base_y + 1 + layer;
		bool placed = false;

		for (const auto &p : *footprint) {
			int px = p.first;
			int pz = p.second;
			double dx = std::abs(px - center_x);
			double dz = std::abs(pz - center_z);

			// Use Chebyshev distance (max of dx, dz) for a square-footprint pyramid
			if (dx <= radius && dz <= radius) {
				// Allow overwriting common terrain blocks so the pyramid is
				// solid even when it intersects higher ground.
				std::vector<Block> whitelist = {GRASS_BLOCK, DIRT, STONE, SAND, GRAVEL,
						COARSE_DIRT, PODZOL, DIRT_PATH, SANDSTONE};
				editor.set_block_absolute(SANDSTONE, px, y, pz,
						std::optional<std::vector<Block>>(whitelist), std::nullopt);
				placed = true;
			}
		}

		if (placed) {
			last_placed_layer = y;
		} else {
			break; // Nothing placed, we've reached the apex
		}
	}

	// Cap with smooth sandstone one block above the last placed layer
	if (last_placed_layer.has_value()) {
		std::vector<Block> whitelist = {GRASS_BLOCK, DIRT, STONE, SAND, GRAVEL,
				COARSE_DIRT, PODZOL, DIRT_PATH, SANDSTONE};
		editor.set_block_absolute(SMOOTH_SANDSTONE,
				static_cast<int>(std::round(center_x)), last_placed_layer.value() + 1,
				static_cast<int>(std::round(center_z)),
				std::optional<std::vector<Block>>(whitelist), std::nullopt);
	}
}

}
}
