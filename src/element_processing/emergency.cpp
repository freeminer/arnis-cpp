#include "emergency.h"
#include "structure_kit.h"
#include "../strict_parse.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace arnis
{
namespace emergency
{

namespace
{
constexpr int ROAD_REACH = 8;
constexpr double MIN_SCALE = 0.5;
constexpr const char *IRON = "minecraft:gray_concrete";
constexpr const char *CAP = "minecraft:light_gray_concrete";

std::optional<std::pair<int, int>> nearest(
		int x, int z, int reach, const std::function<bool(int, int)> &wanted)
{
	std::optional<std::pair<int, int>> best;
	int best_distance = std::numeric_limits<int>::max();
	for (int dx = -reach; dx <= reach; ++dx)
		for (int dz = -reach; dz <= reach; ++dz) {
			const int distance = dx * dx + dz * dz;
			if (distance == 0 || distance > reach * reach || !wanted(x + dx, z + dz))
				continue;
			if (distance < best_distance) {
				best_distance = distance;
				best = std::pair{x + dx, z + dz};
			}
		}
	return best;
}

std::optional<std::pair<double, double>> road_facing(
		const RoadMaskBitmap &roads, int x, int z)
{
	const auto road = nearest(
			x, z, ROAD_REACH, [&](int cx, int cz) { return roads.contains(cx, cz); });
	if (!road)
		return std::nullopt;
	const double dx = road->first - x;
	const double dz = road->second - z;
	const double length = std::hypot(dx, dz);
	if (length <= 0.0)
		return std::nullopt;
	return std::pair{dx / length, dz / length};
}

std::optional<Block> body_block(const std::string &text)
{
	std::string colour = text;
	const auto first = colour.find_first_not_of(" \t\r\n");
	if (first == std::string::npos)
		return std::nullopt;
	const auto last = colour.find_last_not_of(" \t\r\n");
	colour = colour.substr(first, last - first + 1);
	std::transform(colour.begin(), colour.end(), colour.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	if (colour == "red")
		return RED_CONCRETE;
	if (colour == "yellow" || colour == "gold")
		return YELLOW_CONCRETE;
	if (colour == "orange")
		return ORANGE_CONCRETE;
	if (colour == "green")
		return GREEN_CONCRETE;
	if (colour == "blue")
		return BLUE_CONCRETE;
	if (colour == "white")
		return WHITE_CONCRETE;
	if (colour == "silver" || colour == "grey" || colour == "gray")
		return LIGHT_GRAY_CONCRETE;
	if (colour == "black")
		return BLACK_CONCRETE;
	return structure_kit::concrete_for_colour(colour);
}

const char *minecraft_concrete_name(const Block &block)
{
	if (block == WHITE_CONCRETE)
		return "minecraft:white_concrete";
	if (block == ORANGE_CONCRETE)
		return "minecraft:orange_concrete";
	if (block == MAGENTA_CONCRETE)
		return "minecraft:magenta_concrete";
	if (block == LIGHT_BLUE_CONCRETE)
		return "minecraft:light_blue_concrete";
	if (block == YELLOW_CONCRETE)
		return "minecraft:yellow_concrete";
	if (block == LIME_CONCRETE)
		return "minecraft:lime_concrete";
	if (block == PINK_CONCRETE)
		return "minecraft:pink_concrete";
	if (block == GRAY_CONCRETE)
		return "minecraft:gray_concrete";
	if (block == LIGHT_GRAY_CONCRETE)
		return "minecraft:light_gray_concrete";
	if (block == CYAN_CONCRETE)
		return "minecraft:cyan_concrete";
	if (block == PURPLE_CONCRETE)
		return "minecraft:purple_concrete";
	if (block == BLUE_CONCRETE)
		return "minecraft:blue_concrete";
	if (block == BROWN_CONCRETE)
		return "minecraft:brown_concrete";
	if (block == GREEN_CONCRETE)
		return "minecraft:green_concrete";
	if (block == RED_CONCRETE)
		return "minecraft:red_concrete";
	return "minecraft:black_concrete";
}

class Hydrant
{
	WorldEditor &editor;
	std::array<double, 3> centre;
	std::pair<double, double> facing;
	double scale;
	std::int64_t seed;

	void part(std::array<double, 3> at, std::array<double, 3> size, const char *block)
	{
		const auto [fx, fz] = facing;
		const double sx = -fz, sz = fx;
		const std::array<double, 3> pos{centre[0] + (fx * at[0] + sx * at[2]) * scale,
				centre[1] + at[1] * scale, centre[2] + (fz * at[0] + sz * at[2]) * scale};
		for (auto &dimension : size)
			dimension *= scale;
		const auto floats = [](std::initializer_list<double> values) {
			auto result = nlohmann::json::array();
			for (const auto value : values)
				result.push_back(value);
			return result;
		};
		const float yaw =
				static_cast<float>(std::atan2(-fx, fz) * 180.0 / std::numbers::pi);
		nlohmann::json extra{{"block_state", {{"Name", block}}},
				{"transformation",
						{{"left_rotation", floats({0, 0, 0, 1})},
								{"right_rotation", floats({0, 0, 0, 1})},
								{"translation", floats({-size[0] / 2, -size[1] / 2,
														-size[2] / 2})},
								{"scale", floats({size[0], size[1], size[2]})}}},
				{"Rotation", floats({yaw, 0})}};
		seed = std::bit_cast<std::int64_t>(
				std::bit_cast<std::uint64_t>(seed) + std::uint64_t{1});
		(void)editor.add_block_display(pos[0], pos[1], pos[2], seed, std::move(extra));
	}

public:
	Hydrant(WorldEditor &world, std::array<double, 3> origin,
			std::pair<double, double> direction, double scale_, std::int64_t seed_) :
			editor(world), centre(origin), facing(direction), scale(scale_), seed(seed_)
	{
	}

	void build(const char *body)
	{
		part({0.0, 0.03, 0.0}, {0.32, 0.06, 0.32}, IRON);
		part({0.0, 0.35, 0.0}, {0.22, 0.58, 0.22}, body);
		part({0.0, 0.665, 0.0}, {0.26, 0.05, 0.26}, body);
		part({0.0, 0.75, 0.0}, {0.19, 0.12, 0.19}, body);
		part({0.0, 0.835, 0.0}, {0.13, 0.05, 0.13}, body);
		part({0.0, 0.885, 0.0}, {0.06, 0.05, 0.06}, IRON);
		for (const double side : {-1.0, 1.0}) {
			part({0.0, 0.48, side * 0.165}, {0.11, 0.09, 0.09}, body);
			part({0.0, 0.48, side * 0.235}, {0.03, 0.12, 0.12}, CAP);
		}
		part({0.155, 0.42, 0.0}, {0.13, 0.13, 0.09}, body);
		part({0.215, 0.42, 0.0}, {0.15, 0.15, 0.03}, CAP);
	}
};
} // namespace

void generate_emergency(
		WorldEditor &editor, const ProcessedNode &node, const RoadMaskBitmap &road_mask)
{
	// Match Rust parse::<i32>().unwrap_or(0): malformed tags default to zero.
	for (const char *key : {"layer", "level"}) {
		if (const auto it = node.tags.find(key);
				it != node.tags.end() && strict_parse::i32_or_zero(it->second) < 0)
			return;
	}

	auto it_emergency = node.tags.find("emergency");
	if (it_emergency != node.tags.end()) {
		if (it_emergency->second == "fire_hydrant") {
			generate_fire_hydrant(editor, node, road_mask);
		}
	}
}

void generate_fire_hydrant(
		WorldEditor &editor, const ProcessedNode &node, const RoadMaskBitmap &road_mask)
{
	int x = node.x;
	int z = node.z;

	// Get hydrant type - skip underground, wall, and pond types
	std::string hydrant_type = "pillar";
	auto it_type = node.tags.find("fire_hydrant:type");
	if (it_type != node.tags.end()) {
		hydrant_type = it_type->second;
	}

	// Skip non-visible hydrant types
	if (hydrant_type == "underground" || hydrant_type == "wall" ||
			hydrant_type == "pond") {
		return;
	}

	if (editor.format() == 0 && editor.scale() >= MIN_SCALE) {
		const auto &carriageway = editor.carriageway;
		if (carriageway && !carriageway->is_empty() && carriageway->contains(x, z)) {
			const auto beside = nearest(x, z, 3,
					[&](int cx, int cz) { return !carriageway->contains(cx, cz); });
			if (beside) {
				x = beside->first;
				z = beside->second;
			}
		}
		std::optional<std::pair<double, double>> facing;
		if (carriageway && !carriageway->is_empty())
			facing = road_facing(*carriageway, x, z);
		if (!facing)
			facing = road_facing(road_mask, x, z);
		if (!facing) {
			static constexpr std::array<std::pair<double, double>, 4> fallback{
					{{0.0, 1.0}, {1.0, 0.0}, {0.0, -1.0}, {-1.0, 0.0}}};
			facing = fallback[node.id % fallback.size()];
		}
		const auto colour = node.tags.get("colour");
		const Block body =
				colour.empty() ? RED_CONCRETE : body_block(colour).value_or(RED_CONCRETE);
		editor.set_block(BARRIER, x, 1, z, std::nullopt, std::nullopt);
		const int abs_y = editor.get_absolute_y(x, 1, z);
		const auto wrapped_seed = node.id * std::uint64_t{64};
		Hydrant hydrant(editor, {x + 0.5, static_cast<double>(abs_y), z + 0.5}, *facing,
				editor.scale(), std::bit_cast<std::int64_t>(wrapped_seed));
		hydrant.build(minecraft_concrete_name(body));
		return;
	}

	editor.set_block(REDSTONE_BLOCK, x, 1, z, std::nullopt, std::nullopt);
	const int abs_y = editor.get_absolute_y(x, 1, z);
	// Rust uses red banners with a simple flame-like pattern on each side for
	// non-Java backends.
	const std::vector<std::pair<std::string, std::string>> patterns{
			{"orange", "minecraft:triangle_top"}, {"yellow", "minecraft:triangle_bottom"},
			{"red", "minecraft:border"}};
	const std::array<std::tuple<int, int, const char *>, 4> faces{
			{{0, 1, "south"}, {0, -1, "north"}, {1, 0, "east"}, {-1, 0, "west"}}};
	for (const auto &[dx, dz, facing] : faces)
		editor.place_wall_banner(
				RED_WALL_BANNER, x + dx, abs_y, z + dz, facing, "red", patterns);
}

}
}
