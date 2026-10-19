#include "traffic_lights.h"

#include "../bresenham.h"
#include "../block_definitions.h"
#include "connected_blocks.h"
#include "fm_traffic_signals.h"
#include "highways.h"
#include "road_markings.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iterator>
#include <numbers>
#include <optional>
#include <tuple>
#include <unordered_set>

namespace arnis::highways
{
namespace
{
constexpr double DIR_REACH = 6.0;
constexpr int COVER = 6;
constexpr int MAST_Y = 7;
constexpr int MAST_HALF = 3;
constexpr double CAR_HEAD_Y = 3.9;
constexpr double BIKE_HEAD_Y = 3.0;
constexpr double PED_HEAD_Y = 2.3;
constexpr double BIKE_SCALE = 0.62;

using Kind = SignalIndex::Kind;

struct Arm
{
	double dx{}, dz{};
	int half{1};
	bool inbound{};
};

struct SignalNode
{
	int x{}, z{};
	bool crossing{};
	Kind kind{Kind::Car};
	std::vector<Arm> arms;
};

std::optional<std::pair<double, double>> arm_dir(
		const std::vector<ProcessedNode> &nodes, std::size_t index, int step)
{
	const auto &origin = nodes[index];
	std::ptrdiff_t j = static_cast<std::ptrdiff_t>(index) + step;
	std::optional<std::pair<double, double>> result;
	while (j >= 0 && static_cast<std::size_t>(j) < nodes.size()) {
		const auto &node = nodes[static_cast<std::size_t>(j)];
		const double dx = static_cast<double>(node.x) - origin.x;
		const double dz = static_cast<double>(node.z) - origin.z;
		const double length = std::hypot(dx, dz);
		if (length > 0.0) {
			result = std::pair{dx / length, dz / length};
			if (length >= DIR_REACH)
				break;
		}
		j += step;
	}
	return result;
}

bool signal_upstream_impl(const std::vector<ProcessedNode> &nodes, std::size_t index,
		int step, double reach)
{
	const auto &origin = nodes[index];
	std::ptrdiff_t j = static_cast<std::ptrdiff_t>(index) + step;
	while (j >= 0 && static_cast<std::size_t>(j) < nodes.size()) {
		const auto &node = nodes[static_cast<std::size_t>(j)];
		if (std::hypot(static_cast<double>(node.x) - origin.x,
					static_cast<double>(node.z) - origin.z) > reach)
			return false;
		if (node.id != origin.id && is_signal(node.tags))
			return true;
		j += step;
	}
	return false;
}

void add_arms(std::unordered_map<std::uint64_t, SignalNode> &nodes,
		const ProcessedWay &way, int half, int oneway, Kind kind, double scale,
		const std::function<bool(const ProcessedNode &)> &wanted)
{
	const double upstream = std::max(UPSTREAM_M * scale, 6.0);
	for (std::size_t i = 0; i < way.nodes.size(); ++i) {
		const auto &node = way.nodes[i];
		if (!is_signal(node.tags) || !wanted(node))
			continue;
		const auto direction_it = node.tags.find("traffic_signals:direction");
		const std::string direction =
				direction_it == node.tags.end() ? "" : direction_it->second;
		const bool forward_ok = oneway >= 0 && direction != "backward";
		const bool backward_ok = oneway <= 0 && direction != "forward";
		const std::array<std::pair<std::optional<std::pair<double, double>>, bool>, 2>
				candidates{{{arm_dir(way.nodes, i, -1),
									forward_ok &&
											!signal_upstream(way.nodes, i, -1, upstream)},
						{arm_dir(way.nodes, i, 1),
								backward_ok &&
										!signal_upstream(way.nodes, i, 1, upstream)}}};
		auto [entry, inserted] = nodes.try_emplace(
				node.id, SignalNode{node.x, node.z,
								 node.tags.contains("crossing") &&
										 node.tags.at("crossing") == "traffic_signals",
								 kind, {}});
		(void)inserted;
		for (const auto &[direction_vector, inbound] : candidates) {
			if (!direction_vector)
				continue;
			const auto [dx, dz] = *direction_vector;
			auto same = std::find_if(entry->second.arms.begin(), entry->second.arms.end(),
					[&, dx = dx, dz = dz](
							const Arm &arm) { return arm.dx * dx + arm.dz * dz > .9; });
			if (same != entry->second.arms.end()) {
				same->inbound = same->inbound || inbound;
				same->half = std::max(same->half, half);
			} else {
				entry->second.arms.push_back({dx, dz, half, inbound});
			}
		}
	}
}

std::pair<double, double> kerb_side(double tx, double tz, bool left)
{
	return left ? std::pair{tz, -tx} : std::pair{-tz, tx};
}

std::optional<std::tuple<int, int, int>> kerb_cell(double ox, double oz, double sx,
		double sz, int half, const RoadMaskBitmap &road_mask)
{
	for (int offset = half + 1; offset <= half + 4; ++offset) {
		const int x = static_cast<int>(std::floor(ox + sx * offset));
		const int z = static_cast<int>(std::floor(oz + sz * offset));
		if (!road_mask.contains(x, z))
			return std::tuple{x, z, offset};
	}
	return std::nullopt;
}

bool is_bike_path(const std::string &highway)
{
	return highway == "cycleway" || highway == "path" || highway == "bridleway";
}

bool crossable(const ProcessedWay &way)
{
	for (const auto *key : {"footway", "cycleway", "path"})
		if (const auto it = way.tags.find(key);
				it != way.tags.end() && it->second == "crossing")
			return false;
	if (const auto it = way.tags.find("highway"); it != way.tags.end())
		return is_bike_path(it->second);
	if (const auto it = way.tags.find("railway"); it != way.tags.end())
		return it->second == "tram" || it->second == "light_rail" ||
			   it->second == "rail" || it->second == "narrow_gauge";
	return false;
}

enum class Lamp
{
	Red,
	Amber,
	Green
};

const char *lamp_block(Lamp lamp)
{
	switch (lamp) {
	case Lamp::Red:
		return "minecraft:redstone_block";
	case Lamp::Amber:
		return "minecraft:yellow_concrete";
	case Lamp::Green:
		return "minecraft:emerald_block";
	}
	return "minecraft:redstone_block";
}

char lamp_tag(Lamp lamp)
{
	switch (lamp) {
	case Lamp::Red:
		return 'r';
	case Lamp::Amber:
		return 'y';
	case Lamp::Green:
		return 'g';
	}
	return 'r';
}

void add_signal_part(world_editor::WorldEditor &editor, std::array<double, 3> centre,
		double facing_x, double facing_z, double vertical, double forward,
		std::array<float, 3> size, const std::string &block,
		std::vector<std::string> tags, std::optional<std::pair<int, int>> brightness,
		std::int64_t seed)
{
	const double x = centre[0] + facing_x * forward;
	const double y = centre[1] + vertical;
	const double z = centre[2] + facing_z * forward;
	const auto floats = [](std::array<float, 3> value) {
		return nlohmann::json::array({value[0], value[1], value[2]});
	};
	nlohmann::json extra = {{"block_state", {{"Name", block}}},
			{"transformation",
					{{"left_rotation", {0.0f, 0.0f, 0.0f, 1.0f}},
							{"right_rotation", {0.0f, 0.0f, 0.0f, 1.0f}},
							{"translation", floats({-size[0] / 2.0f, -size[1] / 2.0f,
													-size[2] / 2.0f})},
							{"scale", floats(size)}}},
			{"Rotation", {static_cast<float>(std::atan2(-facing_x, facing_z) * 180.0 /
											 std::numbers::pi),
								 0.0f}}};
	if (!tags.empty())
		extra["Tags"] = tags;
	if (brightness)
		extra["brightness"] = {{"sky", brightness->first}, {"block", brightness->second}};
	editor.add_block_display(x, y, z, seed, std::move(extra));
}

void add_signal_head(world_editor::WorldEditor &editor, std::array<double, 3> centre,
		Kind kind, double facing_x, double facing_z, char group, double scale,
		std::int64_t &seed)
{
	const double k = scale;
	const auto part =
			[&](double vertical, double forward, std::array<float, 3> size,
					const std::string &block, std::vector<std::string> tags = {},
					std::optional<std::pair<int, int>> brightness = std::nullopt) {
				add_signal_part(editor, centre, facing_x, facing_z, vertical, forward,
						size, block, std::move(tags), brightness, seed++);
			};
	if (kind == Kind::Pedestrian) {
		part(0.0, 0.0, {0.34f, 0.72f, 0.26f}, "minecraft:black_concrete");
		for (const auto &[lamp, y] : std::array<std::pair<Lamp, double>, 2>{
					 {{Lamp::Red, 0.17}, {Lamp::Green, -0.17}}}) {
			const bool lit = group == 'a' ? lamp == Lamp::Red : lamp == Lamp::Green;
			const std::string tag =
					std::string("arnis.tl_p") + group + "_" + lamp_tag(lamp);
			part(y, 0.15, {0.24f, 0.24f, 0.04f}, lamp_block(lamp), {"arnis.tl", tag},
					lit ? std::pair{15, 15} : std::pair{3, 0});
			part(y + 0.135, 0.19, {0.28f, 0.03f, 0.12f}, "minecraft:black_concrete");
		}
		return;
	}
	part(0.0, 0.0,
			{static_cast<float>(0.42 * k), static_cast<float>(1.10 * k),
					static_cast<float>(0.30 * k)},
			"minecraft:black_concrete");
	for (const auto &[lamp, y] : std::array<std::pair<Lamp, double>, 3>{
				 {{Lamp::Red, 0.34}, {Lamp::Amber, 0.0}, {Lamp::Green, -0.34}}}) {
		const bool lit = group == 'a' ? lamp == Lamp::Green : lamp == Lamp::Red;
		const std::string tag = std::string("arnis.tl_") + group + "_" + lamp_tag(lamp);
		const float lamp_size = static_cast<float>(0.26 * k);
		part(y * k, 0.17 * k, {lamp_size, lamp_size, 0.04f}, lamp_block(lamp),
				{"arnis.tl", tag}, lit ? std::pair{15, 15} : std::pair{3, 0});
		part((y + 0.15) * k, 0.225 * k,
				{static_cast<float>(0.32 * k), static_cast<float>(0.035 * k),
						static_cast<float>(0.15 * k)},
				"minecraft:black_concrete");
	}
}

char group_of_impl(double dx, double dz)
{
	return std::abs(dx) >= std::abs(dz) ? 'a' : 'b';
}

} // namespace

bool signal_upstream(const std::vector<ProcessedNode> &nodes, std::size_t index, int step,
		double reach)
{
	return signal_upstream_impl(nodes, index, step, reach);
}

char group_of(double direction_x, double direction_z)
{
	return group_of_impl(direction_x, direction_z);
}

bool is_signal(const tags_t &tags)
{
	const auto highway = tags.find("highway");
	const auto crossing = tags.find("crossing");
	return (highway != tags.end() && highway->second == "traffic_signals") ||
		   (crossing != tags.end() && crossing->second == "traffic_signals");
}

bool is_vehicular(const std::string &highway)
{
	static const std::unordered_set<std::string> values = {"motorway", "motorway_link",
			"trunk", "trunk_link", "primary", "primary_link", "secondary",
			"secondary_link", "tertiary", "tertiary_link", "unclassified", "residential",
			"living_street", "road", "busway", "service"};
	return values.contains(highway);
}

std::shared_ptr<const SignalIndex> SignalIndex::build(
		const std::vector<ProcessedElement> &elements, double scale, bool drives_on_left,
		const RoadMaskBitmap &road_mask)
{
	std::vector<const ProcessedWay *> ways;
	for (const auto &element : elements)
		if (element.is_way()) {
			const auto &way = element.as_way();
			if (std::any_of(way.nodes.begin(), way.nodes.end(),
						[](const ProcessedNode &node) { return is_signal(node.tags); }))
				ways.push_back(&way);
		}

	std::unordered_map<std::uint64_t, SignalNode> nodes;
	for (const auto *way : ways) {
		const auto highway = way->tags.find("highway");
		if (highway == way->tags.end() || !is_vehicular(highway->second))
			continue;
		add_arms(nodes, *way, highway_block_range(highway->second, way->tags, scale),
				road_markings::oneway_sign(highway->second, way->tags), Kind::Car, scale,
				[](const ProcessedNode &) { return true; });
	}

	auto offroad = std::unordered_map<std::uint64_t, SignalNode>{};
	for (const auto *way : ways) {
		const auto highway_it = way->tags.find("highway");
		const std::string highway =
				highway_it == way->tags.end() ? "" : highway_it->second;
		const bool bike = !highway.empty() && is_bike_path(highway);
		const bool crossed = crossable(*way);
		if (!bike && !crossed)
			continue;
		const int half =
				highway.empty() ? 1 : highway_block_range(highway, way->tags, scale);
		const int oneway =
				highway.empty() ? 0 : road_markings::oneway_sign(highway, way->tags);
		add_arms(offroad, *way, half, oneway, Kind::Bike, scale,
				[&](const ProcessedNode &node) {
					if (nodes.contains(node.id))
						return false;
					const auto crossing = node.tags.find("crossing");
					return crossing != node.tags.end() &&
										   crossing->second == "traffic_signals"
								   ? crossed
								   : bike;
				});
	}
	for (auto &[id, node] : offroad) {
		(void)id;
		if (node.crossing)
			for (auto &arm : node.arms)
				arm.inbound = false;
	}
	nodes.insert(std::make_move_iterator(offroad.begin()),
			std::make_move_iterator(offroad.end()));

	auto result = std::make_shared<SignalIndex>();
	std::vector<std::uint64_t> ids;
	ids.reserve(nodes.size());
	for (const auto &[id, node] : nodes) {
		(void)node;
		ids.push_back(id);
	}
	std::sort(ids.begin(), ids.end());

	auto covered = [&](int x, int z, const Head &head) {
		return std::any_of(
				result->poles_.begin(), result->poles_.end(), [&](const Pole &pole) {
					if (std::abs(pole.x - x) > COVER || std::abs(pole.z - z) > COVER)
						return false;
					return std::any_of(
							pole.heads.begin(), pole.heads.end(), [&](const Head &other) {
								return other.kind == head.kind &&
									   other.facing_x * head.facing_x +
													   other.facing_z * head.facing_z >
											   .9;
							});
				});
	};
	auto pole_for = [&](std::uint64_t owner, int x, int z) {
		for (std::size_t i = 0; i < result->poles_.size(); ++i)
			if (std::abs(result->poles_[i].x - x) <= 1 &&
					std::abs(result->poles_[i].z - z) <= 1) {
				// Rust's Plan::pole keeps the first node as the pole's owner.
				// Later signal nodes may add heads to it, but must not schedule a
				// second placement of the same assembled pole.
				return i;
			}
		Pole new_pole;
		new_pole.x = x;
		new_pole.z = z;
		result->poles_.push_back(std::move(new_pole));
		const auto index = result->poles_.size() - 1;
		result->by_node_[owner].push_back(index);
		return index;
	};
	for (const auto id : ids) {
		const auto &node = nodes.at(id);
		const bool junction = node.arms.size() >= 3;
		for (std::size_t i = 0; i < node.arms.size(); ++i) {
			const auto &arm = node.arms[i];
			if (!arm.inbound)
				continue;
			const auto side = kerb_side(-arm.dx, -arm.dz, drives_on_left);
			int back = 1;
			if (junction) {
				for (std::size_t j = 0; j < node.arms.size(); ++j)
					if (j != i)
						back = std::max(back, node.arms[j].half);
				back += 2;
			}
			const double ox = node.x + .5 + arm.dx * back;
			const double oz = node.z + .5 + arm.dz * back;
			auto kerb = kerb_cell(ox, oz, side.first, side.second, arm.half, road_mask);
			if (!kerb && node.kind == Kind::Bike)
				kerb = kerb_cell(ox, oz, -side.first, -side.second, arm.half, road_mask);
			if (!kerb)
				continue;
			const auto [px, pz, offset] = *kerb;
			Head head{arm.dx, arm.dz, group_of(arm.dx, arm.dz), node.kind};
			if (covered(px, pz, head))
				continue;
			const auto pole = pole_for(id, px, pz);
			result->poles_[pole].heads.push_back(head);
			if (node.kind == Kind::Car && arm.half >= MAST_HALF &&
					!result->poles_[pole].has_mast) {
				bool both_ways = false;
				for (std::size_t j = 0; j < node.arms.size(); ++j)
					if (j != i && node.arms[j].inbound &&
							node.arms[j].dx * arm.dx + node.arms[j].dz * arm.dz < -.9)
						both_ways = true;
				const int lane = both_ways ? arm.half / 2 : 0;
				const double reach = std::max(1, offset - lane);
				auto &target_pole = result->poles_[pole];
				target_pole.mast_x = static_cast<int>(
						std::floor(target_pole.x + .5 - side.first * reach));
				target_pole.mast_z = static_cast<int>(
						std::floor(target_pole.z + .5 - side.second * reach));
				target_pole.mast_head = head;
				target_pole.has_mast = true;
				target_pole.top = MAST_Y;
			}
		}
		if (node.crossing && !junction && !node.arms.empty()) {
			const auto &road = node.arms.front();
			const double ax = -road.dz, az = road.dx;
			for (const double sign : {1.0, -1.0}) {
				const double sx = ax * sign, sz = az * sign;
				std::optional<std::size_t> existing;
				for (std::size_t pi = 0; pi < result->poles_.size(); ++pi) {
					const auto &pole = result->poles_[pi];
					const double dx = pole.x + .5 - (node.x + .5);
					const double dz = pole.z + .5 - (node.z + .5);
					const double lateral = dx * sx + dz * sz;
					if (lateral > 0.0 && std::abs(dx * road.dx + dz * road.dz) < 3.0 &&
							lateral <= road.half + 5) {
						existing = pi;
						break;
					}
				}
				std::size_t pole{};
				if (existing) {
					pole = *existing;
				} else {
					std::optional<std::tuple<int, int, int>> kerb;
					for (const double along : {0.0, 2.0, -2.0}) {
						const double from_x = node.x + .5 + road.dx * along;
						const double from_z = node.z + .5 + road.dz * along;
						kerb = kerb_cell(from_x, from_z, sx, sz, road.half, road_mask);
						if (kerb)
							break;
					}
					if (!kerb)
						continue;
					pole = pole_for(id, std::get<0>(*kerb), std::get<1>(*kerb));
				}
				Head pedestrian{-sx, -sz, group_of(road.dx, road.dz), Kind::Pedestrian};
				const auto &target = result->poles_[pole];
				if (!covered(target.x, target.z, pedestrian))
					result->poles_[pole].heads.push_back(pedestrian);
			}
		}
	}
	return result;
}

bool SignalIndex::place_node(world_editor::WorldEditor &editor, const ProcessedNode &node,
		const std::function<int(int, int)> &base_y_for) const
{
	using namespace block_definitions;
	const auto it = by_node_.find(node.id);
	if (it == by_node_.end())
		return false;
	for (const auto index : it->second) {
		const auto &pole = poles_.at(index);
		const int base_y = base_y_for(pole.x, pole.z);
		editor.set_block_absolute(COBBLESTONE_WALL, pole.x, base_y + 1, pole.z);
		for (int y = base_y + 2; y <= base_y + pole.top; ++y)
			editor.set_block_absolute(IRON_BARS, pole.x, y, pole.z);
		const bool entity_backend =
				editor.map_decals_enabled() || editor.has_entity_sink();
		std::int64_t display_seed = static_cast<std::int64_t>(index) * 1000;
		for (const auto &head : pole.heads) {
			if (entity_backend) {
				const double scale = head.kind == Kind::Bike ? BIKE_SCALE : 1.0;
				const double height = head.kind == Kind::Car	? CAR_HEAD_Y
									  : head.kind == Kind::Bike ? BIKE_HEAD_Y
																: PED_HEAD_Y;
				const double out = 0.0625 + 0.15 * scale;
				const std::array<double, 3> centre{pole.x + 0.5 + head.facing_x * out,
						base_y + height, pole.z + 0.5 + head.facing_z * out};
				add_signal_head(editor, centre, head.kind, head.facing_x, head.facing_z,
						head.group, scale, display_seed);
			} else {
				const int height = head.kind == Kind::Car ? 4 : 3;
				const int fx = std::abs(head.facing_x) >= std::abs(head.facing_z)
									   ? (head.facing_x > 0 ? 1 : -1)
									   : 0;
				const int fz = fx == 0 ? (head.facing_z > 0 ? 1 : -1) : 0;
				if (!fm_place_native_traffic_signal_head(
							editor, pole.x, base_y + height, pole.z, fx, fz))
					editor.set_block_absolute(
							BLACK_WOOL, pole.x + fx, base_y + height, pole.z + fz);
			}
		}
		if (pole.has_mast) {
			std::vector<std::pair<int, int>> points;
			for (const auto &[x, unused_y, z] : crate::bresenham::bresenham_line(
						 pole.x, 0, pole.z, pole.mast_x, 0, pole.mast_z)) {
				(void)unused_y;
				points.emplace_back(x, z);
			}
			const auto cells = connected_blocks::four_connected_line(points);
			for (const auto &[x, z] : cells) {
				const auto joins = [&, x = x, z = z](int dx, int dz) {
					return std::find(cells.begin(), cells.end(),
								   std::pair{x + dx, z + dz}) != cells.end();
				};
				editor.set_block_with_properties_absolute(
						connected_blocks::connected_fence(IRON_BARS, joins(0, -1),
								joins(0, 1), joins(1, 0), joins(-1, 0)),
						x, base_y + MAST_Y, z, std::nullopt, std::nullopt);
			}
			const int fx =
					std::abs(pole.mast_head.facing_x) >= std::abs(pole.mast_head.facing_z)
							? (pole.mast_head.facing_x > 0 ? 1 : -1)
							: 0;
			const int fz = fx == 0 ? (pole.mast_head.facing_z > 0 ? 1 : -1) : 0;
			if (entity_backend) {
				const auto &head = pole.mast_head;
				const double scale = head.kind == Kind::Bike ? BIKE_SCALE : 1.0;
				const std::array<double, 3> centre{
						pole.mast_x + 0.5, base_y + MAST_Y - 0.62, pole.mast_z + 0.5};
				add_signal_head(editor, centre, head.kind, head.facing_x, head.facing_z,
						head.group, scale, display_seed);
			} else {
				fm_place_native_traffic_signal_head(
						editor, pole.mast_x, base_y + MAST_Y, pole.mast_z, fx, fz);
			}
		}
	}
	return true;
}

} // namespace arnis::highways
