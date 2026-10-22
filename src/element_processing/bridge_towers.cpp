#include "bridge_towers.h"

#include "../block_definitions.h"
#include "../block_palette.h"
#include "../strict_parse.h"
#include "bridge_styles.h"
#include "connected_blocks.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <string>
#include <unordered_map>

namespace arnis::bridge_towers
{
namespace
{
constexpr double MIN_TOWER_M = 40.0;
constexpr double PART_GAP_M = 30.0;
constexpr double MIN_DECK_PART_M = 8.0;
constexpr float CABLE_LOW_ABOVE_DECK = 3.0f;
constexpr double HANGER_SPACING_M = 15.24;

struct Part
{
	const ProcessedWay *way = nullptr;
	int min_x = 0, min_z = 0, max_x = 0, max_z = 0;
	std::optional<double> height_m;
	double min_height_m = 0.0;
	int base_y = 0;
};

int rust_float_to_i32(double value)
{
	// Rust's float-to-i32 cast maps NaN to zero and saturates infinities and
	// out-of-range values. Avoid implementation-defined/undefined C++ casts for
	// malformed or extreme OSM dimensions.
	if (std::isnan(value))
		return 0;
	if (value >= static_cast<double>(std::numeric_limits<int>::max()))
		return std::numeric_limits<int>::max();
	if (value <= static_cast<double>(std::numeric_limits<int>::min()))
		return std::numeric_limits<int>::min();
	return static_cast<int>(value);
}

int rust_round_i32(double value)
{
	return rust_float_to_i32(std::round(value));
}

int saturating_add_i32(int a, int b)
{
	return static_cast<int>(std::clamp<std::int64_t>(static_cast<std::int64_t>(a) + b,
			std::numeric_limits<int>::min(), std::numeric_limits<int>::max()));
}

int saturating_sub_i32(int a, int b)
{
	return static_cast<int>(std::clamp<std::int64_t>(static_cast<std::int64_t>(a) - b,
			std::numeric_limits<int>::min(), std::numeric_limits<int>::max()));
}

int rust_total_cmp(double a, double b)
{
	const auto key = [](double value) {
		const auto bits = std::bit_cast<std::uint64_t>(value);
		return (bits & (std::uint64_t{1} << 63)) ? ~bits
												 : bits ^ (std::uint64_t{1} << 63);
	};
	const auto a_key = key(a);
	const auto b_key = key(b);
	return a_key < b_key ? -1 : a_key > b_key ? 1 : 0;
}

std::optional<double> metres(const tags_t &tags, const char *key)
{
	const auto it = tags.find(key);
	if (it == tags.end())
		return std::nullopt;
	std::string value = it->second;
	auto first = value.find_first_not_of(" \t\r\n");
	if (first == std::string::npos)
		return std::nullopt;
	auto last = value.find_last_not_of(" \t\r\n");
	value = value.substr(first, last - first + 1);
	while (!value.empty() && value.back() == 'm')
		value.pop_back();
	const auto trimmed = value.find_last_not_of(" \t\r\n");
	if (trimmed == std::string::npos)
		return std::nullopt;
	value.erase(trimmed + 1);
	return strict_parse::f64(value);
}

std::optional<Part> make_part(
		const ProcessedWay &way, const world_editor::WorldEditor &editor)
{
	if (way.nodes.empty())
		return std::nullopt;
	Part part;
	part.way = &way;
	part.min_x = part.max_x = way.nodes.front().x;
	part.min_z = part.max_z = way.nodes.front().z;
	std::optional<int> base;
	for (const auto &node : way.nodes) {
		part.min_x = std::min(part.min_x, node.x);
		part.max_x = std::max(part.max_x, node.x);
		part.min_z = std::min(part.min_z, node.z);
		part.max_z = std::max(part.max_z, node.z);
		if (const auto level = editor.terrain_level(node.x, node.z))
			base = base ? std::max(*base, *level) : *level;
	}
	part.height_m = metres(way.tags, "height");
	part.min_height_m = metres(way.tags, "min_height").value_or(0.0);
	part.base_y = base.value_or(
			editor.get_ground_level(way.nodes.front().x, way.nodes.front().z));
	return part;
}

int bbox_gap(const Part &a, const Part &b)
{
	const int dx = std::max({b.min_x - a.max_x, a.min_x - b.max_x, 0});
	const int dz = std::max({b.min_z - a.max_z, a.min_z - b.max_z, 0});
	return std::max(dx, dz);
}

std::pair<int, int> centre(const Part &p)
{
	return {(p.min_x + p.max_x) / 2, (p.min_z + p.max_z) / 2};
}
}

float CableSpan::u(int x, int z) const
{
	return (static_cast<float>(x) - origin_x) * dir_x +
		   (static_cast<float>(z) - origin_z) * dir_z;
}

std::optional<float> CableSpan::cable_y(float position) const
{
	for (std::size_t i = 0; i + 1 < points.size(); ++i) {
		const auto &[u0, y0] = points[i];
		const auto &[u1, y1] = points[i + 1];
		// Rust's inclusive range test rejects NaN. Written this way, ordered
		// comparisons in C++ would both be false and produce a NaN cable height.
		if (!(position >= u0 && position <= u1))
			continue;
		const float t = u1 > u0 ? (position - u0) / (u1 - u0) : 0.0f;
		return y0 + (y1 - y0) * t - sags[i] * 4.0f * t * (1.0f - t);
	}
	return std::nullopt;
}

long long CableSpan::hanger_bay(float position) const
{
	// Rust's float-to-int cast saturates (and maps NaN to zero); a direct C++
	// cast of NaN or an out-of-range float is undefined/implementation-defined.
	const float bay = std::floor(position / hanger_spacing);
	if (std::isnan(bay))
		return 0;
	if (bay <= static_cast<float>(std::numeric_limits<long long>::min()))
		return std::numeric_limits<long long>::min();
	if (bay >= static_cast<float>(std::numeric_limits<long long>::max()))
		return std::numeric_limits<long long>::max();
	return static_cast<long long>(bay);
}

bool CableSpan::suspends(int x, int z) const
{
	if (points.empty())
		return false;
	const float position = u(x, z);
	return position >= points.front().first && position <= points.back().first;
}

std::optional<CableSpan> make_cable_span(float origin_x, float origin_z, float dir_x,
		float dir_z, const std::vector<std::pair<float, int>> &towers,
		std::pair<float, float> anchors, const std::function<int(float)> &deck_at,
		Block paint, double scale)
{
	if (towers.size() < 2 || anchors.first >= towers.front().first ||
			anchors.second <= towers.back().first)
		return std::nullopt;
	CableSpan span;
	span.origin_x = origin_x;
	span.origin_z = origin_z;
	span.dir_x = dir_x;
	span.dir_z = dir_z;
	span.paint = paint;
	span.points.emplace_back(
			anchors.first, static_cast<float>(deck_at(anchors.first)) + 1.0f);
	for (const auto &[u, y] : towers)
		span.points.emplace_back(u, static_cast<float>(y));
	span.points.emplace_back(
			anchors.second, static_cast<float>(deck_at(anchors.second)) + 1.0f);
	for (std::size_t i = 1; i < span.points.size(); ++i)
		if (span.points[i].first - span.points[i - 1].first < 1.0f)
			return std::nullopt;
	span.sags.assign(span.points.size() - 1, 0.0f);
	for (std::size_t i = 1; i + 2 < span.points.size(); ++i) {
		const auto &[u0, y0] = span.points[i];
		const auto &[u1, y1] = span.points[i + 1];
		const float low =
				static_cast<float>(deck_at((u0 + u1) * 0.5f)) + CABLE_LOW_ABOVE_DECK;
		span.sags[i] = std::max(0.0f, (y0 + y1) * 0.5f - low);
	}
	const float main_length = span.points[2].first - span.points[1].first;
	const float main_sag = span.sags[1];
	for (const std::size_t i : {std::size_t{0}, span.points.size() - 2}) {
		const float length = span.points[i + 1].first - span.points[i].first;
		span.sags[i] = main_sag * (length / main_length) * (length / main_length);
	}
	const float main_span = towers.back().first - towers.front().first;
	const auto tower_intervals = std::max<std::size_t>(1, towers.size() - 1);
	span.girder_depth = std::clamp(
			static_cast<int>(std::lround(
					(main_span / static_cast<float>(tower_intervals)) / 170.0f)),
			2, 8);
	const double hanger_spacing = HANGER_SPACING_M * scale;
	// Rust's f64::max selects the non-NaN operand, so malformed scale input still
	// gets the minimum hanger pitch rather than propagating NaN into the index.
	span.hanger_spacing = static_cast<float>(
			std::isnan(hanger_spacing) ? 2.0 : std::max(hanger_spacing, 2.0));
	return span;
}

void decorate(world_editor::WorldEditor &editor,
		const std::vector<bridge_styles::BridgePathSample> &path,
		const std::function<std::vector<std::pair<std::size_t, std::pair<int, int>>>(
				std::size_t)> &sides,
		const CableSpan &span, Block hanger)
{
	std::array<std::optional<std::pair<std::pair<int, int>, int>>, 2> previous{};
	std::optional<long long> previous_bay;
	for (std::size_t i = 0; i < path.size(); ++i) {
		const auto &sample = path[i];
		const auto cable = span.cable_y(span.u(sample.x, sample.z));
		if (!cable) {
			previous = {};
			previous_bay.reset();
			continue;
		}
		const int cable_y = std::max(static_cast<int>(std::lround(*cable)), sample.y + 2);
		const auto bay = span.hanger_bay(span.u(sample.x, sample.z));
		const bool new_bay = previous_bay && *previous_bay != bay;
		previous_bay = bay;
		for (const auto &[side, cell] : sides(i)) {
			if (side > 1)
				continue;
			std::vector<std::pair<int, int>> run;
			if (previous[side] && previous[side]->first != cell)
				run = connected_blocks::stair_steps(previous[side]->first, cell);
			else
				run.push_back(cell);
			const int low = previous[side]
									? std::min(previous[side]->second, cable_y) +
											  (previous[side]->second < cable_y ? 1 : 0)
									: cable_y;
			const int high = previous[side]
									 ? std::max(previous[side]->second, cable_y) -
											   (previous[side]->second > cable_y ? 1 : 0)
									 : cable_y;
			for (const auto &[x, z] : run) {
				for (int y = sample.y - span.girder_depth; y < sample.y; ++y)
					editor.set_block_absolute(span.paint, x, y, z, std::nullopt,
							std::optional<std::vector<Block>>{std::vector<Block>{}});
				editor.set_block_absolute(span.paint, x, cable_y, z, std::nullopt,
						std::optional<std::vector<Block>>{std::vector<Block>{}});
			}
			for (int y = low; y <= high; ++y)
				editor.set_block_absolute(span.paint, cell.first, y, cell.second,
						std::nullopt,
						std::optional<std::vector<Block>>{std::vector<Block>{}});
			previous[side] = std::make_pair(cell, cable_y);
			if (new_bay && cable_y > sample.y + 3)
				for (int y = sample.y + 2; y < cable_y; ++y)
					editor.set_block_absolute(hanger, cell.first, y, cell.second,
							std::nullopt, std::nullopt);
		}
	}
}

bool is_support(const tags_t &tags)
{
	return tags.contains("bridge:support") ||
		   (tags.get("man_made") == "tower" && tags.get("tower:type") == "bridge");
}

Block support_block(const tags_t &tags)
{
	for (const auto *key : {"building:colour", "colour"})
		if (const auto it = tags.find(key); it != tags.end())
			if (const auto rgb = color_text_to_rgb_tuple(it->second))
				return block_palette::paint_block_for_color(*rgb);
	std::string material = tags.get("building:material");
	if (material.empty())
		material = tags.get("material");
	std::transform(material.begin(), material.end(), material.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	if (material == "steel" || material == "metal" || material == "iron")
		return block_definitions::IRON_BLOCK;
	if (material == "stone" || material == "granite" || material == "masonry")
		return block_definitions::STONE_BRICKS;
	if (material == "brick")
		return block_definitions::BRICK;
	return block_definitions::SMOOTH_STONE;
}

void place_support(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int, int>> &cells, int base_y, int height,
		bool grounded)
{
	const Block block = support_block(way.tags);
	const int top = base_y + std::max(1, height);
	for (const auto &[x, z] : cells) {
		const int bottom =
				grounded ? std::min(editor.get_ground_level(x, z) + 1, base_y + 1)
						 : base_y + 1;
		for (int y = bottom; y <= top; ++y)
			editor.set_block_absolute(block, x, y, z, std::nullopt, std::nullopt);
		if (grounded)
			editor.register_support_column(x, z, block);
	}
}

MappedSupports collect(const std::vector<ProcessedElement> &elements,
		const world_editor::WorldEditor &editor, double scale)
{
	MappedSupports result;
	if (!std::isfinite(scale) || scale <= 0.0)
		return result;
	std::vector<Part> supports;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if ((!way.tags.contains("building") && !way.tags.contains("building:part")) ||
				!is_support(way.tags))
			continue;
		if (auto part = make_part(way, editor))
			supports.push_back(*part);
	}
	if (supports.empty())
		return result;

	std::vector<std::size_t> parents(supports.size());
	std::iota(parents.begin(), parents.end(), 0);
	auto root = [&](std::size_t i) {
		while (parents[i] != i) {
			parents[i] = parents[parents[i]];
			i = parents[i];
		}
		return i;
	};
	const int gap = rust_float_to_i32(std::ceil(PART_GAP_M * scale));
	for (std::size_t i = 0; i < supports.size(); ++i)
		for (std::size_t j = i + 1; j < supports.size(); ++j)
			if (bbox_gap(supports[i], supports[j]) <= gap) {
				const auto a = root(i), b = root(j);
				parents[a] = b;
			}
	std::unordered_map<std::size_t, std::vector<std::size_t>> grouped;
	for (std::size_t i = 0; i < supports.size(); ++i)
		grouped[root(i)].push_back(i);
	std::vector<std::vector<std::size_t>> groups;
	groups.reserve(grouped.size());
	for (auto &[id, group] : grouped) {
		(void)id;
		groups.push_back(std::move(group));
	}
	std::sort(groups.begin(), groups.end(), [&](const auto &a, const auto &b) {
		return supports[a.front()].way->id < supports[b.front()].way->id;
	});

	for (const auto &group : groups) {
		Part box = supports[group.front()];
		const Part *tallest = nullptr;
		for (const auto index : group) {
			const auto &part = supports[index];
			box.min_x = std::min(box.min_x, part.min_x);
			box.min_z = std::min(box.min_z, part.min_z);
			box.max_x = std::max(box.max_x, part.max_x);
			box.max_z = std::max(box.max_z, part.max_z);
			if (part.height_m &&
					(!tallest || !tallest->height_m ||
							rust_total_cmp(*part.height_m, *tallest->height_m) >= 0))
				tallest = &part;
		}
		const auto [cx, cz] = centre(box);
		if (!tallest || !tallest->height_m || !(*tallest->height_m >= MIN_TOWER_M)) {
			result.minor.emplace_back(cx, cz);
			continue;
		}

		const auto is_near_tower = [&](const ProcessedWay &way) {
			if (way.nodes.empty())
				return false;
			std::int64_t sx = 0, sz = 0;
			for (const auto &node : way.nodes) {
				sx += node.x;
				sz += node.z;
			}
			const auto count = static_cast<std::int64_t>(way.nodes.size());
			const int ox = static_cast<int>(sx / count),
					  oz = static_cast<int>(sz / count);
			return ox >= box.min_x - 3 && ox <= box.max_x + 3 && oz >= box.min_z - 3 &&
				   oz <= box.max_z + 3;
		};
		std::optional<Part> deck_part;
		const auto consider_deck = [&, cx = cx, cz = cz](const Part &part) {
			if (!(part.min_height_m >= 2.0) || !part.height_m ||
					*part.height_m > 0.6 * *tallest->height_m)
				return;
			const int width = part.max_x - part.min_x;
			const int depth = part.max_z - part.min_z;
			if (std::min(width, depth) < rust_float_to_i32(MIN_DECK_PART_M * scale) ||
					cx < part.min_x || cx > part.max_x || cz < part.min_z ||
					cz > part.max_z)
				return;
			const auto area = static_cast<std::int64_t>(width) * depth;
			const auto old_area = deck_part
										  ? static_cast<std::int64_t>(
													deck_part->max_x - deck_part->min_x) *
													(deck_part->max_z - deck_part->min_z)
										  : -1;
			if (area >= old_area)
				deck_part = part;
		};
		// Rust considers both the clustered support ways and nearby untagged
		// building parts. A support slab can itself be the raised deck.
		for (const auto index : group)
			consider_deck(supports[index]);
		for (const auto &element : elements) {
			if (!element.is_way())
				continue;
			const auto &way = element.as_way();
			if (!way.tags.contains("building:part") || is_support(way.tags) ||
					!is_near_tower(way))
				continue;
			auto part = make_part(way, editor);
			if (part)
				consider_deck(*part);
		}
		std::optional<int> deck_y;
		if (deck_part && deck_part->height_m)
			deck_y = saturating_sub_i32(
					saturating_add_i32(deck_part->base_y,
							rust_round_i32(*deck_part->height_m * scale)),
					1);
		result.towers.push_back({cx, cz,
				saturating_add_i32(
						tallest->base_y, rust_round_i32(*tallest->height_m * scale)),
				deck_y, support_block(tallest->way->tags)});
	}
	return result;
}
}
