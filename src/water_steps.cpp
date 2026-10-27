#include "water_steps.h"

#include "block_definitions.h"
#include "element_processing/structure_kit.h"
#include "../../arnis_world_editor.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arnis::water_steps
{
namespace
{
constexpr int MAX_LEVEL = 7;
constexpr int MARGIN = 20;
constexpr int CAVE_DEPTH = 5;
constexpr int DEPTH_CAP = 16;
constexpr int REGION = 512;
constexpr std::uint8_t OPEN = 1;
constexpr std::uint8_t FREE = 2;
constexpr std::uint8_t PENDING = std::numeric_limits<std::uint8_t>::max();

std::uint64_t cell_key(int x, int z)
{
	return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) |
		   static_cast<std::uint32_t>(z);
}

bool wet_block(const BlockWithProperties &block)
{
	using namespace block_definitions;
	const auto id = block.block.id();
	const std::array<Block, 17> water_contents{WATER, SEAGRASS, TALL_SEAGRASS_BOTTOM,
			TALL_SEAGRASS_TOP, KELP, KELP_PLANT, SEA_PICKLE, TUBE_CORAL, BRAIN_CORAL,
			BUBBLE_CORAL, FIRE_CORAL, HORN_CORAL, TUBE_CORAL_FAN, BRAIN_CORAL_FAN,
			BUBBLE_CORAL_FAN, FIRE_CORAL_FAN, HORN_CORAL_FAN};
	if (std::any_of(water_contents.begin(), water_contents.end(),
				[&](const Block &candidate) { return candidate.id() == id; }))
		return true;
	const auto &properties = block.property_values();
	const auto waterlogged = properties.find("waterlogged");
	return waterlogged != properties.end() && waterlogged->second == "true";
}

bool wet_at(const world_editor::WorldEditor &editor, int x, int y, int z)
{
	const auto block = editor.get_block_with_properties_absolute(x, y, z);
	return block && wet_block(*block);
}

Column column(const world_editor::WorldEditor &editor, int x, int z, int top_hint)
{
	int top = top_hint;
	while (!wet_at(editor, x, top, z)) {
		if (top == std::numeric_limits<std::int16_t>::min())
			return {};
		--top;
	}
	while (top < std::numeric_limits<std::int16_t>::max() &&
			wet_at(editor, x, top + 1, z))
		++top;
	std::uint8_t depth = 1;
	while (depth < DEPTH_CAP && wet_at(editor, x, top - depth, z))
		++depth;
	const auto floor = editor.get_block_absolute(x, top - depth, z);
	const bool floored =
			depth == DEPTH_CAP || (floor && floor->id() != block_definitions::AIR.id());
	const auto above = editor.get_block_absolute(x, top + 1, z);
	const bool open = !above || above->id() == block_definitions::AIR.id();
	const auto top_block = editor.get_block_with_properties_absolute(x, top, z);
	const bool plain = top_block &&
					   top_block->block.id() == block_definitions::WATER.id() &&
					   top_block->property_values().empty();
	std::uint8_t flags = open ? OPEN : 0;
	if (open && plain && floored)
		flags |= FREE;
	return {static_cast<std::int16_t>(top), depth, flags};
}

struct Neighbours
{
	std::array<std::size_t, 4> cells{};
	std::size_t count = 0;
	const std::size_t *begin() const { return cells.data(); }
	const std::size_t *end() const { return cells.data() + count; }
};

Neighbours neighbours(std::size_t i, std::size_t w, std::size_t h)
{
	const auto x = i % w, z = i / w;
	Neighbours out;
	if (x > 0)
		out.cells[out.count++] = i - 1;
	if (x + 1 < w)
		out.cells[out.count++] = i + 1;
	if (z > 0)
		out.cells[out.count++] = i - w;
	if (z + 1 < h)
		out.cells[out.count++] = i + w;
	return out;
}

std::vector<std::pair<std::size_t, std::uint8_t>> levels(
		const std::vector<Column> &cols, std::size_t w, std::size_t h)
{
	std::vector<std::uint8_t> distance(cols.size(), 0);
	std::vector<std::size_t> reached;
	for (std::size_t i = 0; i < cols.size(); ++i) {
		if (!cols[i].free())
			continue;
		for (const auto j : neighbours(i, w, h))
			if (static_cast<int>(cols[j].top) == static_cast<int>(cols[i].top) - 1 &&
					(cols[j].flags & OPEN)) {
				distance[i] = 1;
				reached.push_back(i);
				break;
			}
	}
	for (std::size_t head = 0; head < reached.size(); ++head) {
		const auto i = reached[head];
		if (distance[i] > MAX_LEVEL)
			continue;
		for (const auto j : neighbours(i, w, h))
			if (distance[j] == 0 && cols[j].free() && cols[j].top == cols[i].top) {
				distance[j] = static_cast<std::uint8_t>(distance[i] + 1);
				reached.push_back(j);
			}
	}
	std::vector<std::uint8_t> flow(cols.size(), 0);
	std::vector<std::size_t> run_up;
	for (const auto i : reached) {
		if (distance[i] > MAX_LEVEL)
			continue;
		for (const auto j : neighbours(i, w, h))
			if (distance[j] == distance[i] + 1 && cols[j].top == cols[i].top) {
				flow[i] = PENDING;
				run_up.push_back(i);
				break;
			}
	}
	std::deque<std::size_t> queue;
	for (const auto i : run_up) {
		for (const auto j : neighbours(i, w, h))
			if (cols[j].wet_at(cols[i].top) &&
					(cols[j].top != cols[i].top || flow[j] != PENDING)) {
				flow[i] = 1;
				queue.push_back(i);
				break;
			}
	}
	while (!queue.empty()) {
		const auto i = queue.front();
		queue.pop_front();
		for (const auto j : neighbours(i, w, h))
			if (flow[j] == PENDING && cols[j].top == cols[i].top) {
				flow[j] = static_cast<std::uint8_t>(flow[i] + 1);
				queue.push_back(j);
			}
	}
	std::vector<std::pair<std::size_t, std::uint8_t>> out;
	for (const auto i : run_up)
		if (flow[i] <= MAX_LEVEL)
			out.emplace_back(i, flow[i]);
	return out;
}

struct Write
{
	int x, y, z;
	std::uint8_t level;
};

struct Rect
{
	int x0, z0, x1, z1;
	Rect grow(int amount) const
	{
		return {x0 - amount, z0 - amount, x1 + amount, z1 + amount};
	}
	Rect clip(const Rect &other) const
	{
		return {std::max(x0, other.x0), std::max(z0, other.z0), std::min(x1, other.x1),
				std::min(z1, other.z1)};
	}
	bool empty() const { return x0 > x1 || z0 > z1; }
	std::size_t width() const { return static_cast<std::size_t>(x1 - x0 + 1); }
	std::size_t height() const { return static_cast<std::size_t>(z1 - z0 + 1); }
};

int region_floor(int coordinate)
{
	int region = coordinate / REGION;
	if (coordinate % REGION < 0)
		--region;
	return region;
}

void apply_writes(world_editor::WorldEditor &editor, const std::vector<Write> &writes)
{
	std::array<BlockWithProperties, MAX_LEVEL + 1> flowing;
	for (int level = 0; level <= MAX_LEVEL; ++level)
		flowing[static_cast<std::size_t>(level)] = structure_kit::with_prop(
				block_definitions::WATER, "level", std::to_string(level));
	const std::optional<std::vector<Block>> water_only{
			std::vector<Block>{block_definitions::WATER}};
	for (const auto &write : writes)
		editor.set_block_with_properties_absolute(flowing[write.level], write.x, write.y,
				write.z, water_only, std::nullopt);
}

} // namespace

void collect_region_candidates(const world_editor::WorldEditor &editor, Borders &borders,
		int min_x, int max_x, int min_z, int max_z)
{
	std::unordered_map<std::uint64_t, int> tops;
	for (const auto &[x, y, z] : editor.written_cells) {
		if (x < min_x || x > max_x || z < min_z || z > max_z)
			continue;
		const auto block = editor.get_block_with_properties_absolute(x, y, z);
		if (!block || !wet_block(*block))
			continue;
		auto [it, inserted] = tops.emplace(cell_key(x, z), y);
		if (!inserted)
			it->second = std::max(it->second, y);
	}
	for (const auto &[key, y] : tops) {
		const int x = static_cast<std::int32_t>(key >> 32);
		const int z = static_cast<std::int32_t>(key & 0xffffffffU);
		const Borders::Region region{region_floor(x), region_floor(z)};
		borders.candidates[region].emplace_back(x, z, y);
	}
}

void shape_region(world_editor::WorldEditor &editor, Borders &borders,
		Borders::Region region, bool caves,
		const std::function<bool(Borders::Region)> &exists)
{
	const auto [area_min_x, area_min_z] = editor.get_min_coords();
	const auto [area_max_x, area_max_z] = editor.get_max_coords();
	const Rect area{area_min_x, area_min_z, area_max_x, area_max_z};
	const Rect region_rect{region.first * REGION, region.second * REGION,
			region.first * REGION + REGION - 1, region.second * REGION + REGION - 1};
	const Rect rect = region_rect.clip(area);
	if (rect.empty())
		return;
	const Rect win = rect.grow(MARGIN).clip(area);
	const auto width = win.width();
	const auto height = win.height();
	std::unordered_map<std::uint64_t, int> top_hints;
	for (int rz = region.second - 1; rz <= region.second + 1; ++rz) {
		for (int rx = region.first - 1; rx <= region.first + 1; ++rx) {
			const Borders::Region neighbour{rx, rz};
			if (borders.saved.contains(neighbour))
				continue;
			const auto candidates = borders.candidates.find(neighbour);
			if (candidates == borders.candidates.end())
				continue;
			for (const auto &[x, z, y] : candidates->second) {
				if (x < win.x0 || x > win.x1 || z < win.z0 || z > win.z1)
					continue;
				if (!wet_at(editor, x, y, z))
					continue;
				auto [it, inserted] = top_hints.emplace(cell_key(x, z), y);
				if (!inserted)
					it->second = std::max(it->second, y);
			}
		}
	}
	std::vector<Column> cols(width * height);
	for (const auto &[key, top] : top_hints) {
		const int x = static_cast<std::int32_t>(key >> 32);
		const int z = static_cast<std::int32_t>(key & 0xffffffffU);
		auto col = column(editor, x, z, top);
		if (!col.depth)
			continue;
		if (caves &&
				static_cast<int>(col.top) < editor.get_water_level(x, z) - CAVE_DEPTH)
			continue;
		if (x <= area.x0 || x >= area.x1 || z <= area.z0 || z >= area.z1)
			col.flags &= static_cast<std::uint8_t>(~FREE);
		cols[static_cast<std::size_t>(z - win.z0) * width +
				static_cast<std::size_t>(x - win.x0)] = col;
	}
	for (int rz = region.second - 1; rz <= region.second + 1; ++rz) {
		for (int rx = region.first - 1; rx <= region.first + 1; ++rx) {
			const Borders::Region neighbour{rx, rz};
			const auto saved = borders.saved.find(neighbour);
			if (saved == borders.saved.end())
				continue;
			for (const auto &[index, col] : saved->second) {
				const int x = rx * REGION + static_cast<int>(index % REGION);
				const int z = rz * REGION + static_cast<int>(index / REGION);
				if (x >= win.x0 && x <= win.x1 && z >= win.z0 && z <= win.z1)
					cols[static_cast<std::size_t>(z - win.z0) * width +
							static_cast<std::size_t>(x - win.x0)] = col;
			}
		}
	}
	std::vector<Write> writes;
	for (const auto &[index, level] : levels(cols, width, height)) {
		const int x = win.x0 + static_cast<int>(index % width);
		const int z = win.z0 + static_cast<int>(index / width);
		if (x >= rect.x0 && x <= rect.x1 && z >= rect.z0 && z <= rect.z1)
			writes.push_back({x, cols[index].top, z, level});
	}
	apply_writes(editor, writes);

	Borders::Border border;
	for (int z = rect.z0; z <= rect.z1; ++z) {
		const bool edge_z = z - region_rect.z0 < MARGIN || region_rect.z1 - z < MARGIN;
		for (int x = rect.x0; x <= rect.x1; ++x) {
			const auto &col = cols[static_cast<std::size_t>(z - win.z0) * width +
								   static_cast<std::size_t>(x - win.x0)];
			if (col.depth > 0 && (edge_z || x - region_rect.x0 < MARGIN ||
										 region_rect.x1 - x < MARGIN)) {
				const auto index = static_cast<std::uint32_t>(
						(z - region_rect.z0) * REGION + x - region_rect.x0);
				border.emplace_back(index, col);
			}
		}
	}
	borders.candidates.erase(region);
	borders.shaped.insert(region);
	borders.saved.insert_or_assign(region, std::move(border));
	for (int dz = -1; dz <= 1; ++dz) {
		for (int dx = -1; dx <= 1; ++dx) {
			const Borders::Region candidate{region.first + dx, region.second + dz};
			if (!borders.saved.contains(candidate))
				continue;
			bool settled = true;
			for (int nz = -1; nz <= 1 && settled; ++nz)
				for (int nx = -1; nx <= 1; ++nx) {
					const Borders::Region around{
							candidate.first + nx, candidate.second + nz};
					if (around != candidate && exists(around) &&
							!borders.shaped.contains(around)) {
						settled = false;
						break;
					}
				}
			if (settled)
				borders.saved.erase(candidate);
		}
	}
}

void shape_world(world_editor::WorldEditor &editor, bool caves)
{
	const auto [min_x, min_z] = editor.get_min_coords();
	const auto [max_x, max_z] = editor.get_max_coords();
	if (min_x > max_x || min_z > max_z)
		return;

	// Index only generated wet columns. This avoids probing the full vertical
	// world range for dry terrain and keeps runtime proportional to water volume.
	std::unordered_map<std::uint64_t, int> tops;
	tops.reserve(editor.written_cells.size() / 8 + 1);
	for (const auto &[x, y, z] : editor.written_cells) {
		const auto block = editor.get_block_with_properties_absolute(x, y, z);
		if (block && wet_block(*block)) {
			auto [it, inserted] = tops.emplace(cell_key(x, z), y);
			if (!inserted)
				it->second = std::max(it->second, y);
		}
	}
	if (tops.empty())
		return;
	std::unordered_map<std::uint64_t, Column> columns;
	columns.reserve(tops.size());
	for (const auto &[key, top] : tops) {
		const int x = static_cast<std::int32_t>(key >> 32);
		const int z = static_cast<std::int32_t>(key & 0xffffffffU);
		auto col = column(editor, x, z, top);
		if (!col.depth)
			continue;
		if (caves &&
				static_cast<int>(col.top) < editor.get_water_level(x, z) - CAVE_DEPTH)
			continue;
		if (x <= min_x || x >= max_x || z <= min_z || z >= max_z)
			col.flags &= static_cast<std::uint8_t>(~FREE);
		columns.emplace(key, col);
	}

	std::vector<Write> writes;
	// Rust's signed right shift maps negative coordinates to the region below
	// zero; use the shared floor-division helper so this stays explicit and
	// consistent with the streamed-region path on every supported compiler.
	for (int rz = region_floor(min_z); rz <= region_floor(max_z); ++rz) {
		for (int rx = region_floor(min_x); rx <= region_floor(max_x); ++rx) {
			const int x0 = std::max(min_x, rx * REGION - MARGIN);
			const int z0 = std::max(min_z, rz * REGION - MARGIN);
			const int x1 = std::min(max_x, rx * REGION + REGION - 1 + MARGIN);
			const int z1 = std::min(max_z, rz * REGION + REGION - 1 + MARGIN);
			const auto w = static_cast<std::size_t>(x1 - x0 + 1);
			const auto h = static_cast<std::size_t>(z1 - z0 + 1);
			std::vector<Column> cols(w * h);
			for (int z = z0; z <= z1; ++z)
				for (int x = x0; x <= x1; ++x)
					if (const auto found = columns.find(cell_key(x, z));
							found != columns.end())
						cols[static_cast<std::size_t>(z - z0) * w + (x - x0)] =
								found->second;
			for (const auto &[index, level] : levels(cols, w, h)) {
				const int x = x0 + static_cast<int>(index % w);
				const int z = z0 + static_cast<int>(index / w);
				if (x >= rx * REGION && x < rx * REGION + REGION && z >= rz * REGION &&
						z < rz * REGION + REGION && x >= min_x && x <= max_x &&
						z >= min_z && z <= max_z)
					writes.push_back({x, cols[index].top, z, level});
			}
		}
	}
	std::array<BlockWithProperties, MAX_LEVEL + 1> flowing;
	for (int level = 0; level <= MAX_LEVEL; ++level)
		flowing[static_cast<std::size_t>(level)] = structure_kit::with_prop(
				block_definitions::WATER, "level", std::to_string(level));
	const std::optional<std::vector<Block>> water_only{
			std::vector<Block>{block_definitions::WATER}};
	for (const auto &write : writes)
		editor.set_block_with_properties_absolute(flowing[write.level], write.x, write.y,
				write.z, water_only, std::nullopt);
}

} // namespace arnis::water_steps
