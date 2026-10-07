#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <string>
#include <optional>
#include <tuple>
#include <utility>
#include <deque>
#include <initializer_list>
#include <iterator>
#include <map>
#include <set>

#include "buildings_interior.h"

#include "../../../../arnis_adapter.h"
#include "../../floodfill_cache.h"
#include "buildings_loot.h"
namespace arnis
{

namespace
{
bool chest_gate(int x, int z, int floor_y, unsigned salt, unsigned modulus)
{
	const unsigned h = static_cast<unsigned>(x) * 0x9E3779B1u ^
					   static_cast<unsigned>(z) * 0x85EBCA77u ^
					   static_cast<unsigned>(floor_y) * 0xC2B2AE35u ^ salt;
	return h % modulus == 0;
}
constexpr int BUILDING_PASSAGE_HEIGHT = 4;
constexpr std::array<std::pair<int, int>, 4> INTERIOR_DIRS = {
		std::pair{0, -1}, std::pair{1, 0}, std::pair{0, 1}, std::pair{-1, 0}};

std::uint64_t interior_mix(int x, int z, std::uint64_t salt)
{
	std::uint64_t h = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) |
					  static_cast<std::uint32_t>(z);
	h ^= salt * 0x9E3779B97F4A7C15ULL;
	h = (h ^ (h >> 30)) * 0xBF58476D1CE4E5B9ULL;
	h = (h ^ (h >> 27)) * 0x94D049BB133111EBULL;
	return h ^ (h >> 31);
}

const char *ladder_facing(std::pair<int, int> direction)
{
	if (direction.first > 0)
		return "east";
	if (direction.first < 0)
		return "west";
	return direction.second > 0 ? "south" : "north";
}

std::uint64_t interior_cell_key(int x, int z)
{
	return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) |
		   static_cast<std::uint32_t>(z);
}

int floor_mod(int value, int divisor)
{
	return (value % divisor + divisor) % divisor;
}

void connect_partition_zones(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &cells,
		const std::unordered_map<std::uint64_t, std::uint16_t> &zone_of,
		const std::unordered_set<std::uint64_t> &planned,
		std::vector<std::pair<int, int>> &built, std::vector<std::pair<int, int>> &doors,
		std::unordered_set<std::uint64_t> &kept,
		const std::vector<std::pair<int, int>> &starts, int floor_abs, int top_abs)
{
	if (starts.empty() || built.empty())
		return;
	std::vector<std::pair<int, int>> ordered_cells = cells;
	std::sort(
			ordered_cells.begin(), ordered_cells.end(), [](const auto &a, const auto &b) {
				return std::pair(a.second, a.first) < std::pair(b.second, b.first);
			});
	std::sort(built.begin(), built.end());
	const auto door_set = [&] {
		std::unordered_set<std::uint64_t> result;
		result.reserve(doors.size());
		for (const auto &[x, z] : doors)
			result.insert(interior_cell_key(x, z));
		return result;
	};
	auto door_cells = door_set();
	auto passable = [&](int x, int z) {
		const auto key = interior_cell_key(x, z);
		return zone_of.contains(key) &&
			   (!planned.contains(key) || door_cells.contains(key));
	};
	for (unsigned iteration = 0; iteration < 64; ++iteration) {
		std::unordered_set<std::uint64_t> reached;
		std::deque<std::pair<int, int>> queue;
		for (const auto &start : starts)
			if (passable(start.first, start.second) &&
					reached.insert(interior_cell_key(start.first, start.second)).second)
				queue.push_back(start);
		while (!queue.empty()) {
			const auto [x, z] = queue.front();
			queue.pop_front();
			for (const auto &[dx, dz] : INTERIOR_DIRS) {
				const int nx = x + dx, nz = z + dz;
				const auto key = interior_cell_key(nx, nz);
				if (passable(nx, nz) && reached.insert(key).second)
					queue.emplace_back(nx, nz);
			}
		}
		using Cell = std::pair<int, int>;
		std::unordered_map<std::uint64_t, Cell> part;
		std::unordered_map<std::uint64_t, std::array<std::int64_t, 3>> sums;
		for (const auto &[sx, sz] : ordered_cells) {
			const auto seed = interior_cell_key(sx, sz);
			if (!passable(sx, sz) || reached.contains(seed) || part.contains(seed))
				continue;
			part.emplace(seed, Cell{sx, sz});
			std::deque<Cell> pending{{sx, sz}};
			while (!pending.empty()) {
				const auto [x, z] = pending.front();
				pending.pop_front();
				auto &sum = sums[seed];
				sum[0] += x;
				sum[1] += z;
				++sum[2];
				for (const auto &[dx, dz] : INTERIOR_DIRS) {
					const int nx = x + dx, nz = z + dz;
					const auto key = interior_cell_key(nx, nz);
					if (passable(nx, nz) && !reached.contains(key) &&
							part.emplace(key, Cell{sx, sz}).second)
						pending.emplace_back(nx, nz);
				}
			}
		}
		if (part.empty())
			return;
		struct Opening
		{
			std::int64_t distance;
			Cell wall;
			Cell direction;
		};
		std::map<std::uint64_t, Opening> best;
		for (const auto &[wx, wz] : built) {
			if (door_cells.contains(interior_cell_key(wx, wz)))
				continue;
			for (const auto &[dx, dz] : INTERIOR_DIRS) {
				const int fx = wx - dx, fz = wz - dz;
				const int ix = wx + dx, iz = wz + dz;
				const auto part_it = part.find(interior_cell_key(ix, iz));
				if (part_it == part.end() || !reached.contains(interior_cell_key(fx, fz)))
					continue;
				const auto label =
						interior_cell_key(part_it->second.first, part_it->second.second);
				const auto &sum = sums.at(label);
				const auto mx = sum[0] / sum[2], mz = sum[1] / sum[2];
				const auto dxm = static_cast<std::int64_t>(wx) - mx;
				const auto dzm = static_cast<std::int64_t>(wz) - mz;
				const auto distance = dxm * dxm + dzm * dzm;
				auto found = best.find(label);
				if (found == best.end() || distance < found->second.distance)
					best.insert_or_assign(label, Opening{distance, {wx, wz}, {dx, dz}});
			}
		}
		if (best.empty())
			return;
		for (const auto &[label, opening] : best) {
			(void)label;
			const auto [x, z] = opening.wall;
			const auto [dx, dz] = opening.direction;
			const char *facing = ladder_facing({dx, dz});
			const BlockWithProperties lower{DARK_OAK_DOOR_LOWER,
					{{"half", "lower"}, {"facing", facing}, {"hinge", "left"}}};
			const BlockWithProperties upper{DARK_OAK_DOOR_UPPER,
					{{"half", "upper"}, {"facing", facing}, {"hinge", "left"}}};
			editor.set_block_with_properties_absolute(
					lower, x, floor_abs + 1, z, std::nullopt, std::nullopt);
			if (top_abs >= floor_abs + 2)
				editor.set_block_with_properties_absolute(
						upper, x, floor_abs + 2, z, std::nullopt, std::nullopt);
			if (std::find(doors.begin(), doors.end(), opening.wall) == doors.end())
				doors.push_back(opening.wall);
			door_cells.insert(interior_cell_key(x, z));
			for (const auto &side : {Cell{x + dx, z + dz}, Cell{x - dx, z - dz}})
				if (zone_of.contains(interior_cell_key(side.first, side.second)) &&
						!editor.get_block_absolute(
								side.first, floor_abs + 1, side.second))
					kept.insert(interior_cell_key(side.first, side.second));
		}
	}
}

void furnish_shop(WorldEditor &editor, const interior_uses::Unit &unit,
		const std::vector<std::pair<int, int>> &cells,
		const std::vector<interior_uses::Entry> &entries,
		const std::vector<std::pair<int, int>> &door_positions,
		const std::unordered_set<std::uint64_t> &structural,
		std::unordered_set<std::uint64_t> &kept,
		const std::unordered_map<std::uint64_t, double> &claimed_top,
		const CoordinateBitmap &passages, double floor_metres, int passage_top,
		int floor_y, int base_y, int ceiling_y, std::uint64_t seed, unsigned salt)
{
	if (cells.empty())
		return;
	using interior_uses::Goods;
	std::vector<Block> stock, wares;
	switch (unit.use.goods) {
	case Goods::General:
		stock = {BARREL, BARREL, HAY_BALE, PUMPKIN, COMPOSTER};
		break;
	case Goods::Bakery:
		stock = {HAY_BALE, BARREL, HAY_BALE, BROWN_TERRACOTTA};
		wares = {CAKE, CAKE, EMPTY_FLOWER_POT};
		break;
	case Goods::Butcher:
		stock = {SMOKER, BARREL, WHITE_CONCRETE};
		break;
	case Goods::Grocery:
		stock = {HAY_BALE, MELON, PUMPKIN, BARREL, COMPOSTER};
		break;
	case Goods::Books:
		stock = {BOOKSHELF, BOOKSHELF, CHISELLED_BOOKSHELF_NORTH};
		wares = {LECTERN};
		break;
	case Goods::Clothes:
		stock = {WHITE_WOOL, RED_WOOL, BLUE_WOOL, YELLOW_WOOL, GREEN_WOOL, BLACK_WOOL,
				CYAN_WOOL, GRAY_WOOL};
		break;
	case Goods::Electronics:
		stock = {BLACK_CONCRETE, GRAY_CONCRETE, REDSTONE_LAMP, LIGHT_GRAY_CONCRETE};
		wares = {DAYLIGHT_DETECTOR, LANTERN};
		break;
	case Goods::Hardware:
		stock = {BARREL, IRON_BLOCK, BARREL, CHEST, SMITHING_TABLE};
		break;
	case Goods::Pharmacy:
		stock = {QUARTZ_BLOCK, WHITE_CONCRETE, CHISELLED_BOOKSHELF_NORTH};
		wares = {BREWING_STAND, EMPTY_FLOWER_POT};
		break;
	case Goods::Florist:
		stock = {MOSS_BLOCK, AZALEA_LEAVES, FLOWERING_AZALEA_LEAVES, OAK_LEAVES};
		wares = {POTTED_RED_TULIP, POTTED_DANDELION, POTTED_BLUE_ORCHID, FLOWER_POT};
		break;
	case Goods::Jewelry:
		stock = {SMOOTH_QUARTZ, CHISELED_QUARTZ_BLOCK};
		wares = {AMETHYST_CLUSTER, LANTERN, AMETHYST_CLUSTER};
		break;
	case Goods::Toys:
		stock = {RED_CONCRETE, YELLOW_CONCRETE, BLUE_CONCRETE, LIME_CONCRETE,
				MAGENTA_CONCRETE, ORANGE_CONCRETE};
		wares = {NOTE_BLOCK, EMPTY_FLOWER_POT};
		break;
	case Goods::Furniture:
		stock = {BOOKSHELF, BARREL, CHISELLED_BOOKSHELF_NORTH};
		break;
	case Goods::Drinks:
		stock = {BARREL, BARREL, BARREL, HAY_BALE};
		wares = {BREWING_STAND, EMPTY_FLOWER_POT};
		break;
	case Goods::Salon:
		stock = {WHITE_CONCRETE, QUARTZ_BLOCK};
		break;
	}
	const auto zone_key = [](int x, int z) { return interior_cell_key(x, z); };
	std::unordered_set<std::uint64_t> zone;
	zone.reserve(cells.size());
	int min_x = cells.front().first, max_x = min_x;
	int min_z = cells.front().second, max_z = min_z;
	for (const auto &[x, z] : cells) {
		zone.insert(zone_key(x, z));
		min_x = std::min(min_x, x);
		max_x = std::max(max_x, x);
		min_z = std::min(min_z, z);
		max_z = std::max(max_z, z);
	}
	std::optional<interior_uses::Entry> unit_entry;
	for (const auto &candidate : entries) {
		if (zone.contains(zone_key(candidate.cell.first, candidate.cell.second))) {
			unit_entry = candidate;
			break;
		}
	}
	if (!unit_entry) {
		for (const auto &[dx, dz] : door_positions) {
			for (const auto d : INTERIOR_DIRS) {
				const int x = dx + d.first, z = dz + d.second;
				if (zone.contains(zone_key(x, z))) {
					unit_entry = interior_uses::Entry{{x, z}, d};
					break;
				}
			}
			if (unit_entry)
				break;
		}
	}
	const std::pair<int, int> normal =
			unit_entry ? unit_entry->inward
					   : (max_x - min_x >= max_z - min_z ? std::pair{1, 0}
														 : std::pair{0, 1});
	const std::pair<int, int> tangent{-normal.second, normal.first};
	const int origin_x = normal.first + tangent.first > 0 ? min_x : max_x;
	const int origin_z = normal.second + tangent.second > 0 ? min_z : max_z;
	const int width = tangent.first != 0 ? max_x - min_x + 1 : max_z - min_z + 1;
	const int depth = normal.first != 0 ? max_x - min_x + 1 : max_z - min_z + 1;
	auto world = [&](int u, int v) {
		return std::pair{origin_x + tangent.first * u + normal.first * v,
				origin_z + tangent.second * u + normal.second * v};
	};
	auto local = [&](int x, int z) {
		const int dx = x - origin_x, dz = z - origin_z;
		return std::pair{dx * tangent.first + dz * tangent.second,
				dx * normal.first + dz * normal.second};
	};
	const int entry_u =
			unit_entry ? local(unit_entry->cell.first, unit_entry->cell.second).first
					   : width / 2;
	auto free = [&](int x, int z, int dy = 1) {
		const auto key = zone_key(x, z);
		if (!zone.contains(key) || structural.contains(key) || kept.contains(key))
			return false;
		const auto claim = claimed_top.find(key);
		if (claim != claimed_top.end() && claim->second > floor_metres + 1.0)
			return false;
		if (passages.contains(x, z) && floor_y < passage_top)
			return false;
		return !editor.get_block_absolute(x, base_y + dy - 1, z);
	};
	auto put = [&](int x, int z, int dy, const Block &block) {
		if (!free(x, z, dy))
			return false;
		editor.set_block_absolute(
				block, x, base_y + dy - 1, z, std::nullopt, std::nullopt);
		return true;
	};
	auto window_behind = [&](int x, int z, std::pair<int, int> inward) {
		const auto name = editor.block_name_absolute(
				x - inward.first, floor_y + 2, z - inward.second);
		return name && (name->find("glass") != std::string::npos ||
							   name->find("pane") != std::string::npos);
	};
	auto mount = [&](int x, int z, int dy, std::pair<int, int> inward,
						 const Block &block) {
		const int bx = x - inward.first, bz = z - inward.second;
		const auto behind = zone_key(bx, bz);
		const bool backed =
				!zone.contains(behind) || structural.contains(behind) ||
				editor.get_block_absolute(bx, base_y + dy - 1, bz).has_value();
		return backed && put(x, z, dy, block);
	};
	auto put_top_slab = [&](int x, int z, int dy, const Block &block) {
		if (!free(x, z, dy))
			return false;
		const BlockWithProperties slab{block, {{"type", "top"}}};
		editor.set_block_with_properties_absolute(
				slab, x, base_y + dy - 1, z, std::nullopt, std::nullopt);
		return true;
	};
	const auto put_bookshelf = [&](int x, int z, int dy, std::pair<int, int> facing,
									   std::uint64_t hash) {
		static constexpr bool fills[4][6] = {{true, true, false, true, true, true},
				{true, false, true, true, true, false},
				{false, true, true, true, false, true},
				{true, true, true, false, true, true}};
		const auto &slots = fills[hash % 4];
		const BlockWithProperties shelf{CHISELLED_BOOKSHELF_NORTH,
				{{"facing", ladder_facing(facing)},
						{"slot_0_occupied", slots[0] ? "true" : "false"},
						{"slot_1_occupied", slots[1] ? "true" : "false"},
						{"slot_2_occupied", slots[2] ? "true" : "false"},
						{"slot_3_occupied", slots[3] ? "true" : "false"},
						{"slot_4_occupied", slots[4] ? "true" : "false"},
						{"slot_5_occupied", slots[5] ? "true" : "false"}}};
		if (!free(x, z, dy))
			return false;
		editor.set_block_with_properties_absolute(
				shelf, x, base_y + dy - 1, z, std::nullopt, std::nullopt);
		return true;
	};
	std::unordered_set<std::uint64_t> open_geometry;
	for (const auto &[x, z] : cells) {
		const auto key = zone_key(x, z);
		if (!structural.contains(key) &&
				(!editor.get_block_absolute(x, base_y, z) || kept.contains(key)))
			open_geometry.insert(key);
	}
	std::unordered_map<std::uint64_t, int> depth_at;
	std::deque<std::pair<int, int>> depth_queue;
	for (const auto &[x, z] : cells) {
		const auto key = zone_key(x, z);
		if (!open_geometry.contains(key))
			continue;
		if (std::any_of(INTERIOR_DIRS.begin(), INTERIOR_DIRS.end(), [&](const auto &d) {
				return !open_geometry.contains(zone_key(x + d.first, z + d.second));
			})) {
			depth_at.emplace(key, 1);
			depth_queue.emplace_back(x, z);
		}
	}
	while (!depth_queue.empty()) {
		const auto [x, z] = depth_queue.front();
		depth_queue.pop_front();
		const int next_depth = depth_at.at(zone_key(x, z)) + 1;
		for (const auto d : INTERIOR_DIRS) {
			const int nx = x + d.first, nz = z + d.second;
			const auto key = zone_key(nx, nz);
			if (open_geometry.contains(key) && !depth_at.contains(key)) {
				depth_at.emplace(key, next_depth);
				depth_queue.emplace_back(nx, nz);
			}
		}
	}
	const Block counter = unit.use.goods == Goods::Pharmacy ||
										  unit.use.goods == Goods::Jewelry ||
										  unit.use.goods == Goods::Salon
								  ? QUARTZ_BLOCK
						  : unit.use.goods == Goods::Butcher	 ? WHITE_CONCRETE
						  : unit.use.goods == Goods::Electronics ? POLISHED_ANDESITE
																 : OAK_PLANKS;
	const Block top_item = wares.empty() ? LANTERN : wares.front();
	for (const int side : {1, -1}) {
		const int u = entry_u + 2 * side;
		std::array<std::pair<int, int>, 3> desk;
		bool fits = true;
		for (int v = 1; v <= 3; ++v) {
			desk[v - 1] = world(u, v);
			fits &= free(desk[v - 1].first, desk[v - 1].second);
		}
		if (!fits)
			continue;
		for (std::size_t i = 0; i < desk.size(); ++i) {
			put(desk[i].first, desk[i].second, 1, counter);
			if (i == 1)
				put(desk[i].first, desk[i].second, 2, top_item);
		}
		for (int v = 1; v <= 3; ++v) {
			const auto [x, z] = world(u + side, v);
			kept.insert(zone_key(x, z));
		}
		break;
	}
	std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> walls;
	for (const auto &[x, z] : cells) {
		if (!free(x, z))
			continue;
		for (const auto d : INTERIOR_DIRS) {
			const int nx = x + d.first, nz = z + d.second;
			if (!zone.contains(zone_key(nx, nz)) ||
					structural.contains(zone_key(nx, nz)) ||
					editor.get_block_absolute(nx, base_y, nz)) {
				walls.emplace_back(std::pair{x, z}, std::pair{-d.first, -d.second});
				break;
			}
		}
	}
	std::stable_sort(walls.begin(), walls.end(), [](const auto &a, const auto &b) {
		return std::pair(a.first.second, a.first.first) <
			   std::pair(b.first.second, b.first.first);
	});
	if (unit.use.goods == Goods::Salon) {
		for (const auto &[cell, inward] : walls) {
			const auto [u, v] = local(cell.first, cell.second);
			if (v < 2 || (u + v) % 2 != 0)
				continue;
			const int sx = cell.first + inward.first;
			const int sz = cell.second + inward.second;
			if (!free(sx, sz))
				continue;
			const auto h = interior_mix(cell.first, cell.second, seed);
			if (h % 4 == 0) {
				put(cell.first, cell.second, 1, WATER_CAULDRON);
			} else {
				put_top_slab(cell.first, cell.second, 1, OAK_SLAB);
				mount(cell.first, cell.second, 2, inward, LIGHT_GRAY_STAINED_GLASS);
			}
			const BlockWithProperties seat{
					OAK_STAIRS, {{"facing", ladder_facing(inward)}, {"half", "bottom"},
										{"shape", "straight"}}};
			editor.set_block_with_properties_absolute(
					seat, sx, base_y, sz, std::nullopt, std::nullopt);
		}
		for (const int u : {0, width - 1}) {
			const auto [x, z] = world(u, 1);
			const auto h = interior_mix(x, z, seed ^ 1u);
			put(x, z, 1, h % 2 == 0 ? AZALEA : FLOWERING_AZALEA);
		}
		return;
	}
	if (unit.use.goods == Goods::Furniture) {
		for (const auto &[cell, inward] : walls) {
			const auto h = interior_mix(cell.first, cell.second, seed);
			if (h % 3 == 0)
				put_bookshelf(cell.first, cell.second, 1, inward, h);
			else if (h % 3 == 1)
				put(cell.first, cell.second, 1, BARREL);
			(void)inward;
		}
		for (const auto &[x, z] : cells) {
			const auto depth = depth_at.find(zone_key(x, z));
			if (depth == depth_at.end() || depth->second < 3)
				continue;
			const auto [u, v] = local(x, z);
			if (u % 5 != 2 || v % 5 != 2)
				continue;
			switch (interior_mix(x, z, seed ^ 0xF0u) % 3) {
			case 0:
				if (put_top_slab(x, z, 1, OAK_SLAB)) {
					for (const int du : {-1, 1}) {
						const auto [sx, sz] = world(u + du, v);
						if (!free(sx, sz))
							continue;
						const BlockWithProperties seat{OAK_STAIRS,
								{{"facing", ladder_facing({du * tangent.first,
													du * tangent.second})},
										{"half", "bottom"}, {"shape", "straight"}}};
						editor.set_block_with_properties_absolute(
								seat, sx, base_y, sz, std::nullopt, std::nullopt);
					}
				}
				break;
			case 1: {
				const int hx = x + normal.first, hz = z + normal.second;
				if (!free(x, z) || !free(hx, hz))
					break;
				const char *facing = ladder_facing(normal);
				for (const auto &[cell, part] : {std::pair{std::pair{x, z}, "foot"},
							 std::pair{std::pair{hx, hz}, "head"}}) {
					const BlockWithProperties bed{RED_BED_NORTH_HEAD,
							{{"facing", facing}, {"part", part}, {"occupied", "false"}}};
					editor.set_block_with_properties_absolute(bed, cell.first, base_y,
							cell.second, std::nullopt, std::nullopt);
					editor.set_bed_block_entity_absolute(cell.first, base_y, cell.second);
				}
				break;
			}
			default:
				for (const auto &[du, dv] : {std::pair{0, 0}, std::pair{1, 0},
							 std::pair{0, 1}, std::pair{1, 1}}) {
					const auto [cx, cz] = world(u + du, v + dv);
					put(cx, cz, 1, RED_CARPET);
				}
				break;
			}
		}
		return;
	}
	const bool tall = ceiling_y - floor_y >= 3;
	for (const auto &[cell, inward] : walls) {
		const auto h = interior_mix(cell.first, cell.second, seed);
		if (unit.use.goods == Goods::Clothes && h % 9 == 0) {
			const BlockWithProperties loom{LOOM, {{"facing", ladder_facing(inward)}}};
			editor.set_block_with_properties_absolute(
					loom, cell.first, base_y, cell.second, std::nullopt, std::nullopt);
			continue;
		}
		const bool books = unit.use.goods == Goods::Books;
		if (books) {
			if (!put_bookshelf(cell.first, cell.second, 1, inward, h))
				continue;
		} else if (!put(cell.first, cell.second, 1, stock[h % stock.size()])) {
			continue;
		}
		if (tall && free(cell.first, cell.second, 2) &&
				!window_behind(cell.first, cell.second, inward)) {
			if (books)
				put_bookshelf(cell.first, cell.second, 2, inward, h >> 16);
			else
				put(cell.first, cell.second, 2, stock[(h >> 16) % stock.size()]);
		} else if (!wares.empty() && h % 3 == 0)
			put(cell.first, cell.second, 2, wares[(h >> 24) % wares.size()]);
	}
	// Interior display fixtures. The footprint-coordinate cadence is stable across
	// adjacent map tiles and leaves a broad center aisle through each shop.
	for (const auto &[x, z] : cells) {
		if (!free(x, z))
			continue;
		const auto depth = depth_at.find(zone_key(x, z));
		if (depth == depth_at.end() || depth->second < 3)
			continue;
		const auto [u, v] = local(x, z);
		if ((u - entry_u + 8 * width) % 4 != 2 || v % 6 == 0)
			continue;
		const auto h = interior_mix(x, z, seed ^ 0x5A0EULL);
		if (unit.use.goods == Goods::Books) {
			put_top_slab(x, z, 1, OAK_SLAB);
			if (h % 2 == 0)
				put(x, z, 2, LECTERN);
		} else {
			put(x, z, 1, stock[h % stock.size()]);
			if (!wares.empty() && h % 2 == 0)
				put(x, z, 2, wares[(h >> 8) % wares.size()]);
		}
	}
	for (const auto &[cell, inward] : walls) {
		(void)inward;
		if (local(cell.first, cell.second).second < depth - 2 ||
				interior_mix(cell.first, cell.second, seed ^ 0xC4E5ULL) % 11 != 0 ||
				!free(cell.first, cell.second))
			continue;
		std::vector<std::tuple<std::string, int, int>> items;
		for (const auto &item : buildings_loot::chest_loot(cell.first, cell.second, salt))
			items.emplace_back(item.id, item.slot, item.count);
		editor.set_chest_with_items_absolute(cell.first, base_y, cell.second, items);
	}
}

void furnish_supermarket(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &cells,
		const std::vector<interior_uses::Entry> &entries,
		const std::vector<std::pair<int, int>> &door_positions,
		const std::unordered_set<std::uint64_t> &structural,
		std::unordered_set<std::uint64_t> &kept,
		const std::unordered_map<std::uint64_t, double> &claimed_top,
		const CoordinateBitmap &passages, double floor_metres, int passage_top,
		int floor_y, int base_y, int ceiling_y, std::uint64_t seed)
{
	if (cells.empty())
		return;
	auto key = [](int x, int z) { return interior_cell_key(x, z); };
	std::unordered_set<std::uint64_t> zone;
	int min_x = cells.front().first, max_x = min_x;
	int min_z = cells.front().second, max_z = min_z;
	for (const auto &[x, z] : cells) {
		zone.insert(key(x, z));
		min_x = std::min(min_x, x);
		max_x = std::max(max_x, x);
		min_z = std::min(min_z, z);
		max_z = std::max(max_z, z);
	}
	std::optional<interior_uses::Entry> entry;
	for (const auto &candidate : entries)
		if (zone.contains(key(candidate.cell.first, candidate.cell.second))) {
			entry = candidate;
			break;
		}
	if (!entry) {
		for (const auto &[dx, dz] : door_positions) {
			for (const auto d : INTERIOR_DIRS) {
				const int x = dx + d.first, z = dz + d.second;
				if (zone.contains(key(x, z))) {
					entry = interior_uses::Entry{{x, z}, d};
					break;
				}
			}
			if (entry)
				break;
		}
	}
	const auto n =
			entry ? entry->inward
				  : (max_x - min_x >= max_z - min_z ? std::pair{1, 0} : std::pair{0, 1});
	const std::pair<int, int> t{-n.second, n.first};
	const int ox = n.first + t.first > 0 ? min_x : max_x;
	const int oz = n.second + t.second > 0 ? min_z : max_z;
	const int width = t.first != 0 ? max_x - min_x + 1 : max_z - min_z + 1;
	const int depth = n.first != 0 ? max_x - min_x + 1 : max_z - min_z + 1;
	auto world = [&](int u, int v) {
		return std::pair{
				ox + t.first * u + n.first * v, oz + t.second * u + n.second * v};
	};
	auto local = [&](int x, int z) {
		const int dx = x - ox, dz = z - oz;
		return std::pair{dx * t.first + dz * t.second, dx * n.first + dz * n.second};
	};
	const int entry_u =
			entry ? local(entry->cell.first, entry->cell.second).first : width / 2;
	auto free = [&](int x, int z, int dy = 1) {
		const auto cell = key(x, z);
		if (!zone.contains(cell) || structural.contains(cell) || kept.contains(cell))
			return false;
		const auto top = claimed_top.find(cell);
		if (top != claimed_top.end() && top->second > floor_metres + 1.0)
			return false;
		if (passages.contains(x, z) && floor_y < passage_top)
			return false;
		return !editor.get_block_absolute(x, base_y + dy - 1, z);
	};
	auto put = [&](int x, int z, int dy, const Block &block) {
		if (!free(x, z, dy))
			return false;
		editor.set_block_absolute(
				block, x, base_y + dy - 1, z, std::nullopt, std::nullopt);
		return true;
	};
	auto window_behind = [&](int x, int z, std::pair<int, int> inward) {
		const auto name = editor.block_name_absolute(
				x - inward.first, floor_y + 2, z - inward.second);
		return name && (name->find("glass") != std::string::npos ||
							   name->find("pane") != std::string::npos);
	};
	auto put_top_slab = [&](int x, int z, int dy, const Block &block) {
		if (!free(x, z, dy))
			return false;
		const BlockWithProperties slab{block, {{"type", "top"}}};
		editor.set_block_with_properties_absolute(
				slab, x, base_y + dy - 1, z, std::nullopt, std::nullopt);
		return true;
	};
	const bool tall = ceiling_y - floor_y >= 3;
	for (const int side : {1, -1}) {
		for (int lane = 0; lane < 3; ++lane) {
			const int u = entry_u + side * (2 + lane * 3);
			std::array<std::pair<int, int>, 3> checkout;
			bool fits = true;
			for (int i = 0; i < 3; ++i) {
				checkout[i] = world(u, i + 2);
				fits &= free(checkout[i].first, checkout[i].second);
			}
			if (!fits)
				break;
			for (int i = 0; i < 3; ++i) {
				put_top_slab(checkout[i].first, checkout[i].second, 1, SMOOTH_STONE_SLAB);
				if (i == 1)
					put(checkout[i].first, checkout[i].second, 2, DAYLIGHT_DETECTOR);
			}
			const auto [sx, sz] = world(u + side, 3);
			const BlockWithProperties seat{OAK_STAIRS,
					{{"facing", ladder_facing({-side * t.first, -side * t.second})},
							{"half", "bottom"}, {"shape", "straight"}}};
			if (free(sx, sz))
				editor.set_block_with_properties_absolute(
						seat, sx, base_y, sz, std::nullopt, std::nullopt);
		}
	}
	for (const auto &[x, z] : cells) {
		if (!free(x, z))
			continue;
		const auto [u, v] = local(x, z);
		if (v < 6)
			continue;
		bool wall = false;
		std::pair<int, int> inward{};
		for (const auto d : INTERIOR_DIRS) {
			const int nx = x + d.first, nz = z + d.second;
			if (!zone.contains(key(nx, nz)) || structural.contains(key(nx, nz)) ||
					editor.get_block_absolute(nx, base_y, nz)) {
				wall = true;
				inward = {-d.first, -d.second};
				break;
			}
		}
		if (!wall)
			continue;
		const auto h = interior_mix(x, z, seed);
		if (inward == n) {
			put(x, z, 1, WHITE_CONCRETE);
			if (tall)
				put(x, z, 2, LIGHT_BLUE_STAINED_GLASS);
		} else {
			const std::array<Block, 4> lower = {BARREL, HAY_BALE, BARREL, PUMPKIN};
			put(x, z, 1, lower[h % lower.size()]);
			if (tall && !window_behind(x, z, inward) &&
					!editor.get_block_absolute(x, base_y + 1, z)) {
				const std::array<Block, 3> upper = {BARREL, MELON, BARREL};
				put(x, z, 2, upper[(h >> 8) % upper.size()]);
			}
		}
	}
	for (const auto &[x, z] : cells) {
		if (!free(x, z))
			continue;
		const auto [u, v] = local(x, z);
		const int lane = floor_mod(u - entry_u, 4);
		const auto h = interior_mix(x, z, seed ^ 0xA15EULL);
		if (v == 7 && lane == 2) {
			const std::array<Block, 4> produce = {HAY_BALE, MELON, PUMPKIN, COMPOSTER};
			put(x, z, 1, produce[h % produce.size()]);
		} else if (v >= 9 && (lane == 2 || lane == 3) && v % 10 != 0) {
			const std::array<Block, 4> racks = {
					BARREL, BARREL, HAY_BALE, CHISELLED_BOOKSHELF_NORTH};
			put(x, z, 1, racks[h % racks.size()]);
			if (tall && !editor.get_block_absolute(x, base_y + 1, z)) {
				const std::array<Block, 4> upper = {BARREL, MELON, PUMPKIN, BARREL};
				put(x, z, 2, upper[(h >> 8) % upper.size()]);
			}
		}
	}
}

void furnish_eatery(WorldEditor &editor, const interior_uses::Unit &unit,
		const std::vector<std::pair<int, int>> &cells,
		const std::vector<interior_uses::Entry> &entries,
		const std::vector<std::pair<int, int>> &door_positions,
		const std::unordered_set<std::uint64_t> &structural,
		std::unordered_set<std::uint64_t> &kept,
		const std::unordered_map<std::uint64_t, double> &claimed_top,
		const CoordinateBitmap &passages, double floor_metres, int passage_top,
		int floor_y, int base_y, int ceiling_y, std::uint64_t seed)
{
	if (cells.empty())
		return;
	auto key = [](int x, int z) { return interior_cell_key(x, z); };
	std::unordered_set<std::uint64_t> zone;
	int min_x = cells.front().first, max_x = min_x;
	int min_z = cells.front().second, max_z = min_z;
	for (const auto &[x, z] : cells) {
		zone.insert(key(x, z));
		min_x = std::min(min_x, x);
		max_x = std::max(max_x, x);
		min_z = std::min(min_z, z);
		max_z = std::max(max_z, z);
	}
	std::optional<interior_uses::Entry> entry;
	for (const auto &candidate : entries)
		if (zone.contains(key(candidate.cell.first, candidate.cell.second))) {
			entry = candidate;
			break;
		}
	if (!entry) {
		for (const auto &[dx, dz] : door_positions) {
			for (const auto d : INTERIOR_DIRS) {
				const int x = dx + d.first, z = dz + d.second;
				if (zone.contains(key(x, z))) {
					entry = interior_uses::Entry{{x, z}, d};
					break;
				}
			}
			if (entry)
				break;
		}
	}
	const auto n =
			entry ? entry->inward
				  : (max_x - min_x >= max_z - min_z ? std::pair{1, 0} : std::pair{0, 1});
	const std::pair<int, int> t{-n.second, n.first};
	const int ox = n.first + t.first > 0 ? min_x : max_x;
	const int oz = n.second + t.second > 0 ? min_z : max_z;
	const int width = t.first != 0 ? max_x - min_x + 1 : max_z - min_z + 1;
	const int depth = n.first != 0 ? max_x - min_x + 1 : max_z - min_z + 1;
	auto world = [&](int u, int v) {
		return std::pair{
				ox + t.first * u + n.first * v, oz + t.second * u + n.second * v};
	};
	auto local = [&](int x, int z) {
		const int dx = x - ox, dz = z - oz;
		return std::pair{dx * t.first + dz * t.second, dx * n.first + dz * n.second};
	};
	const int entry_u =
			entry ? local(entry->cell.first, entry->cell.second).first : width / 2;
	auto free = [&](int x, int z, int dy = 1) {
		const auto cell = key(x, z);
		if (!zone.contains(cell) || structural.contains(cell) || kept.contains(cell))
			return false;
		const auto top = claimed_top.find(cell);
		if (top != claimed_top.end() && top->second > floor_metres + 1.0)
			return false;
		if (passages.contains(x, z) && floor_y < passage_top)
			return false;
		return !editor.get_block_absolute(x, base_y + dy - 1, z);
	};
	auto put = [&](int x, int z, int dy, const Block &block) {
		if (!free(x, z, dy))
			return false;
		editor.set_block_absolute(
				block, x, base_y + dy - 1, z, std::nullopt, std::nullopt);
		return true;
	};
	auto put_with = [&](int x, int z, int dy, const BlockWithProperties &block) {
		if (!free(x, z, dy))
			return false;
		editor.set_block_with_properties_absolute(
				block, x, base_y + dy - 1, z, std::nullopt, std::nullopt);
		return true;
	};
	std::unordered_map<std::uint64_t, int> distance;
	std::deque<std::pair<int, int>> queue;
	for (const auto &[x, z] : cells) {
		if (structural.contains(key(x, z)) || editor.get_block_absolute(x, base_y, z))
			continue;
		if (std::any_of(INTERIOR_DIRS.begin(), INTERIOR_DIRS.end(), [&](const auto &d) {
				return !zone.contains(key(x + d.first, z + d.second)) ||
					   structural.contains(key(x + d.first, z + d.second));
			})) {
			distance.emplace(key(x, z), 1);
			queue.emplace_back(x, z);
		}
	}
	while (!queue.empty()) {
		const auto [x, z] = queue.front();
		queue.pop_front();
		const int next = distance.at(key(x, z)) + 1;
		for (const auto d : INTERIOR_DIRS) {
			const int nx = x + d.first, nz = z + d.second;
			const auto cell = key(nx, nz);
			if (zone.contains(cell) && !structural.contains(cell) &&
					!distance.contains(cell) &&
					!editor.get_block_absolute(nx, base_y, nz)) {
				distance.emplace(cell, next);
				queue.emplace_back(nx, nz);
			}
		}
	}
	const auto kind = unit.use.eatery;
	const int mid = width / 2;
	const bool has_kitchen = depth >= 13 && cells.size() >= 90;
	const int service_v = std::max(2, has_kitchen ? depth - 7 : depth - 3);
	if (has_kitchen) {
		for (const auto &[x, z] : cells) {
			const auto [u, v] = local(x, z);
			if (v == depth - 2 && free(x, z)) {
				const auto h = interior_mix(x, z, seed ^ 0x4C17ULL);
				const char *facing = ladder_facing({-n.first, -n.second});
				if (h % 5 == 0 || h % 5 == 1) {
					const BlockWithProperties appliance{
							h % 5 == 0 ? SMOKER : FURNACE, {{"facing", facing}}};
					put_with(x, z, 1, appliance);
				} else if (h % 5 == 2) {
					put(x, z, 1, WATER_CAULDRON);
				} else if (h % 5 == 3) {
					put(x, z, 1, CRAFTING_TABLE);
				} else {
					put(x, z, 1, BARREL);
				}
			} else if (v == depth - 4 && distance.contains(key(x, z)) &&
					   distance.at(key(x, z)) >= 3 && interior_mix(x, z, seed) % 2 == 0) {
				const BlockWithProperties prep{SMOOTH_STONE_SLAB, {{"type", "top"}}};
				put_with(x, z, 1, prep);
			}
			(void)u;
		}
	}
	const int half = std::clamp(width / 4, 1, 4);
	const Block counter = kind == interior_uses::Eatery::Bar		? SPRUCE_PLANKS
						  : kind == interior_uses::Eatery::FastFood ? POLISHED_ANDESITE
																	: OAK_PLANKS;
	for (int u = mid - half; u <= mid + half; ++u) {
		const auto [x, z] = world(u, service_v);
		if (!put(x, z, 1, counter))
			continue;
		const auto h = interior_mix(x, z, seed);
		Block item = EMPTY_FLOWER_POT;
		if (kind == interior_uses::Eatery::Cafe)
			item = std::array<Block, 3>{CAKE, EMPTY_FLOWER_POT, BREWING_STAND}[h % 3];
		else if (kind == interior_uses::Eatery::Bar)
			item = std::array<Block, 3>{BREWING_STAND, EMPTY_FLOWER_POT, LANTERN}[h % 3];
		else
			item = std::array<Block, 3>{EMPTY_FLOWER_POT, LANTERN, CAKE}[h % 3];
		if (h % 3 == 0)
			put(x, z, 2, item);
		const auto [bx, bz] = world(u, service_v + 1);
		kept.insert(key(bx, bz));
	}
	if (kind == interior_uses::Eatery::Bar) {
		for (int u = mid - half; u <= mid + half; ++u) {
			const auto [x, z] = world(u, service_v - 1);
			if (u % 2 == 0) {
				const BlockWithProperties seat{
						OAK_STAIRS, {{"facing", ladder_facing({-n.first, -n.second})},
											{"half", "bottom"}, {"shape", "straight"}}};
				put_with(x, z, 1, seat);
			}
		}
		for (const auto &[x, z] : cells) {
			if (local(x, z).second <= service_v || !free(x, z))
				continue;
			if (std::any_of(
						INTERIOR_DIRS.begin(), INTERIOR_DIRS.end(), [&](const auto &d) {
							return !zone.contains(key(x + d.first, z + d.second));
						}))
				put(x, z, 1, BARREL);
		}
	}
	std::vector<std::pair<int, int>> tables;
	for (const auto &[x, z] : cells) {
		const auto [u, v] = local(x, z);
		const auto d = distance.find(key(x, z));
		if (v < 2 || v > service_v - 2 || d == distance.end() || d->second < 2 ||
				floor_mod(u - entry_u, 5) != 2)
			continue;
		const bool target = kind == interior_uses::Eatery::Cafe
									? v % 3 == 2
									: (kind == interior_uses::Eatery::Restaurant ||
											  kind == interior_uses::Eatery::Bar ||
											  kind == interior_uses::Eatery::FastFood) &&
											  v % 4 == 2;
		if (!target || !free(x, z))
			continue;
		const bool long_table = kind == interior_uses::Eatery::Restaurant ||
								kind == interior_uses::Eatery::Bar ||
								kind == interior_uses::Eatery::FastFood;
		if (long_table && kind != interior_uses::Eatery::Restaurant) {
			const auto [x2, z2] = world(u, v + 1);
			if (!free(x2, z2))
				continue;
		}
		const BlockWithProperties table{OAK_SLAB, {{"type", "top"}}};
		if (!put_with(x, z, 1, table))
			continue;
		auto add_seats = [&](int tu, int tv, bool end_seat) {
			for (const int du : {-1, 1}) {
				const auto [sx, sz] = world(tu + du, tv);
				const BlockWithProperties seat{OAK_STAIRS,
						{{"facing", ladder_facing({-du * t.first, -du * t.second})},
								{"half", "bottom"}, {"shape", "straight"}}};
				put_with(sx, sz, 1, seat);
			}
			if (end_seat) {
				const auto [sx, sz] = world(tu, tv + 1);
				const BlockWithProperties seat{
						OAK_STAIRS, {{"facing", ladder_facing({-n.first, -n.second})},
											{"half", "bottom"}, {"shape", "straight"}}};
				put_with(sx, sz, 1, seat);
			}
		};
		add_seats(u, v, kind == interior_uses::Eatery::Restaurant);
		if (long_table && kind != interior_uses::Eatery::Restaurant) {
			const auto [x2, z2] = world(u, v + 1);
			if (put_with(x2, z2, 1, table))
				add_seats(u, v + 1, false);
		}
		tables.emplace_back(x, z);
	}
	for (const int u : {0, width - 1}) {
		const auto [x, z] = world(u, 1);
		const auto h = interior_mix(x, z, seed ^ static_cast<unsigned>(u + 1));
		put(x, z, 1, h % 2 == 0 ? AZALEA : FLOWERING_AZALEA);
	}
	for (const auto &[x, z] : tables) {
		if (ceiling_y - floor_y >= 4) {
			const BlockWithProperties lantern{LANTERN, {{"hanging", "true"}}};
			put_with(x, z, ceiling_y - base_y, lantern);
		} else if (kind == interior_uses::Eatery::Restaurant) {
			put(x, z, 2, LANTERN);
		}
	}
}

void furnish_hall(WorldEditor &editor, const interior_uses::Unit &unit,
		const std::vector<std::pair<int, int>> &cells,
		const std::vector<interior_uses::Entry> &entries,
		const std::vector<std::pair<int, int>> &door_positions,
		const std::unordered_set<std::uint64_t> &structural,
		std::unordered_set<std::uint64_t> &kept,
		const std::unordered_map<std::uint64_t, double> &claimed_top,
		const CoordinateBitmap &passages, double floor_metres, int passage_top,
		int floor_y, int base_y, int ceiling_y, std::uint64_t seed, unsigned loot_salt)
{
	if (cells.empty())
		return;
	auto key = [](int x, int z) { return interior_cell_key(x, z); };
	std::unordered_set<std::uint64_t> zone;
	int min_x = cells.front().first, max_x = min_x;
	int min_z = cells.front().second, max_z = min_z;
	for (const auto &[x, z] : cells) {
		zone.insert(key(x, z));
		min_x = std::min(min_x, x);
		max_x = std::max(max_x, x);
		min_z = std::min(min_z, z);
		max_z = std::max(max_z, z);
	}
	std::optional<interior_uses::Entry> entry;
	for (const auto &candidate : entries)
		if (zone.contains(key(candidate.cell.first, candidate.cell.second))) {
			entry = candidate;
			break;
		}
	if (!entry) {
		for (const auto &[dx, dz] : door_positions) {
			for (const auto d : INTERIOR_DIRS) {
				const int x = dx + d.first, z = dz + d.second;
				if (zone.contains(key(x, z))) {
					entry = interior_uses::Entry{{x, z}, d};
					break;
				}
			}
			if (entry)
				break;
		}
	}
	std::pair<int, int> n =
			max_x - min_x >= max_z - min_z ? std::pair{1, 0} : std::pair{0, 1};
	if (entry) {
		const std::pair<int, int> t0{-n.second, n.first};
		const int ox0 = n.first + t0.first > 0 ? min_x : max_x;
		const int oz0 = n.second + t0.second > 0 ? min_z : max_z;
		const int v = (entry->cell.first - ox0) * n.first +
					  (entry->cell.second - oz0) * n.second;
		const int depth0 = n.first != 0 ? max_x - min_x + 1 : max_z - min_z + 1;
		if (v > depth0 / 2)
			n = {-n.first, -n.second};
	}
	const std::pair<int, int> t{-n.second, n.first};
	const int ox = n.first + t.first > 0 ? min_x : max_x;
	const int oz = n.second + t.second > 0 ? min_z : max_z;
	const int width = t.first != 0 ? max_x - min_x + 1 : max_z - min_z + 1;
	const int depth = n.first != 0 ? max_x - min_x + 1 : max_z - min_z + 1;
	auto world = [&](int u, int v) {
		return std::pair{
				ox + t.first * u + n.first * v, oz + t.second * u + n.second * v};
	};
	auto local = [&](int x, int z) {
		const int dx = x - ox, dz = z - oz;
		return std::pair{dx * t.first + dz * t.second, dx * n.first + dz * n.second};
	};
	const int entry_u =
			entry ? local(entry->cell.first, entry->cell.second).first : width / 2;
	auto free = [&](int x, int z, int dy = 1) {
		const auto cell = key(x, z);
		if (!zone.contains(cell) || structural.contains(cell) || kept.contains(cell))
			return false;
		const auto top = claimed_top.find(cell);
		if (top != claimed_top.end() && top->second > floor_metres + 1.0)
			return false;
		if (passages.contains(x, z) && floor_y < passage_top)
			return false;
		return !editor.get_block_absolute(x, base_y + dy - 1, z);
	};
	auto put = [&](int x, int z, int dy, const Block &block) {
		if (!free(x, z, dy))
			return false;
		editor.set_block_absolute(
				block, x, base_y + dy - 1, z, std::nullopt, std::nullopt);
		return true;
	};
	auto put_with = [&](int x, int z, int dy, const BlockWithProperties &block) {
		if (!free(x, z, dy))
			return false;
		editor.set_block_with_properties_absolute(
				block, x, base_y + dy - 1, z, std::nullopt, std::nullopt);
		return true;
	};
	auto mount = [&](int x, int z, int dy, std::pair<int, int> inward,
						 const Block &block) {
		const int bx = x - inward.first, bz = z - inward.second;
		const bool backed =
				!zone.contains(key(bx, bz)) || structural.contains(key(bx, bz)) ||
				editor.get_block_absolute(bx, base_y + dy - 1, bz).has_value();
		return backed && put(x, z, dy, block);
	};
	auto mount_with = [&](int x, int z, int dy, std::pair<int, int> inward,
							  const BlockWithProperties &block) {
		const int bx = x - inward.first, bz = z - inward.second;
		const bool backed =
				!zone.contains(key(bx, bz)) || structural.contains(key(bx, bz)) ||
				editor.get_block_absolute(bx, base_y + dy - 1, bz).has_value();
		return backed && put_with(x, z, dy, block);
	};
	auto stack = [&](int x, int z, std::initializer_list<Block> blocks) {
		int dy = 1;
		for (const auto &block : blocks) {
			if (!put(x, z, dy++, block))
				break;
		}
	};
	auto wall_cells = [&]() {
		std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> result;
		for (const auto &[x, z] : cells) {
			if (!free(x, z))
				continue;
			for (const auto d : INTERIOR_DIRS) {
				const int nx = x + d.first, nz = z + d.second;
				if (!zone.contains(key(nx, nz)) || structural.contains(key(nx, nz)) ||
						editor.get_block_absolute(nx, base_y, nz)) {
					result.emplace_back(std::pair{x, z}, std::pair{-d.first, -d.second});
					break;
				}
			}
		}
		return result;
	};
	const auto kind = unit.use.kind;
	const int headroom = ceiling_y - floor_y;
	const int mid = width / 2;
	if (kind == interior_uses::UseKind::Worship && width >= 3 && depth >= 6) {
		const int back = depth - 1;
		const int altar_v = back - 2;
		const auto faith = unit.use.faith;
		if (faith == interior_uses::Faith::Muslim) {
			const auto [mx, mz] = world(mid, back);
			for (int dy = 2; dy <= std::min(headroom, 4); ++dy)
				mount(mx, mz, dy, {-n.first, -n.second}, CHISELED_QUARTZ_BLOCK);
			int i = 0;
			for (int v = back - 3; v < back; ++v, ++i) {
				const auto [x, z] = world(mid + 2, v);
				if (i == 2) {
					put(x, z, 1, QUARTZ_BLOCK);
				} else {
					const BlockWithProperties stair{QUARTZ_STAIRS,
							{{"facing", ladder_facing({-n.first, -n.second})},
									{"half", "bottom"}, {"shape", "straight"}}};
					put_with(x, z, 1, stair);
				}
			}
			for (const auto &[x, z] : cells) {
				const int v = local(x, z).second;
				if (v >= 2 && free(x, z))
					put(x, z, 1, (v / 2) % 2 == 0 ? GREEN_CARPET : RED_CARPET);
			}
			return;
		}
		if (faith == interior_uses::Faith::Jewish) {
			for (int du = -1; du <= 1; ++du) {
				const auto [x, z] = world(mid + du, back);
				stack(x, z, {DARK_OAK_PLANKS, DARK_OAK_PLANKS});
			}
			const auto [lx, lz] = world(mid, back);
			put(lx, lz, 3, LANTERN);
			const auto [dx, dz] = world(mid, altar_v);
			put_with(dx, dz, 1,
					BlockWithProperties{LECTERN, {{"facing", ladder_facing(n)}}});
		} else {
			for (int du = -1; du <= 1; ++du) {
				const auto [x, z] = world(mid + du, altar_v);
				const Block altar = du == 0 ? CHISELED_QUARTZ_BLOCK : QUARTZ_BLOCK;
				if (put(x, z, 1, altar) && du != 0)
					put(x, z, 2, LANTERN);
			}
			if (width >= 7) {
				const auto [x, z] = world(mid + 3, altar_v - 1);
				put_with(x, z, 1,
						BlockWithProperties{LECTERN,
								{{"facing", ladder_facing({-n.first, -n.second})}}});
			}
		}
		if (faith == interior_uses::Faith::Christian && headroom >= 5) {
			const auto [x, z] = world(mid, back);
			for (int dy = 3; dy <= 5; ++dy)
				mount(x, z, dy, {-n.first, -n.second}, GOLD_BLOCK);
			for (const int du : {-1, 1}) {
				const auto [cx, cz] = world(mid + du, back);
				mount(cx, cz, 4, {-n.first, -n.second}, GOLD_BLOCK);
			}
		} else if (faith == interior_uses::Faith::Other) {
			const auto [x, z] = world(mid, back);
			stack(x, z, {CHISELED_QUARTZ_BLOCK, GOLD_BLOCK});
		}
		for (const auto &[x, z] : cells) {
			const auto [u, v] = local(x, z);
			if (!free(x, z) || v >= altar_v - 1)
				continue;
			const bool aisle = u == mid || (width >= 9 && u == mid - 1);
			if (aisle) {
				put(x, z, 1, RED_CARPET);
			} else if ((faith == interior_uses::Faith::Christian ||
							   faith == interior_uses::Faith::Jewish) &&
					   v >= 2 && v < altar_v - 2 && v % 2 == 0 && u > 0 &&
					   u < width - 1) {
				put_with(x, z, 1,
						BlockWithProperties{OAK_STAIRS,
								{{"facing", ladder_facing({-n.first, -n.second})},
										{"half", "bottom"}, {"shape", "straight"}}});
			} else if (faith == interior_uses::Faith::Other && v >= 2 && v % 2 == 0) {
				put(x, z, 1, RED_CARPET);
			}
		}
		return;
	}
	if (kind == interior_uses::UseKind::Warehouse) {
		const int rack_height = std::clamp(headroom - 1, 1, 4);
		const std::array<Block, 5> goods = {BARREL, BARREL, HAY_BALE, OAK_PLANKS, BARREL};
		for (const auto &[x, z] : cells) {
			const auto [u, v] = local(x, z);
			const int lane = floor_mod(u - entry_u, 5);
			if (v < 4 || lane != 2 && lane != 3 || v % 9 == 0 || !free(x, z))
				continue;
			const auto h = interior_mix(x, z, seed);
			if (h % 29 == 0) {
				std::vector<std::tuple<std::string, int, int>> items;
				for (const auto &item : buildings_loot::chest_loot(x, z, loot_salt))
					items.emplace_back(item.id, item.slot, item.count);
				editor.set_chest_with_items_absolute(x, base_y, z, items);
				continue;
			}
			for (int dy = 1; dy <= rack_height; ++dy)
				put(x, z, dy, goods[(h >> (8 * (dy - 1))) % goods.size()]);
		}
		for (const auto &[cell, inward] : wall_cells()) {
			const auto [x, z] = cell;
			const auto h = interior_mix(x, z, seed ^ 0x9AULL);
			if (local(x, z).second >= 4 && h % 3 == 0) {
				put(x, z, 1, BARREL);
				put(x, z, 2, h & 1 ? HAY_BALE : BARREL);
			}
			(void)inward;
		}
		return;
	}
	if (kind == interior_uses::UseKind::Barn) {
		for (int v = 0; v < depth; ++v)
			for (int u : {mid - 1, mid}) {
				const auto [x, z] = world(u, v);
				kept.insert(key(x, z));
			}
		for (const auto &[x, z] : cells) {
			const auto [u, v] = local(x, z);
			if (v % 4 == 0 && v > 0 && v < depth - 1 && (u < mid - 2 || u > mid + 1) &&
					free(x, z))
				put(x, z, 1, OAK_FENCE);
		}
		for (const auto &[cell, inward] : wall_cells()) {
			const auto [x, z] = cell;
			const int v = local(x, z).second;
			const auto h = interior_mix(x, z, seed);
			if (v == depth - 1) {
				const int count = std::clamp(headroom - 1, 1, 3);
				for (int dy = 1; dy <= count; ++dy)
					put(x, z, dy, HAY_BALE);
			} else if (v % 4 == 2) {
				const std::array<Block, 3> feed = {HAY_BALE, WATER_CAULDRON, COMPOSTER};
				put(x, z, 1, feed[h % feed.size()]);
			}
			(void)inward;
		}
		for (const auto &[cell, inward] : wall_cells()) {
			const auto [x, z] = cell;
			if (interior_mix(x, z, seed ^ 0xBAULL) % 17 == 0) {
				std::vector<std::tuple<std::string, int, int>> items;
				for (const auto &item : buildings_loot::chest_loot(x, z, loot_salt))
					items.emplace_back(item.id, item.slot, item.count);
				editor.set_chest_with_items_absolute(x, base_y, z, items);
			}
			(void)inward;
		}
		return;
	}
	if (kind == interior_uses::UseKind::Factory) {
		const Block rail = n.first != 0 ? RAIL_EAST_WEST : RAIL_NORTH_SOUTH;
		for (int v = 2; v < depth - 2; ++v) {
			const auto [x, z] = world(mid, v);
			put(x, z, 1, rail);
		}
		for (int v = 3; v < depth - 3; v += 4) {
			for (int side : {-2, 2}) {
				const auto [x, z] = world(mid + side, v);
				const auto h = interior_mix(x, z, seed);
				const std::pair<int, int> facing{-side * t.first, -side * t.second};
				switch (h % 5) {
				case 0:
					put_with(x, z, 1,
							BlockWithProperties{
									BLAST_FURNACE, {{"facing", ladder_facing(facing)}}});
					break;
				case 1:
					put_with(x, z, 1,
							BlockWithProperties{
									FURNACE, {{"facing", ladder_facing(facing)}}});
					break;
				case 2:
					stack(x, z, {IRON_BLOCK, HOPPER});
					break;
				case 3:
					put(x, z, 1, WATER_CAULDRON);
					break;
				default:
					put(x, z, 1, DISPENSER);
					break;
				}
			}
		}
		for (const auto &[cell, inward] : wall_cells()) {
			const auto [x, z] = cell;
			const auto h = interior_mix(x, z, seed ^ 0xFAULL);
			if (local(x, z).second < 2 || h % 2 == 0)
				continue;
			switch ((h >> 8) % 6) {
			case 0:
				put(x, z, 1, CRAFTING_TABLE);
				break;
			case 1:
				put(x, z, 1, SMITHING_TABLE);
				break;
			case 2:
				put_with(x, z, 1,
						BlockWithProperties{GRINDSTONE,
								{{"face", "floor"}, {"facing", ladder_facing(inward)}}});
				break;
			case 3:
				put(x, z, 1, ANVIL);
				break;
			case 4: {
				std::vector<std::tuple<std::string, int, int>> items;
				for (const auto &item : buildings_loot::chest_loot(x, z, loot_salt))
					items.emplace_back(item.id, item.slot, item.count);
				editor.set_chest_with_items_absolute(x, base_y, z, items);
				break;
			}
			default:
				put(x, z, 1, BARREL);
				break;
			}
		}
		return;
	}
	if (kind == interior_uses::UseKind::SportsHall) {
		// Rust Canvas::floor replaces the hall's walkable floor cells with the
		// selected wood planks before painting court markings and seating.
		for (const auto &[x, z] : cells)
			editor.set_block_absolute(
					OAK_PLANKS, x, base_y - 1, z, std::nullopt, std::nullopt);
		const bool stands = width >= 16;
		if (stands) {
			for (int v = 2; v < depth - 2; ++v) {
				const auto [x1, z1] = world(width - 1, v);
				if (put(x1, z1, 1, OAK_PLANKS)) {
					const BlockWithProperties stair{OAK_STAIRS,
							{{"facing", ladder_facing({-t.first, -t.second})},
									{"half", "bottom"}, {"shape", "straight"}}};
					put_with(x1, z1, 2, stair);
				}
				const auto [x2, z2] = world(width - 2, v);
				const BlockWithProperties seat{
						OAK_STAIRS, {{"facing", ladder_facing({-t.first, -t.second})},
											{"half", "bottom"}, {"shape", "straight"}}};
				put_with(x2, z2, 1, seat);
			}
		}
		const int high_u = stands ? width - 5 : width - 3;
		for (const auto &[x, z] : cells) {
			const auto [u, v] = local(x, z);
			if (free(x, z) && u >= 2 && u <= high_u && v >= 2 && v <= depth - 3 &&
					(u == 2 || u == high_u || v == 2 || v == depth - 3 || v == depth / 2))
				put(x, z, 1, WHITE_CARPET);
		}
		const int goal_u = (2 + high_u) / 2;
		if (headroom >= 3)
			for (int v : {3, depth - 4})
				for (int du = -1; du <= 1; ++du) {
					const auto [x, z] = world(goal_u + du, v);
					if (du != 0) {
						put(x, z, 1, WHITE_CONCRETE);
						put(x, z, 2, WHITE_CONCRETE);
					}
					put(x, z, 3, WHITE_CONCRETE);
				}
		for (int v = depth / 2 - 3; v < depth / 2 + 3; ++v) {
			if (v == depth / 2)
				continue;
			const auto [x, z] = world(0, v);
			const BlockWithProperties bench{
					OAK_STAIRS, {{"facing", ladder_facing(t)}, {"half", "bottom"},
										{"shape", "straight"}}};
			put_with(x, z, 1, bench);
		}
		return;
	}
	if (kind == interior_uses::UseKind::Gym) {
		for (const auto &[cell, inward] : wall_cells()) {
			const auto [x, z] = cell;
			if (inward == std::pair{-n.first, -n.second}) {
				mount_with(x, z, 2, inward,
						BlockWithProperties{LIGHT_GRAY_STAINED_GLASS,
								{{"facing", ladder_facing(inward)}}});
				if (interior_mix(x, z, seed) % 2 == 0)
					put(x, z, 1, ANVIL);
			}
		}
		for (const auto &[x, z] : cells) {
			const auto [u, v] = local(x, z);
			if ((u - entry_u + 8 * width) % 3 != 1 || v < 3 || v % 3 != 0)
				continue;
			switch (interior_mix(x, z, seed ^ 0x6EULL) % 4) {
			case 0:
				put(x, z, 1, ANVIL);
				break;
			case 1:
				put_with(x, z, 1,
						BlockWithProperties{GRINDSTONE,
								{{"face", "floor"}, {"facing", ladder_facing({-n.first,
																	   -n.second})}}});
				break;
			case 2:
				put_with(x, z, 1,
						BlockWithProperties{POLISHED_BLACKSTONE_SLAB, {{"type", "top"}}});
				break;
			default:
				stack(x, z, {IRON_BLOCK, IRON_BARS});
				break;
			}
		}
		for (const auto &[x, z] : cells) {
			const int v = local(x, z).second;
			if (free(x, z) && v >= 2 && v < 5)
				put(x, z, 1, LIGHT_GRAY_CARPET);
		}
		const auto [x, z] = world(0, 1);
		put(x, z, 1, WATER_CAULDRON);
		return;
	}
	if (kind == interior_uses::UseKind::Auditorium && depth >= 8) {
		const int stage_v = depth - 4;
		for (const auto &[x, z] : cells) {
			const auto [u, v] = local(x, z);
			if (v >= stage_v && u > 0 && u < width - 1 && v < depth - 1)
				put(x, z, 1, OAK_PLANKS);
		}
		for (int u : {1, width - 2}) {
			const auto [x, z] = world(u, stage_v);
			for (int dy = 2; dy <= headroom; ++dy)
				put(x, z, dy, RED_WOOL);
		}
		for (int u = 2; u < width - 2; ++u) {
			const auto [x, z] = world(u, depth - 1);
			for (int dy = 3; dy <= std::min(headroom, 7); ++dy)
				mount(x, z, dy, {-n.first, -n.second}, WHITE_WOOL);
		}
		const int mid_u = width / 2;
		for (const auto &[x, z] : cells) {
			const auto [u, v] = local(x, z);
			if (u <= 1 || u >= width - 2 || (width >= 12 && u == mid_u) || v < 2 ||
					v >= stage_v - 2 || v % 2 != 0)
				continue;
			put_with(x, z, 1,
					BlockWithProperties{RED_NETHER_BRICK_STAIRS,
							{{"facing", ladder_facing({-n.first, -n.second})},
									{"half", "bottom"}, {"shape", "straight"}}});
		}
	}
}

void split_civic_corridor(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &cells,
		const std::vector<interior_uses::Entry> &entries,
		const std::vector<std::pair<int, int>> &existing_doors,
		std::unordered_set<std::uint64_t> &structural,
		std::unordered_set<std::uint64_t> &planned,
		const std::unordered_set<std::uint64_t> &kept,
		std::vector<std::pair<int, int>> &wall_positions,
		std::vector<std::pair<int, int>> &door_positions,
		const std::unordered_map<std::uint64_t, double> &claimed_top,
		const CoordinateBitmap &passages, double floor_metres, int passage_top,
		int floor_y, int base_y, int ceiling_y, Block wall_block,
		interior_uses::UseKind kind)
{
	int room_len = 0, room_depth = 0;
	if (kind == interior_uses::UseKind::School) {
		room_len = 8;
		room_depth = 6;
	} else if (kind == interior_uses::UseKind::Clinic) {
		room_len = 5;
		room_depth = 4;
	} else if (kind == interior_uses::UseKind::Hospital) {
		room_len = 7;
		room_depth = 5;
	} else {
		return;
	}
	if (cells.empty())
		return;
	auto key = [](int x, int z) { return interior_cell_key(x, z); };
	std::unordered_set<std::uint64_t> zone;
	int min_x = cells.front().first, max_x = min_x;
	int min_z = cells.front().second, max_z = min_z;
	for (const auto &[x, z] : cells) {
		zone.insert(key(x, z));
		min_x = std::min(min_x, x);
		max_x = std::max(max_x, x);
		min_z = std::min(min_z, z);
		max_z = std::max(max_z, z);
	}
	const bool along_x = max_x - min_x >= max_z - min_z;
	const int a0 = along_x ? min_x : min_z;
	const int a1 = along_x ? max_x : max_z;
	const int c0 = along_x ? min_z : min_x;
	const int c1 = along_x ? max_z : max_x;
	const int span = c1 - c0 + 1;
	std::pair<int, int> hall;
	std::vector<std::pair<int, int>> sides;
	if (span >= 2 * (room_depth + 1) + 2) {
		const int h0 = c0 + (span - 2) / 2;
		hall = {h0, h0 + 1};
		sides = {{h0 - 1, 1}, {h0 + 2, -1}};
	} else if (span >= room_depth + 3) {
		hall = {c0, c0 + 1};
		sides = {{c0 + 2, -1}};
	} else {
		return;
	}
	const int length = a1 - a0 + 1;
	const std::size_t count =
			static_cast<std::size_t>(std::max((length + 1) / (room_len + 1), 1));
	std::vector<int> starts;
	starts.reserve(count + 1);
	for (std::size_t k = 0; k <= count; ++k)
		starts.push_back(
				a0 +
				static_cast<int>((k * static_cast<std::size_t>(length + 1)) / count));
	auto side_of = [&](int c) -> std::optional<std::size_t> {
		if (c >= hall.first && c <= hall.second)
			return std::nullopt;
		if (sides.size() == 1 || c < hall.first)
			return 0;
		return 1;
	};
	std::optional<interior_uses::Entry> entry;
	for (const auto &candidate : entries)
		if (zone.contains(key(candidate.cell.first, candidate.cell.second))) {
			entry = candidate;
			break;
		}
	if (!entry) {
		for (const auto &[dx, dz] : existing_doors) {
			for (const auto d : INTERIOR_DIRS) {
				const int x = dx + d.first, z = dz + d.second;
				if (zone.contains(key(x, z))) {
					entry = interior_uses::Entry{{x, z}, d};
					break;
				}
			}
			if (entry)
				break;
		}
	}
	std::optional<std::pair<std::size_t, std::size_t>> lobby;
	if (entry) {
		const int a = along_x ? entry->cell.first : entry->cell.second;
		const int c = along_x ? entry->cell.second : entry->cell.first;
		if (const auto side = side_of(c)) {
			for (std::size_t k = 0; k < count; ++k)
				if (a >= starts[k] && a < starts[k + 1]) {
					lobby = std::pair{*side, k};
					break;
				}
		}
	}
	std::set<std::pair<int, int>> walls;
	for (const auto &[x, z] : cells) {
		const int a = along_x ? x : z;
		const int c = along_x ? z : x;
		const auto side = side_of(c);
		if (!side)
			continue;
		const int wall_row = sides[*side].first;
		const std::size_t k = static_cast<std::size_t>(std::distance(
				starts.begin(), std::upper_bound(starts.begin(), starts.end(), a) - 1));
		const bool in_lobby = lobby == std::pair{*side, k};
		if (c == wall_row) {
			if (!in_lobby)
				walls.emplace(x, z);
			continue;
		}
		if (in_lobby) {
			if (k + 1 < count && a == starts[k + 1] - 1)
				walls.emplace(x, z);
			continue;
		}
		if (k + 1 < count && a == starts[k + 1] - 1)
			walls.emplace(x, z);
	}
	struct PlannedDoor
	{
		std::pair<int, int> wall;
		std::pair<int, int> into_room;
	};
	std::vector<PlannedDoor> doors;
	for (std::size_t side = 0; side < sides.size(); ++side) {
		const int wall_row = sides[side].first;
		const int toward_hall = sides[side].second;
		const std::pair<int, int> d =
				along_x ? std::pair{0, toward_hall} : std::pair{toward_hall, 0};
		for (std::size_t k = 0; k < count; ++k) {
			if (lobby == std::pair{side, k})
				continue;
			const int lo = starts[k], hi = starts[k + 1] - 2;
			if (hi < lo)
				continue;
			const int mid = (lo + hi) / 2;
			for (int offset = 0; offset <= hi - lo; ++offset) {
				const int a = offset % 2 == 0 ? mid + offset / 2 : mid - offset / 2 - 1;
				if (a < lo || a > hi)
					continue;
				const std::pair<int, int> w =
						along_x ? std::pair{a, wall_row} : std::pair{wall_row, a};
				const auto room_cell = std::pair{w.first - d.first, w.second - d.second};
				const auto hall_cell = std::pair{w.first + d.first, w.second + d.second};
				const auto to_ac = [&](const std::pair<int, int> &cell) {
					return along_x ? std::pair{cell.first, cell.second}
								   : std::pair{cell.second, cell.first};
				};
				const auto [ra, rc] = to_ac(room_cell);
				const auto [ha, hc] = to_ac(hall_cell);
				const bool room_side =
						zone.contains(key(room_cell.first, room_cell.second)) &&
						ra >= starts[k] && ra < starts[k + 1] - 1 &&
						(side_of(rc) == side);
				const bool hall_side =
						zone.contains(key(hall_cell.first, hall_cell.second)) &&
						ha >= starts[k] && ha < starts[k + 1] && ha >= a0 &&
						(hc >= hall.first && hc <= hall.second);
				const auto door_key = key(w.first, w.second);
				const auto blocked = [&](int x, int z) {
					const auto k = key(x, z);
					const auto claim = claimed_top.find(k);
					return !kept.contains(k) &&
						   (editor.get_block_absolute(x, base_y, z) ||
								   (claim != claimed_top.end() &&
										   claim->second > floor_metres + 1.0) ||
								   (passages.contains(x, z) && floor_y < passage_top));
				};
				if (!walls.contains(w) || !room_side || !hall_side ||
						kept.contains(door_key) ||
						blocked(room_cell.first, room_cell.second) ||
						blocked(hall_cell.first, hall_cell.second))
					continue;
				doors.push_back({w, {-d.first, -d.second}});
				break;
			}
		}
	}
	const int top_y = base_y + (ceiling_y - floor_y) - 1;
	for (const auto &w : walls) {
		const auto cell_key = key(w.first, w.second);
		planned.insert(cell_key);
		structural.insert(cell_key);
		if (kept.contains(cell_key) ||
				editor.get_block_absolute(w.first, base_y, w.second))
			continue;
		const auto door = std::find_if(
				doors.begin(), doors.end(), [&](const auto &d) { return d.wall == w; });
		if (door != doors.end()) {
			const char *facing = ladder_facing(door->into_room);
			const BlockWithProperties lower{DARK_OAK_DOOR_LOWER,
					{{"half", "lower"}, {"facing", facing}, {"hinge", "left"}}};
			const BlockWithProperties upper{DARK_OAK_DOOR_UPPER,
					{{"half", "upper"}, {"facing", facing}, {"hinge", "left"}}};
			editor.set_block_with_properties_absolute(
					lower, w.first, base_y, w.second, std::nullopt, std::nullopt);
			if (top_y > base_y)
				editor.set_block_with_properties_absolute(
						upper, w.first, base_y + 1, w.second, std::nullopt, std::nullopt);
			door_positions.push_back(w);
			continue;
		}
		wall_positions.push_back(w);
		for (int y = base_y; y <= top_y; ++y)
			editor.set_block_absolute(
					wall_block, w.first, y, w.second, std::nullopt, std::nullopt);
	}
}

void furnish_civic(WorldEditor &editor, const interior_uses::Unit &unit,
		const std::vector<std::pair<int, int>> &cells,
		const std::vector<interior_uses::Entry> &entries,
		std::vector<std::pair<int, int>> &door_positions,
		std::unordered_set<std::uint64_t> &structural,
		const std::unordered_set<std::uint64_t> &kept,
		const std::unordered_map<std::uint64_t, double> &claimed_top,
		const CoordinateBitmap &passages, double floor_metres, int passage_top,
		int floor_y, int base_y, int ceiling_y, std::uint64_t seed, unsigned loot_salt,
		std::size_t floor_index, std::size_t floor_count, Block wall_block,
		std::unordered_set<std::uint64_t> &planned,
		std::vector<std::pair<int, int>> &wall_positions)
{
	if (cells.empty())
		return;
	auto key = [](int x, int z) { return interior_cell_key(x, z); };
	std::unordered_set<std::uint64_t> zone;
	int min_x = cells.front().first, max_x = min_x;
	int min_z = cells.front().second, max_z = min_z;
	for (const auto &[x, z] : cells) {
		zone.insert(key(x, z));
		min_x = std::min(min_x, x);
		max_x = std::max(max_x, x);
		min_z = std::min(min_z, z);
		max_z = std::max(max_z, z);
	}
	const auto existing_doors = door_positions;
	split_civic_corridor(editor, cells, entries, existing_doors, structural, planned,
			kept, wall_positions, door_positions, claimed_top, passages, floor_metres,
			passage_top, floor_y, base_y, ceiling_y, wall_block, unit.use.kind);
	std::optional<interior_uses::Entry> entry;
	for (const auto &candidate : entries)
		if (zone.contains(key(candidate.cell.first, candidate.cell.second))) {
			entry = candidate;
			break;
		}
	if (!entry) {
		for (const auto &[dx, dz] : door_positions) {
			for (const auto d : INTERIOR_DIRS) {
				const int x = dx + d.first, z = dz + d.second;
				if (zone.contains(key(x, z))) {
					entry = interior_uses::Entry{{x, z}, d};
					break;
				}
			}
			if (entry)
				break;
		}
	}
	const auto kind = unit.use.kind;
	std::pair<int, int> n =
			entry ? entry->inward
				  : (max_x - min_x >= max_z - min_z ? std::pair{1, 0} : std::pair{0, 1});
	if (kind == interior_uses::UseKind::School && entry)
		n = {-entry->inward.second, entry->inward.first};
	if (kind == interior_uses::UseKind::Station ||
			kind == interior_uses::UseKind::Museum ||
			kind == interior_uses::UseKind::Office) {
		const std::pair<int, int> axis =
				max_x - min_x >= max_z - min_z ? std::pair{1, 0} : std::pair{0, 1};
		if (kind == interior_uses::UseKind::Station) {
			n = axis;
			if (entry) {
				const std::pair<int, int> at{-axis.second, axis.first};
				const int ax = axis.first + at.first > 0 ? min_x : max_x;
				const int az = axis.second + at.second > 0 ? min_z : max_z;
				const int ev = (entry->cell.first - ax) * axis.first +
							   (entry->cell.second - az) * axis.second;
				const int axis_depth =
						axis.first != 0 ? max_x - min_x + 1 : max_z - min_z + 1;
				if (ev > axis_depth / 2)
					n = {-axis.first, -axis.second};
			}
		} else if (!entry)
			n = axis;
	}
	const std::pair<int, int> t{-n.second, n.first};
	const int ox = n.first + t.first > 0 ? min_x : max_x;
	const int oz = n.second + t.second > 0 ? min_z : max_z;
	const int width = t.first != 0 ? max_x - min_x + 1 : max_z - min_z + 1;
	const int depth = n.first != 0 ? max_x - min_x + 1 : max_z - min_z + 1;
	auto world = [&](int u, int v) {
		return std::pair{
				ox + t.first * u + n.first * v, oz + t.second * u + n.second * v};
	};
	auto local = [&](int x, int z) {
		const int dx = x - ox, dz = z - oz;
		return std::pair{dx * t.first + dz * t.second, dx * n.first + dz * n.second};
	};
	const int entry_u =
			entry ? local(entry->cell.first, entry->cell.second).first : width / 2;
	auto free = [&](int x, int z, int dy = 1) {
		const auto cell = key(x, z);
		if (!zone.contains(cell) || structural.contains(cell) || kept.contains(cell))
			return false;
		const auto top = claimed_top.find(cell);
		if (top != claimed_top.end() && top->second > floor_metres + 1.0)
			return false;
		if (passages.contains(x, z) && floor_y < passage_top)
			return false;
		return !editor.get_block_absolute(x, base_y + dy - 1, z);
	};
	auto put = [&](int x, int z, int dy, const Block &block) {
		if (!free(x, z, dy))
			return false;
		editor.set_block_absolute(
				block, x, base_y + dy - 1, z, std::nullopt, std::nullopt);
		return true;
	};
	auto put_with = [&](int x, int z, int dy, const BlockWithProperties &block) {
		if (!free(x, z, dy))
			return false;
		editor.set_block_with_properties_absolute(
				block, x, base_y + dy - 1, z, std::nullopt, std::nullopt);
		return true;
	};
	auto mount = [&](int x, int z, int dy, std::pair<int, int> inward,
						 const Block &block) {
		const int bx = x - inward.first, bz = z - inward.second;
		const auto behind = key(bx, bz);
		const bool backed =
				!zone.contains(behind) || structural.contains(behind) ||
				editor.get_block_absolute(bx, base_y + dy - 1, bz).has_value();
		return backed && put(x, z, dy, block);
	};
	auto window_behind = [&](int x, int z, std::pair<int, int> inward) {
		const auto name = editor.block_name_absolute(
				x - inward.first, floor_y + 2, z - inward.second);
		return name && (name->find("glass") != std::string::npos ||
							   name->find("pane") != std::string::npos);
	};
	auto top_slab = [&](int x, int z, int dy, const Block &slab) {
		return put_with(x, z, dy, BlockWithProperties{slab, {{"type", "top"}}});
	};
	auto seat = [&](int x, int z, std::pair<int, int> facing) {
		return put_with(x, z, 1,
				BlockWithProperties{
						OAK_STAIRS, {{"facing", ladder_facing(facing)},
											{"half", "bottom"}, {"shape", "straight"}}});
	};
	auto bed = [&](int foot_x, int foot_z, std::pair<int, int> toward_head) {
		const int head_x = foot_x + toward_head.first;
		const int head_z = foot_z + toward_head.second;
		if (!free(foot_x, foot_z) || !free(head_x, head_z))
			return false;
		const char *facing = ladder_facing(toward_head);
		for (const auto &[cell, part] : {std::pair{std::pair{foot_x, foot_z}, "foot"},
					 std::pair{std::pair{head_x, head_z}, "head"}}) {
			const BlockWithProperties state{WHITE_BED,
					{{"facing", facing}, {"part", part}, {"occupied", "false"}}};
			editor.set_block_with_properties_absolute(
					state, cell.first, base_y, cell.second, std::nullopt, std::nullopt);
			editor.set_bed_block_entity_absolute(cell.first, base_y, cell.second);
		}
		return true;
	};
	auto shelf = [&](int x, int z, int dy, std::pair<int, int> facing, std::uint64_t h) {
		static constexpr bool slots[4][6] = {{true, true, false, true, true, true},
				{true, false, true, true, true, false},
				{false, true, true, true, false, true},
				{true, true, true, false, true, true}};
		const auto &fill = slots[h % 4];
		return put_with(x, z, dy,
				BlockWithProperties{CHISELLED_BOOKSHELF_NORTH,
						{{"facing", ladder_facing(facing)},
								{"slot_0_occupied", fill[0] ? "true" : "false"},
								{"slot_1_occupied", fill[1] ? "true" : "false"},
								{"slot_2_occupied", fill[2] ? "true" : "false"},
								{"slot_3_occupied", fill[3] ? "true" : "false"},
								{"slot_4_occupied", fill[4] ? "true" : "false"},
								{"slot_5_occupied", fill[5] ? "true" : "false"}}});
	};
	std::unordered_map<std::uint64_t, int> depth_at;
	std::deque<std::pair<int, int>> queue;
	for (const auto &[x, z] : cells) {
		if (structural.contains(key(x, z)) || editor.get_block_absolute(x, base_y, z))
			continue;
		if (std::any_of(INTERIOR_DIRS.begin(), INTERIOR_DIRS.end(), [&](const auto &d) {
				return !zone.contains(key(x + d.first, z + d.second)) ||
					   structural.contains(key(x + d.first, z + d.second));
			})) {
			depth_at.emplace(key(x, z), 1);
			queue.emplace_back(x, z);
		}
	}
	while (!queue.empty()) {
		const auto [x, z] = queue.front();
		queue.pop_front();
		const int next = depth_at.at(key(x, z)) + 1;
		for (const auto d : INTERIOR_DIRS) {
			const int nx = x + d.first, nz = z + d.second;
			const auto k = key(nx, nz);
			if (zone.contains(k) && !structural.contains(k) && !depth_at.contains(k) &&
					!editor.get_block_absolute(nx, base_y, nz)) {
				depth_at.emplace(k, next);
				queue.emplace_back(nx, nz);
			}
		}
	}
	auto walls = [&]() {
		std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> out;
		for (const auto &[x, z] : cells) {
			if (!free(x, z))
				continue;
			for (const auto d : INTERIOR_DIRS) {
				const int nx = x + d.first, nz = z + d.second;
				if (!zone.contains(key(nx, nz)) || structural.contains(key(nx, nz)) ||
						editor.get_block_absolute(nx, base_y, nz)) {
					out.emplace_back(std::pair{x, z}, std::pair{-d.first, -d.second});
					break;
				}
			}
		}
		return out;
	};
	const int headroom = ceiling_y - floor_y;
	const int mid = width / 2;
	if (kind == interior_uses::UseKind::Bank) {
		const int row = std::clamp(depth / 3, 4, 6);
		std::vector<int> tellers;
		for (int u = 1; u < width - 2; ++u) {
			const auto [x, z] = world(u, row);
			if (put(x, z, 1, POLISHED_ANDESITE)) {
				if (headroom >= 3)
					put(x, z, 2, IRON_BARS);
				if (u % 3 == 1)
					tellers.push_back(u);
			}
		}
		for (const int u : tellers) {
			const auto [x, z] = world(u, row + 1);
			seat(x, z, {-n.first, -n.second});
		}
		for (const auto &[cell, inward] : walls()) {
			const auto [x, z] = cell;
			const int v = local(x, z).second;
			if (v >= 1 && v < row - 1 && inward != n) {
				if (interior_mix(x, z, seed) % 3 != 0)
					seat(x, z, inward);
			} else if (v > row + 2) {
				if (interior_mix(x, z, seed) % 3 == 0) {
					std::vector<std::tuple<std::string, int, int>> items;
					for (const auto &item : buildings_loot::chest_loot(x, z, loot_salt))
						items.emplace_back(item.id, item.slot, item.count);
					editor.set_chest_with_items_absolute(x, base_y, z, items);
				} else {
					put(x, z, 1, BARREL);
				}
			}
		}
		const auto [px, pz] = world(0, 1);
		put(px, pz, 1, AZALEA);
		return;
	}
	if (kind == interior_uses::UseKind::Workshop) {
		for (const auto &[cell, inward] : walls()) {
			const auto [x, z] = cell;
			if (local(x, z).second < 2)
				continue;
			const auto h = interior_mix(x, z, seed ^ 0x3077ULL);
			switch (h % 7) {
			case 0:
				put(x, z, 1, CRAFTING_TABLE);
				break;
			case 1:
				put(x, z, 1, SMITHING_TABLE);
				break;
			case 2:
				put(x, z, 1, ANVIL);
				break;
			case 3:
				put_with(x, z, 1,
						BlockWithProperties{GRINDSTONE,
								{{"face", "floor"}, {"facing", ladder_facing(inward)}}});
				break;
			case 4:
				put(x, z, 1, BARREL);
				break;
			case 5: {
				std::vector<std::tuple<std::string, int, int>> items;
				for (const auto &item : buildings_loot::chest_loot(x, z, loot_salt))
					items.emplace_back(item.id, item.slot, item.count);
				editor.set_chest_with_items_absolute(x, base_y, z, items);
				break;
			}
			default:
				put_with(x, z, 1,
						BlockWithProperties{
								FURNACE, {{"facing", ladder_facing(inward)}}});
				break;
			}
		}
		for (const auto &[x, z] : cells) {
			const auto [u, v] = local(x, z);
			const auto d = depth_at.find(key(x, z));
			if (d != depth_at.end() && d->second >= 3 && v % 6 == 3 &&
					(u % 4 == 1 || u % 4 == 3))
				put(x, z, 1, IRON_BLOCK);
		}
		return;
	}
	if (kind == interior_uses::UseKind::School) {
		for (int u = 1; u < width - 1; ++u) {
			const auto [x, z] = world(u, 0);
			mount(x, z, 2, {n.first, n.second}, GREEN_CONCRETE);
			if (headroom >= 4)
				mount(x, z, 3, {n.first, n.second}, GREEN_CONCRETE);
		}
		const auto [dx, dz] = world(mid, 2);
		top_slab(dx, dz, 1, OAK_SLAB);
		const auto [sx, sz] = world(mid, 1);
		seat(sx, sz, {-n.first, -n.second});
		const auto [lx, lz] = world(mid + 2, 2);
		put_with(lx, lz, 1,
				BlockWithProperties{
						LECTERN, {{"facing", ladder_facing({-n.first, -n.second})}}});
		for (int v = 4; v + 1 < depth - 1; v += 2) {
			for (int u = 1; u < width - 1; ++u) {
				if (u % 3 == 0)
					continue;
				const auto [x, z] = world(u, v);
				const auto [cx, cz] = world(u, v + 1);
				if (free(x, z) && free(cx, cz)) {
					top_slab(x, z, 1, OAK_SLAB);
					seat(cx, cz, {n.first, n.second});
				}
			}
		}
		for (const auto &[cell, inward] : walls()) {
			const auto [x, z] = cell;
			if (local(x, z).second == depth - 1 && interior_mix(x, z, seed) % 2 == 0)
				shelf(x, z, 1, inward, seed);
		}
		return;
	}
	if (kind == interior_uses::UseKind::Library) {
		const int side = entry_u + 3 < width ? 1 : -1;
		for (int v = 1; v <= 2; ++v) {
			const auto [x, z] = world(entry_u + 2 * side, v);
			put(x, z, 1, OAK_PLANKS);
		}
		for (const auto &[cell, inward] : walls()) {
			const auto [x, z] = cell;
			const auto h = interior_mix(x, z, seed);
			if (shelf(x, z, 1, inward, h) && headroom >= 3 &&
					!window_behind(x, z, inward))
				put(x, z, 2, BOOKSHELF);
		}
		const int stacks_from = std::max(depth / 3, 5);
		for (const auto &[x, z] : cells) {
			const auto depth_it = depth_at.find(key(x, z));
			if (depth_it == depth_at.end() || depth_it->second < 3 || !free(x, z))
				continue;
			const auto [u, v] = local(x, z);
			if (v < stacks_from) {
				if (v >= 3 && floor_mod(u - entry_u, 4) == 2 && v % 3 == 0) {
					if (top_slab(x, z, 1, OAK_SLAB))
						put(x, z, 2, LANTERN);
				}
			} else if ((v - stacks_from) % 3 == 0 && std::abs(u - entry_u) > 1 &&
					   u % 7 != 0) {
				put(x, z, 1, BOOKSHELF);
				if (headroom >= 3)
					put(x, z, 2, BOOKSHELF);
			}
		}
		return;
	}
	if (kind == interior_uses::UseKind::Kindergarten) {
		for (const auto &[x, z] : cells) {
			const auto [u, v] = local(x, z);
			const auto d = depth_at.find(key(x, z));
			if (d == depth_at.end() || d->second < 3 || u % 5 != 2 || v % 5 != 3)
				continue;
			if (!top_slab(x, z, 1, OAK_SLAB))
				continue;
			for (const auto &[du, dv] : {std::pair{-1, 0}, {1, 0}, {0, -1}, {0, 1}}) {
				const auto [sx, sz] = world(u + du, v + dv);
				seat(sx, sz,
						{-du * t.first - dv * n.first, -du * t.second - dv * n.second});
			}
		}
		for (const auto &[cell, inward] : walls()) {
			const auto [x, z] = cell;
			const auto h = interior_mix(x, z, seed);
			const int v = local(x, z).second;
			if (v > depth / 2 && h % 3 == 0) {
				const int bx = x + inward.first, bz = z + inward.second;
				if (bed(bx, bz, {-inward.first, -inward.second})) {
					continue;
				}
			}
			switch (h % 5) {
			case 0: {
				std::vector<std::tuple<std::string, int, int>> items;
				for (const auto &item : buildings_loot::chest_loot(x, z, loot_salt))
					items.emplace_back(item.id, item.slot, item.count);
				editor.set_chest_with_items_absolute(x, base_y, z, items);
				break;
			}
			case 1:
				put(x, z, 1, NOTE_BLOCK);
				break;
			case 2: {
				const std::array<Block, 4> toys = {
						RED_WOOL, YELLOW_WOOL, BLUE_WOOL, GREEN_WOOL};
				put(x, z, 1, toys[(h >> 8) % toys.size()]);
				break;
			}
			default:
				break;
			}
		}
		for (const auto &[x, z] : cells) {
			if (!free(x, z))
				continue;
			const auto [u, v] = local(x, z);
			const std::array<Block, 4> carpets = {
					RED_CARPET, LIGHT_BLUE_CARPET, GREEN_CARPET, WHITE_CARPET};
			put(x, z, 1, carpets[floor_mod(u / 2 + v / 2, 4)]);
		}
		return;
	}
	if (kind == interior_uses::UseKind::Clinic ||
			kind == interior_uses::UseKind::Hospital) {
		for (int v = 1; v <= 2; ++v) {
			const auto [x, z] = world(entry_u + 2, v);
			put(x, z, 1, QUARTZ_BLOCK);
		}
		for (const auto &[cell, inward] : walls()) {
			const auto [x, z] = cell;
			const auto [u, v] = local(x, z);
			if (v >= 1 && std::abs(u - entry_u) > 3 && interior_mix(x, z, seed) % 3 != 0)
				seat(x, z, inward);
		}
		bool bed_placed = false;
		for (const auto &[cell, inward] : walls()) {
			const auto [x, z] = cell;
			const int bx = x + inward.first, bz = z + inward.second;
			if (!bed_placed)
				bed_placed = bed(bx, bz, {-inward.first, -inward.second});
		}
		const auto [bx, bz] = world(0, 1);
		put(bx, bz, 1, WATER_CAULDRON);
		return;
	}
	if (kind == interior_uses::UseKind::Museum) {
		const std::array<Block, 8> exhibits = {AMETHYST_CLUSTER, GOLD_BLOCK, LANTERN,
				BREWING_STAND, ANVIL, POTTED_BLUE_ORCHID, END_ROD, CHISELED_STONE_BRICKS};
		for (const auto &[x, z] : cells) {
			const auto [u, v] = local(x, z);
			const auto d = depth_at.find(key(x, z));
			if (d == depth_at.end() || d->second < 3 || floor_mod(u - entry_u, 4) != 2 ||
					v % 4 != 3)
				continue;
			const auto h = interior_mix(x, z, seed);
			const Block plinth = h % 2 == 0 ? CHISELED_QUARTZ_BLOCK : POLISHED_ANDESITE;
			if (put(x, z, 1, plinth))
				put(x, z, 2, exhibits[(h >> 8) % exhibits.size()]);
		}
		for (const auto &[cell, inward] : walls()) {
			const auto [x, z] = cell;
			const auto h = interior_mix(x, z, seed ^ 0x3EULL);
			if (local(x, z).second >= 2 && h % 3 == 0 && put(x, z, 1, SMOOTH_QUARTZ))
				put(x, z, 2, exhibits[(h >> 8) % exhibits.size()]);
			(void)inward;
		}
		for (int v = 1; v <= 2; ++v) {
			const auto [x, z] = world(entry_u + 2, v);
			put(x, z, 1, OAK_PLANKS);
		}
		return;
	}
	if (kind == interior_uses::UseKind::Station) {
		for (const auto &[cell, inward] : walls()) {
			const auto [x, z] = cell;
			const int v = local(x, z).second;
			if (inward == std::pair{-t.first, -t.second} && v >= 2 && v <= 6) {
				put(x, z, 1, POLISHED_ANDESITE);
				if (headroom >= 3)
					put(x, z, 2, IRON_BARS);
			}
		}
		const int board_y = std::min(headroom, 4);
		for (int du = -2; du <= 2; ++du) {
			const auto [x, z] = world(width / 2 + du, depth - 1);
			mount(x, z, board_y, {-n.first, -n.second}, BLACK_CONCRETE);
		}
		for (const auto &[x, z] : cells) {
			const auto [u, v] = local(x, z);
			const auto d = depth_at.find(key(x, z));
			if (d == depth_at.end() || d->second < 3 || floor_mod(u - entry_u, 5) < 1 ||
					floor_mod(u - entry_u, 5) > 3 || v < 4)
				continue;
			if (v % 5 == 1)
				seat(x, z, {t.first, t.second});
			else if (v % 5 == 2)
				seat(x, z, {-t.first, -t.second});
		}
		const auto [kx, kz] = world(width - 2, 1);
		if (put(kx, kz, 1, OAK_PLANKS))
			put(kx, kz, 2, CAKE);
		return;
	}
	if (kind == interior_uses::UseKind::Office) {
		for (const auto &[x, z] : cells) {
			const auto d = depth_at.find(key(x, z));
			const auto [u, v] = local(x, z);
			if (d == depth_at.end() || d->second < 2 || v < 3 ||
					(v % 4 != 1 && v % 4 != 2))
				continue;
			switch (floor_mod(u - entry_u, 6)) {
			case 2:
			case 3:
				top_slab(x, z, 1, OAK_SLAB);
				put(x, z, 2, GLASS_PANE);
				break;
			case 1:
				seat(x, z, {-t.first, -t.second});
				break;
			case 4:
				seat(x, z, {t.first, t.second});
				break;
			default:
				break;
			}
		}
		for (const auto &[cell, inward] : walls()) {
			const auto [x, z] = cell;
			const auto h = interior_mix(x, z, seed);
			if (local(x, z).second <= 3 && h % 7 == 0) {
				if (put(x, z, 1, WHITE_CONCRETE) && headroom >= 3)
					put(x, z, 2, LIGHT_BLUE_STAINED_GLASS);
				continue;
			}
			switch (h % 8) {
			case 0:
				shelf(x, z, 1, inward, h);
				break;
			case 1:
				put(x, z, 1, BARREL);
				break;
			case 2:
				put(x, z, 1, h & 0x100 ? AZALEA : FLOWERING_AZALEA);
				break;
			default:
				break;
			}
		}
	}
	if (kind == interior_uses::UseKind::Hotel) {
		if (floor_index == 0 && floor_count > 1) {
			for (int v = 2; v <= 4; ++v) {
				const auto [x, z] = world(entry_u + 3, v);
				put(x, z, 1, OAK_PLANKS);
			}
			for (const auto &[x, z] : cells) {
				const auto [u, v] = local(x, z);
				const auto d = depth_at.find(key(x, z));
				if (d != depth_at.end() && d->second >= 3 &&
						floor_mod(u - entry_u, 6) == 3 && v % 5 == 3 && v > 3 &&
						top_slab(x, z, 1, OAK_SLAB)) {
					for (const int du : {-1, 1}) {
						const auto [sx, sz] = world(u + du, v);
						seat(sx, sz, {-du * t.first, -du * t.second});
					}
				}
			}
			for (const auto &[cell, inward] : walls()) {
				const auto [x, z] = cell;
				if (interior_mix(x, z, seed) % 7 == 0)
					put(x, z, 1,
							interior_mix(x, z, 3) % 2 == 0 ? AZALEA : FLOWERING_AZALEA);
				(void)inward;
			}
			for (const auto &[x, z] : cells)
				if (free(x, z) && depth_at.contains(key(x, z)) &&
						depth_at.at(key(x, z)) >= 2)
					put(x, z, 1, RED_CARPET);
			return;
		}
		const int back = depth - 1;
		bool slept = false;
		for (const int du : {0, -1, 1, -2, 2}) {
			const auto [fx, fz] = world(mid + du, back - 1);
			if (!bed(fx, fz, n))
				continue;
			slept = true;
			for (const int side : {-1, 1}) {
				const auto [nx, nz] = world(mid + du + side, back);
				if (put(nx, nz, 1, OAK_PLANKS))
					put(nx, nz, 2, LANTERN);
			}
			break;
		}
		bool desk_placed = false;
		for (const auto &[cell, inward] : walls()) {
			const auto [x, z] = cell;
			const int v = local(x, z).second;
			if (v == 0)
				continue;
			if (!desk_placed && v >= 1 && v < back - 1 &&
					inward != std::pair{-n.first, -n.second}) {
				if (top_slab(x, z, 1, OAK_SLAB)) {
					const int sx = x + inward.first, sz = z + inward.second;
					seat(sx, sz, {-inward.first, -inward.second});
					desk_placed = true;
				}
			} else if (v == 1) {
				shelf(x, z, 1, inward, interior_mix(x, z, seed));
			}
		}
		if (slept)
			for (const auto &[x, z] : cells)
				if (free(x, z))
					put(x, z, 1, LIGHT_GRAY_CARPET);
	}
}
}

// INTERIOR1_LAYER1
static constexpr std::array<std::array<char, 23>, 23> INTERIOR1_LAYER1 = {{
		{'1', 'U', ' ', 'W', 'C', ' ', ' ', ' ', 'S', 'S', 'W', 'B', 'T', 'T', 'B', 'W',
				'7', '8', ' ', ' ', ' ', ' ', 'W'},
		{'2', ' ', ' ', 'W', 'F', ' ', ' ', ' ', 'U', 'U', 'W', 'B', 'T', 'T', 'B', 'W',
				'7', '8', ' ', ' ', ' ', 'B', 'W'},
		{' ', ' ', ' ', 'W', 'F', ' ', ' ', ' ', ' ', ' ', 'W', 'B', 'T', 'T', 'B', 'W',
				'W', 'W', 'D', 'W', 'W', 'W', 'W'},
		{'W', 'W', 'D', 'W', 'L', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'A', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', 'B', 'B', ' ', ' ', 'J', 'W',
				' ', ' ', ' ', 'B', 'W', 'W', 'W'},
		{'W', 'W', 'W', 'W', 'D', 'W', ' ', ' ', 'W', 'T', 'S', 'S', 'T', ' ', ' ', 'W',
				'S', 'S', ' ', 'B', 'W', 'W', 'W'},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', 'T', 'T', 'T', 'T', ' ', ' ', 'W',
				'U', 'U', ' ', 'B', 'W', ' ', ' '},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'D', 'T', 'T', 'T', 'T', ' ', 'B', 'W',
				' ', ' ', ' ', 'B', 'W', ' ', ' '},
		{'L', ' ', 'A', 'L', 'W', 'W', ' ', ' ', 'W', 'J', 'U', 'U', ' ', ' ', 'B', 'W',
				'W', 'D', 'W', 'W', 'W', ' ', ' '},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				' ', ' ', 'W', 'C', 'C', 'W', 'W'},
		{'B', 'B', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', 'W', ' ', ' ', 'W', 'W'},
		{' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'D'},
		{' ', '6', ' ', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'D', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'U', '5', ' ', 'W', ' ', ' ', 'W', 'C', 'F', 'F', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', 'W', 'L', ' ', 'W',
				'A', ' ', 'B', 'W', ' ', ' ', 'W'},
		{'B', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				' ', ' ', 'B', 'W', 'J', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W',
				'U', ' ', ' ', 'W', 'B', ' ', 'D'},
		{'J', ' ', ' ', 'C', 'B', 'B', 'W', 'L', 'F', ' ', 'W', 'F', ' ', 'W', 'L', 'W',
				'7', '8', ' ', 'W', 'B', ' ', 'W'},
		{'B', ' ', ' ', 'B', 'W', 'W', 'W', 'W', 'W', ' ', 'W', 'A', ' ', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', 'C', ' ', 'W'},
		{'B', ' ', ' ', 'B', 'W', ' ', ' ', ' ', 'D', ' ', 'W', 'C', ' ', ' ', 'W', 'W',
				'B', 'B', 'B', 'B', 'W', 'D', 'W'},
		{'W', 'W', 'D', 'W', 'C', ' ', ' ', ' ', 'W', 'W', 'W', 'B', 'T', 'T', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
}};

// INTERIOR1_LAYER2
static constexpr std::array<std::array<char, 23>, 23> INTERIOR1_LAYER2 = {{
		{' ', 'P', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'P', 'P', 'W', 'B', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', 'B', 'W'},
		{' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', 'B', 'W',
				'W', 'W', 'D', 'W', 'W', 'W', 'W'},
		{'W', 'W', 'D', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', 'B', 'B', ' ', ' ', ' ', 'W',
				' ', ' ', ' ', 'B', 'W', 'W', 'W'},
		{'W', 'W', 'W', 'W', 'D', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				' ', ' ', ' ', 'B', 'W', 'W', 'W'},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				'P', 'P', ' ', 'B', 'W', ' ', ' '},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', 'B', 'W', ' ', ' '},
		{' ', ' ', ' ', ' ', 'W', 'W', ' ', ' ', 'W', ' ', 'P', 'P', ' ', ' ', 'B', 'W',
				'W', 'D', 'W', 'W', 'W', ' ', ' '},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				' ', ' ', 'W', 'C', 'C', 'W', 'W'},
		{'B', 'B', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', 'W', ' ', ' ', 'W', 'W'},
		{' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'D'},
		{' ', ' ', ' ', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'D', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'P', ' ', ' ', 'W', ' ', ' ', 'W', 'N', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				' ', ' ', 'B', 'W', ' ', ' ', 'W'},
		{'B', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				' ', ' ', 'C', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W',
				'P', ' ', ' ', 'W', 'B', ' ', 'D'},
		{' ', ' ', ' ', ' ', 'B', 'B', 'W', ' ', ' ', ' ', 'W', ' ', ' ', 'W', 'P', 'W',
				' ', ' ', ' ', 'W', 'B', ' ', 'W'},
		{'B', ' ', ' ', 'B', 'W', 'W', 'W', 'W', 'W', ' ', 'W', ' ', ' ', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', ' ', ' ', 'W'},
		{'B', ' ', ' ', 'B', 'W', ' ', ' ', ' ', 'D', ' ', 'W', 'N', ' ', ' ', 'W', 'W',
				'B', 'B', 'B', 'B', 'W', 'D', 'W'},
		{'W', 'W', 'D', 'W', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'B', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
}};

// INTERIOR2_LAYER1
static constexpr std::array<std::array<char, 23>, 23> INTERIOR2_LAYER1 = {{
		{'W', 'W', 'W', 'D', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'D', 'W', 'W', 'W'},
		{'U', ' ', ' ', ' ', ' ', ' ', 'C', 'W', 'L', ' ', ' ', 'L', 'W', 'A', 'A', 'W',
				' ', ' ', ' ', ' ', ' ', 'L', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', 'S', 'S', 'S', ' ', 'W'},
		{' ', ' ', 'W', 'F', ' ', ' ', ' ', 'W', 'C', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				'J', ' ', 'U', 'U', 'U', ' ', 'D'},
		{'U', ' ', 'W', 'F', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'W', 'W', 'W', 'W', 'W', 'W'},
		{'U', ' ', 'W', 'F', ' ', ' ', ' ', 'D', ' ', ' ', 'T', 'T', 'W', ' ', ' ', ' ',
				' ', ' ', 'U', 'W', ' ', 'L', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', 'T', 'J', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				'W', ' ', ' ', 'W', 'L', ' ', 'W'},
		{'J', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'C', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'L', ' ', ' ', ' ', ' ', 'W', 'C', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', 'A', 'B', 'B', 'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'B', 'W', 'L', ' ', ' ', ' ', ' ', 'W', 'L', ' ', ' ', 'B', 'W',
				'W', 'B', 'B', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'B', 'W', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', 'D', ' ', ' ', 'U', ' ', ' ', ' ', 'D', ' ', ' ', 'F', 'F',
				'W', 'A', 'A', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', 'W', ' ', ' ', 'U', ' ', ' ', 'W', 'W', ' ', ' ', ' ', ' ',
				'C', ' ', ' ', 'W', ' ', ' ', 'W'},
		{'C', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', ' ', ' ',
				'L', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'L', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'L', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'U', 'U', ' ', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', 'W', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'U', 'U', ' ', 'W', 'B', ' ', 'U', 'U',
				'B', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'S', 'S', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', 'B', ' ', 'W'},
		{'U', 'U', ' ', ' ', ' ', 'L', 'B', 'B', 'B', ' ', ' ', 'W', 'B', 'B', 'B', 'B',
				'B', 'B', 'B', ' ', 'B', 'D', 'W'},
}};

// INTERIOR2_LAYER2
static constexpr std::array<std::array<char, 23>, 23> INTERIOR2_LAYER2 = {{
		{'W', 'W', 'W', 'D', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'D', 'W', 'W', 'W'},
		{'P', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'E', ' ', ' ', 'E', 'W', ' ', ' ', 'W',
				' ', ' ', ' ', ' ', ' ', 'E', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', 'W', 'F', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				' ', ' ', 'P', 'P', 'P', ' ', 'D'},
		{'P', ' ', 'W', 'F', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'W', 'W', 'W', 'W', 'W', 'W'},
		{'P', ' ', 'W', 'F', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', 'P', 'W', ' ', 'P', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'P', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'E', ' ', ' ', ' ', ' ', 'W', 'P', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', ' ', 'B', 'B', 'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'B', 'W', 'E', ' ', ' ', ' ', ' ', 'W', 'E', ' ', ' ', 'B', 'W',
				'W', 'B', 'B', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'B', 'W', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', 'D', ' ', ' ', 'P', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', 'W', ' ', ' ', 'P', ' ', ' ', 'W', 'W', ' ', ' ', ' ', ' ',
				' ', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', ' ', ' ',
				'E', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'E', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'E', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'P', 'P', ' ', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', 'W', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'P', 'P', ' ', 'W', 'B', ' ', 'P', 'P',
				'B', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', 'B', ' ', 'W'},
		{'P', 'P', ' ', ' ', ' ', 'E', 'B', 'B', 'B', ' ', ' ', 'W', 'B', 'B', 'B', 'B',
				'B', 'B', 'B', ' ', 'B', ' ', 'D'},
}};

// Generic Abandoned Building Interiors
// ABANDONED_INTERIOR1_LAYER1
static constexpr std::array<std::array<char, 23>, 23> ABANDONED_INTERIOR1_LAYER1 = {{
		{'1', 'U', ' ', 'W', 'C', ' ', ' ', ' ', 'S', 'S', 'W', 'b', 'T', 'T', 'd', 'W',
				'7', '8', ' ', ' ', ' ', ' ', 'W'},
		{'2', ' ', ' ', 'W', 'F', ' ', ' ', ' ', 'U', 'U', 'W', 'b', 'T', 'T', 'd', 'W',
				'7', '8', ' ', ' ', ' ', 'B', 'W'},
		{' ', ' ', ' ', 'W', 'F', ' ', ' ', ' ', ' ', ' ', 'W', 'b', 'T', 'T', 'd', 'W',
				'W', 'W', 'D', 'W', 'W', 'W', 'W'},
		{'W', 'W', 'D', 'W', 'L', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'M', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'c', 'c', 'c', ' ', ' ', 'J', 'W',
				' ', ' ', ' ', 'd', 'W', 'W', 'W'},
		{'W', 'W', 'W', 'W', 'D', 'W', ' ', ' ', 'W', 'T', 'S', 'S', 'T', ' ', ' ', 'W',
				'S', 'S', ' ', 'd', 'W', 'W', 'W'},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', 'T', 'T', 'T', 'T', ' ', ' ', 'W',
				'U', 'U', ' ', 'd', 'W', ' ', ' '},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'D', 'T', 'T', 'T', 'T', ' ', 'B', 'W',
				' ', ' ', ' ', 'd', 'W', ' ', ' '},
		{'L', ' ', 'M', 'L', 'W', 'W', ' ', ' ', 'W', 'J', 'U', 'U', ' ', ' ', 'B', 'W',
				'W', 'D', 'W', 'W', 'W', ' ', ' '},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				' ', ' ', 'W', 'C', 'C', 'W', 'W'},
		{'c', 'c', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', 'W', ' ', ' ', 'W', 'W'},
		{' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'D'},
		{' ', '6', ' ', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'D', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'U', '5', ' ', 'W', ' ', ' ', 'W', 'C', 'F', 'F', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', 'W', 'L', ' ', 'W',
				'M', ' ', 'b', 'W', ' ', ' ', 'W'},
		{'B', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				' ', ' ', 'b', 'W', 'J', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W',
				'U', ' ', ' ', 'W', 'B', ' ', 'D'},
		{'J', ' ', ' ', 'C', 'a', 'a', 'W', 'L', 'F', ' ', 'W', 'F', ' ', 'W', 'L', 'W',
				'7', '8', ' ', 'W', 'B', ' ', 'W'},
		{'B', ' ', ' ', 'd', 'W', 'W', 'W', 'W', 'W', ' ', 'W', 'M', ' ', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', 'C', ' ', 'W'},
		{'B', ' ', ' ', 'd', 'W', ' ', ' ', ' ', 'D', ' ', 'W', 'C', ' ', ' ', 'W', 'W',
				'c', 'c', 'c', 'c', 'W', 'D', 'W'},
		{'W', 'W', 'D', 'W', 'C', ' ', ' ', ' ', 'W', 'W', 'W', 'b', 'T', 'T', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
}};

// ABANDONED_INTERIOR1_LAYER2
static constexpr std::array<std::array<char, 23>, 23> ABANDONED_INTERIOR1_LAYER2 = {{
		{' ', 'P', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'P', 'P', 'W', 'B', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', 'B', 'W'},
		{' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', 'B', 'W',
				'W', 'W', 'D', 'W', 'W', 'W', 'W'},
		{'W', 'W', 'D', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', 'B', 'B', ' ', ' ', ' ', 'W',
				' ', ' ', ' ', 'B', 'W', 'W', 'W'},
		{'W', 'W', 'W', 'W', 'D', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				' ', ' ', ' ', 'B', 'W', 'W', 'W'},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				'P', 'P', ' ', 'B', 'W', ' ', ' '},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', 'B', 'W', ' ', ' '},
		{' ', ' ', ' ', ' ', 'W', 'W', ' ', ' ', 'W', ' ', 'P', 'P', ' ', ' ', 'B', 'W',
				'W', 'D', 'W', 'W', 'W', ' ', ' '},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				' ', ' ', 'W', 'C', 'C', 'W', 'W'},
		{'B', 'B', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', 'W', ' ', ' ', 'W', 'W'},
		{' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'D'},
		{' ', ' ', ' ', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'D', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'P', ' ', ' ', 'W', ' ', ' ', 'W', 'N', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				' ', ' ', 'B', 'W', ' ', ' ', 'W'},
		{'B', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				' ', ' ', 'C', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W',
				'P', ' ', ' ', 'W', 'B', ' ', 'D'},
		{' ', ' ', ' ', ' ', 'B', 'B', 'W', ' ', ' ', ' ', 'W', ' ', ' ', 'W', 'P', 'W',
				' ', ' ', ' ', 'W', 'B', ' ', 'W'},
		{'B', ' ', ' ', 'B', 'W', 'W', 'W', 'W', 'W', ' ', 'W', ' ', ' ', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', ' ', ' ', 'W'},
		{'B', ' ', ' ', 'B', 'W', ' ', ' ', ' ', 'D', ' ', 'W', 'N', ' ', ' ', 'W', 'W',
				'B', 'B', 'B', 'B', 'W', 'D', 'W'},
		{'W', 'W', 'D', 'W', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'B', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
}};

// ABANDONED_INTERIOR2_LAYER1
static constexpr std::array<std::array<char, 23>, 23> ABANDONED_INTERIOR2_LAYER1 = {{
		{'W', 'W', 'W', 'D', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'D', 'W', 'W', 'W'},
		{'U', ' ', ' ', ' ', ' ', ' ', 'C', 'W', 'L', ' ', ' ', 'L', 'W', 'M', 'M', 'W',
				' ', ' ', ' ', ' ', ' ', 'L', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'Q', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', 'S', 'S', 'S', ' ', 'W'},
		{' ', ' ', 'W', 'F', ' ', ' ', ' ', 'Q', 'C', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				'J', ' ', 'U', 'U', 'U', ' ', 'D'},
		{'U', ' ', 'W', 'F', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'W', 'W', 'W', 'W', 'W', 'W'},
		{'U', ' ', 'W', 'F', ' ', ' ', ' ', 'D', ' ', ' ', 'T', 'T', 'W', ' ', ' ', ' ',
				' ', ' ', 'U', 'W', ' ', 'L', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', 'T', 'J', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				'W', ' ', ' ', 'W', 'L', ' ', 'W'},
		{'J', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'C', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'L', ' ', ' ', ' ', ' ', 'W', 'C', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', 'M', 'c', 'B', 'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'd', 'W', 'L', ' ', ' ', ' ', ' ', 'W', 'L', ' ', ' ', 'B', 'W',
				'W', 'B', 'B', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'd', 'W', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', 'D', ' ', ' ', 'U', ' ', ' ', ' ', 'D', ' ', ' ', 'F', 'F',
				'W', 'M', 'M', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', 'W', ' ', ' ', 'U', ' ', ' ', 'W', 'W', ' ', ' ', ' ', ' ',
				'C', ' ', ' ', 'W', ' ', ' ', 'W'},
		{'C', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', ' ', ' ',
				'L', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'L', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'L', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'U', 'U', ' ', 'Q', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', 'W', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'U', 'U', ' ', 'Q', 'b', ' ', 'U', 'U',
				'B', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'S', 'S', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'Q', 'b', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', 'd', ' ', 'W'},
		{'U', 'U', ' ', ' ', ' ', 'L', 'a', 'a', 'a', ' ', ' ', 'Q', 'B', 'a', 'a', 'a',
				'a', 'a', 'a', ' ', 'd', 'D', 'W'},
}};

// ABANDONED_INTERIOR2_LAYER2
static constexpr std::array<std::array<char, 23>, 23> ABANDONED_INTERIOR2_LAYER2 = {{
		{'W', 'W', 'W', 'D', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'D', 'W', 'W', 'W'},
		{'P', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'O', ' ', ' ', 'O', 'W', ' ', ' ', 'W',
				' ', ' ', ' ', ' ', ' ', 'O', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'Q', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', 'W', 'F', ' ', ' ', ' ', 'Q', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				' ', ' ', 'P', 'P', 'P', ' ', 'D'},
		{'P', ' ', 'W', 'F', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'W', 'W', 'W', 'W', 'W', 'W'},
		{'P', ' ', 'W', 'F', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', 'P', 'W', ' ', 'P', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'P', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'O', ' ', ' ', ' ', ' ', 'W', 'P', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', ' ', 'c', 'B', 'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'd', 'W', 'O', ' ', ' ', ' ', ' ', 'W', 'O', ' ', ' ', 'B', 'W',
				'W', 'B', 'B', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'd', 'W', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', 'D', ' ', ' ', 'P', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', 'W', ' ', ' ', 'P', ' ', ' ', 'W', 'W', ' ', ' ', ' ', ' ',
				' ', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', ' ', ' ',
				'O', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'O', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'O', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'P', 'P', ' ', 'Q', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', 'W', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'P', 'P', ' ', 'Q', 'b', ' ', 'P', 'P',
				'c', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'Q', 'b', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', 'd', ' ', 'W'},
		{'P', 'P', ' ', ' ', ' ', 'O', 'a', 'a', 'a', ' ', ' ', 'Q', 'b', 'a', 'a', 'a',
				'a', 'a', 'a', ' ', 'd', ' ', 'D'},
}};

/// Maps interior layout characters to actual block types for different floor layers
inline std::optional<Block> get_interior_block(char c, bool is_layer2, Block wall_block)
{
	switch (c) {
	case ' ':
		return {}; // Nothing
	case 'W':
		return wall_block; // Use the building's wall block for interior walls
	case 'U':
		return OAK_FENCE; // Oak Fence
	case 'S':
		return OAK_STAIRS; // Oak Stairs
	case 'B':
		return BOOKSHELF; // Bookshelf
	case 'C':
		return CRAFTING_TABLE; // Crafting Table
	case 'F':
		return FURNACE; // Furnace
	case '1':
		return RED_BED_NORTH_HEAD; // Bed North Head
	case '2':
		return RED_BED_NORTH_FOOT; // Bed North Foot
	case '3':
		return RED_BED_EAST_HEAD; // Bed East Head
	case '4':
		return RED_BED_EAST_FOOT; // Bed East Foot
	case '5':
		return RED_BED_SOUTH_HEAD; // Bed South Head
	case '6':
		return RED_BED_SOUTH_FOOT; // Bed South Foot
	case '7':
		return RED_BED_WEST_HEAD; // Bed West Head
	case '8':
		return RED_BED_WEST_FOOT; // Bed West Foot
	case 'L':
		return CAULDRON; // Cauldron
	case 'A':
		return ANVIL; // Anvil
	case 'P':
		return OAK_PRESSURE_PLATE; // Pressure Plate
	case 'D': {
		// Use different door types for different layers
		if (is_layer2) {
			return DARK_OAK_DOOR_UPPER;
		} else {
			return DARK_OAK_DOOR_LOWER;
		}
	}
	case 'J':
		return NOTE_BLOCK; // Note block
	case 'G':
		return GLOWSTONE; // Glowstone
	case 'N':
		return BREWING_STAND; // Brewing Stand
	case 'T':
		return WHITE_CARPET; // White Carpet
	case 'E':
		return OAK_LEAVES; // Oak Leaves
	case 'O':
		return COBWEB; // Cobweb
	case 'a':
		return CHISELLED_BOOKSHELF_NORTH; // Chiseled Bookshelf
	case 'b':
		return CHISELLED_BOOKSHELF_EAST; // Chiseled Bookshelf East
	case 'c':
		return CHISELLED_BOOKSHELF_SOUTH; // Chiseled Bookshelf South
	case 'd':
		return CHISELLED_BOOKSHELF_WEST; // Chiseled Bookshelf West
	case 'M':
		return DAMAGED_ANVIL; // Damaged Anvil
	case 'Q':
		return SCAFFOLDING; // Scaffolding
	default:
		return {}; // Default case for unknown characters
	}
}

/// Generates interior layouts inside buildings at each floor level
void generate_building_interior(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &floor_area, int min_x, int min_z,
		int max_x, int max_z, int start_y_offset, int building_height, Block wall_block,
		Block floor_block, const std::vector<int> &floor_levels, const Args &args,
		const ProcessedWay &element, int abs_terrain_offset, bool is_abandoned_building,
		const CoordinateBitmap &building_passages, bool has_sloped_roof,
		const interior_uses::InteriorPlan *plan,
		const std::vector<interior_uses::Claim> &claims, double scale,
		const std::vector<interior_uses::Entry> &entries, std::uint64_t interior_seed)
{
	(void)args;
	(void)element;
	// Skip interior generation for very small buildings
	int width = max_x - min_x + 1;
	int depth = max_z - min_z + 1;

	if (width < 8 || depth < 8) {
		return; // Building too small for interior
	}

	// For efficiency, create a unordered_set of floor area coordinates
	std::unordered_set<std::uint64_t> floor_area_set;
	floor_area_set.reserve(floor_area.size());
	for (const auto &p : floor_area) {
		floor_area_set.insert(interior_cell_key(p.first, p.second));
	}
	// Resolve overlapping-outline ownership once per footprint cell, matching
	// Rust's claimed_to array instead of retesting every polygon on every floor.
	std::unordered_map<std::uint64_t, double> claimed_top;
	if (!claims.empty()) {
		claimed_top.reserve(floor_area.size());
		for (const auto &[x, z] : floor_area) {
			double top = 0.0;
			for (const auto &claim : claims)
				if (claim.top_m > top && interior_uses::covers(claim.ring, x, z))
					top = claim.top_m;
			if (top > 0.0) {
				claimed_top.emplace(interior_cell_key(x, z), top);
			}
		}
	}
	std::size_t owned_footprint_cells = 0;
	for (const auto &[x, z] : floor_area) {
		const auto claim = claimed_top.find(interior_cell_key(x, z));
		if (claim == claimed_top.end() || claim->second <= 1.0)
			++owned_footprint_cells;
	}
	const bool homes_fit = width >= 8 && depth >= 8 && owned_footprint_cells > 100;
	std::optional<std::pair<std::pair<int, int>, std::pair<int, int>>> shaft;
	if (floor_levels.size() >= 2) {
		std::unordered_set<std::uint64_t> own;
		own.reserve(floor_area.size());
		for (const auto &[x, z] : floor_area) {
			const auto key = interior_cell_key(x, z);
			const auto claim = claimed_top.find(key);
			if (claim == claimed_top.end() || claim->second <= 1.0)
				own.insert(key);
		}
		using ShaftKey = std::tuple<bool, int, std::uint64_t>;
		std::optional<
				std::pair<ShaftKey, std::pair<std::pair<int, int>, std::pair<int, int>>>>
				best;
		for (const auto &[x, z] : floor_area) {
			const auto key = interior_cell_key(x, z);
			if (!own.contains(key) || building_passages.contains(x, z))
				continue;
			std::vector<std::pair<int, int>> walls;
			for (const auto direction : INTERIOR_DIRS) {
				const auto neighbour =
						interior_cell_key(x + direction.first, z + direction.second);
				if (!own.contains(neighbour))
					walls.push_back(direction);
			}
			if (walls.empty())
				continue;
			const auto inward = std::pair{-walls.front().first, -walls.front().second};
			const auto inner = interior_cell_key(x + inward.first, z + inward.second);
			if (walls.size() > 2 || !own.contains(inner))
				continue;
			int far = 30;
			if (!entries.empty()) {
				far = std::numeric_limits<int>::max();
				for (const auto &entry : entries)
					far = std::min(far, std::abs(entry.cell.first - x) +
												std::abs(entry.cell.second - z));
				far = std::min(far, 30);
			}
			if (far < 4)
				continue;
			const ShaftKey candidate{
					walls.size() == 2, far, interior_mix(x, z, 0x01ADDE25ULL)};
			if (!best || candidate > best->first)
				best = std::pair{candidate, std::pair{std::pair{x, z}, inward}};
		}
		if (best)
			shaft = best->second;
	}

	// Add buffer around edges to avoid placing furniture too close to walls
	int buffer = 2;
	int interior_min_x = min_x + buffer;
	int interior_min_z = min_z + buffer;
	int interior_max_x = max_x - buffer;
	int interior_max_z = max_z - buffer;

	// Generate interiors for each floor
	for (size_t floor_index = 0; floor_index < floor_levels.size(); ++floor_index) {
		int floor_y = floor_levels[floor_index];

		// Store wall and door positions for this floor to extend them to the ceiling
		std::vector<std::pair<int, int>> wall_positions;
		std::vector<std::pair<int, int>> door_positions;
		std::vector<interior_uses::Unit> floor_units;

		// Determine the floor extension height (ceiling) - either next floor or roof
		int current_floor_ceiling;
		if (floor_index < floor_levels.size() - 1) {
			// For intermediate floors, extend walls up to just below the next floor
			current_floor_ceiling = floor_levels[floor_index + 1] - 1;
		} else {
			if (has_sloped_roof) {
				current_floor_ceiling = start_y_offset + building_height;
			} else {
				current_floor_ceiling = start_y_offset + building_height + 1;
			}
		}

		std::unordered_set<std::uint64_t> planned_partition_cells;
		std::unordered_set<std::uint64_t> kept_cells;
		std::vector<std::vector<std::pair<int, int>>> unit_zone_cells;
		std::vector<std::pair<int, int>> connectivity_starts;
		std::vector<std::pair<int, int>> partition_walls;
		std::unordered_set<std::uint64_t> cottage_cells;
		if (plan && !plan->floors.empty() && !is_abandoned_building) {
			// Canvas reserves exterior entries on the ground storey and the
			// stair/ladder cells on every storey before it partitions unit zones.
			auto keep_pair = [&](std::pair<int, int> cell) {
				const auto key = interior_cell_key(cell.first, cell.second);
				if (floor_area_set.contains(key) &&
						!editor.get_block_absolute(cell.first,
								floor_y + abs_terrain_offset + 1, cell.second))
					kept_cells.insert(key);
			};
			// Match Rust Canvas::clear_doorways: furniture must not obstruct the
			// interior side of any shell door, even when it was not discovered as
			// a planned building entry.
			for (const auto &[x, z] : floor_area) {
				if (editor.get_block_absolute(x, floor_y + abs_terrain_offset + 1, z))
					continue;
				for (const auto &[dx, dz] : INTERIOR_DIRS) {
					const int wall_x = x + dx;
					const int wall_z = z + dz;
					const auto block_name = editor.block_name_absolute(
							wall_x, floor_y + abs_terrain_offset + 1, wall_z);
					if (!block_name || !block_name->ends_with("_door") ||
							std::find(door_positions.begin(), door_positions.end(),
									std::pair{wall_x, wall_z}) != door_positions.end())
						continue;
					keep_pair({x, z});
					keep_pair({x - dx, z - dz});
				}
			}
			if (floor_index == 0) {
				for (const auto &entry : entries) {
					keep_pair(entry.cell);
					keep_pair({entry.cell.first + entry.inward.first,
							entry.cell.second + entry.inward.second});
				}
			}
			if (shaft) {
				keep_pair(shaft->first);
				keep_pair({shaft->first.first + shaft->second.first,
						shaft->first.second + shaft->second.second});
			}
			const auto &units = floor_index < plan->floors.size()
										? plan->floors[floor_index]
										: plan->floors.back();
			floor_units = units;
			std::vector<std::pair<int, int>> anchors;
			anchors.reserve(units.size());
			for (const auto &unit : units)
				anchors.push_back(unit.anchor);
			unit_zone_cells.resize(anchors.size());
			const double floor_metres =
					static_cast<double>(floor_y - start_y_offset) / std::max(scale, 0.01);
			std::vector<std::pair<int, int>> layout_cells;
			layout_cells.reserve(floor_area.size());
			std::unordered_map<std::uint64_t, std::uint16_t> zone_of;
			for (const auto &[x, z] : floor_area) {
				const auto key = interior_cell_key(x, z);
				const auto claim = claimed_top.find(key);
				if (claim != claimed_top.end() && claim->second > floor_metres + 1.0)
					continue;
				if (building_passages.contains(x, z) &&
						floor_y < start_y_offset + std::min(BUILDING_PASSAGE_HEIGHT,
														   building_height))
					continue;
				layout_cells.emplace_back(x, z);
				std::uint16_t zone = 1;
				if (anchors.size() > 1) {
					std::int64_t best_distance = std::numeric_limits<std::int64_t>::max();
					for (std::size_t i = 0; i < anchors.size(); ++i) {
						const auto dx = static_cast<std::int64_t>(x) - anchors[i].first;
						const auto dz = static_cast<std::int64_t>(z) - anchors[i].second;
						const auto distance = dx * dx + dz * dz;
						if (distance < best_distance) {
							best_distance = distance;
							zone = static_cast<std::uint16_t>(i + 1);
						}
					}
				}
				zone_of.emplace(key, zone);
				if (zone > 0 && zone <= unit_zone_cells.size())
					unit_zone_cells[zone - 1].emplace_back(x, z);
			}
			if (anchors.size() > 1) {
				std::sort(layout_cells.begin(), layout_cells.end(),
						[](const auto &a, const auto &b) {
							return std::pair(a.second, a.first) <
								   std::pair(b.second, b.first);
						});
				std::map<std::pair<std::uint16_t, std::uint16_t>,
						std::vector<std::pair<int, int>>>
						borders;
				for (const auto &[x, z] : layout_cells) {
					const auto here = zone_of.at(interior_cell_key(x, z));
					for (const auto direction : INTERIOR_DIRS) {
						const auto it = zone_of.find(interior_cell_key(
								x + direction.first, z + direction.second));
						if (it != zone_of.end() && it->second != here &&
								here > it->second) {
							auto &border = borders[{it->second, here}];
							if (std::find(border.begin(), border.end(),
										std::pair{x, z}) == border.end())
								border.emplace_back(x, z);
						}
					}
				}
				const int floor_abs = floor_y + abs_terrain_offset;
				const int top_abs = current_floor_ceiling + abs_terrain_offset;
				for (const auto &[zones, border] : borders) {
					std::optional<std::pair<std::pair<int, int>, std::pair<int, int>>>
							door;
					const auto middle = border.size() / 2;
					std::vector<std::size_t> order(border.size());
					for (std::size_t i = 0; i < order.size(); ++i)
						order[i] = i;
					std::stable_sort(order.begin(), order.end(), [&](auto a, auto b) {
						return std::abs(static_cast<std::int64_t>(a) -
										static_cast<std::int64_t>(middle)) <
							   std::abs(static_cast<std::int64_t>(b) -
										static_cast<std::int64_t>(middle));
					});
					for (const auto i : order) {
						const auto [x, z] = border[i];
						for (const auto direction : INTERIOR_DIRS) {
							const int lx = x - direction.first, lz = z - direction.second;
							const int hx = x + direction.first, hz = z + direction.second;
							auto walkable = [&](int cx, int cz) {
								const auto key = interior_cell_key(cx, cz);
								return zone_of.contains(key) &&
									   (!editor.get_block_absolute(
												cx, floor_abs + 1, cz) ||
											   kept_cells.contains(key));
							};
							if (!kept_cells.contains(interior_cell_key(x, z)) &&
									zone_of.contains(interior_cell_key(lx, lz)) &&
									zone_of.at(interior_cell_key(lx, lz)) ==
											zones.first &&
									zone_of.contains(interior_cell_key(hx, hz)) &&
									zone_of.at(interior_cell_key(hx, hz)) ==
											zones.second &&
									walkable(lx, lz) && walkable(hx, hz)) {
								door = std::pair{std::pair{x, z}, direction};
								break;
							}
						}
						if (door)
							break;
					}
					for (const auto &[x, z] : border) {
						const auto key = interior_cell_key(x, z);
						planned_partition_cells.insert(key);
						if (kept_cells.contains(key))
							continue;
						if (door && door->first == std::pair{x, z}) {
							if (editor.get_block_absolute(x, floor_abs + 1, z))
								continue;
							door_positions.emplace_back(x, z);
							const char *facing = ladder_facing(door->second);
							const BlockWithProperties lower{DARK_OAK_DOOR_LOWER,
									{{"half", "lower"}, {"facing", facing},
											{"hinge", "left"}}};
							const BlockWithProperties upper{DARK_OAK_DOOR_UPPER,
									{{"half", "upper"}, {"facing", facing},
											{"hinge", "left"}}};
							editor.set_block_with_properties_absolute(lower, x,
									floor_abs + 1, z, std::nullopt, std::nullopt);
							if (top_abs >= floor_abs + 2)
								editor.set_block_with_properties_absolute(upper, x,
										floor_abs + 2, z, std::nullopt, std::nullopt);
							continue;
						}
						if (editor.get_block_absolute(x, floor_abs + 1, z))
							continue;
						wall_positions.emplace_back(x, z);
						partition_walls.emplace_back(x, z);
						for (int y = floor_abs + 1; y <= top_abs; ++y)
							editor.set_block_absolute(
									wall_block, x, y, z, std::nullopt, std::nullopt);
					}
				}
			}
			if (floor_index == 0) {
				for (const auto &entry : entries)
					if (zone_of.contains(
								interior_cell_key(entry.cell.first, entry.cell.second)))
						connectivity_starts.push_back(entry.cell);
			}
			if (shaft && (floor_index > 0 || entries.empty())) {
				const auto [x, z] = shaft->first;
				const auto key = interior_cell_key(x, z);
				if (zone_of.contains(key) && kept_cells.contains(key))
					connectivity_starts.emplace_back(x, z);
			}
			const int floor_abs = floor_y + abs_terrain_offset;
			const int top_abs = current_floor_ceiling + abs_terrain_offset;
			connect_partition_zones(editor, layout_cells, zone_of,
					planned_partition_cells, partition_walls, door_positions, kept_cells,
					connectivity_starts, floor_abs, top_abs);
		}
		if (plan && !plan->floors.empty() && !is_abandoned_building) {
			for (std::size_t i = 0; i < floor_units.size() && i < unit_zone_cells.size();
					++i) {
				if (floor_units[i].use.kind == interior_uses::UseKind::Home && homes_fit)
					continue;
				for (const auto &[x, z] : unit_zone_cells[i])
					cottage_cells.insert(interior_cell_key(x, z));
			}
		}

		// Choose the appropriate interior pattern based on floor number
		const std::array<std::array<char, 23>, 23> *layer1;
		const std::array<std::array<char, 23>, 23> *layer2;
		if (is_abandoned_building) {
			if (floor_index == 0) {
				layer1 = &ABANDONED_INTERIOR1_LAYER1;
				layer2 = &ABANDONED_INTERIOR1_LAYER2;
			} else {
				layer1 = &ABANDONED_INTERIOR2_LAYER1;
				layer2 = &ABANDONED_INTERIOR2_LAYER2;
			}
		} else if (floor_index == 0) {
			// Ground floor uses INTERIOR1 patterns
			layer1 = &INTERIOR1_LAYER1;
			layer2 = &INTERIOR1_LAYER2;
		} else {
			// Upper floors use INTERIOR2 patterns
			layer1 = &INTERIOR2_LAYER1;
			layer2 = &INTERIOR2_LAYER2;
		}

		// Get dimensions for the selected pattern
		int pattern_height = static_cast<int>(layer1->size());
		int pattern_width = static_cast<int>((*layer1)[0].size());
		const unsigned building_salt =
				static_cast<unsigned>(min_x) * 0x9E3779B1u ^ static_cast<unsigned>(min_z);
		const std::uint64_t floor_seed =
				interior_seed ^
				(static_cast<std::uint64_t>(floor_index) + 1) * 0x9E3779B97F4A7C15ULL;
		const unsigned chest_modulus = is_abandoned_building ? 160u : 128u;

		// Calculate Y offset - place interior 1 block above floor level consistently
		int y_offset = 1;
		const bool shaft_open =
				shaft && !editor.get_block_absolute(shaft->first.first,
								 floor_y + abs_terrain_offset + 1, shaft->first.second);
		if (floor_index == 0) {
			for (const auto &entry : entries) {
				const auto [x, z] = entry.cell;
				for (const auto [cx, cz] :
						{entry.cell, std::pair{x + entry.inward.first,
											 z + entry.inward.second}}) {
					if (!floor_area_set.contains(interior_cell_key(cx, cz)))
						continue;
					for (int y = floor_y + y_offset + abs_terrain_offset;
							y <= current_floor_ceiling + abs_terrain_offset; ++y)
						editor.set_block_absolute(AIR, cx, y, cz);
				}
			}
		}

		// Create a seamless repeating pattern across the interior of this floor
		for (int z = interior_min_z; z <= interior_max_z; ++z) {
			for (int x = interior_min_x; x <= interior_max_x; ++x) {
				// Skip if outside the building's floor area
				const auto key = interior_cell_key(x, z);
				if (floor_area_set.find(key) == floor_area_set.end()) {
					continue;
				}
				if (planned_partition_cells.contains(key))
					continue;
				if (cottage_cells.contains(key))
					continue;
				if (kept_cells.contains(key))
					continue;
				if (floor_index == 0 &&
						std::any_of(
								entries.begin(), entries.end(), [&](const auto &entry) {
									return (entry.cell.first == x &&
												   entry.cell.second == z) ||
										   (entry.cell.first + entry.inward.first == x &&
												   entry.cell.second +
																   entry.inward.second ==
														   z);
								}))
					continue;
				if (shaft_open && shaft &&
						((shaft->first.first == x && shaft->first.second == z) ||
								(shaft->first.first + shaft->second.first == x &&
										shaft->first.second + shaft->second.second == z)))
					continue;
				const double floor_metres =
						static_cast<double>(floor_y - start_y_offset) /
						std::max(scale, 0.01);
				const auto claim = claimed_top.find(key);
				if (claim != claimed_top.end() && claim->second > floor_metres + 1.0)
					continue;
				if (building_passages.contains(x, z) &&
						floor_y < start_y_offset + std::min(BUILDING_PASSAGE_HEIGHT,
														   building_height)) {
					continue;
				}

				// Map the world coordinates to pattern coordinates using modulo
				// This creates a seamless tiling effect across the entire building
				// Add floor_index offset to create variation between floors
				int pattern_x = ((x - interior_min_x + static_cast<int>(floor_index)) %
												pattern_width +
										pattern_width) %
								pattern_width;
				int pattern_z = ((z - interior_min_z + static_cast<int>(floor_index)) %
												pattern_height +
										pattern_height) %
								pattern_height;

				// Access the pattern arrays safely
				char cell1 = (*layer1)[pattern_z][pattern_x];
				char cell2 = (*layer2)[pattern_z][pattern_x];
				if (cell1 == ' ' && cell2 == ' ') {
					const bool wall_adjacent =
							(*layer1)[(pattern_z + pattern_height - 1) % pattern_height]
									 [pattern_x] == 'W' ||
							(*layer1)[(pattern_z + 1) % pattern_height][pattern_x] ==
									'W' ||
							(*layer1)[pattern_z][(pattern_x + pattern_width - 1) %
												 pattern_width] == 'W' ||
							(*layer1)[pattern_z][(pattern_x + 1) % pattern_width] == 'W';
					if (wall_adjacent &&
							chest_gate(x, z, floor_y, building_salt, chest_modulus)) {
						std::vector<std::tuple<std::string, int, int>> items;
						for (const auto &item :
								buildings_loot::chest_loot(x, z, building_salt))
							items.emplace_back(item.id, item.slot, item.count);
						editor.set_chest_with_items_absolute(
								x, floor_y + y_offset + abs_terrain_offset, z, items);
						continue;
					}
				}

				// Place first layer blocks
				auto opt_block1 = get_interior_block(cell1, false, wall_block);
				if (opt_block1.has_value()) {
					editor.set_block_absolute(opt_block1.value(), x,
							floor_y + y_offset + abs_terrain_offset, z, std::nullopt,
							std::nullopt);
					if (cell1 >= '1' && cell1 <= '8')
						editor.set_bed_block_entity_absolute(
								x, floor_y + y_offset + abs_terrain_offset, z);

					// If this is a wall in layer 1, add to wall positions to extend later
					if (cell1 == 'W') {
						wall_positions.emplace_back(x, z);
					}
					// If this is a door in layer 1, add to door positions to add wall above later
					else if (cell1 == 'D') {
						door_positions.emplace_back(x, z);
					}
				}

				// Place second layer blocks
				auto opt_block2 = get_interior_block(cell2, true, wall_block);
				if (opt_block2.has_value()) {
					editor.set_block_absolute(opt_block2.value(), x,
							floor_y + y_offset + abs_terrain_offset + 1, z, std::nullopt,
							std::nullopt);
				}
			}
		}

		// Preserve the Rust plan's per-unit use signal in addition to the shared
		// legacy pattern: add a small, use-specific focal furnishing in each unit.
		// Candidate cells stay inside that unit's nearest-anchor zone and avoid
		// structural cells; the rest of the existing floor layout remains intact.
		if (plan && !is_abandoned_building && !floor_units.empty()) {
			std::unordered_set<std::uint64_t> structural_cells = planned_partition_cells;
			for (const auto &[x, z] : wall_positions)
				structural_cells.insert(interior_cell_key(x, z));
			for (const auto &[x, z] : door_positions)
				structural_cells.insert(interior_cell_key(x, z));
			const double floor_metres =
					static_cast<double>(floor_y - start_y_offset) / std::max(scale, 0.01);
			for (std::size_t unit_index = 0; unit_index < floor_units.size();
					++unit_index) {
				const auto &unit = floor_units[unit_index];
				if (unit.use.kind == interior_uses::UseKind::Shop) {
					if (unit_index < unit_zone_cells.size())
						furnish_shop(editor, unit, unit_zone_cells[unit_index], entries,
								door_positions, structural_cells, kept_cells, claimed_top,
								building_passages, floor_metres,
								start_y_offset + std::min(BUILDING_PASSAGE_HEIGHT,
														 building_height),
								floor_y, floor_y + y_offset + abs_terrain_offset,
								current_floor_ceiling, floor_seed, building_salt);
					continue;
				}
				if (unit.use.kind == interior_uses::UseKind::Supermarket) {
					if (unit_index < unit_zone_cells.size())
						furnish_supermarket(editor, unit_zone_cells[unit_index], entries,
								door_positions, structural_cells, kept_cells, claimed_top,
								building_passages, floor_metres,
								start_y_offset + std::min(BUILDING_PASSAGE_HEIGHT,
														 building_height),
								floor_y, floor_y + y_offset + abs_terrain_offset,
								current_floor_ceiling, floor_seed);
					continue;
				}
				if (unit.use.kind == interior_uses::UseKind::Food) {
					if (unit_index < unit_zone_cells.size())
						furnish_eatery(editor, unit, unit_zone_cells[unit_index], entries,
								door_positions, structural_cells, kept_cells, claimed_top,
								building_passages, floor_metres,
								start_y_offset + std::min(BUILDING_PASSAGE_HEIGHT,
														 building_height),
								floor_y, floor_y + y_offset + abs_terrain_offset,
								current_floor_ceiling, floor_seed);
					continue;
				}
				if (unit.use.kind == interior_uses::UseKind::School ||
						unit.use.kind == interior_uses::UseKind::Kindergarten ||
						unit.use.kind == interior_uses::UseKind::Library ||
						unit.use.kind == interior_uses::UseKind::Clinic ||
						unit.use.kind == interior_uses::UseKind::Hospital ||
						unit.use.kind == interior_uses::UseKind::Museum ||
						unit.use.kind == interior_uses::UseKind::Station ||
						unit.use.kind == interior_uses::UseKind::Office ||
						unit.use.kind == interior_uses::UseKind::Bank ||
						unit.use.kind == interior_uses::UseKind::Workshop ||
						unit.use.kind == interior_uses::UseKind::Hotel) {
					if (unit_index < unit_zone_cells.size())
						furnish_civic(editor, unit, unit_zone_cells[unit_index], entries,
								door_positions, structural_cells, kept_cells, claimed_top,
								building_passages, floor_metres,
								start_y_offset + std::min(BUILDING_PASSAGE_HEIGHT,
														 building_height),
								floor_y, floor_y + y_offset + abs_terrain_offset,
								current_floor_ceiling, floor_seed, building_salt,
								floor_index, floor_levels.size(), wall_block,
								planned_partition_cells, wall_positions);
					continue;
				}
				if (unit.use.kind == interior_uses::UseKind::Worship ||
						unit.use.kind == interior_uses::UseKind::SportsHall ||
						unit.use.kind == interior_uses::UseKind::Gym ||
						unit.use.kind == interior_uses::UseKind::Auditorium ||
						unit.use.kind == interior_uses::UseKind::Warehouse ||
						unit.use.kind == interior_uses::UseKind::Factory ||
						unit.use.kind == interior_uses::UseKind::Barn) {
					if (unit_index < unit_zone_cells.size())
						furnish_hall(editor, unit, unit_zone_cells[unit_index], entries,
								door_positions, structural_cells, kept_cells, claimed_top,
								building_passages, floor_metres,
								start_y_offset + std::min(BUILDING_PASSAGE_HEIGHT,
														 building_height),
								floor_y, floor_y + y_offset + abs_terrain_offset,
								current_floor_ceiling, floor_seed, building_salt);
					continue;
				}
				if (unit.use.kind == interior_uses::UseKind::Home && !homes_fit) {
					if (unit_index >= unit_zone_cells.size() ||
							unit_zone_cells[unit_index].size() < 45)
						continue;
					auto free_cell = [&](int x, int z) {
						const auto key = interior_cell_key(x, z);
						if (structural_cells.contains(key) || kept_cells.contains(key) ||
								editor.get_block_absolute(
										x, floor_y + y_offset + abs_terrain_offset, z))
							return false;
						const auto claim = claimed_top.find(key);
						if (claim != claimed_top.end() &&
								claim->second > floor_metres + 1.0)
							return false;
						return !building_passages.contains(x, z) ||
							   floor_y >=
									   start_y_offset + std::min(BUILDING_PASSAGE_HEIGHT,
																building_height);
					};
					const auto &zone_cells = unit_zone_cells[unit_index];
					std::unordered_set<std::uint64_t> zone_set;
					zone_set.reserve(zone_cells.size());
					for (const auto &[x, z] : zone_cells)
						zone_set.insert(interior_cell_key(x, z));
					std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>
							walls;
					for (const auto &[x, z] : zone_cells) {
						if (!free_cell(x, z))
							continue;
						for (const auto direction : INTERIOR_DIRS) {
							if (!zone_set.contains(interior_cell_key(
										x + direction.first, z + direction.second))) {
								walls.emplace_back(std::pair{x, z},
										std::pair{-direction.first, -direction.second});
								break;
							}
						}
					}
					std::stable_sort(walls.begin(), walls.end(),
							[&](const auto &a, const auto &b) {
								const auto da =
										std::abs(a.first.first - unit.anchor.first) +
										std::abs(a.first.second - unit.anchor.second);
								const auto db =
										std::abs(b.first.first - unit.anchor.first) +
										std::abs(b.first.second - unit.anchor.second);
								return da > db;
							});
					const int beds = floor_index == 0 && floor_levels.size() > 1
											 ? 0
											 : 1 + static_cast<int>(std::min<std::size_t>(
														   2, zone_cells.size() / 60));

					int slept = 0;
					for (const auto &[wall, inward] : walls) {
						if (slept >= beds)
							break;
						const std::pair<int, int> foot{
								wall.first + inward.first, wall.second + inward.second};
						if (!free_cell(wall.first, wall.second) ||
								!free_cell(foot.first, foot.second) ||
								!zone_set.contains(
										interior_cell_key(foot.first, foot.second)))
							continue;
						const char *facing =
								ladder_facing({-inward.first, -inward.second});
						for (const auto &[cell, part] :
								{std::pair{foot, "foot"}, std::pair{wall, "head"}}) {
							const BlockWithProperties bed{RED_BED_NORTH_HEAD,
									{{"facing", facing}, {"part", part},
											{"occupied", "false"}}};
							editor.set_block_with_properties_absolute(bed, cell.first,
									floor_y + y_offset + abs_terrain_offset, cell.second,
									std::nullopt, std::nullopt);
							editor.set_bed_block_entity_absolute(cell.first,
									floor_y + y_offset + abs_terrain_offset, cell.second);
						}
						++slept;
					}

					if (floor_index == 0) {
						const std::array<Block, 4> kitchen = {
								FURNACE, CRAFTING_TABLE, WATER_CAULDRON, BARREL};
						std::size_t placed = 0;
						for (auto it = walls.rbegin();
								it != walls.rend() && placed < kitchen.size(); ++it) {
							const auto &[cell, inward] = *it;
							if (!free_cell(cell.first, cell.second))
								continue;
							if (kitchen[placed] == FURNACE) {
								const BlockWithProperties furnace{
										FURNACE, {{"facing", ladder_facing(inward)}}};
								editor.set_block_with_properties_absolute(furnace,
										cell.first,
										floor_y + y_offset + abs_terrain_offset,
										cell.second, std::nullopt, std::nullopt);
							} else {
								editor.set_block_absolute(kitchen[placed], cell.first,
										floor_y + y_offset + abs_terrain_offset,
										cell.second, std::nullopt, std::nullopt);
							}
							++placed;
						}
						std::vector<std::pair<int, int>> free_cells;
						for (const auto &cell : zone_cells)
							if (free_cell(cell.first, cell.second))
								free_cells.push_back(cell);
						std::stable_sort(free_cells.begin(), free_cells.end(),
								[&](const auto &a, const auto &b) {
									const auto da =
											std::abs(a.first - unit.anchor.first) +
											std::abs(a.second - unit.anchor.second);
									const auto db =
											std::abs(b.first - unit.anchor.first) +
											std::abs(b.second - unit.anchor.second);
									return da < db;
								});
						for (const auto &[x, z] : free_cells) {
							const bool left =
									free_cell(x - 1, z) &&
									zone_set.contains(interior_cell_key(x - 1, z));
							const bool right =
									free_cell(x + 1, z) &&
									zone_set.contains(interior_cell_key(x + 1, z));
							if (!left && !right)
								continue;
							editor.set_block_absolute(OAK_SLAB_TOP, x,
									floor_y + y_offset + abs_terrain_offset, z,
									std::nullopt, std::nullopt);
							for (const auto &[seat_x, seat_z] :
									{std::pair{x - 1, z}, std::pair{x + 1, z}}) {
								if (!free_cell(seat_x, seat_z) ||
										!zone_set.contains(
												interior_cell_key(seat_x, seat_z)))
									continue;
								const BlockWithProperties seat{OAK_STAIRS,
										{{"facing", seat_x < x ? "east" : "west"},
												{"half", "bottom"},
												{"shape", "straight"}}};
								editor.set_block_with_properties_absolute(seat, seat_x,
										floor_y + y_offset + abs_terrain_offset, seat_z,
										std::nullopt, std::nullopt);
							}
							break;
						}
					}
					bool chest_left = true, shelf_left = true;
					for (const auto &[cell, inward] : walls) {
						(void)inward;
						const auto h = interior_mix(cell.first, cell.second, floor_seed);
						if (chest_left && h % 3 == 0 &&
								free_cell(cell.first, cell.second)) {
							std::vector<std::tuple<std::string, int, int>> items;
							for (const auto &item : buildings_loot::chest_loot(
										 cell.first, cell.second, building_salt))
								items.emplace_back(item.id, item.slot, item.count);
							editor.set_chest_with_items_absolute(cell.first,
									floor_y + y_offset + abs_terrain_offset, cell.second,
									items);
							chest_left = false;
						} else if (shelf_left && h % 3 == 1 &&
								   free_cell(cell.first, cell.second)) {
							editor.set_block_absolute(BOOKSHELF, cell.first,
									floor_y + y_offset + abs_terrain_offset, cell.second,
									std::nullopt, std::nullopt);
							shelf_left = false;
						}
						if (!chest_left && !shelf_left)
							break;
					}
					continue;
				}
				std::vector<Block> fixtures;
				using interior_uses::Goods;
				using interior_uses::UseKind;
				switch (unit.use.kind) {
				case UseKind::Shop:
					if (unit.use.goods == Goods::Books) {
						fixtures = {BOOKSHELF, LECTERN};
					} else if (unit.use.goods == Goods::Bakery) {
						fixtures = {CAKE, BARREL};
					} else if (unit.use.goods == Goods::Pharmacy) {
						fixtures = {BREWING_STAND, CHEST};
					} else {
						fixtures = {BARREL, CHEST};
					}
					break;
				case UseKind::Supermarket:
					fixtures = {BARREL, HAY_BALE, CHEST};
					break;
				case UseKind::Food:
					fixtures = {OAK_FENCE, CAKE};
					break;
				case UseKind::Office:
				case UseKind::School:
				case UseKind::Library:
				case UseKind::Museum:
				case UseKind::Station:
					fixtures = {LECTERN, BOOKSHELF};
					break;
				case UseKind::Bank:
					fixtures = {GOLD_BLOCK, CHEST};
					break;
				case UseKind::Workshop:
				case UseKind::Factory:
					fixtures = {CRAFTING_TABLE, ANVIL};
					break;
				case UseKind::Clinic:
				case UseKind::Hospital:
					fixtures = {BREWING_STAND, CHEST};
					break;
				case UseKind::Kindergarten:
					fixtures = {BOOKSHELF, CAKE};
					break;
				case UseKind::Hotel:
					fixtures = {CHEST, BOOKSHELF};
					break;
				case UseKind::Worship:
					fixtures = {LECTERN, GOLD_BLOCK};
					break;
				case UseKind::SportsHall:
				case UseKind::Gym:
					fixtures = {OAK_FENCE, CRAFTING_TABLE};
					break;
				case UseKind::Auditorium:
					fixtures = {OAK_PLANKS, LECTERN};
					break;
				case UseKind::Warehouse:
				case UseKind::Barn:
					fixtures = {HAY_BALE, BARREL};
					break;
				case UseKind::Home:
					fixtures = {CHEST, BOOKSHELF};
					break;
				}
				if (fixtures.empty())
					continue;
				std::vector<std::pair<int, int>> candidates;
				if (unit_index >= unit_zone_cells.size())
					continue;
				for (const auto &[x, z] : unit_zone_cells[unit_index]) {
					const auto key = interior_cell_key(x, z);
					if (structural_cells.contains(key) || kept_cells.contains(key))
						continue;
					const auto claim = claimed_top.find(key);
					if (claim != claimed_top.end() && claim->second > floor_metres + 1.0)
						continue;
					if (building_passages.contains(x, z) &&
							floor_y < start_y_offset + std::min(BUILDING_PASSAGE_HEIGHT,
															   building_height))
						continue;
					candidates.emplace_back(x, z);
				}
				std::stable_sort(candidates.begin(), candidates.end(),
						[&](const auto &a, const auto &b) {
							const auto distance = [&](const auto &cell) {
								const auto dx = static_cast<std::int64_t>(cell.first) -
												unit.anchor.first;
								const auto dz = static_cast<std::int64_t>(cell.second) -
												unit.anchor.second;
								return dx * dx + dz * dz;
							};
							return std::pair(distance(a), a) < std::pair(distance(b), b);
						});
				std::size_t placed = 0;
				for (const auto &[x, z] : candidates) {
					const int y = floor_y + y_offset + abs_terrain_offset;
					if (editor.get_block_absolute(x, y, z))
						continue;
					editor.set_block_absolute(
							fixtures[placed], x, y, z, std::nullopt, std::nullopt);
					if (++placed == fixtures.size())
						break;
				}
			}
		}

		// Port Canvas::connect: identify every disconnected passable component,
		// then open its nearest built wall to the reached component.
		std::vector<std::pair<int, int>> starts;
		if (floor_index == 0) {
			for (const auto &entry : entries)
				starts.push_back(entry.cell);
		}
		if ((floor_index > 0 || entries.empty()) && shaft_open && shaft)
			starts.push_back(shaft->first);
		if (!starts.empty()) {
			std::unordered_set<std::uint64_t> passable_cells;
			passable_cells.reserve(floor_area.size());
			const double floor_metres =
					static_cast<double>(floor_y - start_y_offset) / std::max(scale, 0.01);
			for (const auto &[x, z] : floor_area) {
				const auto key = interior_cell_key(x, z);
				const auto claim = claimed_top.find(key);
				if (claim != claimed_top.end() && claim->second > floor_metres + 1.0)
					continue;
				if (building_passages.contains(x, z) &&
						floor_y < start_y_offset + std::min(BUILDING_PASSAGE_HEIGHT,
														   building_height))
					continue;
				passable_cells.insert(key);
			}
			auto passable = [&](std::uint64_t key,
									const std::unordered_set<std::uint64_t> &planned,
									const std::unordered_set<std::uint64_t> &doors) {
				return passable_cells.contains(key) &&
					   (!planned.contains(key) || doors.contains(key));
			};
			std::vector<std::pair<int, int>> cells;
			cells.reserve(passable_cells.size());
			for (const auto &[x, z] : floor_area)
				if (passable_cells.contains(interior_cell_key(x, z)))
					cells.emplace_back(x, z);
			std::sort(cells.begin(), cells.end(), [](const auto &a, const auto &b) {
				return std::pair(a.second, a.first) < std::pair(b.second, b.first);
			});
			for (int attempt = 0; attempt < 64; ++attempt) {
				std::unordered_set<std::uint64_t> planned = planned_partition_cells;
				std::unordered_set<std::uint64_t> doors;
				for (const auto &p : wall_positions)
					planned.insert(interior_cell_key(p.first, p.second));
				for (const auto &p : door_positions) {
					const auto key = interior_cell_key(p.first, p.second);
					planned.insert(key);
					doors.insert(key);
				}
				std::unordered_set<std::uint64_t> reached;
				std::deque<std::pair<int, int>> queue;
				for (const auto &start : starts) {
					const auto key = interior_cell_key(start.first, start.second);
					if (passable(key, planned, doors) && reached.insert(key).second)
						queue.push_back(start);
				}
				while (!queue.empty()) {
					const auto [x, z] = queue.front();
					queue.pop_front();
					for (const auto direction : INTERIOR_DIRS) {
						const int nx = x + direction.first, nz = z + direction.second;
						const auto key = interior_cell_key(nx, nz);
						if (passable(key, planned, doors) && reached.insert(key).second)
							queue.emplace_back(nx, nz);
					}
				}
				std::map<std::uint64_t, std::uint64_t> part;
				std::map<std::uint64_t,
						std::tuple<std::int64_t, std::int64_t, std::int64_t>>
						sums;
				for (const auto &[x, z] : cells) {
					const auto first = interior_cell_key(x, z);
					if (!passable(first, planned, doors) || reached.contains(first) ||
							part.contains(first))
						continue;
					part.emplace(first, first);
					std::int64_t sum_x = x, sum_z = z, count = 1;
					std::deque<std::pair<int, int>> component{std::pair{x, z}};
					while (!component.empty()) {
						const auto [cx, cz] = component.front();
						component.pop_front();
						for (const auto direction : INTERIOR_DIRS) {
							const int nx = cx + direction.first,
									  nz = cz + direction.second;
							const auto key = interior_cell_key(nx, nz);
							if (passable(key, planned, doors) && !reached.contains(key) &&
									part.emplace(key, first).second) {
								sum_x += nx;
								sum_z += nz;
								++count;
								component.emplace_back(nx, nz);
							}
						}
					}
					sums.emplace(first, std::tuple{sum_x, sum_z, count});
				}
				if (part.empty())
					break;
				using Opening = std::tuple<std::int64_t, std::pair<int, int>,
						std::pair<int, int>>;
				std::map<std::uint64_t, Opening> best;
				auto built_walls = wall_positions;
				std::sort(built_walls.begin(), built_walls.end(),
						[](const auto &a, const auto &b) { return a < b; });
				for (const auto &wall : built_walls) {
					const auto wall_key = interior_cell_key(wall.first, wall.second);
					if (doors.contains(wall_key))
						continue;
					for (const auto direction : INTERIOR_DIRS) {
						const auto from = interior_cell_key(wall.first - direction.first,
								wall.second - direction.second);
						const int ix = wall.first + direction.first;
						const int iz = wall.second + direction.second;
						const auto into = interior_cell_key(ix, iz);
						const auto component = part.find(into);
						if (component == part.end() || !reached.contains(from))
							continue;
						const auto &[sx, sz, count] = sums.at(component->second);
						const auto mx = sx / count, mz = sz / count;
						const auto dx = static_cast<std::int64_t>(wall.first) - mx;
						const auto dz = static_cast<std::int64_t>(wall.second) - mz;
						const auto distance = dx * dx + dz * dz;
						const Opening candidate{distance, wall, direction};
						const auto current = best.find(component->second);
						if (current == best.end() ||
								distance < std::get<0>(current->second))
							best.insert_or_assign(component->second, candidate);
					}
				}
				if (best.empty())
					break;
				for (const auto &[label, opening] : best) {
					(void)label;
					const auto &[distance, wall, across] = opening;
					(void)distance;
					const auto [x, z] = wall;
					wall_positions.erase(std::remove(wall_positions.begin(),
												 wall_positions.end(), wall),
							wall_positions.end());
					door_positions.emplace_back(wall);
					planned_partition_cells.insert(interior_cell_key(x, z));
					const char *facing = ladder_facing(across);
					const BlockWithProperties lower{DARK_OAK_DOOR_LOWER,
							{{"half", "lower"}, {"facing", facing}, {"hinge", "left"}}};
					const BlockWithProperties upper{DARK_OAK_DOOR_UPPER,
							{{"half", "upper"}, {"facing", facing}, {"hinge", "left"}}};
					editor.set_block_with_properties_absolute(lower, x,
							floor_y + y_offset + abs_terrain_offset, z, std::nullopt,
							std::nullopt);
					if (current_floor_ceiling >= floor_y + y_offset + 1)
						editor.set_block_with_properties_absolute(upper, x,
								floor_y + y_offset + abs_terrain_offset + 1, z,
								std::nullopt, std::nullopt);
				}
			}
		}

		// Extend walls all the way to the next floor ceiling or roof
		for (const auto &p : wall_positions) {
			int x = p.first;
			int z = p.second;
			for (int y = floor_y + y_offset + 2; y <= current_floor_ceiling; ++y) {
				editor.set_block_absolute(wall_block, x, y + abs_terrain_offset, z,
						std::nullopt, std::nullopt);
			}
		}

		// Add wall blocks above doors all the way to the ceiling/next floor
		for (const auto &p : door_positions) {
			int x = p.first;
			int z = p.second;
			for (int y = floor_y + y_offset + 2; y <= current_floor_ceiling; ++y) {
				editor.set_block_absolute(wall_block, x, y + abs_terrain_offset, z,
						std::nullopt, std::nullopt);
			}
		}

		// Port Canvas::light_up: keep light coverage local to connected rooms,
		// account for lights already present, hang lanterns in tall rooms, and
		// fill remaining dark cells using the same world-anchored cadence.
		std::unordered_set<std::uint64_t> wall_cells;
		for (const auto &p : wall_positions)
			wall_cells.insert(interior_cell_key(p.first, p.second));
		std::unordered_set<std::uint64_t> door_cells;
		for (const auto &p : door_positions)
			door_cells.insert(interior_cell_key(p.first, p.second));
		std::vector<std::pair<int, int>> open_cells;
		for (const auto &[x, z] : floor_area) {
			const auto key = interior_cell_key(x, z);
			if (wall_cells.contains(key) || door_cells.contains(key))
				continue;
			if (building_passages.contains(x, z) &&
					floor_y < start_y_offset +
									  std::min(BUILDING_PASSAGE_HEIGHT, building_height))
				continue;
			open_cells.emplace_back(x, z);
		}
		std::unordered_set<std::uint64_t> open_set;
		open_set.reserve(open_cells.size());
		for (const auto &[x, z] : open_cells)
			open_set.insert(interior_cell_key(x, z));
		std::unordered_map<std::uint64_t, std::size_t> component_of;
		std::size_t next_component = 0;
		for (const auto &cell : open_cells) {
			const auto start_key = interior_cell_key(cell.first, cell.second);
			if (component_of.contains(start_key))
				continue;
			const std::size_t component = next_component++;
			std::deque<std::pair<int, int>> queue{cell};
			component_of.emplace(start_key, component);
			while (!queue.empty()) {
				const auto [x, z] = queue.front();
				queue.pop_front();
				for (const auto direction : INTERIOR_DIRS) {
					const int nx = x + direction.first, nz = z + direction.second;
					const auto key = interior_cell_key(nx, nz);
					if (open_set.contains(key) && !component_of.contains(key)) {
						component_of.emplace(key, component);
						queue.emplace_back(nx, nz);
					}
				}
			}
		}
		const int top = current_floor_ceiling + abs_terrain_offset;
		const int absolute_floor = floor_y + abs_terrain_offset;
		const bool slab_above = floor_index + 1 < floor_levels.size();
		const int sunk = slab_above ? top + 1 : absolute_floor;
		auto is_light_block = [](const Block &block) {
			return block == GLOWSTONE || block == SEA_LANTERN || block == SHROOMLIGHT ||
				   block == LANTERN || block == SOUL_LANTERN || block == END_ROD;
		};
		std::unordered_set<std::uint64_t> lit;
		for (const auto &[x, z] : open_cells) {
			for (const int y : {top, sunk}) {
				const auto block = editor.get_block_absolute(x, y, z);
				if (block && is_light_block(*block)) {
					lit.insert(interior_cell_key(x, z));
					break;
				}
			}
		}
		const int headroom = top - absolute_floor;
		if (headroom >= 6) {
			const BlockWithProperties chain{CHAIN_X, {{"axis", "y"}}};
			const BlockWithProperties lantern{LANTERN, {{"hanging", "true"}}};
			for (const auto &[x, z] : open_cells) {
				if (floor_mod(x, 5) != 2 || floor_mod(z, 5) != 2)
					continue;
				if (editor.get_block_absolute(x, absolute_floor + 1, z))
					continue;
				for (int dy = 5; dy <= headroom; ++dy)
					editor.set_block_with_properties_absolute(
							chain, x, absolute_floor + dy, z, std::nullopt, std::nullopt);
				editor.set_block_with_properties_absolute(
						lantern, x, absolute_floor + 4, z, std::nullopt, std::nullopt);
				lit.insert(interior_cell_key(x, z));
			}
		}
		for (const auto &[x, z] : open_cells) {
			if (floor_mod(x + 2 * z, 5) != 0)
				continue;
			const auto component = component_of.at(interior_cell_key(x, z));
			bool near_light = false;
			for (int dx = -2; dx <= 2 && !near_light; ++dx)
				for (int dz = -2; dz <= 2; ++dz) {
					const auto nearby = interior_cell_key(x + dx, z + dz);
					const auto found = component_of.find(nearby);
					if (found != component_of.end() && found->second == component &&
							lit.contains(nearby)) {
						near_light = true;
						break;
					}
				}
			if (near_light)
				continue;
			if (headroom >= 3)
				editor.set_block_absolute(
						GLOWSTONE, x, top, z, std::nullopt, std::nullopt);
			else
				editor.set_block_absolute(GLOWSTONE, x, sunk, z,
						std::optional<std::vector<Block>>(
								std::vector<Block>{floor_block}),
						std::nullopt);
		}
		if (shaft_open && shaft && floor_index + 1 < floor_levels.size()) {
			const auto [cell, inward] = *shaft;
			const BlockWithProperties ladder{LADDER,
					{{"facing", ladder_facing(inward)}, {"waterlogged", "false"}}};
			const int ladder_bottom = floor_y + abs_terrain_offset + 1;
			const int ladder_top = floor_levels[floor_index + 1] + abs_terrain_offset;
			for (int y = ladder_bottom; y <= ladder_top; ++y)
				editor.set_block_with_properties_absolute(ladder, cell.first, y,
						cell.second, std::nullopt,
						std::optional<std::vector<Block>>(std::vector<Block>{}));
		}
	}
}
}
