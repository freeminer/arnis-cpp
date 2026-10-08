#include "buildings_loot.h"
#include "../../deterministic_rng.h"
#include <cstddef>
#include <cstdint>
#include <iterator>

namespace arnis::buildings_loot
{
namespace
{
constexpr unsigned COMMON = 9, UNCOMMON = 3, RARE = 1, EMPTY_WEIGHT = 3;
struct Item
{
	const char *id;
	int min, max;
	unsigned weight;
};
struct Theme
{
	LootTheme kind;
	std::uint32_t weight;
	const Item *items;
	std::size_t count;
};
#define I(id, a, b, w)                                                                   \
	Item                                                                                 \
	{                                                                                    \
		id, a, b, w                                                                      \
	}
static constexpr Item food[] = {I("minecraft:bread", 2, 6, COMMON),
		I("minecraft:potato", 3, 9, COMMON), I("minecraft:carrot", 2, 7, COMMON),
		I("minecraft:wheat", 3, 9, COMMON), I("minecraft:apple", 2, 6, COMMON),
		I("minecraft:baked_potato", 2, 6, COMMON),
		I("minecraft:cooked_chicken", 1, 4, COMMON),
		I("minecraft:sweet_berries", 2, 7, COMMON), I("minecraft:beetroot", 2, 6, COMMON),
		I("minecraft:pumpkin_pie", 1, 3, UNCOMMON),
		I("minecraft:mushroom_stew", 1, 1, UNCOMMON),
		I("minecraft:golden_carrot", 1, 3, RARE), I("minecraft:cake", 1, 1, RARE)};
static constexpr Item junk[] = {I("minecraft:paper", 2, 7, COMMON),
		I("minecraft:bone", 2, 7, COMMON), I("minecraft:string", 2, 7, COMMON),
		I("minecraft:rotten_flesh", 2, 6, COMMON), I("minecraft:book", 1, 4, COMMON),
		I("minecraft:dead_bush", 1, 3, COMMON), I("minecraft:gunpowder", 1, 4, COMMON),
		I("minecraft:flower_pot", 1, 1, UNCOMMON), I("minecraft:cobweb", 1, 3, UNCOMMON),
		I("minecraft:name_tag", 1, 1, UNCOMMON), I("minecraft:map", 1, 1, UNCOMMON)};
static constexpr Item building[] = {I("minecraft:oak_planks", 4, 16, COMMON),
		I("minecraft:cobblestone", 6, 20, COMMON), I("minecraft:coal", 3, 9, COMMON),
		I("minecraft:clay_ball", 2, 7, COMMON), I("minecraft:glass_pane", 3, 9, COMMON),
		I("minecraft:torch", 2, 8, COMMON), I("minecraft:iron_ingot", 2, 6, UNCOMMON),
		I("minecraft:candle", 1, 4, UNCOMMON)};
static constexpr Item tools[] = {I("minecraft:stick", 2, 7, COMMON),
		I("minecraft:bucket", 1, 1, UNCOMMON), I("minecraft:fishing_rod", 1, 1, UNCOMMON),
		I("minecraft:shears", 1, 1, UNCOMMON),
		I("minecraft:flint_and_steel", 1, 1, UNCOMMON),
		I("minecraft:compass", 1, 1, UNCOMMON), I("minecraft:iron_pickaxe", 1, 1, RARE),
		I("minecraft:iron_axe", 1, 1, RARE), I("minecraft:clock", 1, 1, RARE)};
static constexpr Item valuables[] = {I("minecraft:iron_nugget", 3, 8, COMMON),
		I("minecraft:gold_nugget", 2, 7, UNCOMMON),
		I("minecraft:lapis_lazuli", 2, 6, UNCOMMON),
		I("minecraft:emerald", 1, 4, UNCOMMON), I("minecraft:gold_ingot", 1, 3, RARE),
		I("minecraft:amethyst_shard", 1, 4, RARE), I("minecraft:diamond", 1, 2, RARE)};
static constexpr Item adventure[] = {I("minecraft:arrow", 3, 12, COMMON),
		I("minecraft:leather_boots", 1, 1, UNCOMMON),
		I("minecraft:leather_chestplate", 1, 1, UNCOMMON),
		I("minecraft:shield", 1, 1, RARE), I("minecraft:golden_apple", 1, 1, RARE),
		I("minecraft:ender_pearl", 1, 3, RARE)};
static constexpr Theme themes[] = {{LootTheme::Food, 25, food, std::size(food)},
		{LootTheme::Mixed, 20, junk, std::size(junk)},
		{LootTheme::Resources, 18, building, std::size(building)},
		{LootTheme::Tools, 15, tools, std::size(tools)},
		{LootTheme::Valuables, 12, valuables, std::size(valuables)},
		{LootTheme::Mixed, 10, adventure, std::size(adventure)}};
#undef I
template <class R>
const Item &pick_item(const Theme &t, R &rng)
{
	std::uint32_t total = 0;
	for (std::size_t i = 0; i < t.count; ++i)
		total += t.items[i].weight;
	std::uint32_t p = rng.uniform(total);
	for (std::size_t i = 0; i < t.count; ++i) {
		if (p < t.items[i].weight)
			return t.items[i];
		p -= t.items[i].weight;
	}
	return t.items[t.count - 1];
}
}
std::vector<LootEntry> chest_loot(int x, int z, std::uint32_t salt)
{
	return themed_chest_loot(x, z, salt, LootTheme::Mixed);
}

std::vector<LootEntry> themed_chest_loot(
		int x, int z, std::uint32_t salt, LootTheme theme)
{
	constexpr std::uint32_t FAVOURED_FACTOR = 6;
	auto weight = [theme](const Theme &candidate) {
		return candidate.weight * (candidate.kind == theme && theme != LootTheme::Mixed
												  ? FAVOURED_FACTOR
												  : 1u);
	};
	auto rng = coord_rng(x, z, std::uint64_t(salt) ^ 0x1007C0DEULL);
	std::uint32_t theme_total = EMPTY_WEIGHT;
	for (const auto &candidate : themes)
		theme_total += weight(candidate);
	const std::uint32_t rolls = 3 + rng.uniform(6);
	bool used[27]{};
	std::vector<LootEntry> out;
	for (std::uint32_t i = 0; i < rolls; ++i) {
		std::uint32_t pick = rng.uniform(theme_total);
		if (pick < EMPTY_WEIGHT)
			continue;
		pick -= EMPTY_WEIGHT;
		const Theme *chosen = &themes[0];
		for (const Theme &candidate : themes) {
			const auto candidate_weight = weight(candidate);
			if (pick < candidate_weight) {
				chosen = &candidate;
				break;
			}
			pick -= candidate_weight;
		}
		const Item &item = pick_item(*chosen, rng);
		const int count =
				item.min + static_cast<int>(rng.uniform(
								   static_cast<std::uint32_t>(item.max - item.min + 1)));
		int slot = -1;
		for (int attempt = 0; attempt < 4; ++attempt) {
			const int candidate = static_cast<int>(rng.uniform(27));
			if (!used[candidate]) {
				slot = candidate;
				break;
			}
		}
		if (slot < 0)
			continue;
		used[slot] = true;
		out.push_back({item.id, slot, count});
	}
	return out;
}
}
