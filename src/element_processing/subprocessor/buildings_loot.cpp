#include "buildings_loot.h"
#include "../../deterministic_rng.h"
#include <array>
#include <algorithm>
namespace arnis::buildings_loot
{
struct Item
{
	const char *id;
	int min, max, weight;
};
const std::array<Item, 24> items{
		{{"minecraft:bread", 2, 6, 9}, {"minecraft:potato", 3, 9, 9},
				{"minecraft:apple", 2, 6, 9}, {"minecraft:paper", 2, 7, 9},
				{"minecraft:bone", 2, 7, 9}, {"minecraft:string", 2, 7, 9},
				{"minecraft:book", 1, 4, 9}, {"minecraft:oak_planks", 4, 16, 9},
				{"minecraft:cobblestone", 6, 20, 9}, {"minecraft:coal", 3, 9, 9},
				{"minecraft:torch", 2, 8, 9}, {"minecraft:iron_ingot", 2, 6, 3},
				{"minecraft:stick", 2, 7, 9}, {"minecraft:bucket", 1, 1, 3},
				{"minecraft:iron_pickaxe", 1, 1, 3}, {"minecraft:iron_axe", 1, 1, 3},
				{"minecraft:golden_apple", 1, 1, 1}, {"minecraft:ender_pearl", 1, 3, 1},
				{"minecraft:arrow", 4, 16, 9}, {"minecraft:leather", 2, 8, 9},
				{"minecraft:glass_pane", 3, 9, 9}, {"minecraft:candle", 1, 4, 3},
				{"minecraft:shield", 1, 1, 1}, {"minecraft:map", 1, 1, 3}}};
std::vector<LootEntry> chest_loot(int x, int z, unsigned salt)
{
	auto r = coord_rng(x, z, std::uint64_t(salt) ^ 0x1007C0DEULL);
	int rolls = 3 + r() % 6;
	std::array<bool, 27> used{};
	std::vector<LootEntry> out;
	for (int i = 0; i < rolls; ++i) {
		if (r() % 31 < 3)
			continue;
		unsigned total = 0;
		for (auto &it : items)
			total += it.weight;
		unsigned p = r() % total;
		const Item *pick = &items[0];
		for (auto &it : items) {
			if (p < it.weight) {
				pick = &it;
				break;
			}
			p -= it.weight;
		}
		int slot = -1;
		for (int j = 0; j < 4; ++j) {
			int c = r() % 27;
			if (!used[c]) {
				slot = c;
				break;
			}
		}
		if (slot < 0)
			continue;
		used[slot] = true;
		out.push_back({pick->id, slot,
				pick->min + int(r() % unsigned(pick->max - pick->min + 1))});
	}
	return out;
}
}
