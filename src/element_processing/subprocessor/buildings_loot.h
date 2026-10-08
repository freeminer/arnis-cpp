#pragma once
#include <cstdint>
#include <string>
#include <vector>
namespace arnis::buildings_loot
{
enum class LootTheme
{
	Mixed,
	Food,
	Resources,
	Tools,
	Valuables,
};

struct LootEntry
{
	std::string id;
	int slot;
	int count;
};
std::vector<LootEntry> chest_loot(int x, int z, std::uint32_t salt);
std::vector<LootEntry> themed_chest_loot(
		int x, int z, std::uint32_t salt, LootTheme theme);
}
