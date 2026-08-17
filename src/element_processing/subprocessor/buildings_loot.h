#pragma once
#include <string>
#include <vector>
namespace arnis::buildings_loot
{
struct LootEntry
{
	std::string id;
	int slot;
	int count;
};
std::vector<LootEntry> chest_loot(int x, int z, unsigned salt);
}
