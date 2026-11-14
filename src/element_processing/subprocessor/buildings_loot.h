#pragma once
#include <cstdint>
#include <filesystem>
#include <optional>
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
struct TableItem
{
	std::string id;
	int min = 1, max = 1;
	std::uint32_t weight = 0;
};
struct LootGroup
{
	LootTheme kind = LootTheme::Mixed;
	std::uint32_t weight = 0;
	std::vector<TableItem> items;
};
struct LootTable
{
	std::uint32_t empty_weight = 0, rolls_min = 0, rolls_max = 0;
	std::vector<LootGroup> themes;
};
// Loads and validates a custom table once, matching the Rust process-wide
// --loot-table configuration. An empty path retains the built-in table.
bool configure_loot_table(
		const std::optional<std::filesystem::path> &path, std::string *error = nullptr);
bool load_loot_table(const std::filesystem::path &path, LootTable &table,
		std::string *error = nullptr);
std::vector<LootEntry> chest_loot(int x, int z, std::uint32_t salt);
std::vector<LootEntry> themed_chest_loot(
		int x, int z, std::uint32_t salt, LootTheme theme);
}
