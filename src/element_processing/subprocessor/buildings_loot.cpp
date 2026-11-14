#include "buildings_loot.h"
#include "../../deterministic_rng.h"
#include <nlohmann/json.hpp>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>

namespace arnis::buildings_loot
{
namespace
{
constexpr unsigned COMMON = 9, UNCOMMON = 3, RARE = 1, EMPTY_WEIGHT = 3;
constexpr std::uint32_t FAVOURED_FACTOR = 6;
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

std::mutex table_mutex;
std::shared_ptr<const LootTable> custom_table;
std::optional<std::filesystem::path> configured_path;
std::optional<std::filesystem::path> failed_path;
std::string failed_error;

const char *theme_name(LootTheme theme)
{
	switch (theme) {
	case LootTheme::Mixed:
		return "mixed";
	case LootTheme::Food:
		return "food";
	case LootTheme::Resources:
		return "resources";
	case LootTheme::Tools:
		return "tools";
	case LootTheme::Valuables:
		return "valuables";
	}
	return "mixed";
}

bool parse_theme(const nlohmann::json &value, LootTheme &out)
{
	if (!value.is_string())
		return false;
	const auto name = value.get<std::string>();
	for (const auto theme : {LootTheme::Mixed, LootTheme::Food, LootTheme::Resources,
				 LootTheme::Tools, LootTheme::Valuables}) {
		if (name == theme_name(theme)) {
			out = theme;
			return true;
		}
	}
	return false;
}

bool read_u32(const nlohmann::json &value, std::uint32_t &out)
{
	if (!value.is_number_integer() && !value.is_number_unsigned())
		return false;
	if (value.is_number_unsigned()) {
		const auto n = value.get<std::uint64_t>();
		if (n > std::numeric_limits<std::uint32_t>::max())
			return false;
		out = static_cast<std::uint32_t>(n);
		return true;
	}
	const auto n = value.get<std::int64_t>();
	if (n < 0 ||
			static_cast<std::uint64_t>(n) > std::numeric_limits<std::uint32_t>::max())
		return false;
	out = static_cast<std::uint32_t>(n);
	return true;
}

bool read_int(const nlohmann::json &value, int &out)
{
	if (!value.is_number_integer() && !value.is_number_unsigned())
		return false;
	if (value.is_number_unsigned()) {
		const auto n = value.get<std::uint64_t>();
		if (n > static_cast<std::uint64_t>(std::numeric_limits<int>::max()))
			return false;
		out = static_cast<int>(n);
		return true;
	}
	const auto n = value.get<std::int64_t>();
	if (n < std::numeric_limits<int>::min() || n > std::numeric_limits<int>::max())
		return false;
	out = static_cast<int>(n);
	return true;
}

std::shared_ptr<const LootTable> built_in_table()
{
	static const auto table = [] {
		auto result = std::make_shared<LootTable>();
		result->empty_weight = EMPTY_WEIGHT;
		result->rolls_min = 3;
		result->rolls_max = 8;
		for (const auto &theme : themes) {
			LootGroup group;
			group.kind = theme.kind;
			group.weight = theme.weight;
			group.items.reserve(theme.count);
			for (std::size_t i = 0; i < theme.count; ++i) {
				const auto &item = theme.items[i];
				group.items.push_back({item.id, item.min, item.max, item.weight});
			}
			result->themes.push_back(std::move(group));
		}
		return std::shared_ptr<const LootTable>(std::move(result));
	}();
	return table;
}

std::uint64_t group_weight(const LootGroup &group, LootTheme favored)
{
	return static_cast<std::uint64_t>(group.weight) *
		   (group.kind == favored && favored != LootTheme::Mixed ? FAVOURED_FACTOR : 1u);
}

std::optional<std::uint32_t> total_weight(const LootTable &table, LootTheme favored)
{
	std::uint64_t total = table.empty_weight;
	for (const auto &group : table.themes) {
		const auto weight = group_weight(group, favored);
		if (weight > std::numeric_limits<std::uint32_t>::max() - total)
			return std::nullopt;
		total += weight;
	}
	if (total == 0 || total > std::numeric_limits<std::uint32_t>::max())
		return std::nullopt;
	return static_cast<std::uint32_t>(total);
}

bool validate_table(const LootTable &table, std::string *error)
{
	auto fail = [&](std::string message) {
		if (error)
			*error = std::move(message);
		return false;
	};
	if (table.rolls_min > table.rolls_max || table.rolls_max > 64)
		return fail("rolls must satisfy rolls_min <= rolls_max <= 64");
	if (table.themes.empty())
		return fail("at least one theme is required");
	for (std::size_t ti = 0; ti < table.themes.size(); ++ti) {
		const auto &group = table.themes[ti];
		std::uint64_t item_total = 0;
		for (std::size_t ii = 0; ii < group.items.size(); ++ii) {
			const auto &item = group.items[ii];
			item_total += item.weight;
			if (item_total > std::numeric_limits<std::uint32_t>::max())
				return fail("theme " + std::to_string(ti) +
							": item weights add up past 2^32-1");
			if (item.id.find(':') == std::string::npos)
				return fail("theme " + std::to_string(ti) + " item " +
							std::to_string(ii) + ": '" + item.id +
							"' is not a namespaced id like minecraft:bread");
			if (item.min < 1 || item.min > item.max || item.max > 64)
				return fail("theme " + std::to_string(ti) + " item " +
							std::to_string(ii) +
							": count must satisfy 1 <= min <= max <= 64");
		}
		if (item_total == 0)
			return fail("theme " + std::to_string(ti) + ": no item has a weight above 0");
	}
	for (const auto favored : {LootTheme::Mixed, LootTheme::Food, LootTheme::Resources,
				 LootTheme::Tools, LootTheme::Valuables}) {
		if (!total_weight(table, favored))
			return fail(
					std::string(
							"theme weights plus empty_weight must be between 1 and 2^32-1 "
							"(also with ") +
					theme_name(favored) + " themes weighted 6x)");
	}
	return true;
}

std::shared_ptr<const LootTable> active_table()
{
	auto table = std::atomic_load(&custom_table);
	return table ? std::move(table) : built_in_table();
}

template <class R>
const TableItem &pick_item(const LootGroup &t, R &rng)
{
	std::uint32_t total = 0;
	for (const auto &item : t.items)
		total += item.weight;
	std::uint32_t p = rng.uniform(total);
	for (const auto &item : t.items) {
		if (p < item.weight)
			return item;
		p -= item.weight;
	}
	return t.items.back();
}
}

bool load_loot_table(
		const std::filesystem::path &path, LootTable &table, std::string *error)
{
	auto fail = [&](std::string message) {
		if (error)
			*error = std::move(message);
		return false;
	};
	try {
		std::ifstream input(path);
		if (!input)
			return fail("cannot read file");
		const auto root = nlohmann::json::parse(input);
		if (!root.is_object())
			return fail("root must be a JSON object");
		LootTable parsed;
		if (!root.contains("empty_weight") ||
				!read_u32(root.at("empty_weight"), parsed.empty_weight) ||
				!root.contains("rolls_min") ||
				!read_u32(root.at("rolls_min"), parsed.rolls_min) ||
				!root.contains("rolls_max") ||
				!read_u32(root.at("rolls_max"), parsed.rolls_max) ||
				!root.contains("themes") || !root.at("themes").is_array())
			return fail(
					"expected integer empty_weight, rolls_min, rolls_max and array themes");
		for (std::size_t ti = 0; ti < root.at("themes").size(); ++ti) {
			const auto &entry = root.at("themes")[ti];
			if (!entry.is_object() || !entry.contains("weight") ||
					!entry.contains("items") || !entry.at("items").is_array())
				return fail("theme " + std::to_string(ti) +
							": expected integer weight and array items");
			LootGroup group;
			if (!read_u32(entry.at("weight"), group.weight))
				return fail("theme " + std::to_string(ti) +
							": weight must be an unsigned integer");
			if (entry.contains("kind") && !parse_theme(entry.at("kind"), group.kind))
				return fail("theme " + std::to_string(ti) + ": unknown kind");
			for (std::size_t ii = 0; ii < entry.at("items").size(); ++ii) {
				const auto &source = entry.at("items")[ii];
				TableItem item;
				if (!source.is_object() || !source.contains("id") ||
						!source.at("id").is_string() || !source.contains("min") ||
						!read_int(source.at("min"), item.min) ||
						!source.contains("max") ||
						!read_int(source.at("max"), item.max) ||
						!source.contains("weight") ||
						!read_u32(source.at("weight"), item.weight))
					return fail("theme " + std::to_string(ti) + " item " +
								std::to_string(ii) + ": malformed item");
				item.id = source.at("id").get<std::string>();
				group.items.push_back(std::move(item));
			}
			parsed.themes.push_back(std::move(group));
		}
		std::string validation_error;
		if (!validate_table(parsed, &validation_error))
			return fail(std::move(validation_error));
		table = std::move(parsed);
		return true;
	} catch (const std::exception &e) {
		return fail(e.what());
	}
}

bool configure_loot_table(
		const std::optional<std::filesystem::path> &path, std::string *error)
{
	if (!path)
		return true;
	std::lock_guard lock(table_mutex);
	if (configured_path) {
		if (*configured_path == *path)
			return true;
		if (error)
			*error =
					"loot table was already configured from " + configured_path->string();
		return false;
	}
	if (failed_path && *failed_path == *path) {
		if (error)
			*error = failed_error;
		return false;
	}
	LootTable table;
	if (!load_loot_table(*path, table, error)) {
		failed_path = path;
		failed_error = error ? *error : "invalid loot table";
		return false;
	}
	std::atomic_store(&custom_table, std::make_shared<const LootTable>(std::move(table)));
	configured_path = path;
	return true;
}
std::vector<LootEntry> chest_loot(int x, int z, std::uint32_t salt)
{
	return themed_chest_loot(x, z, salt, LootTheme::Mixed);
}

std::vector<LootEntry> themed_chest_loot(
		int x, int z, std::uint32_t salt, LootTheme theme)
{
	const auto table = active_table();
	auto rng = coord_rng(x, z, std::uint64_t(salt) ^ 0x1007C0DEULL);
	const std::uint32_t theme_total = *total_weight(*table, theme);
	const std::uint32_t rolls =
			table->rolls_min + static_cast<std::uint32_t>(rng.uniform(
									   table->rolls_max - table->rolls_min + 1));
	bool used[27]{};
	std::vector<LootEntry> out;
	for (std::uint32_t i = 0; i < rolls; ++i) {
		std::uint32_t pick = rng.uniform(theme_total);
		if (pick < table->empty_weight)
			continue;
		pick -= table->empty_weight;
		const LootGroup *chosen = &table->themes[0];
		for (const LootGroup &candidate : table->themes) {
			const auto candidate_weight =
					static_cast<std::uint32_t>(group_weight(candidate, theme));
			if (pick < candidate_weight) {
				chosen = &candidate;
				break;
			}
			pick -= candidate_weight;
		}
		const TableItem &item = pick_item(*chosen, rng);
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
