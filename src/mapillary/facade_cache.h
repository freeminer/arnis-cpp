#pragma once

#include "types.h"

#include <filesystem>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace arnis::mapillary::cache
{
struct CachedWall
{
	WallProduct product;
	double reach_m{};
};

std::string wall_cache_key(const std::vector<std::int64_t> &node_ids, std::size_t piece);
std::vector<std::int64_t> wall_node_ids(const WallProduct &product);
std::optional<
		std::tuple<std::filesystem::path, std::filesystem::path, std::filesystem::path>>
wall_paths(const std::filesystem::path &directory, const std::string &key);
bool write_atomic(const std::filesystem::path &path,
		const std::vector<std::uint8_t> &bytes, std::string *error = nullptr);
std::optional<std::vector<std::uint8_t>> read_cached(const std::filesystem::path &path);
std::string store_wall(const std::filesystem::path &directory, const WallProduct &product,
		double reach_m = 0.0, std::string *error = nullptr);
std::optional<CachedWall> load_wall(
		const std::filesystem::path &directory, const std::string &key);
} // namespace arnis::mapillary::cache
