#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace arnis::data_plan
{
struct Item
{
	std::string source;
	std::size_t cached = 0;
	std::uint64_t cached_bytes = 0;
	std::size_t total = 0;
	std::optional<std::uint64_t> missing_bytes;
};

struct DataPlan
{
	std::string cache_root;
	std::vector<Item> items;
	std::optional<std::uint64_t> free_bytes;
};

struct SourceFiles
{
	std::string source;
	std::vector<std::filesystem::path> files;
	std::uint64_t typical_bytes = 0;
};

// Build a cache-only inventory from the files a caller expects to read. Files
// that do not exist are treated as missing; this function never creates files
// or performs network requests.
Item count_files(std::string source, const std::vector<std::filesystem::path> &files,
		std::uint64_t typical_bytes);
DataPlan count_cache(
		const std::filesystem::path &root, const std::vector<SourceFiles> &sources);
std::optional<std::uint64_t> free_bytes(const std::filesystem::path &path);

std::uint64_t region_bake_bytes(std::uint64_t pbf_bytes, double selection_share);
std::uint64_t bake_ram_bytes(std::uint64_t pbf_bytes);
std::uint64_t archive_bytes(std::uint64_t pbf_bytes);
std::uint64_t archive_peak_bytes(const std::vector<std::uint64_t> &pbf_sizes);
} // namespace arnis::data_plan
