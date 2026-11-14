#include "data_plan.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <system_error>
#include <utility>

namespace arnis::data_plan
{
namespace
{
constexpr double REGION_BAKE_PER_PBF = 2.0;
constexpr double BAKE_RAM_PER_PBF = 4.5;
constexpr double ARCHIVE_PER_PBF = 0.7;
constexpr double ARCHIVE_PER_PBF_MAX = 1.4;
constexpr double STORE_PER_PBF_MAX = 1.75;

std::uint64_t times(std::uint64_t bytes, double ratio)
{
	if (!std::isfinite(ratio) || ratio <= 0.0 || bytes == 0)
		return 0;
	const double result = static_cast<double>(bytes) * ratio;
	if (!std::isfinite(result) ||
			result >= static_cast<double>(std::numeric_limits<std::uint64_t>::max()))
		return std::numeric_limits<std::uint64_t>::max();
	return static_cast<std::uint64_t>(std::round(result));
}

std::uint64_t saturated_add(std::uint64_t a, std::uint64_t b)
{
	if (b > std::numeric_limits<std::uint64_t>::max() - a)
		return std::numeric_limits<std::uint64_t>::max();
	return a + b;
}
} // namespace

Item count_files(std::string source, const std::vector<std::filesystem::path> &files,
		std::uint64_t typical_bytes)
{
	Item item;
	item.source = std::move(source);
	item.total = files.size();
	for (const auto &file : files) {
		std::error_code ec;
		const auto status = std::filesystem::status(file, ec);
		if (ec || !std::filesystem::is_regular_file(status))
			continue;
		const auto bytes = std::filesystem::file_size(file, ec);
		if (ec)
			continue;
		++item.cached;
		item.cached_bytes = saturated_add(item.cached_bytes, bytes);
	}
	const auto bytes_per_file =
			item.cached ? item.cached_bytes / item.cached : typical_bytes;
	if (!files.empty()) {
		const auto missing = files.size() - item.cached;
		const auto missing_count = static_cast<std::uint64_t>(missing);
		item.missing_bytes =
				missing_count != 0 && bytes_per_file >
											  std::numeric_limits<std::uint64_t>::max() /
													  missing_count
						? std::numeric_limits<std::uint64_t>::max()
						: bytes_per_file * missing_count;
	}
	return item;
}

DataPlan count_cache(
		const std::filesystem::path &root, const std::vector<SourceFiles> &sources)
{
	DataPlan plan;
	plan.cache_root = root.string();
	plan.free_bytes = free_bytes(root);
	plan.items.reserve(sources.size());
	for (const auto &source : sources)
		plan.items.push_back(
				count_files(source.source, source.files, source.typical_bytes));
	return plan;
}

std::optional<std::uint64_t> free_bytes(const std::filesystem::path &path)
{
	std::error_code ec;
	for (auto candidate = path; !candidate.empty();) {
		if (!std::filesystem::exists(candidate, ec)) {
			ec.clear();
		} else {
			const auto space = std::filesystem::space(candidate, ec);
			if (!ec)
				return space.available;
			ec.clear();
		}
		const auto parent = candidate.parent_path();
		if (parent == candidate)
			break;
		candidate = parent;
	}
	return std::nullopt;
}

std::uint64_t region_bake_bytes(std::uint64_t pbf_bytes, double selection_share)
{
	if (!std::isfinite(selection_share))
		selection_share = 0.0;
	return times(pbf_bytes, REGION_BAKE_PER_PBF * std::clamp(selection_share, 0.0, 1.0));
}

std::uint64_t bake_ram_bytes(std::uint64_t pbf_bytes)
{
	return times(pbf_bytes, BAKE_RAM_PER_PBF);
}

std::uint64_t archive_bytes(std::uint64_t pbf_bytes)
{
	return times(pbf_bytes, ARCHIVE_PER_PBF);
}

std::uint64_t archive_peak_bytes(const std::vector<std::uint64_t> &pbf_sizes)
{
	auto sorted = pbf_sizes;
	std::sort(sorted.begin(), sorted.end(), std::greater<>());
	std::uint64_t done = 0;
	std::uint64_t peak = 0;
	for (std::size_t i = 0; i < sorted.size(); ++i) {
		std::uint64_t later = 0;
		for (std::size_t j = i + 1; j < sorted.size(); ++j)
			later = saturated_add(later, sorted[j]);
		const auto store = times(sorted[i], STORE_PER_PBF_MAX);
		const auto archive = times(sorted[i], ARCHIVE_PER_PBF_MAX);
		const auto current = saturated_add(saturated_add(done, later),
				saturated_add(store, std::max(sorted[i], archive)));
		peak = std::max(peak, current);
		done = saturated_add(done, times(sorted[i], ARCHIVE_PER_PBF_MAX));
	}
	return peak;
}
} // namespace arnis::data_plan
