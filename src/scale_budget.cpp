#include "scale_budget.h"

#include <algorithm>
#include <limits>

namespace arnis::scale::budget
{
namespace
{
std::uint64_t piece_share(std::uint64_t memory_mb, std::size_t workers,
		std::uint64_t piece_mb, bool explicit_budget)
{
	const auto share = memory_mb / std::max<std::size_t>(workers, 1);
	if (share >= piece_mb)
		return share;
	return explicit_budget ? piece_mb : 0;
}

std::uint32_t per_worker_downloads(std::uint32_t allowance, std::size_t workers)
{
	if (allowance == 0)
		return 0;
	const auto divided =
			(std::uint64_t{2} * allowance / std::max<std::size_t>(workers, 1));
	return static_cast<std::uint32_t>(
			std::min<std::uint64_t>(std::max<std::uint64_t>(divided, 2), allowance));
}
} // namespace

std::uint64_t estimate_piece_mb(std::uint64_t regions, double scale)
{
	const std::uint64_t per_region = scale >= 0.5 ? 10 : 9;
	if (regions > (std::numeric_limits<std::uint64_t>::max() - 660) / per_region)
		return std::numeric_limits<std::uint64_t>::max();
	return 660 + per_region * regions;
}

int auto_max_regions(std::uint64_t usable_memory_mb, double scale)
{
	constexpr struct
	{
		int regions;
		std::size_t concurrent;
	} candidates[] = {
			{16, MAX_WORKERS}, {12, MAX_WORKERS}, {8, MIN_WORKERS}, {4, MIN_WORKERS}};
	for (const auto &candidate : candidates) {
		const auto piece_mb = estimate_piece_mb(
				static_cast<std::uint64_t>(candidate.regions) * candidate.regions, scale);
		if (piece_mb <= usable_memory_mb / candidate.concurrent)
			return candidate.regions;
	}
	return 2;
}

std::size_t auto_plan(std::size_t budget_threads, std::size_t pieces,
		std::uint64_t usable_mb, std::uint64_t piece_mb)
{
	const auto by_ram = usable_mb / std::max<std::uint64_t>(piece_mb, 1);
	if (budget_threads < TWO_WORKERS_FROM || by_ram == 0)
		return 1;
	const auto by_cpu = budget_threads / MIN_THREADS;
	return std::clamp<std::size_t>(
			std::min({pieces, by_cpu,
					static_cast<std::size_t>(std::min<std::uint64_t>(
							by_ram, std::numeric_limits<std::size_t>::max()))}),
			MIN_WORKERS, MAX_WORKERS);
}

std::size_t Sizing::cap_for(std::uint64_t observed_peak_mb) const
{
	if (!usable_mb)
		return workers;
	const auto fit = *usable_mb / std::max<std::uint64_t>(observed_peak_mb, 1);
	return std::clamp<std::size_t>(
			static_cast<std::size_t>(std::min<std::uint64_t>(fit, workers)), 1, workers);
}

Sizing sizing(std::size_t budget_threads, std::size_t pieces, std::uint64_t usable_mb,
		std::uint64_t piece_regions, double scale,
		std::optional<std::size_t> requested_workers, bool workers_auto,
		std::uint32_t download_allowance, bool ram_budget_was_explicit)
{
	budget_threads = std::max<std::size_t>(budget_threads, 1);
	pieces = std::max<std::size_t>(pieces, 1);
	const auto piece_mb = estimate_piece_mb(piece_regions, scale);
	const auto workers =
			workers_auto ? std::min(pieces,
								   auto_plan(budget_threads, pieces, usable_mb, piece_mb))
						 : std::min(pieces, std::max<std::size_t>(
													requested_workers.value_or(1), 1));

	Sizing result;
	result.workers = workers;
	result.threads = budget_threads;
	result.downloads = per_worker_downloads(download_allowance, workers);
	if (workers_auto) {
		result.usable_mb = usable_mb;
		const auto share =
				piece_share(usable_mb, workers, piece_mb, ram_budget_was_explicit);
		if (share != 0)
			result.ram_mb = share;
	} else if (workers > 1 || ram_budget_was_explicit) {
		const auto share =
				piece_share(usable_mb, workers, piece_mb, ram_budget_was_explicit);
		if (share != 0)
			result.ram_mb = share;
	}
	return result;
}

} // namespace arnis::scale::budget
