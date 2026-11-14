#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

namespace arnis::scale::budget
{
inline constexpr std::size_t MIN_THREADS = 5;
inline constexpr std::size_t MIN_WORKERS = 2;
inline constexpr std::size_t TWO_WORKERS_FROM = 6;
inline constexpr std::size_t MAX_WORKERS = 6;
inline constexpr std::size_t CPU_TARGET_PERCENT = 90;
inline constexpr double AUTO_SINGLE_BELOW_KM2 = 100.0;
inline constexpr int AUTO_SIZES[] = {16, 12, 8, 4, 2};

// Approximate peak memory of a generated work unit, based on measured
// generation runs. `regions` counts 512x512-block Java regions.
std::uint64_t estimate_piece_mb(std::uint64_t regions, double scale);

// Choose a maximum square unit size from the available memory budget.
int auto_max_regions(std::uint64_t usable_memory_mb, double scale);

// Largest number of processes that fit both the thread and memory budgets.
std::size_t auto_plan(std::size_t budget_threads, std::size_t pieces,
		std::uint64_t usable_mb, std::uint64_t piece_mb);

struct Sizing
{
	std::size_t workers = 1;
	// All workers receive this per-process thread limit.
	std::size_t threads = 1;
	std::optional<std::uint64_t> ram_mb;
	std::uint32_t downloads = 1;
	// Set only for adaptive sizing, where runtime observations can lower workers.
	std::optional<std::uint64_t> usable_mb;

	std::size_t cap_for(std::uint64_t observed_peak_mb) const;
};

// Pure sizing counterpart of Rust's `scale::budget::sizing`. The caller owns
// system memory/core discovery and supplies resolved policy inputs so this
// helper remains usable by embedders without process-global dependencies.
Sizing sizing(std::size_t budget_threads, std::size_t pieces, std::uint64_t usable_mb,
		std::uint64_t piece_regions, double scale,
		std::optional<std::size_t> requested_workers, bool workers_auto,
		std::uint32_t download_allowance, bool ram_budget_was_explicit);

} // namespace arnis::scale::budget
