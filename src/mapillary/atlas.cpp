#include "atlas.h"

#include <atomic>
#include <limits>

namespace arnis::mapillary::atlas
{
namespace
{
constexpr std::uint64_t budget_for(std::uint32_t side)
{
	return static_cast<std::uint64_t>(side) * side * 6 / 10;
}

std::atomic<std::uint64_t> active_budget{budget_for(ATLAS_SIDE_STANDARD)};
}

std::uint64_t atlas_budget_for(std::uint32_t side)
{
	return budget_for(side);
}

void set_atlas_side(std::uint32_t side)
{
	// A zero-sized atlas is not useful and would make every panel overflow. The
	// Rust implementation receives a validated setting; retaining the standard
	// budget here gives C++ library callers the same safe behavior for malformed
	// input without introducing a divide-by-zero path downstream.
	if (side == 0)
		side = ATLAS_SIDE_STANDARD;
	active_budget.store(budget_for(side), std::memory_order_relaxed);
}

std::uint64_t atlas_budget()
{
	return active_budget.load(std::memory_order_relaxed);
}
} // namespace arnis::mapillary::atlas
