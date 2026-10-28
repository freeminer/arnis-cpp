#include "atlas.h"

#include <algorithm>
#include <atomic>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <utility>

namespace arnis::mapillary::atlas
{
namespace
{
constexpr std::uint64_t budget_for(std::uint32_t side)
{
	return static_cast<std::uint64_t>(side) * side * 6 / 10;
}

std::atomic<std::uint64_t> active_budget{budget_for(ATLAS_SIDE_STANDARD)};
std::atomic<std::uint32_t> active_pixel_cap{std::numeric_limits<std::uint32_t>::max()};
std::mutex usage_mutex;
std::unordered_map<std::uint64_t, std::pair<std::uint64_t, std::uint64_t>> used_textures;
std::uint64_t used_area = 0;
std::uint64_t rejected = 0;
bool saturated = false;

std::uint64_t texture_key(const std::vector<std::uint8_t> &pixels, std::uint32_t width,
		std::uint32_t height)
{
	std::uint64_t hash = 14695981039346656037ull;
	auto add = [&](std::uint8_t byte) {
		hash ^= byte;
		hash *= 1099511628211ull;
	};
	for (int i = 0; i < 4; ++i) {
		add(static_cast<std::uint8_t>(width >> (i * 8)));
		add(static_cast<std::uint8_t>(height >> (i * 8)));
	}
	for (const auto byte : pixels)
		add(byte);
	return hash;
}
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
	reset_texture_usage();
}

void set_pixel_resolution_cap(std::uint32_t pixels_per_block)
{
	active_pixel_cap.store(
			std::max(MIN_PX_PER_BLOCK, pixels_per_block), std::memory_order_relaxed);
}

std::uint32_t pixel_resolution_cap()
{
	return active_pixel_cap.load(std::memory_order_relaxed);
}

std::uint64_t atlas_budget()
{
	return active_budget.load(std::memory_order_relaxed);
}

std::uint64_t used_texture_area()
{
	std::lock_guard lock(usage_mutex);
	return used_area;
}

void reset_texture_usage()
{
	std::lock_guard lock(usage_mutex);
	used_textures.clear();
	used_area = 0;
	rejected = 0;
	saturated = false;
	active_pixel_cap.store(
			std::numeric_limits<std::uint32_t>::max(), std::memory_order_relaxed);
}

bool reserve_texture(const std::vector<std::uint8_t> &pixels, std::uint32_t width,
		std::uint32_t height)
{
	if (!width || !height || pixels.size() != std::uint64_t(width) * height * 3)
		return false;
	const auto hash = texture_key(pixels, width, height);
	const std::uint64_t area = std::uint64_t(width) * height;
	const auto budget = atlas_budget();
	std::lock_guard lock(usage_mutex);
	if (auto found = used_textures.find(hash); found != used_textures.end()) {
		++found->second.first;
		return true;
	}
	// Match Rust's one-way admission cutoff: once the atlas cannot take the
	// next unique panel, do not make later smaller panels produce scattered gaps.
	if (saturated) {
		++rejected;
		return false;
	}
	if (area > budget || used_area > budget - area) {
		saturated = true;
		++rejected;
		return false;
	}
	used_textures.emplace(hash, std::pair<std::uint64_t, std::uint64_t>{1, area});
	used_area += area;
	return true;
}

bool admission_closed()
{
	std::lock_guard lock(usage_mutex);
	return saturated;
}

void release_texture(const std::vector<std::uint8_t> &pixels, std::uint32_t width,
		std::uint32_t height)
{
	if (!width || !height || pixels.size() != std::uint64_t(width) * height * 3)
		return;
	const auto hash = texture_key(pixels, width, height);
	std::lock_guard lock(usage_mutex);
	const auto found = used_textures.find(hash);
	if (found == used_textures.end())
		return;
	if (--found->second.first == 0) {
		used_area -= found->second.second;
		used_textures.erase(found);
	}
}

std::uint64_t rejected_textures()
{
	std::lock_guard lock(usage_mutex);
	return rejected;
}
} // namespace arnis::mapillary::atlas
