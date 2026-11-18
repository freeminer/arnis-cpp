#pragma once

#include "source.h"

#include <filesystem>
#include <optional>
#include <vector>

namespace arnis::survey
{
struct Features;

// A Rust-compatible survey plan: projected source rectangle, covered 1 km
// tiles, cache gaps, and the estimated laser/photo download volume.
struct Plan
{
	source::Sheet sheet;
	std::vector<source::Tile> tiles;
	source::Rect rect{};
	std::uint32_t download_mb = 0;
	std::vector<source::Tile> missing;

	double area_km2() const;
	bool too_large() const;
	std::vector<bool> want(source::Tile tile) const;
	void recount(const std::filesystem::path &root);
	std::uint32_t need_mb(const std::filesystem::path &root, Features features) const;
};

std::filesystem::path cache_root();
std::optional<Plan> plan(const geographic::LLBBox &bbox);
std::optional<Plan> plan_in(
		const std::filesystem::path &root, const geographic::LLBBox &bbox);
} // namespace arnis::survey
