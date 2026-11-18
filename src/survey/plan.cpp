#include "plan.h"

#include "../cache_root.h"
#include "laser_cache.h"
#include "photo.h"
#include "survey.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arnis::survey
{
namespace
{
constexpr double MARGIN_M = 40.0;
constexpr double MAX_AREA_KM2 = 16.0;
constexpr std::uint64_t ZIP_MB = 200;

bool sliver(const source::Tile &tile, const source::Rect &rect)
{
	const auto [east, north] = tile;
	const double x = static_cast<double>(east) * 1000.0;
	const double y = static_cast<double>(north) * 1000.0;
	const double width = std::min(rect[2], x + 1000.0) - std::max(rect[0], x);
	const double height = std::min(rect[3], y + 1000.0) - std::max(rect[1], y);
	return std::min(width, height) <= MARGIN_M;
}

std::optional<std::uint64_t> estimate(const std::filesystem::path &root,
		const source::Sheet &sheet, const source::Tile &tile, const source::Rect &rect)
{
	const auto [east, north] = tile;
	const double km2 = laser_cache::missing_km2(
			root, sheet, east, north, laser_cache::blocks_for(east, north, rect));
	if (km2 <= 0.0)
		return std::nullopt;
	auto whole = [&](std::uint64_t typical_mb) {
		return laser_cache::noted_size(root, sheet, east, north)
				.value_or(typical_mb * 1'000'000);
	};
	switch (sheet.source->laser.kind) {
	case source::Laser::Kind::Whole:
		return whole(sheet.source->laser.tile_mb);
	case source::Laser::Kind::Copc:
		if (laser_cache::noted_whole(root, sheet, east, north))
			return whole(ZIP_MB);
		[[fallthrough]];
	case source::Laser::Kind::Ept:
		return static_cast<std::uint64_t>(std::ceil(
				km2 * static_cast<double>(sheet.source->laser.mb_per_km2) * 1'000'000.0));
	}
	return std::nullopt;
}

std::pair<std::vector<source::Tile>, std::uint32_t> lacking(
		const std::filesystem::path &root, const source::Sheet &sheet,
		const std::vector<source::Tile> &tiles, const source::Rect &rect)
{
	std::vector<source::Tile> missing;
	std::uint64_t total = 0;
	for (const auto &tile : tiles) {
		if (auto bytes = estimate(root, sheet, tile, rect)) {
			missing.push_back(tile);
			total += *bytes;
		}
	}
	const auto mb = (total + 999'999) / 1'000'000;
	return {std::move(missing), static_cast<std::uint32_t>(std::min<std::uint64_t>(
										mb, std::numeric_limits<std::uint32_t>::max()))};
}

std::uint64_t photo_bytes(const std::filesystem::path &root, const Plan &plan)
{
	double mb_per_km2 = 0.0;
	switch (plan.sheet.source->photo.kind) {
	case source::Photo::Kind::TiledTiff:
		mb_per_km2 = 80.0;
		break;
	case source::Photo::Kind::Sheets:
		mb_per_km2 = 90.0;
		break;
	case source::Photo::Kind::Wms:
		mb_per_km2 = 10.0;
		break;
	case source::Photo::Kind::None:
		return 0;
	}
	const auto [e0, n0, e1, n1] = plan.rect;
	std::uint64_t bytes = 0;
	for (const auto &[east, north] : plan.tiles) {
		const auto cache = photo::cache_path(root, plan.sheet, east, north);
		std::error_code ec;
		if (std::filesystem::exists(cache, ec) && !ec)
			continue;
		const double x = static_cast<double>(east) * 1000.0;
		const double y = static_cast<double>(north) * 1000.0;
		const double width = std::max(0.0, std::min(e1, x + 1000.0) - std::max(e0, x));
		const double height = std::max(0.0, std::min(n1, y + 1000.0) - std::max(n0, y));
		bytes += static_cast<std::uint64_t>(std::ceil(width * height * mb_per_km2));
	}
	return bytes;
}
} // namespace

double Plan::area_km2() const
{
	return (rect[2] - rect[0]) * (rect[3] - rect[1]) / 1'000'000.0;
}

bool Plan::too_large() const
{
	return area_km2() > MAX_AREA_KM2;
}

std::vector<bool> Plan::want(source::Tile tile) const
{
	return laser_cache::blocks_for(tile.first, tile.second, rect);
}

void Plan::recount(const std::filesystem::path &root)
{
	auto result = lacking(root, sheet, tiles, rect);
	missing = std::move(result.first);
	download_mb = result.second;
}

std::uint32_t Plan::need_mb(const std::filesystem::path &root, Features features) const
{
	const auto photos = features.garden || features.trees ? photo_bytes(root, *this) : 0;
	const auto photo_mb = (photos + 999'999) / 1'000'000;
	return static_cast<std::uint32_t>(
			std::min<std::uint64_t>(static_cast<std::uint64_t>(download_mb) + photo_mb,
					std::numeric_limits<std::uint32_t>::max()));
}

std::filesystem::path cache_root()
{
	return arnis::cache::base_directory() / "laser";
}

std::optional<Plan> plan(const geographic::LLBBox &bbox)
{
	return plan_in(cache_root(), bbox);
}

std::optional<Plan> plan_in(
		const std::filesystem::path &root, const geographic::LLBBox &bbox)
{
	const auto *coverage = source::for_bbox(bbox);
	if (!coverage)
		return std::nullopt;
	const double longitude = (bbox.min().lng() + bbox.max().lng()) / 2.0;
	Plan result;
	result.sheet = source::Sheet::at(*coverage, longitude);
	result.rect = result.sheet.rect(bbox, MARGIN_M);
	result.tiles = result.sheet.tiles(result.rect);
	if (result.sheet.source->laser.kind == source::Laser::Kind::Whole) {
		std::erase_if(result.tiles,
				[&](const auto &tile) { return sliver(tile, result.rect); });
	} else if (result.sheet.source->laser.kind == source::Laser::Kind::Copc) {
		std::erase_if(result.tiles, [&](const auto &tile) {
			return sliver(tile, result.rect) &&
				   laser_cache::noted_whole(root, result.sheet, tile.first, tile.second);
		});
	}
	result.recount(root);
	return result;
}
} // namespace arnis::survey
