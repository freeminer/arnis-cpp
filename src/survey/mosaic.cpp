#include "mosaic.h"

#include "classify.h"
#include "fetch.h"
#include "laser.h"
#include "laser_cache.h"
#include "photo.h"
#include "plan.h"

#include "../net.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <utility>

namespace arnis::survey
{
namespace
{
constexpr double MAX_AREA_KM2 = 16.0;

bool copy_tile(classify::Inputs &input, const Plan &plan, const source::Tile &position,
		const laser::Tile &scan, const photo::Tile *image)
{
	const auto [east, north] = position;
	const double west = static_cast<double>(east) * laser::TILE;
	const double north_edge = static_cast<double>(north + 1) * laser::TILE;
	const auto width = input.w;
	const auto height = input.h;
	const double e0 = plan.rect[0];
	const double n1 = plan.rect[3];
	bool copied = false;
	for (std::size_t z = 0; z < laser::TILE; ++z) {
		const auto row = static_cast<std::int64_t>(std::floor(n1 - (north_edge - z)));
		if (row < 0 || row >= static_cast<std::int64_t>(height))
			continue;
		for (std::size_t x = 0; x < laser::TILE; ++x) {
			const auto column = static_cast<std::int64_t>(std::floor(west + x - e0));
			if (column < 0 || column >= static_cast<std::int64_t>(width))
				continue;
			const auto source_index = z * laser::TILE + x;
			const auto target_index = static_cast<std::size_t>(row) * width +
									  static_cast<std::size_t>(column);
			input.n_all[target_index] = scan.n_all[source_index];
			input.n_gnd[target_index] = scan.n_gnd[source_index];
			input.n_obj[target_index] = scan.n_obj[source_index];
			input.n_bld[target_index] = scan.n_bld[source_index];
			input.n_veg[target_index] = scan.n_veg[source_index];
			input.h_obj_q[target_index] = scan.h_obj[source_index];
			input.h_bld_q[target_index] = scan.h_bld[source_index];
			input.i_gnd_q[target_index] = scan.i_gnd[source_index];
			if (image && image->valid() && source_index < image->cells.size())
				input.photo[target_index] = image->cells[source_index];
			copied = true;
		}
	}
	return copied;
}
} // namespace

std::optional<SurveyData> fetch_cached_survey_data(const geographic::LLBBox &bbox,
		std::size_t grid_width, std::size_t grid_height, Features features,
		std::uint32_t max_megabytes, const std::filesystem::path &root)
{
	if (grid_width < 2 || grid_height < 2 ||
			grid_height > std::numeric_limits<std::size_t>::max() / grid_width)
		return std::nullopt;
	auto plan = plan_in(root, bbox);
	if (!plan || plan->too_large() || plan->area_km2() > MAX_AREA_KM2)
		return std::nullopt;
	const auto width_value = plan->rect[2] - plan->rect[0];
	const auto height_value = plan->rect[3] - plan->rect[1];
	if (!(width_value > 0.0) || !(height_value > 0.0) ||
			width_value > static_cast<double>(std::numeric_limits<std::size_t>::max()) ||
			height_value > static_cast<double>(std::numeric_limits<std::size_t>::max()))
		return std::nullopt;
	const auto width = static_cast<std::size_t>(width_value);
	const auto height = static_cast<std::size_t>(height_value);
	if (height > std::numeric_limits<std::size_t>::max() / width)
		return std::nullopt;
	classify::Inputs input(width, height);
	if (input.w != width || input.h != height)
		return std::nullopt;
	input.laser_plants = plan->sheet.source->laser_plants;
	input.strict_pools = plan->sheet.source->strict_pools;
	input.paving_brighter = plan->sheet.source->paving_brighter;
	const bool with_photo = features.garden || features.trees;
	std::map<source::Tile, std::vector<std::string>> tile_urls;
	bool lookup_failed = false;
	const auto &locate = plan->sheet.source->laser.locate;
	if (locate.kind == source::Locate::Kind::Urls) {
		if (locate.urls) {
			for (const auto &tile : plan->tiles)
				tile_urls[tile] = locate.urls(tile.first, tile.second);
		}
	} else {
		const auto has_uncached = std::any_of(
				plan->tiles.begin(), plan->tiles.end(), [&](const auto &tile) {
					return !laser_cache::is_cached(
							root, plan->sheet, tile.first, tile.second, plan->want(tile));
				});
		if (has_uncached && locate.lookup) {
			try {
				auto answer = locate.lookup(plan->sheet, plan->rect);
				if (const auto *error = std::get_if<std::string>(&answer)) {
					std::cerr << "Warning: laser scan index unavailable: " << *error
							  << '\n';
					net::note_uncached("laser scans");
					lookup_failed = true;
				} else {
					auto found = std::get<source::TileUrls>(std::move(answer));
					for (auto &[tile, url] : found) {
						tile_urls[tile].push_back(std::move(url));
					}
					for (auto &[tile, urls] : tile_urls) {
						std::sort(urls.begin(), urls.end());
						urls.erase(std::unique(urls.begin(), urls.end()), urls.end());
						const bool zipped = std::any_of(urls.begin(), urls.end(),
								[](const auto &url) { return laser_cache::is_zip(url); });
						laser_cache::note_whole(
								root, plan->sheet, tile.first, tile.second, zipped);
					}
					for (const auto &tile : plan->tiles) {
						if (tile_urls.contains(tile))
							continue;
						laser_cache::mark_absent(root, plan->sheet, tile.first,
								tile.second, plan->want(tile));
					}
				}
			} catch (const std::exception &error) {
				std::cerr << "Warning: laser scan index unavailable: " << error.what()
						  << '\n';
				net::note_uncached("laser scans");
				lookup_failed = true;
			}
		}
	}
	plan->recount(root);
	const auto need_mb = plan->need_mb(root, features);
	if (need_mb > max_megabytes) {
		std::cerr << "Laser data skipped: the area needs about " << need_mb
				  << " MB of laser scans and photos, more than the " << max_megabytes
				  << " MB allowed.\n";
		net::note_uncached("laser scans");
		return std::nullopt;
	}
	const bool fetch_photos = with_photo;
	fetch::limit_laser_bytes(static_cast<std::uint64_t>(max_megabytes) * 1'000'000);
	bool any_scan = false;
	for (const auto &position : plan->tiles) {
		const auto wanted = plan->want(position);
		const auto scan_path = laser_cache::cache_path(
				root, plan->sheet, position.first, position.second);
		auto scan = laser::Tile::read(scan_path);
		if (!scan || !scan->valid() || !scan->has(wanted)) {
			if (lookup_failed)
				continue;
			auto fetched = laser::fetch_tile(root, plan->sheet, position.first,
					position.second, wanted, tile_urls[position]);
			if (const auto *failure = std::get_if<fetch::Failure>(&fetched)) {
				std::cerr << "Warning: survey tile " << position.first << '_'
						  << position.second << " unavailable: " << failure->message
						  << '\n';
				net::note_uncached("laser scans");
				if (failure->message == fetch::PAST_LIMIT)
					return std::nullopt;
				continue;
			}
			scan = std::get<std::optional<laser::Tile>>(std::move(fetched));
		}
		if (!scan || !scan->valid() || !scan->has(wanted))
			continue;
		any_scan |= copy_tile(input, *plan, position, *scan, nullptr);
	}
	if (!any_scan)
		return std::nullopt;
	if (with_photo) {
		std::atomic_bool photos_down{false};
		for (const auto &position : plan->tiles) {
			const auto wanted = plan->want(position);
			auto scan = laser::Tile::read(laser_cache::cache_path(
					root, plan->sheet, position.first, position.second));
			if (!scan || !scan->valid() || !scan->has(wanted))
				continue;
			const double west = static_cast<double>(position.first) * laser::TILE;
			const double north = static_cast<double>(position.second + 1) * laser::TILE;
			const auto c0 = static_cast<std::size_t>(
					std::clamp(plan->rect[0] - west, 0.0, 1000.0));
			const auto c1 = static_cast<std::size_t>(
					std::clamp(plan->rect[2] - west, 0.0, 1000.0));
			const auto r0 = static_cast<std::size_t>(
					std::clamp(north - plan->rect[3], 0.0, 1000.0));
			const auto r1 = static_cast<std::size_t>(
					std::clamp(north - plan->rect[1], 0.0, 1000.0));
			std::optional<photo::Tile> image;
			if (fetch_photos) {
				image = photo::tile(root, plan->sheet, position.first, position.second,
						c0, r0, c1, r1, photos_down);
			} else {
				image = photo::read_cache(photo::cache_path(
						root, plan->sheet, position.first, position.second));
			}
			copy_tile(input, *plan, position, *scan,
					image && image->valid() ? &*image : nullptr);
		}
	}
	if (!plan->sheet.source->classes.unsorted.empty())
		classify::crowns_not_roofs(input);
	const auto thresholds =
			classify::calibrate(input, plan->sheet.source->intensity_defaults);
	const auto classes = classify::classify(input, thresholds);
	if (classes.size() != width * height)
		return std::nullopt;

	const double lat_step =
			(bbox.min().lat() - bbox.max().lat()) / static_cast<double>(grid_height - 1);
	const double lon_step =
			(bbox.max().lng() - bbox.min().lng()) / static_cast<double>(grid_width - 1);
	std::vector<std::array<std::uint8_t, 5>> cells(grid_width * grid_height);
	for (std::size_t z = 0; z < grid_height; ++z) {
		const double latitude = bbox.max().lat() + lat_step * z;
		for (std::size_t x = 0; x < grid_width; ++x) {
			const double longitude = bbox.min().lng() + lon_step * x;
			const auto [east, north] =
					plan->sheet.projection.forward(latitude, longitude);
			const double column = std::floor(east - plan->rect[0]);
			const double row = std::floor(plan->rect[3] - north);
			if (column < 0.0 || row < 0.0 || column >= width || row >= height)
				continue;
			const auto &value = classes[static_cast<std::size_t>(row) * width +
										static_cast<std::size_t>(column)];
			cells[z * grid_width + x] = {value.cls, value.height_q, value.color[0],
					value.color[1], value.color[2]};
		}
	}
	return SurveyData::from_cells(std::move(cells), grid_width, grid_height, features);
}
} // namespace arnis::survey
