#include "pipeline.h"

#include <algorithm>
#include <chrono>
#include <utility>

namespace arnis::mapillary
{
PipelineResult acquire_images(const Client &client, const PipelineConfig &config)
{
	PipelineResult result;
	// Keep the selected cache root visible to later facade stages and library
	// hosts, even when acquisition returns early (cache-only or cancelled).
	result.export_dir = config.facade_cache;
	const auto started = std::chrono::steady_clock::now();
	auto fetch_osm_data = [&] {
		if (!config.osm_fetch)
			return;
		std::string osm_error;
		result.osm = fetch_osm(*config.osm_fetch, &osm_error);
		if (!result.osm && !osm_error.empty())
			result.osm_error = std::move(osm_error);
	};
	// Cache-only runs must never fall through to the Graph client.  The full
	// facade cache reader is supplied by the host, so an empty result here is
	// preferable to violating Rust's no-network contract. Rust still fetches
	// OSM geometry in this mode, so keep that separate from Graph acquisition.
	if (config.cancelled() || config.cache_only) {
		result.cancelled = config.cancelled();
		result.cache_only = config.cache_only.has_value();
		if (!result.cancelled)
			fetch_osm_data();
		result.stats.total_seconds =
				std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
						.count();
		return result;
	}
	const auto cells = search_cells(config.bounds, config.maximum_cells);
	if (!cells) {
		result.stats.total_seconds =
				std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
						.count();
		return result;
	}
	result.cells = cells->size();
	if (config.cancelled()) {
		result.cancelled = true;
		result.stats.total_seconds =
				std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
						.count();
		return result;
	}
	result.images = client.search(
			config.bounds, config.endpoint, config.access_token, config.maximum_cells);
	if (result.images.empty())
		result.failed_cells = result.cells;
	std::sort(result.images.begin(), result.images.end(),
			[](const auto &a, const auto &b) { return a.id < b.id; });
	result.images.erase(
			std::unique(result.images.begin(), result.images.end(),
					[](const auto &a, const auto &b) { return a.id == b.id; }),
			result.images.end());
	result.stats.images = result.images.size();
	if (config.image_sink)
		for (const auto &image : result.images)
			config.image_sink(image);
	// OSM follows image metadata, and an OSM outage must not discard imagery.
	fetch_osm_data();
	result.stats.total_seconds =
			std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
					.count();
	result.stats.fetch_seconds = result.stats.total_seconds;
	return result;
}
} // namespace arnis::mapillary
