#include "selector.h"
namespace arnis::elevation
{
std::optional<double> Selector::sample(double lat, double lon)
{
	auto t = cached_.tile_for(lat, lon);
	if (t)
		if (auto v = sample_tile(*t, lat, lon))
			return v;
	for (int dl = -1; dl <= 1; ++dl)
		for (int dn = -1; dn <= 1; ++dn)
			if (dl || dn) {
				auto n = cached_.tile_for(lat + dl, lon + dn);
				if (n)
					if (auto v = sample_tile(*n, lat, lon))
						return v;
			}
	return std::nullopt;
}
ElevationData Selector::grid(
		double a, double b, double c, double d, std::size_t w, std::size_t h,
		const LandCoverRepairConfig &repair)
{
	auto tiles = tiles_for_bbox(a, b, c, d, cached_.root());
	return build_processed_grid(tiles, a, b, c, d, w, h, repair);
}
ElevationData Selector::normalized_grid(double a, double b, double c, double d,
		std::size_t w, std::size_t h, double sea, double scale, double lo, double hi,
		const LandCoverRepairConfig &repair)
{
	auto g = grid(a, b, c, d, w, h, repair);
	normalize_grid(g, sea, scale, lo, hi);
	return g;
}
ElevationData Selector::pipeline(double a, double b, double c, double d, std::size_t sw,
		std::size_t sh, std::size_t ow, std::size_t oh, double sea, double scale,
		double lo, double hi, const LandCoverRepairConfig &repair)
{
	auto key = grid_cache_key(a, b, c, d, sw, sh) + "_" + std::to_string(ow) + "x" +
			   std::to_string(oh);
	// Land-cover repair mutates both elevation and classification. Loading only
	// the cached heights would skip those classification updates, so this path
	// deliberately bypasses the old height-only cache.
	if (!repair) {
		if (auto cached = cache_.load(key))
			return *cached;
		if (auto compact = cache_.load_compact(key))
			return *compact;
	}
	auto g = normalized_grid(a, b, c, d, sw, sh, sea, scale, lo, hi, repair);
	auto out = resample_grid(g, ow, oh);
	if (!repair) {
		cache_.save(key, out);
		cache_.save_compact(key, out);
	}
	return out;
}
SourceGrid Selector::source_pipeline(double a, double b, double c, double d,
		std::size_t w, std::size_t h, double sigma, double fallback)
{
	auto key = grid_cache_key(a, b, c, d, w, h) + "_src";
	if (auto cached = cache_.load_sources(key))
		return *cached;
	auto g = build_source_grid(cached_, a, b, c, d, w, h);
	process_source_grid(g, sigma, fallback);
	cache_.save_sources(key, g);
	return g;
}
SourceGrid Selector::source_pipeline_normalized(double a, double b, double c, double d,
		std::size_t sw, std::size_t sh, std::size_t ow, std::size_t oh, double sigma,
		double fallback, double sea, double scale, double lo, double hi)
{
	auto g = source_pipeline(a, b, c, d, sw, sh, sigma, fallback);
	normalize_source_grid(g, sea, scale, lo, hi);
	return resample_source_grid(g, ow, oh);
}
}
