#include "selector.h"
#include "../coordinate_system/geographic/llbbox.h"
#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace arnis::elevation
{
RawElevationGrid AwsElevationProvider::fetch_raw(
		const geographic::LLBBox &bbox, std::size_t width, std::size_t height) const
{
	const providers::GeoBBox bounds{
			bbox.min().lat(), bbox.min().lng(), bbox.max().lat(), bbox.max().lng()};
	return {providers::fetch_aws_terrain_grid(root_, bounds, width, height),
			std::nullopt};
}

RawElevationGrid MapterhornElevationProvider::fetch_raw(
		const geographic::LLBBox &bbox, std::size_t width, std::size_t height) const
{
	const providers::GeoBBox bounds{
			bbox.min().lat(), bbox.min().lng(), bbox.max().lat(), bbox.max().lng()};
	SourcePixels source_pixels;
	auto heights = providers::fetch_mapterhorn_terrain_grid(
			root_, bounds, width, height, &source_pixels);
	if (!heights)
		throw std::runtime_error("Mapterhorn returned no elevation grid");
	return {std::move(*heights), std::move(source_pixels)};
}

namespace
{
double non_finite_ratio(const std::vector<std::vector<double>> &heights)
{
	std::size_t total = 0;
	std::size_t missing = 0;
	for (const auto &row : heights)
		for (const double value : row) {
			++total;
			if (!std::isfinite(value))
				++missing;
		}
	return total ? static_cast<double>(missing) / total : 1.0;
}
} // namespace

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
ElevationData Selector::raw_grid(
		double a, double b, double c, double d, std::size_t w, std::size_t h)
{
	if (cached_.mode() == providers::SourceMode::Planetary) {
		if (!planetary_body_)
			throw std::invalid_argument(
					"Planetary elevation mode requires a celestial body");
		const geographic::LLBBox bbox(a, b, c, d);
		PlanetaryElevationProvider provider(*planetary_body_, cached_.root());
		auto raw = provider.fetch_raw(bbox, w, h);
		ElevationData out;
		out.width = w;
		out.height = h;
		out.world_width = w;
		out.world_height = h;
		out.heights = std::move(raw.heights_meters);
		out.source_pixels = std::move(raw.source_pixels);
		return out;
	}
	if (cached_.mode() == providers::SourceMode::AwsOnly ||
			cached_.mode() == providers::SourceMode::Auto) {
		AwsElevationProvider aws(cached_.root());
		MapterhornElevationProvider mapterhorn(cached_.root());
		const geographic::LLBBox bbox(a, b, c, d);
		std::vector<std::reference_wrapper<const ElevationProvider>> chain;
		if (cached_.mode() == providers::SourceMode::Auto)
			chain.emplace_back(mapterhorn);
		chain.emplace_back(aws);
		auto raw = fetch_raw_with_fallback(chain, bbox, w, h);
		ElevationData out;
		out.width = w;
		out.height = h;
		out.world_width = w;
		out.world_height = h;
		out.heights = std::move(raw.heights_meters);
		out.source_pixels = std::move(raw.source_pixels);
		return out;
	}
	if (cached_.mode() != providers::SourceMode::Planetary) {
		std::optional<std::vector<std::vector<double>>> heights;
		SourcePixels source_pixels;
		std::string fetch_error;
		try {
			heights = providers::fetch_mapterhorn_terrain_grid(
					cached_.root(), {a, b, c, d}, w, h, &source_pixels);
		} catch (const std::exception &error) {
			fetch_error = error.what();
		} catch (...) {
			fetch_error = "unknown error";
		}
		if (heights) {
			const double missing_ratio = non_finite_ratio(*heights);
			if (cached_.mode() != providers::SourceMode::Auto || missing_ratio <= 0.5) {
				ElevationData out;
				out.width = w;
				out.height = h;
				out.world_width = w;
				out.world_height = h;
				out.heights = std::move(*heights);
				out.source_pixels = std::move(source_pixels);
				return out;
			}
			std::cerr << "Warning: Elevation provider 'mapterhorn' returned "
					  << std::round(missing_ratio * 100.0)
					  << "% empty data. Falling back to 'aws'.\n";
		}
		if (cached_.mode() == providers::SourceMode::Auto) {
			if (!fetch_error.empty())
				std::cerr << "Warning: Elevation provider 'mapterhorn' failed: "
						  << fetch_error << ". Falling back to 'aws'.\n";
			else if (!heights)
				std::cerr << "Warning: Elevation provider 'mapterhorn' failed. "
						  << "Falling back to 'aws'.\n";
			ElevationData out;
			out.width = w;
			out.height = h;
			out.world_width = w;
			out.world_height = h;
			out.heights =
					providers::fetch_aws_terrain_grid(cached_.root(), {a, b, c, d}, w, h);
			return out;
		}
	}
	auto tiles = tiles_for_bbox(a, b, c, d, cached_.root(), cached_.mode());
	return build_grid(tiles, a, b, c, d, w, h);
}
ElevationData Selector::grid(double a, double b, double c, double d, std::size_t w,
		std::size_t h, const LandCoverRepairConfig &repair)
{
	auto tiles = tiles_for_bbox(a, b, c, d, cached_.root(), cached_.mode());
	return build_processed_grid(tiles, a, b, c, d, w, h, repair, cached_.mode());
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
