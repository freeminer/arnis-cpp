#pragma once
#include "provider.h"
#include "cache.h"
namespace arnis::elevation
{
class Selector
{
	CachedProvider cached_;
	GridCache cache_;

public:
	explicit Selector(std::filesystem::path root,
			providers::SourceMode mode = providers::SourceMode::Auto) :
			cached_(root, mode), cache_(std::move(root))
	{
	}
	providers::SourceMode source_mode() const { return cached_.mode(); }
	std::optional<double> sample(double lat, double lon);
	ElevationData raw_grid(double min_lat, double min_lon, double max_lat, double max_lon,
			std::size_t width, std::size_t height);
	ElevationData grid(double a, double b, double c, double d, std::size_t w,
			std::size_t h, const LandCoverRepairConfig &repair = {});
	ElevationData normalized_grid(double a, double b, double c, double d, std::size_t w,
			std::size_t h, double sea, double scale, double lo, double hi,
			const LandCoverRepairConfig &repair = {});
	ElevationData pipeline(double a, double b, double c, double d, std::size_t source_w,
			std::size_t source_h, std::size_t out_w, std::size_t out_h, double sea,
			double scale, double lo, double hi, const LandCoverRepairConfig &repair = {});
	SourceGrid source_pipeline(double a, double b, double c, double d, std::size_t width,
			std::size_t height, double sigma, double fallback);
	SourceGrid source_pipeline_normalized(double a, double b, double c, double d,
			std::size_t source_w, std::size_t source_h, std::size_t out_w,
			std::size_t out_h, double sigma, double fallback, double sea, double scale,
			double lo, double hi);
};
}
