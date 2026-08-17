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
	explicit Selector(std::filesystem::path root) : cached_(root), cache_(std::move(root))
	{
	}
	std::optional<double> sample(double lat, double lon);
	ElevationData grid(
			double a, double b, double c, double d, std::size_t w, std::size_t h);
	ElevationData normalized_grid(double a, double b, double c, double d, std::size_t w,
			std::size_t h, double sea, double scale, double lo, double hi);
	ElevationData pipeline(double a, double b, double c, double d, std::size_t source_w,
			std::size_t source_h, std::size_t out_w, std::size_t out_h, double sea,
			double scale, double lo, double hi);
	SourceGrid source_pipeline(double a, double b, double c, double d, std::size_t width,
			std::size_t height, double sigma, double fallback);
	SourceGrid source_pipeline_normalized(double a, double b, double c, double d,
			std::size_t source_w, std::size_t source_h, std::size_t out_w,
			std::size_t out_h, double sigma, double fallback, double sea, double scale,
			double lo, double hi);
};
}
