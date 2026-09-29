#pragma once

#include "api.h"

#include <cstddef>
#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <array>
#include <optional>
#include <string>
#include <vector>

namespace arnis::mapillary
{
// Rust's pipeline keeps acquisition separate from geometry/facade synthesis.
// This C++ contract exposes the same reusable first stage to library hosts.
struct PipelineConfig
{
	SearchCell bounds{};
	std::string endpoint = "https://graph.mapillary.com/images";
	std::string access_token;
	std::size_t maximum_cells = mapillary_max_cells;
	// Rust's later stages share these controls even when the caller uses a
	// cache-only run.  Keeping them in the C++ contract avoids silently losing
	// settings when a library host switches from acquisition to facades.
	std::filesystem::path facade_cache;
	std::size_t threads = 0;
	std::shared_ptr<std::atomic_bool> cancel;
	std::optional<std::string> cache_only;
	std::function<void(const cache::ImageRecord &)> image_sink;
	bool cancelled() const { return cancel && cancel->load(std::memory_order_relaxed); }
};

struct PipelineStats
{
	std::size_t images = 0;
	std::size_t images_downloaded = 0;
	std::size_t originals_downloaded = 0;
	std::size_t clusters = 0;
	std::size_t buildings = 0;
	std::size_t walls = 0;
	std::size_t reachable = 0;
	std::size_t walls_with_views = 0;
	std::array<std::size_t, 4> tiers{};
	std::size_t cache_hits = 0;
	std::size_t cache_misses = 0;
	double fetch_seconds = 0.0;
	double geometry_seconds = 0.0;
	double align_seconds = 0.0;
	double texture_seconds = 0.0;
	double export_seconds = 0.0;
	double total_seconds = 0.0;
};

struct PipelineResult
{
	std::vector<cache::ImageRecord> images;
	std::size_t cells = 0;
	std::size_t failed_cells = 0;
	bool cancelled = false;
	bool cache_only = false;
	PipelineStats stats;
	std::filesystem::path export_dir;
};

PipelineResult acquire_images(const Client &, const PipelineConfig &);
} // namespace arnis::mapillary
