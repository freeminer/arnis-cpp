#pragma once
#include "elevation.h"
#include "provider.h"
#include <filesystem>
#include <optional>
#include <utility>
#include <chrono>
#include <cstdint>
namespace arnis::elevation
{
// Matches cache.rs.  The caller supplies the application cache root so this
// library does not impose a platform-specific home-directory policy.
inline constexpr std::chrono::hours TILE_CACHE_MAX_AGE{24 * 30};
struct CacheClearStats
{
	std::uint64_t files_deleted = 0, bytes_freed = 0, errors = 0;
	CacheClearStats combined(const CacheClearStats &other) const
	{
		return {files_deleted + other.files_deleted, bytes_freed + other.bytes_freed,
				errors + other.errors};
	}
};
std::filesystem::path provider_cache_dir(
		const std::filesystem::path &base, const std::string &provider_name);
CacheClearStats clear_cache_dir(const std::filesystem::path &dir);
CacheClearStats cleanup_old_cached_files(const std::filesystem::path &dir,
		std::chrono::hours max_age = TILE_CACHE_MAX_AGE);
class GridCache
{
	std::filesystem::path root_;

public:
	explicit GridCache(std::filesystem::path root) : root_(std::move(root)) {}
	std::optional<ElevationData> load(const std::string &key) const;
	bool save(const std::string &key, const ElevationData &) const;
	bool save_compact(const std::string &key, const ElevationData &, double scale = 10.0,
			double offset = 0.0) const;
	std::optional<ElevationData> load_compact(const std::string &key) const;
	bool save_sources(const std::string &key, const SourceGrid &) const;
	std::optional<SourceGrid> load_sources(const std::string &key) const;
	void set_max_age(std::chrono::seconds age) { max_age_ = age; }
	std::size_t purge_stale() const;
	bool invalidate(const std::string &key) const;
	std::size_t clear() const;
	std::size_t bytes() const;

private:
	std::chrono::seconds max_age_{0};
};
std::string grid_cache_key(double min_lat, double min_lon, double max_lat, double max_lon,
		std::size_t width, std::size_t height);
}
