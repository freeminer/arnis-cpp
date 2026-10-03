#include "cache_root.h"

#include <cstdlib>
#include <mutex>
#include <utility>

namespace arnis::cache
{
namespace
{
std::mutex base_mutex;
std::filesystem::path configured_base;

std::filesystem::path platform_base()
{
#if defined(_WIN32)
	if (const char *local = std::getenv("LOCALAPPDATA"); local && *local)
		return std::filesystem::path(local) / "arnis";
#elif defined(__APPLE__)
	if (const char *home = std::getenv("HOME"); home && *home)
		return std::filesystem::path(home) / "Library" / "Caches" / "arnis";
#endif
	if (const char *xdg = std::getenv("XDG_CACHE_HOME"); xdg && *xdg)
		return std::filesystem::path(xdg) / "arnis";
	if (const char *home = std::getenv("HOME"); home && *home)
		return std::filesystem::path(home) / ".cache" / "arnis";
	return std::filesystem::path("./.arnis_cache");
}
} // namespace

void set_base_directory(std::filesystem::path path)
{
	const std::lock_guard<std::mutex> lock(base_mutex);
	configured_base = std::move(path);
}

std::filesystem::path base_directory()
{
	const std::lock_guard<std::mutex> lock(base_mutex);
	return configured_base.empty() ? platform_base() : configured_base;
}

std::filesystem::path tile_cache_root()
{
	return base_directory() / "arnis-tile-cache";
}

std::filesystem::path provider_cache_root(const std::string &provider)
{
	return tile_cache_root() / provider;
}

std::filesystem::path facade_cache_root()
{
	return base_directory() / "earth" / "facade";
}
} // namespace arnis::cache
