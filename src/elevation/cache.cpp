#include "cache.h"
#include "provider.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <limits>
#include <thread>
#include <iostream>
#include <fstream>
namespace arnis::elevation
{
namespace
{
void add_saturating(std::uint64_t &total, std::uint64_t value)
{
	if (std::numeric_limits<std::uint64_t>::max() - total < value)
		total = std::numeric_limits<std::uint64_t>::max();
	else
		total += value;
}

std::uint64_t directory_size_bytes(const std::filesystem::path &dir)
{
	std::error_code ec;
	std::filesystem::directory_iterator it(dir, ec), end;
	if (ec)
		return 0;
	std::uint64_t total = 0;
	for (; it != end; it.increment(ec)) {
		if (ec) {
			ec.clear();
			continue;
		}
		const auto path = it->path();
		const auto type = it->symlink_status(ec);
		if (ec) {
			ec.clear();
			continue;
		}
		if (std::filesystem::is_symlink(type)) {
			// Count the stored target text, not the bytes in its target file.
			// On POSIX this is the symlink's on-disk length and never follows it.
			const auto target = std::filesystem::read_symlink(path, ec);
			if (!ec) {
				const auto chars = target.native().size();
				const auto bytes =
						chars > std::numeric_limits<std::uint64_t>::max() /
												sizeof(std::filesystem::path::value_type)
								? std::numeric_limits<std::uint64_t>::max()
								: static_cast<std::uint64_t>(chars) *
										  sizeof(std::filesystem::path::value_type);
				add_saturating(total, bytes);
			} else {
				ec.clear();
			}
		} else if (std::filesystem::is_directory(type)) {
			add_saturating(total, directory_size_bytes(path));
		} else if (std::filesystem::is_regular_file(type)) {
			const auto bytes = std::filesystem::file_size(path, ec);
			if (!ec)
				add_saturating(total, bytes);
			else
				ec.clear();
		}
	}
	return total;
}

void clear_recursive(const std::filesystem::path &dir, CacheClearStats &stats)
{
	std::error_code ec;
	std::filesystem::directory_iterator it(dir, ec), end;
	if (ec) {
		++stats.errors;
		return;
	}
	for (; it != end; it.increment(ec)) {
		if (ec) {
			++stats.errors;
			ec.clear();
			continue;
		}
		const auto path = it->path();
		const auto type = it->symlink_status(ec);
		if (ec) {
			++stats.errors;
			ec.clear();
			continue;
		}
		if (std::filesystem::is_symlink(type)) {
			std::filesystem::remove(path, ec);
			if (ec)
				++stats.errors;
			continue;
		}
		if (std::filesystem::is_directory(type)) {
			clear_recursive(path, stats);
			std::filesystem::remove(path, ec);
			if (ec)
				++stats.errors;
			continue;
		}
		if (!std::filesystem::is_regular_file(type))
			continue;
		const auto bytes = std::filesystem::file_size(path, ec);
		if (ec)
			ec.clear();
		std::filesystem::remove(path, ec);
		if (ec)
			++stats.errors;
		else {
			++stats.files_deleted;
			stats.bytes_freed += bytes;
		}
	}
}
void cleanup_recursive(const std::filesystem::path &dir,
		std::filesystem::file_time_type now,
		std::filesystem::file_time_type::duration max_age, CacheClearStats &stats)
{
	std::error_code ec;
	std::filesystem::directory_iterator it(dir, ec), end;
	if (ec) {
		++stats.errors;
		return;
	}
	for (; it != end; it.increment(ec)) {
		if (ec) {
			++stats.errors;
			ec.clear();
			continue;
		}
		const auto path = it->path();
		const auto type = it->symlink_status(ec);
		if (ec) {
			++stats.errors;
			ec.clear();
			continue;
		}
		if (std::filesystem::is_symlink(type))
			continue;
		if (std::filesystem::is_directory(type)) {
			cleanup_recursive(path, now, max_age, stats);
			continue;
		}
		if (!std::filesystem::is_regular_file(type))
			continue;
		const auto modified = std::filesystem::last_write_time(path, ec);
		if (ec || now < modified || now - modified <= max_age) {
			ec.clear();
			continue;
		}
		const auto bytes = std::filesystem::file_size(path, ec);
		if (ec)
			ec.clear();
		std::filesystem::remove(path, ec);
		if (ec)
			++stats.errors;
		else {
			++stats.files_deleted;
			stats.bytes_freed += bytes;
		}
	}
}
}
std::filesystem::path provider_cache_dir(
		const std::filesystem::path &base, const std::string &provider_name)
{
	return base / "arnis-tile-cache" / provider_name;
}
CacheClearStats clear_cache_dir(const std::filesystem::path &dir)
{
	CacheClearStats stats;
	std::error_code ec;
	const auto type = std::filesystem::symlink_status(dir, ec);
	if (ec && ec == std::errc::no_such_file_or_directory)
		return stats;
	if (ec || std::filesystem::is_symlink(type) || !std::filesystem::is_directory(type)) {
		++stats.errors;
		return stats;
	}
	clear_recursive(dir, stats);
	return stats;
}
std::uint64_t dir_size_bytes(const std::filesystem::path &dir)
{
	std::error_code ec;
	const auto type = std::filesystem::symlink_status(dir, ec);
	if (ec || !std::filesystem::is_directory(type))
		return 0;
	return directory_size_bytes(dir);
}
std::string format_size(std::uint64_t bytes)
{
	constexpr double KB = 1024.0;
	constexpr double MB = KB * 1024.0;
	constexpr double GB = MB * 1024.0;
	const double value = static_cast<double>(bytes);
	std::ostringstream out;
	if (value >= GB)
		out << std::fixed << std::setprecision(1) << value / GB << " GB";
	else if (value >= MB)
		out << std::fixed << std::setprecision(0) << value / MB << " MB";
	else if (value >= KB)
		out << std::fixed << std::setprecision(0) << value / KB << " KB";
	else
		out << bytes << " B";
	return out.str();
}
CacheClearStats cleanup_old_cached_files(
		const std::filesystem::path &dir, std::chrono::hours age)
{
	CacheClearStats stats;
	std::error_code ec;
	const auto type = std::filesystem::symlink_status(dir, ec);
	if (ec && ec == std::errc::no_such_file_or_directory)
		return stats;
	if (ec || std::filesystem::is_symlink(type) || !std::filesystem::is_directory(type)) {
		if (ec)
			++stats.errors;
		return stats;
	}
	const auto duration =
			std::chrono::duration_cast<std::filesystem::file_time_type::duration>(age);
	cleanup_recursive(
			dir, std::filesystem::file_time_type::clock::now(), duration, stats);
	return stats;
}
void spawn_throttled_cleanup(const std::filesystem::path &base_cache_dir)
{
	std::error_code ec;
	const auto root_type = std::filesystem::symlink_status(base_cache_dir, ec);
	if (ec || !std::filesystem::is_directory(root_type) ||
			std::filesystem::is_symlink(root_type))
		return;

	constexpr auto cleanup_interval = std::chrono::hours(24);
	const auto stamp = base_cache_dir / ".last-cleanup";
	const auto stamp_type = std::filesystem::symlink_status(stamp, ec);
	if (!ec && std::filesystem::is_symlink(stamp_type)) {
		std::filesystem::remove(stamp, ec);
		ec.clear();
	} else if (!ec && std::filesystem::is_regular_file(stamp_type)) {
		const auto modified = std::filesystem::last_write_time(stamp, ec);
		if (!ec) {
			const auto now = std::filesystem::file_time_type::clock::now();
			// A future timestamp (clock rollback) is treated as stale and replaced.
			if (now >= modified && now - modified < cleanup_interval)
				return;
		}
		ec.clear();
	} else {
		ec.clear();
	}

	// A failed stamp write means cleanup would be retried on every mapchunk.
	// Skip this sweep instead; a later generation can retry once writable.
	{
		std::ofstream marker(stamp, std::ios::binary | std::ios::trunc);
		if (!marker)
			return;
	}

	std::thread([base_cache_dir]() {
		const auto stats = cleanup_old_cached_files(base_cache_dir, TILE_CACHE_MAX_AGE);
		if (stats.files_deleted > 0)
			std::cout << "Cleaned up " << stats.files_deleted
					  << " old cached elevation files (older than "
					  << TILE_CACHE_MAX_AGE.count() / 24 << " days)\n";
		if (stats.errors > 1)
			std::cerr << "Warning: Failed to delete " << stats.errors
					  << " old cached files\n";
	}).detach();
}
std::optional<ElevationData> GridCache::load(const std::string &k) const
{
	auto p = root_ / (k + ".grid");
	if (!std::filesystem::exists(p))
		return std::nullopt;
	if (max_age_.count() > 0) {
		auto t = std::filesystem::last_write_time(p);
		auto now = std::filesystem::file_time_type::clock::now();
		if (now - t > max_age_)
			return std::nullopt;
	}
	std::ifstream f(p, std::ios::binary);
	if (!f)
		return std::nullopt;
	ElevationData g;
	f.read(reinterpret_cast<char *>(&g.width), sizeof(g.width));
	f.read(reinterpret_cast<char *>(&g.height), sizeof(g.height));
	if (!f || g.width > 4096 || g.height > 4096)
		return std::nullopt;
	g.heights.assign(g.height, std::vector<double>(g.width));
	for (auto &r : g.heights)
		f.read(reinterpret_cast<char *>(r.data()),
				std::streamsize(r.size() * sizeof(double)));
	if (!f)
		return std::nullopt;
	for (const auto &r : g.heights)
		for (double v : r)
			if (!std::isfinite(v))
				return std::nullopt;
	return g;
}
bool GridCache::save(const std::string &k, const ElevationData &g) const
{
	std::filesystem::create_directories(root_);
	auto tmp = root_ / (k + ".tmp");
	auto dst = root_ / (k + ".grid");
	std::ofstream f(tmp, std::ios::binary);
	if (!f)
		return false;
	f.write(reinterpret_cast<const char *>(&g.width), sizeof(g.width));
	f.write(reinterpret_cast<const char *>(&g.height), sizeof(g.height));
	for (const auto &r : g.heights)
		f.write(reinterpret_cast<const char *>(r.data()),
				std::streamsize(r.size() * sizeof(double)));
	f.close();
	if (!f)
		return false;
	std::error_code ec;
	std::filesystem::rename(tmp, dst, ec);
	if (ec) {
		std::filesystem::remove(tmp);
		return false;
	}
	return true;
}
bool GridCache::save_compact(
		const std::string &k, const ElevationData &g, double scale, double off) const
{
	std::filesystem::create_directories(root_);
	auto p = root_ / (k + ".h16");
	std::ofstream f(p, std::ios::binary);
	if (!f)
		return false;
	f.write(reinterpret_cast<const char *>(&g.width), sizeof(g.width));
	f.write(reinterpret_cast<const char *>(&g.height), sizeof(g.height));
	f.write(reinterpret_cast<const char *>(&scale), sizeof(scale));
	f.write(reinterpret_cast<const char *>(&off), sizeof(off));
	auto v = encode_heightmap(g, scale, off);
	f.write(reinterpret_cast<const char *>(v.data()),
			std::streamsize(v.size() * sizeof(std::int16_t)));
	return bool(f);
}
bool GridCache::save_sources(const std::string &k, const SourceGrid &g) const
{
	std::ofstream f(root_ / (k + ".src"), std::ios::binary);
	if (!f)
		return false;
	f.write(reinterpret_cast<const char *>(&g.data.width), sizeof(g.data.width));
	f.write(reinterpret_cast<const char *>(&g.data.height), sizeof(g.data.height));
	for (const auto &r : g.data.heights)
		f.write(reinterpret_cast<const char *>(r.data()),
				std::streamsize(r.size() * sizeof(double)));
	for (const auto &r : g.sources)
		f.write(reinterpret_cast<const char *>(r.data()), std::streamsize(r.size()));
	return bool(f);
}
std::optional<SourceGrid> GridCache::load_sources(const std::string &k) const
{
	std::ifstream f(root_ / (k + ".src"), std::ios::binary);
	if (!f)
		return std::nullopt;
	SourceGrid g;
	f.read(reinterpret_cast<char *>(&g.data.width), sizeof(g.data.width));
	f.read(reinterpret_cast<char *>(&g.data.height), sizeof(g.data.height));
	if (!f || g.data.width > 4096 || g.data.height > 4096)
		return std::nullopt;
	g.data.heights.assign(g.data.height, std::vector<double>(g.data.width));
	for (auto &r : g.data.heights)
		f.read(reinterpret_cast<char *>(r.data()),
				std::streamsize(r.size() * sizeof(double)));
	g.sources.assign(g.data.height, std::vector<providers::Source>(g.data.width));
	for (auto &r : g.sources)
		f.read(reinterpret_cast<char *>(r.data()), std::streamsize(r.size()));
	return f ? std::optional<SourceGrid>(std::move(g)) : std::nullopt;
}
std::optional<ElevationData> GridCache::load_compact(const std::string &k) const
{
	std::ifstream f(root_ / (k + ".h16"), std::ios::binary);
	if (!f)
		return std::nullopt;
	std::size_t w, h;
	double s, o;
	f.read(reinterpret_cast<char *>(&w), sizeof(w));
	f.read(reinterpret_cast<char *>(&h), sizeof(h));
	f.read(reinterpret_cast<char *>(&s), sizeof(s));
	f.read(reinterpret_cast<char *>(&o), sizeof(o));
	if (!f || w > 4096 || h > 4096 || w * h > std::size_t(4096) * 4096)
		return std::nullopt;
	std::vector<std::int16_t> v(w * h);
	f.read(reinterpret_cast<char *>(v.data()),
			std::streamsize(v.size() * sizeof(std::int16_t)));
	if (!f)
		return std::nullopt;
	return decode_heightmap(v, w, h, s, o);
}
std::size_t GridCache::purge_stale() const
{
	if (max_age_.count() <= 0 || !std::filesystem::exists(root_))
		return 0;
	std::size_t n = 0;
	auto now = std::filesystem::file_time_type::clock::now();
	for (const auto &e : std::filesystem::directory_iterator(root_))
		if (e.path().extension() == ".grid" && now - e.last_write_time() > max_age_) {
			std::error_code ec;
			std::filesystem::remove(e.path(), ec);
			if (!ec)
				++n;
		}
	return n;
}
bool GridCache::invalidate(const std::string &k) const
{
	std::error_code ec;
	bool a = std::filesystem::remove(root_ / (k + ".grid"), ec);
	ec.clear();
	bool b = std::filesystem::remove(root_ / (k + ".h16"), ec);
	return a || b;
}
std::size_t GridCache::clear() const
{
	if (!std::filesystem::exists(root_))
		return 0;
	std::size_t n = 0;
	for (const auto &e : std::filesystem::directory_iterator(root_))
		if (e.path().extension() == ".grid" || e.path().extension() == ".h16") {
			std::error_code ec;
			std::filesystem::remove(e.path(), ec);
			if (!ec)
				++n;
		}
	return n;
}
std::size_t GridCache::bytes() const
{
	if (!std::filesystem::exists(root_))
		return 0;
	std::size_t n = 0;
	for (const auto &e : std::filesystem::directory_iterator(root_))
		if (e.path().extension() == ".grid" || e.path().extension() == ".h16") {
			std::error_code ec;
			n += std::filesystem::file_size(e.path(), ec);
		}
	return n;
}
std::string grid_cache_key(
		double a, double b, double c, double d, std::size_t w, std::size_t h)
{
	std::ostringstream s;
	s << std::fixed << std::setprecision(5) << a << '_' << b << '_' << c << '_' << d
	  << '_' << w << 'x' << h;
	auto k = s.str();
	for (char &x : k)
		if (x == '-' || x == '.')
			x = '_';
	return k;
}
}
