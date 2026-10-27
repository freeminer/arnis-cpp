#include "world_utils.h"
#include "args.h"
#include "assets_root.h"
#include "data_processing.h"
#include "retrieve_data.h"
#include "projection/projection_spec.h"
#include "osm_tiles.h"
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <array>
#include <mutex>
#include <cerrno>
#include <zlib.h>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif
namespace arnis::world_utils
{
WorldSizeOptions WorldSizeOptions::of(const Args &args)
{
	return {args.interior, args.bake_lighting, args.voxy_lod, args.caves, args.fillground,
			args.disable_height_limit};
}

const char *RunSizeError::short_message() const
{
	return area ? "Area too large, try a smaller one."
				: "Not enough disk space available.";
}

std::uint64_t estimated_world_bytes(
		double area_km2, WorldFormat format, const WorldSizeOptions &o)
{
	double per_km2 = 17.0;
	if (o.interior)
		per_km2 += 10.0;
	if (o.bake_lighting)
		per_km2 += 10.0;
	if (o.voxy_lod)
		per_km2 += 20.0;
	if (o.caves)
		per_km2 += o.tall ? 35.0 : 12.0;
	else if (o.fillground)
		per_km2 += o.tall ? 15.0 : 2.0;
	if (format != WorldFormat::JavaAnvil)
		per_km2 *= 0.8;
	const double mib = (std::max(0.0, area_km2) * per_km2 + 8.0) * 1024.0 * 1024.0;
	if (!std::isfinite(mib) || mib >= static_cast<double>(UINT64_MAX))
		return UINT64_MAX;
	return static_cast<std::uint64_t>(mib);
}

double blocks_km2(const Args &args, const geographic::LLBBox &bbox)
{
	try {
		const auto [transformer, rect] =
				projection::ProjectionSpec::from_args(args).transformer(bbox);
		(void)transformer;
		const auto min = rect.min();
		const auto max = rect.max();
		const double width = static_cast<double>(std::int64_t(max.x) - min.x + 1);
		const double height = static_cast<double>(std::int64_t(max.z) - min.z + 1);
		return width * height / 1'000'000.0;
	} catch (...) {
		return bbox.area_km2() * args.scale * args.scale;
	}
}

bool check_disk_space(
		const std::filesystem::path &dir, std::uint64_t needed, std::string *error)
{
	std::filesystem::path probe = dir;
	std::error_code ec;
	while (!probe.empty() && !std::filesystem::exists(probe, ec)) {
		ec.clear();
		const auto parent = probe.parent_path();
		if (parent == probe)
			break;
		probe = parent;
	}
	if (probe.empty())
		probe = std::filesystem::current_path(ec);
	if (ec)
		return true;
	ec.clear();
	const auto space = std::filesystem::space(probe, ec);
	if (ec || space.available == 0)
		return true;
	const auto required = needed > UINT64_MAX - DISK_HEADROOM_BYTES
								  ? UINT64_MAX
								  : needed + DISK_HEADROOM_BYTES;
	if (space.available >= required)
		return true;
	constexpr double GIB = 1024.0 * 1024.0 * 1024.0;
	if (error) {
		std::ostringstream message;
		message << std::fixed << std::setprecision(1)
				<< "Not enough disk space: this world needs about " << needed / GIB
				<< " GB and " << DISK_HEADROOM_BYTES / GIB
				<< " GB should stay free, but only " << space.available / GIB
				<< " GB are free.";
		*error = message.str();
	}
	return false;
}

bool check_run_size(const Args &args, const geographic::LLBBox &bbox, WorldFormat format,
		const std::filesystem::path &dir, RunSizeError *error)
{
	const auto fail = [&](bool area, std::string detail) {
		if (error)
			*error = {area, std::move(detail)};
		return false;
	};
	if (args.mapillary_probe)
		return true;
	if (!args.skip_osm() && !args.file && !args.no_tile_archive &&
			!args.osm_tiles_url.empty()) {
		std::string detail;
		if (!osm_tiles::check_size(bbox, args.scale, &detail))
			return fail(true,
					"The area is too large: " + detail + ". Select a smaller area.");
	}
	const auto needed = estimated_world_bytes(
			blocks_km2(args, bbox), format, WorldSizeOptions::of(args));
	std::string detail;
	if (!check_disk_space(dir, needed, &detail))
		return fail(false, std::move(detail));
	return true;
}

bool check_disk_for_extended_height(const Args &args, const geographic::LLBBox &bbox,
		WorldFormat format, const std::filesystem::path &dir, std::string *error)
{
	const double area = blocks_km2(args, bbox);
	auto options = WorldSizeOptions::of(args);
	auto tall_options = options;
	tall_options.tall = true;
	const auto needed = estimated_world_bytes(area, format, tall_options);
	const auto before = estimated_world_bytes(area, format, options);
	if (needed <= before)
		return true;
	return check_disk_space(dir, needed, error);
}

namespace
{
struct HeldSessionLock
{
	std::filesystem::path path;
	std::uint64_t a = 0, b = 0;
};
std::mutex session_lock_mutex;
std::vector<HeldSessionLock> held_session_locks;

bool same_lock(const HeldSessionLock &held, const std::filesystem::path &path,
		std::uint64_t a, std::uint64_t b)
{
	if (a != 0 || b != 0)
		return held.a == a && held.b == b;
	return held.path.lexically_normal() == path.lexically_normal();
}

void set_lock_error(std::string *out, const std::string &message)
{
	if (out)
		*out = message;
}

class NbtReader
{
	const std::vector<std::uint8_t> &bytes_;
	std::size_t pos_{0};

	bool read_u8(std::uint8_t &value)
	{
		if (pos_ >= bytes_.size())
			return false;
		value = bytes_[pos_++];
		return true;
	}
	bool read_u16(std::uint16_t &value)
	{
		if (bytes_.size() - pos_ < 2)
			return false;
		value = (std::uint16_t(bytes_[pos_]) << 8) | bytes_[pos_ + 1];
		pos_ += 2;
		return true;
	}
	bool read_i32(std::int32_t &value)
	{
		if (bytes_.size() - pos_ < 4)
			return false;
		const auto raw = (std::uint32_t(bytes_[pos_]) << 24) |
						 (std::uint32_t(bytes_[pos_ + 1]) << 16) |
						 (std::uint32_t(bytes_[pos_ + 2]) << 8) | bytes_[pos_ + 3];
		pos_ += 4;
		value = static_cast<std::int32_t>(raw);
		return true;
	}
	bool skip(std::size_t count)
	{
		if (count > bytes_.size() - pos_)
			return false;
		pos_ += count;
		return true;
	}
	bool read_name(std::string &name)
	{
		std::uint16_t size = 0;
		if (!read_u16(size) || size > bytes_.size() - pos_)
			return false;
		name.assign(reinterpret_cast<const char *>(bytes_.data() + pos_), size);
		pos_ += size;
		return true;
	}
	bool skip_payload(std::uint8_t type, unsigned depth)
	{
		if (depth > 64)
			return false;
		std::int32_t length = 0;
		std::uint8_t element_type = 0;
		std::uint16_t string_length = 0;
		switch (type) {
		case 1:
			return skip(1);
		case 2:
			return skip(2);
		case 3:
		case 5:
			return skip(4);
		case 4:
		case 6:
			return skip(8);
		case 7:
			return read_i32(length) && length >= 0 && skip(std::size_t(length));
		case 8:
			return read_u16(string_length) && skip(string_length);
		case 9:
			if (!read_u8(element_type) || !read_i32(length) || length < 0 ||
					(element_type == 0 && length != 0))
				return false;
			for (std::int32_t i = 0; i < length; ++i)
				if (!skip_payload(element_type, depth + 1))
					return false;
			return true;
		case 10:
			for (;;) {
				std::uint8_t child_type = 0;
				if (!read_u8(child_type))
					return false;
				if (child_type == 0)
					return true;
				std::string child_name;
				if (!read_name(child_name) || !skip_payload(child_type, depth + 1))
					return false;
			}
		case 11:
			return read_i32(length) && length >= 0 &&
				   std::size_t(length) <= std::numeric_limits<std::size_t>::max() / 4 &&
				   skip(std::size_t(length) * 4);
		case 12:
			return read_i32(length) && length >= 0 &&
				   std::size_t(length) <= std::numeric_limits<std::size_t>::max() / 8 &&
				   skip(std::size_t(length) * 8);
		default:
			return false;
		}
	}
	std::optional<std::int32_t> read_data_compound(unsigned depth)
	{
		if (depth > 64)
			return std::nullopt;
		std::optional<std::int32_t> version;
		for (;;) {
			std::uint8_t type = 0;
			if (!read_u8(type))
				return std::nullopt;
			if (type == 0)
				return version;
			std::string name;
			if (!read_name(name))
				return std::nullopt;
			if (name == "DataVersion" && type == 3) {
				std::int32_t value = 0;
				if (!read_i32(value))
					return std::nullopt;
				version = value;
			} else if (name == "Data" && type == 10) {
				if (const auto found = read_data_compound(depth + 1))
					version = found;
			} else if (!skip_payload(type, depth + 1)) {
				return std::nullopt;
			}
		}
	}
	std::optional<std::int32_t> read_map_data_compound(unsigned depth)
	{
		if (depth > 64)
			return std::nullopt;
		std::optional<std::int32_t> map_id;
		for (;;) {
			std::uint8_t type = 0;
			if (!read_u8(type) || type == 0)
				return map_id;
			std::string name;
			if (!read_name(name))
				return std::nullopt;
			if (name == "map" && type == 3) {
				std::int32_t value = 0;
				if (!read_i32(value))
					return std::nullopt;
				map_id = value;
			} else if (!skip_payload(type, depth + 1)) {
				return std::nullopt;
			}
		}
	}
	std::optional<std::pair<std::int32_t, std::int32_t>> read_spawn_data_compound(
			unsigned depth)
	{
		if (depth > 64)
			return std::nullopt;
		std::optional<std::int32_t> spawn_x, spawn_z;
		std::optional<std::pair<std::int32_t, std::int32_t>> modern_spawn;
		for (;;) {
			std::uint8_t type = 0;
			if (!read_u8(type))
				return std::nullopt;
			if (type == 0) {
				if (modern_spawn)
					return modern_spawn;
				if (!spawn_x || !spawn_z)
					return std::nullopt;
				return std::pair{*spawn_x, *spawn_z};
			}
			std::string name;
			if (!read_name(name))
				return std::nullopt;
			if (name == "spawn" && type == 10) {
				bool valid = true;
				const auto parsed = read_modern_spawn_compound(depth + 1, valid);
				if (!valid)
					return std::nullopt;
				if (parsed)
					modern_spawn = parsed;
			} else if ((name == "SpawnX" || name == "SpawnZ") && type == 3) {
				std::int32_t value = 0;
				if (!read_i32(value))
					return std::nullopt;
				(name == "SpawnX" ? spawn_x : spawn_z) = value;
			} else if (!skip_payload(type, depth + 1)) {
				return std::nullopt;
			}
		}
	}
	std::optional<std::pair<std::int32_t, std::int32_t>> read_modern_spawn_compound(
			unsigned depth, bool &valid)
	{
		if (depth > 64) {
			valid = false;
			return std::nullopt;
		}
		std::optional<std::pair<std::int32_t, std::int32_t>> spawn;
		for (;;) {
			std::uint8_t type = 0;
			if (!read_u8(type)) {
				valid = false;
				return std::nullopt;
			}
			if (type == 0)
				return spawn;
			std::string name;
			if (!read_name(name)) {
				valid = false;
				return std::nullopt;
			}
			if (name == "pos" && type == 11) {
				std::int32_t count = 0;
				if (!read_i32(count) || count < 0 ||
						std::size_t(count) > (bytes_.size() - pos_) / 4) {
					valid = false;
					return std::nullopt;
				}
				std::int32_t x = 0, y = 0, z = 0;
				if (count >= 3 && (!read_i32(x) || !read_i32(y) || !read_i32(z))) {
					valid = false;
					return std::nullopt;
				}
				const auto remaining = std::size_t(count) - (count >= 3 ? 3 : 0);
				if (!skip(remaining * 4)) {
					valid = false;
					return std::nullopt;
				}
				if (count >= 3) {
					(void)y;
					spawn = std::pair{x, z};
				}
				continue;
			}
			if (!skip_payload(type, depth + 1)) {
				valid = false;
				return std::nullopt;
			}
		}
	}
	std::optional<std::int32_t> read_map_root()
	{
		std::uint8_t root_type = 0;
		std::string root_name;
		if (!read_u8(root_type) || root_type != 10 || !read_name(root_name))
			return std::nullopt;
		for (;;) {
			std::uint8_t type = 0;
			if (!read_u8(type) || type == 0)
				return std::nullopt;
			std::string name;
			if (!read_name(name))
				return std::nullopt;
			if (name == "data" && type == 10) {
				if (const auto id = read_map_data_compound(1))
					return id;
			} else if (!skip_payload(type, 1)) {
				return std::nullopt;
			}
		}
	}
	std::optional<std::pair<std::int32_t, std::int32_t>> read_spawn_root()
	{
		std::uint8_t root_type = 0;
		std::string root_name;
		if (!read_u8(root_type) || root_type != 10 || !read_name(root_name))
			return std::nullopt;
		for (;;) {
			std::uint8_t type = 0;
			if (!read_u8(type) || type == 0)
				return std::nullopt;
			std::string name;
			if (!read_name(name))
				return std::nullopt;
			if (name == "Data" && type == 10)
				return read_spawn_data_compound(1);
			if (!skip_payload(type, 1))
				return std::nullopt;
		}
	}

public:
	explicit NbtReader(const std::vector<std::uint8_t> &bytes) : bytes_(bytes) {}
	std::optional<std::int32_t> data_version()
	{
		std::uint8_t root_type = 0;
		std::string root_name;
		if (!read_u8(root_type) || root_type != 10 || !read_name(root_name))
			return std::nullopt;
		return read_data_compound(0);
	}
	std::optional<std::int32_t> map_id() { return read_map_root(); }
	std::optional<std::pair<std::int32_t, std::int32_t>> spawn_xz()
	{
		return read_spawn_root();
	}
};

std::optional<std::vector<std::uint8_t>> decompress_level_dat(
		const std::filesystem::path &path)
{
	std::ifstream input(path, std::ios::binary);
	if (!input)
		return std::nullopt;
	std::vector<std::uint8_t> compressed((std::istreambuf_iterator<char>(input)), {});
	if (compressed.empty() || compressed.size() > std::numeric_limits<uInt>::max())
		return std::nullopt;
	z_stream stream{};
	if (inflateInit2(&stream, 16 + MAX_WBITS) != Z_OK)
		return std::nullopt;
	stream.next_in = compressed.data();
	stream.avail_in = static_cast<uInt>(compressed.size());
	std::vector<std::uint8_t> output;
	std::array<std::uint8_t, 32768> chunk{};
	int status = Z_OK;
	while (status == Z_OK && output.size() <= 64 * 1024 * 1024) {
		stream.next_out = chunk.data();
		stream.avail_out = static_cast<uInt>(chunk.size());
		status = inflate(&stream, Z_NO_FLUSH);
		const auto produced = chunk.size() - stream.avail_out;
		output.insert(output.end(), chunk.begin(), chunk.begin() + produced);
	}
	inflateEnd(&stream);
	if (status != Z_STREAM_END || output.size() > 64 * 1024 * 1024)
		return std::nullopt;
	return output;
}
} // namespace

SessionLock::SessionLock(std::filesystem::path path, std::intptr_t handle,
		std::uint64_t identity_a, std::uint64_t identity_b) :
		path_(std::move(path)), handle_(handle), identity_a_(identity_a),
		identity_b_(identity_b)
{
}

std::unique_ptr<SessionLock> SessionLock::acquire(
		const std::filesystem::path &world, std::string *error)
{
	const auto path = world / "session.lock";
	std::lock_guard<std::mutex> guard(session_lock_mutex);
#if defined(_WIN32)
	HANDLE native = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE,
			FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_ALWAYS,
			FILE_ATTRIBUTE_NORMAL, nullptr);
	if (native == INVALID_HANDLE_VALUE) {
		set_lock_error(error,
				"Failed to create session.lock file: " +
						std::system_category().message(static_cast<int>(GetLastError())));
		return {};
	}
	BY_HANDLE_FILE_INFORMATION info{};
	if (!GetFileInformationByHandle(native, &info)) {
		const auto message =
				std::system_category().message(static_cast<int>(GetLastError()));
		CloseHandle(native);
		set_lock_error(error, "Failed to identify session.lock file: " + message);
		return {};
	}
	const auto identity_a = std::uint64_t(info.dwVolumeSerialNumber);
	const auto identity_b =
			(std::uint64_t(info.nFileIndexHigh) << 32) | info.nFileIndexLow;
#else
	const int fd = ::open(path.c_str(), O_RDWR | O_CREAT, 0666);
	if (fd < 0) {
		set_lock_error(
				error, "Failed to create session.lock file: " +
							   std::error_code(errno, std::generic_category()).message());
		return {};
	}
	struct stat info
	{
	};
	if (::fstat(fd, &info) != 0) {
		const auto message = std::error_code(errno, std::generic_category()).message();
		::close(fd);
		set_lock_error(error, "Failed to identify session.lock file: " + message);
		return {};
	}
	const auto identity_a = static_cast<std::uint64_t>(info.st_dev);
	const auto identity_b = static_cast<std::uint64_t>(info.st_ino);
#endif
	if (std::any_of(held_session_locks.begin(), held_session_locks.end(),
				[&](const auto &held) {
					return same_lock(held, path, identity_a, identity_b);
				})) {
#if defined(_WIN32)
		CloseHandle(native);
#else
		::close(fd);
#endif
		set_lock_error(
				error, "Failed to acquire lock on session.lock file: already held");
		return {};
	}
#if defined(_WIN32)
	OVERLAPPED overlap{};
	if (!LockFileEx(native, LOCKFILE_EXCLUSIVE_LOCK | LOCKFILE_FAIL_IMMEDIATELY, 0,
				MAXDWORD, MAXDWORD, &overlap)) {
		const auto message =
				std::system_category().message(static_cast<int>(GetLastError()));
		CloseHandle(native);
		set_lock_error(error, "Failed to acquire lock on session.lock file: " + message);
		return {};
	}
	LARGE_INTEGER zero{};
	constexpr char marker[] = "\xe2\x98\x83";
	DWORD written = 0;
	if (!SetFilePointerEx(native, zero, nullptr, FILE_BEGIN) || !SetEndOfFile(native) ||
			!WriteFile(native, marker, sizeof(marker) - 1, &written, nullptr) ||
			written != sizeof(marker) - 1) {
		const auto message =
				std::system_category().message(static_cast<int>(GetLastError()));
		UnlockFileEx(native, 0, MAXDWORD, MAXDWORD, &overlap);
		CloseHandle(native);
		set_lock_error(error, "Failed to write session.lock file: " + message);
		return {};
	}
#else
	struct flock lock
	{
	};
	lock.l_type = F_WRLCK;
	lock.l_whence = SEEK_SET;
	if (::fcntl(fd, F_SETLK, &lock) != 0) {
		const auto message = std::error_code(errno, std::generic_category()).message();
		::close(fd);
		set_lock_error(error, "Failed to acquire lock on session.lock file: " + message);
		return {};
	}
	constexpr char marker[] = "\xe2\x98\x83";
	if (::ftruncate(fd, 0) != 0 || ::write(fd, marker, sizeof(marker) - 1) !=
										   static_cast<ssize_t>(sizeof(marker) - 1)) {
		const auto message = std::error_code(errno, std::generic_category()).message();
		struct flock unlock
		{
		};
		unlock.l_type = F_UNLCK;
		unlock.l_whence = SEEK_SET;
		::fcntl(fd, F_SETLK, &unlock);
		::close(fd);
		set_lock_error(error, "Failed to write session.lock file: " + message);
		return {};
	}
#endif
	held_session_locks.push_back({path, identity_a, identity_b});
#if defined(_WIN32)
	const auto handle = reinterpret_cast<std::intptr_t>(native);
#else
	const auto handle = static_cast<std::intptr_t>(fd);
#endif
	return std::unique_ptr<SessionLock>(
			new SessionLock(path, handle, identity_a, identity_b));
}

SessionLock::~SessionLock()
{
	if (handle_ < 0)
		return;
	std::lock_guard<std::mutex> guard(session_lock_mutex);
	std::error_code ignored;
	std::filesystem::remove(path_, ignored);
#if defined(_WIN32)
	const auto native = reinterpret_cast<HANDLE>(handle_);
	OVERLAPPED overlap{};
	UnlockFileEx(native, 0, MAXDWORD, MAXDWORD, &overlap);
	CloseHandle(native);
#else
	const int fd = static_cast<int>(handle_);
	struct flock unlock
	{
	};
	unlock.l_type = F_UNLCK;
	unlock.l_whence = SEEK_SET;
	::fcntl(fd, F_SETLK, &unlock);
	::close(fd);
#endif
	held_session_locks.erase(
			std::remove_if(held_session_locks.begin(), held_session_locks.end(),
					[&](const auto &held) {
						return same_lock(held, path_, identity_a_, identity_b_);
					}),
			held_session_locks.end());
}

bool world_is_locked(const std::filesystem::path &world)
{
	const auto path = world / "session.lock";
	std::error_code ec;
	if (!std::filesystem::exists(path, ec) || ec)
		return false;
	std::lock_guard<std::mutex> guard(session_lock_mutex);
#if defined(_WIN32)
	HANDLE native = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE,
			FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
			OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (native == INVALID_HANDLE_VALUE)
		return GetLastError() == ERROR_SHARING_VIOLATION ||
			   GetLastError() == ERROR_ACCESS_DENIED;
	BY_HANDLE_FILE_INFORMATION info{};
	if (GetFileInformationByHandle(native, &info)) {
		const auto a = std::uint64_t(info.dwVolumeSerialNumber);
		const auto b = (std::uint64_t(info.nFileIndexHigh) << 32) | info.nFileIndexLow;
		if (std::any_of(held_session_locks.begin(), held_session_locks.end(),
					[&](const auto &held) { return same_lock(held, path, a, b); })) {
			CloseHandle(native);
			return false;
		}
	}
	OVERLAPPED overlap{};
	const bool locked =
			!LockFileEx(native, LOCKFILE_EXCLUSIVE_LOCK | LOCKFILE_FAIL_IMMEDIATELY, 0,
					MAXDWORD, MAXDWORD, &overlap);
	if (!locked)
		UnlockFileEx(native, 0, MAXDWORD, MAXDWORD, &overlap);
	CloseHandle(native);
	return locked;
#else
	const int fd = ::open(path.c_str(), O_RDWR);
	if (fd < 0)
		return errno == EACCES || errno == EAGAIN;
	struct stat info
	{
	};
	if (::fstat(fd, &info) != 0) {
		::close(fd);
		return false;
	}
	const auto a = static_cast<std::uint64_t>(info.st_dev);
	const auto b = static_cast<std::uint64_t>(info.st_ino);
	if (std::any_of(held_session_locks.begin(), held_session_locks.end(),
				[&](const auto &held) { return same_lock(held, path, a, b); })) {
		::close(fd);
		return false;
	}
	struct flock lock
	{
	};
	lock.l_type = F_WRLCK;
	lock.l_whence = SEEK_SET;
	const bool locked = ::fcntl(fd, F_GETLK, &lock) == 0 && lock.l_type != F_UNLCK;
	::close(fd);
	return locked;
#endif
}

std::optional<std::int32_t> level_data_version(const std::filesystem::path &world)
{
	const auto bytes = decompress_level_dat(world / "level.dat");
	return bytes ? NbtReader(*bytes).data_version() : std::nullopt;
}

std::optional<std::pair<std::int32_t, std::int32_t>> level_spawn_xz(
		const std::filesystem::path &world)
{
	const auto bytes = decompress_level_dat(world / "level.dat");
	return bytes ? NbtReader(*bytes).spawn_xz() : std::nullopt;
}

std::optional<std::int32_t> map_id_counter(const std::filesystem::path &path)
{
	const auto bytes = decompress_level_dat(path);
	return bytes ? NbtReader(*bytes).map_id() : std::nullopt;
}

WorldLayout WorldLayout::of(const std::filesystem::path &world)
{
	const auto version = level_data_version(world);
	return {version && *version >= DIMENSION_FOLDERS_DATA_VERSION ? Kind::Dimensions
																  : Kind::Legacy};
}

std::filesystem::path WorldLayout::overworld_dir(const std::filesystem::path &world) const
{
	return kind == Kind::Dimensions ? world / "dimensions" / "minecraft" / "overworld"
									: world;
}

std::filesystem::path WorldLayout::maps_dir(const std::filesystem::path &world) const
{
	return kind == Kind::Dimensions ? world / "data" / "minecraft" / "maps"
									: world / "data";
}

bool install_tall_datapack(const std::filesystem::path &world, std::string *error)
{
	if (error)
		error->clear();
	const auto root = world / "datapacks" / "arnis_tall";
	const std::array<std::pair<std::filesystem::path, std::filesystem::path>, 4> files{{
			{"minecraft/datapack_tall/pack.mcmeta", "pack.mcmeta"},
			{"minecraft/datapack_tall/data/minecraft/dimension_type/overworld.json",
					"data/minecraft/dimension_type/overworld.json"},
			{"minecraft/datapack_tall/overlay_attributes/data/minecraft/dimension_type/overworld.json",
					"overlay_attributes/data/minecraft/dimension_type/overworld.json"},
			{"minecraft/datapack_tall/overlay_2601/data/minecraft/dimension_type/overworld.json",
					"overlay_2601/data/minecraft/dimension_type/overworld.json"},
	}};
	for (const auto &[asset_name, pack_name] : files) {
		const auto source = assets::path(asset_name);
		std::ifstream input(source, std::ios::binary);
		if (!input) {
			if (error)
				*error = "failed to read bundled datapack file " + source.string();
			return false;
		}
		const std::vector<std::uint8_t> contents((std::istreambuf_iterator<char>(input)),
				std::istreambuf_iterator<char>());
		const auto destination = root / pack_name;
		std::error_code ec;
		std::filesystem::create_directories(destination.parent_path(), ec);
		if (ec || !replace_file_atomically(destination, contents)) {
			if (error)
				*error = "failed to write datapack file " + destination.string() +
						 (ec ? ": " + ec.message() : std::string{});
			return false;
		}
	}
	return enable_datapack_in_level_dat(world, "arnis_tall", error);
}

static std::filesystem::path home()
{
	if (const char *h = std::getenv("HOME"))
		return h;
	if (const char *h = std::getenv("USERPROFILE"))
		return h;
	return ".";
}

namespace
{
std::string unique_path_suffix()
{
	static std::atomic<std::uint64_t> counter{0};
	const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
#if defined(_WIN32)
	const auto pid = static_cast<std::uint64_t>(GetCurrentProcessId());
#else
	const auto pid = static_cast<std::uint64_t>(getpid());
#endif
	std::ostringstream suffix;
	suffix << std::hex << pid << '-' << ticks << '-'
		   << counter.fetch_add(1, std::memory_order_relaxed);
	return suffix.str();
}

std::filesystem::path user_directory(const char *xdg_env, const char *name)
{
	if (const char *configured = std::getenv(xdg_env); configured && *configured)
		return configured;
	std::filesystem::path config_home;
	if (const char *configured_home = std::getenv("XDG_CONFIG_HOME");
			configured_home && *configured_home)
		config_home = configured_home;
	else
		config_home = home() / ".config";
	std::ifstream config(config_home / "user-dirs.dirs");
	const std::string key = std::string(xdg_env) + "=";
	std::string line;
	while (std::getline(config, line)) {
		const auto first = line.find_first_not_of(" \t");
		if (first == std::string::npos || line.compare(first, key.size(), key) != 0)
			continue;
		std::string value = line.substr(first + key.size());
		if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
			value = value.substr(1, value.size() - 2);
		const auto home_token = value.find("$HOME");
		if (home_token != std::string::npos)
			value.replace(home_token, 5, home().string());
		return value;
	}
	return home() / name;
}

bool is_writable_directory(const std::filesystem::path &dir)
{
	std::error_code ec;
	if (!std::filesystem::is_directory(dir, ec) || ec)
		return false;
	const auto probe = dir / (".arnis-write-test-" + unique_path_suffix());
	if (!std::filesystem::create_directory(probe, ec) || ec)
		return false;
	std::filesystem::remove(probe, ec);
	return true;
}
} // namespace

std::filesystem::path default_bedrock_output_directory()
{
	return user_directory("XDG_DESKTOP_DIR", "Desktop");
}

std::filesystem::path get_bedrock_output_directory()
{
	const std::array<std::filesystem::path, 4> candidates = {
			user_directory("XDG_DESKTOP_DIR", "Desktop"),
			user_directory("XDG_DOCUMENTS_DIR", "Documents"),
			user_directory("XDG_DOWNLOAD_DIR", "Downloads"), home()};
	for (const auto &candidate : candidates)
		if (is_writable_directory(candidate))
			return candidate;
	return ".";
}
std::filesystem::path get_luanti_worlds_directory()
{
	std::filesystem::path base;
#if defined(_WIN32)
	if (const char *appdata = std::getenv("APPDATA"))
		base = std::filesystem::path(appdata) / "Minetest";
#elif defined(__APPLE__)
	if (std::getenv("HOME"))
		base = home() / "Library" / "Application Support" / "minetest";
#else
	if (std::getenv("HOME"))
		base = home() / ".minetest";
#endif
	// Match Rust: only use the platform directory when it can be resolved;
	// otherwise keep generated Luanti worlds in a visible desktop fallback.
	if (!base.empty())
		return base / "worlds";
	std::filesystem::path desktop;
	if (const char *xdg = std::getenv("XDG_DESKTOP_DIR"))
		desktop = xdg;
	else if (!home().empty())
		desktop = home() / "Desktop";
	if (desktop.empty())
		desktop = ".";
	return desktop / "Arnis Luanti Worlds";
}
bool replace_file_atomically(
		const std::filesystem::path &path, const std::vector<std::uint8_t> &bytes)
{
	if (path.parent_path().empty())
		return false;
	static std::atomic_uint64_t sequence{0};
	const auto temporary = path.string() + ".tmp" + std::to_string(++sequence);
	std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
	if (!output)
		return false;
	if (!bytes.empty())
		output.write(reinterpret_cast<const char *>(bytes.data()),
				static_cast<std::streamsize>(bytes.size()));
	output.close();
	if (!output) {
		std::error_code ignored;
		std::filesystem::remove(temporary, ignored);
		return false;
	}
	std::error_code rename_error;
	std::filesystem::rename(temporary, path, rename_error);
	if (rename_error) {
		std::error_code ignored;
		std::filesystem::remove(temporary, ignored);
		return false;
	}
	return true;
}
std::string sanitize_for_filename(const std::string &name)
{
	constexpr std::size_t max_bytes = 64;
	static constexpr char invalid[] = "<>:\"/\\|?*";
	std::string out;
	out.reserve(std::min(name.size(), max_bytes));
	for (const unsigned char c : name)
		out.push_back(c < 32 || std::strchr(invalid, c) ? '_' : char(c));
	const auto trim = [](std::string &value) {
		std::size_t first = 0;
		while (first < value.size() &&
				std::isspace(static_cast<unsigned char>(value[first])))
			++first;
		std::size_t last = value.size();
		while (last > first &&
				(std::isspace(static_cast<unsigned char>(value[last - 1])) ||
						value[last - 1] == '.'))
			--last;
		value = value.substr(first, last - first);
	};
	trim(out);
	if (out.size() > max_bytes) {
		out.resize(max_bytes);
		// Do not leave a partial UTF-8 sequence after the byte cap. This mirrors
		// Rust's char-boundary truncation and keeps generated world paths valid.
		while (!out.empty() && (static_cast<unsigned char>(out.back()) & 0xc0) == 0x80)
			out.pop_back();
		if (!out.empty() && static_cast<unsigned char>(out.back()) >= 0xc0) {
			const auto lead = static_cast<unsigned char>(out.back());
			const std::size_t width = (lead & 0xe0) == 0xc0	  ? 2
									  : (lead & 0xf0) == 0xe0 ? 3
									  : (lead & 0xf8) == 0xf0 ? 4
															  : 1;
			if (out.size() - 1 + width > max_bytes)
				out.pop_back();
		}
		trim(out);
	}
	return out.empty() ? "Unknown Location" : out;
}

std::string world_folder_name(const std::string &raw)
{
	// Rust caps custom names by characters (not bytes), then applies the same
	// filesystem-safe filtering used for export filenames.  Preserve UTF-8
	// boundaries while enforcing the 48-character UI limit.
	std::string capped;
	std::size_t chars = 0;
	for (std::size_t i = 0; i < raw.size() && chars < 48;) {
		const auto c = static_cast<unsigned char>(raw[i]);
		std::size_t width = c < 0x80 ? 1 : (c < 0xe0 ? 2 : (c < 0xf0 ? 3 : 4));
		if (i + width > raw.size())
			width = 1;
		capped.append(raw, i, width);
		i += width;
		++chars;
	}
	static constexpr char invalid[] = "<>:\"/\\|?*";
	std::string sanitized;
	sanitized.reserve(capped.size());
	for (const unsigned char c : capped)
		sanitized.push_back(c < 32 || std::strchr(invalid, c) ? '_' : char(c));
	const auto trim = [](std::string &value) {
		std::size_t first = 0;
		while (first < value.size() &&
				std::isspace(static_cast<unsigned char>(value[first])))
			++first;
		std::size_t last = value.size();
		while (last > first &&
				(std::isspace(static_cast<unsigned char>(value[last - 1])) ||
						value[last - 1] == '.'))
			--last;
		value = value.substr(first, last - first);
	};
	trim(sanitized);
	return sanitized;
}

std::string unique_world_folder_name(
		const std::filesystem::path &base_path, const std::string &custom_name)
{
	const auto usable = world_folder_name(custom_name);
	if (!usable.empty()) {
		if (!std::filesystem::exists(base_path / usable))
			return usable;
		for (std::uint32_t n = 2;; ++n) {
			const auto candidate = usable + " (" + std::to_string(n) + ")";
			if (!std::filesystem::exists(base_path / candidate))
				return candidate;
		}
	}
	for (std::uint32_t n = 1;; ++n) {
		const auto candidate = "Arnis World " + std::to_string(n);
		if (std::filesystem::exists(base_path / candidate))
			continue;
		bool location_collision = false;
		std::error_code error;
		for (const auto &entry : std::filesystem::directory_iterator(base_path, error)) {
			if (error)
				break;
			const auto name = entry.path().filename().string();
			if (name.rfind(candidate + ": ", 0) == 0) {
				location_collision = true;
				break;
			}
		}
		if (!location_collision)
			return candidate;
	}
}

std::string get_area_name_for_bedrock(const geographic::LLBBox &bbox)
{
	const double lat = (bbox.min().lat() + bbox.max().lat()) * 0.5;
	const double lon = (bbox.min().lng() + bbox.max().lng()) * 0.5;
	return retrieve_data::fetch_area_name(lat, lon).value_or("Unknown Location");
}

std::pair<std::filesystem::path, std::string> build_bedrock_output(
		const geographic::LLBBox &bbox, const std::filesystem::path &output_dir,
		CelestialBody body)
{
	// Reverse geocoding lunar/planetary coordinates can produce a misleading
	// terrestrial place name. Match Rust by naming those exports after the body.
	const auto raw_name = is_earth(body) ? get_area_name_for_bedrock(bbox)
										 : std::string(celestial_body_display_name(body));
	const auto safe_name = sanitize_for_filename(raw_name);
	return {output_dir / ("Arnis " + safe_name + ".mcworld"),
			"Arnis World: " + safe_name};
}
}
