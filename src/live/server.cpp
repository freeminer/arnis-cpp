#include "server.h"

#include "../world_utils.h"

#include <boost/asio.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__APPLE__)
#include <sys/types.h>
#include <sys/sysctl.h>
#elif defined(__linux__)
#include <sys/sysinfo.h>
#else
#include <unistd.h>
#endif

namespace arnis::live::server
{
namespace
{
using Tcp = boost::asio::ip::tcp;

void set_error(std::string *out, std::string message)
{
	if (out)
		*out = std::move(message);
}

std::filesystem::path home_dir()
{
#if defined(_WIN32)
	if (const char *profile = std::getenv("USERPROFILE"); profile && *profile)
		return profile;
	if (const char *drive = std::getenv("HOMEDRIVE"))
		if (const char *path = std::getenv("HOMEPATH"))
			return std::filesystem::path(std::string(drive) + path);
#else
	if (const char *home = std::getenv("HOME"); home && *home)
		return home;
#endif
	return {};
}

bool bind_loopback(std::uint16_t port)
{
	boost::asio::io_context io;
	Tcp::acceptor acceptor(io);
	boost::system::error_code ec;
	acceptor.open(Tcp::v4(), ec);
	if (!ec)
		acceptor.bind(Tcp::endpoint(boost::asio::ip::address_v4::loopback(), port), ec);
	return !ec;
}

bool write_text_file(
		const std::filesystem::path &path, const std::string &text, std::string *error)
{
	std::ofstream output(path, std::ios::binary | std::ios::trunc);
	if (!output ||
			!output.write(text.data(), static_cast<std::streamsize>(text.size()))) {
		set_error(error, "could not write " + path.filename().string());
		return false;
	}
	output.close();
	if (!output) {
		set_error(error, "could not write " + path.filename().string());
		return false;
	}
	keep_private(path);
	return true;
}
} // namespace

void StartupMonitor::observe_line(const std::string &line)
{
	if (line.find("Done (") != std::string::npos &&
			line.find("For help") != std::string::npos) {
		ready_.store(true, std::memory_order_release);
		return;
	}
	static const std::pair<const char *, const char *> failures[]{
			{"FAILED TO BIND TO PORT", "The server port is taken by another program."},
			{"You need to agree to the EULA",
					"The Minecraft EULA has not been accepted."},
			{"Session lock is already acquired",
					"The world is open in Minecraft. Leave it first."},
			{"session.lock: already locked",
					"The world is open in Minecraft. Leave it first."},
			{"UnsupportedClassVersionError", "This Java is too old for the server."},
			{"Failed to start the minecraft server", "The server failed to start."},
	};
	for (const auto &[needle, message] : failures) {
		if (line.find(needle) == std::string::npos)
			continue;
		std::lock_guard lock(failure_mutex_);
		if (!failure_)
			failure_ = message;
		return;
	}
}

bool StartupMonitor::ready() const
{
	return ready_.load(std::memory_order_acquire);
}

std::optional<std::string> StartupMonitor::failure() const
{
	std::lock_guard lock(failure_mutex_);
	return failure_;
}

RecentLog::RecentLog(std::size_t capacity) : capacity_(capacity)
{
}

void RecentLog::append(std::string line)
{
	if (capacity_ == 0)
		return;
	std::lock_guard lock(mutex_);
	if (lines_.size() == capacity_)
		lines_.pop_front();
	lines_.push_back(std::move(line));
}

std::string RecentLog::tail(std::size_t count) const
{
	std::lock_guard lock(mutex_);
	const auto begin = lines_.size() - std::min(lines_.size(), count);
	std::string result;
	for (std::size_t i = begin; i < lines_.size(); ++i) {
		if (!result.empty())
			result.push_back('\n');
		result += lines_[i];
	}
	return result;
}

std::filesystem::path server_dir()
{
	if (const char *dir = std::getenv("ARNIS_LIVE_SERVER_DIR"); dir && *dir)
		return dir;
	std::filesystem::path base;
#if defined(_WIN32)
	if (const char *local = std::getenv("LOCALAPPDATA"); local && *local)
		base = local;
#elif defined(__APPLE__)
	if (const auto home = home_dir(); !home.empty())
		base = home / "Library" / "Application Support";
#else
	if (const char *xdg = std::getenv("XDG_DATA_HOME"); xdg && *xdg)
		base = xdg;
	else if (const auto home = home_dir(); !home.empty())
		base = home / ".local" / "share";
#endif
	if (base.empty())
		base = home_dir();
	if (base.empty())
		base = std::filesystem::temp_directory_path();
	return base / "arnis" / "live-server";
}

bool eula_accepted(const std::filesystem::path &dir)
{
	std::ifstream input(dir / "eula.txt");
	std::string line;
	while (std::getline(input, line)) {
		const auto first = line.find_first_not_of(" \t\r\n");
		if (first == std::string::npos)
			continue;
		const auto last = line.find_last_not_of(" \t\r\n");
		line = line.substr(first, last - first + 1);
		std::transform(line.begin(), line.end(), line.begin(),
				[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		if (line == "eula=true")
			return true;
	}
	return false;
}

bool accept_eula(const std::filesystem::path &dir, std::string *error)
{
	std::error_code ec;
	std::filesystem::create_directories(dir, ec);
	if (ec) {
		set_error(error, ec.message());
		return false;
	}
	const auto text = std::string("# Agreed in Arnis to the Minecraft EULA (") +
					  EULA_URL + ")\neula=true\n";
	std::ofstream output(dir / "eula.txt", std::ios::binary | std::ios::trunc);
	if (!output ||
			!output.write(text.data(), static_cast<std::streamsize>(text.size()))) {
		set_error(error, "could not write eula.txt");
		return false;
	}
	output.close();
	if (!output) {
		set_error(error, "could not write eula.txt");
		return false;
	}
	return true;
}

std::uint16_t free_port(std::optional<std::uint16_t> preferred)
{
	if (preferred && bind_loopback(*preferred))
		return *preferred;
	boost::asio::io_context io;
	Tcp::acceptor acceptor(io);
	boost::system::error_code ec;
	acceptor.open(Tcp::v4(), ec);
	if (!ec)
		acceptor.bind(Tcp::endpoint(boost::asio::ip::address_v4::loopback(), 0), ec);
	if (!ec) {
		const auto endpoint = acceptor.local_endpoint(ec);
		if (!ec)
			return endpoint.port();
	}
	return 25565;
}

std::uint16_t free_port_besides(std::uint16_t taken)
{
	boost::asio::io_context io;
	Tcp::acceptor held(io);
	boost::system::error_code ec;
	held.open(Tcp::v4(), ec);
	if (!ec)
		held.bind(Tcp::endpoint(boost::asio::ip::address_v4::loopback(), taken), ec);
	return free_port(std::nullopt);
}

bool port_free(std::uint16_t port)
{
	return bind_loopback(port);
}

std::string motd(const std::string &world_name)
{
	return "Arnis Live \xC2\xB7 " + world_name;
}

std::uint64_t system_total_memory_bytes()
{
#if defined(_WIN32)
	MEMORYSTATUSEX status{};
	status.dwLength = sizeof(status);
	if (GlobalMemoryStatusEx(&status))
		return status.ullTotalPhys;
#elif defined(__APPLE__)
	std::uint64_t bytes = 0;
	size_t size = sizeof(bytes);
	if (sysctlbyname("hw.memsize", &bytes, &size, nullptr, 0) == 0)
		return bytes;
#elif defined(__linux__)
	struct sysinfo info
	{
	};
	if (sysinfo(&info) == 0)
		return static_cast<std::uint64_t>(info.totalram) * info.mem_unit;
#elif defined(_SC_PHYS_PAGES) && defined(_SC_PAGESIZE)
	const long pages = sysconf(_SC_PHYS_PAGES);
	const long page_size = sysconf(_SC_PAGESIZE);
	if (pages > 0 && page_size > 0)
		return static_cast<std::uint64_t>(pages) * static_cast<std::uint64_t>(page_size);
#endif
	return 4ULL * 1024 * 1024 * 1024;
}

std::uint64_t heap_limit_mb(std::uint64_t total_memory_bytes)
{
	const std::uint64_t total_mb = total_memory_bytes / (1024 * 1024);
	return std::clamp<std::uint64_t>(total_mb / 4, 1024, 4096);
}

std::vector<std::string> java_arguments(
		const Launch &launch, std::uint64_t total_memory_bytes)
{
	return {"-Xmx" + std::to_string(heap_limit_mb(total_memory_bytes)) + "M", "-Xms512M",
			"-jar", launch.jar.string(), "--nogui", "--universe",
			launch.saves_dir.string(), "--world", launch.world_name};
}

bool prepare_server_files(
		const std::filesystem::path &dir, const Launch &launch, std::string *error)
{
	std::error_code ec;
	std::filesystem::create_directories(dir, ec);
	if (ec) {
		set_error(error, ec.message());
		return false;
	}
	if (!write_properties(dir, launch, error))
		return false;
	// All worlds share this server folder; permissions are derived from the world on each run.
	if (!write_text_file(dir / "ops.json", "[]", error))
		return false;
	// All players connect through loopback, so an IP ban would lock out every player.
	return write_text_file(dir / "banned-ips.json", "[]", error);
}

bool write_properties(
		const std::filesystem::path &dir, const Launch &launch, std::string *error)
{
	std::error_code ec;
	std::filesystem::create_directories(dir, ec);
	if (ec) {
		set_error(error, ec.message());
		return false;
	}
	const std::vector<std::pair<std::string, std::string>> properties{
			{"server-ip", "127.0.0.1"},
			{"server-port", std::to_string(launch.port)},
			{"level-name", launch.world_name},
			{"motd", motd(launch.world_name)},
			{"online-mode", "true"},
			{"white-list", "false"},
			{"enforce-whitelist", "false"},
			{"enable-rcon", "true"},
			{"rcon.port", std::to_string(launch.rcon_port)},
			{"rcon.password", launch.rcon_password},
			{"broadcast-rcon-to-ops", "false"},
			{"broadcast-console-to-ops", "false"},
			{"view-distance", std::to_string(launch.view_distance)},
			{"simulation-distance", std::to_string(std::min(launch.view_distance, 10u))},
			{"gamemode", launch.rules.gamemode},
			{"difficulty", launch.rules.difficulty},
			{"hardcore", launch.rules.hardcore ? "true" : "false"},
			{"force-gamemode", "false"},
			{"allow-flight", "true"},
			{"spawn-protection", "0"},
			{"enable-command-block", "true"},
			{"max-players", "8"},
			{"pause-when-empty-seconds", "-1"},
			{"log-ips", "false"},
			{"sync-chunk-writes", "true"},
	};
	std::string text = "# Written by Arnis for live mode on every start.\n";
	for (const auto &[key, value] : properties)
		text += key + "=" + value + "\n";
	const std::vector<std::uint8_t> bytes(text.begin(), text.end());
	const auto path = dir / "server.properties";
	if (!world_utils::replace_file_atomically(path, bytes)) {
		set_error(error, "could not write server.properties");
		return false;
	}
	keep_private(path);
	return true;
}

void keep_private(const std::filesystem::path &path)
{
#if defined(__unix__) || defined(__APPLE__)
	std::error_code ec;
	std::filesystem::permissions(path,
			std::filesystem::perms::owner_read | std::filesystem::perms::owner_write,
			std::filesystem::perm_options::replace, ec);
#else
	(void)path;
#endif
}
} // namespace arnis::live::server
