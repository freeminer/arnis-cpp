#pragma once

#include <cstdint>
#include <deque>
#include <filesystem>
#include <mutex>
#include <optional>
#include <atomic>
#include <string>
#include <vector>

namespace arnis::live::server
{
inline constexpr const char *EULA_URL = "https://aka.ms/MinecraftEULA";

struct WorldRules
{
	std::string gamemode = "survival";
	std::string difficulty = "normal";
	bool hardcore = false;
};

struct Launch
{
	std::filesystem::path java;
	std::filesystem::path jar;
	std::filesystem::path saves_dir;
	std::string world_name;
	std::uint16_t port = 25565;
	std::uint16_t rcon_port = 25575;
	std::string rcon_password;
	std::uint32_t view_distance = 12;
	WorldRules rules;
};

class StartupMonitor
{
public:
	void observe_line(const std::string &line);
	bool ready() const;
	std::optional<std::string> failure() const;

private:
	std::atomic<bool> ready_{false};
	mutable std::mutex failure_mutex_;
	std::optional<std::string> failure_;
};

class RecentLog
{
public:
	explicit RecentLog(std::size_t capacity = 200);
	void append(std::string line);
	std::string tail(std::size_t count) const;

private:
	std::size_t capacity_;
	mutable std::mutex mutex_;
	std::deque<std::string> lines_;
};

std::filesystem::path server_dir();
bool eula_accepted(const std::filesystem::path &dir);
bool accept_eula(const std::filesystem::path &dir, std::string *error = nullptr);
std::uint16_t free_port(std::optional<std::uint16_t> preferred = std::nullopt);
std::uint16_t free_port_besides(std::uint16_t taken);
bool port_free(std::uint16_t port);
std::string motd(const std::string &world_name);
std::uint64_t system_total_memory_bytes();
std::uint64_t heap_limit_mb(std::uint64_t total_memory_bytes);
std::vector<std::string> java_arguments(
		const Launch &launch, std::uint64_t total_memory_bytes);
bool prepare_server_files(const std::filesystem::path &dir, const Launch &launch,
		std::string *error = nullptr);
bool write_properties(const std::filesystem::path &dir, const Launch &launch,
		std::string *error = nullptr);
void keep_private(const std::filesystem::path &path);
} // namespace arnis::live::server
