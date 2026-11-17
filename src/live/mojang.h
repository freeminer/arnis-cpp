#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace arnis::live::mojang
{
inline constexpr const char *OLDEST_VERSION = "1.21.4";
using Progress = std::function<bool(const std::string &, std::optional<double>)>;

struct VersionInfo
{
	std::string id;
	std::string server_url;
	std::string server_sha1;
	std::uint64_t server_size = 0;
	std::string java_component;
	std::uint32_t java_major = 21;
};

class Manifest
{
public:
	static std::optional<Manifest> load(
			const std::filesystem::path &cache_dir, std::string *error = nullptr);

	std::optional<std::string> latest_release() const;
	std::vector<std::string> supported_releases() const;
	bool is_supported(const std::string &id) const;
	std::optional<std::string> release_time(const std::string &id) const;
	std::optional<VersionInfo> version_info(const std::string &id,
			const std::filesystem::path &cache_dir, std::string *error = nullptr) const;
	std::optional<std::string> last_played(const std::filesystem::path &game_dir) const;

private:
	std::shared_ptr<const nlohmann::json> json_;
	explicit Manifest(std::shared_ptr<const nlohmann::json> json) : json_(std::move(json))
	{
	}
};

std::optional<std::filesystem::path> game_dir(const std::filesystem::path &saves_dir);
std::optional<std::filesystem::path> default_game_dir();
std::optional<std::filesystem::path> find_java(
		const std::optional<std::filesystem::path> &game_dir,
		const std::string &component, const std::filesystem::path &cache_dir);
} // namespace arnis::live::mojang
