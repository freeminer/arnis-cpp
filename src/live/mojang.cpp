#include "mojang.h"

#include "../net.h"
#include "../world_utils.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <memory>
#include <string_view>
#include <system_error>

#if USE_CURL
#include <curl/curl.h>
#endif

namespace arnis::live::mojang
{
namespace
{
constexpr const char *VERSION_MANIFEST =
		"https://piston-meta.mojang.com/mc/game/version_manifest_v2.json";
constexpr auto CACHE_AGE = std::chrono::hours(1);

std::optional<std::string> read_file(const std::filesystem::path &path)
{
	std::ifstream in(path, std::ios::binary);
	if (!in)
		return std::nullopt;
	return std::string(
			std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

std::size_t append_body(char *data, std::size_t size, std::size_t count, void *user)
{
	const auto bytes = size * count;
	static_cast<std::string *>(user)->append(data, bytes);
	return bytes;
}

std::optional<std::string> get_text(const std::string &url, std::string *error)
{
#if USE_CURL
	auto permit = net::request_permit("Mojang metadata");
	CURL *curl = curl_easy_init();
	if (!curl) {
		if (error)
			*error = "Could not initialize HTTP client";
		return std::nullopt;
	}
	std::string body;
	curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_body);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
	curl_easy_setopt(curl, CURLOPT_USERAGENT, "arnis/1.0");
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
	curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 15L);
	curl_easy_setopt(curl, CURLOPT_TIMEOUT, 900L);
	const auto result = curl_easy_perform(curl);
	long status = 0;
	curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
	curl_easy_cleanup(curl);
	if (result != CURLE_OK || status < 200 || status >= 300) {
		if (error)
			*error = result == CURLE_OK ? "HTTP status " + std::to_string(status)
										: curl_easy_strerror(result);
		return std::nullopt;
	}
	return body;
#else
	(void)url;
	if (error)
		*error = "HTTP support is not available in this build";
	return std::nullopt;
#endif
}

std::vector<std::uint8_t> as_bytes(std::string_view text)
{
	return {text.begin(), text.end()};
}

std::optional<nlohmann::json> parse_json(const std::string &text)
{
	try {
		return nlohmann::json::parse(text);
	} catch (const nlohmann::json::exception &) {
		return std::nullopt;
	}
}

std::optional<std::string> json_string(const nlohmann::json &value, const char *key)
{
	const auto it = value.find(key);
	if (it == value.end() || !it->is_string())
		return std::nullopt;
	return it->get<std::string>();
}

std::filesystem::path home_dir()
{
#if defined(_WIN32)
	if (const char *profile = std::getenv("USERPROFILE"))
		return profile;
#else
	if (const char *home = std::getenv("HOME"))
		return home;
#endif
	return {};
}

std::optional<std::string> runtime_platform()
{
#if defined(__APPLE__) && defined(__aarch64__)
	return "mac-os-arm64";
#elif defined(__APPLE__)
	return "mac-os";
#elif defined(_WIN32) && defined(_M_ARM64)
	return "windows-arm64";
#elif defined(_WIN32) && (defined(_M_IX86) || defined(__i386__))
	return "windows-x86";
#elif defined(_WIN32)
	return "windows-x64";
#elif defined(__linux__) && defined(__i386__)
	return "linux-i386";
#elif defined(__linux__) && (defined(__x86_64__) || defined(_M_X64))
	return "linux";
#else
	return std::nullopt;
#endif
}

std::optional<std::filesystem::path> java_in(const std::filesystem::path &root)
{
#if defined(_WIN32)
	constexpr const char *exe = "java.exe";
#else
	constexpr const char *exe = "java";
#endif
	std::error_code ec;
	for (const auto &path :
			{root / "bin" / exe, root / "jre.bundle/Contents/Home/bin" / exe})
		if (std::filesystem::is_regular_file(path, ec))
			return path;
	return std::nullopt;
}

std::optional<std::string> followed_version(const std::string &initial,
		const std::filesystem::path &game_dir, const Manifest &manifest);
} // namespace

std::optional<std::filesystem::path> default_game_dir()
{
	std::filesystem::path home = home_dir();
	if (home.empty())
		return std::nullopt;
#if defined(_WIN32)
	if (const char *appdata = std::getenv("APPDATA"))
		return std::filesystem::path(appdata) / ".minecraft";
	return home / "AppData" / "Roaming" / ".minecraft";
#elif defined(__APPLE__)
	return home / "Library" / "Application Support" / "minecraft";
#else
	auto flatpak = home / ".var/app/com.mojang.Minecraft/.minecraft";
	std::error_code ec;
	if (std::filesystem::is_directory(flatpak, ec))
		return flatpak;
	return home / ".minecraft";
#endif
}

std::optional<std::filesystem::path> game_dir(const std::filesystem::path &saves_dir)
{
	const auto parent = saves_dir.parent_path();
	std::error_code ec;
	if (!parent.empty() &&
			(std::filesystem::is_regular_file(parent / "launcher_profiles.json", ec) ||
					std::filesystem::is_directory(parent / "versions", ec)))
		return parent;
	auto fallback = default_game_dir();
	if (fallback && std::filesystem::is_directory(*fallback, ec))
		return fallback;
	return std::nullopt;
}

std::optional<std::filesystem::path> find_java(
		const std::optional<std::filesystem::path> &game_dir,
		const std::string &component, const std::filesystem::path &cache_dir)
{
	const auto platform = runtime_platform();
	if (!platform)
		return std::nullopt;
	std::vector<std::filesystem::path> roots;
	if (game_dir)
		roots.push_back(*game_dir / "runtime");
#if defined(_WIN32)
	if (const char *local = std::getenv("LOCALAPPDATA"))
		roots.emplace_back(
				std::filesystem::path(local) /
				"Packages/Microsoft.4297127D64EC6_8wekyb3d8bbwe/LocalCache/Local/runtime");
	for (const char *var : {"ProgramFiles(x86)", "ProgramFiles"})
		if (const char *program_files = std::getenv(var))
			roots.emplace_back(
					std::filesystem::path(program_files) / "Minecraft Launcher/runtime");
#endif
	for (const auto &root : roots)
		if (auto java = java_in(root / component / *platform / component))
			return java;
	return java_in(cache_dir / "runtime" / component);
}

std::optional<Manifest> Manifest::load(
		const std::filesystem::path &cache_dir, std::string *error)
{
	const auto path = cache_dir / "version_manifest_v2.json";
	auto cached = read_file(path);
	std::error_code ec;
	bool fresh = false;
	if (std::filesystem::exists(path, ec)) {
		const auto modified = std::filesystem::last_write_time(path, ec);
		if (!ec) {
			const auto age = decltype(modified)::clock::now() - modified;
			fresh = age >= std::filesystem::file_time_type::duration::zero() &&
					age < CACHE_AGE;
		}
	}
	if (!fresh) {
		std::string download_error;
		if (auto text = get_text(VERSION_MANIFEST, &download_error)) {
			std::filesystem::create_directories(cache_dir, ec);
			(void)world_utils::replace_file_atomically(path, as_bytes(*text));
			cached = std::move(text);
		} else if (!cached) {
			if (error)
				*error = "Could not reach Mojang's version list: " + download_error;
			return std::nullopt;
		}
	}
	if (!cached) {
		if (error)
			*error = "Could not read Mojang version manifest";
		return std::nullopt;
	}
	auto json = parse_json(*cached);
	if (!json) {
		if (error)
			*error = "Could not parse Mojang version manifest";
		return std::nullopt;
	}
	return Manifest(std::make_shared<const nlohmann::json>(std::move(*json)));
}

std::optional<std::string> Manifest::latest_release() const
{
	if (!json_ || !json_->contains("latest") || !json_->at("latest").is_object())
		return std::nullopt;
	return json_string(json_->at("latest"), "release");
}

std::vector<std::string> Manifest::supported_releases() const
{
	std::vector<std::string> out;
	if (!json_ || !json_->contains("versions") || !json_->at("versions").is_array())
		return out;
	for (const auto &entry : json_->at("versions")) {
		auto id = json_string(entry, "id");
		if (!id)
			continue;
		if (json_string(entry, "type") == std::optional<std::string>("release"))
			out.push_back(*id);
		if (*id == OLDEST_VERSION)
			break;
	}
	return out;
}

std::optional<std::string> Manifest::release_time(const std::string &id) const
{
	if (!json_ || !json_->contains("versions") || !json_->at("versions").is_array())
		return std::nullopt;
	for (const auto &entry : json_->at("versions"))
		if (json_string(entry, "id") == id)
			return json_string(entry, "releaseTime");
	return std::nullopt;
}

bool Manifest::is_supported(const std::string &id) const
{
	const auto oldest = release_time(OLDEST_VERSION);
	const auto version = release_time(id);
	return oldest && version && *version >= *oldest;
}

std::optional<VersionInfo> Manifest::version_info(const std::string &id,
		const std::filesystem::path &cache_dir, std::string *error) const
{
	if (!json_ || !json_->contains("versions") || !json_->at("versions").is_array()) {
		if (error)
			*error = "Mojang version list is missing versions";
		return std::nullopt;
	}
	const nlohmann::json *entry = nullptr;
	for (const auto &candidate : json_->at("versions"))
		if (json_string(candidate, "id") == id) {
			entry = &candidate;
			break;
		}
	if (!entry) {
		if (error)
			*error = "Minecraft " + id + " is not in Mojang's version list";
		return std::nullopt;
	}
	const auto url = json_string(*entry, "url");
	if (!url) {
		if (error)
			*error = "Mojang version entry has no URL";
		return std::nullopt;
	}
	const auto path = cache_dir / "versions" / id / "version.json";
	auto text = read_file(path);
	if (!text) {
		std::string request_error;
		text = get_text(*url, &request_error);
		if (!text) {
			if (error)
				*error = std::move(request_error);
			return std::nullopt;
		}
		std::error_code ec;
		std::filesystem::create_directories(path.parent_path(), ec);
		(void)world_utils::replace_file_atomically(path, as_bytes(*text));
	}
	auto json = parse_json(*text);
	if (!json) {
		if (error)
			*error = "Could not parse Minecraft version metadata";
		return std::nullopt;
	}
	const nlohmann::json *server = nullptr;
	if (json->contains("downloads") && json->at("downloads").is_object() &&
			json->at("downloads").contains("server"))
		server = &json->at("downloads").at("server");
	const auto server_url = server ? json_string(*server, "url") : std::nullopt;
	if (!server_url) {
		if (error)
			*error = "Mojang offers no server for " + id;
		return std::nullopt;
	}
	VersionInfo result;
	result.id = id;
	result.server_url = *server_url;
	if (server)
		result.server_sha1 = json_string(*server, "sha1").value_or("");
	if (server && server->contains("size") && server->at("size").is_number_unsigned())
		result.server_size = server->at("size").get<std::uint64_t>();
	if (json->contains("javaVersion") && json->at("javaVersion").is_object()) {
		const auto &java = json->at("javaVersion");
		result.java_component =
				json_string(java, "component").value_or("java-runtime-delta");
		if (java.contains("majorVersion") && java.at("majorVersion").is_number_unsigned())
			result.java_major = java.at("majorVersion").get<std::uint32_t>();
	}
	return result;
}

std::optional<std::string> Manifest::last_played(
		const std::filesystem::path &game_dir) const
{
	std::optional<std::pair<std::string, std::string>> latest;
	for (const char *filename :
			{"launcher_profiles.json", "launcher_profiles_microsoft_store.json"}) {
		auto text = read_file(game_dir / filename);
		if (!text)
			continue;
		auto document = parse_json(*text);
		if (!document || !document->contains("profiles") ||
				!document->at("profiles").is_object())
			continue;
		for (auto it = document->at("profiles").begin();
				it != document->at("profiles").end(); ++it) {
			const auto version = json_string(it.value(), "lastVersionId");
			if (!version)
				continue;
			const auto used = json_string(it.value(), "lastUsed").value_or("");
			if (!latest || used > latest->first)
				latest = std::pair{used, *version};
		}
	}
	if (!latest)
		return std::nullopt;
	const auto id = latest->second;
	if (id == "latest-release")
		return latest_release();
	if (id == "latest-snapshot" && json_ && json_->contains("latest"))
		return json_string(json_->at("latest"), "snapshot");
	return followed_version(id, game_dir, *this);
}

namespace
{
std::optional<std::string> followed_version(const std::string &initial,
		const std::filesystem::path &game_dir, const Manifest &manifest)
{
	std::string id = initial;
	for (int hop = 0; hop < 4; ++hop) {
		if (manifest.release_time(id))
			return id;
		const auto path = game_dir / "versions" / id / (id + ".json");
		auto text = read_file(path);
		if (!text)
			return std::nullopt;
		auto json = parse_json(*text);
		if (!json)
			return std::nullopt;
		auto parent = json_string(*json, "inheritsFrom");
		if (!parent)
			return std::nullopt;
		id = std::move(*parent);
	}
	return std::nullopt;
}
} // namespace
} // namespace arnis::live::mojang
