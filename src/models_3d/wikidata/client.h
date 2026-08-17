#pragma once
#include <filesystem>
#include <optional>
#include <string>
#include <vector>
#include <cstdint>
namespace arnis::models_3d::wikidata_client
{
inline constexpr std::size_t MAX_MODEL_BYTES = 128 * 1024 * 1024;
std::filesystem::path cache_root(const std::filesystem::path &base = {});
std::string url_hash(const std::string &url);
std::filesystem::path cache_path(const std::filesystem::path &, const std::string &url);
bool valid_model_bytes(const std::vector<std::uint8_t> &);
std::optional<std::vector<std::uint8_t>> load_cached(
		const std::filesystem::path &, const std::string &url);
bool save_cached(const std::filesystem::path &, const std::string &url,
		const std::vector<std::uint8_t> &);
std::size_t clear_cache(const std::filesystem::path &base = {});
}
