#pragma once
#include "../provider.h"
#include <filesystem>
#include <utility>
#include <array>
#include <functional>
namespace arnis::models_3d::three_dmr
{
inline constexpr std::size_t MAX_GLB_BYTES = 64 * 1024 * 1024;
struct ClientConfig
{
	std::string api_base = "https://3dmr.eu/api";
	std::filesystem::path cache;
	unsigned timeout_seconds = 20;
};
// Library-owned transport seam.  It receives the absolute API URL and byte
// cap, returning no value on HTTP/transport failure.  This keeps the model
// pipeline usable by Freeminer and embedding hosts without hard-wiring a
// second HTTP stack.
using FetchBytes = std::function<std::optional<std::vector<std::uint8_t>>(
		const std::string &url, std::size_t max_bytes)>;
struct ModelInfo
{
	std::uint64_t id = 0;
	std::string title, author, license;
	double lat = 0, lon = 0, rotation = 0, scale = 1;
	std::array<double, 3> translation{0, 0, 0};
};
std::filesystem::path cache_root(const std::filesystem::path &base = {});
std::string info_url(std::uint64_t id);
std::string model_url(std::uint64_t id);
std::string info_url(const ClientConfig &, std::uint64_t id);
std::string model_url(const ClientConfig &, std::uint64_t id);
std::filesystem::path info_cache_path(const std::filesystem::path &, std::uint64_t id);
std::filesystem::path model_cache_path(const std::filesystem::path &, std::uint64_t id);
std::filesystem::path info_cache_path(const ClientConfig &, std::uint64_t id);
std::filesystem::path model_cache_path(const ClientConfig &, std::uint64_t id);
std::optional<std::vector<std::uint8_t>> read_capped(
		const std::filesystem::path &, std::size_t cap = MAX_GLB_BYTES);
bool write_atomic(const std::filesystem::path &, const std::vector<std::uint8_t> &);
bool invalidate_cache(const std::filesystem::path &);
std::optional<ModelInfo> parse_model_info(const std::vector<std::uint8_t> &);
std::vector<std::uint8_t> encode_model_info(const ModelInfo &);
ModelInfo normalize_model_info(ModelInfo);
bool valid_model_info(const ModelInfo &);
std::optional<ModelInfo> load_valid_info_cache(
		const std::filesystem::path &, std::uint64_t id);
std::size_t clear_cache(const std::filesystem::path &base = {});
std::size_t clear_cache(const ClientConfig &);
std::optional<std::vector<std::uint8_t>> load_valid_glb_cache(
		const ClientConfig &, std::uint64_t id);
std::optional<ModelInfo> load_valid_info_cache(const ClientConfig &, std::uint64_t id);
std::optional<ModelInfo> load_info_cache(const std::filesystem::path &, std::uint64_t id);
bool save_info_cache(const std::filesystem::path &, const ModelInfo &);
std::optional<std::vector<std::uint8_t>> load_glb_cache(
		const std::filesystem::path &, std::uint64_t id);
bool save_glb_cache(const std::filesystem::path &, std::uint64_t id,
		const std::vector<std::uint8_t> &);
bool valid_glb_bytes(const std::vector<std::uint8_t> &);
std::optional<std::vector<std::uint8_t>> load_valid_glb_cache(
		const std::filesystem::path &, std::uint64_t id);
class Client : public ModelProvider
{
	ClientConfig config_;
	FetchBytes fetch_bytes_;

public:
	explicit Client(std::filesystem::path cache) { config_.cache = std::move(cache); }
	Client(ClientConfig config, FetchBytes fetch_bytes = {}) :
			config_(std::move(config)), fetch_bytes_(std::move(fetch_bytes))
	{
	}
	void set_fetcher(FetchBytes fetch_bytes) { fetch_bytes_ = std::move(fetch_bytes); }
	const ClientConfig &config() const { return config_; }
	std::optional<ModelInfo> fetch_info(std::uint64_t id) const;
	std::optional<std::vector<std::uint8_t>> fetch_glb(std::uint64_t id) const;
	std::optional<ModelAsset> fetch(const std::string &key) override;
	std::optional<ModelInfo> info(std::uint64_t id) const { return fetch_info(id); }
};
}
