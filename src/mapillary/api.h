#pragma once
#include "cache.h"
#include "types.h"
#include <nlohmann/json.hpp>
#include <functional>
#include <map>
#include <filesystem>
#include <memory>
#include <mutex>
namespace arnis::mapillary
{
using JsonFetcher = std::function<std::optional<std::vector<std::uint8_t>>(
		const std::string &, std::size_t)>;
struct HttpReply
{
	int status = 200;
	std::vector<std::uint8_t> body;
};
// Status-aware Graph fetch path, required to distinguish a dense-cell 500
// (which Rust subdivides) from an unavailable response or rejected token.
using GraphFetcher =
		std::function<std::optional<HttpReply>(const std::string &, std::size_t)>;

inline constexpr double mapillary_max_cell_deg = .008;
inline constexpr std::size_t mapillary_max_cells = 4096;
struct SearchCell
{
	double min_longitude{}, min_latitude{}, max_longitude{}, max_latitude{};
};
// Graph API bbox searches are deliberately tiled below its 0.01-degree limit.
std::optional<std::vector<SearchCell>> search_cells(const SearchCell &bounds,
		std::size_t maximum = mapillary_max_cells, std::string *error = nullptr);
std::vector<cache::ImageRecord> parse_search_response(const std::vector<std::uint8_t> &);
std::optional<cache::ImageRecord> parse_image_record(const std::vector<std::uint8_t> &);
std::optional<PanoMeta> parse_pano_meta(const std::vector<std::uint8_t> &);
class Client
{
	struct Credentials
	{
		std::mutex mutex;
		std::string endpoint, token;
	};
	cache::Layout cache_;
	JsonFetcher fetch_;
	GraphFetcher graph_fetch_;
	std::shared_ptr<Credentials> credentials_ = std::make_shared<Credentials>();
	// The built-in HTTP transport is safe to use from the bounded search pool.
	// Injected callbacks stay serialized unless they gain an explicit thread-safe
	// contract, which keeps the API's testing/embedding seam predictable.
	bool concurrent_fetch_ = false;
	std::optional<std::vector<std::uint8_t>> fetch_bytes(
			const std::string &url, std::size_t max_bytes) const;
	std::optional<HttpReply> fetch_reply(
			const std::string &url, std::size_t max_bytes) const;
	std::optional<cache::ImageRecord> refresh_metadata(const std::string &id) const;

public:
	explicit Client(cache::Layout cache = cache::Layout{});
	Client(cache::Layout cache, JsonFetcher fetch) :
			cache_(std::move(cache)), fetch_(std::move(fetch))
	{
	}
	Client(cache::Layout cache, GraphFetcher fetch) :
			cache_(std::move(cache)), graph_fetch_(std::move(fetch))
	{
	}
	std::optional<cache::ImageRecord> image(
			const std::string &id, const std::string &url) const;
	std::optional<PanoMeta> metadata(const cache::ImageRecord &) const;
	std::optional<std::filesystem::path> download_image(
			const cache::ImageRecord &, cache::ImageSize) const;
	std::optional<cache::ImageRecord> cached_metadata(const std::string &id) const
	{
		return cache_.load_metadata(id);
	}
	std::optional<std::filesystem::path> download_cluster(
			const cache::ImageRecord &) const;
	std::optional<nlohmann::json> cluster_document(
			const cache::ImageRecord &, std::string *error = nullptr) const;
	std::vector<cache::ImageRecord> search(const SearchCell &,
			const std::string &endpoint, const std::string &token,
			std::size_t maximum = mapillary_max_cells,
			std::size_t *failed_cells = nullptr, std::string *error = nullptr) const;
};
}
