#pragma once
#include "../provider.h"
#include "wikidata_index.h"
#include <unordered_map>
#include <functional>
// QID-level memoization keeps multi-pass map generation deterministic and network-sparse.
namespace arnis::models_3d
{
class RemoteModelProvider : public ModelProvider
{
	std::filesystem::path cache_;
	std::unordered_map<std::string, std::optional<ModelAsset>> memo_;
	std::function<std::optional<std::vector<std::uint8_t>>(
			const std::string &, std::size_t)>
			fetch_bytes_;

public:
	explicit RemoteModelProvider(std::filesystem::path cache) : cache_(std::move(cache))
	{
	}
	RemoteModelProvider(std::filesystem::path cache,
			std::function<std::optional<std::vector<std::uint8_t>>(
					const std::string &, std::size_t)>
					fetch_bytes) :
			cache_(std::move(cache)), fetch_bytes_(std::move(fetch_bytes))
	{
	}
	void set_fetcher(std::function<std::optional<std::vector<std::uint8_t>>(
					const std::string &, std::size_t)>
					fetch_bytes)
	{
		fetch_bytes_ = std::move(fetch_bytes);
	}
	std::optional<ModelAsset> fetch(const std::string &) override;
};
}
