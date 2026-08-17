#pragma once
#include <cstdint>
#include "../provider.h"
#include <filesystem>
#include <functional>
#include <optional>
#include <utility>
#include <vector>
namespace arnis::models_3d::custom
{
class Client : public ModelProvider
{
	std::filesystem::path root_;
	// The caller owns networking.  This keeps map generation usable as a
	// library, while preserving Rust's cache-first model acquisition policy.
	using FetchBytes = std::function<std::optional<std::vector<std::uint8_t>>(
			const std::string &url, std::size_t maximum_bytes)>;
	FetchBytes fetcher_;

public:
	explicit Client(std::filesystem::path root, FetchBytes fetcher = {}) :
			root_(std::move(root)), fetcher_(std::move(fetcher))
	{
	}
	void set_fetcher(FetchBytes fetcher) { fetcher_ = std::move(fetcher); }
	std::optional<ModelAsset> fetch(const std::string &key) override;
};
}
