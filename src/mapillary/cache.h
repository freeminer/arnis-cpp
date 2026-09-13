#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace arnis::mapillary::cache
{
enum class ImageSize
{
	W1024,
	W2048,
	Original
};
struct ImageRecord
{
	std::string id, thumb_1024_url, thumb_2048_url, creator;
	double longitude = 0, latitude = 0, compass_angle = 0;
	bool panorama = false;
};

// Rust-compatible Mapillary cache layout.  It deliberately exposes paths only
// after validating untrusted IDs, so library clients cannot escape the cache.
class Layout
{
	std::filesystem::path root_;

public:
	explicit Layout(std::filesystem::path root) : root_(std::move(root)) {}
	const std::filesystem::path &root() const { return root_; }
	std::optional<std::filesystem::path> meta_path(const std::string &) const;
	std::optional<std::filesystem::path> image_path(const std::string &, ImageSize) const;
	std::optional<std::filesystem::path> cluster_path(const std::string &) const;
	std::filesystem::path facade_dir(
			const std::string &params_digest, unsigned epoch = 1) const;
	std::optional<ImageRecord> load_metadata(const std::string &) const;
	bool save_metadata(const ImageRecord &) const;
	std::optional<std::vector<std::uint8_t>> load_image(
			const std::string &, ImageSize) const;
	bool save_image(
			const std::string &, ImageSize, const std::vector<std::uint8_t> &) const;
	std::optional<std::vector<std::uint8_t>> load_cluster(const std::string &) const;
	bool save_cluster(const std::string &, const std::vector<std::uint8_t> &) const;
	static bool safe_key(const std::string &);
};
} // namespace arnis::mapillary::cache
