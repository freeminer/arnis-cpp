#include "cache.h"
#include "../cache_root.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>

#if defined(_WIN32)
#include <process.h>
#define ARNIS_CACHE_GETPID _getpid
#else
#include <unistd.h>
#define ARNIS_CACHE_GETPID getpid
#endif

namespace arnis::mapillary::cache
{
std::filesystem::path default_root()
{
	// Keep the Mapillary cache below the same shared tree as elevation and
	// land-cover tiles.  This is the Rust layout (cache/arnis-tile-cache/
	// mapillary) and lets the existing cache cleanup reach imagery, SfM
	// clusters, and reconstructed facade products together.
	return arnis::cache::provider_cache_root("mapillary");
}

namespace
{
std::atomic<std::uint64_t> temp_counter{0};

const char *suffix(ImageSize size)
{
	switch (size) {
	case ImageSize::W1024:
		return "1024";
	case ImageSize::W2048:
		return "2048";
	case ImageSize::Original:
		return "orig";
	}
	return "2048";
}
std::string shard(const std::string &key)
{
	return key.size() < 2 ? key : key.substr(key.size() - 2);
}
} // namespace

bool is_cached(const std::filesystem::path &path)
{
	std::error_code ec;
	return std::filesystem::is_regular_file(path, ec) && !ec &&
		   std::filesystem::file_size(path, ec) > 0 && !ec;
}

bool write_atomic(
		const std::filesystem::path &path, const std::vector<std::uint8_t> &bytes)
{
	if (bytes.empty() || path.parent_path().empty())
		return false;
	std::error_code ec;
	std::filesystem::create_directories(path.parent_path(), ec);
	if (ec)
		return false;
	auto temporary = path;
	temporary += ".tmp" + std::to_string(ARNIS_CACHE_GETPID()) +
				 std::to_string(temp_counter.fetch_add(1, std::memory_order_relaxed));
	{
		std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
		if (!output)
			return false;
		output.write(reinterpret_cast<const char *>(bytes.data()),
				static_cast<std::streamsize>(bytes.size()));
		output.close();
		if (!output) {
			std::filesystem::remove(temporary, ec);
			return false;
		}
	}
	std::filesystem::rename(temporary, path, ec);
	if (!ec)
		return true;
	std::filesystem::remove(temporary, ec);
	return false;
}

bool Layout::safe_key(const std::string &key)
{
	return !key.empty() && key.size() <= 128 &&
		   std::all_of(key.begin(), key.end(), [](unsigned char c) {
			   return std::isalnum(c) || c == '_' || c == '-';
		   });
}

std::optional<std::filesystem::path> Layout::meta_path(const std::string &id) const
{
	if (!safe_key(id))
		return {};
	return root_ / "meta" / shard(id) / (id + ".json");
}

std::optional<std::filesystem::path> Layout::image_path(
		const std::string &id, ImageSize size) const
{
	if (!safe_key(id))
		return {};
	return root_ / "img" / shard(id) / (id + "_" + suffix(size) + ".jpg");
}

std::optional<std::filesystem::path> Layout::cluster_path(const std::string &id) const
{
	if (!safe_key(id))
		return {};
	return root_ / "sfm" / shard(id) / (id + ".json.zz");
}

std::filesystem::path Layout::facade_dir(
		const std::string &params_digest, unsigned epoch) const
{
	return root_ / "facades" /
		   ("e" + std::to_string(epoch) + "-" +
				   (safe_key(params_digest) ? params_digest : std::string{"invalid"}));
}
std::optional<ImageRecord> Layout::load_metadata(const std::string &id) const
{
	auto p = meta_path(id);
	if (!p)
		return {};
	std::ifstream in(*p);
	if (!in)
		return {};
	try {
		nlohmann::json j;
		in >> j;
		ImageRecord r;
		r.id = j.value("id", "");
		r.raw_json = j.value("raw_json", std::string{});
		r.thumb_1024_url = j.value("thumb_1024_url", "");
		r.thumb_2048_url = j.value("thumb_2048_url", "");
		r.thumb_original_url = j.value("thumb_original_url", "");
		r.creator = j.value("creator", "");
		r.creator_id = j.value("creator_id", std::string{});
		r.longitude = j.value("longitude", 0.0);
		r.latitude = j.value("latitude", 0.0);
		r.compass_angle = j.value("compass_angle", 0.0);
		r.panorama = j.value("is_pano", false);
		return r.id == id && safe_key(r.id) ? std::optional<ImageRecord>{std::move(r)}
											: std::nullopt;
	} catch (...) {
		return {};
	}
}
bool Layout::save_metadata(const ImageRecord &r) const
{
	auto p = meta_path(r.id);
	if (!p)
		return false;
	const auto serialized = nlohmann::json{{"id", r.id}, {"raw_json", r.raw_json},
			{"thumb_1024_url", r.thumb_1024_url}, {"thumb_2048_url", r.thumb_2048_url},
			{"thumb_original_url", r.thumb_original_url}, {"creator", r.creator},
			{"creator_id", r.creator_id}, {"longitude", r.longitude},
			{"latitude", r.latitude}, {"compass_angle", r.compass_angle},
			{"is_pano", r.panorama}}.dump();
	return write_atomic(
			*p, std::vector<std::uint8_t>(serialized.begin(), serialized.end()));
}

namespace
{
std::optional<std::vector<std::uint8_t>> read_bytes(
		const std::optional<std::filesystem::path> &p)
{
	if (!p)
		return {};
	std::ifstream in(*p, std::ios::binary);
	if (!in)
		return {};
	std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)), {});
	return bytes.empty() ? std::nullopt
						 : std::optional<std::vector<std::uint8_t>>{std::move(bytes)};
}
bool write_bytes(
		const std::optional<std::filesystem::path> &p, const std::vector<std::uint8_t> &b)
{
	return p && write_atomic(*p, b);
}
}
std::optional<std::vector<std::uint8_t>> Layout::load_image(
		const std::string &id, ImageSize s) const
{
	return read_bytes(image_path(id, s));
}
bool Layout::save_image(
		const std::string &id, ImageSize s, const std::vector<std::uint8_t> &b) const
{
	return write_bytes(image_path(id, s), b);
}
std::optional<std::vector<std::uint8_t>> Layout::load_cluster(const std::string &id) const
{
	return read_bytes(cluster_path(id));
}
bool Layout::save_cluster(const std::string &id, const std::vector<std::uint8_t> &b) const
{
	return write_bytes(cluster_path(id), b);
}
} // namespace arnis::mapillary::cache
