#include "cache.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>

namespace arnis::mapillary::cache
{
namespace
{
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
		r.thumb_1024_url = j.value("thumb_1024_url", "");
		r.thumb_2048_url = j.value("thumb_2048_url", "");
		r.creator = j.value("creator", "");
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
	std::error_code e;
	std::filesystem::create_directories(p->parent_path(), e);
	if (e)
		return false;
	auto tmp = *p;
	tmp += ".tmp";
	std::ofstream out(tmp, std::ios::trunc);
	if (!out)
		return false;
	out << nlohmann::json{{"id", r.id}, {"thumb_1024_url", r.thumb_1024_url},
			{"thumb_2048_url", r.thumb_2048_url}, {"creator", r.creator},
			{"longitude", r.longitude}, {"latitude", r.latitude},
			{"compass_angle", r.compass_angle}, {"is_pano", r.panorama}};
	out.close();
	if (!out)
		return false;
	std::filesystem::rename(tmp, *p, e);
	if (!e)
		return true;
	std::filesystem::remove(tmp, e);
	return false;
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
	if (!p || b.empty())
		return false;
	std::error_code e;
	std::filesystem::create_directories(p->parent_path(), e);
	if (e)
		return false;
	auto tmp = *p;
	tmp += ".tmp";
	{
		std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
		if (!out)
			return false;
		out.write(reinterpret_cast<const char *>(b.data()), std::streamsize(b.size()));
		if (!out)
			return false;
	}
	std::filesystem::rename(tmp, *p, e);
	if (!e)
		return true;
	std::filesystem::remove(tmp, e);
	return false;
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
