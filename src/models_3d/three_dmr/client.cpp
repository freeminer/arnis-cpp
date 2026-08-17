#include "client.h"
#include "../model_asset.h"
#include <fstream>
#include <cmath>
#include <nlohmann/json.hpp>
namespace arnis::models_3d::three_dmr
{
std::filesystem::path cache_root(const std::filesystem::path &base)
{
	return base.empty() ? std::filesystem::path("./.arnis_3dmr_cache") : base;
}
std::string info_url(std::uint64_t id)
{
	return "https://3dmr.eu/api/info/" + std::to_string(id);
}
std::string model_url(std::uint64_t id)
{
	return "https://3dmr.eu/api/model/" + std::to_string(id);
}
std::string info_url(const ClientConfig &c, std::uint64_t id)
{
	return c.api_base + "/info/" + std::to_string(id);
}
std::string model_url(const ClientConfig &c, std::uint64_t id)
{
	return c.api_base + "/model/" + std::to_string(id);
}
std::filesystem::path info_cache_path(const std::filesystem::path &b, std::uint64_t id)
{
	return cache_root(b) / (std::to_string(id) + ".json");
}
std::filesystem::path model_cache_path(const std::filesystem::path &b, std::uint64_t id)
{
	return cache_root(b) / (std::to_string(id) + ".glb");
}
std::filesystem::path info_cache_path(const ClientConfig &c, std::uint64_t id)
{
	return info_cache_path(c.cache, id);
}
std::filesystem::path model_cache_path(const ClientConfig &c, std::uint64_t id)
{
	return model_cache_path(c.cache, id);
}
std::optional<std::vector<std::uint8_t>> read_capped(
		const std::filesystem::path &p, std::size_t cap)
{
	std::ifstream in(p, std::ios::binary);
	if (!in)
		return std::nullopt;
	in.seekg(0, std::ios::end);
	auto n = in.tellg();
	if (n < 0 || std::uint64_t(n) > cap)
		return std::nullopt;
	in.seekg(0);
	std::vector<std::uint8_t> b(static_cast<std::size_t>(n));
	in.read(reinterpret_cast<char *>(b.data()), std::streamsize(b.size()));
	if (!in)
		return std::nullopt;
	return b;
}
bool write_atomic(const std::filesystem::path &p, const std::vector<std::uint8_t> &b)
{
	if (!p.parent_path().empty())
		std::filesystem::create_directories(p.parent_path());
	auto t = p;
	t += ".tmp";
	std::ofstream out(t, std::ios::binary);
	if (!out)
		return false;
	out.write(reinterpret_cast<const char *>(b.data()), std::streamsize(b.size()));
	if (!out)
		return false;
	out.close();
	std::error_code ec;
	std::filesystem::rename(t, p, ec);
	if (ec) {
		std::filesystem::remove(t);
		return false;
	}
	return true;
}
bool invalidate_cache(const std::filesystem::path &p)
{
	std::error_code ec;
	return std::filesystem::remove(p, ec) || !std::filesystem::exists(p);
}
std::optional<ModelInfo> parse_model_info(const std::vector<std::uint8_t> &b)
{
	try {
		const auto j = nlohmann::json::parse(b.begin(), b.end());
		if (!j.is_object() || !j.contains("id") || !j["id"].is_number_unsigned())
			return std::nullopt;
		ModelInfo m;
		m.id = j["id"].get<std::uint64_t>();
		if (const auto it = j.find("title"); it != j.end() && it->is_string())
			m.title = it->get<std::string>();
		if (const auto it = j.find("author"); it != j.end() && it->is_string())
			m.author = it->get<std::string>();
		// Rust accepts either numeric or string license codes.
		if (const auto it = j.find("license"); it != j.end() && !it->is_null()) {
			if (it->is_string())
				m.license = it->get<std::string>();
			else if (it->is_number_integer() || it->is_number_unsigned() ||
					 it->is_number_float())
				m.license = it->dump();
		}
		auto number = [&](const char *key, double fallback) {
			const auto it = j.find(key);
			return it != j.end() && it->is_number() ? it->get<double>() : fallback;
		};
		m.lat = number("lat", 0.0);
		m.lon = number("lon", 0.0);
		m.rotation = number("rotation", 0.0);
		m.scale = number("scale", 1.0);
		if (const auto it = j.find("translation");
				it != j.end() && it->is_array() && it->size() == 3 &&
				(*it)[0].is_number() && (*it)[1].is_number() && (*it)[2].is_number())
			for (std::size_t i = 0; i < 3; ++i)
				m.translation[i] = (*it)[i].get<double>();
		return normalize_model_info(std::move(m));
	} catch (...) {
		return std::nullopt;
	}
}
std::vector<std::uint8_t> encode_model_info(const ModelInfo &m)
{
	const auto v = normalize_model_info(m);
	nlohmann::json j{{"id", v.id}, {"rotation", v.rotation}, {"scale", v.scale},
			{"translation", v.translation}, {"lat", v.lat}, {"lon", v.lon}};
	if (!v.title.empty())
		j["title"] = v.title;
	if (!v.author.empty())
		j["author"] = v.author;
	if (!v.license.empty())
		j["license"] = v.license;
	const std::string s = j.dump();
	return {s.begin(), s.end()};
}
ModelInfo normalize_model_info(ModelInfo m)
{
	if (!std::isfinite(m.scale) || m.scale <= 0)
		m.scale = 1;
	if (!std::isfinite(m.rotation))
		m.rotation = 0;
	for (auto &v : m.translation)
		if (!std::isfinite(v))
			v = 0;
	return m;
}
bool valid_model_info(const ModelInfo &m)
{
	return m.id > 0 && std::isfinite(m.scale) && m.scale > 0 && std::isfinite(m.rotation);
}
std::optional<ModelInfo> load_valid_info_cache(
		const std::filesystem::path &b, std::uint64_t id)
{
	auto m = load_info_cache(b, id);
	if (!m || !valid_model_info(*m)) {
		invalidate_cache(info_cache_path(b, id));
		return std::nullopt;
	}
	return m;
}
std::size_t clear_cache(const std::filesystem::path &base)
{
	const auto d = cache_root(base);
	if (!std::filesystem::exists(d))
		return 0;
	std::size_t n = 0;
	for (const auto &e : std::filesystem::directory_iterator(d)) {
		auto x = e.path().extension();
		if (x == ".json" || x == ".glb" || x == ".tmp") {
			std::error_code ec;
			std::filesystem::remove(e.path(), ec);
			if (!ec)
				++n;
		}
	}
	return n;
}
std::size_t clear_cache(const ClientConfig &c)
{
	return clear_cache(c.cache);
}
std::optional<std::vector<std::uint8_t>> load_valid_glb_cache(
		const ClientConfig &c, std::uint64_t id)
{
	auto b = read_capped(model_cache_path(c, id), MAX_GLB_BYTES);
	if (!b || !valid_glb_bytes(*b)) {
		invalidate_cache(model_cache_path(c, id));
		return std::nullopt;
	}
	return b;
}
std::optional<ModelInfo> load_valid_info_cache(const ClientConfig &c, std::uint64_t id)
{
	auto m = load_info_cache(c.cache, id);
	if (!m || !valid_model_info(*m)) {
		invalidate_cache(info_cache_path(c, id));
		return std::nullopt;
	}
	return m;
}
std::optional<ModelInfo> load_info_cache(const std::filesystem::path &b, std::uint64_t id)
{
	const auto bytes = read_capped(info_cache_path(b, id), 1024 * 1024);
	if (!bytes)
		return std::nullopt;
	auto m = parse_model_info(*bytes);
	return m ? std::optional<ModelInfo>(normalize_model_info(*m)) : std::nullopt;
}
bool save_info_cache(const std::filesystem::path &b, const ModelInfo &m)
{
	return write_atomic(
			info_cache_path(b, m.id), encode_model_info(normalize_model_info(m)));
}
std::optional<std::vector<std::uint8_t>> load_glb_cache(
		const std::filesystem::path &b, std::uint64_t id)
{
	auto bytes = read_capped(model_cache_path(b, id), MAX_GLB_BYTES);
	if (!bytes || bytes->empty())
		return std::nullopt;
	return bytes;
}
bool save_glb_cache(const std::filesystem::path &b, std::uint64_t id,
		const std::vector<std::uint8_t> &bytes)
{
	if (bytes.empty() || bytes.size() > MAX_GLB_BYTES)
		return false;
	return write_atomic(model_cache_path(b, id), bytes);
}
bool valid_glb_bytes(const std::vector<std::uint8_t> &b)
{
	return b.size() >= 12 && b[0] == 'g' && b[1] == 'l' && b[2] == 'T' && b[3] == 'F' &&
		   b[4] == 2;
}
std::optional<std::vector<std::uint8_t>> load_valid_glb_cache(
		const std::filesystem::path &base, std::uint64_t id)
{
	auto b = read_capped(model_cache_path(base, id), MAX_GLB_BYTES);
	if (!b || !valid_glb_bytes(*b)) {
		invalidate_cache(model_cache_path(base, id));
		return std::nullopt;
	}
	return b;
}
std::optional<ModelAsset> Client::fetch(const std::string &key)
{
	if (key.empty() || key.size() > 20)
		return std::nullopt;
	std::uint64_t id = 0;
	try {
		std::size_t end = 0;
		id = std::stoull(key, &end);
		if (end != key.size() || id == 0)
			return std::nullopt;
	} catch (...) {
		return std::nullopt;
	}
	const auto path = model_cache_path(config_, id);
	if (!fetch_glb(id))
		return std::nullopt;
	try {
		return load_model_asset(path, ModelFormat::GLB);
	} catch (...) {
		return std::nullopt;
	}
}

std::optional<ModelInfo> Client::fetch_info(std::uint64_t id) const
{
	if (!id)
		return std::nullopt;
	if (auto cached = load_valid_info_cache(config_, id))
		return cached;
	if (!fetch_bytes_)
		return std::nullopt;
	auto bytes = fetch_bytes_(info_url(config_, id), 1024 * 1024);
	if (!bytes || bytes->empty())
		return std::nullopt;
	auto info = parse_model_info(*bytes);
	if (!info || info->id != id || !valid_model_info(*info))
		return std::nullopt;
	// Preserve the server response verbatim when possible; save_info_cache is
	// still used as a safe normalized fallback for library fetchers.
	if (!write_atomic(info_cache_path(config_, id), *bytes))
		save_info_cache(config_.cache, *info);
	return info;
}

std::optional<std::vector<std::uint8_t>> Client::fetch_glb(std::uint64_t id) const
{
	if (!id)
		return std::nullopt;
	if (auto cached = load_valid_glb_cache(config_, id))
		return cached;
	if (!fetch_bytes_)
		return std::nullopt;
	auto bytes = fetch_bytes_(model_url(config_, id), MAX_GLB_BYTES);
	if (!bytes || bytes->size() > MAX_GLB_BYTES || !valid_glb_bytes(*bytes))
		return std::nullopt;
	if (!save_glb_cache(config_.cache, id, *bytes))
		return std::nullopt;
	return bytes;
}
}
