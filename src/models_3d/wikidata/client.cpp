#include "client.h"
#include <fstream>
#include <cstdio>
namespace arnis::models_3d::wikidata_client
{
std::filesystem::path cache_root(const std::filesystem::path &b)
{
	return b.empty() ? std::filesystem::path("./.arnis_wikidata_cache") : b;
}
std::string url_hash(const std::string &u)
{
	// `fnv::FnvHasher` in the Rust source is 64-bit FNV-1a.  Unlike
	// std::hash this is stable across libstdc++ versions and processes, so
	// both implementations choose the same cache filename for a URL.
	std::uint64_t hash = 0xcbf29ce484222325ULL;
	for (const unsigned char c : u) {
		hash ^= c;
		hash *= 0x100000001b3ULL;
	}
	char b[17];
	std::snprintf(b, sizeof(b), "%016llx", static_cast<unsigned long long>(hash));
	return b;
}
std::filesystem::path cache_path(const std::filesystem::path &b, const std::string &u)
{
	return cache_root(b) / (url_hash(u) + ".bin");
}
bool valid_model_bytes(const std::vector<std::uint8_t> &b)
{
	return b.size() >= 12;
}
std::optional<std::vector<std::uint8_t>> load_cached(
		const std::filesystem::path &b, const std::string &u)
{
	auto p = cache_path(b, u);
	std::ifstream in(p, std::ios::binary);
	if (!in)
		return std::nullopt;
	in.seekg(0, std::ios::end);
	auto n = in.tellg();
	if (n < 12 || std::uint64_t(n) > MAX_MODEL_BYTES)
		return std::nullopt;
	in.seekg(0);
	std::vector<std::uint8_t> v(static_cast<std::size_t>(n));
	in.read(reinterpret_cast<char *>(v.data()), std::streamsize(v.size()));
	return in ? std::optional<std::vector<std::uint8_t>>(std::move(v)) : std::nullopt;
}
bool save_cached(const std::filesystem::path &b, const std::string &u,
		const std::vector<std::uint8_t> &v)
{
	if (!valid_model_bytes(v) || v.size() > MAX_MODEL_BYTES)
		return false;
	auto p = cache_path(b, u);
	std::filesystem::create_directories(p.parent_path());
	auto t = p;
	t += ".tmp";
	std::ofstream out(t, std::ios::binary);
	if (!out)
		return false;
	out.write(reinterpret_cast<const char *>(v.data()), std::streamsize(v.size()));
	out.close();
	std::error_code ec;
	std::filesystem::rename(t, p, ec);
	return !ec;
}
std::size_t clear_cache(const std::filesystem::path &b)
{
	auto d = cache_root(b);
	if (!std::filesystem::exists(d))
		return 0;
	std::size_t n = 0;
	for (auto &e : std::filesystem::directory_iterator(d)) {
		if (e.path().extension() == ".bin" || e.path().extension() == ".tmp") {
			std::error_code ec;
			std::filesystem::remove(e.path(), ec);
			if (!ec)
				++n;
		}
	}
	return n;
}
}
