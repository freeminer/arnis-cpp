#include "remote_provider.h"
#include "client.h"
#include "../../../../http.h"
#include <filesystem>
#include <fstream>
namespace arnis::models_3d
{
std::optional<ModelAsset> RemoteModelProvider::fetch(const std::string &q)
{
	auto it = memo_.find(q);
	if (it != memo_.end())
		return it->second;
	auto *e = lookup_wikidata(q);
	if (!e)
		return memo_[q] = {};
	const auto format = wikidata_model_format(q);
	if (!format)
		return memo_[q] = {};
	auto ext = *format == ModelFormat::BinarySTL ? ".stl" : ".glb";
	auto cached = wikidata_client::load_cached(cache_, e->url);
	auto p = cache_ / (wikidata_client::url_hash(e->url) + ext);
	if (!cached) {
		std::vector<std::uint8_t> bytes;
		if (fetch_bytes_) {
			auto fetched = fetch_bytes_(e->url, wikidata_client::MAX_MODEL_BYTES);
			if (!fetched || fetched->size() > wikidata_client::MAX_MODEL_BYTES)
				return memo_[q] = {};
			bytes = std::move(*fetched);
		} else {
			// Compatibility fallback for the in-engine mapgen host.
			if (!http_to_file(e->url, p.string()))
				return memo_[q] = {};
			std::ifstream in(p, std::ios::binary);
			in.seekg(0, std::ios::end);
			auto n = in.tellg();
			in.seekg(0);
			if (n < 0 || std::uint64_t(n) > wikidata_client::MAX_MODEL_BYTES)
				return memo_[q] = {};
			bytes.resize(static_cast<std::size_t>(n));
			in.read(reinterpret_cast<char *>(bytes.data()),
					std::streamsize(bytes.size()));
			if (!in)
				return memo_[q] = {};
		}
		if (!wikidata_client::save_cached(cache_, e->url, bytes))
			return memo_[q] = {};
		std::filesystem::create_directories(cache_);
		std::ofstream out(p, std::ios::binary);
		out.write(reinterpret_cast<const char *>(bytes.data()),
				std::streamsize(bytes.size()));
		if (!out)
			return memo_[q] = {};
	} else {
		std::filesystem::create_directories(cache_);
		std::ofstream out(p, std::ios::binary);
		out.write(reinterpret_cast<const char *>(cached->data()),
				std::streamsize(cached->size()));
	}
	try {
		return memo_[q] = load_model_asset(p, *format);
	} catch (...) {
		return memo_[q] = {};
	}
}
}
