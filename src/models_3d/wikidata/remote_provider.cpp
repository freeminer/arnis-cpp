#include "remote_provider.h"
#include "client.h"
#include "../../../../http.h"
#include "../../world_utils.h"
#include <filesystem>
#include <fstream>
#include <cctype>
namespace arnis::models_3d
{
std::optional<ModelAsset> RemoteModelProvider::fetch(const std::string &q)
{
	std::string key = q;
	const auto first = key.find_first_not_of(" \t\r\n");
	const auto last = key.find_last_not_of(" \t\r\n");
	key = first == std::string::npos ? std::string{}
									 : key.substr(first, last - first + 1);
	if (key.size() >= 1 && key[0] == 'q')
		key[0] = 'Q';
	auto it = memo_.find(key);
	if (it != memo_.end())
		return it->second;
	auto *e = lookup_wikidata(key);
	if (!e)
		return memo_[key] = {};
	auto cached = wikidata_client::load_cached(cache_, e->url);
	// Rust's cache is keyed solely by URL; select GLB/STL from downloaded bytes.
	auto p = cache_ / (wikidata_client::url_hash(e->url) + ".bin");
	std::filesystem::create_directories(cache_);
	if (!cached) {
		std::vector<std::uint8_t> bytes;
		if (fetch_bytes_) {
			auto fetched = fetch_bytes_(e->url, wikidata_client::MAX_MODEL_BYTES);
			if (!fetched || fetched->size() > wikidata_client::MAX_MODEL_BYTES)
				return memo_[key] = {};
			bytes = std::move(*fetched);
		} else {
			// Compatibility fallback for the in-engine mapgen host.
			if (!http_to_file(e->url, p.string()))
				return memo_[key] = {};
			std::ifstream in(p, std::ios::binary);
			in.seekg(0, std::ios::end);
			auto n = in.tellg();
			in.seekg(0);
			if (n < 0 || std::uint64_t(n) > wikidata_client::MAX_MODEL_BYTES)
				return memo_[key] = {};
			bytes.resize(static_cast<std::size_t>(n));
			in.read(reinterpret_cast<char *>(bytes.data()),
					std::streamsize(bytes.size()));
			if (!in)
				return memo_[key] = {};
		}
		if (!wikidata_client::save_cached(cache_, e->url, bytes))
			return memo_[key] = {};
		if (!arnis::world_utils::replace_file_atomically(p, bytes))
			return memo_[key] = {};
	} else {
		// Materialize the validated cache entry through the same atomic path;
		// concurrent prescan workers must never observe a partially rewritten
		// model file.
		if (!arnis::world_utils::replace_file_atomically(p, *cached))
			return memo_[key] = {};
	}
	try {
		return memo_[key] = load_model_asset_auto(p);
	} catch (...) {
		return memo_[key] = {};
	}
}
}
