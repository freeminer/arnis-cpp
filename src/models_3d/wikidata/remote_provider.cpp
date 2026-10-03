#include "remote_provider.h"
#include "client.h"
#include "../../../../http.h"
#include "../../world_utils.h"
#include <filesystem>
#include <fstream>
#include <cctype>
#include <memory>
#include <mutex>
#include <unordered_map>
namespace arnis::models_3d
{
namespace
{
struct ModelFetchState
{
	std::mutex mutex;
	// Remember the first successful cache file so providers constructed with
	// different cache roots still share a single download for this URL.
	std::filesystem::path cache_file;
};

std::shared_ptr<ModelFetchState> model_fetch_state(const std::string &url)
{
	static std::mutex registry_mutex;
	static std::unordered_map<std::string, std::shared_ptr<ModelFetchState>> registry;
	std::lock_guard<std::mutex> lock(registry_mutex);
	auto &state = registry[url];
	if (!state)
		state = std::make_shared<ModelFetchState>();
	return state;
}
} // namespace

std::optional<ModelAsset> RemoteModelProvider::fetch(const std::string &q)
{
	std::string key = q;
	const auto first = key.find_first_not_of(" \t\r\n");
	const auto last = key.find_last_not_of(" \t\r\n");
	key = first == std::string::npos ? std::string{}
									 : key.substr(first, last - first + 1);
	if (key.size() >= 1 && key[0] == 'q')
		key[0] = 'Q';
	auto remember = [&](std::optional<ModelAsset> result) {
		std::lock_guard<std::mutex> lock(memo_mutex_);
		memo_.insert_or_assign(key, result);
		return result;
	};
	{
		std::lock_guard<std::mutex> lock(memo_mutex_);
		auto it = memo_.find(key);
		if (it != memo_.end())
			return it->second;
	}
	auto *e = lookup_wikidata(key);
	if (!e)
		return remember({});
	const auto p = cache_ / (wikidata_client::url_hash(e->url) + ".bin");
	// Emerge threads may create independent provider instances (and cache roots
	// may be expressed through different relative paths).  Coordinate by URL,
	// not by provider or filename, so one in-flight request serves every caller.
	auto fetch_state = model_fetch_state(e->url);
	std::lock_guard<std::mutex> download_lock(fetch_state->mutex);
	// Another provider instance may have completed this QID while we waited.
	{
		std::lock_guard<std::mutex> lock(memo_mutex_);
		auto it = memo_.find(key);
		if (it != memo_.end())
			return it->second;
	}
	auto cached = wikidata_client::load_cached(cache_, e->url);
	// Rust's cache is keyed solely by URL; select GLB/STL from downloaded bytes.
	std::filesystem::create_directories(cache_);
	std::vector<std::uint8_t> bytes;
	if (cached) {
		bytes = std::move(*cached);
		fetch_state->cache_file = p;
	} else {
		// Reuse an earlier successful cache entry even when this provider has a
		// different root.  This also prevents serialized redownloads on later
		// chunk-generation calls, not just simultaneous requests.
		if (!fetch_state->cache_file.empty())
			if (auto shared = wikidata_client::load_cached(
						fetch_state->cache_file.parent_path(), e->url))
				bytes = std::move(*shared);
		if (!bytes.empty()) {
			if (!wikidata_client::save_cached(cache_, e->url, bytes))
				return remember({});
		} else {
			if (fetch_bytes_) {
				auto fetched = fetch_bytes_(e->url, wikidata_client::MAX_MODEL_BYTES);
				if (!fetched || fetched->size() > wikidata_client::MAX_MODEL_BYTES)
					return remember({});
				bytes = std::move(*fetched);
			} else {
				// Compatibility fallback for the in-engine mapgen host.
				if (!http_to_file(e->url, p.string()))
					return remember({});
				std::ifstream in(p, std::ios::binary);
				in.seekg(0, std::ios::end);
				auto n = in.tellg();
				in.seekg(0);
				if (n < 0 || std::uint64_t(n) > wikidata_client::MAX_MODEL_BYTES)
					return remember({});
				bytes.resize(static_cast<std::size_t>(n));
				in.read(reinterpret_cast<char *>(bytes.data()),
						std::streamsize(bytes.size()));
				if (!in)
					return remember({});
			}
			if (!wikidata_client::save_cached(cache_, e->url, bytes))
				return remember({});
		}
		fetch_state->cache_file = p;
	}
	// Materialize the validated cache entry through the same atomic path;
	// callers can safely parse it while another cache root is being populated.
	if (!arnis::world_utils::replace_file_atomically(p, bytes))
		return remember({});
	try {
		return remember(load_model_asset_auto(p));
	} catch (...) {
		return remember({});
	}
}
}
