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
std::shared_ptr<std::mutex> model_download_mutex(const std::filesystem::path &path)
{
	static std::mutex registry_mutex;
	static std::unordered_map<std::string, std::weak_ptr<std::mutex>> registry;
	const auto key = path.lexically_normal().string();
	std::lock_guard<std::mutex> lock(registry_mutex);
	auto &entry = registry[key];
	auto mutex = entry.lock();
	if (!mutex) {
		mutex = std::make_shared<std::mutex>();
		entry = mutex;
	}
	return mutex;
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
	auto download_mutex = model_download_mutex(p);
	std::lock_guard<std::mutex> download_lock(*download_mutex);
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
	if (!cached) {
		std::vector<std::uint8_t> bytes;
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
		if (!arnis::world_utils::replace_file_atomically(p, bytes))
			return remember({});
	} else {
		// Materialize the validated cache entry through the same atomic path;
		// concurrent prescan workers must never observe a partially rewritten
		// model file.
		if (!arnis::world_utils::replace_file_atomically(p, *cached))
			return remember({});
	}
	try {
		return remember(load_model_asset_auto(p));
	} catch (...) {
		return remember({});
	}
}
}
