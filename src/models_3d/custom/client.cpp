#include "client.h"
#include "archetypes.h"
#include "../model_asset.h"
#include "../../world_utils.h"
#include "../../../../http.h"
#include <fstream>
#include <thread>
#include <cstdlib>
#include <algorithm>
namespace arnis::models_3d::custom
{
static std::filesystem::path cache_root(const std::filesystem::path &configured)
{
	if (!configured.empty())
		return configured;
	if (const char *xdg = std::getenv("XDG_CACHE_HOME"); xdg && *xdg)
		return std::filesystem::path(xdg) / "arnis" / "custom_models";
	if (const char *home = std::getenv("HOME"); home && *home)
		return std::filesystem::path(home) / ".cache" / "arnis" / "custom_models";
	return std::filesystem::path("./.arnis_custom_cache");
}

std::optional<ModelAsset> Client::fetch(const std::string &key)
{
	if (key.empty())
		return std::nullopt;
	if (key.front() == ' ' || key.back() == ' ')
		return std::nullopt;
	if (key.find("..") != std::string::npos || key.find('/') != std::string::npos ||
			key.find('\\') != std::string::npos)
		return std::nullopt;
	// Match Rust's cache_root() fallback for library callers that construct the
	// provider without an application-specific cache directory.  An empty
	// root must not disable remote archetypes: the Rust client still downloads
	// into ./.arnis_custom_cache when the platform cache directory is absent.
	const std::filesystem::path root = cache_root(root_);
	constexpr std::size_t max_glb_bytes = 16 * 1024 * 1024;
	std::error_code ec;
	std::filesystem::create_directories(root, ec);
	std::string base = key;
	std::replace(base.begin(), base.end(), ':', '_');
	if (base.size() > 4 && (base.ends_with(".glb") || base.ends_with(".stl") ||
								   base.ends_with(".GLB") || base.ends_with(".STL")))
		base.resize(base.size() - 4);
	for (const auto &ext : {".glb", ".stl"}) {
		auto p = root / (base + ext);
		if (std::filesystem::exists(p)) {
			// Rust's fetch_glb applies the same hard cap to cache hits and
			// network responses.  Reject oversized stale files before parsing.
			ec.clear();
			const auto size = std::filesystem::file_size(p, ec);
			if (ec || size > max_glb_bytes)
				continue;
			try {
				auto a = load_model_asset_auto(p);
				if (a.max[0] > a.min[0] && a.max[1] > a.min[1] && a.max[2] > a.min[2])
					return a;
			} catch (...) {
			}
		}
	}
	if (base != "plane" && base != "stadium")
		return std::nullopt;
	const std::string url = base == "plane" ? PLANE_MODEL_URL : STADIUM_MODEL_URL;
	std::optional<std::vector<std::uint8_t>> bytes;
	if (fetcher_) {
		bytes = fetcher_(url, max_glb_bytes);
	} else {
		const auto download = (root / (base + "." +
											  std::to_string(std::hash<std::thread::id>{}(
													  std::this_thread::get_id())) +
											  ".download"))
									  .string();
		if (http_to_file(url, download)) {
			std::ifstream input(download, std::ios::binary);
			input.seekg(0, std::ios::end);
			const auto size = input.tellg();
			if (size >= 0 && static_cast<std::uint64_t>(size) <= max_glb_bytes) {
				input.seekg(0);
				std::vector<std::uint8_t> data(static_cast<std::size_t>(size));
				input.read(reinterpret_cast<char *>(data.data()),
						std::streamsize(data.size()));
				if (input)
					bytes = std::move(data);
			}
		}
		std::error_code remove_error;
		std::filesystem::remove(download, remove_error);
	}
	if (!bytes || bytes->empty() || bytes->size() > max_glb_bytes)
		return std::nullopt;
	const auto cached = root / (base + ".glb");
	if (!arnis::world_utils::replace_file_atomically(cached, *bytes))
		return std::nullopt;
	try {
		auto asset = load_model_asset_auto(cached);
		if (asset.max[0] > asset.min[0] && asset.max[1] > asset.min[1] &&
				asset.max[2] > asset.min[2])
			return asset;
	} catch (...) {
	}
	return std::nullopt;
}
}
