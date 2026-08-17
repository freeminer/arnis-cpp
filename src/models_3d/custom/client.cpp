#include "client.h"
#include "archetypes.h"
#include "../model_asset.h"
#include <algorithm>
#include <fstream>
namespace arnis::models_3d::custom
{
std::optional<ModelAsset> Client::fetch(const std::string &key)
{
	if (key.empty())
		return std::nullopt;
	if (key.front() == ' ' || key.back() == ' ')
		return std::nullopt;
	if (key.find("..") != std::string::npos || key.find('/') != std::string::npos ||
			key.find('\\') != std::string::npos)
		return std::nullopt;
	if (root_.empty())
		return std::nullopt;
	std::error_code ec;
	std::filesystem::create_directories(root_, ec);
	std::string base = key;
	std::replace(base.begin(), base.end(), ':', '_');
	if (base.size() > 4 && (base.ends_with(".glb") || base.ends_with(".stl") ||
								   base.ends_with(".GLB") || base.ends_with(".STL")))
		base.resize(base.size() - 4);
	for (const auto &ext : {".glb", ".stl"}) {
		auto p = root_ / (base + ext);
		if (std::filesystem::exists(p))
			try {
				auto a = load_model_asset_auto(p);
				if (a.max[0] > a.min[0] && a.max[1] > a.min[1] && a.max[2] > a.min[2])
					return a;
			} catch (...) {
			}
	}
	if (!fetcher_ || (base != "plane" && base != "stadium"))
		return std::nullopt;
	const std::string url = base == "plane" ? PLANE_MODEL_URL : STADIUM_MODEL_URL;
	constexpr std::size_t max_glb_bytes = 16 * 1024 * 1024;
	auto bytes = fetcher_(url, max_glb_bytes);
	if (!bytes || bytes->empty() || bytes->size() > max_glb_bytes)
		return std::nullopt;
	const auto cached = root_ / (base + ".glb");
	{
		std::ofstream out(cached, std::ios::binary | std::ios::trunc);
		if (!out)
			return std::nullopt;
		out.write(reinterpret_cast<const char *>(bytes->data()),
				static_cast<std::streamsize>(bytes->size()));
		if (!out)
			return std::nullopt;
	}
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
