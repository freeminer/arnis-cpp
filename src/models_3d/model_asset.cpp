#include "model_asset.h"
#include "voxelize.h"
#include "wikidata/stl.h"
#include <fstream>
#include <stdexcept>
#include <iterator>
namespace arnis::models_3d
{
ModelAsset load_model_asset_auto(const std::filesystem::path &p)
{
	const auto ext = p.extension().string();
	if (ext != ".glb" && ext != ".GLB" && ext != ".stl" && ext != ".STL")
		throw std::runtime_error("unsupported model asset extension");
	return load_model_asset(p,
			(ext == ".stl" || ext == ".STL") ? ModelFormat::BinarySTL : ModelFormat::GLB);
}

ModelAsset load_model_asset(const std::filesystem::path &p, ModelFormat f)
{
	std::ifstream in(p, std::ios::binary);
	if (!in)
		throw std::runtime_error("model asset open failed");
	ModelAsset a{f, std::vector<std::uint8_t>((std::istreambuf_iterator<char>(in)), {})};
	if (a.bytes.empty())
		throw std::runtime_error("empty model asset");
	if (f == ModelFormat::GLB) {
		auto b = glb_model_bbox(a.bytes);
		a.min = b.first;
		a.max = b.second;
	} else {
		auto b = stl_bbox(parse_binary_stl(a.bytes));
		a.min = b.first;
		a.max = b.second;
	}
	return a;
}
}
