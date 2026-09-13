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
	std::ifstream in(p, std::ios::binary);
	if (!in)
		throw std::runtime_error("model asset open failed");
	std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)), {});
	auto format = detect_model_format(bytes);
	if (!format)
		throw std::runtime_error("unrecognised model asset");
	ModelAsset a{*format, std::move(bytes)};
	if (a.format == ModelFormat::GLB) {
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

std::optional<ModelFormat> detect_model_format(const std::vector<std::uint8_t> &bytes)
{
	if (bytes.size() >= 4 && bytes[0] == 'g' && bytes[1] == 'l' && bytes[2] == 'T' &&
			bytes[3] == 'F')
		return ModelFormat::GLB;
	// Binary STL has no magic. Its strict parser is the final validation,
	// matching Rust's non-GLB decoding path.
	return bytes.size() >= 84 ? std::optional<ModelFormat>{ModelFormat::BinarySTL}
							  : std::nullopt;
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
