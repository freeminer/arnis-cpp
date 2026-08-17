#pragma once
#include <string>
#include <unordered_map>
#include <optional>
#include <vector>
#include "../model_asset.h"
namespace arnis::models_3d
{
struct WikidataEntry
{
	std::string label, url, license, license_url, artist;
	std::optional<double> height_m;
	// Rust index palette_layers: optional Y-banded palette override used by
	// coloured/voxelized models before falling back to OSM surface colours.
	struct PaletteLayer
	{
		float y_max_frac{1.0f};
		std::vector<std::string> blocks;
		std::optional<std::string> hex;
	};
	std::vector<PaletteLayer> palette_layers;
};
const WikidataEntry *lookup_wikidata(const std::string &qid);
std::vector<WikidataEntry> wikidata_attributions();
std::vector<std::string> wikidata_ids();
std::optional<double> wikidata_height_m(const std::string &qid);
std::optional<WikidataEntry> wikidata_attribution(const std::string &qid);
std::vector<std::string> search_wikidata(const std::string &text);
std::vector<std::string> wikidata_downloadable_ids();
bool wikidata_url_supported(const std::string &qid);
std::optional<std::string> wikidata_url(const std::string &qid);
struct WikidataModel
{
	std::string id;
	WikidataEntry entry;
};
std::vector<WikidataModel> wikidata_models();
std::vector<WikidataModel> wikidata_models_named(const std::string &label);
std::vector<WikidataModel> wikidata_models_up_to(double max_height_m);
std::vector<WikidataModel> wikidata_models_between(
		double min_height_m, double max_height_m);
std::vector<WikidataEntry> wikidata_attributions_named(const std::string &text);
std::optional<ModelFormat> wikidata_model_format(const std::string &qid);
std::vector<WikidataEntry::PaletteLayer> wikidata_palette_layers(const std::string &qid);
}
