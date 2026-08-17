#include "wikidata_index.h"
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cctype>
namespace arnis::models_3d
{
static const std::unordered_map<std::string, WikidataEntry> &index_data()
{
	static const auto idx = [] {
		std::unordered_map<std::string, WikidataEntry> m;
		for (const char *file :
				{"wikidata_3d_models.json", "wikidata_3d_models_manual.json"}) {
			std::ifstream f(std::filesystem::path(__FILE__)
									.parent_path()
									.parent_path()
									.parent_path() /
							"assets" / file);
			if (!f)
				continue;
			nlohmann::json j;
			f >> j;
			for (auto &[id, v] : j.value("models", nlohmann::json::object()).items()) {
				WikidataEntry e;
				e.label = v.value("label", "");
				e.url = v.value("url", "");
				e.license = v.value("license", "");
				e.license_url = v.value("license_url", "");
				e.artist = v.value("artist", "");
				if (v.contains("height_m"))
					e.height_m = v["height_m"].get<double>();
				for (const auto &layer :
						v.value("palette_layers", nlohmann::json::array())) {
					WikidataEntry::PaletteLayer p;
					p.y_max_frac = layer.value("y_max_frac", 1.0f);
					p.blocks = layer.value("blocks", std::vector<std::string>{});
					if (layer.contains("hex") && layer["hex"].is_string())
						p.hex = layer["hex"].get<std::string>();
					e.palette_layers.push_back(std::move(p));
				}
				m[id] = std::move(e);
			}
		}
		return m;
	}();
	return idx;
}
const WikidataEntry *lookup_wikidata(const std::string &q)
{
	// One shared lazy index keeps pointer lifetime and the manual-over-auto
	// overlay identical for lookup, discovery, attribution and filtering.
	const auto &idx = index_data();
	auto it = idx.find(q);
	return it == idx.end() ? nullptr : &it->second;
}
std::vector<WikidataEntry> wikidata_attributions()
{
	std::vector<WikidataEntry> out;
	for (const auto &[q, e] : index_data())
		out.push_back(e);
	std::sort(out.begin(), out.end(),
			[](const auto &a, const auto &b) { return a.label < b.label; });
	return out;
}
std::vector<std::string> wikidata_ids()
{
	std::vector<std::string> out;
	for (const auto &[q, e] : index_data())
		out.push_back(q);
	std::sort(out.begin(), out.end());
	return out;
}
std::optional<double> wikidata_height_m(const std::string &q)
{
	if (auto *e = lookup_wikidata(q))
		return e->height_m;
	return std::nullopt;
}
std::optional<WikidataEntry> wikidata_attribution(const std::string &q)
{
	if (auto *e = lookup_wikidata(q))
		return *e;
	return std::nullopt;
}
std::vector<std::string> search_wikidata(const std::string &text)
{
	std::string needle = text;
	std::transform(needle.begin(), needle.end(), needle.begin(),
			[](unsigned char c) { return char(std::tolower(c)); });
	std::vector<std::string> out;
	for (const auto &[id, e] : index_data()) {
		std::string l = e.label, a = e.artist;
		std::transform(l.begin(), l.end(), l.begin(),
				[](unsigned char c) { return char(std::tolower(c)); });
		std::transform(a.begin(), a.end(), a.begin(),
				[](unsigned char c) { return char(std::tolower(c)); });
		if (l.find(needle) != std::string::npos || a.find(needle) != std::string::npos)
			out.push_back(id);
	}
	std::sort(out.begin(), out.end());
	return out;
}
std::vector<std::string> wikidata_downloadable_ids()
{
	std::vector<std::string> out;
	for (const auto &[id, e] : index_data())
		if (wikidata_url_supported(id))
			out.push_back(id);
	std::sort(out.begin(), out.end());
	return out;
}
bool wikidata_url_supported(const std::string &q)
{
	auto *e = lookup_wikidata(q);
	if (!e)
		return false;
	auto u = e->url;
	std::transform(u.begin(), u.end(), u.begin(),
			[](unsigned char c) { return char(std::tolower(c)); });
	return u.ends_with(".glb") || u.ends_with(".stl") ||
		   u.find(".glb?") != std::string::npos || u.find(".stl?") != std::string::npos;
}
std::optional<std::string> wikidata_url(const std::string &q)
{
	if (auto *e = lookup_wikidata(q); e && !e->url.empty())
		return e->url;
	return std::nullopt;
}
std::vector<WikidataModel> wikidata_models()
{
	std::vector<WikidataModel> out;
	for (const auto &[id, e] : index_data())
		if (wikidata_url_supported(id))
			out.push_back({id, e});
	std::sort(out.begin(), out.end(),
			[](const auto &a, const auto &b) { return a.id < b.id; });
	return out;
}
std::vector<WikidataModel> wikidata_models_named(const std::string &label)
{
	std::string n = label;
	std::transform(n.begin(), n.end(), n.begin(),
			[](unsigned char c) { return char(std::tolower(c)); });
	std::vector<WikidataModel> out;
	for (const auto &m : wikidata_models()) {
		std::string l = m.entry.label;
		std::transform(l.begin(), l.end(), l.begin(),
				[](unsigned char c) { return char(std::tolower(c)); });
		if (l.find(n) != std::string::npos)
			out.push_back(m);
	}
	std::sort(out.begin(), out.end(),
			[](const auto &a, const auto &b) { return a.entry.label < b.entry.label; });
	return out;
}
std::vector<WikidataModel> wikidata_models_up_to(double maxh)
{
	std::vector<WikidataModel> out;
	for (const auto &m : wikidata_models())
		if (!m.entry.height_m || *m.entry.height_m <= maxh)
			out.push_back(m);
	return out;
}
std::vector<WikidataModel> wikidata_models_between(
		double min_height_m, double max_height_m)
{
	std::vector<WikidataModel> out;
	for (const auto &m : wikidata_models())
		if (!m.entry.height_m ||
				(*m.entry.height_m >= min_height_m && *m.entry.height_m <= max_height_m))
			out.push_back(m);
	return out;
}
std::vector<WikidataEntry> wikidata_attributions_named(const std::string &text)
{
	std::vector<WikidataEntry> out;
	for (const auto &m : wikidata_models_named(text))
		out.push_back(m.entry);
	std::sort(out.begin(), out.end(),
			[](const auto &a, const auto &b) { return a.label < b.label; });
	return out;
}
std::optional<ModelFormat> wikidata_model_format(const std::string &q)
{
	auto *e = lookup_wikidata(q);
	if (!e)
		return std::nullopt;
	std::string u = e->url;
	std::transform(u.begin(), u.end(), u.begin(),
			[](unsigned char c) { return char(std::tolower(c)); });
	if (u.find(".stl") != std::string::npos)
		return ModelFormat::BinarySTL;
	if (u.find(".glb") != std::string::npos)
		return ModelFormat::GLB;
	return std::nullopt;
}
std::vector<WikidataEntry::PaletteLayer> wikidata_palette_layers(const std::string &q)
{
	if (auto *entry = lookup_wikidata(q))
		return entry->palette_layers;
	return {};
}
}
