#include "wikidata_index.h"
#include "../../assets_root.h"
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cctype>
#include <tuple>
namespace arnis::models_3d
{
static const std::unordered_map<std::string, WikidataEntry> &index_data()
{
	static const auto idx = [] {
		std::unordered_map<std::string, WikidataEntry> m;
		for (const char *file :
				{"wikidata_3d_models.json", "wikidata_3d_models_manual.json"}) {
			std::ifstream f(assets::path(file));
			if (!f)
				continue;
			nlohmann::json j;
			try {
				f >> j;
			} catch (const nlohmann::json::exception &) {
				// An optional/generated index must not disable the manual index
				// or prevent world generation when it is truncated or malformed.
				continue;
			}
			if (!j.is_object())
				continue;
			// Keep the JSON owner alive while items() iterates over it.
			const auto models = j.value("models", nlohmann::json::object());
			for (const auto &[id, v] : models.items()) {
				WikidataEntry e;
				e.label = v.value("label", "");
				e.url = v.value("url", "");
				e.license = v.value("license", "");
				const auto optional_string =
						[&, v = v](const char *key) -> std::optional<std::string> {
					const auto field = v.find(key);
					if (field == v.end() || field->is_null())
						return std::nullopt;
					if (field->is_string())
						return field->get<std::string>();
					return std::nullopt;
				};
				e.license_url = optional_string("license_url");
				e.artist = optional_string("artist");
				if (v.contains("height_m"))
					if (v["height_m"].is_number())
						e.height_m = v["height_m"].get<double>();
				if (v.contains("yaw_deg"))
					if (v["yaw_deg"].is_number())
						e.yaw_deg = v["yaw_deg"].get<double>();
				if (v.contains("palette_layers") && !v["palette_layers"].is_null()) {
					if (v["palette_layers"].is_array()) {
						std::vector<WikidataEntry::PaletteLayer> layers;
						for (const auto &layer : v["palette_layers"]) {
							if (!layer.is_object() || !layer.contains("y_max_frac") ||
									!layer["y_max_frac"].is_number())
								continue;
							WikidataEntry::PaletteLayer p;
							p.y_max_frac = layer["y_max_frac"].get<float>();
							if (layer.contains("blocks") && layer["blocks"].is_array())
								p.blocks =
										layer["blocks"].get<std::vector<std::string>>();
							if (layer.contains("hex") && layer["hex"].is_string())
								p.hex = layer["hex"].get<std::string>();
							layers.push_back(std::move(p));
						}
						e.palette_layers = std::move(layers);
					} else {
						e.palette_layers = std::vector<WikidataEntry::PaletteLayer>{};
					}
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
	// Rust orders by label and then QID, making equal-label attribution lists
	// stable across hash-map iteration order.
	std::vector<std::pair<std::string, WikidataEntry>> entries;
	entries.reserve(index_data().size());
	for (const auto &[q, e] : index_data())
		entries.emplace_back(q, e);
	std::sort(entries.begin(), entries.end(), [](const auto &a, const auto &b) {
		return std::tie(a.second.label, a.first) < std::tie(b.second.label, b.first);
	});
	std::vector<WikidataEntry> out;
	out.reserve(entries.size());
	for (auto &entry : entries)
		out.push_back(std::move(entry.second));
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
		std::string l = e.label, a = e.artist.value_or("");
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
	// A P4896/Commons URL can be a redirect or have no useful suffix. Rust
	// detects GLB by magic after download and validates everything else as STL.
	return e->url.starts_with("https://");
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
	std::sort(out.begin(), out.end(), [](const auto &a, const auto &b) {
		return std::tie(a.entry.label, a.id) < std::tie(b.entry.label, b.id);
	});
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
	const auto models = wikidata_models_named(text);
	for (const auto &m : models)
		out.push_back(m.entry);
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
		return entry->palette_layers.value_or(std::vector<WikidataEntry::PaletteLayer>{});
	return {};
}
}
