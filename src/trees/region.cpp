#include "region.h"
#include "schematic.h"
#include "tree_pack.h"
#include "../land_cover/land_cover.h"
#include "mapped.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <utility>
#include <array>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <tuple>
namespace arnis::trees
{
bool is_palm(const std::string &s)
{
	// Rust classifies by the genus prefix, not by an arbitrary substring.  This
	// keeps names such as "palmetto_oak" from being stripped while covering the
	// complete palm list used by the realm packs.
	const auto end = s.find('_');
	std::string genus = s.substr(0, end == std::string::npos ? s.size() : end);
	for (char &c : genus)
		c = char(std::tolower(static_cast<unsigned char>(c)));
	static constexpr const char *genera[] = {"acrocomia", "archontophoenix", "areca",
			"astrocarym", "attalea", "beccariophoenix", "bismarckia", "borassus",
			"calyptronoma", "ceroxylon", "cocos", "cyrtostachys", "elaeis", "euterpe",
			"hyphaene", "jubaea", "livistona", "mauritia", "nypa", "phoenix", "raphia",
			"rhopalostylis", "roystonea", "sabal", "serenoa", "socratea", "washingtonia"};
	for (const auto *p : genera)
		if (genus == p)
			return true;
	return genus == "palm" || genus == "palms";
}
bool subtropical_latitude(double lat)
{
	return std::abs(lat) <= 35.0;
}
Habitat habitat_for_land_cover(std::uint8_t lc)
{
	using namespace land_cover;
	if (lc == LC_WATER || lc == LC_WETLAND || lc == LC_MANGROVES)
		return Habitat::Wet;
	if (lc == LC_BARE)
		return Habitat::Dry;
	if (lc == LC_SNOW_ICE || lc == LC_MOSS)
		return Habitat::Conifer;
	if (lc == LC_SHRUBLAND)
		return Habitat::Dry;
	return Habitat::Lowland;
}
const std::vector<std::string> &width_candidates(const Species &s, unsigned w)
{
	return w <= 1 ? s.w1 : w == 2 ? s.w2 : s.w3;
}
const Species *choose_species(
		const Community &c, unsigned width, std::uint64_t seed, bool subtropical)
{
	std::vector<const Species *> choices;
	for (const auto &s : c.species)
		if (subtropical || !is_palm(s.name))
			if (!width_candidates(s, width).empty())
				choices.push_back(&s);
	if (choices.empty())
		return nullptr;
	return choices[seed % choices.size()];
}
std::string choose_schematic(
		const Community &c, unsigned width, std::uint64_t seed, bool subtropical)
{
	const Species *s = choose_species(c, width, seed, subtropical);
	unsigned selected_width = width;
	for (unsigned f = width; !s && f > 1; --f) {
		s = choose_species(c, f - 1, seed + f, subtropical);
		if (s)
			selected_width = f - 1;
	}
	if (!s)
		return {};
	const auto &v = width_candidates(*s, selected_width);
	if (v.empty())
		return {};
	return v[seed % v.size()];
}
std::vector<Community> load_communities(const std::filesystem::path &path)
{
	std::ifstream in(path);
	if (!in)
		return {};
	nlohmann::json root;
	in >> root;
	std::vector<Community> out;
	const auto &items = root.contains("communities") ? root["communities"] : root;
	for (const auto &j : items) {
		Community c;
		c.name = j.value("name", "");
		c.habitat = habitat_from_string(j.value("habitat", "lowland"));
		c.density = j.value("density", 20u);
		for (const auto &s : j.value("species", nlohmann::json::array())) {
			Species sp;
			sp.name = s.value("name", "");
			if (s.contains("w1"))
				sp.w1 = s["w1"].get<std::vector<std::string>>();
			if (s.contains("w2"))
				sp.w2 = s["w2"].get<std::vector<std::string>>();
			if (s.contains("w3"))
				sp.w3 = s["w3"].get<std::vector<std::string>>();
			c.species.push_back(std::move(sp));
		}
		out.push_back(std::move(c));
	}
	return out;
}
RegionLibrary RegionLibrary::load(const std::filesystem::path &p)
{
	RegionLibrary r;
	r.communities_ = load_communities(p);
	return r;
}
RegionLibrary RegionLibrary::combine(
		const std::filesystem::path &a, const std::filesystem::path &b)
{
	RegionLibrary r;
	r.communities_ = load_communities(a);
	auto q = load_communities(b);
	r.communities_.insert(r.communities_.end(), q.begin(), q.end());
	return r;
}
std::string RegionLibrary::choose(
		Habitat h, unsigned width, std::uint64_t seed, bool subtropical) const
{
	width = std::clamp(width, 1u, 3u);
	std::vector<const Community *> matches;
	for (const auto &c : communities_)
		if (c.habitat == h)
			matches.push_back(&c);
	if (matches.empty())
		for (const auto &c : communities_)
			matches.push_back(&c);
	if (matches.empty())
		return {};
	std::uint64_t total = 0;
	for (const auto *c : matches)
		total += std::max(1u, c->density);
	std::uint64_t pick = seed % total;
	for (const auto *c : matches) {
		const auto w = std::max(1u, c->density);
		if (pick < w)
			return choose_schematic(*c, width, seed / 7 + 1, subtropical);
		pick -= w;
	}
	return choose_schematic(*matches.back(), width, seed / 7 + 1, subtropical);
}
bool RegionLibrary::accepts(std::uint64_t seed) const
{
	if (communities_.empty())
		return false;
	unsigned density = 0;
	for (const auto &c : communities_)
		density = std::max(density, c.density);
	return (seed % 100u) < std::min(100u, density);
}
bool RegionLibrary::place(world_editor::WorldEditor &e, const TreePackSource &source,
		Habitat h, unsigned width, std::uint64_t seed, bool subtropical, int x, int y,
		int z, unsigned rotation) const
{
	const auto name = choose(h, width, seed, subtropical);
	if (name.empty())
		return false;
	const auto path = resolve_tree_asset(source, name);
	if (path.empty())
		return false;
	auto schem = load_schem(path);
	return place_schematic_rooted(e, schem, x, y, z, rotation);
}

namespace
{
int habitat_index(Habitat h)
{
	return static_cast<int>(h);
}
std::vector<std::pair<const char *, unsigned>> vanilla_sprinkle(ecoregion::EcoBiome biome)
{
	using ecoregion::EcoBiome;
	switch (biome) {
	case EcoBiome::MoistTropical:
		return {{"VN+ Jungle Tall", 3}, {"VN+ Jungle Sparse", 2}};
	case EcoBiome::DryTropical:
		return {{"VN+ Jungle Sparse", 2}, {"VN+ Acacia", 2}};
	case EcoBiome::TropicalConifer:
		return {{"VN+ Pine", 2}, {"VN+ Oaks", 1}, {"VN+ Jungle Sparse", 1}};
	case EcoBiome::TemperateBroadleaf:
		return {{"VN+ Oaks", 4}, {"VN+ Birches", 2}, {"VN+ Dark Oaks", 1},
				{"VN+ Old Growth Birches", 1}, {"VN+ Swamp Oaks", 1}};
	case EcoBiome::TemperateConifer:
		return {{"VN+ Spruce", 2}, {"VN+ Pine", 2}, {"VN+ Old Growth Spruces", 1},
				{"VN+ Old Growth Pines", 1}, {"VN+ Oaks", 1}};
	case EcoBiome::Boreal:
		return {{"VN+ Spruce", 3}, {"VN+ Pine", 2}, {"VN+ Birches", 2},
				{"VN+ Old Growth Spruces", 1}};
	case EcoBiome::TropicalGrassland:
		return {{"VN+ Acacia", 4}, {"VN+ Jungle Sparse", 1}};
	case EcoBiome::TemperateGrassland:
		return {{"VN+ Oaks", 3}, {"VN+ Birches", 1}, {"VN+ Swamp Oaks", 1}};
	case EcoBiome::Flooded:
		return {{"VN+ Swamp Oaks", 2}, {"VN+ Oaks", 1}};
	case EcoBiome::MontaneGrassland:
		return {{"VN+ Spruce", 1}, {"VN+ Pine", 1}, {"VN+ Birches", 1}, {"VN+ Oaks", 1}};
	case EcoBiome::Tundra:
		return {{"VN+ Spruce", 2}, {"VN+ Birches", 2}};
	case EcoBiome::Mediterranean:
		return {{"VN+ Oaks", 3}, {"VN+ Pine", 2}, {"VN+ Acacia", 1}};
	case EcoBiome::Desert:
		return {{"VN+ Acacia", 3}, {"VN+ Oaks", 1}};
	case EcoBiome::Mangroves:
		return {{"VN+ Swamp Oaks", 1}, {"VN+ Jungle Sparse", 1}};
	}
	return {};
}
double smooth_noise(int x, int z, int scale)
{
	const int s = std::max(1, scale);
	auto div = [s](int v) { return v >= 0 ? v / s : -(((-v) + s - 1) / s); };
	const int x0 = div(x) * s, z0 = div(z) * s;
	const double tx = double(x - x0) / s, tz = double(z - z0) / s;
	auto smooth = [](double v) { return v * v * (3 - 2 * v); };
	auto sample = [](int a, int b) {
		return double(land_cover::coord_hash(a, b) % 1000) / 1000.;
	};
	const double a = sample(x0, z0) * (1 - smooth(tx)) + sample(x0 + s, z0) * smooth(tx);
	const double b =
			sample(x0, z0 + s) * (1 - smooth(tx)) + sample(x0 + s, z0 + s) * smooth(tx);
	return a * (1 - smooth(tz)) + b * smooth(tz);
}
}

struct RegionSelector::Data
{
	enum class Niche : std::uint8_t
	{
		Any,
		Montane,
		Wet
	};
	struct EcoChoice
	{
		std::size_t community = 0;
		unsigned weight = 1;
		Niche niche = Niche::Any;
	};
	struct Entry
	{
		Schematic schem;
		TreeSize size;
		std::uint8_t width;
	};
	struct Community
	{
		std::string name;
		std::string pack;
		Habitat habitat = Habitat::Lowland;
		unsigned density = 20;
		std::vector<std::vector<std::size_t>> species;
		std::vector<std::string> species_names;
		std::vector<bool> species_conifer;
		std::vector<bool> species_palm;
	};
	struct Pack
	{
		std::vector<Community> communities;
		std::array<std::vector<std::size_t>, 5> by_habitat;
		std::size_t fallback = 0;
		bool empty() const { return communities.empty(); }
	};
	std::vector<Entry> entries;
	Pack realm, vanilla;
	std::unordered_map<std::uint16_t, std::vector<EcoChoice>> eco_choices;
	std::unordered_map<std::uint16_t, std::vector<std::size_t>> eco_communities;
	double scale = 1.;
	double blocks_per_meter = 0.;
	int ground_level = 0;
	double latitude = 0.0;
	bool latitude_known = false;
	bool palms_default = true;
	SizeFilter sizes{};
	bool allowed(TreeSize s) const
	{
		return sizes.allows(s) && (s != TreeSize::Giant || scale >= 1.0);
	}
	TreeSize roll_size(int x, int z) const
	{
		const auto r = land_cover::coord_hash(x + 101, z + 233) % 1000;
		if (scale < .3)
			return r < 650 ? TreeSize::Small : r < 985 ? TreeSize::Medium : TreeSize::Big;
		if (scale < .7)
			return r < 380	 ? TreeSize::Small
				   : r < 820 ? TreeSize::Medium
				   : r < 985 ? TreeSize::Big
							 : TreeSize::Tall;
		if (scale < 1.)
			return r < 260	 ? TreeSize::Small
				   : r < 700 ? TreeSize::Medium
				   : r < 930 ? TreeSize::Big
							 : TreeSize::Tall;
		return r < 200	 ? TreeSize::Small
			   : r < 600 ? TreeSize::Medium
			   : r < 880 ? TreeSize::Big
			   : r < 975 ? TreeSize::Tall
						 : TreeSize::Giant;
	}
};

std::optional<RegionSelector> RegionSelector::load(const TreePackSource &source,
		double scale, int ground_level, const SizeFilter &sizes, bool exclude_palms,
		double blocks_per_meter)
{
	auto data = std::make_shared<Data>();
	data->scale = scale;
	data->blocks_per_meter = blocks_per_meter;
	data->ground_level = ground_level;
	data->palms_default = !exclude_palms;
	data->sizes = sizes;
	auto load_pack = [&](const std::filesystem::path &manifest, Data::Pack &out,
							 const std::string &pack_name) {
		std::ifstream in(manifest);
		if (!in)
			return;
		nlohmann::json root;
		try {
			in >> root;
		} catch (...) {
			return;
		}
		const auto communities = root.value("communities", nlohmann::json::array());
		const std::string fallback = root.value("default_community", "");
		bool explicit_fallback = false;
		for (const auto &json : communities) {
			Data::Community community;
			community.name = json.value("name", "");
			community.pack = pack_name;
			community.habitat = habitat_from_string(json.value("habitat", "lowland"));
			community.density = json.value("density", 20u);
			std::vector<std::vector<std::size_t>> species;
			for (const auto &sp : json.value("species", nlohmann::json::array())) {
				const auto name = sp.value("name", "");
				if (exclude_palms && is_palm(name))
					continue;
				std::vector<std::size_t> variants;
				for (const auto &[field, width] :
						std::array<std::pair<const char *, unsigned>, 3>{
								{{"w1", 1}, {"w2", 2}, {"w3", 3}}})
					if (sp.contains(field))
						for (const auto &relative : sp[field]) {
							auto path =
									manifest.parent_path() / relative.get<std::string>();
							try {
								auto schem = load_schem(path);
								if (has_leaves(schem)) {
									const auto size = schematic_size(schem);
									data->entries.push_back({std::move(schem), size,
											std::uint8_t(width)});
									variants.push_back(data->entries.size() - 1);
								}
							} catch (...) {
							}
						}
				if (!variants.empty()) {
					const auto sep = name.find('_');
					const auto genus =
							name.substr(0, sep == std::string::npos ? name.size() : sep);
					species.push_back(std::move(variants));
					community.species_names.push_back(name);
					community.species_conifer.push_back(mapped::is_conifer_genus(genus));
					community.species_palm.push_back(is_palm(name));
				}
			}
			if (species.empty())
				continue;
			community.species = std::move(species);
			const auto idx = out.communities.size();
			if (!fallback.empty() && json.value("name", "") == fallback) {
				out.fallback = idx;
				explicit_fallback = true;
			}
			out.by_habitat[habitat_index(community.habitat)].push_back(idx);
			out.communities.push_back(std::move(community));
		}
		if (!explicit_fallback)
			for (std::size_t i = 0; i < out.communities.size(); ++i)
				if (out.communities[i].habitat == Habitat::Lowland) {
					out.fallback = i;
					break;
				}
	};
	load_pack(source.realm_path("region.json"), data->realm, source.realm());
	if (data->realm.empty())
		return std::nullopt;
	// The directory name is `vanilla-plus`, while its manifest identifies the
	// realm as `vnplus`.  Rust checks the manifest value; accept both spellings
	// so callers constructing a source directly do not load the vanilla pack
	// twice.
	if (source.realm() != "vanilla-plus" && source.realm() != "vnplus")
		load_pack(source.vanilla_path("region.json"), data->vanilla, "vanilla-plus");
	// Resolve mixes against the active realm and vanilla-plus pack.  Other realm
	// packs are loaded on a separate location-specific selector rather than
	// eagerly decoding every global pack (which stalls all emerge workers and
	// consumes several gigabytes before the first chunk is ready).
	// Match Rust's realm_pack.own: derived communities must never become
	// sources for later mixes, or repeated exclusions multiply their copies.
	const auto community_count = data->realm.communities.size();
	for (std::uint16_t id = 0; id < 900; ++id) {
		const auto mix = ecoregion::tree_mix(id);
		if (!mix)
			continue;
		std::size_t begin = 0;
		while (begin <= mix->second.size()) {
			const auto end = mix->second.find(';', begin);
			const auto raw = mix->second.substr(
					begin, end == std::string_view::npos ? mix->second.size() - begin
														 : end - begin);
			if (!raw.empty()) {
				auto body = raw;
				Data::Niche niche = Data::Niche::Any;
				if (body.front() == '^') {
					niche = Data::Niche::Montane;
					body.remove_prefix(1);
				} else if (body.front() == '~') {
					niche = Data::Niche::Wet;
					body.remove_prefix(1);
				}
				const auto star = body.rfind('*');
				const auto colon = body.find(':');
				if (star != std::string_view::npos && colon != std::string_view::npos &&
						colon < star) {
					unsigned weight = 0;
					try {
						weight = static_cast<unsigned>(
								std::stoul(std::string(body.substr(star + 1))));
					} catch (...) {
					}
					if (weight) {
						const auto pack_name = body.substr(0, colon);
						const auto stop = body.find('!', colon + 1);
						const auto name = body.substr(colon + 1,
								(stop == std::string_view::npos ? star : stop) - colon -
										1);
						// Rust allows a mix item to exclude one or more species or
						// genera (`pack:community!Oak!Pinus*weight`).  Materialize
						// that filtered community once while resolving the mix, so
						// subsequent slot picks retain the same weighted selection.
						std::vector<std::string_view> excluded;
						if (stop != std::string_view::npos) {
							auto p = stop + 1;
							while (p < star) {
								const auto next = body.find('!', p);
								excluded.push_back(body.substr(
										p, (next == std::string_view::npos || next > star
														   ? star
														   : next) -
												   p));
								if (next == std::string_view::npos || next >= star)
									break;
								p = next + 1;
							}
						}
						for (std::size_t ci = 0; ci < community_count; ++ci)
							if (data->realm.communities[ci].pack == pack_name &&
									data->realm.communities[ci].name == name) {
								std::size_t selected = ci;
								if (!excluded.empty()) {
									Data::Community filtered =
											data->realm.communities[ci];
									filtered.species.clear();
									filtered.species_names.clear();
									filtered.species_conifer.clear();
									filtered.species_palm.clear();
									for (std::size_t si = 0;
											si <
											data->realm.communities[ci].species.size();
											++si) {
										const auto &species = data->realm.communities[ci]
																	  .species_names[si];
										const auto sep = species.find('_');
										const auto genus = species.substr(0,
												sep == std::string::npos ? species.size()
																		 : sep);
										bool omit = false;
										for (const auto ex : excluded)
											if (species == ex || genus == ex) {
												omit = true;
												break;
											}
										if (!omit) {
											filtered.species.push_back(
													data->realm.communities[ci]
															.species[si]);
											filtered.species_names.push_back(species);
											filtered.species_conifer.push_back(
													data->realm.communities[ci]
															.species_conifer[si]);
											filtered.species_palm.push_back(
													data->realm.communities[ci]
															.species_palm[si]);
										}
									}
									if (!filtered.species.empty()) {
										selected = data->realm.communities.size();
										data->realm.communities.push_back(
												std::move(filtered));
									}
								}
								if (selected != ci || excluded.empty())
									data->eco_choices[id].push_back(
											{selected, weight, niche});
							}
					}
				}
			}
			if (end == std::string_view::npos)
				break;
			begin = end + 1;
		}
		if (auto it = data->eco_choices.find(id); it != data->eco_choices.end()) {
			auto &communities = data->eco_communities[id];
			std::unordered_set<std::size_t> known(communities.begin(), communities.end());
			for (const auto &choice : it->second)
				if (known.insert(choice.community).second)
					communities.push_back(choice.community);
		}
	}
	return RegionSelector{std::move(data)};
}
std::optional<RegionSelector> RegionSelector::load_for_location(double latitude,
		double longitude, const std::filesystem::path &root, double scale,
		int ground_level, const SizeFilter &sizes, double blocks_per_meter,
		const std::optional<std::string> &preferred_realm)
{
	// Rust always reads the compiled tree-pack bundle.  Preserve the C++
	// override when supplied, but resolve the bundled asset root for the common
	// empty-root call path as well.
	TreePackSource source = TreePackSource::embedded(
			preferred_realm.value_or(realm_for_latlon(latitude, longitude)), root);
	// Keep palms in the index: Rust gates them per ecoregion, and a
	// latitude-only load filter would discard Mediterranean palms before that
	// context is available.
	auto result = load(source, scale, ground_level, sizes, false, blocks_per_meter);
	if (result) {
		result->data_->latitude = latitude;
		result->data_->latitude_known = true;
		result->data_->palms_default = subtropical_latitude(latitude);
	}
	return result;
}

bool RegionSelector::empty() const
{
	return !data_ || data_->entries.empty() || data_->realm.empty();
}
std::size_t RegionSelector::entry_count() const
{
	return data_ ? data_->entries.size() : 0;
}
int RegionSelector::base_spacing() const
{
	return !data_ ? 5 : data_->scale < .3 ? 7 : data_->scale < .7 ? 6 : 5;
}
const Schematic *RegionSelector::schematic(std::size_t index) const
{
	return data_ && index < data_->entries.size() ? &data_->entries[index].schem
												  : nullptr;
}

std::optional<SlotSelection> RegionSelector::pick_slot(
		int x, int z, Habitat hint, int elevation, SlotRequest request) const
{
	if (empty())
		return std::nullopt;
	// An explicit ecoregion is authoritative for untagged slots, matching the
	// Rust selector's EcoMix request.  Tag-derived/wetland hints remain stronger
	// and are refined below by the montane and wet-ground rules.
	if (request.eco && !request.tagged && !request.wet_ground) {
		switch (request.eco->biome) {
		case ecoregion::EcoBiome::MoistTropical:
		case ecoregion::EcoBiome::DryTropical:
		case ecoregion::EcoBiome::TropicalConifer:
		case ecoregion::EcoBiome::TropicalGrassland:
			hint = Habitat::Tropical;
			break;
		case ecoregion::EcoBiome::TemperateConifer:
		case ecoregion::EcoBiome::Boreal:
		case ecoregion::EcoBiome::MontaneGrassland:
		case ecoregion::EcoBiome::Tundra:
			hint = Habitat::Conifer;
			break;
		case ecoregion::EcoBiome::Flooded:
		case ecoregion::EcoBiome::Mangroves:
			hint = Habitat::Wet;
			break;
		case ecoregion::EcoBiome::Desert:
		case ecoregion::EcoBiome::Mediterranean:
			hint = Habitat::Dry;
			break;
		default:
			hint = Habitat::Lowland;
			break;
		}
	}
	const int spacing = base_spacing();
	auto [sx, sz] = trunk_slot_s(x, z, spacing);
	// Invert the vertical affine, including terrain compression, as in region.rs.
	const double per_metre =
			data_->blocks_per_meter > 0.0 ? data_->blocks_per_meter : data_->scale;
	const bool montane =
			((double(elevation) - data_->ground_level) / std::max(.001, per_metre) >
					450.) &&
			smooth_noise(sx, sz, 64) < .6;
	if (request.wet_ground)
		hint = Habitat::Wet;
	const bool palms_allowed =
			request.eco ? (data_->latitude_known ? ecoregion::palms_belong(
														   *request.eco, data_->latitude)
												 : data_->palms_default)
						: data_->palms_default;
	if (montane && (hint == Habitat::Lowland || hint == Habitat::Wet))
		hint = Habitat::Conifer;
	const auto blend = land_cover::coord_hash(sx + 7, sz + 13) % 100;
	const Data::Pack *pack = &data_->realm;
	if (blend >= 67 && blend < 97 && !data_->vanilla.empty())
		pack = &data_->vanilla;
	const std::vector<std::size_t> *candidates = &pack->by_habitat[habitat_index(hint)];
	if (candidates->empty())
		candidates = &pack->by_habitat[habitat_index(Habitat::Lowland)];
	std::size_t ci = candidates->empty()
							 ? std::min(pack->fallback, pack->communities.size() - 1)
							 : (*candidates)[std::min<std::size_t>(candidates->size() - 1,
									   std::size_t(smooth_noise(sx, sz, 160) *
												   candidates->size()))];
	if (blend >= 97)
		ci = land_cover::coord_hash(sx + 5, sz + 9) % pack->communities.size();
	if (request.eco && pack == &data_->vanilla) {
		const auto sprinkle = vanilla_sprinkle(request.eco->biome);
		unsigned total = 0;
		for (const auto &[name, weight] : sprinkle)
			total += weight;
		if (total) {
			unsigned pick = land_cover::coord_hash(sx ^ 0x4e31, sz ^ 0x8a17) % total;
			for (const auto &[name, weight] : sprinkle) {
				if (pick < weight) {
					for (std::size_t i = 0; i < pack->communities.size(); ++i)
						if (pack->communities[i].name == name) {
							ci = i;
							break;
						}
					break;
				}
				pick -= weight;
			}
		}
	}
	if (request.eco && pack == &data_->realm) {
		if (const auto it = data_->eco_choices.find(request.eco->id);
				it != data_->eco_choices.end() && !it->second.empty()) {
			const bool wet = request.wet_ground ||
							 request.eco->biome == ecoregion::EcoBiome::Flooded ||
							 request.eco->biome == ecoregion::EcoBiome::Mangroves;
			const bool montane =
					elevation >
					data_->ground_level +
							450 * std::max(1.0, data_->blocks_per_meter > 0.0
														? data_->blocks_per_meter
														: data_->scale);
			std::vector<const Data::EcoChoice *> preferred, fallback;
			for (const auto &choice : it->second) {
				fallback.push_back(&choice);
				if ((wet && choice.niche == Data::Niche::Wet) ||
						(montane && choice.niche == Data::Niche::Montane) ||
						(!wet && !montane && choice.niche == Data::Niche::Any))
					preferred.push_back(&choice);
			}
			const auto &pool = preferred.empty() ? fallback : preferred;
			unsigned total = 0;
			for (const auto *choice : pool)
				total += choice->weight;
			if (total) {
				unsigned roll = land_cover::coord_hash(sx ^ 0x31a7, sz ^ 0x9c41) % total;
				for (const auto *choice : pool) {
					if (roll < choice->weight) {
						ci = choice->community;
						break;
					}
					roll -= choice->weight;
				}
			}
		}
	}
	const auto &community = pack->communities[ci];
	if (!request.density_decided) {
		const double grove = smooth_noise(sx, sz, 22),
					 jitter = double(land_cover::coord_hash(sx ^ 0x71c3, sz ^ 0x2d9b) %
									  1000) /
							  1000.,
					 keep = std::clamp(.34 + community.density / 90., .30, 1.);
		if (grove * .82 + jitter * .18 >= keep)
			return std::nullopt;
	}
	std::vector<std::size_t> all;
	std::vector<std::size_t> typed;
	std::vector<std::size_t> mapped;
	const bool want_conifer =
			request.conifer.value_or(request.tagged && hint == Habitat::Conifer);
	const bool want_broadleaf = request.conifer && !*request.conifer
										? true
										: request.tagged && hint == Habitat::Lowland;
	const bool want_beach_palm = request.beach && request.eco && !request.wet_ground;
	if (want_beach_palm && palms_allowed) {
		const auto mix = data_->eco_communities.find(request.eco->id);
		if (mix != data_->eco_communities.end() &&
				land_cover::coord_hash(sx ^ 0x0BEA, sz ^ 0x0C11) % 100 < 70) {
			for (const auto ci_mix : mix->second) {
				if (ci_mix >= data_->realm.communities.size())
					continue;
				const auto &candidate = data_->realm.communities[ci_mix];
				for (std::size_t si = 0; si < candidate.species.size(); ++si) {
					if (si >= candidate.species_palm.size() ||
							!candidate.species_palm[si])
						continue;
					for (const auto i : candidate.species[si])
						if (data_->allowed(data_->entries[i].size))
							mapped.push_back(i);
				}
			}
		}
	}
	for (std::size_t si = 0; si < community.species.size(); ++si)
		for (auto i : community.species[si])
			if (data_->allowed(data_->entries[i].size) &&
					(palms_allowed || si >= community.species_palm.size() ||
							!community.species_palm[si])) {
				if (request.genus && si < community.species_names.size()) {
					const auto &species = community.species_names[si];
					const auto sep = species.find('_');
					const auto species_genus = species.substr(
							0, sep == std::string::npos ? species.size() : sep);
					if (species_genus == *request.genus)
						mapped.push_back(i);
				}
				if (want_beach_palm && si < community.species_palm.size() &&
						community.species_palm[si])
					typed.push_back(i);
				else if ((want_conifer && si < community.species_conifer.size() &&
								 community.species_conifer[si]) ||
						 (want_broadleaf && si < community.species_conifer.size() &&
								 !community.species_conifer[si]))
					typed.push_back(i);
				all.push_back(i);
			}
	// Rust's mapped path searches every community in the active mix, then the
	// complete realm and vanilla packs.  The initial community above is only a
	// locality hint; do not replace a mapped genus with an unrelated species
	// merely because that one grove lacks it.
	if (request.genus && mapped.empty()) {
		auto same_genus = [&](const std::string &species) {
			const auto sep = species.find('_');
			auto genus =
					species.substr(0, sep == std::string::npos ? species.size() : sep);
			if (genus.size() != request.genus->size())
				return false;
			for (std::size_t n = 0; n < genus.size(); ++n)
				if (std::tolower(static_cast<unsigned char>(genus[n])) !=
						std::tolower(static_cast<unsigned char>((*request.genus)[n])))
					return false;
			return true;
		};
		auto scan_pack = [&](const Data::Pack &candidate) {
			for (const auto &c : candidate.communities)
				for (std::size_t si = 0; si < c.species.size(); ++si)
					if (si < c.species_names.size() && same_genus(c.species_names[si]))
						for (const auto i : c.species[si])
							if (data_->allowed(data_->entries[i].size) &&
									(palms_allowed || si >= c.species_palm.size() ||
											!c.species_palm[si]))
								mapped.push_back(i);
		};
		scan_pack(data_->realm);
		if (mapped.empty())
			scan_pack(data_->vanilla);
	}
	if (!mapped.empty())
		all = std::move(mapped);
	if (all.empty() && request.conifer && typed.empty()) {
		auto scan_type = [&](const Data::Pack &candidate) {
			for (const auto &c : candidate.communities)
				for (std::size_t si = 0; si < c.species.size(); ++si)
					if (si < c.species_conifer.size() &&
							c.species_conifer[si] == *request.conifer)
						for (const auto i : c.species[si])
							if (data_->allowed(data_->entries[i].size) &&
									(palms_allowed || si >= c.species_palm.size() ||
											!c.species_palm[si]))
								typed.push_back(i);
		};
		scan_type(data_->realm);
		if (typed.empty())
			scan_type(data_->vanilla);
	}
	if (mapped.empty() && !typed.empty())
		all = std::move(typed);
	if (all.empty())
		return std::nullopt;
	TreeSize target = request.want_size.value_or(data_->roll_size(sx, sz));
	if (request.want_size) {
		TreeSize best = TreeSize::Giant;
		bool found = false;
		for (auto i : all)
			if (data_->entries[i].size <= target &&
					(!found || data_->entries[i].size > best)) {
				best = data_->entries[i].size;
				found = true;
			}
		if (found)
			target = best;
	}
	std::vector<std::size_t> tier;
	for (auto i : all)
		if (data_->entries[i].size == target)
			tier.push_back(i);
	if (tier.empty())
		tier = all;
	const auto roll = land_cover::coord_hash(sx + 5, sz + 11) % 100;
	const int wanted = roll < 78 ? 1 : roll < 96 ? 2 : 3;
	std::vector<std::size_t> width;
	for (int w = wanted; w >= 1 && width.empty(); --w)
		for (auto i : tier)
			if (data_->entries[i].width == w)
				width.push_back(i);
	if (width.empty())
		width = tier;
	const auto index = width[land_cover::coord_hash(sx + 313, sz + 727) % width.size()];
	return SlotSelection{sx, sz, index,
			unsigned(land_cover::coord_hash(sx ^ 0x5bd1, sz ^ 0x9e37) % 4)};
}

std::optional<SlotSelection> RegionSelector::pick_mapped(
		int x, int z, Habitat hint, int elevation, const MappedRequest &mapped) const
{
	SlotRequest request;
	request.want_size = mapped.want_size;
	request.eco = mapped.eco;
	request.beach = mapped.beach;
	request.tagged = mapped.genus.has_value() || mapped.conifer.has_value();
	request.genus = mapped.genus;
	request.conifer = mapped.conifer;
	// Mapped OSM trees are density-decided: Rust never applies the grove
	// clearing roll to an explicitly tagged trunk.
	request.density_decided = true;
	return pick_slot(x, z, hint, elevation, request);
}
}
