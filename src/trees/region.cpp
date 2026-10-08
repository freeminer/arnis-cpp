#include "region.h"
#include "schematic.h"
#include "tree_pack.h"
#include "../land_cover/land_cover.h"
#include "../ground_generation.h"
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
	return false;
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
	return ground_generation::value_noise_01(x, z, scale);
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
	struct WeightedCommunity
	{
		std::size_t community = 0;
		unsigned weight = 1;
	};
	using Pool = std::vector<WeightedCommunity>;
	using Pools = std::array<std::array<Pool, 3>, 3>;
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
		// Communities loaded from this pack's own manifest. Mix-derived entries
		// are appended to `communities` but must not enter the broad random fallback.
		std::size_t own_count = 0;
		std::size_t fallback = 0;
		bool empty() const { return communities.empty(); }
	};
	std::vector<Entry> entries;
	Pack realm, vanilla;
	std::unordered_map<std::uint16_t, std::vector<EcoChoice>> eco_choices;
	std::unordered_map<std::uint16_t, std::vector<std::size_t>> eco_communities;
	std::unordered_map<std::uint16_t, std::size_t> eco_beach_community;
	// Index 0 excludes palms, index 1 permits them. Latitude may be attached
	// after pack loading, so both Rust-equivalent pool sets are prepared once.
	std::unordered_map<std::uint16_t, std::array<Pools, 2>> eco_pools;
	std::unordered_map<std::uint16_t, std::array<Pools, 2>> vanilla_pools;
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
		double blocks_per_meter, const std::vector<std::uint16_t> &ecoregion_ids)
{
	auto data = std::make_shared<Data>();
	data->scale = scale;
	data->blocks_per_meter = blocks_per_meter;
	data->ground_level = ground_level;
	data->palms_default = !exclude_palms;
	data->sizes = sizes;
	auto build_community = [&](const nlohmann::json &json,
								   const std::filesystem::path &manifest,
								   const std::string &pack_name,
								   const std::vector<std::string_view> &excluded)
			-> std::optional<Data::Community> {
		Data::Community community;
		community.name = json.value("name", "");
		community.pack = pack_name;
		community.habitat = habitat_from_string(json.value("habitat", "lowland"));
		community.density = json.value("density", 20u);
		std::vector<std::vector<std::size_t>> species;
		for (const auto &sp : json.value("species", nlohmann::json::array())) {
			const auto name = sp.value("name", "");
			if ((exclude_palms && is_palm(name)) ||
					std::any_of(excluded.begin(), excluded.end(), [&](const auto ex) {
						const auto sep = name.find('_');
						const auto genus = name.substr(
								0, sep == std::string::npos ? name.size() : sep);
						return name == ex || genus == ex;
					}))
				continue;
			std::vector<std::size_t> variants;
			for (const auto &[field, width] :
					std::array<std::pair<const char *, unsigned>, 3>{
							{{"w1", 1}, {"w2", 2}, {"w3", 3}}})
				if (sp.contains(field))
					for (const auto &relative : sp[field]) {
						auto path = manifest.parent_path() / relative.get<std::string>();
						try {
							auto schem = load_schem(path);
							if (has_leaves(schem)) {
								const auto size = schematic_size(schem);
								data->entries.push_back(
										{std::move(schem), size, std::uint8_t(width)});
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
			return std::nullopt;
		community.species = std::move(species);
		return community;
	};
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
			auto community = build_community(json, manifest, pack_name, {});
			if (!community)
				continue;
			const auto idx = out.communities.size();
			if (!fallback.empty() && json.value("name", "") == fallback) {
				out.fallback = idx;
				explicit_fallback = true;
			}
			out.by_habitat[habitat_index(community->habitat)].push_back(idx);
			out.communities.push_back(std::move(*community));
		}
		out.own_count = out.communities.size();
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
	else
		data->vanilla.own_count = data->vanilla.communities.size();
	// Resolve the area's ecoregion mixes, including communities referenced from
	// other realm packs, as Rust's attach_ecoregions does. Cache manifests and resolved
	// mix keys so each referenced community is decoded only once. Match Rust's
	// realm_pack.own: derived communities must never become sources for later
	// mixes, or repeated exclusions multiply their copies.
	const auto community_count = data->realm.communities.size();
	std::unordered_map<std::string, std::optional<std::size_t>> resolved_mixes;
	std::unordered_map<std::string, std::optional<nlohmann::json>> mix_manifests;
	for (const auto id : ecoregion_ids) {
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
						const std::string mix_key(body.substr(0, star));
						std::optional<std::size_t> resolved;
						if (const auto found = resolved_mixes.find(mix_key);
								found != resolved_mixes.end()) {
							resolved = found->second;
						} else {
							for (std::size_t ci = 0; ci < community_count; ++ci)
								if (data->realm.communities[ci].pack == pack_name &&
										data->realm.communities[ci].name == name) {
									resolved = ci;
									break;
								}
							if (resolved && !excluded.empty()) {
								const auto ci = *resolved;
								Data::Community filtered = data->realm.communities[ci];
								filtered.species.clear();
								filtered.species_names.clear();
								filtered.species_conifer.clear();
								filtered.species_palm.clear();
								for (std::size_t si = 0;
										si < data->realm.communities[ci].species.size();
										++si) {
									const auto &species =
											data->realm.communities[ci].species_names[si];
									const auto sep = species.find('_');
									const auto genus = species.substr(
											0, sep == std::string::npos ? species.size()
																		: sep);
									const bool omit = std::any_of(excluded.begin(),
											excluded.end(), [&](auto ex) {
												return species == ex || genus == ex;
											});
									if (!omit) {
										filtered.species.push_back(
												data->realm.communities[ci].species[si]);
										filtered.species_names.push_back(species);
										filtered.species_conifer.push_back(
												data->realm.communities[ci]
														.species_conifer[si]);
										filtered.species_palm.push_back(
												data->realm.communities[ci]
														.species_palm[si]);
									}
								}
								if (filtered.species.empty()) {
									resolved.reset();
								} else {
									resolved = data->realm.communities.size();
									data->realm.communities.push_back(
											std::move(filtered));
								}
							}
							if (!resolved) {
								const std::string pack_key(pack_name);
								auto manifest_it = mix_manifests.find(pack_key);
								if (manifest_it == mix_manifests.end()) {
									const auto other_pack = TreePackSource::embedded(
											pack_key, source.root());
									std::optional<nlohmann::json> manifest_json;
									if (const auto manifest =
													other_pack.realm_manifest()) {
										try {
											std::ifstream file(*manifest);
											nlohmann::json parsed;
											file >> parsed;
											manifest_json = std::move(parsed);
										} catch (...) {
										}
									}
									manifest_it =
											mix_manifests
													.emplace(pack_key,
															std::move(manifest_json))
													.first;
								}
								if (manifest_it->second) {
									const auto manifest_path =
											source.root() / pack_key / "region.json";
									for (const auto &community_json :
											manifest_it->second->value("communities",
													nlohmann::json::array()))
										if (community_json.value("name", "") == name) {
											auto community = build_community(
													community_json, manifest_path,
													pack_key, excluded);
											if (community) {
												resolved = data->realm.communities.size();
												data->realm.communities.push_back(
														std::move(*community));
											}
											break;
										}
								}
							}
							resolved_mixes.emplace(mix_key, resolved);
						}
						if (resolved)
							data->eco_choices[id].push_back({*resolved, weight, niche});
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
	// Rust prepares one palm-only community per ecoregion mix. Beach selection
	// then samples that community, instead of biasing toward mixes with more
	// palm species or selecting a palm from an unrelated community.
	for (const auto &[id, community_ids] : data->eco_communities) {
		Data::Community grove;
		grove.name = "__beach_palms_" + std::to_string(id);
		grove.habitat = Habitat::Wet;
		std::unordered_set<std::string> included;
		for (const auto community_id : community_ids) {
			if (community_id >= data->realm.communities.size())
				continue;
			const auto &community = data->realm.communities[community_id];
			for (std::size_t si = 0; si < community.species.size(); ++si) {
				if (si >= community.species_names.size() ||
						si >= community.species_palm.size() ||
						!community.species_palm[si] ||
						!included.insert(community.species_names[si]).second)
					continue;
				grove.species.push_back(community.species[si]);
				grove.species_names.push_back(community.species_names[si]);
				grove.species_conifer.push_back(community.species_conifer[si]);
				grove.species_palm.push_back(true);
			}
		}
		if (!grove.species.empty()) {
			data->eco_beach_community[id] = data->realm.communities.size();
			data->realm.communities.push_back(std::move(grove));
		}
	}
	// Resolve Rust's plain/montane/wet and untagged/conifer/broadleaf weighted
	// pools once per ecoregion. Slot selection then performs only one weighted
	// roll, rather than allocating and filtering communities for every tree.
	auto build_pools = [](const std::vector<Data::EcoChoice> &choices,
							   const Data::Pack &pack, bool palms) {
		using Pool = Data::Pool;
		auto keep = [&](const Pool &pool, auto predicate) {
			Pool out;
			for (const auto &item : pool)
				if (item.community < pack.communities.size() &&
						predicate(pack.communities[item.community]))
					out.push_back(item);
			return out;
		};
		auto or_pool = [](Pool first, Pool second) {
			return first.empty() ? std::move(second) : std::move(first);
		};
		Pool all, any, montane, wet;
		for (const auto &choice : choices) {
			const Data::WeightedCommunity item{choice.community, choice.weight};
			all.push_back(item);
			if (choice.niche == Data::Niche::Any)
				any.push_back(item);
			else if (choice.niche == Data::Niche::Montane)
				montane.push_back(item);
			else if (choice.niche == Data::Niche::Wet)
				wet.push_back(item);
		}
		Pool plain = or_pool(std::move(any), all);
		Pool montane_pool = or_pool(std::move(montane),
				or_pool(keep(plain,
								[](const auto &c) {
									return c.habitat == Habitat::Conifer;
								}),
						plain));
		Pool wet_pool = or_pool(std::move(wet),
				or_pool(keep(plain,
								[](const auto &c) { return c.habitat == Habitat::Wet; }),
						plain));
		const Pool all_conifers = or_pool(
				keep(all, [](const auto &c) { return c.habitat == Habitat::Conifer; }),
				keep(all, [](const auto &c) {
					return std::any_of(c.species_conifer.begin(), c.species_conifer.end(),
							[](bool conifer) { return conifer; });
				}));
		Data::Pools result;
		const std::array<Pool, 3> places{
				std::move(plain), std::move(montane_pool), std::move(wet_pool)};
		for (std::size_t place = 0; place < places.size(); ++place) {
			Pool pool = places[place];
			if (!palms) {
				auto non_palm = keep(pool, [](const auto &c) {
					return !std::all_of(c.species_palm.begin(), c.species_palm.end(),
							[](bool palm) { return palm; });
				});
				pool = or_pool(std::move(non_palm), std::move(pool));
			}
			result[place][0] = pool;
			const auto conifers = or_pool(
					keep(pool,
							[](const auto &c) { return c.habitat == Habitat::Conifer; }),
					or_pool(keep(pool,
									[](const auto &c) {
										return std::any_of(c.species_conifer.begin(),
												c.species_conifer.end(),
												[](bool value) { return value; });
									}),
							all_conifers));
			result[place][1] = conifers;
			auto broadleaf = [](const auto &c) {
				return std::any_of(c.species_conifer.begin(), c.species_conifer.end(),
						[](bool value) { return !value; });
			};
			result[place][2] = or_pool(keep(pool,
											   [&](const auto &c) {
												   return c.habitat != Habitat::Conifer &&
														  broadleaf(c);
											   }),
					or_pool(keep(pool, broadleaf), pool));
		}
		return result;
	};
	for (const auto &[id, choices] : data->eco_choices) {
		const auto eco = ecoregion::lookup(id);
		if (!eco)
			continue;
		std::vector<Data::EcoChoice> sprinkle;
		for (const auto &[name, weight] : vanilla_sprinkle(eco->biome))
			for (std::size_t i = 0; i < data->vanilla.communities.size(); ++i)
				if (data->vanilla.communities[i].name == name) {
					sprinkle.push_back({i, weight, Data::Niche::Any});
					break;
				}
		data->eco_pools[id][0] = build_pools(choices, data->realm, false);
		data->eco_pools[id][1] = build_pools(choices, data->realm, true);
		data->vanilla_pools[id][0] = build_pools(sprinkle, data->vanilla, false);
		data->vanilla_pools[id][1] = build_pools(sprinkle, data->vanilla, true);
	}
	return RegionSelector{std::move(data)};
}
std::optional<RegionSelector> RegionSelector::load_for_location(double latitude,
		double longitude, const std::filesystem::path &root, double scale,
		int ground_level, const SizeFilter &sizes, double blocks_per_meter,
		const std::optional<std::string> &preferred_realm,
		const std::vector<std::uint16_t> &ecoregion_ids, bool exclude_palms)
{
	// Rust always reads the compiled tree-pack bundle.  Preserve the C++
	// override when supplied, but resolve the bundled asset root for the common
	// empty-root call path as well.
	TreePackSource source = TreePackSource::embedded(
			preferred_realm.value_or(realm_for_latlon(latitude, longitude)), root);
	// Rust drops palm schematics only when the area's ecoregion data is complete
	// and none of its mapped tree communities can use palms. Otherwise retain
	// them so per-cell ecoregion and latitude rules can decide at placement time.
	auto result = load(source, scale, ground_level, sizes, exclude_palms,
			blocks_per_meter, ecoregion_ids);
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

std::optional<SlotSelection> RegionSelector::pick_slot_impl(int x, int z, Habitat hint,
		int elevation, SlotRequest request, bool mapped_selection) const
{
	if (empty())
		return std::nullopt;
	const int spacing = base_spacing();
	auto [sx, sz] = trunk_slot_s(x, z, spacing);
	if (mapped_selection) {
		sx = x;
		sz = z;
	}
	// Invert the vertical affine, including terrain compression, as in region.rs.
	const double per_metre =
			data_->blocks_per_meter > 0.0 ? data_->blocks_per_meter : data_->scale;
	const bool montane =
			((double(elevation) - data_->ground_level) / std::max(.001, per_metre) >
					450.) &&
			smooth_noise(sx, sz, 64) < .6;
	const Habitat tagged_hint = hint;
	const auto eco_mix = request.eco ? data_->eco_choices.find(request.eco->id)
									 : data_->eco_choices.end();
	const bool has_eco_mix =
			eco_mix != data_->eco_choices.end() && !eco_mix->second.empty();
	const bool palms_allowed =
			has_eco_mix && request.eco && data_->latitude_known
					? ecoregion::palms_belong(*request.eco, data_->latitude)
					: data_->palms_default;
	const Habitat effective_hint =
			montane && (tagged_hint == Habitat::Lowland || tagged_hint == Habitat::Wet)
					? Habitat::Conifer
					: tagged_hint;
	const auto blend = land_cover::coord_hash(sx + 7, sz + 13) % 100;
	const bool want_wet = request.wet_ground ||
						  (request.tagged && tagged_hint == Habitat::Wet) ||
						  (mapped_selection && tagged_hint == Habitat::Wet);
	const bool want_conifer =
			request.conifer.value_or(request.tagged && tagged_hint == Habitat::Conifer);
	const bool want_broadleaf =
			request.conifer ? !*request.conifer
							: request.tagged && tagged_hint == Habitat::Lowland;
	const Data::Pack *pack = &data_->realm;
	auto choose_community = [&](const Data::Pack &source) {
		const auto &preferred = source.by_habitat[habitat_index(effective_hint)];
		const auto &original = source.by_habitat[habitat_index(tagged_hint)];
		const auto *candidates = !preferred.empty()	 ? &preferred
								 : !original.empty() ? &original
													 : nullptr;
		if (!candidates)
			return std::min(source.fallback, source.communities.size() - 1);
		const auto selected =
				static_cast<std::size_t>(smooth_noise(sx, sz, 160) * candidates->size());
		return (*candidates)[std::min(candidates->size() - 1, selected)];
	};
	std::size_t ci = choose_community(*pack);
	if (!mapped_selection && blend >= 97 && pack->own_count > 0)
		ci = land_cover::coord_hash(sx + 5, sz + 9) % pack->own_count;

	// Pools are prepared when the region library loads, matching Rust's O(1)
	// pool resolution and avoiding per-slot temporary allocations.
	auto choose_cached = [&](const Data::Pools &pools,
								 std::uint32_t salt) -> std::optional<std::size_t> {
		const std::size_t place = montane ? 1 : want_wet ? 2 : 0;
		const std::size_t tag = want_conifer ? 1 : want_broadleaf ? 2 : 0;
		const auto &pool = pools[place][tag];
		unsigned total = 0;
		for (const auto &choice : pool)
			total += choice.weight;
		if (!total)
			return std::nullopt;
		const auto h = land_cover::coord_hash(sx ^ static_cast<int>(salt), sz ^ 0x5a17);
		const double roll = h % 5 == 0 ? double((h >> 8) % 10000) / 10000.
									   : ground_generation::patch_noise(sx, sz, 48, salt);
		unsigned selected = std::min(total - 1, static_cast<unsigned>(roll * total));
		for (const auto &choice : pool) {
			if (selected < choice.weight)
				return choice.community;
			selected -= choice.weight;
		}
		return pool.back().community;
	};
	auto choose_vanilla = [&]() -> std::optional<std::size_t> {
		if (!request.eco || data_->vanilla.empty())
			return std::nullopt;
		const auto pools = data_->vanilla_pools.find(request.eco->id);
		if (pools != data_->vanilla_pools.end())
			if (const auto selected = choose_cached(
						pools->second[palms_allowed ? 1 : 0], 0x3c0b22f))
				return selected;
		const auto &conifers = data_->vanilla.by_habitat[habitat_index(Habitat::Conifer)];
		return conifers.empty()
					   ? std::optional<std::size_t>(data_->vanilla.fallback)
					   : std::optional<std::size_t>(conifers[std::min<std::size_t>(
								 conifers.size() - 1,
								 static_cast<std::size_t>(
										 smooth_noise(sx, sz, 160) * conifers.size()))]);
	};

	bool selected_from_mix = false;
	if (request.eco) {
		const auto mix = data_->eco_pools.find(request.eco->id);
		if (mix != data_->eco_pools.end()) {
			const auto beach = data_->eco_beach_community.find(request.eco->id);
			const bool choose_beach_palms =
					request.beach && palms_allowed && !want_wet && !want_conifer &&
					beach != data_->eco_beach_community.end() &&
					land_cover::coord_hash(sx ^ 0x0BEA, sz ^ 0x0C11) % 100 < 70;
			if (choose_beach_palms) {
				ci = beach->second;
				selected_from_mix = true;
			} else if (!mapped_selection && blend >= 67 && blend < 97 &&
					   !data_->vanilla.empty()) {
				if (const auto vanilla = choose_vanilla()) {
					pack = &data_->vanilla;
					ci = *vanilla;
				}
			} else {
				if (const auto selected = choose_cached(
							mix->second[palms_allowed ? 1 : 0], 0x3c0a11e)) {
					ci = *selected;
					selected_from_mix = true;
				}
				if (!mapped_selection && !selected_from_mix && !data_->vanilla.empty())
					if (const auto vanilla = choose_vanilla()) {
						pack = &data_->vanilla;
						ci = *vanilla;
					}
			}
		}
	}
	if (!mapped_selection && !selected_from_mix && pack == &data_->realm && blend >= 67 &&
			blend < 97 && !data_->vanilla.empty()) {
		pack = &data_->vanilla;
		ci = choose_community(*pack);
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
	using SpeciesPool = std::vector<std::vector<std::size_t>>;
	SpeciesPool all_species;
	SpeciesPool typed_species;
	SpeciesPool mapped_species;
	auto same_genus = [&](const std::string &species) {
		const auto sep = species.find('_');
		const auto genus =
				species.substr(0, sep == std::string::npos ? species.size() : sep);
		if (genus.size() != request.genus->size())
			return false;
		for (std::size_t n = 0; n < genus.size(); ++n)
			if (std::tolower(static_cast<unsigned char>(genus[n])) !=
					std::tolower(static_cast<unsigned char>((*request.genus)[n])))
				return false;
		return true;
	};
	for (std::size_t si = 0; si < community.species.size(); ++si) {
		const bool palm =
				si < community.species_palm.size() && community.species_palm[si];
		if (palm && !palms_allowed)
			continue;
		all_species.push_back(community.species[si]);
		if (request.genus && !mapped_selection && si < community.species_names.size()) {
			const auto &species = community.species_names[si];
			if (same_genus(species))
				mapped_species.push_back(community.species[si]);
		}
		if ((want_conifer && si < community.species_conifer.size() &&
					community.species_conifer[si]) ||
				(want_broadleaf && si < community.species_conifer.size() &&
						!community.species_conifer[si]))
			typed_species.push_back(community.species[si]);
	}
	// Rust's mapped path searches every community in the active mix, then the
	// complete realm and vanilla packs.  The initial community above is only a
	// locality hint; do not replace a mapped genus with an unrelated species
	// merely because that one grove lacks it.
	if (request.genus && mapped_selection && mapped_species.empty()) {
		auto scan_community = [&](std::size_t ci) {
			if (ci >= data_->realm.communities.size())
				return;
			const auto &candidate = data_->realm.communities[ci];
			for (std::size_t si = 0; si < candidate.species.size(); ++si)
				if (si < candidate.species_names.size() &&
						same_genus(candidate.species_names[si]))
					mapped_species.push_back(candidate.species[si]);
		};
		if (request.eco) {
			if (const auto it = data_->eco_communities.find(request.eco->id);
					it != data_->eco_communities.end())
				for (const auto ci_mix : it->second)
					scan_community(ci_mix);
		}
		auto scan_pack = [&](const Data::Pack &candidate) {
			for (const auto &c : candidate.communities)
				for (std::size_t si = 0; si < c.species.size(); ++si)
					if (si < c.species_names.size() && same_genus(c.species_names[si]) &&
							(mapped_selection || palms_allowed ||
									si >= c.species_palm.size() || !c.species_palm[si]))
						mapped_species.push_back(c.species[si]);
		};
		if (mapped_species.empty()) {
			scan_pack(data_->realm);
			if (mapped_species.empty())
				scan_pack(data_->vanilla);
		}
	}
	const bool genus_selected = !mapped_species.empty();
	if ((mapped_selection || all_species.empty()) && request.conifer &&
			typed_species.empty() && !genus_selected) {
		if (mapped_selection && request.eco) {
			if (const auto it = data_->eco_communities.find(request.eco->id);
					it != data_->eco_communities.end())
				for (const auto ci_mix : it->second) {
					if (ci_mix >= data_->realm.communities.size())
						continue;
					const auto &c = data_->realm.communities[ci_mix];
					for (std::size_t si = 0; si < c.species.size(); ++si)
						if (si < c.species_conifer.size() &&
								c.species_conifer[si] == *request.conifer &&
								(palms_allowed || si >= c.species_palm.size() ||
										!c.species_palm[si]))
							typed_species.push_back(c.species[si]);
				}
		}
		auto scan_type = [&](const Data::Pack &candidate) {
			for (const auto &c : candidate.communities)
				for (std::size_t si = 0; si < c.species.size(); ++si)
					if (si < c.species_conifer.size() &&
							c.species_conifer[si] == *request.conifer &&
							(palms_allowed || si >= c.species_palm.size() ||
									!c.species_palm[si]))
						typed_species.push_back(c.species[si]);
		};
		if (typed_species.empty())
			scan_type(data_->realm);
	}
	SpeciesPool species_candidates = genus_selected			  ? std::move(mapped_species)
									 : !typed_species.empty() ? std::move(typed_species)
															  : std::move(all_species);
	// Rust never drops an explicitly mapped tree solely because palm filtering
	// exhausted the selected community; its final fallback retries with palms.
	if (species_candidates.empty() && mapped_selection)
		species_candidates = community.species;
	if (species_candidates.empty())
		return std::nullopt;
	// Rust chooses a species in proportion to its usable schematic count, then
	// applies width and preferred size within that species. Flattening the pack
	// first skews the species mix and the width fallback.
	std::optional<TreeSize> requested_tier;
	if (request.want_size) {
		for (const auto &species : species_candidates)
			for (const auto i : species)
				if (data_->allowed(data_->entries[i].size) &&
						data_->entries[i].size <= *request.want_size &&
						(!requested_tier || data_->entries[i].size > *requested_tier))
					requested_tier = data_->entries[i].size;
		if (!requested_tier)
			for (const auto &species : species_candidates)
				for (const auto i : species)
					if (data_->allowed(data_->entries[i].size) &&
							(!requested_tier || data_->entries[i].size < *requested_tier))
						requested_tier = data_->entries[i].size;
	}
	std::vector<std::vector<std::size_t>> usable_species;
	std::size_t total = 0;
	for (const auto &species : species_candidates) {
		std::vector<std::size_t> usable;
		for (const auto i : species)
			if (data_->allowed(data_->entries[i].size) &&
					(!request.want_size || !requested_tier ||
							data_->entries[i].size == *requested_tier))
				usable.push_back(i);
		total += usable.size();
		usable_species.push_back(std::move(usable));
	}
	if (!total) {
		// Rust falls back to any non-excluded model if UI or scale filtering
		// removes every candidate, bypassing width and size preferences.
		std::size_t any = 0;
		for (const auto &species : species_candidates)
			any += species.size();
		if (!any)
			return std::nullopt;
		std::size_t fallback = land_cover::coord_hash(sx + 31, sz + 57) % any;
		for (const auto &species : species_candidates) {
			if (fallback < species.size()) {
				const auto index = species[land_cover::coord_hash(sx + 313, sz + 727) %
										   species.size()];
				return SlotSelection{sx, sz, index,
						unsigned(land_cover::coord_hash(sx ^ 0x5bd1, sz ^ 0x9e37) % 4)};
			}
			fallback -= species.size();
		}
		return std::nullopt;
	}
	std::size_t pick = land_cover::coord_hash(sx + 31, sz + 57) % total;
	std::vector<std::size_t> selected_species;
	for (const auto &species : usable_species) {
		if (pick < species.size()) {
			selected_species = species;
			break;
		}
		pick -= species.size();
	}
	if (selected_species.empty())
		return std::nullopt;
	const auto roll = land_cover::coord_hash(sx + 5, sz + 11) % 100;
	const int wanted = roll < 78 ? 1 : roll < 96 ? 2 : 3;
	std::vector<std::size_t> width;
	for (int w = wanted; w >= 1 && width.empty(); --w)
		for (auto i : selected_species)
			if (data_->entries[i].width == w)
				width.push_back(i);
	if (width.empty())
		width = selected_species;
	if (!request.want_size) {
		const auto target = data_->roll_size(sx, sz);
		std::vector<std::size_t> size_matches;
		for (const auto i : width)
			if (data_->entries[i].size == target)
				size_matches.push_back(i);
		if (!size_matches.empty())
			width = std::move(size_matches);
	}
	const auto index = width[land_cover::coord_hash(sx + 313, sz + 727) % width.size()];
	return SlotSelection{sx, sz, index,
			unsigned(land_cover::coord_hash(sx ^ 0x5bd1, sz ^ 0x9e37) % 4)};
}

std::optional<SlotSelection> RegionSelector::pick_slot(
		int x, int z, Habitat hint, int elevation, SlotRequest request) const
{
	return pick_slot_impl(x, z, hint, elevation, request, false);
}

std::optional<SlotSelection> RegionSelector::pick_mapped(
		int x, int z, Habitat hint, int elevation, const MappedRequest &mapped) const
{
	SlotRequest request;
	request.want_size = mapped.want_size;
	request.eco = mapped.eco;
	request.beach = mapped.beach;
	request.tagged = mapped.conifer.has_value();
	request.genus = mapped.genus;
	request.conifer = mapped.conifer;
	// Mapped OSM trees are density-decided: Rust never applies the grove
	// clearing roll to an explicitly tagged trunk.
	request.density_decided = true;
	return pick_slot_impl(x, z, hint, elevation, request, true);
}
}
