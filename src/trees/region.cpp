#include "region.h"
#include "schematic.h"
#include "tree_pack.h"
#include "../land_cover/land_cover.h"
#include "../ground_generation.h"
#include "../element_processing/tree.h"
#include "../rust_numeric.h"
#include "mapped.h"
#include <algorithm>
#include <atomic>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <utility>
#include <array>
#include <memory>
#include <map>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <tuple>
#include <thread>
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
		if (c >= 'A' && c <= 'Z')
			c = char(c + ('a' - 'A'));
	static constexpr const char *genera[] = {"acrocomia", "archontophoenix", "areca",
			"astrocarym", "astrocaryum", "attalea", "bactris", "beccariophoenix",
			"bismarckia", "borassus", "calyptronoma", "ceroxylon", "cocos",
			"cyrtostachys", "elaeis", "euterpe", "hyphaene", "jubaea", "leopoldinia",
			"livistona", "mauritia", "nypa", "phoenix", "raphia", "rhopalostylis",
			"roystonea", "sabal", "serenoa", "socratea", "washingtonia"};
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
int height_tolerance(int want)
{
	return std::max(2, rust_numeric::round_i32(want * .15));
}
int height_slack(int want)
{
	return std::max(1, rust_numeric::round_i32(want * .08));
}
double default_height_m(int x, int z)
{
	static constexpr std::array<std::tuple<std::uint64_t, double, double>, 5> bands = {
			{{150, 4., 7.}, {500, 7., 12.}, {830, 12., 18.}, {965, 18., 25.},
					{1000, 25., 32.}}};
	const auto roll = land_cover::coord_hash(x + 101, z + 233) % 1000;
	const double t =
			double(land_cover::coord_hash(x ^ 0x2C61, z ^ 0x5E0B) % 1000) / 1000.;
	for (const auto &[limit, low, high] : bands)
		if (roll < limit)
			return low + (high - low) * t;
	return 7. + 5. * t;
}
int default_height_blocks(int x, int z, double scale)
{
	// Match RegionSelector::default_height: even at very small map scales, a
	// requested schematic height is at least one block.
	return std::max(1, rust_numeric::round_i32(default_height_m(x, z) * scale));
}
}

struct RegionSelector::Data
{
	static constexpr std::size_t CROWN_TABLE_MAX = 64;
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
		std::shared_ptr<const Schematic> schem;
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
		// Keep the source's explicit per-species genus, matching Rust Community.
		// It is currently derived from the species key at load time, but retaining
		// it avoids re-parsing names in mix exclusion and mapped-genus selection.
		std::vector<std::string> species_genera;
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
	std::vector<double> crowns;
	std::string realm_name;
	std::size_t total_realm = 0;
	std::size_t total_vanilla = 0;
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
	int capped_height(int wanted) const
	{
		int cap = sizes.max_height();
		if (scale < 1.)
			cap = std::min(cap, size_cap_height(TreeSize::Tall));
		// Rust caps only the upper end here. In particular, do not silently
		// normalize an explicit non-positive requested height; selection and
		// fallback behavior should see the same value as RegionSelector::capped_height.
		return std::min(wanted, cap);
	}
	void build_crowns()
	{
		std::vector<double> sum(CROWN_TABLE_MAX + 1, 0.0);
		std::vector<std::uint32_t> count(CROWN_TABLE_MAX + 1, 0);
		for (const auto &entry : entries) {
			const auto height =
					std::clamp(entry.schem->height, 1, static_cast<int>(CROWN_TABLE_MAX));
			sum[height] += entry.schem->leaf_columns;
			++count[height];
		}
		crowns.resize(CROWN_TABLE_MAX + 1, 50.0);
		for (std::size_t h = 0; h <= CROWN_TABLE_MAX; ++h) {
			for (std::size_t width = 0; width <= CROWN_TABLE_MAX; ++width) {
				const auto lo = std::max<std::size_t>(1, h > width ? h - width : 0);
				const auto hi = std::min(CROWN_TABLE_MAX, h + width);
				std::uint32_t n = 0;
				double total = 0.0;
				for (std::size_t i = lo; i <= hi; ++i) {
					n += count[i];
					total += sum[i];
				}
				if (n >= 3 || (lo == 1 && hi == CROWN_TABLE_MAX)) {
					crowns[h] = n ? total / n : 50.0;
					break;
				}
			}
		}
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
	data->sizes = sizes;
	// Cache failed and successful model reads alike: manifests can reference
	// the same missing or leafless asset from many communities.
	std::unordered_map<std::string, std::optional<std::size_t>> asset_entries;
	// Rust parses each pack's unique model files in parallel, then appends them in
	// manifest order so seeded picks stay stable. Keep this bounded: schematic
	// decoding can temporarily hold both compressed and expanded voxel data.
	auto preload_communities = [&](const nlohmann::json &communities,
									   const std::filesystem::path &manifest,
									   const std::vector<std::string_view> &excluded) {
		struct Job
		{
			std::string key;
			std::filesystem::path path;
			std::uint8_t width;
		};
		std::vector<Job> jobs;
		std::unordered_set<std::string> queued;
		for (const auto &community : communities) {
			for (const auto &species :
					community.value("species", nlohmann::json::array())) {
				const auto name = species.value("name", std::string{});
				if (name.empty())
					continue;
				const auto separator = name.find('_');
				const auto genus = name.substr(
						0, separator == std::string::npos ? name.size() : separator);
				if ((exclude_palms && is_palm(name)) ||
						std::any_of(excluded.begin(), excluded.end(), [&](auto value) {
							return name == value || genus == value;
						}))
					continue;
				for (const auto &[field, width] :
						std::array<std::pair<const char *, std::uint8_t>, 3>{
								{{"w1", 1}, {"w2", 2}, {"w3", 3}}}) {
					const auto paths = species.find(field);
					if (paths == species.end() || !paths->is_array())
						continue;
					for (const auto &relative : *paths) {
						if (!relative.is_string())
							continue;
						auto path = manifest.parent_path() / relative.get<std::string>();
						const auto key = std::filesystem::absolute(path)
												 .lexically_normal()
												 .string();
						if (!asset_entries.contains(key) && queued.insert(key).second)
							jobs.push_back({key, std::move(path), width});
					}
				}
			}
		}
		if (jobs.empty())
			return;

		std::vector<std::shared_ptr<const Schematic>> parsed(jobs.size());
		std::atomic_size_t next{0};
		auto worker = [&] {
			for (;;) {
				const auto index = next.fetch_add(1, std::memory_order_relaxed);
				if (index >= jobs.size())
					return;
				try {
					auto schematic = load_schem_shared(jobs[index].path);
					if (has_leaves(*schematic))
						parsed[index] = std::move(schematic);
				} catch (...) {
					// A missing or invalid asset is cached as a miss below.
				}
			}
		};
		const auto hardware = std::max(1u, std::thread::hardware_concurrency());
		const auto worker_count =
				std::min<std::size_t>(jobs.size(), std::min<std::size_t>(16, hardware));
		std::vector<std::jthread> workers;
		workers.reserve(worker_count > 0 ? worker_count - 1 : 0);
		for (std::size_t i = 1; i < worker_count; ++i)
			workers.emplace_back(worker);
		worker();
		workers.clear(); // jthread joins before results are consumed.
		for (std::size_t i = 0; i < jobs.size(); ++i) {
			std::optional<std::size_t> entry;
			if (parsed[i]) {
				auto schematic = std::move(parsed[i]);
				const auto size = schematic_size(*schematic);
				data->entries.push_back({std::move(schematic), size, jobs[i].width});
				entry = data->entries.size() - 1;
			}
			asset_entries.emplace(jobs[i].key, entry);
		}
	};
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
							const auto asset_key = std::filesystem::absolute(path)
														   .lexically_normal()
														   .string();
							if (const auto it = asset_entries.find(asset_key);
									it != asset_entries.end()) {
								if (it->second)
									variants.push_back(*it->second);
								continue;
							}
							auto [asset_it, inserted] =
									asset_entries.emplace(asset_key, std::nullopt);
							(void)inserted;
							auto schem = load_schem_shared(path);
							if (has_leaves(*schem)) {
								const auto size = schematic_size(*schem);
								data->entries.push_back(
										{std::move(schem), size, std::uint8_t(width)});
								variants.push_back(data->entries.size() - 1);
								asset_it->second = data->entries.size() - 1;
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
				community.species_genera.push_back(genus);
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
							 const std::string &pack_name) -> std::string {
		std::ifstream in(manifest);
		if (!in)
			return {};
		nlohmann::json root;
		try {
			in >> root;
		} catch (...) {
			return {};
		}
		// Mirror Rust's required MRegion/MCommunity fields and serde defaults:
		// only species width lists and community density are optional. Reject a
		// malformed pack instead of silently generating from a partial manifest.
		if (!root.is_object() || !root.contains("realm") || !root["realm"].is_string() ||
				!root.contains("default_community") ||
				!root["default_community"].is_string() || !root.contains("communities") ||
				!root["communities"].is_array())
			return {};
		for (const auto &community : root["communities"]) {
			if (!community.is_object() || !community.contains("name") ||
					!community["name"].is_string() || !community.contains("habitat") ||
					!community["habitat"].is_string() || !community.contains("species") ||
					!community["species"].is_array())
				return {};
			if (community.contains("density")) {
				const auto &density = community["density"];
				const bool valid_density =
						density.is_number_unsigned()
								? density.get<std::uint64_t>() <=
										  std::numeric_limits<std::uint32_t>::max()
								: density.is_number_integer() &&
										  density.get<std::int64_t>() >= 0 &&
										  static_cast<std::uint64_t>(
												  density.get<std::int64_t>()) <=
												  std::numeric_limits<
														  std::uint32_t>::max();
				if (!valid_density)
					return {};
			}
			for (const auto &species : community["species"]) {
				if (!species.is_object())
					return {};
				if (species.contains("name") && !species["name"].is_string())
					return {};
				for (const char *field : {"w1", "w2", "w3"}) {
					if (!species.contains(field))
						continue;
					const auto &paths = species[field];
					if (!paths.is_array() ||
							std::any_of(paths.begin(), paths.end(),
									[](const auto &path) { return !path.is_string(); }))
						return {};
				}
			}
		}
		const auto manifest_realm = root.value("realm", std::string{});
		const auto communities = root.value("communities", nlohmann::json::array());
		preload_communities(communities, manifest, {});
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
		return manifest_realm;
	};
	const auto manifest_realm =
			load_pack(source.realm_path("region.json"), data->realm, source.realm());
	if (data->realm.empty())
		return std::nullopt;
	data->realm_name = manifest_realm;
	data->total_realm = data->entries.size();
	// Match Rust: whether to load the vanilla sprinkle is determined by the
	// selected pack's manifest realm, not by the source directory name.
	if (manifest_realm != "vnplus")
		load_pack(source.vanilla_path("region.json"), data->vanilla, "vanilla-plus");
	else
		data->vanilla.own_count = data->vanilla.communities.size();
	data->total_vanilla = data->entries.size() - data->total_realm;
	// Resolve the area's ecoregion mixes, including communities referenced from
	// other realm packs, as Rust's attach_ecoregions does. Cache manifests and resolved
	// mix keys so each referenced community is decoded only once. Match Rust's
	// realm_pack.own: derived communities must never become sources for later
	// mixes, or repeated exclusions multiply their copies.
	const auto community_count = data->realm.communities.size();
	std::unordered_map<std::string, std::optional<std::size_t>> resolved_mixes;
	std::unordered_map<std::string, std::optional<nlohmann::json>> mix_manifests;
	// Rust attaches ecoregion mixes only when full-size schematic trees can be
	// placed. At micro-tree scales resolving cross-pack communities is unused
	// work and can retain a large set of extra schematics for this selector.
	for (const auto id : ecoregion_ids) {
		if (scale < MICRO_TREE_MAX_SCALE)
			continue;
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
					auto weight_text = body.substr(star + 1);
					// Rust's u32::from_str accepts a leading '+', while
					// std::from_chars deliberately does not. Preserve that grammar.
					if (weight_text.starts_with('+'))
						weight_text.remove_prefix(1);
					std::uint32_t weight = 0;
					const auto [weight_end, weight_error] =
							std::from_chars(weight_text.data(),
									weight_text.data() + weight_text.size(), weight);
					if (weight_error != std::errc{} ||
							weight_end != weight_text.data() + weight_text.size())
						weight = 0;
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
								filtered.species_genera.clear();
								filtered.species_conifer.clear();
								filtered.species_palm.clear();
								for (std::size_t si = 0;
										si < data->realm.communities[ci].species.size();
										++si) {
									const auto &species =
											data->realm.communities[ci].species_names[si];
									const auto &genus = data->realm.communities[ci]
																.species_genera[si];
									const bool omit = std::any_of(excluded.begin(),
											excluded.end(), [&](auto ex) {
												return species == ex || genus == ex;
											});
									if (!omit) {
										filtered.species.push_back(
												data->realm.communities[ci].species[si]);
										filtered.species_names.push_back(species);
										filtered.species_genera.push_back(genus);
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
									const auto other_communities =
											manifest_it->second->value("communities",
													nlohmann::json::array());
									const auto community_it =
											std::find_if(other_communities.begin(),
													other_communities.end(),
													[&](const auto &community) {
														return community.value("name",
																	   "") == name;
													});
									if (community_it != other_communities.end()) {
										// Rust preloads only the referenced community. Loading every
										// community from this pack changes crown statistics (unused
										// models are counted) and needlessly retains their schematics.
										auto selected_community = nlohmann::json::array();
										selected_community.push_back(*community_it);
										preload_communities(selected_community,
												manifest_path, excluded);
										auto community = build_community(*community_it,
												manifest_path, pack_key, excluded);
										if (community) {
											resolved = data->realm.communities.size();
											data->realm.communities.push_back(
													std::move(*community));
										}
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
			// Rust sorts and deduplicates the community IDs before palm-grove
			// construction; keep species order and seeded selection identical.
			std::sort(communities.begin(), communities.end());
		}
	}
	// Rust prepares one palm-only community per ecoregion mix. Beach selection
	// then samples that community, instead of biasing toward mixes with more
	// palm species or selecting a palm from an unrelated community.
	for (const auto id : ecoregion_ids) {
		const auto communities_it = data->eco_communities.find(id);
		if (communities_it == data->eco_communities.end())
			continue;
		const auto &community_ids = communities_it->second;
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
						si >= community.species_genera.size() ||
						si >= community.species_palm.size() ||
						!community.species_palm[si] ||
						!included.insert(community.species_names[si]).second)
					continue;
				grove.species.push_back(community.species[si]);
				grove.species_names.push_back(community.species_names[si]);
				grove.species_genera.push_back(community.species_genera[si]);
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
	// Reuse leaf-type variants across all ecoregions resolved by this load.
	std::map<std::pair<std::size_t, bool>, std::size_t> typed_variants;
	for (const auto id : ecoregion_ids) {
		const auto choices_it = data->eco_choices.find(id);
		if (choices_it == data->eco_choices.end())
			continue;
		const auto &choices = choices_it->second;
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
		auto pools_without_palms = build_pools(choices, data->realm, false);
		auto pools_with_palms = build_pools(choices, data->realm, true);
		// Rust's attach_ecoregions replaces each tagged pool entry with a
		// community variant containing only the requested leaf type. Preserve
		// that community-level weighting here instead of selecting from a mixed
		// community and filtering its species only after the grove roll.
		auto leaf_type_variant = [&](std::size_t community_id, bool conifer) {
			const auto key = std::make_pair(community_id, conifer);
			if (const auto found = typed_variants.find(key);
					found != typed_variants.end())
				return found->second;
			if (community_id >= data->realm.communities.size())
				return community_id;
			const auto &base = data->realm.communities[community_id];
			std::vector<std::size_t> keep;
			for (std::size_t si = 0; si < base.species_conifer.size(); ++si)
				if (base.species_conifer[si] == conifer)
					keep.push_back(si);
			if (keep.empty() || keep.size() == base.species.size()) {
				typed_variants.emplace(key, community_id);
				return community_id;
			}
			Data::Community variant = base;
			variant.species.clear();
			variant.species_names.clear();
			variant.species_genera.clear();
			variant.species_conifer.clear();
			variant.species_palm.clear();
			for (const auto si : keep) {
				variant.species.push_back(base.species[si]);
				if (si < base.species_names.size())
					variant.species_names.push_back(base.species_names[si]);
				if (si < base.species_genera.size())
					variant.species_genera.push_back(base.species_genera[si]);
				variant.species_conifer.push_back(base.species_conifer[si]);
				if (si < base.species_palm.size())
					variant.species_palm.push_back(base.species_palm[si]);
			}
			const auto variant_id = data->realm.communities.size();
			data->realm.communities.push_back(std::move(variant));
			typed_variants.emplace(key, variant_id);
			return variant_id;
		};
		auto apply_leaf_type_variants = [&](Data::Pools &pools) {
			for (auto &place : pools) {
				for (const auto &[tag, conifer] :
						std::array<std::pair<std::size_t, bool>, 2>{
								{{1, true}, {2, false}}}) {
					for (auto &choice : place[tag])
						choice.community = leaf_type_variant(choice.community, conifer);
				}
			}
		};
		apply_leaf_type_variants(pools_without_palms);
		apply_leaf_type_variants(pools_with_palms);
		data->eco_pools[id][0] = std::move(pools_without_palms);
		data->eco_pools[id][1] = std::move(pools_with_palms);
		data->vanilla_pools[id][0] = build_pools(sprinkle, data->vanilla, false);
		data->vanilla_pools[id][1] = build_pools(sprinkle, data->vanilla, true);
	}
	data->build_crowns();
	return RegionSelector{std::move(data)};
}
std::optional<RegionSelector> RegionSelector::load_for_location(double latitude,
		double longitude, const std::filesystem::path &root, double scale,
		int ground_level, const SizeFilter &sizes, double blocks_per_meter,
		const std::optional<std::string> &preferred_realm,
		const std::vector<std::uint16_t> &ecoregion_ids, bool exclude_palms)
{
	// Rust chooses the pack belonging to the largest mapped ecoregion before
	// falling back to the bbox-centre latitude/longitude.  `ecoregion_ids` is
	// supplied in EcoMap::by_area order, so its first usable tree mix carries
	// the same dominant-pack hint even for callers that do not pre-resolve it.
	std::string realm;
	if (preferred_realm) {
		realm = *preferred_realm;
	} else {
		for (const auto id : ecoregion_ids)
			if (const auto mix = ecoregion::tree_mix(id)) {
				realm = std::string(mix->first);
				break;
			}
		if (realm.empty())
			realm = realm_for_latlon(latitude, longitude);
	}
	// Preserve the C++ asset-root override while resolving bundled assets for
	// the common empty-root call path.
	TreePackSource source = TreePackSource::embedded(realm, root);
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

void RegionSelector::report() const
{
	if (!data_)
		return;
	std::array<std::size_t, 5> counts{};
	for (const auto &entry : data_->entries)
		++counts[static_cast<std::size_t>(entry.size)];
	const auto on = [](bool value) { return value ? "on" : "off"; };
	std::cout << "Region tree pack loaded: realm " << data_->realm_name << " - "
			  << data_->total_realm << " regional trees (" << data_->realm.own_count
			  << " communities) + " << data_->total_vanilla << " vanilla sprinkle trees ("
			  << data_->vanilla.communities.size() << " communities)\n";
	const auto borrowed = data_->realm.communities.size() - data_->realm.own_count;
	if (!data_->eco_choices.empty())
		std::cout << "  ecoregions: " << data_->eco_choices.size() << " mapped, "
				  << borrowed << " extra communities ("
				  << data_->entries.size() - data_->total_realm - data_->total_vanilla
				  << " trees)\n";
	std::cout << "  size tiers [schems]: small " << counts[0] << " ["
			  << on(data_->sizes.allows(TreeSize::Small)) << "], medium " << counts[1]
			  << " [" << on(data_->sizes.allows(TreeSize::Medium)) << "], big "
			  << counts[2] << " [" << on(data_->sizes.allows(TreeSize::Big)) << "], tall "
			  << counts[3] << " [" << on(data_->sizes.allows(TreeSize::Tall))
			  << "], giant " << counts[4] << " ["
			  << on(data_->sizes.allows(TreeSize::Giant)) << "]\n";
}

int RegionSelector::base_spacing() const
{
	return !data_ ? 5 : data_->scale < .3 ? 7 : data_->scale < .7 ? 6 : 5;
}
int RegionSelector::slot_spacing(bool density_decided) const
{
	if (!data_)
		return 5;
	const bool micro = data_->scale < .35;
	return density_decided && !micro && data_->scale < .7 ? 4 : base_spacing();
}
double RegionSelector::crown_columns(int height) const
{
	if (!data_ || data_->crowns.empty())
		return 50.0;
	const auto capped = data_->capped_height(height);
	const auto index = std::min<std::size_t>(
			static_cast<std::size_t>(std::max(capped, 1)), data_->crowns.size() - 1);
	return std::max(4.0, data_->crowns[index]);
}
const Schematic *RegionSelector::schematic(std::size_t index) const
{
	return data_ && index < data_->entries.size() ? data_->entries[index].schem.get()
												  : nullptr;
}

std::optional<SlotSelection> RegionSelector::pick_slot_impl(int x, int z, Habitat hint,
		int elevation, SlotRequest request, bool mapped_selection) const
{
	if (empty())
		return std::nullopt;
	const int spacing = slot_spacing(request.density_decided);
	auto [sx, sz] = trunk_slot_s(x, z, spacing);
	if (mapped_selection) {
		sx = x;
		sz = z;
	}
	const int wanted_height = data_->capped_height(
			request.want_height.value_or(default_height_blocks(sx, sz, data_->scale)));
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
	auto choose_community = [&, sz = sz, sx = sx](const Data::Pack &source) {
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
	auto community_fits = [&](const Data::Pack &source, std::size_t index) {
		if (index >= source.communities.size())
			return false;
		const auto &candidate = source.communities[index];
		for (std::size_t si = 0; si < candidate.species.size(); ++si) {
			if (si < candidate.species_palm.size() && candidate.species_palm[si] &&
					!palms_allowed)
				continue;
			for (const auto entry : candidate.species[si])
				if (data_->allowed(data_->entries[entry].size) &&
						std::abs(data_->entries[entry].schem->height - wanted_height) <=
								height_tolerance(wanted_height))
					return true;
		}
		return false;
	};
	auto choose_cached = [&, sx = sx, sz = sz](const Data::Pools &pools,
								 const Data::Pack &source,
								 std::uint32_t salt) -> std::optional<std::size_t> {
		const std::size_t place = montane ? 1 : want_wet ? 2 : 0;
		const std::size_t tag = want_conifer ? 1 : want_broadleaf ? 2 : 0;
		const auto &pool = pools[place][tag];
		const auto h = land_cover::coord_hash(sx ^ static_cast<int>(salt), sz ^ 0x5a17);
		const double roll = h % 5 == 0 ? double((h >> 8) % 10000) / 10000.
									   : ground_generation::patch_noise(sx, sz, 48, salt);
		auto select = [&](const auto &choices) -> std::optional<std::size_t> {
			unsigned total = 0;
			for (const auto &choice : choices)
				total += choice.weight;
			if (!total)
				return std::nullopt;
			unsigned selected = std::min(total - 1, static_cast<unsigned>(roll * total));
			for (const auto &choice : choices) {
				if (selected < choice.weight)
					return choice.community;
				selected -= choice.weight;
			}
			return choices.back().community;
		};
		const auto first = select(pool);
		if (!first || community_fits(source, *first))
			return first;
		std::vector<Data::WeightedCommunity> fitting;
		for (const auto &choice : pool)
			if (community_fits(source, choice.community))
				fitting.push_back(choice);
		return select(fitting).value_or(*first);
	};
	auto choose_vanilla = [&, sx = sx, sz = sz]() -> std::optional<std::size_t> {
		if (!request.eco || data_->vanilla.empty())
			return std::nullopt;
		const auto pools = data_->vanilla_pools.find(request.eco->id);
		if (pools != data_->vanilla_pools.end())
			if (const auto selected = choose_cached(
						pools->second[palms_allowed ? 1 : 0], data_->vanilla, 0x3c0b22f))
				return selected;
		const auto &conifers = data_->vanilla.by_habitat[habitat_index(Habitat::Conifer)];
		return conifers.empty()
					   ? std::optional<std::size_t>(data_->vanilla.fallback)
					   : std::optional<std::size_t>(conifers[std::min<std::size_t>(
								 conifers.size() - 1,
								 static_cast<std::size_t>(
										 smooth_noise(sx, sz, 160) * conifers.size()))]);
	};
	auto choose_fitting_vanilla_mix = [&]() -> std::optional<std::size_t> {
		if (!request.eco || data_->vanilla.empty())
			return std::nullopt;
		const auto pools = data_->vanilla_pools.find(request.eco->id);
		if (pools == data_->vanilla_pools.end())
			return std::nullopt;
		const auto selected = choose_cached(
				pools->second[palms_allowed ? 1 : 0], data_->vanilla, 0x3c0b22f);
		return selected && community_fits(data_->vanilla, *selected) ? selected
																	 : std::nullopt;
	};

	bool selected_from_mix = false;
	if (request.eco) {
		const auto mix = data_->eco_pools.find(request.eco->id);
		if (mix != data_->eco_pools.end()) {
			auto choose_mix_community = [&]() {
				const auto selected = choose_cached(
						mix->second[palms_allowed ? 1 : 0], data_->realm, 0x3c0a11e);
				if (!selected)
					return false;
				pack = &data_->realm;
				ci = *selected;
				selected_from_mix = true;
				// Rust lets a mix community without a model near this crown give way
				// to the fitting vanilla sprinkle before retaining the mix selection.
				if (!community_fits(data_->realm, ci))
					if (const auto vanilla = choose_fitting_vanilla_mix()) {
						pack = &data_->vanilla;
						ci = *vanilla;
					}
				return true;
			};
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
				if (const auto vanilla = choose_vanilla();
						vanilla && community_fits(data_->vanilla, *vanilla)) {
					pack = &data_->vanilla;
					ci = *vanilla;
					selected_from_mix = true;
				} else {
					const bool selected_mix = choose_mix_community();
					// Rust retains the vanilla sprinkle as its final fallback even
					// when no mix community has a model at this height.
					if (!selected_mix)
						if (const auto vanilla = choose_vanilla()) {
							pack = &data_->vanilla;
							ci = *vanilla;
							selected_from_mix = true;
						}
				}
			} else {
				if (!choose_mix_community()) {
					// Procedural selection mirrors Rust's pick_from_mix().or_else(
					// pick_vanilla()). Mapped trees instead fall back to the realm's
					// locality-selected community below; Rust's pick_mapped does not
					// substitute a vanilla community when the tagged mix has no pool.
					if (!mapped_selection) {
						if (const auto vanilla = choose_vanilla()) {
							pack = &data_->vanilla;
							ci = *vanilla;
							selected_from_mix = true;
						}
					}
				}
			}
		}
	}
	// Rust gives an ecoregion-mix result precedence over the rare pack-owned
	// community roll. That roll is only the no-mix fallback, and is a clearing
	// when the realm pack has no manifest-owned communities.
	if (!mapped_selection && !selected_from_mix && blend >= 97) {
		if (pack == &data_->realm) {
			if (!pack->own_count)
				return std::nullopt;
			ci = land_cover::coord_hash(sx + 5, sz + 9) % pack->own_count;
			if (!community_fits(*pack, ci))
				ci = choose_community(*pack);
		}
	}
	if (!mapped_selection && !selected_from_mix && pack == &data_->realm && blend >= 67 &&
			blend < 97 && !data_->vanilla.empty()) {
		const auto vanilla_community = choose_community(data_->vanilla);
		// Rust only lets a vanilla sprinkle stand in when it has a model near the
		// requested height. Otherwise retain the realm's locality-selected stand.
		if (community_fits(data_->vanilla, vanilla_community)) {
			pack = &data_->vanilla;
			ci = vanilla_community;
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
	using SpeciesPool = std::vector<std::vector<std::size_t>>;
	SpeciesPool all_species;
	SpeciesPool typed_species;
	SpeciesPool mapped_species;
	auto same_genus = [&](const std::string &genus) {
		if (genus.size() != request.genus->size())
			return false;
		const auto ascii_lower = [](unsigned char c) {
			return c >= 'A' && c <= 'Z' ? static_cast<unsigned char>(c + ('a' - 'A')) : c;
		};
		for (std::size_t n = 0; n < genus.size(); ++n)
			if (ascii_lower(static_cast<unsigned char>(genus[n])) !=
					ascii_lower(static_cast<unsigned char>((*request.genus)[n])))
				return false;
		return true;
	};
	for (std::size_t si = 0; si < community.species.size(); ++si) {
		const bool palm =
				si < community.species_palm.size() && community.species_palm[si];
		if (palm && !palms_allowed)
			continue;
		all_species.push_back(community.species[si]);
		if (request.genus && !mapped_selection && si < community.species_genera.size()) {
			if (same_genus(community.species_genera[si]))
				mapped_species.push_back(community.species[si]);
		}
		// Procedural trees without an ecoregion mix use the selected habitat
		// community as-is, just like Rust's pick_community -> pick_in_community
		// path. Leaf-type filtering is already baked into the mix pools; an
		// explicit type filter here is only needed for mapped OSM trees, whose
		// requested conifer/broadleaf type may require searching beyond the
		// locality-selected community.
		if (mapped_selection &&
				((want_conifer && si < community.species_conifer.size() &&
						 community.species_conifer[si]) ||
						(want_broadleaf && si < community.species_conifer.size() &&
								!community.species_conifer[si])))
			typed_species.push_back(community.species[si]);
	}
	// Rust's mapped path searches every community in the active mix, then the
	// complete realm and vanilla packs.  The initial community above is only a
	// locality hint; do not replace a mapped genus with an unrelated species
	// merely because that one grove lacks it.
	if (request.genus && mapped_selection) {
		// Rust resolves a mapped genus against every community in the active
		// ecoregion mix before falling back to the whole realm and vanilla packs.
		// The locality-selected community above is only a hint for procedural
		// trees; using it here could replace a mapped species with a different
		// member of the same grove.
		mapped_species.clear();
		auto scan_community = [&](std::size_t ci) {
			if (ci >= data_->realm.communities.size())
				return;
			const auto &candidate = data_->realm.communities[ci];
			for (std::size_t si = 0; si < candidate.species.size(); ++si)
				if (si < candidate.species_genera.size() &&
						same_genus(candidate.species_genera[si]))
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
					if (si < c.species_genera.size() &&
							same_genus(c.species_genera[si]) &&
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
	// Match Rust's canopy-height selection: first honor the enabled size cap,
	// then choose among models nearest the requested height. Species remain
	// weighted by the number of matching models, not flattened across the pack.
	const auto capped = [&](std::size_t i) {
		return data_->allowed(data_->entries[i].size);
	};
	const bool any_capped = std::any_of(species_candidates.begin(),
			species_candidates.end(), [&](const auto &species) {
				return std::any_of(species.begin(), species.end(), capped);
			});
	const auto eligible = [&](std::size_t i) { return !any_capped || capped(i); };
	// If every model in the selected community is above the user's size cap,
	// Rust still plants its smallest available model rather than leaving a hole.
	int pick_height = wanted_height;
	if (!any_capped) {
		pick_height = std::numeric_limits<int>::max();
		for (const auto &species : species_candidates)
			for (const auto &i : species)
				pick_height = std::min(pick_height, data_->entries[i].schem->height);
	}
	const auto distance = [&](std::size_t i) {
		return std::abs(data_->entries[i].schem->height - pick_height);
	};
	int best_distance = std::numeric_limits<int>::max();
	for (const auto &species : species_candidates)
		for (const auto i : species)
			if (eligible(i))
				best_distance = std::min(best_distance, distance(i));
	const int nearest_limit = best_distance + height_slack(pick_height);
	std::vector<std::vector<std::size_t>> usable_species;
	std::size_t total = 0;
	for (const auto &species : species_candidates) {
		std::vector<std::size_t> usable;
		for (const auto i : species)
			if (eligible(i) && distance(i) <= nearest_limit)
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
	request.want_height = mapped.want_height;
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
