#include "region.h"
#include "schematic.h"
#include "tree_pack.h"
#include "../land_cover/land_cover.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <utility>
#include <array>
#include <memory>
namespace arnis::trees
{
bool is_palm(const std::string &s)
{
	std::string n = s;
	for (char &c : n)
		c = char(std::tolower((unsigned char)c));
	for (const char *p : {"palm", "cocos", "roystonea", "sabal", "acrocomia", "phoenix",
				 "washingtonia", "borassus", "elaeis"})
		if (n.find(p) != std::string::npos)
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
	struct Entry
	{
		Schematic schem;
		TreeSize size;
		std::uint8_t width;
	};
	struct Community
	{
		Habitat habitat = Habitat::Lowland;
		unsigned density = 20;
		std::vector<std::vector<std::size_t>> species;
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
	double scale = 1.;
	int ground_level = 0;
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
		double scale, int ground_level, const SizeFilter &sizes, bool exclude_palms)
{
	auto data = std::make_shared<Data>();
	data->scale = scale;
	data->ground_level = ground_level;
	data->sizes = sizes;
	auto load_pack = [&](const std::filesystem::path &manifest, Data::Pack &out) {
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
		for (const auto &json : communities) {
			Data::Community community;
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
								bool leaves = std::any_of(schem.voxels.begin(),
										schem.voxels.end(), [](const auto &v) {
											return v.block.find("leaves") !=
												   std::string::npos;
										});
								if (leaves) {
									const auto size = schematic_size(schem);
									data->entries.push_back({std::move(schem), size,
											std::uint8_t(width)});
									variants.push_back(data->entries.size() - 1);
								}
							} catch (...) {
							}
						}
				if (!variants.empty())
					species.push_back(std::move(variants));
			}
			if (species.empty())
				continue;
			community.species = std::move(species);
			const auto idx = out.communities.size();
			if (json.value("name", "") == fallback)
				out.fallback = idx;
			out.by_habitat[habitat_index(community.habitat)].push_back(idx);
			out.communities.push_back(std::move(community));
		}
	};
	load_pack(source.realm_path("region.json"), data->realm);
	if (data->realm.empty())
		return std::nullopt;
	if (source.realm() != "vanilla-plus")
		load_pack(source.vanilla_path("region.json"), data->vanilla);
	return RegionSelector{std::move(data)};
}
std::optional<RegionSelector> RegionSelector::load_for_location(double latitude,
		double longitude, const std::filesystem::path &root, double scale,
		int ground_level, const SizeFilter &sizes)
{
	TreePackSource source(realm_for_latlon(latitude, longitude), root);
	return load(source, scale, ground_level, sizes, !subtropical_latitude(latitude));
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
	const int spacing = base_spacing();
	auto [sx, sz] = trunk_slot_s(x, z, spacing);
	const bool montane =
			(double(elevation - data_->ground_level) / std::max(.001, data_->scale) >
					450.) &&
			smooth_noise(sx, sz, 64) < .6;
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
	for (const auto &species : community.species)
		for (auto i : species)
			if (data_->allowed(data_->entries[i].size))
				all.push_back(i);
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
}
