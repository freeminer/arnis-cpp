#include "prescan.h"
#include "../../../../arnis_adapter.h"
#include "../palette.h"
#include "../../colors.h"
#include "../../block_definitions.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <unordered_set>
#include <utility>
namespace arnis::models_3d::wikidata
{
namespace
{
std::string tag(const ProcessedElement &e, const char *n)
{
	auto i = e.tags().find(n);
	return i == e.tags().end() ? std::string{} : i->second;
}
std::string trim(std::string value)
{
	const auto first = value.find_first_not_of(" \t\r\n");
	if (first == std::string::npos)
		return {};
	const auto last = value.find_last_not_of(" \t\r\n");
	return value.substr(first, last - first + 1);
}
std::string normalized_name(std::string value)
{
	value = trim(std::move(value));
	std::transform(value.begin(), value.end(), value.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	for (char &c : value)
		if (c == '_' || c == '-')
			c = ' ';
	return value;
}
std::string qid_for_name(const std::string &name)
{
	const auto wanted = normalized_name(name);
	if (wanted.empty())
		return {};
	for (const auto &qid : wikidata_ids())
		if (const auto *entry = lookup_wikidata(qid);
				entry && normalized_name(entry->label) == wanted)
			return qid;
	return {};
}
bool has(const std::vector<std::pair<std::string, std::uint64_t>> &v,
		const std::pair<std::string, std::uint64_t> &k)
{
	return std::find(v.begin(), v.end(), k) != v.end();
}
std::vector<std::pair<int, int>> points(const ProcessedElement &e)
{
	std::vector<std::pair<int, int>> o;
	if (e.is_node())
		o.push_back({e.as_node().x, e.as_node().z});
	else if (e.is_way())
		for (auto &n : e.as_way().nodes)
			o.push_back({n.x, n.z});
	else
		for (auto &m : e.as_relation().members)
			for (auto &n : m.way.nodes)
				o.push_back({n.x, n.z});
	return o;
}
bool direction(const std::string &s, double &o)
{
	const auto value = trim(s);
	if (value.empty())
		return false;
	try {
		std::size_t n = 0;
		o = std::stod(value, &n);
		if (n == value.size()) {
			o = std::fmod(std::fmod(o, 360) + 360, 360);
			return true;
		}
	} catch (...) {
	}
	static const std::pair<const char *, double> v[] = {{"N", 0}, {"NORTH", 0},
			{"NE", 45}, {"E", 90}, {"EAST", 90}, {"SE", 135}, {"S", 180}, {"SOUTH", 180},
			{"SW", 225}, {"W", 270}, {"WEST", 270}, {"NW", 315}, {"NNE", 22.5},
			{"ENE", 67.5}, {"ESE", 112.5}, {"SSE", 157.5}, {"SSW", 202.5}, {"WSW", 247.5},
			{"WNW", 292.5}, {"NNW", 337.5}};
	std::string u = value;
	std::transform(u.begin(), u.end(), u.begin(),
			[](unsigned char c) { return std::toupper(c); });
	for (auto [n, d] : v)
		if (u == n) {
			o = d;
			return true;
		}
	return false;
}
std::optional<double> meters(std::string s)
{
	std::size_t first = 0;
	while (first < s.size() && std::isspace(static_cast<unsigned char>(s[first])))
		++first;
	s.erase(0, first);
	while (!s.empty() && std::isspace((unsigned char)s.back()))
		s.pop_back();
	if (!s.empty() && (s.back() == 'm' || s.back() == 'M'))
		s.pop_back();
	try {
		std::size_t consumed = 0;
		double d = std::stod(s, &consumed);
		while (consumed < s.size() &&
				std::isspace(static_cast<unsigned char>(s[consumed])))
			++consumed;
		if (consumed != s.size())
			return std::nullopt;
		return std::isfinite(d) && d > 0 ? std::optional<double>(d) : std::nullopt;
	} catch (...) {
		return std::nullopt;
	}
}
std::vector<Block> palette_for(const ProcessedElement &e)
{
	// Keep these pools in lockstep with models_3d/wikidata/placement.rs.  They
	// are intentionally broad: voxelized models have no material information
	// after STL conversion, so deterministic variation from the contextual pool
	// is preferable to a single generic stone block.
	static const std::vector<Block> tower = {STONE_BRICKS, COBBLESTONE,
			CRACKED_STONE_BRICKS, POLISHED_ANDESITE, ANDESITE, DEEPSLATE_BRICKS,
			SMOOTH_STONE, CHISELED_STONE_BRICKS};
	static const std::vector<Block> historic = {STONE_BRICKS, CRACKED_STONE_BRICKS,
			CHISELED_STONE_BRICKS, COBBLESTONE, POLISHED_BLACKSTONE_BRICKS,
			MOSSY_STONE_BRICKS, MOSSY_COBBLESTONE, COBBLED_DEEPSLATE, ANDESITE,
			DEEPSLATE_BRICKS};
	static const std::vector<Block> religious = {STONE_BRICKS, CHISELED_STONE_BRICKS,
			QUARTZ_BLOCK, WHITE_CONCRETE, SANDSTONE, SMOOTH_SANDSTONE, POLISHED_DIORITE,
			END_STONE_BRICKS};
	static const std::vector<Block> statue = {
			STONE, SMOOTH_STONE, ANDESITE, POLISHED_ANDESITE, DIORITE, POLISHED_DIORITE};
	static const std::vector<Block> residential = {BRICK, STONE_BRICKS, OAK_PLANKS,
			MUD_BRICKS, SANDSTONE, TERRACOTTA, BROWN_TERRACOTTA};
	static const std::vector<Block> industrial = {GRAY_CONCRETE, LIGHT_GRAY_CONCRETE,
			STONE, SMOOTH_STONE, POLISHED_ANDESITE, DEEPSLATE_BRICKS};
	static const std::vector<Block> lighthouse = {
			WHITE_CONCRETE, QUARTZ_BLOCK, SMOOTH_QUARTZ, POLISHED_DIORITE};
	static const std::vector<Block> fallback = {
			STONE_BRICKS, ANDESITE, POLISHED_ANDESITE, COBBLESTONE, SMOOTH_STONE};
	auto rgb = [&](const char *key) -> std::optional<RGBTuple> {
		auto value = tag(e, key);
		return value.empty() ? std::nullopt : color_text_to_rgb_tuple(value);
	};
	// An explicit colour wins over every category, exactly like Rust.
	if (auto c = rgb("building:colour"))
		return closest_blocks(*c, 5);
	if (auto c = rgb("colour"))
		return closest_blocks(*c, 5);
	if (auto c = rgb("color"))
		return closest_blocks(*c, 5);
	const auto material = tag(e, "building:material");
	const auto lower = [&] {
		std::string v = material;
		std::transform(v.begin(), v.end(), v.begin(),
				[](unsigned char c) { return std::tolower(c); });
		return v;
	}();
	if (lower == "brick")
		return closest_blocks(RGBTuple{151, 98, 83}, 5);
	if (lower == "stone")
		return closest_blocks(RGBTuple{132, 135, 134}, 5);
	if (lower == "sandstone")
		return closest_blocks(RGBTuple{216, 203, 156}, 5);
	if (lower == "concrete")
		return closest_blocks(RGBTuple{128, 127, 128}, 5);
	if (lower == "wood" || lower == "timber")
		return closest_blocks(RGBTuple{162, 131, 79}, 5);
	if (lower == "marble")
		return closest_blocks(RGBTuple{230, 226, 220}, 5);
	if (lower == "granite")
		return closest_blocks(RGBTuple{149, 103, 86}, 5);
	if (lower == "limestone")
		return closest_blocks(RGBTuple{210, 195, 165}, 5);
	if (lower == "metal" || lower == "steel" || lower == "iron")
		return closest_blocks(RGBTuple{180, 180, 180}, 5);
	if (lower == "glass")
		return closest_blocks(RGBTuple{180, 200, 215}, 5);
	if (lower == "copper")
		return closest_blocks(RGBTuple{192, 108, 80}, 5);

	const auto man_made = tag(e, "man_made");
	if (man_made == "tower" || man_made == "obelisk" || man_made == "chimney")
		return tower;
	if (man_made == "lighthouse")
		return lighthouse;
	if (man_made == "monument")
		return statue;

	const auto historic_tag = tag(e, "historic");
	if (historic_tag == "castle" || historic_tag == "fort" || historic_tag == "ruins" ||
			historic_tag == "city_gate" || historic_tag == "archaeological_site")
		return historic;
	if (historic_tag == "memorial" || historic_tag == "monument")
		return statue;

	const auto amenity = tag(e, "amenity");
	if (amenity == "place_of_worship")
		return religious;
	if (amenity == "fountain")
		return lighthouse;
	if (tag(e, "tourism") == "artwork")
		return statue;

	const auto building = tag(e, "building");
	if (building == "industrial" || building == "warehouse")
		return industrial;
	if (building == "house" || building == "detached" || building == "residential" ||
			building == "apartments" || building == "terrace")
		return residential;
	if (building == "church" || building == "cathedral" || building == "mosque" ||
			building == "temple" || building == "synagogue" || building == "chapel" ||
			building == "religious")
		return religious;
	if (building == "tower" || building == "clock_tower")
		return tower;
	return fallback;
}
}
PrescanResult prescan(const std::vector<ProcessedElement> &elements, double rotation,
		double scale, const std::vector<std::pair<std::string, std::uint64_t>> &already)
{
	PrescanResult r;
	if (scale <= 0)
		return r;
	struct FootprintOwner
	{
		Bounds bounds;
		std::uint64_t osm_id;
	};
	std::vector<FootprintOwner> footprints;
	for (const auto &e : elements) {
		auto key = std::make_pair(std::string(e.kind()), e.id());
		if (has(already, key))
			continue;
		auto qid = trim(tag(e, "wikidata"));
		if (qid.empty())
			qid = qid_for_name(tag(e, "name"));
		auto *entry = lookup_wikidata(qid);
		if (!entry)
			continue;
		auto p = points(e);
		if (p.empty())
			continue;
		long long sx = 0, sz = 0;
		int minx = p[0].first, maxx = minx, minz = p[0].second, maxz = minz;
		for (auto [x, z] : p) {
			sx += x;
			sz += z;
			minx = std::min(minx, x);
			maxx = std::max(maxx, x);
			minz = std::min(minz, z);
			maxz = std::max(maxz, z);
		}
		int ax = sx / p.size(), az = sz / p.size();
		bool raw = maxx > minx || maxz > minz;
		Bounds fp = raw ? Bounds{minx, minz, maxx, maxz}
						: Bounds{ax - 8, az - 8, ax + 8, az + 8};
		double yaw = 0;
		direction(tag(e, "direction"), yaw);
		std::optional<double> h =
				entry->height_m ? entry->height_m : meters(tag(e, "height"));
		if (h && *h > 600.0)
			continue;
		std::optional<double> extent;
		if (!entry->height_m) {
			// Match Rust's synthetic footprint for point landmarks.  Using the
			// raw one-point bbox here produces a zero extent and incorrectly
			// suppresses otherwise valid statue/tower models.
			extent = std::max(fp.max_x - fp.min_x, fp.max_z - fp.min_z) / scale;
		}
		if (extent && (*extent > 225.0 || *extent < 2.0))
			continue;
		r.placements.push_back({e.id(), std::string(e.kind()), raw, qid, ax, az, fp,
				yaw + rotation, h, extent, palette_for(e), entry->palette_layers});
		r.suppressed.push_back(key);
		r.suppression_claims.push_back({key, e.id()});
		if (raw)
			footprints.push_back({fp, e.id()});
	}
	for (const auto &e : elements) {
		auto key = std::make_pair(std::string(e.kind()), e.id());
		if (has(already, key) || has(r.suppressed, key) ||
				(tag(e, "building").empty() && tag(e, "building:part").empty()))
			continue;
		auto p = points(e);
		if (p.empty())
			continue;
		long long sx = 0, sz = 0;
		for (auto [x, z] : p) {
			sx += x;
			sz += z;
		}
		int x = sx / p.size(), z = sz / p.size();
		for (const auto &owner : footprints)
			if (owner.bounds.contains(x, z)) {
				r.suppressed.push_back(key);
				r.suppression_claims.push_back({key, owner.osm_id});
				break;
			}
	}
	return r;
}
std::vector<std::pair<int, int>> deferred_regions(const PrescanResult &r, double scale)
{
	std::vector<std::pair<int, int>> o;
	int d = std::ceil(225 * scale);
	for (auto &p : r.placements)
		for (int z = (p.anchor_z - d) >> 9; z <= (p.anchor_z + d) >> 9; ++z)
			for (int x = (p.anchor_x - d) >> 9; x <= (p.anchor_x + d) >> 9; ++x)
				o.push_back({x, z});
	std::sort(o.begin(), o.end());
	o.erase(std::unique(o.begin(), o.end()), o.end());
	return o;
}
void retain_fetchable(
		PrescanResult &r, const std::function<bool(const std::string &)> &fetchable)
{
	if (!fetchable)
		return;
	std::unordered_set<std::uint64_t> kept;
	r.placements.erase(std::remove_if(r.placements.begin(), r.placements.end(),
							   [&](const Placement &p) {
								   if (!fetchable(p.qid))
									   return true;
								   kept.insert(p.osm_id);
								   return false;
							   }),
			r.placements.end());
	r.suppression_claims.erase(
			std::remove_if(r.suppression_claims.begin(), r.suppression_claims.end(),
					[&](const SuppressionClaim &claim) {
						return !kept.contains(claim.owner_osm_id);
					}),
			r.suppression_claims.end());
	r.suppressed.clear();
	for (const auto &claim : r.suppression_claims)
		if (!has(r.suppressed, claim.key))
			r.suppressed.push_back(claim.key);
}
}
