#include "prescan.h"
#include "../../../../arnis_adapter.h"
#include "../palette.h"
#include "../../colors.h"
#include "../../block_definitions.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <unordered_set>
namespace arnis::models_3d::wikidata
{
namespace
{
std::string tag(const ProcessedElement &e, const char *n)
{
	auto i = e.tags().find(n);
	return i == e.tags().end() ? std::string{} : i->second;
}
bool has(const std::vector<std::pair<std::string, std::int64_t>> &v,
		const std::pair<std::string, std::int64_t> &k)
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
	try {
		std::size_t n = 0;
		o = std::stod(s, &n);
		if (n == s.size()) {
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
	std::string u = s;
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
	if (lower == "metal" || lower == "steel" || lower == "iron")
		return closest_blocks(RGBTuple{180, 180, 180}, 5);
	if (tag(e, "man_made") == "lighthouse")
		return {WHITE_CONCRETE, QUARTZ_BLOCK, SMOOTH_QUARTZ, POLISHED_DIORITE};
	if (tag(e, "man_made") == "tower" || tag(e, "historic") == "castle")
		return {STONE_BRICKS, COBBLESTONE, CRACKED_STONE_BRICKS, POLISHED_ANDESITE,
				ANDESITE, DEEPSLATE_BRICKS};
	if (tag(e, "amenity") == "place_of_worship")
		return {STONE_BRICKS, CHISELED_STONE_BRICKS, QUARTZ_BLOCK, WHITE_CONCRETE,
				SANDSTONE};
	if (tag(e, "building") == "industrial" || tag(e, "building") == "warehouse")
		return {GRAY_CONCRETE, LIGHT_GRAY_CONCRETE, STONE, SMOOTH_STONE,
				POLISHED_ANDESITE};
	return {STONE_BRICKS, ANDESITE, POLISHED_ANDESITE, COBBLESTONE, SMOOTH_STONE};
}
}
PrescanResult prescan(const std::vector<ProcessedElement> &elements, double rotation,
		double scale, const std::vector<std::pair<std::string, std::int64_t>> &already)
{
	PrescanResult r;
	if (scale <= 0)
		return r;
	struct FootprintOwner
	{
		Bounds bounds;
		std::int64_t osm_id;
	};
	std::vector<FootprintOwner> footprints;
	for (const auto &e : elements) {
		auto key = std::make_pair(std::string(e.kind()), e.id());
		if (has(already, key))
			continue;
		auto qid = tag(e, "wikidata");
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
		if (!entry->height_m)
			extent = std::max(maxx - minx, maxz - minz) / scale;
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
	std::unordered_set<std::int64_t> kept;
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
