#include "stadiums.h"

#include "../bresenham.h"
#include "../ground_decoration.h"
#include "../land_cover/land_cover.h"
#include "../water_depth.h"
#include "../../../arnis_world_editor.h"
#include "sport_pitches.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <deque>
#include <limits>
#include <map>
#include <numeric>
#include <string_view>
#include <tuple>
#include <iostream>

namespace arnis::element_processing::stadiums
{
namespace
{
constexpr double RAKE = 0.5;
constexpr double UNIT = 5.0;
constexpr double MAX_DEPTH_M = 55.0;
constexpr double MIN_DEPTH_M = 5.0;
constexpr double LIP_M = 1.5;
constexpr double TIER_M = 20.0;
constexpr double TIER_JUMP_M = 3.5;
constexpr double WALKWAY_M = 2.0;
constexpr double SEATS_CAP_M = 48.0;
constexpr double ROOF_CLEARANCE_M = 5.0;
constexpr double MAST_ABOVE_M = 18.0;
constexpr double GRANDSTAND_REACH_M = 60.0;
constexpr double AISLE_SPACING = 22.0;
constexpr int TREE_SWEEP_REACH = 10;
const std::array<std::pair<Block, Block>, 6> SEATS{
		{{RED_CONCRETE, WHITE_CONCRETE}, {BLUE_CONCRETE, WHITE_CONCRETE},
				{LIGHT_BLUE_CONCRETE, WHITE_CONCRETE}, {YELLOW_CONCRETE, BLACK_CONCRETE},
				{CYAN_CONCRETE, WHITE_CONCRETE}, {ORANGE_CONCRETE, BLACK_CONCRETE}}};
const std::array<std::pair<Block, Block>, 3> FACADES{
		{{LIGHT_GRAY_CONCRETE, WHITE_CONCRETE}, {WHITE_CONCRETE, LIGHT_GRAY_CONCRETE},
				{POLISHED_ANDESITE, LIGHT_GRAY_CONCRETE}}};

struct Candidate
{
	const ProcessedElement *element = nullptr;
	Shape shape;
	Key key() const { return {element->kind(), element->id()}; }
	const tags_t &tags() const { return element->tags(); }
};

struct Planned
{
	Venue venue;
	std::vector<Key> suppress, taken;
};

std::optional<std::string_view> tag(const tags_t &tags, std::string_view key)
{
	const auto it = tags.find(std::string(key));
	return it == tags.end() ? std::nullopt : std::optional<std::string_view>(it->second);
}

std::string_view trim(std::string_view value)
{
	while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())))
		value.remove_prefix(1);
	while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())))
		value.remove_suffix(1);
	return value;
}

double ring_area(const std::vector<std::pair<int, int>> &ring)
{
	double twice = 0.0;
	for (std::size_t i = 1; i < ring.size(); ++i)
		twice += static_cast<double>(ring[i - 1].first) * ring[i].second -
				 ring[i].first * ring[i - 1].second;
	return std::abs(twice * 0.5);
}

template <typename F>
void for_edges(const Shape &shape, F &&f)
{
	for (const auto &ring : shape.rings)
		for (std::size_t i = 1; i < ring.size(); ++i)
			f(ring[i - 1], ring[i]);
}

bool contains_key(const std::set<Key> &keys, const Key &key)
{
	return keys.find(key) != keys.end();
}

bool tags_are_plain_building(const tags_t &tags)
{
	const auto it = tags.find("building");
	return it != tags.end() && it->second != "no" && it->second != "roof" &&
		   it->second != "stadium" && it->second != "grandstand" &&
		   it->second != "bleachers" && it->second != "tribune" &&
		   !tags.contains("building:part");
}

std::optional<Bbox> quick_bbox(const ProcessedElement &element)
{
	std::optional<Bbox> bounds;
	auto add = [&](int x, int z) {
		const Bbox point{x, z, x, z};
		bounds = bounds ? bounds->united(point) : point;
	};
	if (element.is_way()) {
		for (const auto &node : element.as_way().nodes)
			add(node.x, node.z);
	} else if (element.is_relation()) {
		for (const auto &member : element.as_relation().members)
			if (member.way)
				for (const auto &node : member.way->nodes)
					add(node.x, node.z);
	}
	return bounds;
}

double bbox_distance(const Bbox &b, int x, int z)
{
	const double dx = std::max({b.min_x - x, 0, x - b.max_x});
	const double dz = std::max({b.min_z - z, 0, z - b.max_z});
	return std::hypot(dx, dz);
}

bool on_a_building(const Candidate &stand, const std::vector<Candidate> &others)
{
	const auto [x, z] = stand.shape.centroid;
	for (const auto &other : others) {
		if (!tags_are_plain_building(other.tags()) || !other.shape.bbox.contains(x, z))
			continue;
		const auto local = Grid::from_bbox(other.shape.bbox);
		if (!local.size() || local.size() > MAX_GRID_CELLS)
			continue;
		std::vector<bool> mask(local.size(), false);
		rasterize(local, other.shape, mask);
		const auto index = local.index(x, z);
		if (index && mask[*index])
			return true;
	}
	return false;
}

const Candidate *main_field(const std::vector<const Candidate *> &fields)
{
	const Candidate *pitch = nullptr, *track = nullptr;
	for (const auto *field : fields) {
		const auto leisure = tag(field->tags(), "leisure");
		auto **selected = leisure == "pitch"   ? &pitch
						  : leisure == "track" ? &track
											   : nullptr;
		// Rust's Iterator::max_by returns the last item when the ordering is equal.
		if (selected && (!*selected || field->shape.area >= (*selected)->shape.area))
			*selected = field;
	}
	return pitch ? pitch : track;
}

std::vector<const Candidate *> field_group(const std::vector<const Candidate *> &fields)
{
	const auto *main = main_field(fields);
	if (!main)
		return {};
	std::vector<const Candidate *> group;
	for (const auto *field : fields)
		if (field->key() == main->key() ||
				field->shape.bbox.contains(
						main->shape.centroid.first, main->shape.centroid.second) ||
				main->shape.bbox.contains(
						field->shape.centroid.first, field->shape.centroid.second))
			group.push_back(field);
	return group;
}

Palette palette_for(std::uint64_t id, const tags_t &tags)
{
	const auto h = arnis::land_cover::coord_hash(
			static_cast<int>(id), static_cast<int>(id >> 32));
	const auto [seat, accent] = SEATS[h % SEATS.size()];
	const bool brick = tag(tags, "building:material") == "brick" ||
					   tag(tags, "building:material") == "bricks";
	const auto [wall, rib] = brick ? std::pair{BRICK, LIGHT_GRAY_CONCRETE}
								   : FACADES[(h >> 8) % FACADES.size()];
	return {seat, accent, wall, rib};
}

std::pair<std::vector<bool>, std::vector<bool>> edges(const Grid &grid,
		const std::vector<bool> &stand, const std::vector<bool> &field,
		const std::vector<std::uint16_t> &field_dist)
{
	std::vector<bool> front(grid.size(), false), back(grid.size(), false);
	if (stand.size() != grid.size() || field.size() != grid.size() ||
			field_dist.size() != grid.size())
		return {std::move(front), std::move(back)};
	for (std::size_t i = 0; i < grid.size(); ++i) {
		if (!stand[i])
			continue;
		const auto [x, z] = grid.coordinate(i);
		for (const auto &[dx, dz] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
			const auto neighbor = grid.index(x + dx, z + dz);
			if (!neighbor) {
				back[i] = true;
			} else if (stand[*neighbor]) {
				continue;
			} else if (field[*neighbor] || field_dist[*neighbor] + 3 <= field_dist[i]) {
				front[i] = true;
			} else if (field_dist[*neighbor] >= field_dist[i] + 3) {
				back[i] = true;
			}
		}
	}
	return {std::move(front), std::move(back)};
}

struct ProfileInput
{
	const tags_t &tags;
	const std::vector<const Candidate *> &stand_elems;
	std::optional<Sport> sport;
	double scale;
};

double scaled(double scale, double metres)
{
	return scale * metres;
}

std::optional<Venue> build_venue(const Key &key, const tags_t &tags, const Grid &grid,
		std::vector<bool> stand, const std::vector<bool> &field,
		const std::vector<std::uint16_t> &field_dist,
		std::optional<SynthField> synth_field, const std::vector<Candidate> &others,
		const std::set<Key> &claimed, const ProfileInput &input)
{
	auto [front_seed, unused_back] = edges(grid, stand, field, field_dist);
	(void)unused_back;
	const auto front = chamfer(grid, front_seed);
	const auto max_front =
			static_cast<std::uint16_t>(scaled(input.scale, MAX_DEPTH_M) * UNIT);
	for (std::size_t i = 0; i < stand.size(); ++i)
		if (stand[i] && front[i] > max_front)
			stand[i] = false;
	auto [unused_front, back_seed] = edges(grid, stand, field, field_dist);
	(void)unused_front;
	const auto back = chamfer(grid, back_seed);
	const auto stand_count =
			static_cast<std::size_t>(std::count(stand.begin(), stand.end(), true));
	if (stand_count < 24 && !synth_field)
		return std::nullopt;
	std::vector<std::pair<int, int>> field_cells;
	for (std::size_t i = 0; i < field.size(); ++i)
		if (field[i])
			field_cells.push_back(grid.coordinate(i));
	const auto field_frame = frame_of(field_cells);
	const auto bounds = grid.bbox();
	const std::pair<double, double> centre =
			field_frame ? std::pair{field_frame->cx, field_frame->cz}
						: std::pair{(bounds.min_x + bounds.max_x) * 0.5,
								  (bounds.min_z + bounds.max_z) * 0.5};
	std::vector<std::uint32_t> depths;
	depths.reserve(stand_count);
	for (std::size_t i = 0; i < stand.size(); ++i)
		if (stand[i])
			depths.push_back(static_cast<std::uint32_t>(front[i]) + back[i]);
	std::sort(depths.begin(), depths.end());
	const double median_depth = depths.empty() ? 0.0 : depths[depths.size() / 2] / UNIT;
	const double tier_len = std::max(scaled(input.scale, TIER_M), 6.0);
	const std::uint8_t tiers = median_depth < tier_len * 1.5   ? 1
							   : median_depth < tier_len * 2.6 ? 2
															   : 3;
	auto blocks = [&](double m) {
		return std::max(1, static_cast<int>(std::lround(scaled(input.scale, m))));
	};
	std::optional<double> tag_top;
	auto consider_height = [&](const tags_t &source) {
		if (const auto h = metres(source, "height")) {
			const double scaled_h = scaled(input.scale, *h);
			if (scaled_h >= scaled(input.scale, 4.0) && scaled_h >= median_depth * 0.3)
				tag_top = tag_top ? std::max(*tag_top, scaled_h) : scaled_h;
		}
	};
	consider_height(input.tags);
	for (const auto *s : input.stand_elems)
		consider_height(s->tags());
	int cap = blocks(SEATS_CAP_M);
	if (tag_top)
		cap = std::min(cap, static_cast<int>(std::lround(*tag_top)));
	Profile profile{blocks(LIP_M), static_cast<float>(tier_len),
			static_cast<float>(std::max(scaled(input.scale, WALKWAY_M), 1.0)),
			std::max(blocks(TIER_JUMP_M), 2), tiers, std::max(cap, 2)};
	const int seats_top =
			depths.empty()
					? profile.lip
					: std::get<0>(profile.at(depths.back() / static_cast<float>(UNIT)));
	std::vector<bool> roof_mask(grid.size(), false);
	for (const auto &o : others)
		if (!contains_key(claimed, o.key()) && is_roofish(o.tags()) &&
				grid.bbox().contains(o.shape.centroid.first, o.shape.centroid.second))
			rasterize(grid, o.shape, roof_mask);
	std::size_t roofed_cells = 0;
	for (std::size_t i = 0; i < stand.size(); ++i)
		roofed_cells += stand[i] && roof_mask[i];
	const float mapped_cover = static_cast<float>(roofed_cells) /
							   static_cast<float>(std::max<std::size_t>(stand_count, 1));
	auto covered = [](const tags_t &t) {
		return tag(t, "covered") == "yes" || tag(t, "roof") == "yes" ||
			   (tag(t, "roof:shape") && tag(t, "roof:shape") != "flat" &&
					   tag(t, "roof:shape") != "no");
	};
	const auto roof_top =
			tag_top ? std::optional<int>(static_cast<int>(std::lround(*tag_top)))
					: std::nullopt;
	const auto feasible_roof =
			roof_top && *roof_top >= seats_top + blocks(6.0) ? roof_top : std::nullopt;
	float roof_cover = 0;
	if (feasible_roof)
		roof_cover = 0.95f;
	else if (mapped_cover >= 0.08f)
		roof_cover = std::min(mapped_cover * 1.2f, 0.95f);
	else if (covered(input.tags) ||
			 std::any_of(input.stand_elems.begin(), input.stand_elems.end(),
					 [&](const auto *s) { return covered(s->tags()); }))
		roof_cover = 0.75f;
	else if ((!input.sport || usually_roofed(*input.sport)) && seats_top >= blocks(14.0))
		roof_cover = 0.65f;
	const float roof_min_depth = static_cast<float>(
			std::max(depths.empty() ? 0.0 : depths[depths.size() * 3 / 4] / UNIT * 0.5,
					scaled(input.scale, 12.0)));
	std::vector<double> front_radii;
	for (std::size_t i = 0; i < front_seed.size(); ++i) {
		if (!front_seed[i] || !stand[i])
			continue;
		const auto [x, z] = grid.coordinate(i);
		front_radii.push_back(std::hypot(x - centre.first, z - centre.second));
	}
	const double front_radius = front_radii.empty() ? 1.0
													: std::accumulate(front_radii.begin(),
															  front_radii.end(), 0.0) /
															  front_radii.size();
	Venue venue;
	if (const auto it = tags.find("name"); it != tags.end())
		venue.name = it->second;
	venue.grid = grid;
	venue.stand = std::move(stand);
	venue.field = field;
	venue.front = front;
	venue.back = back;
	venue.centre = centre;
	venue.field_frame = field_frame;
	venue.synth_field = std::move(synth_field);
	venue.profile = profile;
	venue.roof_cover = roof_cover;
	venue.roof_top = feasible_roof;
	venue.roof_min_depth = roof_min_depth;
	venue.roof_clearance = std::max(blocks(ROOF_CLEARANCE_M), 3);
	venue.mast_above = std::max(blocks(MAST_ABOVE_M), 8);
	venue.palette = palette_for(key.second, tags);
	venue.front_radius = std::max(front_radius, 1.0);
	return venue;
}

double venue_share(const Venue &venue, const Shape &shape)
{
	const auto local = Grid::from_bbox(shape.bbox);
	if (!local.size() || local.size() > MAX_GRID_CELLS)
		return 0.0;
	std::vector<bool> mask(local.size(), false);
	rasterize(local, shape, mask);
	std::size_t on = 0, all = 0;
	for (std::size_t i = 0; i < mask.size(); ++i) {
		if (!mask[i])
			continue;
		++all;
		const auto [x, z] = local.coordinate(i);
		if (const auto j = venue.grid.index(x, z);
				j && (venue.stand[*j] || venue.field[*j]))
			++on;
	}
	return all ? static_cast<double>(on) / all : 0.0;
}

std::vector<Key> covered_buildings(const Venue &venue,
		const std::vector<Candidate> &others, const std::set<Key> &claimed)
{
	std::vector<Key> result;
	const auto area = venue.grid.bbox();
	for (const auto &other : others) {
		if (contains_key(claimed, other.key()))
			continue;
		const auto &b = other.shape.bbox;
		if (b.max_x < area.min_x || b.min_x > area.max_x || b.max_z < area.min_z ||
				b.min_z > area.max_z)
			continue;
		if (venue_share(venue, other.shape) >= 0.3)
			result.push_back(other.key());
	}
	return result;
}

std::optional<Planned> plan_venue(const Candidate &outline,
		const std::vector<Candidate> &stands, const std::vector<Candidate> &fields,
		const std::vector<Candidate> &others, const std::set<Key> &claimed, double scale)
{
	const auto &tags = outline.tags();
	const auto building = tag(tags, "building");
	const bool outline_is_stand =
			building && (*building == "stadium" || *building == "grandstand");
	if (building && *building != "no" && !outline_is_stand)
		return std::nullopt;
	const bool hall_tags = (tag(tags, "roof:shape") && tag(tags, "roof:shape") != "no") ||
						   tags.contains("building:levels");
	if (hall_tags && !outline.shape.has_hole)
		return std::nullopt;
	const auto probe = Grid::from_bbox(outline.shape.bbox.padded(2));
	if (!probe.size() || probe.size() > MAX_GRID_CELLS)
		return std::nullopt;
	std::vector<bool> inside(probe.size(), false);
	rasterize(probe, outline.shape.filled(), inside);
	auto within = [&](const Candidate &c) {
		const auto i = probe.index(c.shape.centroid.first, c.shape.centroid.second);
		return i && inside[*i];
	};
	for (const auto &field : fields)
		if (contains_key(claimed, field.key()) && within(field))
			return std::nullopt;
	std::vector<const Candidate *> inner_fields, inner_stands;
	for (const auto &field : fields)
		if (within(field))
			inner_fields.push_back(&field);
	for (const auto &stand : stands)
		if (stand.key() != outline.key() && !contains_key(claimed, stand.key()) &&
				within(stand))
			inner_stands.push_back(&stand);
	auto sport = sport_of(tags);
	if (!sport)
		if (const auto *field = main_field(inner_fields))
			sport = sport_of(field->tags());
	const bool has_field = !inner_fields.empty();
	if (outline_is_stand && !has_field && !outline.shape.has_hole)
		return std::nullopt;
	if (sport == Sport::Indoor && !has_field)
		return std::nullopt;
	if (is_indoor(tags) && !has_field)
		return std::nullopt;
	auto field_set = field_group(inner_fields);
	auto stand_elems = inner_stands;
	if (outline_is_stand)
		stand_elems.push_back(&outline);
	Bbox bounds = outline.shape.bbox;
	for (const auto *c : stand_elems)
		bounds = bounds.united(c->shape.bbox);
	for (const auto *c : field_set)
		bounds = bounds.united(c->shape.bbox);
	const auto grid = Grid::from_bbox(bounds.padded(4));
	if (!grid.size() || grid.size() > MAX_GRID_CELLS)
		return std::nullopt;
	std::vector<bool> venue_mask(grid.size(), false), field(grid.size(), false),
			stand(grid.size(), false), keep_out(grid.size(), false);
	rasterize(grid, outline.shape.filled(), venue_mask);
	for (const auto *f : field_set)
		rasterize(grid, f->shape, field);
	for (const auto *s : stand_elems)
		rasterize(grid, s->shape, stand);
	const bool mapped_stands =
			std::any_of(stand.begin(), stand.end(), [](bool v) { return v; });
	for (const auto &o : others)
		if (within(o) && !contains_key(claimed, o.key()) &&
				tags_are_plain_building(o.tags()))
			rasterize(grid, o.shape, keep_out);
	const auto venue_cells = static_cast<std::size_t>(
			std::count(venue_mask.begin(), venue_mask.end(), true));
	std::size_t roofed_over = 0;
	for (std::size_t i = 0; i < grid.size(); ++i)
		if (venue_mask[i] && keep_out[i])
			++roofed_over;
	if (roofed_over * 2 >= venue_cells && stand_elems.empty())
		return std::nullopt;
	std::optional<SynthField> synth_field;
	if (std::none_of(field.begin(), field.end(), [](bool v) { return v; })) {
		std::vector<bool> hole;
		if (mapped_stands)
			hole = enclosed(grid, stand);
		const auto hole_cells =
				static_cast<std::size_t>(std::count(hole.begin(), hole.end(), true));
		if (hole_cells >= scaled(scale, 30.0) * scaled(scale, 20.0)) {
			field = std::move(hole);
		} else {
			synth_field = synthesize_field(
					grid, venue_mask, sport, outline.key().second, scale);
			if (synth_field)
				for (const auto &[x, z] : synth_field->cells)
					if (const auto i = grid.index(x, z))
						field[*i] = true;
		}
	}
	if (std::none_of(field.begin(), field.end(), [](bool v) { return v; }))
		return std::nullopt;
	const auto field_dist = chamfer(grid, field);
	const Sport resolved_sport = sport.value_or(Sport::Soccer);
	const auto apron =
			static_cast<std::uint32_t>(scaled(scale, apron_m(resolved_sport)) * UNIT);
	if (!mapped_stands) {
		const auto max_d =
				static_cast<std::uint32_t>(scaled(scale, MAX_DEPTH_M) * UNIT) + apron;
		for (std::size_t i = 0; i < grid.size(); ++i) {
			const auto d = static_cast<std::uint32_t>(field_dist[i]);
			stand[i] = venue_mask[i] && !field[i] && !keep_out[i] && d >= apron &&
					   d <= max_d;
		}
		thin_out(grid, stand,
				static_cast<std::uint16_t>(scaled(scale, MIN_DEPTH_M) * UNIT));
	} else {
		close_gaps(grid, stand, venue_mask, 4.0 * scale);
		for (std::size_t i = 0; i < grid.size(); ++i)
			if (field[i] || field_dist[i] < 10)
				stand[i] = false;
	}
	ProfileInput input{tags, stand_elems, sport, scale};
	auto venue = build_venue(outline.key(), tags, grid, std::move(stand), field,
			field_dist, std::move(synth_field), others, claimed, input);
	if (!venue)
		return std::nullopt;
	Planned result{std::move(*venue), {}, {}};
	for (const auto *s : stand_elems)
		result.suppress.push_back(s->key());
	if (outline_is_stand)
		result.suppress.push_back(outline.key());
	auto covered = covered_buildings(result.venue, others, claimed);
	result.suppress.insert(result.suppress.end(), covered.begin(), covered.end());
	std::sort(result.suppress.begin(), result.suppress.end());
	result.suppress.erase(std::unique(result.suppress.begin(), result.suppress.end()),
			result.suppress.end());
	for (const auto *f : field_set)
		result.taken.push_back(f->key());
	result.taken.push_back(outline.key());
	return result;
}

std::optional<Planned> plan_grandstands(const Candidate &field,
		const std::vector<const Candidate *> &members,
		const std::vector<Candidate> &others, const std::set<Key> &claimed, double scale)
{
	if (members.empty())
		return std::nullopt;
	Bbox bounds = field.shape.bbox;
	for (const auto *s : members)
		bounds = bounds.united(s->shape.bbox);
	const auto grid = Grid::from_bbox(bounds.padded(4));
	if (!grid.size() || grid.size() > MAX_GRID_CELLS)
		return std::nullopt;
	std::vector<bool> field_mask(grid.size(), false), stand(grid.size(), false);
	rasterize(grid, field.shape, field_mask);
	for (const auto *s : members)
		rasterize(grid, s->shape, stand);
	const auto field_dist = chamfer(grid, field_mask);
	for (std::size_t i = 0; i < stand.size(); ++i)
		if (field_mask[i])
			stand[i] = false;
	const auto &tags = members.front()->tags();
	std::optional<Sport> sport = sport_of(field.tags());
	ProfileInput input{tags, members, sport, scale};
	auto venue = build_venue(members.front()->key(), tags, grid, std::move(stand),
			field_mask, field_dist, std::nullopt, others, claimed, input);
	if (!venue)
		return std::nullopt;
	Planned result{std::move(*venue), {}, {field.key()}};
	for (const auto *s : members)
		result.suppress.push_back(s->key());
	auto covered = covered_buildings(result.venue, others, claimed);
	result.suppress.insert(result.suppress.end(), covered.begin(), covered.end());
	std::sort(result.suppress.begin(), result.suppress.end());
	result.suppress.erase(std::unique(result.suppress.begin(), result.suppress.end()),
			result.suppress.end());
	return result;
}

std::optional<Candidate> make_candidate(
		const ProcessedElement &element, const std::set<Key> &already)
{
	if (element.is_node() || contains_key(already, {element.kind(), element.id()}))
		return std::nullopt;
	if (auto shape = shape_of(element))
		return Candidate{&element, std::move(*shape)};
	return std::nullopt;
}

int coarse_cell(int coordinate)
{
	const auto value = static_cast<std::int64_t>(coordinate);
	return static_cast<int>(value >= 0 ? value / 128 : -((-value + 127) / 128));
}

int region_cell(int coordinate)
{
	const auto value = static_cast<std::int64_t>(coordinate);
	return static_cast<int>(value >= 0 ? value / 512 : -((-value + 511) / 512));
}

std::set<std::pair<int, int>> coarse_cells(const Bbox &b)
{
	std::set<std::pair<int, int>> cells;
	for (int x = coarse_cell(b.min_x); x <= coarse_cell(b.max_x); ++x)
		for (int z = coarse_cell(b.min_z); z <= coarse_cell(b.max_z); ++z)
			cells.emplace(x, z);
	return cells;
}

void take_plan(StadiumPlan &plan, Planned &&planned, std::set<Key> &claimed)
{
	claimed.insert(planned.suppress.begin(), planned.suppress.end());
	claimed.insert(planned.taken.begin(), planned.taken.end());
	plan.suppressed.insert(planned.suppress.begin(), planned.suppress.end());
	plan.venues.push_back(std::move(planned.venue));
}
} // namespace

Bbox Bbox::united(const Bbox &o) const
{
	return {std::min(min_x, o.min_x), std::min(min_z, o.min_z), std::max(max_x, o.max_x),
			std::max(max_z, o.max_z)};
}

Bbox Bbox::padded(int p) const
{
	return {min_x - p, min_z - p, max_x + p, max_z + p};
}

bool Bbox::contains(int x, int z) const
{
	return x >= min_x && x <= max_x && z >= min_z && z <= max_z;
}

Shape Shape::filled() const
{
	Shape out = *this;
	out.rings.clear();
	out.outer.clear();
	for (std::size_t i = 0; i < rings.size(); ++i)
		if (i < outer.size() && outer[i]) {
			out.rings.push_back(rings[i]);
			out.outer.push_back(true);
		}
	out.has_hole = false;
	return out;
}

Grid Grid::from_bbox(const Bbox &b)
{
	const auto width =
			std::max<std::int64_t>(1, static_cast<std::int64_t>(b.max_x) - b.min_x + 1);
	const auto height =
			std::max<std::int64_t>(1, static_cast<std::int64_t>(b.max_z) - b.min_z + 1);
	return {b.min_x, b.min_z, static_cast<std::size_t>(width),
			static_cast<std::size_t>(height)};
}

std::size_t Grid::size() const
{
	if (h && w > std::numeric_limits<std::size_t>::max() / h)
		return 0;
	return w * h;
}

std::optional<std::size_t> Grid::index(int x, int z) const
{
	const auto dx = static_cast<std::int64_t>(x) - x0;
	const auto dz = static_cast<std::int64_t>(z) - z0;
	if (dx < 0 || dz < 0 || static_cast<std::uint64_t>(dx) >= w ||
			static_cast<std::uint64_t>(dz) >= h)
		return std::nullopt;
	return static_cast<std::size_t>(dz) * w + static_cast<std::size_t>(dx);
}

std::pair<int, int> Grid::coordinate(std::size_t i) const
{
	return {x0 + static_cast<int>(i % w), z0 + static_cast<int>(i / w)};
}

Bbox Grid::bbox() const
{
	return {x0, z0, x0 + static_cast<int>(w) - 1, z0 + static_cast<int>(h) - 1};
}

std::optional<Sport> sport_of(const tags_t &tags)
{
	const auto value = tag(tags, "sport");
	if (!value)
		return std::nullopt;
	std::optional<Sport> named;
	std::size_t begin = 0;
	while (begin <= value->size()) {
		const auto sep = value->find(';', begin);
		const auto end = sep == std::string_view::npos ? value->size() : sep;
		const auto s = trim(value->substr(begin, end - begin));
		if (!named && (s == "soccer" || s == "association_football"))
			named = Sport::Soccer;
		else if (!named && (s == "american_football" || s == "canadian_football"))
			named = Sport::AmericanFootball;
		else if (!named && (s == "rugby" || s == "rugby_union" || s == "rugby_league"))
			named = Sport::Rugby;
		else if (!named && (s == "baseball" || s == "softball"))
			named = Sport::Baseball;
		else if (!named && s == "cricket")
			named = Sport::Cricket;
		else if (!named && (s == "athletics" || s == "running"))
			named = Sport::Athletics;
		else if (!named && (s == "australian_football" || s == "gaelic_games" ||
								   s == "gaelic_football" || s == "hurling"))
			named = Sport::Oval;
		else if (!named &&
				 (s == "basketball" || s == "ice_hockey" || s == "handball" ||
						 s == "volleyball" || s == "boxing" || s == "futsal" ||
						 s == "gymnastics" || s == "badminton" || s == "table_tennis" ||
						 s == "wrestling" || s == "martial_arts" || s == "ice_skating" ||
						 s == "swimming" || s == "fencing" || s == "judo" ||
						 s == "squash" || s == "bowling" || s == "10pin" ||
						 s == "netball" || s == "esports" || s == "darts" ||
						 s == "billiards"))
			named = Sport::Indoor;
		if (sep == std::string_view::npos)
			break;
		begin = sep + 1;
	}
	if (named)
		return named;
	begin = 0;
	while (begin <= value->size()) {
		const auto sep = value->find(';', begin);
		const auto end = sep == std::string_view::npos ? value->size() : sep;
		if (trim(value->substr(begin, end - begin)) == "football")
			return Sport::Soccer;
		if (sep == std::string_view::npos)
			break;
		begin = sep + 1;
	}
	return std::nullopt;
}

double apron_m(Sport sport)
{
	switch (sport) {
	case Sport::Soccer:
	case Sport::Rugby:
		return 6.0;
	case Sport::AmericanFootball:
		return 8.0;
	case Sport::Baseball:
	case Sport::Athletics:
		return 2.0;
	case Sport::Cricket:
	case Sport::Oval:
	case Sport::Indoor:
		return 4.0;
	}
	return 4.0;
}

std::optional<std::pair<double, double>> field_m(Sport sport)
{
	switch (sport) {
	case Sport::Soccer:
	case Sport::Athletics:
		return std::pair{105.0, 68.0};
	case Sport::AmericanFootball:
		return std::pair{110.0, 49.0};
	case Sport::Rugby:
		return std::pair{120.0, 70.0};
	case Sport::Baseball:
	case Sport::Cricket:
	case Sport::Oval:
	case Sport::Indoor:
		return std::nullopt;
	}
	return std::nullopt;
}

const char *pitch_tag(Sport sport)
{
	if (sport == Sport::AmericanFootball)
		return "american_football";
	if (sport == Sport::Rugby)
		return "rugby";
	return "soccer";
}

bool usually_roofed(Sport sport)
{
	return sport == Sport::Soccer || sport == Sport::Rugby;
}

bool is_venue(const tags_t &tags)
{
	return tag(tags, "leisure") == "stadium";
}

bool is_stand(const tags_t &tags)
{
	const auto building = tag(tags, "building");
	const auto part = tag(tags, "building:part");
	const auto is_stand_type = [](std::optional<std::string_view> value) {
		return value == "grandstand" || value == "stadium" || value == "bleachers" ||
			   value == "tribune";
	};
	return is_stand_type(building) || is_stand_type(part) || part == "stand" ||
		   tag(tags, "leisure") == "bleachers";
}

bool is_indoor(const tags_t &tags)
{
	return tag(tags, "indoor") == "yes" || tag(tags, "covered") == "yes" ||
		   tag(tags, "location") == "indoor";
}

bool is_field(const tags_t &tags)
{
	const auto leisure = tag(tags, "leisure");
	return (leisure == "pitch" || leisure == "track") && !is_indoor(tags);
}

bool is_building(const tags_t &tags)
{
	const auto building = tag(tags, "building");
	const auto part = tag(tags, "building:part");
	return (building && *building != "no") || (part && *part != "no");
}

std::optional<double> metres(const tags_t &tags, const std::string &key)
{
	const auto it = tags.find(key);
	if (it == tags.end())
		return std::nullopt;
	std::string_view v(it->second);
	while (!v.empty() && std::isspace(static_cast<unsigned char>(v.front())))
		v.remove_prefix(1);
	while (!v.empty() && std::isspace(static_cast<unsigned char>(v.back())))
		v.remove_suffix(1);
	while (!v.empty() && v.back() == 'm')
		v.remove_suffix(1);
	while (!v.empty() && std::isspace(static_cast<unsigned char>(v.back())))
		v.remove_suffix(1);
	if (v.empty())
		return std::nullopt;
	std::string copy(v);
	char *end = nullptr;
	const double value = std::strtod(copy.c_str(), &end);
	if (end != copy.c_str() + copy.size() || !std::isfinite(value) || value <= 0.0)
		return std::nullopt;
	return value;
}

bool is_roofish(const tags_t &tags)
{
	if (tag(tags, "building") == "roof" || tag(tags, "building:part") == "roof")
		return true;
	const auto min = metres(tags, "min_height"), top = metres(tags, "height");
	return tags.contains("building:part") && min && top && *min >= 4.0 &&
		   *top - *min <= 3.0;
}

std::optional<Shape> shape_of(const ProcessedElement &element)
{
	std::vector<std::pair<std::vector<std::pair<int, int>>, bool>> rings;
	if (element.is_way()) {
		const auto &way = element.as_way();
		std::vector<std::pair<int, int>> ring;
		for (const auto &n : way.nodes)
			ring.emplace_back(n.x, n.z);
		if (ring.size() >= 3 && ring.front() != ring.back())
			ring.push_back(ring.front());
		rings.emplace_back(std::move(ring), true);
	} else if (element.is_relation()) {
		for (const auto &member : element.as_relation().members) {
			if (!member.way || member.role == ProcessedMemberRole::Part)
				continue;
			std::vector<std::pair<int, int>> ring;
			for (const auto &n : member.way->nodes)
				ring.emplace_back(n.x, n.z);
			rings.emplace_back(
					std::move(ring), member.role == ProcessedMemberRole::Outer);
		}
	} else {
		return std::nullopt;
	}
	rings.erase(std::remove_if(rings.begin(), rings.end(),
						[](const auto &r) { return r.first.size() < 2; }),
			rings.end());
	if (rings.empty())
		return std::nullopt;
	std::int64_t sx = 0, sz = 0, count = 0;
	std::optional<Bbox> bounds;
	for (const auto &[ring, outer] : rings) {
		if (!outer)
			continue;
		for (const auto &[x, z] : ring) {
			sx += x;
			sz += z;
			++count;
			const Bbox point{x, z, x, z};
			bounds = bounds ? bounds->united(point) : point;
		}
	}
	if (!bounds || count == 0)
		return std::nullopt;
	Shape shape;
	shape.bbox = *bounds;
	shape.centroid = {static_cast<int>(sx / count), static_cast<int>(sz / count)};
	for (auto &entry : rings) {
		shape.outer.push_back(entry.second);
		shape.rings.push_back(std::move(entry.first));
		shape.has_hole |= !entry.second;
	}
	const bool closed = std::all_of(shape.rings.begin(), shape.rings.end(),
			[](const auto &r) { return !r.empty() && r.front() == r.back(); });
	if (closed) {
		for (std::size_t i = 0; i < shape.rings.size(); ++i)
			shape.area += ring_area(shape.rings[i]) * (shape.outer[i] ? 1.0 : -1.0);
		shape.area = std::abs(shape.area);
	} else {
		const auto grid = Grid::from_bbox(shape.bbox);
		if (grid.size() > MAX_GRID_CELLS)
			return std::nullopt;
		std::vector<bool> mask(grid.size(), false);
		rasterize(grid, shape, mask);
		shape.area = static_cast<double>(std::count(mask.begin(), mask.end(), true));
	}
	return shape;
}

void rasterize(const Grid &grid, const Shape &shape, std::vector<bool> &out)
{
	if (out.size() != grid.size())
		return;
	const auto bounds = shape.bbox;
	const int z0 = std::max(bounds.min_z, grid.z0);
	const int z1 = std::min(bounds.max_z, grid.z0 + static_cast<int>(grid.h) - 1);
	for (int z = z0; z <= z1; ++z) {
		const double scan = z + 1e-4;
		std::vector<double> crossings;
		for_edges(shape, [&](const auto &a, const auto &b) {
			if ((a.second <= scan) != (b.second <= scan))
				crossings.push_back(
						a.first +
						(scan - a.second) * (static_cast<double>(b.first - a.first) /
													(b.second - a.second)));
		});
		std::sort(crossings.begin(), crossings.end());
		for (std::size_t k = 0; k + 1 < crossings.size(); k += 2) {
			const int x0 = static_cast<int>(std::ceil(crossings[k]));
			const int x1 = static_cast<int>(std::floor(crossings[k + 1]));
			for (int x = x0; x <= x1; ++x)
				if (const auto i = grid.index(x, z))
					out[*i] = true;
		}
	}
	for_edges(shape, [&](const auto &a, const auto &b) {
		for (const auto &[x, y, z] :
				bresenham::bresenham_line(a.first, 0, a.second, b.first, 0, b.second)) {
			(void)y;
			if (const auto i = grid.index(x, z))
				out[*i] = true;
		}
	});
}

template <typename SeedPredicate>
std::vector<std::uint16_t> chamfer_with_seed(const Grid &grid, SeedPredicate is_seed)
{
	const auto n = grid.size();
	std::vector<std::uint16_t> d(n, std::numeric_limits<std::uint16_t>::max());
	for (std::size_t i = 0; i < n; ++i)
		if (is_seed(i))
			d[i] = 0;
	auto relax = [&](std::size_t i, std::size_t j, std::uint16_t cost) {
		d[i] = std::min(d[i], static_cast<std::uint16_t>(std::min<unsigned>(
									  std::numeric_limits<std::uint16_t>::max(),
									  static_cast<unsigned>(d[j]) + cost)));
	};
	for (std::size_t z = 0; z < grid.h; ++z)
		for (std::size_t x = 0; x < grid.w; ++x) {
			const auto i = z * grid.w + x;
			if (x)
				relax(i, i - 1, 5);
			if (z) {
				relax(i, i - grid.w, 5);
				if (x)
					relax(i, i - grid.w - 1, 7);
				if (x + 1 < grid.w)
					relax(i, i - grid.w + 1, 7);
			}
		}
	for (std::size_t z = grid.h; z-- > 0;)
		for (std::size_t x = grid.w; x-- > 0;) {
			const auto i = z * grid.w + x;
			if (x + 1 < grid.w)
				relax(i, i + 1, 5);
			if (z + 1 < grid.h) {
				relax(i, i + grid.w, 5);
				if (x + 1 < grid.w)
					relax(i, i + grid.w + 1, 7);
				if (x)
					relax(i, i + grid.w - 1, 7);
			}
		}
	return d;
}

std::vector<std::uint16_t> chamfer(const Grid &grid, const std::vector<bool> &seed)
{
	if (seed.size() != grid.size())
		return {};
	return chamfer_with_seed(grid, [&seed](std::size_t i) { return seed[i]; });
}

template <typename SeedPredicate>
std::vector<std::uint16_t> chamfer(const Grid &grid, SeedPredicate is_seed)
{
	return chamfer_with_seed(grid, is_seed);
}

std::vector<bool> enclosed(const Grid &grid, const std::vector<bool> &mask)
{
	const auto n = grid.size();
	if (mask.size() != n)
		return {};
	std::vector<bool> outside(n, false);
	std::deque<std::size_t> queue;
	auto push = [&](std::size_t i) {
		if (!mask[i] && !outside[i]) {
			outside[i] = true;
			queue.push_back(i);
		}
	};
	for (std::size_t x = 0; x < grid.w; ++x) {
		push(x);
		if (grid.h)
			push((grid.h - 1) * grid.w + x);
	}
	for (std::size_t z = 0; z < grid.h; ++z) {
		if (grid.w) {
			push(z * grid.w);
			push(z * grid.w + grid.w - 1);
		}
	}
	while (!queue.empty()) {
		const auto i = queue.front();
		queue.pop_front();
		const auto x = i % grid.w, z = i / grid.w;
		if (x)
			push(i - 1);
		if (x + 1 < grid.w)
			push(i + 1);
		if (z)
			push(i - grid.w);
		if (z + 1 < grid.h)
			push(i + grid.w);
	}
	std::vector<bool> result(n, false);
	for (std::size_t i = 0; i < n; ++i)
		result[i] = !outside[i] && !mask[i];
	return result;
}

std::tuple<int, std::uint8_t, bool> Profile::at(float f) const
{
	int base = lip;
	float rest = f;
	for (std::uint8_t tier = 0; tier < tiers; ++tier) {
		const bool last = tier + 1 == tiers;
		if (last || rest < tier_len) {
			const int h = base + static_cast<int>(std::floor(rest * RAKE));
			return {std::min(h, cap), tier, false};
		}
		base += static_cast<int>(std::floor(tier_len * RAKE));
		rest -= tier_len;
		if (rest < walkway)
			return {std::min(base, cap), tier, true};
		rest -= walkway;
		base += jump;
	}
	return {std::min(base, cap), static_cast<std::uint8_t>(tiers ? tiers - 1 : 0), false};
}

std::optional<Frame> frame_of(const std::vector<std::pair<int, int>> &cells)
{
	const auto hull = sport_pitches::hull_ccw(cells);
	if (hull.size() < 3)
		return std::nullopt;
	double best_area = std::numeric_limits<double>::infinity();
	Frame best;
	for (std::size_t i = 0; i < hull.size(); ++i) {
		const auto [x0, z0] = hull[i];
		const auto [x1, z1] = hull[(i + 1) % hull.size()];
		const double len = std::hypot(x1 - x0, z1 - z0);
		if (len < 1e-9)
			continue;
		const double ux = (x1 - x0) / len, uz = (z1 - z0) / len;
		const double vx = -uz, vz = ux;
		double a0 = INFINITY, a1 = -INFINITY, b0 = INFINITY, b1 = -INFINITY;
		for (const auto &[x, z] : hull) {
			const double a = x * ux + z * uz, b = x * vx + z * vz;
			a0 = std::min(a0, a);
			a1 = std::max(a1, a);
			b0 = std::min(b0, b);
			b1 = std::max(b1, b);
		}
		const double area = (a1 - a0) * (b1 - b0);
		if (area >= best_area)
			continue;
		best_area = area;
		const double ca = (a0 + a1) * 0.5, cb = (b0 + b1) * 0.5;
		best.cx = ca * ux + cb * vx;
		best.cz = ca * uz + cb * vz;
		best.ux = ux;
		best.uz = uz;
		best.half_len = (a1 - a0) * 0.5;
		best.half_wid = (b1 - b0) * 0.5;
	}
	if (best.half_wid > best.half_len) {
		std::swap(best.half_len, best.half_wid);
		const double ux = -best.uz, uz = best.ux;
		best.ux = ux;
		best.uz = uz;
	}
	return best;
}

void close_gaps(const Grid &grid, std::vector<bool> &stand,
		const std::vector<bool> &within, double radius)
{
	if (stand.size() != grid.size() || within.size() != grid.size())
		return;
	const auto r = static_cast<std::uint16_t>(std::max(0.0, radius * UNIT));
	if (!r)
		return;
	const auto to_stand = chamfer(grid, stand);
	std::vector<bool> grown(grid.size());
	for (std::size_t i = 0; i < grown.size(); ++i)
		grown[i] = to_stand[i] <= r;
	const auto to_outside = chamfer(grid, [&grown](std::size_t i) { return !grown[i]; });
	for (std::size_t i = 0; i < stand.size(); ++i)
		if (!stand[i] && within[i] && grown[i] && to_outside[i] > r)
			stand[i] = true;
}

void thin_out(const Grid &grid, std::vector<bool> &stand, std::uint16_t min_depth)
{
	if (stand.size() != grid.size())
		return;
	const auto outside = chamfer(grid, [&stand](std::size_t i) { return !stand[i]; });
	const auto threshold = min_depth / 2;
	const auto deep = chamfer(grid, [&stand, &outside, threshold](std::size_t i) {
		return stand[i] && outside[i] >= threshold;
	});
	for (std::size_t i = 0; i < stand.size(); ++i)
		if (stand[i] && deep[i] > threshold + static_cast<std::uint16_t>(UNIT))
			stand[i] = false;
}

std::optional<SynthField> synthesize_field(const Grid &grid,
		const std::vector<bool> &venue, std::optional<Sport> sport, std::uint64_t id,
		double scale)
{
	if (venue.size() != grid.size())
		return std::nullopt;
	const Sport code = sport.value_or(Sport::Soccer);
	const auto dimensions = field_m(code);
	if (!dimensions)
		return std::nullopt;
	std::vector<std::pair<int, int>> cells;
	for (std::size_t i = 0; i < grid.size(); ++i)
		if (venue[i])
			cells.push_back(grid.coordinate(i));
	const auto frame = frame_of(cells);
	if (!frame)
		return std::nullopt;
	const double apron = scale * (apron_m(code) + MIN_DEPTH_M);
	const double half_len =
			std::min(scale * dimensions->first * 0.5, frame->half_len - apron);
	const double half_wid =
			std::min(scale * dimensions->second * 0.5, frame->half_wid - apron);
	if (half_len < scale * dimensions->first * 0.3 ||
			half_wid < scale * dimensions->second * 0.3 ||
			frame->half_len > scale * dimensions->first * 2.0 ||
			frame->half_wid > scale * dimensions->second * 2.5)
		return std::nullopt;
	const double vx = -frame->uz, vz = frame->ux;
	auto corner = [&](double a, double b) {
		return std::pair{
				static_cast<int>(std::lround(frame->cx + a * frame->ux + b * vx)),
				static_cast<int>(std::lround(frame->cz + a * frame->uz + b * vz))};
	};
	const std::array<std::pair<int, int>, 5> corners{{corner(-half_len, -half_wid),
			corner(half_len, -half_wid), corner(half_len, half_wid),
			corner(-half_len, half_wid), corner(-half_len, -half_wid)}};
	SynthField out;
	out.way.id = id;
	out.way.tags = {{"leisure", "pitch"}, {"sport", pitch_tag(code)}};
	for (std::size_t i = 0; i < corners.size(); ++i) {
		ProcessedNode node{};
		node.id = i;
		node.x = corners[i].first;
		node.z = corners[i].second;
		out.way.nodes.push_back(std::move(node));
	}
	Shape field_shape;
	field_shape.rings.emplace_back(corners.begin(), corners.end());
	field_shape.outer.push_back(true);
	field_shape.bbox = {
			corners[0].first, corners[0].second, corners[0].first, corners[0].second};
	for (const auto &[x, z] : corners) {
		field_shape.bbox.min_x = std::min(field_shape.bbox.min_x, x);
		field_shape.bbox.min_z = std::min(field_shape.bbox.min_z, z);
		field_shape.bbox.max_x = std::max(field_shape.bbox.max_x, x);
		field_shape.bbox.max_z = std::max(field_shape.bbox.max_z, z);
	}
	field_shape.centroid = {static_cast<int>(frame->cx), static_cast<int>(frame->cz)};
	std::vector<bool> mask(grid.size(), false);
	rasterize(grid, field_shape, mask);
	for (std::size_t i = 0; i < mask.size(); ++i)
		if (mask[i])
			out.cells.push_back(grid.coordinate(i));
	return out;
}

StadiumPlan prescan(const std::vector<ProcessedElement> &elements, const Args &args,
		const std::set<Key> &already)
{
	StadiumPlan plan;
	if (args.mode != GenerationMode::GeoTerrain && args.mode != GenerationMode::GeoOnly)
		return plan;
	if (std::none_of(elements.begin(), elements.end(),
				[](const auto &e) { return is_venue(e.tags()) || is_stand(e.tags()); }))
		return plan;
	std::vector<Candidate> venues, stands;
	for (const auto &element : elements) {
		const bool venue = is_venue(element.tags());
		const bool stand = is_stand(element.tags());
		if (!venue && !stand)
			continue;
		auto candidate = make_candidate(element, already);
		if (!candidate)
			continue;
		if (venue && stand) {
			venues.push_back(*candidate);
			stands.push_back(std::move(*candidate));
		} else if (venue) {
			venues.push_back(std::move(*candidate));
		} else {
			stands.push_back(std::move(*candidate));
		}
	}
	if (venues.empty() && stands.empty())
		return plan;
	const int reach = static_cast<int>(std::ceil(GRANDSTAND_REACH_M * args.scale)) + 8;
	std::set<std::pair<int, int>> nearby;
	for (const auto &c : venues)
		for (const auto cell : coarse_cells(c.shape.bbox.padded(reach)))
			nearby.insert(cell);
	for (const auto &c : stands)
		for (const auto cell : coarse_cells(c.shape.bbox.padded(reach)))
			nearby.insert(cell);
	std::vector<Candidate> fields, others;
	for (const auto &element : elements) {
		if (is_venue(element.tags()) || is_stand(element.tags()))
			continue;
		const bool field = is_field(element.tags());
		const bool building = is_building(element.tags());
		if (!field && !building)
			continue;
		const auto quick = quick_bbox(element);
		if (!quick)
			continue;
		bool near_ = false;
		for (const auto cell : coarse_cells(*quick))
			if (nearby.contains(cell)) {
				near_ = true;
				break;
			}
		if (!near_)
			continue;
		auto candidate = make_candidate(element, already);
		if (!candidate)
			continue;
		(field ? fields : others).push_back(std::move(*candidate));
	}
	// Rust's slice::sort_by is stable; preserve OSM source order when equal-area
	// venue outlines overlap so claim/suppression order is identical.
	std::stable_sort(venues.begin(), venues.end(),
			[](const auto &a, const auto &b) { return a.shape.area > b.shape.area; });
	std::set<Key> claimed;
	for (const auto &outline : venues) {
		if (contains_key(claimed, outline.key()))
			continue;
		if (auto p = plan_venue(outline, stands, fields, others, claimed, args.scale))
			take_plan(plan, std::move(*p), claimed);
	}
	for (const auto &stand : stands) {
		if (tag(stand.tags(), "building") != "stadium" ||
				contains_key(claimed, stand.key()))
			continue;
		if (auto p = plan_venue(stand, stands, fields, others, claimed, args.scale))
			take_plan(plan, std::move(*p), claimed);
	}
	std::map<std::size_t, std::vector<const Candidate *>> groups;
	const double stand_reach = GRANDSTAND_REACH_M * args.scale;
	for (const auto &stand : stands) {
		if (contains_key(claimed, stand.key()) ||
				tag(stand.tags(), "building") == "stadium")
			continue;
		if (!is_building(stand.tags()) && on_a_building(stand, others))
			continue;
		const auto [sx, sz] = stand.shape.centroid;
		std::optional<std::pair<std::size_t, double>> nearest;
		for (std::size_t i = 0; i < fields.size(); ++i) {
			if (contains_key(claimed, fields[i].key()))
				continue;
			const double distance = bbox_distance(fields[i].shape.bbox, sx, sz);
			if (distance <= stand_reach && (!nearest || distance < nearest->second))
				nearest = std::pair{i, distance};
		}
		if (nearest)
			groups[nearest->first].push_back(&stand);
	}
	for (const auto &[field_index, members] : groups)
		if (auto p = plan_grandstands(
					fields[field_index], members, others, claimed, args.scale))
			take_plan(plan, std::move(*p), claimed);
	return plan;
}

std::vector<std::vector<std::pair<int, int>>> StadiumPlan::placement_regions() const
{
	std::vector<std::vector<std::pair<int, int>>> regions;
	regions.reserve(venues.size());
	for (const auto &venue : venues) {
		const auto b = venue.grid.bbox().padded(TREE_SWEEP_REACH);
		std::vector<std::pair<int, int>> keys;
		for (int x = region_cell(b.min_x); x <= region_cell(b.max_x); ++x)
			for (int z = region_cell(b.min_z); z <= region_cell(b.max_z); ++z)
				keys.emplace_back(x, z);
		regions.push_back(std::move(keys));
	}
	return regions;
}

void place_venue(::arnis::world_editor::WorldEditor &, const Venue &);

void place_stadiums(::arnis::world_editor::WorldEditor &editor, const StadiumPlan &plan,
		const std::vector<std::size_t> &indices)
{
	if (indices.empty())
		return;
	std::cout << "  [+] Building " << indices.size() << " sports venue"
			  << (indices.size() == 1 ? "...\n" : "s...\n");
	for (const auto index : indices)
		if (index < plan.venues.size())
			place_venue(editor, plan.venues[index]);
}

void place_masts(::arnis::world_editor::WorldEditor &editor, const Venue &venue, int base)
{
	if (!venue.field_frame)
		return;
	const auto &frame = *venue.field_frame;
	const auto &[vx, vz] = std::pair{-frame.uz, frame.ux};
	int top_seat = 0;
	for (std::size_t i = 0; i < venue.stand.size(); ++i)
		if (venue.stand[i])
			top_seat = std::max(top_seat,
					std::get<0>(
							venue.profile.at(venue.front[i] / static_cast<float>(UNIT))));
	const int mast_top = base + top_seat + venue.mast_above;
	for (const auto &[sa, sb] :
			{std::pair{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}}) {
		const double tx =
				frame.cx + sa * frame.half_len * frame.ux + sb * frame.half_wid * vx;
		const double tz =
				frame.cz + sa * frame.half_len * frame.uz + sb * frame.half_wid * vz;
		const double rx = tx - frame.cx, rz = tz - frame.cz;
		const double length = std::max(std::hypot(rx, rz), 1.0);
		const double dx = rx / length, dz = rz / length;
		std::optional<std::pair<int, int>> spot;
		for (double t = length; t < length + venue.grid.w + venue.grid.h; t += 1.0) {
			const int x = static_cast<int>(std::lround(frame.cx + dx * t));
			const int z = static_cast<int>(std::lround(frame.cz + dz * t));
			const auto i = venue.grid.index(x, z);
			if (!i)
				break;
			if (venue.stand[*i])
				spot = {{x, z}};
		}
		const auto [x, z] =
				spot.value_or(std::pair{static_cast<int>(std::lround(tx + dx * 8.0)),
						static_cast<int>(std::lround(tz + dz * 8.0))});
		if (!venue.grid.index(x, z))
			continue;
		const int from = editor.get_ground_level(x, z) + 1;
		for (int y = from; y < mast_top; ++y) {
			const auto existing = editor.get_block_absolute(x, y, z);
			if (existing && *existing != AIR)
				continue;
			editor.set_block_absolute(LIGHT_GRAY_CONCRETE, x, y, z, std::nullopt,
					std::optional<std::vector<Block>>(std::vector<Block>{}));
		}
		for (int k = -2; k <= 2; ++k)
			for (int dy = 0; dy < 3; ++dy) {
				const int lx = static_cast<int>(std::lround(x - dz * k - dx));
				const int lz = static_cast<int>(std::lround(z + dx * k - dz));
				const Block &block =
						dy == 1 || k == 0 ? SEA_LANTERN : LIGHT_GRAY_CONCRETE;
				editor.set_block_absolute(block, lx, mast_top + dy, lz, std::nullopt,
						std::optional<std::vector<Block>>(std::vector<Block>{}));
			}
	}
}

void place_venue(::arnis::world_editor::WorldEditor &editor, const Venue &venue)
{
	const auto &grid = venue.grid;
	const int base = [&] {
		std::vector<int> levels;
		const std::size_t step = std::max<std::size_t>(grid.size() / 4000, 1);
		for (std::size_t i = 0; i < grid.size(); i += step)
			if (venue.field[i]) {
				const auto [x, z] = grid.coordinate(i);
				levels.push_back(editor.get_ground_level(x, z));
			}
		if (levels.empty())
			return editor.get_ground_level(static_cast<int>(venue.centre.first),
					static_cast<int>(venue.centre.second));
		std::sort(levels.begin(), levels.end());
		return levels[levels.size() / 2];
	}();
	// Match models_3d::clear_trees_in_mask: include enclosed bowl cells so a tree
	// rooted inside a ring of mapped stand sections is removed as one connected tree.
	std::vector<bool> clear_mask = venue.stand;
	const auto holes = enclosed(grid, venue.stand);
	for (std::size_t i = 0; i < clear_mask.size(); ++i)
		clear_mask[i] = clear_mask[i] || holes[i];
	const auto gx0 = grid.x0, gz0 = grid.z0;
	const auto keep_column = [gx0, gz0, &grid](int x, int z) {
		return x >= gx0 - TREE_SWEEP_REACH &&
			   x <= gx0 + static_cast<int>(grid.w) - 1 + TREE_SWEEP_REACH &&
			   z >= gz0 - TREE_SWEEP_REACH &&
			   z <= gz0 + static_cast<int>(grid.h) - 1 + TREE_SWEEP_REACH;
	};
	const int tree_top = base + venue.profile.cap + 30;
	for (std::size_t i = 0; i < clear_mask.size(); ++i) {
		if (!clear_mask[i])
			continue;
		const auto [x, z] = grid.coordinate(i);
		const int ground = editor.get_ground_level(x, z);
		for (int y = ground; y <= std::min(ground + 64, tree_top); ++y)
			if (water_depth::is_tree_block(editor.block_name_absolute(x, y, z)))
				water_depth::clear_tree_within(editor, x, y, z, keep_column);
	}
	if (venue.synth_field)
		for (const auto &[x, z] : venue.synth_field->cells) {
			editor.set_block(GRASS_BLOCK, x, 0, z, std::nullopt,
					std::optional<std::vector<Block>>(std::vector<Block>{}));
			editor.set_block(
					AIR, x, 1, z, ground_decoration::loose_plant_blocks(), std::nullopt);
		}
	if (venue.synth_field)
		sport_pitches::draw_pitch_markings(
				editor, venue.synth_field->way, venue.synth_field->cells, GRASS_BLOCK);
	const auto roof_y = venue.roof_top ? base + *venue.roof_top : [&] {
		int top = venue.profile.lip;
		for (std::size_t i = 0; i < venue.stand.size(); ++i)
			if (venue.stand[i])
				top = std::max(top, std::get<0>(venue.profile.at(venue.front[i] / UNIT)));
		return base + top + venue.roof_clearance;
	}();
	const double sections = std::max(
			4.0, std::round(2.0 * std::acos(-1.0) * venue.front_radius / AISLE_SPACING));
	for (std::size_t i = 0; i < venue.stand.size(); ++i) {
		if (!venue.stand[i])
			continue;
		const auto [x, z] = grid.coordinate(i);
		const float f = venue.front[i] / static_cast<float>(UNIT);
		const float b = venue.back[i] / static_cast<float>(UNIT);
		const auto [height, tier, walkway] = venue.profile.at(f);
		const int ground = editor.get_ground_level(x, z);
		const int seat_y = base + height;
		const double dx = x - venue.centre.first, dz = z - venue.centre.second;
		const double angle = std::atan2(dz, dx);
		const double turn =
				std::fmod(angle / (2.0 * std::acos(-1.0)) + 1.0, 1.0) * sections;
		const bool aisle = turn - std::floor(turn) < 1.0 / AISLE_SPACING;
		const double roof_from = (1.0 - venue.roof_cover) * (f + b);
		const bool roofed =
				venue.roof_cover > 0 && f >= roof_from && f + b >= venue.roof_min_depth;
		auto put = [&](int y, const Block &block) {
			editor.set_block_absolute(block, x, y, z, std::nullopt,
					std::optional<std::vector<Block>>(std::vector<Block>{}));
		};
		for (int y = seat_y + 1; y <= ground + 2; ++y)
			put(y, AIR);
		if (b < 1.0f) {
			const int top = std::max(roofed ? roof_y : seat_y + 1, ground);
			const auto along =
					static_cast<std::int64_t>(std::llround(angle * std::hypot(dx, dz)));
			for (int y = std::min(ground, seat_y) + 1; y <= top; ++y) {
				if (y == seat_y)
					continue;
				const Block &block =
						along % 6 == 0 ? venue.palette.rib
						: y - std::min(ground, seat_y) > 2 &&
										(y - std::min(ground, seat_y)) % 6 == 0
								? GRAY_STAINED_GLASS
								: venue.palette.wall;
				put(y, block);
			}
		} else {
			const bool boxes = !walkway && tier > 0 &&
							   std::get<2>(venue.profile.at(std::max(f - 1.0f, 0.0f)));
			const int walk_top =
					base + std::get<0>(venue.profile.at(std::max(f - 1.0f, 0.0f)));
			const Block &fill = f < 1.0f ? venue.palette.accent : venue.palette.wall;
			for (int y = ground + 1; y < seat_y; ++y) {
				const Block &block =
						boxes && y > walk_top
								? ((y - walk_top) % 3 == 0 ? LIGHT_GRAY_CONCRETE
														   : GRAY_STAINED_GLASS)
								: fill;
				put(y, block);
			}
		}
		put(seat_y, walkway || aisle ? LIGHT_GRAY_CONCRETE : venue.palette.seat);
		if (roofed) {
			put(roof_y, f < roof_from + 1.5f   ? LIGHT_GRAY_CONCRETE
						: f < roof_from + 6.0f ? WHITE_STAINED_GLASS
											   : WHITE_CONCRETE);
			if (f < roof_from + 1.5f && arnis::land_cover::coord_hash(x, z) % 5 == 0)
				put(roof_y - 1, SEA_LANTERN);
		}
	}
	if (venue.roof_cover < 0.3f)
		place_masts(editor, venue, base);
	std::cout << "  " << venue.name.value_or("Sports venue") << " ("
			  << std::count(venue.stand.begin(), venue.stand.end(), true)
			  << " stand columns" << (venue.roof_cover > 0 ? ", roofed)\n" : ")\n");
}
} // namespace arnis::element_processing::stadiums
