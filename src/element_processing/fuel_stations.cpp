#include "fuel_stations.h"

#include "../block_definitions.h"
#include "../mapillary/geometry.h"
#include "../structures/structures.h"
#include "../unicode_text.h"
#include "road_mask_search.h"
#include "structure_kit.h"

#include "../../../arnis_world_editor.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_set>

namespace arnis::fuel_stations
{
namespace
{
bool tag_is(const tags_t &tags, const char *key, const char *value)
{
	auto it = tags.find(key);
	return it != tags.end() && it->second == value;
}

bool is_fuel(const tags_t &tags)
{
	return tag_is(tags, "amenity", "fuel");
}
bool is_roof(const tags_t &tags)
{
	return tag_is(tags, "building", "roof") || tag_is(tags, "building:part", "roof");
}

bool other_roof_use(const tags_t &tags)
{
	if (const auto it = tags.find("amenity"); it != tags.end() && it->second != "fuel")
		return true;
	for (const char *key : {"shop", "parking", "shelter", "leisure", "public_transport"})
		if (tags.contains(key))
			return true;
	return false;
}

std::pair<double, double> centre(const std::vector<std::pair<double, double>> &ring)
{
	if (ring.empty())
		return {};
	double x = 0, z = 0;
	for (const auto &[px, pz] : ring) {
		x += px;
		z += pz;
	}
	return {x / ring.size(), z / ring.size()};
}

bool inside(const std::vector<std::pair<double, double>> &ring, double x, double z)
{
	if (ring.size() < 3)
		return false;
	bool hit = false;
	for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
		const auto [xi, zi] = ring[i];
		const auto [xj, zj] = ring[j];
		if ((zi > z) != (zj > z) && x < (xj - xi) * (z - zi) / (zj - zi) + xi)
			hit = !hit;
	}
	return hit;
}

Livery livery_for_impl(const tags_t &tags, std::uint64_t seed)
{
	Block white = WHITE_CONCRETE, red = RED_CONCRETE, blue = BLUE_CONCRETE,
		  yellow = YELLOW_CONCRETE, green = GREEN_CONCRETE, orange = ORANGE_CONCRETE;
	const std::pair<const char *, Livery> brands[] = {
			{"aral", {LIGHT_BLUE_CONCRETE, white, LIGHT_BLUE_CONCRETE, std::nullopt}},
			{"shell", {yellow, red, white}},
			{"esso", {white, red, white, blue}},
			{"exxon", {white, red, white, blue}},
			{"mobil", {white, blue, white, red}},
			{"bp", {green, LIME_CONCRETE, white}},
			{"total", {white, red, white, orange}},
			{"totalenergies", {white, red, white, orange}},
			{"jet", {yellow, blue, yellow}},
			{"avia", {white, red, red}},
			{"agip", {yellow, BLACK_CONCRETE, yellow}},
			{"eni", {yellow, BLACK_CONCRETE, yellow}},
			{"omv", {white, blue, white, LIME_CONCRETE}},
			{"q8", {blue, orange, blue}},
			{"texaco", {BLACK_CONCRETE, red, white}},
			{"chevron", {white, blue, white, red}},
			{"circle k", {red, orange, white}},
			{"repsol", {orange, red, white}},
			{"lukoil", {red, white, red}},
			{"gazprom", {blue, white, blue}},
			{"rosneft", {yellow, BLACK_CONCRETE, yellow}},
			{"petrobras", {green, yellow, white}},
			{"pemex", {green, red, white}},
			{"indian oil", {orange, blue, white}},
			{"sinopec", {white, red, white}},
			{"petrochina", {red, yellow, white}},
			{"eneos", {white, red, white}},
			{"caltex", {red, blue, white}},
			{"ampol", {white, red, white, blue}},
			{"speedway", {red, white, white}},
			{"marathon", {white, blue, white, red}},
			{"valero", {blue, yellow, white}},
			{"sunoco", {blue, yellow, white, red}},
			{"citgo", {white, red, white, blue}},
			{"phillips 66", {BLACK_CONCRETE, red, white}},
			{"conoco", {red, white, white}},
			{"sinclair", {green, white, white}},
			{"costco", {red, blue, white}},
			{"arco", {blue, white, white, red}},
			{"76", {orange, blue, white}},
			{"wawa", {red, yellow, white}},
			{"sheetz", {red, white, red}},
			{"casey", {red, yellow, white}},
			{"quiktrip", {red, white, white}},
			{"love's", {yellow, red, white}},
			{"pilot", {red, yellow, white}},
			{"tesco", {blue, red, white}},
			{"sainsbury", {orange, white, white}},
			{"morrisons", {yellow, green, white}},
			{"asda", {green, white, white}},
			{"gulf", {orange, blue, white}},
			{"tamoil", {orange, red, white}},
			{"orlen", {red, white, white}},
			{"star", {red, yellow, white}},
			{"bft", {blue, red, white}},
			{"hem", {red, white, white}},
			{"westfalen", {green, yellow, white}},
			{"raiffeisen", {green, yellow, white}},
			{"neste", {green, blue, white}},
			{"okq8", {blue, orange, white}},
			{"st1", {orange, blue, white}},
			{"tinq", {blue, orange, white}},
	};
	for (const char *key : {"brand", "name", "operator"}) {
		auto it = tags.find(key);
		if (it == tags.end())
			continue;
		std::string value = unicode_text::lowercase(it->second);
		for (std::size_t pos = 0;
				(pos = value.find("\xE2\x80\x99", pos)) != std::string::npos;) {
			value.replace(pos, 3, "'");
			++pos;
		}
		// Rust tokenizes brand names and values on any non-alphanumeric character
		// except apostrophes. Matching token sequences rather than literal phrases
		// keeps brands such as "Circle   K" and "Indian/Oil" recognizable too.
		const auto tokens = [](std::string_view text) {
			std::vector<std::string_view> result;
			for (std::size_t i = 0; i < text.size();) {
				const auto [first, first_length] = unicode_text::codepoint_at(text, i);
				const bool first_is_word =
						unicode_text::is_alphanumeric(first) || first == '\'';
				if (!first_is_word) {
					i += first_length;
					continue;
				}
				const auto begin = i;
				while (i < text.size()) {
					const auto [codepoint, length] = unicode_text::codepoint_at(text, i);
					const bool is_word =
							unicode_text::is_alphanumeric(codepoint) || codepoint == '\'';
					if (!is_word)
						break;
					i += length;
				}
				result.push_back(text.substr(begin, i - begin));
			}
			return result;
		};
		const auto value_words = tokens(value);
		for (const auto &[brand, look] : brands) {
			const auto brand_words = tokens(brand);
			if (brand_words.empty() || brand_words.size() > value_words.size())
				continue;
			for (std::size_t first = 0; first + brand_words.size() <= value_words.size();
					++first) {
				bool match = true;
				for (std::size_t i = 0; i < brand_words.size(); ++i) {
					auto word = value_words[first + i];
					if (word == brand_words[i])
						continue;
					if (word.size() > 2 && word.substr(word.size() - 2) == "'s" &&
							word.substr(0, word.size() - 2) == brand_words[i])
						continue;
					match = false;
					break;
				}
				if (match)
					return look;
			}
		}
	}
	const Block colors[] = {red, blue, green, orange};
	return {white, colors[seed % 4], white, std::nullopt};
}

std::tuple<double, double, double, double> long_axis(
		const std::vector<std::pair<int, int>> &cells)
{
	const double n = static_cast<double>(std::max<std::size_t>(1, cells.size()));
	double mx = 0.0, mz = 0.0;
	for (const auto &[x, z] : cells) {
		mx += x + 0.5;
		mz += z + 0.5;
	}
	mx /= n;
	mz /= n;
	double sxx = 0.0, szz = 0.0, sxz = 0.0;
	for (const auto &[x, z] : cells) {
		const double dx = x + 0.5 - mx, dz = z + 0.5 - mz;
		sxx += dx * dx;
		szz += dz * dz;
		sxz += dx * dz;
	}
	const double angle = 0.5 * std::atan2(2.0 * sxz, sxx - szz);
	return {mx, mz, std::cos(angle), std::sin(angle)};
}

Block pump_head(const Livery &look)
{
	return look.stripe == WHITE_CONCRETE ? look.fascia : look.stripe;
}

std::uint64_t cell_key(int x, int z)
{
	return (std::uint64_t(std::uint32_t(x)) << 32) | std::uint32_t(z);
}

void draw_canopy(WorldEditor &editor, const std::vector<std::pair<int, int>> &filled,
		const Site &site, int base, int min_roof, int max_roof,
		const RoadMaskBitmap &roads, std::uint64_t seed)
{
	std::unordered_set<std::uint64_t> cells;
	for (const auto &[x, z] : filled)
		cells.insert(cell_key(x, z));
	for (const auto &[x, z] : filled) {
		if (!roads.contains(x, z)) {
			editor.register_road_surface_y(x, z, base);
			editor.set_block_absolute(LIGHT_GRAY_CONCRETE, x, base, z);
		}
		const bool edge = !cells.contains(cell_key(x + 1, z)) ||
						  !cells.contains(cell_key(x - 1, z)) ||
						  !cells.contains(cell_key(x, z + 1)) ||
						  !cells.contains(cell_key(x, z - 1));
		for (int y = base + min_roof; y <= base + max_roof; ++y) {
			const int from_top = base + max_roof - y;
			Block block;
			if (edge) {
				block = from_top == 0	? site.livery.fascia
						: from_top == 1 ? site.livery.stripe
						: from_top == 2 ? site.livery.stripe2.value_or(site.livery.fascia)
										: site.livery.fascia;
			} else {
				if (y == base + min_roof) {
					const auto mod3 = [](int value) { return (value % 3 + 3) % 3; };
					block = mod3(x) == 0 && mod3(z) == 0 ? SEA_LANTERN : WHITE_CONCRETE;
				} else {
					block = LIGHT_GRAY_CONCRETE;
				}
			}
			editor.set_block_absolute(block, x, y, z);
		}
	}
	// Pump islands follow the canopy's long axis. Each bay gets two dispensers,
	// hoses on both sides, bollards and a central support.
	const auto [mx, mz, ux, uz] = long_axis(filled);
	auto frame = structure_kit::Frame::from_heading(mx, mz, ux, uz);
	double a0 = std::numeric_limits<double>::max(), a1 = -a0;
	double b0 = a0, b1 = -a0;
	for (const auto &[x, z] : filled) {
		const auto [a, b] = frame.local(x + 0.5, z + 0.5);
		a0 = std::min(a0, a);
		a1 = std::max(a1, a);
		b0 = std::min(b0, b);
		b1 = std::max(b1, b);
	}
	const double length = a1 - a0, width = b1 - b0;
	if (length < 6.0 || width < 4.0)
		return;
	const std::size_t rows =
			static_cast<std::size_t>(std::max(1.0, std::round((width - 1.0) / 7.0)));
	const std::size_t bays =
			static_cast<std::size_t>(std::max(1.0, std::round((length - 2.0) / 8.0)));
	const auto along = structure_kit::axis_of(ux, uz);
	const std::pair<int, int> across{-along.second, along.first};
	const bool axis_aligned = std::abs(ux) > 0.92 || std::abs(uz) > 0.92;
	std::uint64_t bay_seed = 0;
	for (std::size_t row = 0; row < rows; ++row) {
		const double b = b0 + width * (row + 0.5) / rows;
		for (std::size_t bay = 0; bay < bays; ++bay) {
			const double a = a0 + length * (bay + 0.5) / bays;
			auto at = [&](double da) { return frame.cell(a + da, b); };
			const auto edge_a = at(-3.0), edge_b = at(3.0);
			if (!cells.contains(cell_key(edge_a.first, edge_a.second)) ||
					!cells.contains(cell_key(edge_b.first, edge_b.second)))
				continue;
			for (double da : {-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0}) {
				const auto [x, z] = at(da);
				structure_kit::place(editor,
						structure_kit::with_prop(SMOOTH_STONE_SLAB, "type", "bottom"), x,
						base + 1, z);
			}
			const auto [support_x, support_z] = at(0.0);
			for (int y = base + 1; y < base + min_roof; ++y)
				editor.set_block_absolute(site.livery.column, support_x, y, support_z);
			for (double da : {-2.0, 2.0}) {
				const auto [x, z] = at(da);
				editor.set_block_absolute(WHITE_CONCRETE, x, base + 1, z, std::nullopt,
						std::optional<std::vector<Block>>{
								std::vector<Block>{SMOOTH_STONE_SLAB}});
				editor.set_block_absolute(pump_head(site.livery), x, base + 2, z);
				structure_kit::place(editor,
						structure_kit::with_prop(SMOOTH_STONE_SLAB, "type", "bottom"), x,
						base + 3, z);
				for (const auto &side :
						{across, std::pair<int, int>{-across.first, -across.second}})
					structure_kit::place(editor,
							structure_kit::with_props(TRIPWIRE_HOOK,
									{{"facing", structure_kit::facing_name(side)},
											{"attached", "false"}, {"powered", "false"}}),
							x + side.first, base + 2, z + side.second);
			}
			for (double da : {-3.0, 3.0}) {
				const auto [x, z] = at(da);
				editor.set_block_absolute(ANDESITE_WALL, x, base + 1, z, std::nullopt,
						std::optional<std::vector<Block>>{
								std::vector<Block>{SMOOTH_STONE_SLAB}});
				editor.set_block_absolute(YELLOW_CONCRETE, x, base + 2, z);
			}
			const auto [bowl_x, bowl_z] = at(-1.0);
			editor.set_block_absolute(CAULDRON, bowl_x, base + 1, bowl_z, std::nullopt,
					std::optional<std::vector<Block>>{
							std::vector<Block>{SMOOTH_STONE_SLAB}});
			if (axis_aligned)
				for (double side : {-3.0, 3.0}) {
					++bay_seed;
					if (structure_kit::mix(seed, bay_seed) % 3 != 0)
						continue;
					const auto [x, z] = frame.cell(a - 0.5, b + side);
					if (cells.contains(cell_key(x, z)))
						structures::car::maybe_place_car_on_deck(editor, x, z, base + 1,
								along.first != 0 ? 1 : 0, min_roof - 1);
				}
		}
	}
}

int ground_at(WorldEditor &editor, int x, int z);

void draw_kiosk(WorldEditor &editor, const Site &site, const structure_kit::Frame &canopy,
		double canopy_half_width, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &roads, const std::unordered_set<std::uint64_t> &area)
{
	const double b0 = canopy_half_width + 3.0, b1 = canopy_half_width + 9.0;
	const auto cells = canopy.cells(-4.5, 4.5, b0, b1);
	for (const auto &cell : cells) {
		const int x = static_cast<int>(cell[0]), z = static_cast<int>(cell[1]);
		if (footprints.contains(x, z) || roads.contains(x, z) ||
				(!area.empty() && !area.contains(cell_key(x, z))))
			return;
	}
	std::vector<int> heights;
	heights.reserve(cells.size());
	for (const auto &cell : cells)
		heights.push_back(
				ground_at(editor, static_cast<int>(cell[0]), static_cast<int>(cell[1])));
	if (heights.empty())
		return;
	std::sort(heights.begin(), heights.end());
	const int base = heights[heights.size() / 2];
	const auto side = canopy.v();
	const auto front = structure_kit::axis_of(-side.first, -side.second);
	for (const auto &cell : cells) {
		const int x = static_cast<int>(cell[0]), z = static_cast<int>(cell[1]);
		const double a = cell[2], b = cell[3];
		editor.register_road_surface_y(x, z, base);
		editor.set_block_absolute(LIGHT_GRAY_CONCRETE, x, base, z);
		const bool wall = std::abs(a) > 4.0 || b - b0 < 1.0 || b1 - b < 1.0;
		if (!wall) {
			editor.set_block_absolute(LIGHT_GRAY_CONCRETE, x, base + 4, z);
			continue;
		}
		const bool glazed = b - b0 < 1.0 && std::abs(a) < 3.5;
		for (int y = base + 1; y <= base + 3; ++y)
			editor.set_block_absolute(
					glazed && y < base + 3 ? GLASS : WHITE_CONCRETE, x, y, z);
		editor.set_block_absolute(site.livery.fascia, x, base + 4, z);
	}
	const auto [door_x, door_z] = canopy.cell(0.0, b0 + 0.5);
	const std::vector<Block> glass{GLASS};
	editor.set_block_absolute(AIR, door_x, base + 1, door_z, &glass, nullptr);
	editor.set_block_absolute(AIR, door_x, base + 2, door_z, &glass, nullptr);
	structure_kit::place(editor,
			structure_kit::with_props(IRON_DOOR,
					{{"facing", structure_kit::facing_name(front)}, {"half", "lower"}}),
			door_x, base + 1, door_z);
	structure_kit::place(editor,
			structure_kit::with_props(IRON_DOOR,
					{{"facing", structure_kit::facing_name(front)}, {"half", "upper"}}),
			door_x, base + 2, door_z);
}

int ground_at(WorldEditor &editor, int x, int z)
{
	return editor.terrain_level(x, z).value_or(editor.get_ground_level(x, z));
}

} // namespace

Livery livery_for(const tags_t &tags, std::uint64_t seed)
{
	return livery_for_impl(tags, seed);
}

FuelIndex FuelIndex::build(const std::vector<ProcessedElement> &elements, double scale)
{
	FuelIndex out;
	for (const auto &element : elements) {
		if (!is_fuel(element.tags()))
			continue;
		Site site;
		site.id = element.id();
		site.livery =
				livery_for(element.tags(), structure_kit::mix(element.id(), 0xF0E1));
		site.has_canopy = is_roof(element.tags());
		site.has_shop = element.tags().contains("building") && !site.has_canopy;
		if (element.is_node()) {
			site.x = element.as_node().x + 0.5;
			site.z = element.as_node().z + 0.5;
		} else if (element.is_way() && element.as_way().nodes.size() >= 4) {
			for (const auto &node : element.as_way().nodes)
				site.ring.emplace_back(node.x + 0.5, node.z + 0.5);
			std::tie(site.x, site.z) = centre(site.ring);
		} else {
			continue;
		}
		out.sites.push_back(std::move(site));
	}
	// Only area sites participate in de-duplicating nearby station nodes. Keep
	// this temporary proportional to mapped station footprints, as in Rust,
	// rather than cloning every node site as well.
	std::vector<Site> areas;
	areas.reserve(std::count_if(out.sites.begin(), out.sites.end(),
			[&](const Site &site) { return !site.ring.empty(); }));
	for (const auto &site : out.sites)
		if (!site.ring.empty())
			areas.push_back(site);
	const double merge = 30.0 * scale;
	out.sites.erase(std::remove_if(out.sites.begin(), out.sites.end(),
							[&](const Site &s) {
								if (!s.ring.empty())
									return false;
								return std::any_of(
										areas.begin(), areas.end(), [&](const Site &a) {
											return !a.ring.empty() &&
												   (inside(a.ring, s.x, s.z) ||
														   std::hypot(a.x - s.x,
																   a.z - s.z) < merge);
										});
							}),
			out.sites.end());
	if (out.sites.empty())
		return out;
	const double node_reach = 40.0 * scale;
	std::vector<std::array<double, 4>> site_bounds;
	site_bounds.reserve(out.sites.size());
	double bounds_min_x = std::numeric_limits<double>::max();
	double bounds_min_z = std::numeric_limits<double>::max();
	double bounds_max_x = std::numeric_limits<double>::lowest();
	double bounds_max_z = std::numeric_limits<double>::lowest();
	for (const auto &site : out.sites) {
		std::array<double, 4> bounds;
		if (site.ring.empty()) {
			bounds = {site.x - node_reach, site.z - node_reach, site.x + node_reach,
					site.z + node_reach};
		} else {
			bounds = {std::numeric_limits<double>::max(),
					std::numeric_limits<double>::max(),
					std::numeric_limits<double>::lowest(),
					std::numeric_limits<double>::lowest()};
			for (const auto &[x, z] : site.ring) {
				bounds[0] = std::min(bounds[0], x);
				bounds[1] = std::min(bounds[1], z);
				bounds[2] = std::max(bounds[2], x);
				bounds[3] = std::max(bounds[3], z);
			}
		}
		bounds_min_x = std::min(bounds_min_x, bounds[0]);
		bounds_min_z = std::min(bounds_min_z, bounds[1]);
		bounds_max_x = std::max(bounds_max_x, bounds[2]);
		bounds_max_z = std::max(bounds_max_z, bounds[3]);
		site_bounds.push_back(bounds);
	}
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (way.nodes.empty() ||
				(!way.tags.contains("building") && !way.tags.contains("building:part")))
			continue;
		const bool roof = is_roof(way.tags);
		if (roof && other_roof_use(way.tags))
			continue;
		double x = 0, z = 0;
		for (const auto &n : way.nodes) {
			x += n.x + 0.5;
			z += n.z + 0.5;
		}
		x /= way.nodes.size();
		z /= way.nodes.size();
		if (x < bounds_min_x || x > bounds_max_x || z < bounds_min_z || z > bounds_max_z)
			continue;
		std::optional<std::pair<std::size_t, double>> best;
		for (std::size_t i = 0; i < out.sites.size(); ++i) {
			const auto &site = out.sites[i];
			if (site.id == way.id)
				continue;
			const auto &bounds = site_bounds[i];
			if (x < bounds[0] || x > bounds[2] || z < bounds[1] || z > bounds[3])
				continue;
			const double d = std::hypot(site.x - x, site.z - z);
			if ((site.ring.empty() && d <= node_reach) ||
					(!site.ring.empty() && inside(site.ring, x, z)))
				if (!best || d < best->second)
					best = std::make_pair(i, d);
		}
		if (best) {
			if (roof) {
				out.canopies[way.id] = best->first;
				out.sites[best->first].has_canopy = true;
			} else {
				out.sites[best->first].has_shop = true;
			}
		}
	}
	for (std::size_t i = 0; i < out.sites.size(); ++i)
		if (out.sites[i].has_canopy && !out.sites[i].ring.empty())
			out.canopies.try_emplace(out.sites[i].id, i);
	return out;
}

const Site *FuelIndex::site(std::uint64_t id) const
{
	auto it = std::find_if(
			sites.begin(), sites.end(), [id](const Site &s) { return s.id == id; });
	return it == sites.end() ? nullptr : &*it;
}

const Site *FuelIndex::site_for_canopy(std::uint64_t id) const
{
	auto it = canopies.find(id);
	return it == canopies.end() || it->second >= sites.size() ? nullptr
															  : &sites[it->second];
}

bool is_station_canopy(const WorldEditor &editor, const ProcessedWay &way)
{
	return is_roof(way.tags) && editor.fuel_index &&
		   editor.fuel_index->canopies.contains(way.id);
}

bool generate_canopy(WorldEditor &editor, const ProcessedWay &way,
		const FloodFillCache &fills, const RoadMaskBitmap &roads, double scale,
		const std::optional<std::chrono::milliseconds> &timeout)
{
	if (!editor.fuel_index)
		return false;
	const Site *site = editor.fuel_index->site_for_canopy(way.id);
	if (!site)
		return false;
	auto filled = fills.get_or_compute(way, timeout);
	if (!filled || filled->size() < 12)
		return false;
	std::vector<int> heights;
	for (const auto &[x, z] : *filled)
		heights.push_back(ground_at(editor, x, z));
	std::nth_element(
			heights.begin(), heights.begin() + heights.size() / 2, heights.end());
	const int base = heights[heights.size() / 2];
	auto height_tag = [&](const char *key) -> std::optional<double> {
		auto it = way.tags.find(key);
		if (it == way.tags.end())
			return std::nullopt;
		auto parsed = mapillary::geodesy::parse_length_m(it->second);
		return parsed && *parsed > 0.0 && *parsed < 30.0 ? parsed : std::nullopt;
	};
	const auto min_height = height_tag("min_height");
	const int min_roof = std::clamp(
			min_height ? static_cast<int>(std::lround(*min_height * scale)) : 5, 4, 9);
	const auto height = height_tag("height");
	const int max_roof = std::clamp(
			height ? static_cast<int>(std::lround(*height * scale)) : min_roof + 2,
			min_roof + 1, min_roof + 3);
	std::vector<std::pair<int, int>> cells(filled->begin(), filled->end());
	draw_canopy(editor, cells, *site, base, min_roof, max_roof, roads, way.id);
	return true;
}

void generate_station(WorldEditor &editor, const ProcessedElement &element,
		const FloodFillCache &fills, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &roads, double scale,
		const std::optional<std::chrono::milliseconds> &timeout)
{
	if (!editor.fuel_index)
		return;
	const Site *site = editor.fuel_index->site(element.id());
	if (!site || site->has_canopy)
		return;
	const int site_x = static_cast<int>(site->x), site_z = static_cast<int>(site->z);
	const auto road =
			element_processing::get_nearest_road_block(site_x, site_z, 40, roads);
	const bool shop_building = element.tags().contains("building");
	if (shop_building && !road)
		return;
	std::unordered_set<std::uint64_t> area;
	if (element.is_way() && !site->ring.empty() && !shop_building) {
		if (auto filled = fills.get_or_compute(element.as_way(), timeout))
			for (const auto &[x, z] : *filled)
				area.insert(cell_key(x, z));
	}
	const auto [rx, rz] = road ? std::pair<double, double>{road->first + 0.5 - site->x,
										 road->second + 0.5 - site->z}
							   : std::pair<double, double>{0.0, 1.0};
	const double distance = std::max(1e-6, std::hypot(rx, rz));
	const std::pair<double, double> away{-rx / distance, -rz / distance};
	const double half_l = std::max(7.0 * scale, 5.0);
	const double half_w = std::max(4.5 * scale, 3.5);
	double origin_x = site->x, origin_z = site->z, first = 0.0;
	if (road && shop_building) {
		origin_x = road->first + 0.5;
		origin_z = road->second + 0.5;
		first = half_w + 2.0;
	}
	for (double road_share : {0.0, 0.5}) {
		for (double shift : {0.0, 3.0, 6.0, 9.0}) {
			const double step = first + shift;
			const double cx = origin_x + away.first * step;
			const double cz = origin_z + away.second * step;
			auto frame =
					structure_kit::Frame::from_heading(cx, cz, -away.second, away.first);
			const auto framed_cells = frame.cells(-half_l, half_l, -half_w, half_w);
			std::vector<std::pair<int, int>> cells;
			cells.reserve(framed_cells.size());
			bool blocked = false;
			std::size_t on_road = 0;
			for (const auto &cell : framed_cells) {
				const int x = static_cast<int>(cell[0]), z = static_cast<int>(cell[1]);
				cells.emplace_back(x, z);
				if (footprints.contains(x, z) ||
						(!area.empty() && !area.contains(cell_key(x, z))))
					blocked = true;
				on_road += roads.contains(x, z);
			}
			if (blocked || on_road > road_share * cells.size())
				continue;
			std::vector<int> heights;
			heights.reserve(cells.size());
			for (const auto &[x, z] : cells)
				heights.push_back(ground_at(editor, x, z));
			if (heights.empty())
				return;
			std::sort(heights.begin(), heights.end());
			const int base = heights[heights.size() / 2];
			draw_canopy(editor, cells, *site, base, 5, 7, roads, site->id);
			if (!site->has_shop)
				draw_kiosk(
						editor, *site, frame.flipped(), half_w, footprints, roads, area);
			return;
		}
	}
}

} // namespace arnis::fuel_stations
