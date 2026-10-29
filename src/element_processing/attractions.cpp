#include "attractions.h"

#include "../block_definitions.h"
#include "connected_blocks.h"
#include "../mapillary/geometry.h"
#include "../unicode_text.h"
#include "structure_kit.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <iterator>
#include <string>
#include <string_view>
#include <unordered_set>

namespace arnis::attractions
{
namespace
{
constexpr double TAU = 6.28318530717958647692;
constexpr double PI = 3.14159265358979323846;
const Block RAINBOW[] = {RED_CONCRETE, ORANGE_CONCRETE, YELLOW_CONCRETE, LIME_CONCRETE,
		LIGHT_BLUE_CONCRETE, BLUE_CONCRETE, PURPLE_CONCRETE, PINK_CONCRETE};

std::string lower_name(std::string_view input)
{
	return unicode_text::lowercase(input);
}

bool unicode_word_char(std::string_view text, std::size_t offset)
{
	const auto [codepoint, _] = unicode_text::codepoint_at(text, offset);
	return unicode_text::is_alphanumeric(codepoint);
}

enum class Ride
{
	Carousel,
	Swing,
	StarFlyer,
	Wheel,
	Tower,
	Ship,
	Bumper,
	Teacups,
	Spinner
};
struct Site
{
	double x{}, z{}, length{}, width{};
	double ux{1.0}, uz{};
	std::vector<std::pair<int, int>> cells;
};
using OwnCells = std::unordered_set<std::uint64_t>;

void fence_ring(WorldEditor &editor, const Site &site, const OwnCells &own,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &roads,
		std::uint64_t seed, double radius, int floor);

const std::string *tag(const tags_t &tags, const char *name)
{
	const auto it = tags.find(name);
	return it == tags.end() ? nullptr : &it->second;
}

std::optional<Ride> hint(std::string name)
{
	name = lower_name(name);
	auto has = [&](std::initializer_list<const char *> values) {
		return std::any_of(values.begin(), values.end(),
				[&](const char *s) { return name.find(s) != std::string::npos; });
	};
	if (has({"schiffschaukel", "pirat", "viking", "galeone", "swinging ship"}))
		return Ride::Ship;
	const auto has_word = [&](const char *word) {
		for (std::size_t at = name.find(word); at != std::string::npos;
				at = name.find(word, at + 1)) {
			std::size_t previous = at;
			if (previous > 0) {
				--previous;
				while (previous > 0 &&
						(static_cast<unsigned char>(name[previous]) & 0xc0) == 0x80)
					--previous;
			}
			const bool left = at == 0 || !unicode_word_char(name, previous);
			const auto end = at + std::char_traits<char>::length(word);
			const bool right = end == name.size() || !unicode_word_char(name, end);
			if (left && right)
				return true;
		}
		return false;
	};
	if (has_word("eye") || has({"ferris", "wheel", "riesenrad", "grande roue", "ruota",
								   "noria", "reuzenrad", "diabelski", "pariserhjul"}))
		return Ride::Wheel;
	if (has({"kettenkarussell", "wellenflieger", "swing", "star flyer", "starflyer",
				"kettenflieger", "chairoplane"}))
		return Ride::Swing;
	if (has({"freifall", "freefall", "free fall", "fallturm", "power tower"}) ||
			has_word("drop"))
		return Ride::Tower;
	if (has({"bumper", "dodgem", "autoscooter", "autodrom", "skooter", "radiobil"}))
		return Ride::Bumper;
	if (has({"teacup", "tea cup", "tasse", "kopjes", "kaffeetass"}))
		return Ride::Teacups;
	if (has({"carousel", "carrousel", "karussell", "merry", "galopper", "manège"}))
		return Ride::Carousel;
	const auto has_tower_compound = [&] {
		for (std::size_t at = name.find("turm"); at != std::string::npos;
				at = name.find("turm", at + 1)) {
			const auto end = at + 4;
			if (end < name.size() && unicode_word_char(name, end))
				continue;
			std::size_t start = at;
			while (start > 0) {
				std::size_t previous = start - 1;
				while (previous > 0 &&
						(static_cast<unsigned char>(name[previous]) & 0xc0) == 0x80)
					--previous;
				if (!unicode_word_char(name, previous))
					break;
				start = previous;
			}
			if (std::string_view(name).substr(start, end - start).ends_with("sturm"))
				continue;
			return true;
		}
		return false;
	};
	if (has({"tower", "tårn", "la tour "}) || has_word("torre") || has_tower_compound())
		return Ride::Tower;
	return std::nullopt;
}

std::optional<Ride> kind(const tags_t &tags)
{
	const auto *a = tag(tags, "attraction");
	if (!a)
		return std::nullopt;
	const bool building = tags.contains("building") || tags.contains("building:part");
	const auto *name = tag(tags, "name");
	const auto named = [&]() { return name ? hint(*name) : std::nullopt; };
	Ride r;
	if (*a == "carousel") {
		auto h = named();
		r = h && (*h == Ride::Swing || *h == Ride::Teacups) ? *h : Ride::Carousel;
	} else if (*a == "swing_carousel" || *a == "swing_ride")
		r = Ride::Swing;
	else if (*a == "big_wheel")
		r = Ride::Wheel;
	else if (*a == "drop_tower")
		r = Ride::Tower;
	else if (*a == "pirate_ship" || *a == "swinging_ship" || *a == "dragon_swing" ||
			 *a == "swinging_boat")
		r = Ride::Ship;
	else if (*a == "teacup")
		r = Ride::Teacups;
	else if (*a == "bumper_car" && !building)
		r = Ride::Bumper;
	else if (*a == "amusement_ride" || *a == "kiddie_ride") {
		auto h = named();
		if (h)
			r = *h;
		else if (tag(tags, "man_made") && *tag(tags, "man_made") == "tower")
			r = Ride::Tower;
		else if (building)
			return std::nullopt;
		else
			r = Ride::Spinner;
	} else
		return std::nullopt;
	if (building && (r == Ride::Bumper || r == Ride::Spinner || r == Ride::Teacups))
		return std::nullopt;
	if (r == Ride::Spinner)
		for (const char *k :
				{"leisure", "landuse", "natural", "amenity", "highway", "waterway",
						"railway", "barrier", "tourism", "man_made", "roller_coaster"})
			if (tags.contains(k))
				return std::nullopt;
	if (r == Ride::Swing) {
		const auto *h = tag(tags, "height");
		const auto height = h ? mapillary::geodesy::parse_length_m(*h) : std::nullopt;
		if ((height && *height > 1.0 && *height < 400.0 && *height >= 30.0) ||
				(tag(tags, "man_made") && *tag(tags, "man_made") == "tower"))
			r = Ride::StarFlyer;
	}
	return r;
}

std::pair<Block, Block> ride_colours(const tags_t &tags, std::uint64_t seed)
{
	static const Block palette[] = {RED_CONCRETE, YELLOW_CONCRETE, BLUE_CONCRETE,
			LIGHT_BLUE_CONCRETE, PINK_CONCRETE, ORANGE_CONCRETE, LIME_CONCRETE,
			PURPLE_CONCRETE, MAGENTA_CONCRETE};
	const auto *text = tag(tags, "colour");
	if (!text)
		text = tag(tags, "building:colour");
	const Block main = text ? structure_kit::concrete_for_colour(*text).value_or(
									  palette[seed % std::size(palette)])
							: palette[seed % std::size(palette)];
	const Block accent =
			main == YELLOW_CONCRETE || main == WHITE_CONCRETE || main == ORANGE_CONCRETE
					? RED_CONCRETE
					: WHITE_CONCRETE;
	return {main, accent};
}

Site make_site(const ProcessedElement &e, const FloodFillCache &fills,
		const std::optional<std::chrono::milliseconds> &timeout)
{
	Site s;
	if (e.is_node()) {
		s.x = e.as_node().x + 0.5;
		s.z = e.as_node().z + 0.5;
		return s;
	}
	const auto &nodes = e.nodes();
	if (nodes.empty())
		return s;
	for (const auto &n : nodes) {
		s.x += n.x + 0.5;
		s.z += n.z + 0.5;
	}
	s.x /= nodes.size();
	s.z /= nodes.size();
	if (nodes.size() < 2)
		return s;
	double xx = 0, zz = 0, xz = 0;
	for (const auto &n : nodes) {
		const double x = n.x + 0.5 - s.x, z = n.z + 0.5 - s.z;
		xx += x * x;
		zz += z * z;
		xz += x * z;
	}
	const double a = 0.5 * std::atan2(2 * xz, xx - zz), ux = std::cos(a),
				 uz = std::sin(a);
	s.ux = ux;
	s.uz = uz;
	double a0 = 1e30, a1 = -1e30, b0 = 1e30, b1 = -1e30;
	for (const auto &n : nodes) {
		const double x = n.x + 0.5 - s.x, z = n.z + 0.5 - s.z, p = x * ux + z * uz,
					 q = -x * uz + z * ux;
		a0 = std::min(a0, p);
		a1 = std::max(a1, p);
		b0 = std::min(b0, q);
		b1 = std::max(b1, q);
	}
	s.length = a1 - a0;
	s.width = b1 - b0;
	s.x += (a0 + a1) * 0.5 * ux - (b0 + b1) * 0.5 * uz;
	s.z += (a0 + a1) * 0.5 * uz + (b0 + b1) * 0.5 * ux;
	if (e.is_way() && nodes.size() > 3 && nodes.front().id == nodes.back().id)
		if (auto fill = fills.get_or_compute(e.as_way(), timeout))
			s.cells = *fill;
	return s;
}

std::vector<std::pair<int, int>> disc(double x, double z, double r)
{
	std::vector<std::pair<int, int>> v;
	const int extent = static_cast<int>(std::ceil(r)) + 1;
	for (int px = int(std::floor(x)) - extent; px <= int(std::floor(x)) + extent; ++px)
		for (int pz = int(std::floor(z)) - extent; pz <= int(std::floor(z)) + extent;
				++pz)
			if (std::sqrt((px + 0.5 - x) * (px + 0.5 - x) +
						  (pz + 0.5 - z) * (pz + 0.5 - z)) <= r)
				v.emplace_back(px, pz);
	return v;
}

void draw_big_wheel(WorldEditor &editor, const ProcessedElement &element,
		const Site &site, int floor, double height, bool explicit_height, Block tagged)
{
	const double diameter =
			std::clamp(explicit_height	   ? std::max(double(height) - 4.0, 12.0)
					   : site.length > 0.0 ? std::max(site.length, 12.0)
										   : 30.0,
					12.0, 150.0);
	const double radius = diameter * 0.5;
	const double half_gap = diameter >= 50.0 ? 3.0 : 2.0;
	const int cabin_half = static_cast<int>(half_gap - 1.0);
	const auto frame =
			structure_kit::Frame::from_heading(site.x, site.z, site.ux, site.uz);
	const auto at = [&](double a, double b, double y) {
		const auto [x, z] = frame.world(a, b);
		return std::array<int, 3>{static_cast<int>(std::floor(x)),
				static_cast<int>(std::floor(y)), static_cast<int>(std::floor(z))};
	};
	const int cabin_height = diameter >= 50.0 ? 4 : 3;
	const double hub_y = floor + 3.0 + cabin_height + radius;
	const Block rim = tagged == AIR ? WHITE_CONCRETE : tagged;
	const auto steps = static_cast<std::size_t>(std::ceil(TAU * radius * 2.5));
	for (const double side : {-half_gap, half_gap}) {
		for (std::size_t k = 0; k < steps; ++k) {
			const double t = TAU * k / steps;
			const auto [x, y, z] =
					at(radius * std::cos(t), side, hub_y + radius * std::sin(t));
			editor.set_block_absolute(k % 9 == 0 ? SEA_LANTERN : rim, x, y, z);
		}
	}
	const auto count = static_cast<std::size_t>(
			std::clamp(std::round(TAU * radius / 6.0), 8.0, 48.0));
	static const Block cabin_colours[] = {RED_CONCRETE, ORANGE_CONCRETE, YELLOW_CONCRETE,
			LIME_CONCRETE, LIGHT_BLUE_CONCRETE, BLUE_CONCRETE, PURPLE_CONCRETE,
			PINK_CONCRETE};
	// Rust keeps this seed as signed bits and increments it per beam. Do the
	// arithmetic in unsigned space so the wraparound is defined in C++ too.
	std::uint64_t beam_seed =
			structure_kit::mix(element.id(), 0x52'49'44'45ULL) * std::uint64_t{31};
	for (std::size_t k = 0; k < count; ++k) {
		const double t = TAU * (k + 0.25) / count;
		const double a = radius * std::cos(t), gondola_y = hub_y + radius * std::sin(t);
		for (const double side : {-half_gap, half_gap}) {
			const auto [x0, z0] = frame.world(0.0, side);
			const auto [x1, z1] = frame.world(a, side);
			structure_kit::beam(editor, {x0, hub_y + 0.5, z0}, {x1, gondola_y + 0.5, z1},
					0.18, "minecraft:white_concrete",
					std::bit_cast<std::int64_t>(++beam_seed), IRON_BARS, IRON_BARS);
		}
		for (int b = -static_cast<int>(half_gap); b <= static_cast<int>(half_gap); ++b) {
			const auto [x, y, z] = at(a, b, gondola_y);
			editor.set_block_absolute(rim, x, y, z);
		}
		const Block colour = cabin_colours[k % std::size(cabin_colours)];
		const int roof = static_cast<int>(std::floor(gondola_y)) - 1;
		const auto [suspension_x, suspension_y, suspension_z] = at(a, 0.0, gondola_y);
		editor.set_block_absolute(CHAIN_Y, suspension_x, roof, suspension_z);
		const int floor_y = roof - cabin_height;
		const auto cells = frame.cells(a - cabin_half - 0.5, a + cabin_half + 0.5,
				-cabin_half - 0.5, cabin_half + 0.5);
		for (const auto &cell : cells) {
			const int x = cell.x, z = cell.z;
			const bool edge_a = std::abs(cell.a - a) > cabin_half - 0.5;
			const bool edge_b = std::abs(cell.b) > cabin_half - 0.5;
			editor.set_block_absolute(colour, x, roof - 1, z);
			editor.set_block_absolute(WHITE_CONCRETE, x, floor_y, z);
			if (!(edge_a || edge_b))
				continue;
			for (int y = floor_y + 1; y < roof - 1; ++y) {
				const bool post = edge_a && edge_b;
				const bool sill = y == floor_y + 1 && cabin_height >= 4;
				editor.set_block_absolute(
						post || sill ? colour : GRAY_STAINED_GLASS, x, y, z);
			}
		}
	}
	for (const auto &cell : frame.cells(-1.6, 1.6, -half_gap - 1.5, half_gap + 1.5))
		for (int y = static_cast<int>(std::floor(hub_y - 1.5));
				y <= static_cast<int>(std::floor(hub_y + 1.5)); ++y)
			editor.set_block_absolute(LIGHT_GRAY_CONCRETE, cell.x, y, cell.z);
	for (const double side : {-(half_gap + 1.0), half_gap + 1.0})
		for (const double direction : {-1.0, 1.0}) {
			const double foot = radius * 0.62;
			const int n = static_cast<int>(std::ceil(radius * 2.5));
			for (int k = 0; k <= n; ++k) {
				const double t = double(k) / n, a = direction * foot * t;
				const double y = hub_y - (hub_y - floor) * t;
				const auto [x, yy, z] = at(a, side, y);
				editor.set_block_absolute(WHITE_CONCRETE, x, yy, z);
				if (diameter >= 60.0) {
					const auto adjacent = at(a, side + std::copysign(1.0, side), y);
					editor.set_block_absolute(
							WHITE_CONCRETE, adjacent[0], yy, adjacent[2]);
				}
			}
		}
	for (const auto &cell : frame.cells(-2.5, 2.5, -half_gap - 1.5, half_gap + 1.5))
		editor.set_block_absolute(SMOOTH_STONE_SLAB, cell.x, floor + 1, cell.z);
}

void draw_drop_tower(WorldEditor &editor, const Site &site, const OwnCells &own,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &roads,
		std::uint64_t seed, int floor, double height, Block main, Block accent)
{
	const double requested_height =
			height > 0.0 ? height
						 : (site.length > 0.0 ? std::clamp(site.length * 3.0, 15.0, 60.0)
											  : 40.0);
	const int h =
			static_cast<int>(std::lround(std::clamp(requested_height, 12.0, 140.0)));
	const int x = static_cast<int>(std::floor(site.x)),
			  z = static_cast<int>(std::floor(site.z));
	const int half = h >= 70 ? 2 : 1, top = floor + h;
	for (int y = floor + 1; y <= top; ++y)
		for (int dx = -half; dx <= half; ++dx)
			for (int dz = -half; dz <= half; ++dz) {
				const bool corner = std::abs(dx) == half && std::abs(dz) == half;
				const Block b = (y - floor) % 10 < 2 ? main
								: corner			 ? WHITE_CONCRETE
													 : LIGHT_GRAY_CONCRETE;
				editor.set_block_absolute(b, x + dx, y, z + dz);
			}
	const int cap = half + 2;
	for (int dx = -cap; dx <= cap; ++dx)
		for (int dz = -cap; dz <= cap; ++dz)
			for (int y = top + 1; y <= top + 3; ++y) {
				const bool edge = std::abs(dx) == cap || std::abs(dz) == cap;
				const Block b =
						y == top + 2 && edge && (dx + dz) % 2 == 0 ? GLOWSTONE : accent;
				editor.set_block_absolute(b, x + dx, y, z + dz);
			}
	editor.set_block_absolute(LIGHTNING_ROD, x, top + 4, z);
	editor.set_block_absolute(REDSTONE_LAMP, x, top + 5, z);
	const int ring_y = top - std::max(h / 6, 4), ring = half + 1;
	for (int dx = -ring; dx <= ring; ++dx)
		for (int dz = -ring; dz <= ring; ++dz) {
			if (std::abs(dx) != ring && std::abs(dz) != ring)
				continue;
			const int px = x + dx, pz = z + dz;
			editor.set_block_absolute(main, px, ring_y - 1, pz);
			editor.set_block_absolute(main, px, ring_y + 2, pz);
			if (std::abs(dx) == ring && std::abs(dz) == ring) {
				editor.set_block_absolute(main, px, ring_y, pz);
				editor.set_block_absolute(main, px, ring_y + 1, pz);
				continue;
			}
			const auto outward = std::abs(dx) == ring ? std::pair{(dx > 0) - (dx < 0), 0}
													  : std::pair{0, (dz > 0) - (dz < 0)};
			structure_kit::place(editor,
					structure_kit::stair_backed(
							POLISHED_DEEPSLATE_STAIRS, {-outward.first, -outward.second}),
					px, ring_y, pz);
			editor.set_block_absolute(YELLOW_CONCRETE, px, ring_y + 1, pz);
		}
	fence_ring(editor, site, own, footprints, roads, seed, 7.0, floor);
}

using Cells = std::vector<std::pair<int, int>>;
std::uint64_t cell_key(int x, int z)
{
	return (std::uint64_t(std::uint32_t(x)) << 32) | std::uint32_t(z);
}
double distance_to(double cx, double cz, int x, int z)
{
	const double dx = x + 0.5 - cx, dz = z + 0.5 - cz;
	return std::sqrt(dx * dx + dz * dz);
}
double angle_of(double cx, double cz, int x, int z)
{
	return std::fmod(std::atan2(z + 0.5 - cz, x + 0.5 - cx) + TAU, TAU);
}
int level_ride(WorldEditor &editor, const Cells &cells, const OwnCells &own,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &roads,
		Block paving)
{
	std::vector<int> levels;
	levels.reserve(cells.size());
	for (const auto &[x, z] : cells)
		levels.push_back(
				editor.terrain_level(x, z).value_or(editor.get_ground_level(x, z)));
	if (levels.empty())
		return 0;
	auto mid = levels.begin() + levels.size() / 2;
	std::nth_element(levels.begin(), mid, levels.end());
	const int floor = *mid;
	for (const auto &[x, z] : cells) {
		if (roads.contains(x, z) ||
				(footprints.contains(x, z) && !own.contains(cell_key(x, z))))
			continue;
		editor.register_road_surface_y(x, z, floor);
		editor.set_block_absolute(paving, x, floor, z);
	}
	return floor;
}
void fence_ring(WorldEditor &editor, const Site &s, const OwnCells &own,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &roads,
		std::uint64_t seed, double radius, int floor)
{
	const double gate = (seed % 360) * PI / 180.0;
	for (const auto &[x, z] : disc(s.x, s.z, radius + 0.5)) {
		const double d = distance_to(s.x, s.z, x, z);
		if (d < radius - 0.5 || roads.contains(x, z) ||
				(footprints.contains(x, z) && !own.contains(cell_key(x, z))))
			continue;
		const double off = std::fmod(angle_of(s.x, s.z, x, z) - gate + TAU, TAU);
		if (off < 1.6 / radius || off > TAU - 1.6 / radius)
			continue;
		connected_blocks::place_connected(editor, IRON_BARS, x, floor + 1, z);
	}
}
Cells ride_disc(double x, double z, double r)
{
	Cells out;
	const int ri = int(std::ceil(r)) + 1, bx = int(std::floor(x)),
			  bz = int(std::floor(z));
	for (int px = bx - ri; px <= bx + ri; ++px)
		for (int pz = bz - ri; pz <= bz + ri; ++pz)
			if (distance_to(x, z, px, pz) <= r)
				out.emplace_back(px, pz);
	return out;
}

void draw_carousel(WorldEditor &e, const Site &s, const OwnCells &own,
		const BuildingFootprintBitmap &fp, const RoadMaskBitmap &roads,
		std::uint64_t seed, Block main, Block accent)
{
	const double r = std::clamp(s.length > 0 ? (s.length + s.width) / 4 : 7.0, 4.0, 14.0);
	const Cells pad = ride_disc(s.x, s.z, r + 1.5);
	const int floor = level_ride(e, pad, own, fp, roads, POLISHED_ANDESITE),
			  deck = floor + 1;
	const int eave = deck + 5, apex = eave + std::max(2, int(std::lround(r * 0.55)));
	const int sectors = std::clamp(int(std::lround(r * 1.6)), 8, 20) & ~1;
	for (const auto &[x, z] : ride_disc(s.x, s.z, r)) {
		const double d = distance_to(s.x, s.z, x, z);
		const int sector = int(angle_of(s.x, s.z, x, z) / TAU * sectors);
		const Block board =
				d > r - 1 ? GOLD_BLOCK : (sector % 2 ? BIRCH_PLANKS : SPRUCE_PLANKS);
		e.set_block_absolute(board, x, deck, z);
		const int roof_y = std::min(apex, eave + int(std::lround((r - d) * 0.55)));
		e.set_block_absolute(sector % 2 ? accent : main, x, roof_y, z);
		if (d > r - 1)
			e.set_block_absolute(
					(x + z) % 3 == 0 ? GLOWSTONE : GOLD_BLOCK, x, eave - 1, z);
		if (d <= 1.6)
			for (int y = deck + 1; y < roof_y; ++y)
				e.set_block_absolute(y == deck + 1 || y == eave - 1
											 ? GOLD_BLOCK
											 : (sector % 2 ? WHITE_CONCRETE : main),
						x, y, z);
	}
	const int ax = int(std::floor(s.x)), az = int(std::floor(s.z));
	e.set_block_absolute(GOLD_BLOCK, ax, apex + 1, az);
	e.set_block_absolute(LIGHTNING_ROD, ax, apex + 2, az);
	std::unordered_set<std::uint64_t> used;
	int placed_horses = 0;
	const int rings = r >= 9 ? 2 : 1;
	static const Block coats[] = {
			WHITE_WOOL, BROWN_TERRACOTTA, WHITE_WOOL, BLACK_WOOL, GRAY_WOOL};
	for (int ri = 0; ri < rings; ++ri) {
		const double ring = r - (ri == 0 ? 2.0 : 4.5);
		const int count = std::max(4, int(TAU * ring / 3.2));
		for (int h = 0; h < count; ++h) {
			const double a = TAU * (h + 0.5 * ri) / count;
			const int x = int(std::floor(s.x + ring * std::cos(a)));
			const int z = int(std::floor(s.z + ring * std::sin(a)));
			const auto tangent = structure_kit::axis_of(-std::sin(a), std::cos(a));
			const int nx = x + tangent.first, nz = z + tangent.second;
			// Preserve Rust's short-circuit insertion order: the body cell is
			// claimed before checking the head cell, even when that head collides.
			if (!used.insert(cell_key(x, z)).second ||
					!used.insert(cell_key(nx, nz)).second)
				continue;
			const int rise = (h + ri) % 2;
			const int roof = std::min(apex,
					eave + int(std::lround((r - distance_to(s.x, s.z, x, z)) * .55)));
			const Block coat = coats[(placed_horses + seed) % std::size(coats)];
			for (int y = deck + 1; y < roof; ++y)
				if (y != deck + 2 + rise)
					e.set_block_absolute(LIGHTNING_ROD, x, y, z);
			e.set_block_absolute(coat, x, deck + 2 + rise, z);
			structure_kit::place(e, structure_kit::stair_backed(QUARTZ_STAIRS, tangent),
					nx, deck + 2 + rise, nz);
			e.set_block_absolute(coat, nx, deck + 3 + rise, nz);
			++placed_horses;
		}
	}
	if (s.cells.empty())
		fence_ring(e, s, own, fp, roads, seed, r + 1.5, floor);
}

void draw_teacups(WorldEditor &e, const Site &s, const OwnCells &own,
		const BuildingFootprintBitmap &fp, const RoadMaskBitmap &roads,
		std::uint64_t seed, Block main)
{
	const double r = std::clamp(s.length > 0 ? (s.length + s.width) / 4 : 8.0, 5.0, 14.0);
	const int floor = level_ride(
			e, ride_disc(s.x, s.z, r + 1.5), own, fp, roads, POLISHED_ANDESITE);
	for (const auto &[x, z] : ride_disc(s.x, s.z, r))
		e.set_block_absolute(distance_to(s.x, s.z, x, z) > r - 1 ? main : WHITE_CONCRETE,
				x, floor + 1, z);
	const int cups = std::clamp(int(TAU * r * .55 / 4.5), 3, 8);
	for (int c = 0; c < cups; ++c) {
		const double a = TAU * c / cups, ux = s.x + r * .55 * std::cos(a),
					 uz = s.z + r * .55 * std::sin(a);
		const Block colour = RAINBOW[(c * 3 + seed) % std::size(RAINBOW)];
		for (const auto &[x, z] : ride_disc(ux, uz, 1.6)) {
			e.set_block_absolute(colour, x, floor + 2, z);
			if (distance_to(ux, uz, x, z) > .8)
				e.set_block_absolute(colour, x, floor + 3, z);
		}
		const int x = int(std::floor(ux)), z = int(std::floor(uz));
		e.set_block_absolute(OAK_FENCE, x, floor + 3, z);
		structure_kit::place(
				e, structure_kit::with_prop(OAK_SLAB, "type", "bottom"), x, floor + 4, z);
	}
	const int ax = int(std::floor(s.x)), az = int(std::floor(s.z));
	for (int y = floor + 2; y <= floor + 4; ++y)
		e.set_block_absolute(main, ax, y, az);
	e.set_block_absolute(GLOWSTONE, ax, floor + 5, az);
	fence_ring(e, s, own, fp, roads, seed, r + 1.0, floor);
}

void draw_swing_ride(WorldEditor &e, const Site &s, const OwnCells &own,
		const BuildingFootprintBitmap &fp, const RoadMaskBitmap &roads,
		std::uint64_t seed, Block main, Block accent, std::optional<double> tagged_height)
{
	const double r = std::clamp(s.length > 0 ? (s.length + s.width) / 4 : 8.0, 5.0, 14.0);
	const int floor = level_ride(
			e, ride_disc(s.x, s.z, r + 1.5), own, fp, roads, POLISHED_ANDESITE);
	const double crown_r = std::max(r * .8, 4.0);
	const int top = floor + std::clamp(tagged_height ? int(std::lround(*tagged_height))
													 : 9 + int(r * .4),
									8, 25);
	const int rim = top - 2, sectors = 16;
	for (const auto &[x, z] : ride_disc(s.x, s.z, crown_r + .5)) {
		const double d = distance_to(s.x, s.z, x, z), a = angle_of(s.x, s.z, x, z);
		const Block stripe = (int(a / TAU * sectors) % 2) ? accent : main;
		const int y = std::min(top + 1, rim + int(std::lround((crown_r - d) * .45)));
		e.set_block_absolute(stripe, x, y, z);
		if (d > crown_r - 1)
			e.set_block_absolute((x + z) % 3 == 0 ? GLOWSTONE : main, x, rim - 1, z);
		if (d <= 1.0)
			for (int cy = floor + 1; cy < rim; ++cy)
				e.set_block_absolute(
						(cy - floor) % 4 == 0 ? main : WHITE_CONCRETE, x, cy, z);
	}
	const double swing_r = crown_r - 1.2;
	const int count = std::max(6, int(TAU * swing_r / 2.5));
	std::unordered_set<std::uint64_t> used;
	for (int k = 0; k < count; ++k) {
		const double a = TAU * k / count;
		const int x = int(std::floor(s.x + swing_r * std::cos(a)));
		const int z = int(std::floor(s.z + swing_r * std::sin(a)));
		if (!used.insert(cell_key(x, z)).second)
			continue;
		const int crown =
				rim + int(std::lround((crown_r - distance_to(s.x, s.z, x, z)) * .45));
		for (int y = floor + 3; y < crown; ++y)
			e.set_block_absolute(CHAIN_Y, x, y, z);
		const auto outward = structure_kit::axis_of(std::cos(a), std::sin(a));
		structure_kit::place(e,
				structure_kit::stair_backed(
						DARK_OAK_STAIRS, {-outward.first, -outward.second}),
				x, floor + 2, z);
	}
	fence_ring(e, s, own, fp, roads, seed, r + 1.0, floor);
}

void draw_star_flyer(WorldEditor &e, const Site &s, const OwnCells &own,
		const BuildingFootprintBitmap &fp, const RoadMaskBitmap &roads,
		std::uint64_t seed, Block main, Block accent, std::optional<double> tagged_height)
{
	const int floor =
			level_ride(e, ride_disc(s.x, s.z, 9.5), own, fp, roads, POLISHED_ANDESITE);
	const int h = std::clamp(int(std::lround(tagged_height.value_or(60.0))), 30, 130);
	const int ax = int(std::floor(s.x)), az = int(std::floor(s.z)), top = floor + h;
	for (int y = floor + 1; y <= top; ++y) {
		const Block b = (y - floor) % 12 < 2 ? main : WHITE_CONCRETE;
		e.set_block_absolute(b, ax, y, az);
		if (y < top - 3)
			for (const auto &[dx, dz] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}})
				e.set_block_absolute(b, ax + dx, y, az + dz);
	}
	for (const auto &[x, z] : ride_disc(s.x, s.z, 2.6)) {
		for (int y = top - 2; y <= top; ++y)
			e.set_block_absolute(accent, x, y, z);
		e.set_block_absolute(GLOWSTONE, x, top + 1, z);
	}
	e.set_block_absolute(LIGHTNING_ROD, ax, top + 2, az);
	e.set_block_absolute(REDSTONE_LAMP, ax, top + 3, az);
	const int ring_y = top - int(std::lround(h * .18)) - 4;
	for (const auto &[x, z] : ride_disc(s.x, s.z, 8.0)) {
		const double d = distance_to(s.x, s.z, x, z);
		if (d <= 1.6)
			continue;
		const Block b = int(angle_of(s.x, s.z, x, z) / TAU * 16) % 2 ? accent : main;
		e.set_block_absolute(b, x, ring_y + (d > 6.5 ? 0 : 1), z);
		if (d > 6.5 && (x + z) % 3 == 0)
			e.set_block_absolute(GLOWSTONE, x, ring_y - 1, z);
	}
	for (int k = 0; k < 24; ++k) {
		const double a = TAU * k / 24, ux = std::cos(a), uz = std::sin(a);
		const structure_kit::Point3 from{s.x + ux * 7.0, double(ring_y), s.z + uz * 7.0};
		const structure_kit::Point3 to{
				from[0] + ux * 9.0 * .57, from[1] - 9.0 * .82, from[2] + uz * 9.0 * .57};
		structure_kit::beam(e, from, to, .06, "minecraft:gray_concrete",
				std::bit_cast<std::int64_t>(seed + static_cast<std::uint64_t>(k)),
				CHAIN_X, CHAIN_Y);
		const auto out = structure_kit::axis_of(ux, uz);
		structure_kit::place(e,
				structure_kit::stair_backed(DARK_OAK_STAIRS, {-out.first, -out.second}),
				int(std::floor(to[0])), int(std::floor(to[1])), int(std::floor(to[2])));
	}
	fence_ring(e, s, own, fp, roads, seed, 9.0, floor);
}

void draw_pirate_ship(WorldEditor &e, const Site &s, const OwnCells &own,
		const BuildingFootprintBitmap &fp, const RoadMaskBitmap &roads,
		std::uint64_t seed, Block main, Block accent)
{
	const double len = std::clamp(s.length > 0 ? s.length : 16.0, 10.0, 26.0);
	const double hl = len * .5, hw = len >= 16 ? 3.0 : 2.0;
	const auto f = structure_kit::Frame::from_heading(s.x, s.z, s.ux, s.uz);
	Cells pad;
	for (const auto &c : f.cells(-hl - 2, hl + 2, -hw - 4, hw + 4))
		pad.emplace_back(c.x, c.z);
	const int floor = level_ride(e, pad, own, fp, roads, POLISHED_ANDESITE);
	const int keel = floor + 2;
	const double pivot = keel + len * .6 + 2;
	for (const auto &c : f.cells(-hl, hl, -hw - .5, hw + .5)) {
		const int x = c.x, z = c.z;
		const double a = c.a, b = c.b, u = std::min(std::abs(a) / hl, 1.0);
		const int lift = int(std::lround(u * u * u * 4));
		const int bottom = keel + lift + (std::abs(b) > hw - 1 ? 1 : 0);
		const int gunwale = keel + 3 + lift;
		const bool side = std::abs(b) > hw - .5 || std::abs(a) > hl - 1;
		for (int y = bottom; y <= gunwale; ++y) {
			if (!side && y != bottom)
				continue;
			const Block block = y == gunwale && side	   ? accent
								: y == gunwale - 1 && side ? main
														   : DARK_OAK_PLANKS;
			e.set_block_absolute(block, x, y, z);
		}
		if (!side && int(std::lround(a)) % 2 == 0 && std::abs(a) > 1) {
			const auto back = structure_kit::axis_of(
					s.ux * (a < 0 ? -1 : 1), s.uz * (a < 0 ? -1 : 1));
			structure_kit::place(e, structure_kit::stair_backed(SPRUCE_STAIRS, back), x,
					bottom + 1, z);
		}
	}
	auto [bx, bz] = f.cell(hl + .5, 0);
	e.set_block_absolute(GOLD_BLOCK, bx, keel + 6, bz);
	auto [mx, mz] = f.cell(0, 0);
	for (int y = keel + 1; y < keel + 9; ++y)
		e.set_block_absolute(SPRUCE_LOG, mx, y, mz);
	for (const auto &c : f.cells(.5, 3.5, -.5, .5))
		for (int y = keel + 6; y < keel + 9; ++y)
			e.set_block_absolute(BLACK_WOOL, c.x, y, c.z);
	const double frame_b = hw + 2, foot = len * .45;
	for (double side : {-frame_b, frame_b}) {
		for (double dir : {-1.0, 1.0}) {
			const int n = std::max(1, int(std::ceil(std::max(pivot - floor, foot) * 2)));
			for (int k = 0; k <= n; ++k) {
				const double t = double(k) / n;
				auto [x, z] = f.cell(dir * foot * t, side);
				e.set_block_absolute(WHITE_CONCRETE, x,
						int(std::floor(pivot - (pivot - floor) * t)), z);
			}
			const double arm_b = side - std::copysign(1.0, side);
			const int an = std::max(
					1, int(std::ceil(std::max(pivot - (keel + 3), hl * .55) * 2)));
			for (int k = 0; k <= an; ++k) {
				const double t = double(k) / an;
				auto [x, z] = f.cell(dir * hl * .55 * t, arm_b);
				e.set_block_absolute(
						main, x, int(std::floor(pivot - (pivot - (keel + 3)) * t)), z);
			}
		}
	}
	for (const auto &c : f.cells(-.5, .5, -frame_b - .5, frame_b + .5))
		e.set_block_absolute(GRAY_CONCRETE, c.x, int(std::floor(pivot)), c.z);
	for (const auto &c : f.cells(-3, 3, hw + .5, hw + 1.5))
		e.set_block_absolute(SMOOTH_STONE_SLAB, c.x, floor + 1, c.z);
	(void)seed;
}

void draw_bumper_cars(WorldEditor &e, const Site &s, const OwnCells &own,
		const BuildingFootprintBitmap &fp, const RoadMaskBitmap &roads,
		std::uint64_t seed, Block main, Block accent)
{
	Cells cells = s.cells;
	if (cells.size() < 30) {
		const auto f = structure_kit::Frame::from_heading(s.x, s.z, s.ux, s.uz);
		for (const auto &c : f.cells(-8, 8, -5, 5))
			cells.emplace_back(c.x, c.z);
	}
	std::unordered_set<std::uint64_t> set;
	for (const auto &[x, z] : cells)
		set.insert(cell_key(x, z));
	const int floor = level_ride(e, cells, own, fp, roads, SMOOTH_STONE),
			  ceiling = floor + 5;
	auto edge = [&](int x, int z) {
		return !set.contains(cell_key(x + 1, z)) || !set.contains(cell_key(x - 1, z)) ||
			   !set.contains(cell_key(x, z + 1)) || !set.contains(cell_key(x, z - 1));
	};
	for (const auto &[x, z] : cells) {
		if (edge(x, z)) {
			e.set_block_absolute(BLACK_CONCRETE, x, floor + 1, z);
			if ((x + 2 * z) % 6 == 0)
				for (int y = floor + 2; y < ceiling; ++y)
					e.set_block_absolute(GRAY_CONCRETE, x, y, z);
			const int band_phase = ((x + z) % 4 + 4) % 4;
			const Block band = band_phase < 2 ? main : accent;
			e.set_block_absolute(band, x, ceiling, z);
			e.set_block_absolute((x + z) % 3 == 0 ? GLOWSTONE : band, x, ceiling + 1, z);
		} else {
			e.set_block_absolute(x % 4 == 0 && z % 4 == 0 ? SEA_LANTERN : GRAY_CONCRETE,
					x, ceiling, z);
		}
	}
	Cells inner;
	for (const auto &[x, z] : cells) {
		bool inside = true;
		for (int dx = -2; dx <= 2 && inside; ++dx)
			for (int dz = -2; dz <= 2; ++dz)
				if (!set.contains(cell_key(x + dx, z + dz))) {
					inside = false;
					break;
				}
		if (inside)
			inner.emplace_back(x, z);
	}
	if (inner.empty())
		return;
	OwnCells taken;
	const int wanted = std::clamp(int(inner.size() / 14), 3, 16);
	int placed = 0;
	for (std::size_t i = 0; placed < wanted && i < inner.size() * 2; ++i) {
		const auto pick = inner[structure_kit::mix(seed, i) % inner.size()];
		const auto dir = [&]() -> std::pair<int, int> {
			// Rust advances the placement counter after selecting `pick`, then uses
			// that incremented value for the direction roll.
			switch (structure_kit::mix(seed, 1001 + i) % 4) {
			case 0:
				return {1, 0};
			case 1:
				return {-1, 0};
			case 2:
				return {0, 1};
			default:
				return {0, -1};
			}
		}();
		const std::pair<int, int> front{pick.first + dir.first, pick.second + dir.second};
		bool conflict = false;
		for (int dx = -1; dx <= 1; ++dx)
			for (int dz = -1; dz <= 1; ++dz)
				conflict |= taken.contains(cell_key(pick.first + dx, pick.second + dz)) ||
							taken.contains(cell_key(front.first + dx, front.second + dz));
		if (conflict)
			continue;
		taken.insert(cell_key(pick.first, pick.second));
		taken.insert(cell_key(front.first, front.second));
		const Block colour = RAINBOW[(placed + seed) % std::size(RAINBOW)];
		e.set_block_absolute(colour, front.first, floor + 1, front.second);
		structure_kit::place(e,
				structure_kit::stair_backed(
						POLISHED_DEEPSLATE_STAIRS, {-dir.first, -dir.second}),
				pick.first, floor + 1, pick.second);
		for (int y = floor + 2; y < ceiling; ++y)
			e.set_block_absolute(IRON_BARS, pick.first, y, pick.second);
		++placed;
	}
}

void draw_spinner(WorldEditor &e, const Site &s, const OwnCells &own,
		const BuildingFootprintBitmap &fp, const RoadMaskBitmap &roads,
		std::uint64_t seed, Block main, Block accent)
{
	const double r = std::clamp(s.length > 0 ? (s.length + s.width) / 4 : 8.0, 4.0, 13.0);
	const int floor = level_ride(
			e, ride_disc(s.x, s.z, r + 1.5), own, fp, roads, POLISHED_ANDESITE);
	for (const auto &[x, z] : ride_disc(s.x, s.z, r)) {
		const double d = distance_to(s.x, s.z, x, z), a = angle_of(s.x, s.z, x, z);
		const Block b =
				d > r - 1 ? accent : (int(a / TAU * 12) % 2 ? WHITE_CONCRETE : main);
		e.set_block_absolute(b, x, floor + 1, z);
	}
	const int hub_top = floor + 6;
	for (const auto &[x, z] : ride_disc(s.x, s.z, 1.6))
		for (int y = floor + 2; y <= hub_top; ++y)
			e.set_block_absolute(y == hub_top ? GLOWSTONE : main, x, y, z);
	for (int k = 0; k < 6; ++k) {
		const double a = TAU * k / 6 + (seed % 60) * PI / 180.0;
		const double ux = std::cos(a), uz = std::sin(a), reach = r - 2.0;
		const int n = std::max(1, int(reach * 3));
		for (int i = 0; i <= n; ++i) {
			const double t = double(i) / n, d = 1.5 + (reach - 1.5) * t;
			const int x = int(std::floor(s.x + ux * d)),
					  z = int(std::floor(s.z + uz * d));
			e.set_block_absolute(accent, x, hub_top - 1 - int(std::lround(t * 2)), z);
		}
		const int ex = int(std::floor(s.x + ux * reach)),
				  ez = int(std::floor(s.z + uz * reach));
		const auto out = structure_kit::axis_of(ux, uz);
		const Block colour = RAINBOW[(k + seed) % std::size(RAINBOW)];
		for (const auto &[dx, dz] :
				{std::pair{0, 0}, std::pair{out.second, -out.first}}) {
			e.set_block_absolute(colour, ex + dx, floor + 2, ez + dz);
			structure_kit::place(e,
					structure_kit::stair_backed(
							POLISHED_DEEPSLATE_STAIRS, {-out.first, -out.second}),
					ex + dx - out.first, floor + 2, ez + dz - out.second);
		}
		e.set_block_absolute(CHAIN_Y, ex, floor + 3, ez);
	}
	fence_ring(e, s, own, fp, roads, seed, r + 1.0, floor);
}

} // namespace

bool claims(const tags_t &tags)
{
	return kind(tags).has_value();
}
bool replaces_parts(const tags_t &tags)
{
	auto r = kind(tags);
	return r && *r != Ride::Spinner;
}

bool generate_ride(WorldEditor &editor, const ProcessedElement &e,
		const FloodFillCache &fills, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &roads, double scale,
		const std::optional<std::chrono::milliseconds> &timeout)
{
	auto ride = kind(e.tags());
	if (!ride)
		return false;
	Site s = make_site(e, fills, timeout);
	if (!e.is_node() && e.nodes().empty())
		return false;
	int r = int(std::round((s.length + s.width) * 0.25));
	if (r <= 0)
		r = *ride == Ride::Wheel ? 12 : *ride == Ride::Tower ? 4 : 7;
	if (*ride == Ride::Spinner && s.length > 0 &&
			(s.length > 30 * scale || s.width < s.length * 0.55))
		return false;
	// Keep ownership tied to the mapped outline. Node rides have no owned polygon;
	// their generated pad must not make overlapping buildings look like the same feature.
	OwnCells own;
	for (auto [x, z] : s.cells)
		own.insert(cell_key(x, z));
	const auto centre = std::pair{
			static_cast<int>(std::floor(s.x)), static_cast<int>(std::floor(s.z))};
	const bool is_building =
			e.tags().contains("building") || e.tags().contains("building:part");
	if (footprints.contains(centre.first, centre.second) &&
			!(is_building && own.contains(cell_key(centre.first, centre.second))))
		return false;
	const auto [main, accent] =
			ride_colours(e.tags(), structure_kit::mix(e.id(), 0x52'49'44'45ULL));
	const auto seed = structure_kit::mix(e.id(), 0x52'49'44'45ULL);
	double h = 0.0;
	bool explicit_height = false;
	if (const auto *height = tag(e.tags(), "height"))
		if (const auto metres = mapillary::geodesy::parse_length_m(*height);
				metres && *metres > 1.0 && *metres < 400.0) {
			h = *metres * scale;
			explicit_height = true;
		}
	if (!explicit_height) {
		if (*ride == Ride::StarFlyer)
			h = 60.0;
		else if (*ride == Ride::Wheel)
			h = 2.0 * r;
		else if (*ride == Ride::Tower)
			h = s.length > 0.0 ? std::clamp(s.length * 3.0, 15.0, 60.0) : 40.0;
	}
	int floor = 0;
	if (*ride == Ride::Carousel) {
		draw_carousel(editor, s, own, footprints, roads, seed, main, accent);
	} else if (*ride == Ride::Teacups) {
		draw_teacups(editor, s, own, footprints, roads, seed, main);
	} else if (*ride == Ride::Swing) {
		draw_swing_ride(editor, s, own, footprints, roads, seed, main, accent,
				explicit_height ? std::optional<double>(h) : std::nullopt);
	} else if (*ride == Ride::StarFlyer) {
		draw_star_flyer(editor, s, own, footprints, roads, seed, main, accent,
				explicit_height ? std::optional<double>(h) : std::nullopt);
	} else if (*ride == Ride::Wheel) {
		const auto frame = structure_kit::Frame::from_heading(s.x, s.z, s.ux, s.uz);
		const double diameter = std::clamp(explicit_height ? std::max(h - 4.0, 12.0)
										   : s.length > 0  ? std::max(s.length, 12.0)
														   : 30.0,
				12.0, 150.0);
		const double gap = diameter >= 50 ? 3.0 : 2.0;
		Cells pad;
		for (const auto &cell :
				frame.cells(-diameter * .31 - 2, diameter * .31 + 2, -gap - 3, gap + 3))
			pad.emplace_back(cell.x, cell.z);
		floor = level_ride(editor, pad, own, footprints, roads, POLISHED_ANDESITE);
		const auto *wheel_colour = tag(e.tags(), "colour");
		if (!wheel_colour)
			wheel_colour = tag(e.tags(), "building:colour");
		const Block tagged_rim =
				wheel_colour
						? structure_kit::concrete_for_colour(*wheel_colour).value_or(AIR)
						: AIR;
		draw_big_wheel(editor, e, s, floor, h, explicit_height, tagged_rim);
	} else if (*ride == Ride::Tower) {
		floor = level_ride(editor, ride_disc(s.x, s.z, 7.5), own, footprints, roads,
				POLISHED_ANDESITE);
		draw_drop_tower(editor, s, own, footprints, roads, seed, floor, h, main, accent);
	} else if (*ride == Ride::Ship) {
		draw_pirate_ship(editor, s, own, footprints, roads, seed, main, accent);
	} else if (*ride == Ride::Bumper) {
		draw_bumper_cars(editor, s, own, footprints, roads, seed, main, accent);
	} else if (*ride == Ride::Spinner) {
		draw_spinner(editor, s, own, footprints, roads, seed, main, accent);
	}
	return true;
}

} // namespace arnis::attractions
