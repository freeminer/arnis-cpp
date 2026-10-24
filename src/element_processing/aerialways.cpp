#include "aerialways.h"

#include "../block_definitions.h"
#include "../bresenham.h"
#include "../clipping.h"
#include "connected_blocks.h"
#include "structure_kit.h"

#include "../../../arnis_world_editor.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cctype>
#include <cmath>
#include <iterator>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <string_view>

namespace arnis::aerialways
{
namespace
{
enum class Kind
{
	CableCar,
	Gondola,
	Mixed,
	Chair,
	TBar,
	JBar,
	Platter,
	RopeTow,
	MagicCarpet,
	ZipLine,
	Goods,
};

std::optional<Kind> kind_of(const tags_t &tags)
{
	const auto it = tags.find("aerialway");
	if (it == tags.end())
		return std::nullopt;
	const std::string_view value(it->second);
	if (value == "cable_car")
		return Kind::CableCar;
	if (value == "gondola")
		return Kind::Gondola;
	if (value == "mixed_lift")
		return Kind::Mixed;
	if (value == "chair_lift")
		return Kind::Chair;
	if (value == "drag_lift" || value == "t-bar")
		return Kind::TBar;
	if (value == "j-bar")
		return Kind::JBar;
	if (value == "platter")
		return Kind::Platter;
	if (value == "rope_tow")
		return Kind::RopeTow;
	if (value == "magic_carpet")
		return Kind::MagicCarpet;
	if (value == "zip_line")
		return Kind::ZipLine;
	if (value == "goods")
		return Kind::Goods;
	return std::nullopt;
}

void mark_band(
		CoordinateBitmap &map, double ax, double az, double bx, double bz, double half)
{
	const double dx = bx - ax, dz = bz - az;
	const double len2 = std::max(1e-9, dx * dx + dz * dz);
	const int r = static_cast<int>(std::ceil(half));
	// Rust's float-to-int casts truncate toward zero; keep the raster bounds
	// identical, including for the recursive samples of long diagonal lines.
	const int x0 = static_cast<int>(std::min(ax, bx)) - r;
	const int x1 = static_cast<int>(std::max(ax, bx)) + r;
	const int z0 = static_cast<int>(std::min(az, bz)) - r;
	const int z1 = static_cast<int>(std::max(az, bz)) + r;
	const double length = std::sqrt(len2);
	if (length > 2.0 && static_cast<double>(x1 - x0) * (z1 - z0) >
								4.0 * length * (2.0 * half + 2.0)) {
		const int steps = std::max(1, static_cast<int>(std::ceil(length)));
		for (int i = 0; i <= steps; ++i) {
			const double t = static_cast<double>(i) / steps;
			// Rust casts each sampled centre to i32 before rasterizing its local
			// radius. Preserve truncation toward zero here as well; retaining the
			// fractional centre changes corridor cells along long diagonal lifts.
			const int cx = static_cast<int>(ax + dx * t);
			const int cz = static_cast<int>(az + dz * t);
			mark_band(map, cx, cz, cx, cz, half);
		}
		return;
	}
	for (int x = x0; x <= x1; ++x)
		for (int z = z0; z <= z1; ++z) {
			const double t = std::clamp(((x - ax) * dx + (z - az) * dz) / len2, 0.0, 1.0);
			if (std::hypot(x - ax - t * dx, z - az - t * dz) <= half)
				map.set(x, z);
		}
}

struct Support
{
	double x{}, z{}, ground{}, rope_y{};
	enum class Role
	{
		Station,
		Pylon,
		Edge
	} role = Role::Pylon;
	bool inferred = false;
};

struct RopeSpec
{
	double station_rope = 5.5;
	double pylon_rope = 9.0;
	double min_rope = 6.0;
	double max_rope = 22.0;
	double hang = 4.5;
	double clearance = 2.0;
	double sag = 0.015;
	double spacing = 70.0;
};

double rope_gauge(Kind kind, const tags_t &tags);
double terminal_mouth(Kind kind, const tags_t &tags);

RopeSpec rope_spec(Kind kind)
{
	switch (kind) {
	case Kind::CableCar:
		return {8.5, 30.0, 14.0, 90.0, 8.5, 6.0, 0.02, 0.0};
	case Kind::Gondola:
	case Kind::Mixed:
		return {6.5, 13.0, 9.0, 45.0, 6.5, 3.0, 0.015, 110.0};
	case Kind::TBar:
	case Kind::JBar:
	case Kind::Platter:
		return {6.5, 6.5, 5.0, 11.0, 4.5, 1.0, 0.01, 55.0};
	case Kind::RopeTow:
		return {1.3, 1.3, 1.1, 2.0, 0.0, 0.2, 0.0, 20.0};
	case Kind::Goods:
		return {4.5, 10.0, 5.0, 30.0, 2.5, 2.0, 0.015, 120.0};
	case Kind::ZipLine:
		return {10.5, 10.5, 2.0, 10.5, 0.0, 0.0, 0.02, 0.0};
	case Kind::Chair:
		return {};
	case Kind::MagicCarpet:
		return {};
	}
	return {};
}

double ground_at(const WorldEditor &editor, double x, double z)
{
	const int bx = static_cast<int>(std::floor(x)), bz = static_cast<int>(std::floor(z));
	return editor.terrain_level(bx, bz).value_or(editor.get_ground_level(bx, bz));
}

double rope_at(const Support &a, const Support &b, double t, double sag)
{
	const double len = std::hypot(b.x - a.x, b.z - a.z);
	return a.rope_y + (b.rope_y - a.rope_y) * t - 4.0 * sag * len * t * (1.0 - t);
}

std::vector<Support> supports_for(
		const ProcessedWay &way, const WorldEditor &editor, Kind kind)
{
	struct Point
	{
		double x, z;
		Support::Role role;
	};
	std::vector<Point> points;
	const RopeSpec spec = rope_spec(kind);
	for (std::size_t i = 0; i < way.nodes.size(); ++i) {
		const auto &node = way.nodes[i];
		const bool endpoint = i == 0 || i + 1 == way.nodes.size();
		const auto tag = node.tags.find("aerialway");
		const bool edge = endpoint && clipping::is_invented_node_id(node.id);
		if (!endpoint && tag != node.tags.end() && tag->second != "pylon" &&
				tag->second != "station")
			continue;
		Support::Role role = edge ? Support::Role::Edge : Support::Role::Pylon;
		// The Rust planner treats each real way endpoint as a terminal. Interior
		// vertices become pylons unless explicitly marked aerialway=station.
		if (!edge && (endpoint || (tag != node.tags.end() && tag->second == "station")))
			role = Support::Role::Station;
		const double x = node.x + 0.5, z = node.z + 0.5;
		if (!points.empty() &&
				std::hypot(points.back().x - x, points.back().z - z) < 1.0 &&
				i + 1 != way.nodes.size())
			continue;
		points.push_back({x, z, role});
	}
	if (points.size() < 2)
		return {};
	std::vector<double> along(points.size(), 0.0);
	for (std::size_t i = 1; i < points.size(); ++i)
		along[i] = along[i - 1] + std::hypot(points[i].x - points[i - 1].x,
										  points[i].z - points[i - 1].z);
	std::size_t section_start = 0;
	for (std::size_t i = 1; i + 1 < points.size(); ++i) {
		if (points[i].role != Support::Role::Station)
			continue;
		if (along[i] - along[section_start] >= 40.0 && along.back() - along[i] >= 40.0)
			section_start = i;
	}
	std::vector<Support> supports;
	supports.reserve(points.size() + 8);
	for (const auto &point : points) {
		const double ground = ground_at(editor, point.x, point.z);
		const double height = point.role == Support::Role::Station ? spec.station_rope
							  : point.role == Support::Role::Pylon ? spec.pylon_rope
																   : spec.pylon_rope;
		supports.push_back(
				{point.x, point.z, ground, ground + height, point.role, false});
	}
	// Avoid placing a pylon immediately beside another support or inside a
	// terminal mouth. The Rust planner preserves stations while dropping only
	// the redundant intermediate supports.
	const double terminal_clearance = terminal_mouth(kind, way.tags) + 4.0;
	std::vector<Support> spaced;
	spaced.reserve(supports.size());
	for (std::size_t i = 0; i < supports.size(); ++i) {
		const auto &support = supports[i];
		if (support.role == Support::Role::Pylon && !spaced.empty()) {
			const auto &previous = spaced.back();
			const double to_previous =
					std::hypot(support.x - previous.x, support.z - previous.z);
			const bool crowded =
					to_previous <
					(previous.role == Support::Role::Station ? terminal_clearance : 8.0);
			// Rust checks the section's final support here, not only the next
			// vertex, so pylons anywhere inside the terminal mouth are dropped.
			const auto &end = supports.back();
			const bool near_station =
					i + 1 < supports.size() && end.role == Support::Role::Station &&
					std::hypot(support.x - end.x, support.z - end.z) < terminal_clearance;
			if (crowded || near_station)
				continue;
		}
		spaced.push_back(support);
	}
	supports = std::move(spaced);
	// Rust plans each station-to-station section independently. If that section
	// contains only its endpoints, add its regular supports even when another
	// section elsewhere on the same way has a mapped pylon or mid-station.
	if (spec.spacing > 0.0 && supports.size() >= 2) {
		std::vector<Support> planned;
		planned.reserve(supports.size() + 8);
		std::size_t section_start = 0;
		planned.push_back(supports.front());
		for (std::size_t end = 1; end < supports.size(); ++end) {
			const bool section_end = end + 1 == supports.size() ||
									 (supports[end].role == Support::Role::Station &&
											 end + 1 < supports.size());
			if (!section_end)
				continue;
			if (end == section_start + 1) {
				const Support &a = supports[section_start];
				const Support &b = supports[end];
				const double length = std::hypot(b.x - a.x, b.z - a.z);
				const int count = static_cast<int>(std::round(length / spec.spacing));
				for (int k = 1; k < count; ++k) {
					const double t = static_cast<double>(k) / count;
					const double x = a.x + (b.x - a.x) * t;
					const double z = a.z + (b.z - a.z) * t;
					const double ground = ground_at(editor, x, z);
					planned.push_back({x, z, ground, ground + spec.pylon_rope,
							Support::Role::Pylon, true});
				}
			}
			planned.push_back(supports[end]);
			section_start = end;
		}
		supports = std::move(planned);
	}
	const double gauge_half = rope_gauge(kind, way.tags) * 0.5;
	const double mouth = terminal_mouth(kind, way.tags);
	auto required_rope_y = [&](double x, double z, const Support &a, const Support &b) {
		const double dx = b.x - a.x;
		const double dz = b.z - a.z;
		const double length = std::max(1e-6, std::hypot(dx, dz));
		const double rx = -dz / length, rz = dx / length;
		double ground = ground_at(editor, x, z);
		if (gauge_half > 0.0) {
			ground = std::max(
					ground, ground_at(editor, x + rx * gauge_half, z + rz * gauge_half));
			ground = std::max(
					ground, ground_at(editor, x - rx * gauge_half, z - rz * gauge_half));
		}
		double nearest_station = std::numeric_limits<double>::max();
		for (const auto &support : supports)
			if (support.role == Support::Role::Station)
				nearest_station = std::min(
						nearest_station, std::hypot(x - support.x, z - support.z));
		const double ramp = std::clamp((nearest_station - mouth) / 20.0, 0.0, 1.0);
		return ground + spec.hang + spec.clearance * ramp;
	};
	auto worst_shortfall =
			[&](const Support &a,
					const Support &b) -> std::optional<std::pair<double, double>> {
		const double len = std::hypot(b.x - a.x, b.z - a.z);
		if (len < 1.0)
			return std::nullopt;
		const int steps = static_cast<int>(std::ceil(len));
		std::optional<std::pair<double, double>> worst;
		for (int k = 1; k < steps; ++k) {
			const double t = static_cast<double>(k) / steps;
			const double x = a.x + (b.x - a.x) * t;
			const double z = a.z + (b.z - a.z) * t;
			const double shortfall =
					required_rope_y(x, z, a, b) - rope_at(a, b, t, spec.sag);
			if (shortfall > 0.15 && (!worst || shortfall > worst->second))
				worst = std::make_pair(t, shortfall);
		}
		return worst;
	};
	auto span_has_shortfall = [&](const Support &a, const Support &b) {
		return worst_shortfall(a, b).has_value();
	};
	double total_length = 0.0;
	for (std::size_t i = 1; i < supports.size(); ++i)
		total_length += std::hypot(
				supports[i].x - supports[i - 1].x, supports[i].z - supports[i - 1].z);
	const std::size_t max_supports = 2 + static_cast<std::size_t>(total_length / 12.0);
	const bool can_insert = kind != Kind::ZipLine;
	// Match Rust's per-span worst-shortfall updates and inferred-support cap.
	for (int pass = 0; pass < 400; ++pass) {
		bool changed = false;
		std::size_t i = 0;
		while (i + 1 < supports.size()) {
			const auto shortfall = worst_shortfall(supports[i], supports[i + 1]);
			if (!shortfall) {
				++i;
				continue;
			}
			const auto [t, short_amount] = *shortfall;
			auto room = [&](const Support &support) {
				return support.role == Support::Role::Station
							   ? 0.0
							   : std::max(0.0,
										 support.ground + spec.max_rope - support.rope_y);
			};
			const double ra = room(supports[i]), rb = room(supports[i + 1]);
			const double wa = 1.0 - t, wb = t;
			double da = 0.0, db = 0.0;
			if (ra > 0.01 && rb > 0.01) {
				da = std::min(short_amount, ra);
				db = std::min(short_amount, rb);
			} else if (ra > 0.01) {
				da = std::min(short_amount / std::max(wa, 0.25), ra);
			} else if (rb > 0.01) {
				db = std::min(short_amount / std::max(wb, 0.25), rb);
			}
			supports[i].rope_y += da;
			supports[i + 1].rope_y += db;
			const double gained = da * wa + db * wb;
			if (da > 0.01 || db > 0.01)
				changed = true;
			if (gained + 0.1 < short_amount) {
				const double len = std::hypot(supports[i + 1].x - supports[i].x,
						supports[i + 1].z - supports[i].z);
				if (can_insert && supports.size() < max_supports && t * len >= 8.0 &&
						(1.0 - t) * len >= 8.0) {
					const auto &a = supports[i];
					const auto &b = supports[i + 1];
					const double x = a.x + (b.x - a.x) * t;
					const double z = a.z + (b.z - a.z) * t;
					const double ground = ground_at(editor, x, z);
					const double rope =
							std::clamp(rope_at(a, b, t, spec.sag) + short_amount,
									ground + spec.min_rope, ground + spec.max_rope);
					supports.insert(supports.begin() + i + 1,
							{x, z, ground, rope, Support::Role::Pylon, true});
					changed = true;
					continue;
				}
			}
			++i;
		}
		if (!changed)
			break;
	}
	for (std::size_t i = 1; i + 1 < supports.size();) {
		const auto &support = supports[i];
		if (support.inferred && support.role == Support::Role::Pylon) {
			const auto &a = supports[i - 1];
			const auto &b = supports[i + 1];
			const double whole = std::max(1e-6, std::hypot(b.x - a.x, b.z - a.z));
			const double t = std::hypot(support.x - a.x, support.z - a.z) / whole;
			const double chord = a.rope_y + (b.rope_y - a.rope_y) * t;
			if (chord - support.ground > spec.max_rope && !span_has_shortfall(a, b)) {
				supports.erase(supports.begin() + i);
				continue;
			}
		}
		++i;
	}
	for (auto &support : supports)
		if (support.role == Support::Role::Pylon)
			support.rope_y = std::ceil(support.rope_y) + 0.1;
	return supports;
}
Block lift_colour(const ProcessedWay &way)
{
	if (const auto it = way.tags.find("colour"); it != way.tags.end()) {
		if (const auto colour = structure_kit::concrete_for_colour(it->second))
			return *colour;
	}
	const Block colours[] = {RED_CONCRETE, RED_CONCRETE, WHITE_CONCRETE, YELLOW_CONCRETE,
			BLACK_CONCRETE, BLUE_CONCRETE, LIGHT_GRAY_CONCRETE, ORANGE_CONCRETE,
			LIGHT_BLUE_CONCRETE};
	return colours[structure_kit::mix(way.id, 0xA1) % std::size(colours)];
}

Block bubble_glass(Block colour)
{
	if (colour == ORANGE_CONCRETE || colour == RED_CONCRETE)
		return ORANGE_STAINED_GLASS;
	if (colour == YELLOW_CONCRETE)
		return YELLOW_STAINED_GLASS;
	if (colour == GREEN_CONCRETE || colour == LIME_CONCRETE)
		return LIME_STAINED_GLASS;
	return LIGHT_BLUE_STAINED_GLASS;
}

int occupancy(const tags_t &tags, int fallback)
{
	const auto it = tags.find("aerialway:occupancy");
	if (it == tags.end())
		return fallback;
	int value = 0;
	const auto *begin = it->second.data();
	const auto *end = begin + it->second.size();
	while (begin != end && std::isspace(static_cast<unsigned char>(*begin)))
		++begin;
	while (begin != end && std::isspace(static_cast<unsigned char>(end[-1])))
		--end;
	const auto parsed = std::from_chars(begin, end, value);
	return parsed.ec == std::errc{} && parsed.ptr == end && value >= 1 && value <= 250
				   ? value
				   : fallback;
}

double rope_gauge(Kind kind, const tags_t &tags)
{
	const int seats =
			occupancy(tags, kind == Kind::CableCar ? 80 : (kind == Kind::Chair ? 4 : 8));
	switch (kind) {
	case Kind::CableCar:
		return (seats <= 40 ? 5.0 : seats <= 110 ? 5.0 : 7.0) + 5.0;
	case Kind::Gondola:
		return (seats > 20 ? 5.0 : 3.0) + 3.0;
	case Kind::Mixed:
		return 8.0;
	case Kind::Chair:
		return (seats == 1 ? 1.0 : seats <= 4 ? 3.0 : 5.0) + 3.0;
	case Kind::TBar:
	case Kind::JBar:
	case Kind::Platter:
		return 3.0;
	case Kind::RopeTow:
		return 2.0;
	case Kind::Goods:
		return 3.0;
	case Kind::ZipLine:
	case Kind::MagicCarpet:
		return 0.0;
	}
	return 0.0;
}

double terminal_mouth(Kind kind, const tags_t &tags)
{
	if (kind == Kind::CableCar)
		return 8.0;
	if (kind == Kind::Gondola || kind == Kind::Mixed)
		return 9.0;
	if (kind == Kind::Chair) {
		const int seats = occupancy(tags, 4);
		const bool bubble =
				tags.contains("aerialway:bubble") && tags.at("aerialway:bubble") == "yes";
		const bool detachable = tags.contains("aerialway:detachable")
										? tags.at("aerialway:detachable") == "yes"
										: seats >= 4 || bubble;
		if (detachable)
			return 8.0;
	}
	return rope_gauge(kind, tags) / 2.0;
}

// Aerialway carriers are ordinary blocks, so keep their layout deterministic
// and visible to all backends rather than representing them as display entities.
void cabin_body(WorldEditor &editor, Block colour, int hx, int hz, int fx, int fz,
		int roof, int half_w, int half_l, int windows)
{
	const int right_x = -fz, right_z = fx;
	const int floor = roof - windows - 1;
	for (int f = -half_l; f <= half_l; ++f)
		for (int r = -half_w; r <= half_w; ++r) {
			const int x = hx + fx * f + right_x * r;
			const int z = hz + fz * f + right_z * r;
			const bool corner = std::abs(f) == half_l && std::abs(r) == half_w;
			const bool rim = std::abs(f) == half_l || std::abs(r) == half_w;
			if (!corner) {
				editor.set_block_absolute(colour, x, roof, z);
				editor.set_block_absolute(colour, x, floor, z);
			}
			if (!rim)
				continue;
			const bool post = corner || (std::abs(f) == half_l && r % 2 == 0 &&
												half_w > 1 && r != 0);
			for (int y = floor + 1; y < roof; ++y)
				editor.set_block_absolute(post ? colour : GRAY_STAINED_GLASS, x, y, z);
		}
}

void draw_carrier(WorldEditor &editor, Kind kind, Block colour, double x, double y,
		double z, double dx, double dz, int width, int length, bool bubble,
		std::int64_t seed)
{
	const auto [fx, fz] = structure_kit::axis_of(dx, dz);
	if (fx == 0 && fz == 0)
		return;
	const int cx = static_cast<int>(std::floor(x));
	const int cy = static_cast<int>(std::floor(y));
	const int cz = static_cast<int>(std::floor(z));
	const auto at = [&](int f, int r) {
		return std::pair{cx + fx * f - fz * r, cz + fz * f + fx * r};
	};
	const Block glass = bubble_glass(colour);
	if (kind == Kind::Chair) {
		for (int chain_y = cy - 2; chain_y <= cy; ++chain_y)
			editor.set_block_absolute(CHAIN_Y, cx, chain_y, cz);
		const std::pair<int, int> back{-fx, -fz};
		for (int r = -width; r <= width; ++r) {
			const auto [bx, bz] = at(0, r);
			editor.set_block_absolute(colour, bx, cy - 3, bz);
			editor.set_block_absolute(colour, bx, cy - 4, bz);
			const auto [sx, sz] = at(1, r);
			structure_kit::place(editor,
					structure_kit::stair_backed(POLISHED_DEEPSLATE_STAIRS, back), sx,
					cy - 4, sz);
		}
		if (bubble) {
			for (int r = -width; r <= width; ++r) {
				const auto [gx, gz] = at(1, r);
				const auto [fx2, fz2] = at(2, r);
				editor.set_block_absolute(glass, gx, cy - 2, gz);
				editor.set_block_absolute(glass, fx2, cy - 3, fz2);
			}
		} else {
			for (int r : {-width, width}) {
				const auto [bx, bz] = at(1, r);
				connected_blocks::place_connected(editor, IRON_BARS, bx, cy - 3, bz);
			}
			for (int r = -width; r <= width; ++r) {
				const auto [bx, bz] = at(2, r);
				connected_blocks::place_connected(editor, IRON_BARS, bx, cy - 3, bz);
				structure_kit::place(editor,
						structure_kit::top_slab(POLISHED_BLACKSTONE_SLAB), bx, cy - 5,
						bz);
			}
			const auto [bx, bz] = at(2, 0);
			editor.set_block_absolute(IRON_BARS, bx, cy - 4, bz);
		}
	} else if (kind == Kind::Gondola || kind == Kind::Mixed) {
		editor.set_block_absolute(CHAIN_Y, cx, cy, cz);
		editor.set_block_absolute(CHAIN_Y, cx, cy - 1, cz);
		cabin_body(editor, colour, cx, cz, fx, fz, cy - 2, width, length, 2);
	} else if (kind == Kind::CableCar) {
		for (int f = -2; f <= 2; ++f) {
			const auto [tx, tz] = at(f, 0);
			editor.set_block_absolute(POLISHED_DEEPSLATE, tx, cy, tz);
		}
		for (int f : {-1, 1}) {
			const auto [tx, tz] = at(f, 0);
			for (int yy = cy - 3; yy < cy; ++yy)
				editor.set_block_absolute(POLISHED_DEEPSLATE_WALL, tx, yy, tz);
		}
		cabin_body(editor, colour, cx, cz, fx, fz, cy - 4, width, length, 2);
	} else if (kind == Kind::TBar || kind == Kind::JBar || kind == Kind::Platter) {
		for (int chain_y = cy - 3; chain_y <= cy; ++chain_y)
			editor.set_block_absolute(CHAIN_Y, cx, chain_y, cz);
		if (kind == Kind::TBar) {
			for (int r = -width; r <= width; ++r) {
				const auto [bx, bz] = at(0, r);
				connected_blocks::place_connected(editor, IRON_BARS, bx, cy - 4, bz);
			}
		} else if (kind == Kind::JBar) {
			for (int r = 0; r <= 1; ++r) {
				const auto [bx, bz] = at(0, r);
				connected_blocks::place_connected(editor, IRON_BARS, bx, cy - 4, bz);
			}
		} else {
			structure_kit::place(editor,
					structure_kit::with_props(
							IRON_TRAPDOOR, {{"half", "top"}, {"open", "false"}}),
					cx, cy - 4, cz);
		}
	} else if (kind == Kind::Goods) {
		editor.set_block_absolute(CHAIN_Y, cx, cy, cz);
		editor.set_block_absolute(CHAIN_Y, cx, cy - 1, cz);
		editor.set_block_absolute(CAULDRON, cx, cy - 2, cz);
	}
	(void)seed;
}

std::size_t utf8_length(const std::string &text)
{
	return static_cast<std::size_t>(std::count_if(text.begin(), text.end(),
			[](unsigned char c) { return (c & 0xC0) != 0x80; }));
}

void place_lift_name(WorldEditor &editor, const ProcessedWay &way,
		const structure_kit::Frame &frame, double a, double b, int y)
{
	const auto name = way.tags.find("name");
	if (name == way.tags.end() || name->second.empty())
		return;
	std::istringstream words(name->second);
	std::vector<std::string> lines(1);
	std::string word;
	for (; words >> word;) {
		auto &line = lines.back();
		if (!line.empty() && utf8_length(line) + utf8_length(word) + 1 > 15) {
			if (lines.size() == 4)
				break;
			lines.emplace_back(word);
		} else {
			if (!line.empty())
				line.push_back(' ');
			line += word;
		}
	}
	std::string text;
	for (const auto &line : lines) {
		if (!text.empty())
			text.push_back('\n');
		text += line;
	}
	const auto [x, z] = frame.cell(a, b);
	const auto origin = frame.world(0.0, 0.0);
	const auto panel = frame.world(a + 1.0, b);
	const auto facing_axis = structure_kit::axis_of(
			panel.first - origin.first, panel.second - origin.second);
	const std::int8_t facing = facing_axis == std::pair{0, -1}	 ? 2
							   : facing_axis == std::pair{0, 1}	 ? 3
							   : facing_axis == std::pair{-1, 0} ? 4
																 : 5;
	editor.place_text_sign(x, y, z, facing, text, true);
}

void place_carriers_segment(WorldEditor &editor, const ProcessedWay &way, Kind kind,
		const std::vector<structure_kit::Point3> &supports,
		const std::vector<Support::Role> &roles, std::uint64_t segment_id)
{
	if (kind == Kind::MagicCarpet || kind == Kind::RopeTow || kind == Kind::ZipLine)
		return;
	const int seats = occupancy(
			way.tags, kind == Kind::CableCar ? 80 : (kind == Kind::Chair ? 4 : 8));
	const bool bubble = way.tags.contains("aerialway:bubble") &&
						way.tags.at("aerialway:bubble") == "yes";
	const bool detachable = way.tags.contains("aerialway:detachable")
									? way.tags.at("aerialway:detachable") == "yes"
									: seats >= 4 || bubble;
	const double gap = kind == Kind::CableCar ? 0.0
					   : kind == Kind::Chair  ? (detachable ? 36.0 : 18.0)
					   : kind == Kind::Mixed
							   ? 45.0
							   : (kind == Kind::Gondola ? (seats > 20 ? 80.0 : 54.0)
										 : kind == Kind::TBar || kind == Kind::JBar ? 12.0
										 : kind == Kind::Platter					? 10.0
										 : kind == Kind::Goods						? 30.0
															   : 0.0);
	const Block colour = lift_colour(way);
	const int carrier_width = kind == Kind::Chair	   ? (seats == 1		  ? 0
																 : seats <= 4 ? 1
																			  : 2)
							  : kind == Kind::CableCar ? (seats <= 40			? 2
																 : seats <= 110 ? 2
																				: 3)
							  : (kind == Kind::Gondola || kind == Kind::Mixed)
									  ? (seats > 20 ? 2 : 1)
							  : kind == Kind::TBar ? 1
												   : 0;
	const int carrier_length =
			kind == Kind::CableCar
					? (seats <= 40			 ? 2
							  : seats <= 110 ? 3
											 : 4)
					: ((kind == Kind::Gondola || kind == Kind::Mixed) && seats > 10 ? 2
																					: 1);
	const double rail_gauge = rope_gauge(kind, way.tags);
	const RopeSpec spec = rope_spec(kind);
	const bool enclosed = kind == Kind::CableCar || kind == Kind::Gondola ||
						  kind == Kind::Mixed || (kind == Kind::Chair && detachable);
	auto anchors_for_side = [&](double side) {
		std::vector<structure_kit::Point3> result;
		result.reserve(supports.size());
		for (std::size_t i = 0; i < supports.size(); ++i) {
			const std::size_t previous = i == 0 ? 0 : i - 1;
			const std::size_t next = std::min(i + 1, supports.size() - 1);
			double dx = supports[next][0] - supports[previous][0];
			double dz = supports[next][2] - supports[previous][2];
			const double length = std::max(1e-6, std::hypot(dx, dz));
			dx /= length;
			dz /= length;
			const double rx = -dz, rz = dx;
			double x = supports[i][0], z = supports[i][2];
			const double lateral = rail_gauge * 0.5 * side;
			if (roles[i] == Support::Role::Pylon) {
				x = std::floor(x + rx * lateral) + 0.5;
				z = std::floor(z + rz * lateral) + 0.5;
			} else {
				x += rx * lateral;
				z += rz * lateral;
				if (roles[i] == Support::Role::Station && enclosed) {
					const double inward = i == 0 ? 1.0 : -1.0;
					x += dx * inward * terminal_mouth(kind, way.tags);
					z += dz * inward * terminal_mouth(kind, way.tags);
				}
			}
			result.push_back({x, supports[i][1], z});
		}
		return result;
	};
	// Rust measures span lengths on the positive-side anchors and then places
	// the opposing carrier sequence at the same cumulative distances.
	const auto length_anchors = anchors_for_side(1.0);
	std::vector<double> lengths;
	double total = 0.0;
	for (std::size_t i = 1; i < length_anchors.size(); ++i) {
		const double len = std::hypot(length_anchors[i][0] - length_anchors[i - 1][0],
				length_anchors[i][2] - length_anchors[i - 1][2]);
		lengths.push_back(len);
		total += len;
	}
	if (total < 12.0)
		return;
	auto place_at = [&](double side, double distance, Kind carrier_kind,
							std::uint64_t identity, int width, int length,
							bool has_bubble) {
		const auto route = side == 1.0 ? length_anchors : anchors_for_side(side);
		double remaining = distance;
		for (std::size_t i = 0; i < lengths.size(); ++i) {
			if (remaining > lengths[i]) {
				remaining -= lengths[i];
				continue;
			}
			const auto keep = [&](Support::Role role) {
				return role == Support::Role::Station
							   ? terminal_mouth(kind, way.tags) + 3.0
					   : role == Support::Role::Pylon ? 3.0
													  : 0.0;
			};
			if (remaining < keep(roles[i]) || lengths[i] - remaining < keep(roles[i + 1]))
				return;
			const auto &a = route[i];
			const auto &b = route[i + 1];
			const double t = remaining / std::max(lengths[i], 1e-6);
			const double dx = b[0] - a[0], dz = b[2] - a[2];
			const double sag = spec.sag * lengths[i];
			const double x = a[0] + (b[0] - a[0]) * t;
			const double z = a[2] + (b[2] - a[2]) * t;
			const double y = a[1] + (b[1] - a[1]) * t - 4.0 * sag * t * (1.0 - t);
			const int cell_y = static_cast<int>(std::floor(y));
			const int bottom = cell_y - static_cast<int>(std::lround(spec.hang - 0.5));
			if (bottom <= ground_at(editor, x, z))
				return;
			const double travel = side >= 0.0 ? 1.0 : -1.0;
			draw_carrier(editor, carrier_kind, colour, x, y, z, dx * travel, dz * travel,
					width, length, has_bubble,
					static_cast<std::int64_t>(segment_id + identity));
			return;
		}
	};
	if (kind == Kind::CableCar) {
		place_at(1.0, total * 0.42, kind, 0, carrier_width, carrier_length, false);
		place_at(-1.0, total * 0.58, kind, 1, carrier_width, carrier_length, false);
		return;
	}
	for (const double side : {1.0, -1.0}) {
		double distance = side > 0.0 ? gap * 0.5 : gap;
		std::size_t index = 0;
		while (distance < total) {
			const bool mixed_chair = kind == Kind::Mixed && index % 3 == 2;
			place_at(side, distance, mixed_chair ? Kind::Chair : kind,
					index + (side < 0.0 ? 1 : 0), mixed_chair ? 2 : carrier_width,
					carrier_length, mixed_chair || bubble);
			distance += gap;
			++index;
		}
	}
}

void draw_enclosed_terminal(WorldEditor &editor, const ProcessedWay &way,
		std::uint64_t section_id, Kind kind, const structure_kit::Frame &frame, int floor,
		int rope, bool bottom, const CoordinateBitmap &footprints,
		const RoadMaskBitmap &road_mask)
{
	const bool tram = kind == Kind::CableCar;
	const double half_a = tram || kind == Kind::Chair ? 8.0 : 9.0;
	const double radius = rope_gauge(kind, way.tags) / 2.0;
	const double half_b = radius + (tram ? 4.0 : 2.5);
	const int roof = rope + (tram ? 3 : 2);
	const double apron = tram ? 1.0 : 1.5;
	for (const auto &cell : frame.cells(
				 -half_a - apron, half_a + apron, -half_b - apron, half_b + apron)) {
		const int x = static_cast<int>(cell[0]), z = static_cast<int>(cell[1]);
		if (footprints.contains(x, z) || road_mask.contains(x, z))
			continue;
		editor.register_road_surface_y(x, z, floor);
		editor.set_block_absolute(POLISHED_ANDESITE, x, floor, z);
	}
	for (const auto &cell : frame.cells(-half_a, half_a, -half_b, half_b)) {
		const int x = static_cast<int>(cell[0]), z = static_cast<int>(cell[1]);
		const double a = cell[2], b = cell[3];
		const bool edge_a = std::abs(a) > half_a - 1.0;
		const bool edge_b = std::abs(b) > half_b - 1.0;
		const bool wall = edge_a || edge_b;
		const bool entry = a > half_a - 1.0 && std::abs(b) < radius + 3.0;
		if (tram) {
			if (wall && !entry) {
				for (int y = floor + 1; y <= roof; ++y) {
					const Block block = y == floor + 1
												? LIGHT_GRAY_CONCRETE
												: ((y - floor) % 5 == 3 || y == rope
																  ? GLASS
																  : WHITE_CONCRETE);
					editor.set_block_absolute(block, x, y, z);
				}
				editor.set_block_absolute(WHITE_CONCRETE, x, roof + 1, z);
			} else if (!wall) {
				editor.set_block_absolute(LIGHT_GRAY_CONCRETE, x, roof, z);
				if ((x + z) % 5 == 0)
					editor.set_block_absolute(SEA_LANTERN, x, roof - 1, z);
			}
			continue;
		}
		const double edge_profile = 1.0 - std::pow(b / half_b, 2.0);
		const double end_profile =
				std::pow(std::max(0.0, std::abs(a) - (half_a - 3.0)) / 3.0, 2.0);
		const int cap = rope + 2 +
						static_cast<int>(std::lround(
								2.4 * std::max(0.0, edge_profile - end_profile)));
		for (int y = rope + 2; y <= cap; ++y)
			editor.set_block_absolute(WHITE_CONCRETE, x, y, z);
		if (wall) {
			editor.set_block_absolute(lift_colour(way), x, rope + 1, z);
			if (a < half_a - 1.0) {
				editor.set_block_absolute(LIGHT_GRAY_CONCRETE, x, rope, z);
				editor.set_block_absolute(LIGHT_GRAY_CONCRETE, x, rope - 1, z);
			}
		}
	}
	if (!tram) {
		// Housing's regular side columns and glazed back wall. The tram hall has
		// continuous walls instead, as in the Rust terminal renderer.
		for (const double a : {-half_a + 0.5, -half_a / 3.0, half_a / 3.0, half_a - 0.5})
			for (const double b : {-half_b + 0.5, half_b - 0.5}) {
				const auto [x, z] = frame.cell(a, b);
				for (int y = floor + 1; y < rope - 1; ++y)
					editor.set_block_absolute(GRAY_CONCRETE, x, y, z);
			}
		for (int b = -static_cast<int>(half_b); b <= static_cast<int>(half_b); ++b) {
			const auto [x, z] = frame.cell(-half_a + 0.5, b);
			for (int y = floor + 1; y < rope - 1; ++y)
				editor.set_block_absolute(LIGHT_GRAY_STAINED_GLASS, x, y, z);
		}
	}
	if (!tram) {
		// Bullwheel, spokes, paired carrier rails, and interior lighting.
		const double wheel_a = -(half_a - radius - 1.5);
		for (int k = 0; k < 48; ++k) {
			const double angle = 6.283185307179586 * k / 48.0;
			const auto [x, z] = frame.cell(
					wheel_a + radius * std::cos(angle), radius * std::sin(angle));
			editor.set_block_absolute(IRON_BLOCK, x, rope, z);
		}
		for (int d = 1; d < static_cast<int>(radius); ++d)
			for (const auto &[sa, sb] :
					{std::pair{1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0}}) {
				const auto [x, z] = frame.cell(wheel_a + sa * d, sb * d);
				structure_kit::place(
						editor, structure_kit::top_slab(SMOOTH_STONE_SLAB), x, rope, z);
			}
		const auto [wheel_x, wheel_z] = frame.cell(wheel_a, 0.0);
		structure_kit::place(editor, structure_kit::top_slab(SMOOTH_STONE_SLAB), wheel_x,
				rope, wheel_z);
		editor.set_block_absolute(POLISHED_DEEPSLATE, wheel_x, rope, wheel_z);
		editor.set_block_absolute(POLISHED_DEEPSLATE, wheel_x, rope + 1, wheel_z);
		for (double a = wheel_a; a <= half_a - 0.5; a += 0.5)
			for (const double side : {-radius, radius}) {
				const auto [x, z] = frame.cell(a, side);
				editor.set_block_absolute(SMOOTH_STONE_SLAB, x, rope + 1, z);
			}
		for (double a = -half_a + 2.5; a < half_a - 1.0; a += 4.0) {
			const auto [x, z] = frame.cell(a, 0.0);
			editor.set_block_absolute(SEA_LANTERN, x, rope + 1, z);
		}
	}
	if (!tram) {
		// Housing stations have loading rails and an operator booth; tram halls
		// use the open portal and rope anchors instead.
		for (const double b : {-half_b - 1.0, half_b + 1.0})
			for (double a = -half_a; a <= half_a - 3.0; a += 1.0) {
				const auto [x, z] = frame.cell(a, b);
				if (!road_mask.contains(x, z))
					connected_blocks::place_connected(editor, IRON_BARS, x, floor + 1, z);
			}
		const double booth_b = -(half_b + 3.5);
		const double booth_a0 = half_a - 4.0, booth_a1 = half_a - 1.0;
		for (const auto &cell :
				frame.cells(booth_a0, booth_a1, booth_b - 1.5, booth_b + 1.5)) {
			const int x = static_cast<int>(cell[0]), z = static_cast<int>(cell[1]);
			if (road_mask.contains(x, z) || footprints.contains(x, z))
				continue;
			const double a = cell[2], b = cell[3];
			const bool wall = std::abs(a - booth_a0) < 0.8 ||
							  std::abs(a - booth_a1) < 0.8 || std::abs(b - booth_b) > 0.8;
			editor.register_road_surface_y(x, z, floor);
			editor.set_block_absolute(POLISHED_ANDESITE, x, floor, z);
			if (wall) {
				editor.set_block_absolute(WHITE_CONCRETE, x, floor + 1, z);
				editor.set_block_absolute(GLASS, x, floor + 2, z);
			}
			editor.set_block_absolute(lift_colour(way), x, floor + 3, z);
		}
	}
	if (tram) {
		for (const double side : {-1.0, 1.0}) {
			const auto [ax, az] = frame.cell(-half_a + 1.5, side * radius);
			for (int y = floor + 1; y <= rope; ++y)
				editor.set_block_absolute(POLISHED_DEEPSLATE, ax, y, az);
			const auto p = frame.world(half_a, side * radius);
			const auto q = frame.world(-half_a + 1.5, side * radius);
			structure_kit::beam(editor, {p.first, rope + 0.5, p.second},
					{q.first, rope + 0.5, q.second}, 0.09, "minecraft:gray_concrete",
					static_cast<std::int64_t>(section_id * 7919 + (side > 0.0 ? 1 : 2)),
					CHAIN_X, CHAIN_X);
		}
	}
	place_lift_name(editor, way, frame, half_a - 0.5, tram ? half_b - 0.5 : -half_b + 0.5,
			tram ? rope + 2 : floor + 3);
	const int seats = occupancy(
			way.tags, kind == Kind::CableCar ? 80 : (kind == Kind::Chair ? 4 : 8));
	const int half_w = kind == Kind::CableCar ? (seats > 110 ? 3 : 2)
					   : kind == Kind::Chair  ? (seats == 1			 ? 0
														: seats <= 4 ? 1
																	 : 2)
					   : seats > 20			  ? 2
											  : 1;
	const int half_l = kind == Kind::CableCar				 ? (seats <= 40			  ? 2
																	   : seats <= 110 ? 3
																					  : 4)
					   : kind == Kind::Gondola && seats > 10 ? 2
															 : 1;
	const bool bubble = way.tags.contains("aerialway:bubble") &&
						way.tags.at("aerialway:bubble") == "yes";
	if (!tram) {
		for (const double side : {-1.0, 1.0}) {
			const auto [x, z] = frame.world(half_a - 3.0, side * radius);
			const double direction = bottom ? side : -side;
			draw_carrier(editor, kind, lift_colour(way), x, rope, z, frame.ux * direction,
					frame.uz * direction, half_w, half_l, kind == Kind::Chair && bubble,
					static_cast<std::int64_t>(section_id + (side > 0.0 ? 0 : 1)));
		}
	}
}

void draw_lift_terminal(WorldEditor &editor, const ProcessedWay &way,
		std::uint64_t section_id, Kind kind, const std::vector<Support> &supports,
		std::size_t i, bool bottom, double heading_x, double heading_z,
		const CoordinateBitmap &footprints, const RoadMaskBitmap &road_mask)
{
	const auto &support = supports[i];
	const auto frame = structure_kit::Frame::from_heading(
			support.x, support.z, heading_x, heading_z);
	const int floor = static_cast<int>(std::floor(support.ground));
	const int rope = static_cast<int>(std::floor(support.rope_y));
	const int seats = occupancy(
			way.tags, kind == Kind::CableCar ? 80 : (kind == Kind::Chair ? 4 : 8));
	const bool detachable =
			way.tags.contains("aerialway:detachable")
					? way.tags.at("aerialway:detachable") == "yes"
					: seats >= 4 || (way.tags.contains("aerialway:bubble") &&
											way.tags.at("aerialway:bubble") == "yes");
	const bool enclosed = kind == Kind::CableCar || kind == Kind::Gondola ||
						  kind == Kind::Mixed || (kind == Kind::Chair && detachable);
	if (enclosed) {
		draw_enclosed_terminal(editor, way, section_id, kind, frame, floor, rope, bottom,
				footprints, road_mask);
		return;
	}
	if (!enclosed) {
		const auto [cx, cz] = frame.cell(0.0, 0.0);
		for (int y = floor + 1; y < rope; ++y)
			editor.set_block_absolute(GRAY_CONCRETE, cx, y, cz);
		editor.set_block_absolute(POLISHED_DEEPSLATE, cx, rope, cz);
		editor.set_block_absolute(POLISHED_DEEPSLATE, cx, rope + 1, cz);
		const double radius = rope_gauge(kind, way.tags) / 2.0;
		const int steps = std::max(16, static_cast<int>(std::ceil(radius * 10.0)));
		for (int k = 0; k < steps; ++k) {
			const double angle = 6.283185307179586 * k / steps;
			const auto [x, z] =
					frame.cell(radius * std::cos(angle), radius * std::sin(angle));
			editor.set_block_absolute(IRON_BLOCK, x, rope, z);
		}
		structure_kit::place(
				editor, structure_kit::top_slab(SMOOTH_STONE_SLAB), cx, rope + 2, cz);
		for (int d = 1; d < static_cast<int>(std::ceil(radius)); ++d)
			for (const auto &[sa, sb] :
					{std::pair{1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0}}) {
				const auto [x, z] = frame.cell(sa * d, sb * d);
				structure_kit::place(
						editor, structure_kit::top_slab(SMOOTH_STONE_SLAB), x, rope, z);
			}
		if (bottom) {
			const double hut_a = -(radius + 4.0);
			for (const auto &cell : frame.cells(hut_a - 1.5, hut_a + 1.5, -2.5, 2.5)) {
				const int x = static_cast<int>(cell[0]), z = static_cast<int>(cell[1]);
				if (!footprints.contains(x, z) && !road_mask.contains(x, z)) {
					editor.register_road_surface_y(x, z, floor);
					editor.set_block_absolute(POLISHED_ANDESITE, x, floor, z);
				}
				const double a = cell[2], b = cell[3];
				const bool wall = std::abs(a - hut_a) > 0.8 || std::abs(b) > 1.8;
				if (wall) {
					editor.set_block_absolute(WHITE_CONCRETE, x, floor + 1, z);
					const Block upper = std::abs(b) < 0.8 || std::abs(a - hut_a) < 0.8
												? GLASS
												: WHITE_CONCRETE;
					editor.set_block_absolute(upper, x, floor + 2, z);
				}
				editor.set_block_absolute(lift_colour(way), x, floor + 3, z);
			}
			place_lift_name(editor, way, frame, hut_a + 1.5, 0.0, floor + 2);
			if (kind == Kind::Chair) {
				const auto [deck_x, deck_z] = frame.world((radius + 5.0) / 2.0, radius);
				const auto deck = structure_kit::Frame::from_heading(
						deck_x, deck_z, frame.ux, frame.uz);
				for (const auto &cell : deck.cells(
							 -(radius + 5.0) / 2.0, (radius + 5.0) / 2.0, -1.5, 1.5)) {
					const int x = static_cast<int>(cell[0]),
							  z = static_cast<int>(cell[1]);
					if (footprints.contains(x, z) || road_mask.contains(x, z))
						continue;
					editor.register_road_surface_y(x, z, floor);
					editor.set_block_absolute(SPRUCE_PLANKS, x, floor, z);
				}
			}
			return;
		}
		return;
	}
}

void generate_magic_carpet(WorldEditor &editor, const ProcessedWay &way)
{
	if (way.nodes.size() < 2)
		return;
	const auto covered_tag = way.tags.find("covered");
	const bool covered = covered_tag == way.tags.end() || covered_tag->second != "no";
	std::vector<std::pair<int, int>> belt;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		for (const auto &[x, y, z] : bresenham::bresenham_line(way.nodes[i - 1].x, 0,
					 way.nodes[i - 1].z, way.nodes[i].x, 0, way.nodes[i].z)) {
			(void)y;
			const std::pair<int, int> cell{x, z};
			if (belt.empty() || belt.back() != cell)
				belt.push_back(cell);
		}
	}
	if (belt.empty())
		return;
	const std::set<std::pair<int, int>> cells(belt.begin(), belt.end());
	const auto &first = way.nodes.front();
	const auto &last = way.nodes.back();
	const auto [ax, az] = structure_kit::axis_of(last.x - first.x, last.z - first.z);
	const int side_x = -az, side_z = ax;
	const auto bottom_slab =
			structure_kit::with_prop(SMOOTH_STONE_SLAB, "type", "bottom");
	for (const auto &[x, z] : belt) {
		const int ground = static_cast<int>(ground_at(editor, x, z));
		editor.set_block_absolute(BLACK_CONCRETE, x, ground, z);
		for (const int sign : {-1, 1}) {
			const int ex = x + side_x * sign, ez = z + side_z * sign;
			if (cells.contains(std::pair<int, int>{ex, ez}))
				continue;
			const int edge_ground = static_cast<int>(ground_at(editor, ex, ez));
			editor.set_block_absolute(SMOOTH_STONE, ex, edge_ground, ez);
			if (covered) {
				for (int y = ground + 1; y <= ground + 2; ++y)
					editor.set_block_absolute(WHITE_STAINED_GLASS, ex, y, ez);
				editor.set_block_absolute(WHITE_STAINED_GLASS, ex, ground + 3, ez);
			} else {
				structure_kit::place(editor, bottom_slab, ex, edge_ground + 1, ez);
			}
		}
		if (covered)
			editor.set_block_absolute(WHITE_STAINED_GLASS, x, ground + 3, z);
	}
}

void draw_zipline_terminal(
		WorldEditor &editor, const std::vector<Support> &supports, std::size_t i)
{
	if (supports.size() < 2 || i >= supports.size())
		return;
	const auto &support = supports[i];
	const auto &neighbor = supports[i == 0 ? 1 : i - 1];
	const auto [ax, az] =
			structure_kit::axis_of(neighbor.x - support.x, neighbor.z - support.z);
	const int cx = static_cast<int>(std::floor(support.x));
	const int cz = static_cast<int>(std::floor(support.z));
	const int ground = static_cast<int>(std::floor(support.ground));
	const int rope = static_cast<int>(std::floor(support.rope_y));
	if (i == 0) {
		const int deck = rope - 2;
		for (const auto &[dx, dz] : std::array<std::pair<int, int>, 4>{
					 {{-1, -1}, {-1, 1}, {1, -1}, {1, 1}}}) {
			const int x = cx + dx, z = cz + dz;
			const int base = static_cast<int>(ground_at(editor, x, z));
			for (int y = base + 1; y < deck; ++y)
				editor.set_block_absolute(SPRUCE_LOG, x, y, z);
		}
		for (int dx = -1; dx <= 1; ++dx)
			for (int dz = -1; dz <= 1; ++dz) {
				editor.set_block_absolute(SPRUCE_PLANKS, cx + dx, deck, cz + dz);
				if (std::abs(dx) == 1 || std::abs(dz) == 1)
					connected_blocks::place_connected(
							editor, SPRUCE_FENCE, cx + dx, deck + 1, cz + dz);
			}
		// Leave the climb/launch direction open and provide a ladder on the back.
		editor.set_block_absolute(AIR, cx + ax, deck + 1, cz + az,
				std::optional<std::vector<Block>>{std::vector<Block>{SPRUCE_FENCE}},
				std::nullopt);
		const int px = cx - ax, pz = cz - az;
		editor.set_block_absolute(AIR, px, deck + 1, pz,
				std::optional<std::vector<Block>>{std::vector<Block>{SPRUCE_FENCE}},
				std::nullopt);
		const int post_ground = static_cast<int>(ground_at(editor, px, pz));
		for (int y = post_ground + 1; y < deck; ++y)
			editor.set_block_absolute(SPRUCE_LOG, px, y, pz);
		const int lx = cx - ax * 2, lz = cz - az * 2;
		const int ladder_ground = static_cast<int>(ground_at(editor, lx, lz));
		const auto ladder = structure_kit::with_prop(
				LADDER, "facing", structure_kit::facing_name({-ax, -az}));
		for (int y = ladder_ground + 1; y <= deck; ++y)
			structure_kit::place(editor, ladder, lx, y, lz);
		for (int y = deck + 1; y <= deck + 2; ++y)
			editor.set_block_absolute(SPRUCE_LOG, cx, y, cz);
	} else {
		for (int y = ground + 1; y <= rope; ++y)
			editor.set_block_absolute(SPRUCE_LOG, cx, y, cz);
		const auto [sx, sz] =
				structure_kit::axis_of(support.x - neighbor.x, support.z - neighbor.z);
		for (const int side : {-1, 1})
			editor.set_block_absolute(HAY_BALE, cx + sx * 2 + side * -sz, ground + 1,
					cz + sz * 2 + side * sx);
	}
}

void draw_lattice_tower(WorldEditor &editor, Block tower, int cx, int cz, int base,
		int top, double tx, double tz)
{
	const auto frame = structure_kit::Frame::from_heading(cx + 0.5, cz + 0.5, tx, tz);
	const double height = top - base;
	const double foot = std::clamp(height / 7.0, 3.0, 7.0);
	for (int y = base + 1; y < top; ++y) {
		const double k = (y - base) / height;
		const double half = foot + (1.5 - foot) * k;
		for (const auto &[sa, sb] :
				{std::pair{1.0, 1.0}, {1.0, -1.0}, {-1.0, 1.0}, {-1.0, -1.0}}) {
			const auto [x, z] = frame.cell(sa * half, sb * half);
			editor.set_block_absolute(tower, x, y, z);
		}
		if ((y - base) % 6 != 0)
			continue;
		const std::array<std::array<double, 4>, 4> braces{
				{{{-half, -half, half, -half}}, {{-half, half, half, half}},
						{{-half, -half, -half, half}}, {{half, -half, half, half}}}};
		for (const auto &edge : braces) {
			const auto &[a0, b0, a1, b1] = edge;
			const auto [x0, z0] = frame.cell(a0, b0);
			const auto [x1, z1] = frame.cell(a1, b1);
			for (const auto &[x, z] :
					connected_blocks::four_connected_line({{x0, z0}, {x1, z1}}))
				editor.set_block_absolute(IRON_BARS, x, y, z);
		}
	}
	for (const auto &[sa, sb] :
			{std::pair{1.0, 1.0}, {1.0, -1.0}, {-1.0, 1.0}, {-1.0, -1.0}}) {
		const auto [x, z] = frame.cell(sa * foot, sb * foot);
		const int ground = static_cast<int>(ground_at(editor, x, z));
		for (int y = ground; y <= base; ++y)
			editor.set_block_absolute(POLISHED_ANDESITE, x, y, z);
	}
	editor.set_block_absolute(REDSTONE_LAMP, cx, top + 1, cz);
}

void draw_lift_pylon(WorldEditor &editor, const ProcessedWay &way, Kind kind,
		const std::vector<Support> &supports, std::size_t i)
{
	const auto &support = supports[i];
	const int cx = static_cast<int>(std::floor(support.x));
	const int cz = static_cast<int>(std::floor(support.z));
	const int base = static_cast<int>(support.ground);
	const int sheave_y = static_cast<int>(std::floor(support.rope_y)) - 1;
	const auto previous = i == 0 ? i : i - 1;
	const auto next = std::min(i + 1, supports.size() - 1);
	const double dx = supports[next].x - supports[previous].x;
	const double dz = supports[next].z - supports[previous].z;
	const auto [tx, tz] = structure_kit::axis_of(dx, dz);
	if (kind == Kind::RopeTow) {
		for (int y = base + 1; y <= base + 2; ++y)
			connected_blocks::place_connected(editor, SPRUCE_FENCE, cx, y, cz);
		return;
	}
	const int height = sheave_y - base;
	if (height < 2)
		return;
	const Block tower = kind == Kind::CableCar ? IRON_BLOCK : LIGHT_GRAY_CONCRETE;
	if (kind == Kind::CableCar && height >= 14) {
		draw_lattice_tower(editor, tower, cx, cz, base, sheave_y, tx, tz);
	} else {
		for (int ox = -1; ox <= 1; ++ox)
			for (int oz = -1; oz <= 1; ++oz) {
				const int x = cx + ox, z = cz + oz;
				const int ground = static_cast<int>(ground_at(editor, x, z));
				editor.set_block_absolute(POLISHED_ANDESITE, x, ground, z);
				if (ox == 0 && oz == 0)
					continue;
				for (int y = ground + 1; y <= base; ++y)
					editor.set_block_absolute(POLISHED_ANDESITE, x, y, z);
			}
		const int stout = height >= 16 ? base + height * 2 / 5 : base;
		for (int y = base + 1; y < sheave_y; ++y) {
			editor.set_block_absolute(tower, cx, y, cz);
			if (y <= stout)
				for (const auto &[ox, oz] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}})
					editor.set_block_absolute(tower, cx + ox, y, cz + oz);
		}
		const std::pair<int, int> back{-tx, -tz};
		const int from = height >= 16 ? stout + 1 : base + 2;
		for (int y = from; y < sheave_y; ++y)
			structure_kit::place(editor,
					structure_kit::with_prop(
							LADDER, "facing", structure_kit::facing_name(back)),
					cx + back.first, y, cz + back.second);
	}
	const double half_gauge = rope_gauge(kind, way.tags) / 2.0;
	for (const double side : {-1.0, 1.0}) {
		const double ax = cx + 0.5 - tz * half_gauge * side;
		const double az = cz + 0.5 + tx * half_gauge * side;
		const auto train = structure_kit::Frame::from_heading(ax, az, tx, tz);
		std::vector<std::pair<int, int>> cells;
		for (const double offset : {-1.2, -0.6, 0.0, 0.6, 1.2})
			cells.push_back(train.cell(offset, 0.0));
		for (const auto &[x, z] : connected_blocks::four_connected_line(cells))
			structure_kit::place(editor,
					structure_kit::top_slab(POLISHED_BLACKSTONE_SLAB), x, sheave_y, z);
	}
	const auto left = structure_kit::Frame::from_heading(cx + 0.5, cz + 0.5, tx, tz)
							  .cell(0.0, -half_gauge);
	const auto right = structure_kit::Frame::from_heading(cx + 0.5, cz + 0.5, tx, tz)
							   .cell(0.0, half_gauge);
	for (const auto &[x, z] : connected_blocks::four_connected_line({left, right}))
		editor.set_block_absolute(tower, x, sheave_y, z);
	editor.set_block_absolute(tower, cx, sheave_y, cz);
}
} // namespace

bool is_lift(const tags_t &tags)
{
	return kind_of(tags).has_value();
}

CoordinateBitmap clear_corridors(
		const std::vector<ProcessedElement> &elements, const XZBBox &bounds)
{
	CoordinateBitmap map;
	std::vector<std::pair<const ProcessedWay *, Kind>> lifts;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (way.nodes.size() < 2)
			continue;
		if (const auto kind = kind_of(way.tags);
				kind && *kind != Kind::MagicCarpet && *kind != Kind::RopeTow)
			lifts.emplace_back(&way, *kind);
	}
	if (lifts.empty())
		return map;
	map = CoordinateBitmap(bounds);
	for (const auto &[way, kind] : lifts) {
		const double half = rope_gauge(kind, way->tags) / 2.0 + 6.0;
		for (std::size_t i = 1; i < way->nodes.size(); ++i)
			mark_band(map, way->nodes[i - 1].x, way->nodes[i - 1].z, way->nodes[i].x,
					way->nodes[i].z, half);
		const double reach = std::max(terminal_mouth(kind, way->tags), 4.0) + 8.0;
		for (const auto *node : {&way->nodes.front(), &way->nodes.back()})
			mark_band(map, node->x, node->z, node->x, node->z, reach);
	}
	return map;
}

static void generate_aerialway_section(WorldEditor &editor, const ProcessedWay &way,
		std::uint64_t section_id, const CoordinateBitmap &building_footprints,
		const RoadMaskBitmap &road_mask)
{
	const auto kind = kind_of(way.tags);
	if (!kind || way.nodes.size() < 2)
		return;
	if (*kind == Kind::MagicCarpet) {
		generate_magic_carpet(editor, way);
		return;
	}
	auto supports = supports_for(way, editor, *kind);
	if (*kind == Kind::ZipLine && supports.size() >= 2) {
		if (supports.front().ground < supports.back().ground)
			std::reverse(supports.begin(), supports.end());
		supports.back().rope_y = supports.back().ground + 2.5;
	}
	double run_length = 0.0;
	for (std::size_t i = 1; i < supports.size(); ++i)
		run_length += std::hypot(
				supports[i].x - supports[i - 1].x, supports[i].z - supports[i - 1].z);
	if (supports.size() < 2 || run_length < 12.0)
		return;
	std::vector<structure_kit::Point3> anchors;
	anchors.reserve(supports.size());
	std::vector<double> support_along(supports.size(), 0.0);
	for (std::size_t i = 1; i < supports.size(); ++i)
		support_along[i] =
				support_along[i - 1] + std::hypot(supports[i].x - supports[i - 1].x,
											   supports[i].z - supports[i - 1].z);
	std::size_t section_start = 0;
	for (std::size_t i = 0; i < supports.size(); ++i) {
		const auto &support = supports[i];
		const int x = static_cast<int>(std::floor(support.x));
		const int z = static_cast<int>(std::floor(support.z));
		if (support.role != Support::Role::Pylon) {
			const auto previous = i == 0 ? i : i - 1;
			const auto next = std::min(i + 1, supports.size() - 1);
			const bool split = support.role == Support::Role::Station && i > 0 &&
							   i + 1 < supports.size() &&
							   support_along[i] - support_along[section_start] >= 40.0 &&
							   support_along.back() - support_along[i] >= 40.0;
			if (split)
				section_start = i;
			if (support.role == Support::Role::Station &&
					!building_footprints.contains(x, z) && *kind != Kind::ZipLine &&
					*kind != Kind::RopeTow) {
				if (split) {
					draw_lift_terminal(editor, way, section_id, *kind, supports, i, false,
							supports[previous].x - support.x,
							supports[previous].z - support.z, building_footprints,
							road_mask);
					draw_lift_terminal(editor, way, section_id, *kind, supports, i, true,
							supports[next].x - support.x, supports[next].z - support.z,
							building_footprints, road_mask);
				} else if (i == 0) {
					draw_lift_terminal(editor, way, section_id, *kind, supports, i, true,
							supports[next].x - support.x, supports[next].z - support.z,
							building_footprints, road_mask);
				} else if (i + 1 == supports.size()) {
					draw_lift_terminal(editor, way, section_id, *kind, supports, i, false,
							supports[previous].x - support.x,
							supports[previous].z - support.z, building_footprints,
							road_mask);
				} else {
					draw_lift_terminal(editor, way, section_id, *kind, supports, i, false,
							supports[previous].x - supports[next].x,
							supports[previous].z - supports[next].z, building_footprints,
							road_mask);
				}
			}
			if (!building_footprints.contains(x, z) && *kind == Kind::ZipLine &&
					support.role == Support::Role::Station &&
					(i == 0 || i + 1 == supports.size()))
				draw_zipline_terminal(editor, supports, i);
			if (!building_footprints.contains(x, z) &&
					support.role == Support::Role::Station && *kind == Kind::RopeTow) {
				const int base = static_cast<int>(std::floor(support.ground));
				for (int y = base + 1; y <= base + 2; ++y)
					editor.set_block_absolute(SPRUCE_LOG, x, y, z);
				editor.set_block_absolute(GRINDSTONE, x, base + 3, z);
			}
			anchors.push_back({support.x, support.rope_y, support.z});
			continue;
		}
		draw_lift_pylon(editor, way, *kind, supports, i);
		anchors.push_back({support.x, support.rope_y, support.z});
	}
	const RopeSpec line_spec = rope_spec(*kind);
	const double rail_gauge = rope_gauge(*kind, way.tags);
	const int rail_count = rail_gauge > 0.0 ? 2 : 1;
	const int seats = occupancy(
			way.tags, *kind == Kind::CableCar ? 80 : (*kind == Kind::Chair ? 4 : 8));
	const bool detachable =
			way.tags.contains("aerialway:detachable")
					? way.tags.at("aerialway:detachable") == "yes"
					: seats >= 4 || (way.tags.contains("aerialway:bubble") &&
											way.tags.at("aerialway:bubble") == "yes");
	const bool enclosed_terminal = *kind == Kind::CableCar || *kind == Kind::Gondola ||
								   *kind == Kind::Mixed ||
								   (*kind == Kind::Chair && detachable);
	for (std::size_t i = 1; i < supports.size(); ++i) {
		const auto &a = supports[i - 1];
		const auto &b = supports[i];
		const double len = std::hypot(b.x - a.x, b.z - a.z);
		const double ux = (b.x - a.x) / std::max(len, 1e-6);
		const double uz = (b.z - a.z) / std::max(len, 1e-6);
		const double rx = -uz, rz = ux;
		for (int rail = 0; rail < rail_count; ++rail) {
			const double side = rail_count == 1 ? 0.0 : (rail == 0 ? -1.0 : 1.0);
			const auto endpoint = [&](const Support &support, bool at_start) {
				double x = support.x + rx * rail_gauge * 0.5 * side;
				double z = support.z + rz * rail_gauge * 0.5 * side;
				if (support.role == Support::Role::Station && rail_gauge > 0.0 &&
						enclosed_terminal) {
					const double mouth =
							*kind == Kind::CableCar || *kind == Kind::Chair ? 8.0 : 9.0;
					x += (at_start ? ux : -ux) * mouth;
					z += (at_start ? uz : -uz) * mouth;
				} else if (support.role == Support::Role::Pylon) {
					x = std::floor(x) + 0.5;
					z = std::floor(z) + 0.5;
				}
				return structure_kit::Point3{x, support.rope_y, z};
			};
			const auto p0 = endpoint(a, true), p1 = endpoint(b, false);
			const double rail_len = std::hypot(p1[0] - p0[0], p1[2] - p0[2]);
			const int rail_chords =
					std::max(1, static_cast<int>(std::ceil(rail_len / 12.0)));
			for (int chord = 0; chord < rail_chords; ++chord) {
				const auto point = [&](double t) -> structure_kit::Point3 {
					return {p0[0] + (p1[0] - p0[0]) * t,
							p0[1] + (p1[1] - p0[1]) * t -
									4.0 * line_spec.sag * rail_len * t * (1.0 - t),
							p0[2] + (p1[2] - p0[2]) * t};
				};
				const auto from = point(static_cast<double>(chord) / rail_chords);
				const auto to = point(static_cast<double>(chord + 1) / rail_chords);
				if (building_footprints.contains(static_cast<int>(std::floor(from[0])),
							static_cast<int>(std::floor(from[2]))) &&
						building_footprints.contains(static_cast<int>(std::floor(to[0])),
								static_cast<int>(std::floor(to[2]))))
					continue;
				structure_kit::beam(editor, from, to, 0.09, "minecraft:gray_concrete",
						static_cast<std::int64_t>(
								section_id + i * 1000 + chord * 2 + rail),
						CHAIN_X, CHAIN_X);
			}
		}
	}
	if (enclosed_terminal && *kind != Kind::CableCar) {
		for (const std::size_t station_index : {std::size_t{0}, supports.size() - 1}) {
			const auto &station = supports[station_index];
			if (station.role != Support::Role::Station ||
					building_footprints.contains(static_cast<int>(std::floor(station.x)),
							static_cast<int>(std::floor(station.z))))
				continue;
			const auto &neighbor = supports[station_index == 0 ? 1 : station_index - 1];
			const auto frame = structure_kit::Frame::from_heading(
					station.x, station.z, neighbor.x - station.x, neighbor.z - station.z);
			const double half_length =
					*kind == Kind::CableCar || *kind == Kind::Chair ? 8.0 : 9.0;
			const double radius = rail_gauge / 2.0;
			const double wheel = -(half_length - radius - 1.5);
			std::vector<std::pair<double, double>> path{
					{half_length, radius}, {wheel, radius}};
			for (int k = 1; k < 12; ++k) {
				const double angle = 3.141592653589793 * k / 12.0;
				path.emplace_back(
						wheel - radius * std::sin(angle), radius * std::cos(angle));
			}
			path.emplace_back(wheel, -radius);
			path.emplace_back(half_length, -radius);
			for (std::size_t k = 1; k < path.size(); ++k) {
				const auto p = frame.world(path[k - 1].first, path[k - 1].second);
				const auto q = frame.world(path[k].first, path[k].second);
				structure_kit::beam(editor, {p.first, station.rope_y, p.second},
						{q.first, station.rope_y, q.second}, 0.09,
						"minecraft:gray_concrete",
						static_cast<std::int64_t>(
								section_id + 0xB011 + station_index * 32 + k),
						CHAIN_X, CHAIN_X);
			}
		}
	}
	std::vector<Support::Role> support_roles;
	support_roles.reserve(supports.size());
	for (std::size_t i = 0; i < supports.size(); ++i) {
		support_roles.push_back(supports[i].role);
	}
	place_carriers_segment(editor, way, *kind, anchors, support_roles, section_id);
}

void generate_aerialway(WorldEditor &editor, const ProcessedWay &way,
		const CoordinateBitmap &building_footprints, const RoadMaskBitmap &road_mask)
{
	const auto kind = kind_of(way.tags);
	if (!kind || way.nodes.size() < 2)
		return;
	if (*kind == Kind::MagicCarpet) {
		generate_magic_carpet(editor, way);
		return;
	}
	enum class Hint
	{
		Station,
		Pylon,
		Edge,
		Skip
	};
	struct Point
	{
		std::size_t node_index;
		double x, z;
		Hint hint;
	};
	std::vector<Point> points;
	const std::size_t last_node = way.nodes.size() - 1;
	for (std::size_t i = 0; i < way.nodes.size(); ++i) {
		const auto &node = way.nodes[i];
		const double x = node.x + 0.5, z = node.z + 0.5;
		if (!points.empty() &&
				std::hypot(points.back().x - x, points.back().z - z) < 1.0 &&
				i != last_node)
			continue;
		Hint hint;
		if ((i == 0 || i == last_node) && clipping::is_invented_node_id(node.id)) {
			hint = Hint::Edge;
		} else if (const auto tag = node.tags.find("aerialway");
				   tag == node.tags.end() || tag->second == "pylon") {
			hint = Hint::Pylon;
		} else if (node.tags.at("aerialway") == "station") {
			hint = Hint::Station;
		} else {
			hint = Hint::Skip;
		}
		points.push_back({i, x, z, hint});
	}
	if (points.size() < 2)
		return;
	const std::size_t last_point = points.size() - 1;
	std::vector<double> along(points.size(), 0.0);
	for (std::size_t i = 1; i < points.size(); ++i)
		along[i] = along[i - 1] + std::hypot(points[i].x - points[i - 1].x,
										  points[i].z - points[i - 1].z);
	if (along.back() < 12.0)
		return;
	if (*kind == Kind::ZipLine &&
			ground_at(editor, points.front().x, points.front().z) <
					ground_at(editor, points.back().x, points.back().z)) {
		std::reverse(points.begin(), points.end());
		std::reverse(along.begin(), along.end());
		const double total = along.front();
		for (double &distance : along)
			distance = total - distance;
	}

	std::size_t start = 0, section = 0;
	for (std::size_t i = 1; i < points.size(); ++i) {
		const bool mid_station = points[i].hint == Hint::Station &&
								 along[i] - along[start] >= 40.0 &&
								 along.back() - along[i] >= 40.0;
		if (i != last_point && !mid_station)
			continue;
		ProcessedWay segment = way;
		const auto section_id =
				section == 0 ? way.id : structure_kit::mix(way.id, section);
		segment.nodes.clear();
		segment.nodes.reserve(i - start + 1);
		for (std::size_t p = start; p <= i; ++p)
			segment.nodes.push_back(way.nodes[points[p].node_index]);
		generate_aerialway_section(
				editor, segment, section_id, building_footprints, road_mask);
		++section;
		start = i;
	}
}

} // namespace arnis::aerialways
