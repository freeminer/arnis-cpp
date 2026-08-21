#include "signage.h"
#include "../block_definitions.h"
#include "../decals/font.h"
#include "../decals/pictograms.h"
#include "../bresenham.h"
#include <charconv>
#include <set>
namespace arnis::signage
{
namespace
{
bool signed_road(const std::string &h)
{
	static const std::set<std::string> kinds{"motorway", "motorway_link", "trunk",
			"trunk_link", "primary", "primary_link", "secondary", "secondary_link",
			"tertiary", "tertiary_link", "unclassified", "residential", "living_street"};
	return kinds.contains(h);
}
decals::ShieldStyle shield(
		const std::string &highway, const std::string &ref, decals::SignRegion region)
{
	if (region == decals::SignRegion::NorthAmerica)
		return ref.starts_with('I') ? decals::ShieldStyle::Interstate
									: decals::ShieldStyle::White;
	if (region == decals::SignRegion::UkIreland)
		return ref.starts_with('M') ? decals::ShieldStyle::Blue
									: decals::ShieldStyle::Green;
	if (region == decals::SignRegion::Germanic)
		return ref.starts_with('A') || highway.starts_with("motorway")
					   ? decals::ShieldStyle::Blue
					   : decals::ShieldStyle::Yellow;
	return highway.starts_with("motorway") || ref.starts_with('A') || ref.starts_with('E')
				   ? decals::ShieldStyle::Blue
				   : decals::ShieldStyle::Yellow;
}
std::optional<std::pair<std::uint16_t, bool>> maxspeed(
		const tags_t &tags, decals::SignRegion region)
{
	auto raw = tags.get("maxspeed");
	std::size_t n = 0;
	while (n < raw.size() && std::isdigit((unsigned char)raw[n]))
		++n;
	if (!n)
		return std::nullopt;
	unsigned value = 0;
	auto result = std::from_chars(raw.data(), raw.data() + n, value);
	if (result.ec != std::errc{} || value == 0 || value > 200)
		return std::nullopt;
	bool mph = raw.find("mph") != std::string::npos ||
			   (raw.find("km") == std::string::npos && decals::default_mph(region));
	return std::pair{std::uint16_t(value), mph};
}

std::int8_t facing_for_dir(double dx, double dz)
{
	if (std::abs(dx) >= std::abs(dz))
		return dx >= 0.0 ? 5 : 4;
	return dz >= 0.0 ? 3 : 2;
}

std::int8_t opposite(std::int8_t f)
{
	switch (f) {
		case 2: return 3;
		case 3: return 2;
		case 4: return 5;
		case 5: return 4;
		default: return f;
	}
}

std::pair<int, int> right_dir(std::int8_t facing)
{
	// facing: 2=N, 3=S, 4=W, 5=E
	// right perpendicular direction
	switch (facing) {
		case 2: return {1, 0};   // N -> E
		case 3: return {-1, 0};  // S -> W  
		case 4: return {0, -1};  // W -> S
		case 5: return {0, 1};   // E -> N
		default: return {0, 1};
	}
}

std::optional<std::pair<int, int>> get_nearest_road_block(int x, int z, int radius,
		const RoadMaskBitmap &road_mask)
{
	// Search in expanding squares from the given position
	for (int r = 1; r <= radius; ++r) {
		for (int dz = -r; dz <= r; ++dz) {
			for (int dx = -r; dx <= r; ++dx) {
				if (std::abs(dx) != r && std::abs(dz) != r)
					continue; // skip inner square
				int nx = x + dx;
				int nz = z + dz;
				if (road_mask.contains(nx, nz)) {
					return {{nx, nz}};
				}
			}
		}
	}
	return std::nullopt;
}

void place_roadside_sign(world_editor::WorldEditor &editor, const char *kind,
		const std::vector<std::pair<int, int>> &cells, std::size_t idx,
		int offset, const decals::DecalKey &key, bool reverse,
		const RoadMaskBitmap &road_mask, const BuildingFootprintBitmap &footprints)
{
	(void)kind; (void)offset; (void)footprints;
	if (cells.size() < 3) return;
	idx = std::min(idx, cells.size() - 2);
	
	const int x = cells[idx].first;
	const int z = cells[idx].second;
	if (!editor.owns(x, z)) return;
	
	const int bx = cells[idx > 0 ? idx - 1 : 0].first;
	const int bz = cells[idx > 0 ? idx - 1 : 0].second;
	const int fx = cells[idx + 1 < cells.size() ? idx + 1 : cells.size() - 1].first;
	const int fz = cells[idx + 1 < cells.size() ? idx + 1 : cells.size() - 1].second;
	
	double dx = fx - bx, dz = fz - bz;
	const double len = std::hypot(dx, dz);
	if (len == 0.0) return;
	dx /= len; dz /= len;
	if (reverse) { dx = -dx; dz = -dz; }
	
	for (int k = 2; k <= 6; ++k) {
		const int sx = x + static_cast<int>(std::round(dz * k));
		const int sz = z + static_cast<int>(std::round(-dx * k));
		if (road_mask.contains(sx, sz))
			continue;
		if (footprints.contains(sx, sz) || editor.is_lc_water(sx, sz))
			continue;
			
		const int ground = editor.get_ground_level(sx, sz);
		editor.set_block_absolute(block_definitions::STONE_BRICK_WALL,
				sx, ground + 1, sz, std::nullopt, std::nullopt);
		editor.set_block_absolute(block_definitions::STONE_BRICK_WALL,
				sx, ground + 2, sz, std::nullopt, std::nullopt);
		editor.set_block_absolute(block_definitions::LIGHT_GRAY_CONCRETE,
				sx, ground + 3, sz, std::nullopt, std::nullopt);
		
		const std::int8_t facing = facing_for_dir(-dx, -dz);
		editor.place_decal_panel(sx, ground + 3, sz, facing, key, false, false);
		editor.place_decal_panel(sx, ground + 3, sz, opposite(facing), key, false, false);
		return;
	}
}

} // anonymous namespace
WaySigns highway_way_signs(const tags_t &tags, decals::SignRegion region)
{
	WaySigns out;
	const auto highway = tags.get("highway");
	if (tags.get("area") == "yes" ||
			(!tags.get("tunnel").empty() && tags.get("tunnel") != "no") ||
			tags.get("indoor") == "yes")
		return out;
	if (highway == "cycleway") {
		out.cycleway = decals::TrafficKey{decals::TrafficSign::Bicycle};
		return out;
	}
	if (!signed_road(highway))
		return out;
	if (tags.get("oneway") == "yes" || tags.get("oneway") == "-1")
		out.no_entry = decals::TrafficKey{decals::TrafficSign::NoEntry};
	if (auto speed = maxspeed(tags, region))
		out.speed = decals::SpeedLimitKey{
				speed->first, speed->second, decals::speed_style(region)};
	if (highway == "motorway" || highway == "trunk" || highway == "primary" ||
			highway == "secondary" || highway == "tertiary") {
		auto ref = tags.get("ref");
		if (auto semi = ref.find(';'); semi != std::string::npos)
			ref.resize(semi);
		if (ref.size() > 6)
			ref.resize(6);
		if (!ref.empty() && decals::font::supports(ref))
			out.shield = decals::RouteShieldKey{shield(highway, ref, region), ref};
	}
	return out;
}
std::optional<decals::DecalKey> highway_node_sign(const tags_t &tags, SignageLevel level)
{
	if (tags.get("railway") == "level_crossing")
		return decals::TrafficKey{decals::TrafficSign::LevelCrossing};
	const auto h = tags.get("highway");
	if (h == "stop")
		return decals::TrafficKey{decals::TrafficSign::Stop};
	if (h == "give_way")
		return decals::TrafficKey{decals::TrafficSign::GiveWay};
	if (h == "crossing" && level == SignageLevel::Full &&
			tags.get("crossing") != "unmarked" && tags.get("crossing") != "no" &&
			tags.get("crossing") != "traffic_signals")
		return decals::TrafficKey{decals::TrafficSign::Crossing};
	return std::nullopt;
}
std::optional<decals::DecalKey> power_sign(const tags_t &tags)
{
	const auto p = tags.get("power");
	return p == "substation" || p == "plant" || p == "generator" || p == "transformer"
				   ? std::optional<decals::DecalKey>{decals::TrafficKey{
							 decals::TrafficSign::HighVoltage}}
				   : std::nullopt;
}
std::vector<decals::DecalKey> advertising_keys(const tags_t &tags, std::uint64_t id)
{
	std::vector<decals::DecalKey> out;
	const auto a = tags.get("advertising");
	if (a == "billboard")
		out.emplace_back(decals::PosterKey{std::uint8_t(id % 6)});
	else if (a == "column")
		for (unsigned i = 0; i < 4; ++i)
			out.emplace_back(decals::ColumnPosterKey{std::uint8_t((id + i) % 5)});
	else if (a == "poster_box") {
		out.emplace_back(decals::ColumnPosterKey{std::uint8_t(id % 5)});
		out.emplace_back(decals::ColumnPosterKey{std::uint8_t((id + 2) % 5)});
	}
	return out;
}
std::optional<decals::DecalKey> information_key(const ProcessedNode &node)
{
	if (node.tags.get("tourism") != "information")
		return std::nullopt;
	const auto type = node.tags.get("information");
	if (type == "office" || type == "visitor_centre")
		return std::nullopt;
	if (type == "map" || type == "board" || type == "terminal")
		return decals::LocalMapKey{node.x, node.z};
	return decals::PictogramKey{"information"};
}
std::optional<decals::DecalKey> furniture_pictogram(const tags_t &tags)
{
	const auto a = tags.get("amenity");
	if (a == "recycling" || a == "waste_basket" || a == "waste_disposal")
		return decals::PictogramKey{"recycling"};
	if (a == "vending_machine")
		return decals::PictogramKey{"vending_machine"};
	if (a == "atm")
		return decals::PictogramKey{"atm"};
	if (tags.get("emergency") == "fire_hydrant" &&
			tags.get("fire_hydrant:type") != "underground" &&
			tags.get("fire_hydrant:type") != "wall" &&
			tags.get("fire_hydrant:type") != "pond")
		return decals::PictogramKey{"hydrant"};
	return std::nullopt;
}
std::shared_ptr<const decals::DecalRegistry> build_registry(
		const std::vector<ProcessedElement> &elements, SignageLevel level,
		decals::SignRegion region)
{
	if (level == SignageLevel::None)
		return {};
	std::set<decals::DecalKey> keys;
	for (const auto &element : elements) {
		if (element.is_way()) {
			const auto &way = element.as_way();
			if (level == SignageLevel::Full && !way.tags.contains("building:part")) {
				if (decals::pictograms::business_kind(way.tags)) {
					const auto name = way.tags.get("name");
					if (!name.empty() && decals::font::supports(name)) {
						const std::uint8_t cols = name.size() <= 26	  ? 2
												  : name.size() <= 44 ? 3
																	  : 4;
						keys.insert(decals::DecalKey::text(
								{decals::TextStyleKind::Fascia}, name, cols));
					}
				}
				const auto number = way.tags.get("addr:housenumber");
				if (!number.empty() && number.size() <= 8 &&
						decals::font::supports(number))
					keys.insert(decals::DecalKey::text(
							{decals::TextStyleKind::HouseNumber}, number, 1));
			}
			auto signs = highway_way_signs(way.tags, region);
			for (const auto *key :
					{&signs.speed, &signs.shield, &signs.no_entry, &signs.cycleway})
				if (*key)
					keys.insert(**key);
			for (auto &key : advertising_keys(way.tags, way.id))
				keys.insert(std::move(key));
			if (auto key = power_sign(way.tags))
				keys.insert(*key);
			if (way.tags.get("amenity") == "parking")
				keys.insert(decals::PictogramKey{"parking"});
		} else if (element.is_node()) {
			const auto &node = element.as_node();
			if (auto key = highway_node_sign(node.tags, level))
				keys.insert(*key);
			if (auto key = information_key(node))
				keys.insert(*key);
			if (auto key = furniture_pictogram(node.tags))
				keys.insert(*key);
			for (auto &key : advertising_keys(node.tags, node.id))
				keys.insert(std::move(key));
			const auto railway = node.tags.get("railway");
			if (railway == "station" || railway == "halt")
				keys.insert(decals::PictogramKey{"train"});
			else if (railway == "tram_stop")
				keys.insert(decals::PictogramKey{"tram"});
			else if (railway == "subway_entrance")
				keys.insert(decals::PictogramKey{decals::metro_logo(region)});
		}
	}
	return keys.empty() ? std::shared_ptr<const decals::DecalRegistry>{}
						: std::make_shared<const decals::DecalRegistry>(
								  decals::DecalRegistry::from_keys(keys));
}

bool place_post(world_editor::WorldEditor &editor, int x, int z,
		const decals::DecalKey &key, bool all_faces = false, bool glow = false)
{
	if (!editor.signage_enabled() || !editor.owns(x, z) ||
			!editor.decal_registry->contains(key))
		return false;
	const int head = editor.get_ground_level(x, z) + 3;
	editor.set_block_absolute(block_definitions::LIGHT_GRAY_CONCRETE, x, head, z,
			std::nullopt, std::nullopt);
	bool placed = false;
	for (const std::int8_t facing : std::array<std::int8_t, 4>{2, 3, 4, 5}) {
		placed |= editor.place_decal_panel(
				x, head, z, all_faces ? facing : 3, key, glow, false);
		if (!all_faces)
			break;
	}
	return placed;
}

void place_node_signage(world_editor::WorldEditor &editor, const ProcessedNode &node,
		SignageLevel level, decals::SignRegion region)
{
	if (!editor.signage_enabled() || !editor.owns(node.x, node.z))
		return;
	if (const auto key = highway_node_sign(node.tags, level)) {
		place_post(editor, node.x, node.z, *key);
		return;
	}
	if (const auto key = information_key(node)) {
		place_post(editor, node.x, node.z, *key, true);
		return;
	}
	if (const auto key = furniture_pictogram(node.tags)) {
		place_post(editor, node.x, node.z, *key, true);
		return;
	}
	const auto railway = node.tags.get("railway");
	if (railway == "station" || railway == "halt")
		place_post(editor, node.x, node.z, decals::PictogramKey{"train"}, true);
	else if (railway == "tram_stop")
		place_post(editor, node.x, node.z, decals::PictogramKey{"tram"}, true);
	else if (railway == "subway_entrance")
		place_post(editor, node.x, node.z,
				decals::PictogramKey{decals::metro_logo(region)}, true, true);

	const int ground = editor.get_ground_level(node.x, node.z);
	if (node.tags.get("advertising") == "column") {
		for (int face = 0; face < 4; ++face)
			editor.place_decal_panel(node.x, ground + 3, node.z, std::int8_t(face + 2),
					decals::ColumnPosterKey{std::uint8_t((node.id + face) % 5)});
	} else if (node.tags.get("advertising") == "poster_box") {
		editor.place_decal_panel(node.x + 1, ground + 3, node.z, 2,
				decals::ColumnPosterKey{std::uint8_t(node.id % 5)}, true);
		editor.place_decal_panel(node.x, ground + 3, node.z, 2,
				decals::ColumnPosterKey{std::uint8_t((node.id + 2) % 5)}, true);
	}
}

void place_way_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		decals::SignRegion region)
{
	if (!editor.signage_enabled() || way.nodes.empty())
		return;
	const auto signs = highway_way_signs(way.tags, region);
	const auto &node = way.nodes.front();
	for (const auto *key : {&signs.speed, &signs.shield, &signs.cycleway})
		if (*key && place_post(editor, node.x, node.z, **key))
			break;
	if (const auto key = power_sign(way.tags))
		place_post(editor, node.x, node.z, *key, true);
	if (way.tags.get("amenity") == "parking")
		place_post(editor, node.x, node.z, decals::PictogramKey{"parking"}, true);
}

// Building and POI signage implementation
namespace {

std::optional<std::string> get_name(const tags_t &tags)
{
	for (const char *key : {"name", "official:name"}) {
		auto it = tags.find(key);
		if (it != tags.end() && !it->second.empty())
			return it->second;
	}
	return std::nullopt;
}

std::optional<std::string> get_house_number(const tags_t &tags)
{
	auto it = tags.find("addr:housenumber");
	if (it != tags.end() && !it->second.empty())
		return it->second;
	it = tags.find("building:housenumber");
	if (it != tags.end() && !it->second.empty())
		return it->second;
	return std::nullopt;
}

} // anonymous namespace

NameSign poi_name(const tags_t &tags, SignageLevel level)
{
	NameSign result;
	if (auto name = get_name(tags)) {
		result.text = *name;
		(void)level;
	}
	return result;
}

NameSign house_number(const tags_t &tags)
{
	NameSign result;
	if (auto num = get_house_number(tags)) {
		result.text = *num;
	}
	return result;
}

void generate_building_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		std::optional<std::pair<int, int>> anchor,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &road_mask)
{
	if (!editor.signage_enabled())
		return;
	if (!anchor)
		return;
	if (way.tags.contains("building:part"))
		return;

	auto name = poi_name(way.tags, SignageLevel::Full);
	auto number = house_number(way.tags);

	if (name.text.empty() && number.text.empty())
		return;

	int ax = anchor->first;
	int az = anchor->second;
	int ground = editor.get_ground_level(ax, az);

	if (!name.text.empty()) {
		editor.place_text_sign(ax, ground + 3, az, 3, name.text, true);
	}
	if (!number.text.empty()) {
		editor.place_text_sign(ax + 1, ground + 2, az, 3, number.text, false);
	}
	(void)footprints;
	(void)road_mask;
}

void generate_node_signage(world_editor::WorldEditor &editor, const ProcessedNode &node,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &road_mask)
{
	if (!editor.signage_enabled())
		return;
	if (!editor.owns(node.x, node.z))
		return;

	const auto &tags = node.tags;

	if (tags.contains("shop") || tags.contains("amenity") ||
			tags.contains("office") || tags.contains("tourism") ||
			tags.contains("leisure") || tags.contains("healthcare") ||
			tags.contains("craft")) {
		if (footprints.contains(node.x, node.z)) {
			auto name = poi_name(tags, SignageLevel::Full);
			if (!name.text.empty()) {
				int ground = editor.get_ground_level(node.x, node.z);
				editor.place_text_sign(node.x, ground + 3, node.z, 3,
						name.text, true);
			}
		}
	}

	if (tags.contains("highway") || tags.get("railway") == "level_crossing") {
		place_node_signage(editor, node, SignageLevel::Full, decals::SignRegion::Europe);
	}

	if (tags.contains("railway")) {
		place_node_signage(editor, node, SignageLevel::Full, decals::SignRegion::Europe);
	}
}

void generate_power_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const RoadMaskBitmap &road_mask)
{
	if (!editor.signage_enabled())
		return;

	auto sign = power_sign(way.tags);
	if (!sign)
		return;

	if (way.nodes.empty())
		return;

	const auto &node = way.nodes.front();
	int x = node.x, z = node.z;

	if (road_mask.contains(x, z))
		return;
	if (!editor.owns(x, z))
		return;

	int ground = editor.get_ground_level(x, z);
	int head = ground + 3;

	editor.place_decal_panel(x, head, z, 3, *sign, true, false);
	editor.place_decal_panel(x, head, z, 5, *sign, true, false);
}
void generate_highway_way_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &road_mask)
{
	// Highway signage: speed limit signs, route shields, cycleway signs, no-entry and periodic signs
	if (!editor.signage_enabled() || way.nodes.size() < 2)
		return;

	// Skip elevated/deck/tunnel/layer!=0 highways
	const auto bridge = way.tags.get("bridge");
	const auto tunnel = way.tags.get("tunnel");
	const auto layer = way.tags.get("layer");
	if ((!bridge.empty() && bridge != "no") ||
			(!tunnel.empty() && tunnel != "no") ||
			(!layer.empty() && layer != "0"))
		return;

	// Skip area highways
	if (way.tags.get("area") == "yes")
		return;

	const std::string highway = way.tags.get("highway");
	const bool oneway = way.tags.get("oneway") == "yes";
	const bool reversed = way.tags.get("oneway") == "-1";

	// Build Bresenham line along the way
	std::vector<std::pair<int, int>> cells;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &from = way.nodes[i - 1];
		const auto &to = way.nodes[i];
		const auto line = bresenham::bresenham_line(from.x, 0, from.z, to.x, 0, to.z);
		for (const auto &pt : line)
			cells.emplace_back(std::get<0>(pt), std::get<2>(pt));
	}

	const auto signs = highway_way_signs(way.tags, decals::SignRegion::Europe);
	if (!signs.speed.has_value() && !signs.shield.has_value() &&
			!signs.no_entry.has_value() && !signs.cycleway.has_value())
		return;

	// Place cycleway signs (if cycleway, place just these and return)
	if (signs.cycleway.has_value()) {
		place_roadside_sign(editor, "cycleway signs", cells, 6, 0,
				signs.cycleway.value(), false, road_mask, footprints);
		return;
	}

	// Place speed limit signs at start and periodically (every 8 cells for short gaps)
	if (signs.speed.has_value()) {
		if (cells.size() >= 24) {
			place_roadside_sign(editor, "speed limits", cells, 8, 0,
					signs.speed.value(), false, road_mask, footprints);
			if (cells.size() >= 60) {
				place_roadside_sign(editor, "speed limits", cells,
						cells.size() - 9, 0, signs.speed.value(), true, road_mask, footprints);
			}
		}
	}

	// No-entry where wrong-way traffic would enter a one-way street
	if (signs.no_entry.has_value() && cells.size() >= 10) {
		if (reversed && !oneway) {
			place_roadside_sign(editor, "no-entry signs", cells, 4, 0,
					signs.no_entry.value(), false, road_mask, footprints);
		} else if (oneway) {
			place_roadside_sign(editor, "no-entry signs", cells,
					cells.size() - 5, 0, signs.no_entry.value(), true, road_mask, footprints);
		}
	}

	// Reassurance shields: thinned so a road split into many OSM ways is not carpeted
	if (signs.shield.has_value() && cells.size() >= 40) {
		if (cells.size() >= 120 || way.id % 3 == 0)
			place_roadside_sign(editor, "route shields", cells,
					std::min<std::size_t>(40, cells.size() / 2), 0, signs.shield.value(),
					false, road_mask, footprints);

		std::size_t pos = 250;
		while (pos + 30 < cells.size()) {
			place_roadside_sign(editor, "route shields", cells, pos, 0,
					signs.shield.value(), false, road_mask, footprints);
			pos += 250;
		}
	}

	(void)way.tags;
	(void)footprints;
}

void generate_parking_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const RoadMaskBitmap &road_mask)
{
	// Parking lot: a "P" post at the first outline node, off the road.
	if (!editor.signage_enabled())
		return;

	const auto key = decals::PictogramKey{"parking"};
	if (!editor.decal_registry || !editor.decal_registry->contains(key))
		return;

	if (way.nodes.size() < 3)
		return;

	const auto &node = way.nodes.front();
	const int x = node.x;
	const int z = node.z;

	// Don't place on carriageway
	if (road_mask.contains(x, z))
		return;

	if (!editor.owns(x, z))
		return;

	const int ground = editor.get_ground_level(x, z);
	const int head = ground + 3;

	// Place sign post
	editor.set_block_absolute(block_definitions::STONE_BRICK_WALL, x, ground + 1, z,
			std::nullopt, std::nullopt);
	editor.set_block_absolute(block_definitions::STONE_BRICK_WALL, x, ground + 2, z,
			std::nullopt, std::nullopt);
	editor.set_block_absolute(block_definitions::LIGHT_GRAY_CONCRETE, x, head, z,
			std::nullopt, std::nullopt);

	// Find nearest road and face toward it
	std::int8_t facing = 3; // default south
	for (int dist = 0; dist <= 10; ++dist) {
		for (int dx = -dist; dx <= dist; ++dx) {
			for (int dz = -dist; dz <= dist; ++dz) {
				if (std::max(std::abs(dx), std::abs(dz)) != dist)
					continue;
				const int rx = x + dx;
				const int rz = z + dz;
				if (road_mask.contains(rx, rz)) {
					// Calculate facing based on direction to road
					if (dx != 0 || dz != 0) {
						if (std::abs(dx) >= std::abs(dz)) {
							facing = dx >= 0 ? 5 : 4;
						} else {
							facing = dz >= 0 ? 3 : 2;
						}
					}
					goto place_signs;
				}
			}
		}
	}

place_signs:
	// Place parking pictogram on both sides
	editor.place_decal_panel(x, head, z, facing, key, false, false);
	editor.place_decal_panel(x, head, z, facing == 2 ? 3 : (facing == 3 ? 2 :
			(facing == 4 ? 5 : 4)), key, false, false);
}

bool generate_billboard(WorldEditor &editor, const ProcessedNode &node,
		const RoadMaskBitmap &road_mask)
{
	if (!editor.signage_enabled())
		return false;

	decals::DecalKey key = decals::PosterKey{static_cast<std::uint8_t>(node.id % 6)};
	if (!editor.decal_registry->contains(key))
		return false;

	int x = node.x;
	int z = node.z;

	// Face the nearest road; the panel then runs perpendicular to that direction.
	std::int8_t facing = 3;
	auto road_pos = get_nearest_road_block(x, z, 20, road_mask);
	if (road_pos) {
		facing = facing_for_dir(road_pos->first - x, road_pos->second - z);
	}

	auto [rx, rz] = right_dir(facing);
	int ground = editor.get_ground_level(x, z);
	int height = 6;
	auto it_height = node.tags.find("height");
	if (it_height != node.tags.end()) {
		std::string h_str = it_height->second;
		size_t end_pos = h_str.find('m');
		if (end_pos != std::string::npos)
			h_str = h_str.substr(0, end_pos);
		try {
			double h_val = std::stod(h_str) * 100.0;
			height = static_cast<int>(std::round(h_val));
		} catch (...) {
			height = 6;
		}
	}
	height = std::clamp(height, 4, 12);
	int top = ground + height;

	for (int k : {-1, 1}) {
		for (int dy = 1; dy <= height - 2; ++dy) {
			editor.set_block_absolute(STONE_BRICK_WALL,
					x + rx * k, ground + dy, z + rz * k,
					std::nullopt, std::nullopt);
		}
	}
	for (int k = -1; k <= 1; ++k) {
		for (int y = top - 1; y <= top; ++y) {
			editor.set_block_absolute(BLACK_CONCRETE, x + rx * k, y, z + rz * k,
					std::nullopt, std::nullopt);
		}
	}

	auto [lx, lz] = WorldEditor::panel_left_anchor(x, z, facing, 3);
	editor.place_decal_panel(lx, top, lz, facing, key, false, false);

	std::int8_t back = opposite(facing);
	auto [bx, bz] = WorldEditor::panel_left_anchor(x, z, back, 3);
	editor.place_decal_panel(bx, top, bz, back, key, false, false);
	return true;
}

bool generate_column(WorldEditor &editor, const ProcessedNode &node)
{
	if (!editor.signage_enabled())
		return false;

	int x = node.x;
	int z = node.z;
	int ground = editor.get_ground_level(x, z);
	for (int dy = 1; dy <= 3; ++dy) {
		editor.set_block_absolute(GRAY_CONCRETE, x, ground + dy, z, std::nullopt, std::nullopt);
	}
	editor.set_block_absolute(STONE_BRICK_SLAB, x, ground + 4, z, std::nullopt, std::nullopt);

	bool any = false;
	for (int f = 0; f < 4; ++f) {
		std::int8_t facing = static_cast<std::int8_t>(f + 2);
		decals::DecalKey key = decals::ColumnPosterKey{
				static_cast<std::uint8_t>((node.id + f) % 5)};
		if (editor.decal_registry->contains(key)) {
			any |= editor.place_decal_panel(x, ground + 3, z, facing, key, false, false);
		}
	}
	return any;
}

bool generate_poster_box_posters(WorldEditor &editor, const ProcessedNode &node)
{
	if (!editor.signage_enabled())
		return false;

	int x = node.x;
	int z = node.z;
	int ground = editor.get_ground_level(x, z);

	decals::DecalKey a = decals::ColumnPosterKey{static_cast<std::uint8_t>(node.id % 5)};
	decals::DecalKey b = decals::ColumnPosterKey{
			static_cast<std::uint8_t>((node.id + 2) % 5)};

	if (!editor.decal_registry->contains(a) || !editor.decal_registry->contains(b))
		return false;

	// North face: viewer's left is +x.
	editor.place_decal_panel(x + 1, ground + 3, z, 2, a, true, false);
	editor.place_decal_panel(x, ground + 3, z, 2, b, true, false);
	editor.place_decal_panel(x, ground + 3, z, 3, a, true, false);
	editor.place_decal_panel(x + 1, ground + 3, z, 3, b, true, false);
	return true;
}

bool generate_information_board(WorldEditor &editor, const ProcessedNode &node,
		const RoadMaskBitmap &road_mask)
{
	if (!editor.signage_enabled())
		return false;

	// Get information decal key based on node tags
	std::optional<decals::DecalKey> key;
	auto it_tourism = node.tags.find("tourism");
	if (it_tourism == node.tags.end() || it_tourism->second != "information")
		return false;
	
	auto it_info = node.tags.find("information");
	if (it_info != node.tags.end()) {
		if (it_info->second == "map" || it_info->second == "board" ||
			it_info->second == "terminal") {
		key = decals::DecalKey(decals::LocalMapKey{node.x, node.z});
	} else {
		key = decals::DecalKey(decals::PictogramKey{"information"});
	}
	}
	if (!key.has_value())
		return false;
	if (!editor.decal_registry->contains(key.value()))
		return false;

	int x = node.x;
	int z = node.z;
	int ground = editor.get_ground_level(x, z);

	// Face nearest road
	std::int8_t facing = 3;
	auto road_pos = get_nearest_road_block(x, z, 10, road_mask);
	if (road_pos) {
		facing = facing_for_dir(road_pos->first - x, road_pos->second - z);
	}

	auto [rx, rz] = right_dir(facing);

	bool is_local_map = std::holds_alternative<decals::LocalMapKey>(key.value());
	if (is_local_map) {
		// Board back: 2 wide x 2 tall dark oak panel on two short legs
		auto [lx, lz] = WorldEditor::panel_left_anchor(x, z, facing, 2);
		for (int c = 0; c < 2; ++c) {
			int bx = lx + rx * c;
			int bz = lz + rz * c;
			editor.set_block_absolute(OAK_FENCE, bx, ground + 1, bz, std::nullopt, std::nullopt);
			for (int y = ground + 2; y <= ground + 3; ++y) {
				editor.set_block_absolute(DARK_OAK_PLANKS, bx, y, bz, std::nullopt, std::nullopt);
			}
		}
		bool ok = editor.place_decal_panel(lx, ground + 3, lz, facing, key.value(), false, false);
		return ok;
	} else {
		// "i" post
		int head = ground + 3;
		editor.set_block_absolute(STONE_BRICK_WALL, x, ground + 1, z, std::nullopt, std::nullopt);
		editor.set_block_absolute(STONE_BRICK_WALL, x, ground + 2, z, std::nullopt, std::nullopt);
		editor.set_block_absolute(LIGHT_GRAY_CONCRETE, x, head, z, std::nullopt, std::nullopt);

		bool any = false;
		for (std::int8_t f : {2, 3, 4, 5}) {
			any |= editor.place_decal_panel(x, head, z, f, key.value(), false, false);
		}
		return any;
	}
}

} // namespace arnis::signage
