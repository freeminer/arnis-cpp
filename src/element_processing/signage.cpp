#include "signage.h"
#include "../decals/font.h"
#include "../decals/pictograms.h"
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
}
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
}
