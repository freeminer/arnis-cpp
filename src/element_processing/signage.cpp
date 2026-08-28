#include "signage.h"
#include "../block_definitions.h"
#include "../decals/font.h"
#include "../decals/pictograms.h"
#include "../bresenham.h"
#include <charconv>
#include <cctype>
#include <limits>
#include <map>
#include <set>
#include <sstream>
namespace arnis::highways
{
int highway_block_range(const std::string &,
		const std::unordered_map<std::string, std::string> &, double);
}
namespace arnis::signage
{
namespace arnis_highways = ::arnis::highways;
bool place_post(world_editor::WorldEditor &editor, int x, int z,
		const decals::DecalKey &key, bool all_faces = false, bool glow = false,
		std::int8_t selected_facing = 3);
namespace
{
std::string clean_street_name(const std::string &raw)
{
	std::string out;
	bool pending_space = false;
	for (unsigned char ch : raw) {
		if (std::isspace(ch)) {
			pending_space = !out.empty();
			continue;
		}
		if (pending_space && out.size() < 60)
			out.push_back(' ');
		pending_space = false;
		if (out.size() < 60)
			out.push_back(static_cast<char>(ch));
	}
	return out;
}

std::string clean_name(const std::string &raw)
{
	std::istringstream words(raw);
	std::string word, out;
	while (words >> word) {
		if (!out.empty())
			out.push_back(' ');
		if (out.size() + word.size() > 60) {
			out.append(word, 0, 60 - out.size());
			break;
		}
		out += word;
	}
	return out;
}

std::uint8_t fascia_cols(const std::string &name)
{
	return name.size() <= 26 ? 2 : name.size() <= 44 ? 3 : 4;
}

bool named_street(const std::string &highway)
{
	static const std::set<std::string> kinds{"motorway", "trunk", "primary",
			"secondary", "tertiary", "motorway_link", "trunk_link", "primary_link",
			"secondary_link", "tertiary_link", "unclassified", "residential",
			"living_street", "pedestrian", "service", "road"};
	return kinds.contains(highway);
}

std::optional<std::pair<double, double>> way_direction_at(
		const std::vector<ProcessedNode> &nodes, std::size_t i)
{
	if (nodes.empty() || i >= nodes.size())
		return std::nullopt;
	const auto &prev = nodes[i ? i - 1 : i];
	const auto &next = nodes[i + 1 < nodes.size() ? i + 1 : i];
	double dx = next.x - prev.x, dz = next.z - prev.z;
	const double length = std::hypot(dx, dz);
	if (length <= 0.0)
		return std::nullopt;
	return std::pair{dx / length, dz / length};
}

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

struct NativeSign
{
	Block block;
	std::string text;
	bool facedir{false};
};

std::int8_t streets_facedir(std::int8_t facing)
{
	switch (facing) {
	case 2: return 0;
	case 3: return 2;
	case 4: return 1;
	case 5: return 3;
	default: return 0;
	}
}

std::optional<NativeSign> native_street_sign(
		const decals::DecalKey &key, decals::SignRegion region)
{
	using namespace block_definitions;
	Block block = AIR;
	std::string text;
	const bool eu = region == decals::SignRegion::Europe ||
			region == decals::SignRegion::Germanic ||
			region == decals::SignRegion::UkIreland;
	const bool us = region == decals::SignRegion::NorthAmerica;
	if (STREETS_AVAILABLE && (eu || us) &&
			std::holds_alternative<decals::TrafficKey>(key)) {
		const auto sign = std::get<decals::TrafficKey>(key).sign;
		switch (sign) {
		case decals::TrafficSign::Stop:
			block = eu ? STREETS_EU_SIGN_STOP : STREETS_US_SIGN_STOP;
			break;
		case decals::TrafficSign::GiveWay:
			block = eu ? STREETS_EU_SIGN_YIELD : STREETS_US_SIGN_YIELD;
			break;
		case decals::TrafficSign::NoEntry:
			block = eu ? STREETS_EU_SIGN_NO_ENTRY : STREETS_US_SIGN_NO_ENTRY;
			break;
		case decals::TrafficSign::Crossing:
			block = eu ? STREETS_EU_SIGN_CROSSING : STREETS_US_SIGN_CROSSING;
			break;
		case decals::TrafficSign::LevelCrossing:
			block = eu ? STREETS_EU_SIGN_CROSSBUCK : STREETS_US_SIGN_CROSSBUCK;
			break;
		default:
			break;
		}
		if (block.id() != CONTENT_AIR)
			return NativeSign{block, {}, true};
	}
	if (STREETS_AVAILABLE && eu &&
			std::holds_alternative<decals::SpeedLimitKey>(key)) {
		const auto &speed = std::get<decals::SpeedLimitKey>(key);
		if (!speed.mph) {
			constexpr std::array<std::uint16_t, 6> values{{10, 30, 50, 70, 100, 120}};
			const auto found = std::find(values.begin(), values.end(), speed.value);
			if (found != values.end()) {
				block = STREETS_EU_SPEED_SIGNS[static_cast<std::size_t>(
						std::distance(values.begin(), found))];
				if (block.id() != CONTENT_AIR)
					return NativeSign{block, {}, true};
			}
		}
	}

	if (!STREET_SIGNS_AVAILABLE)
		return std::nullopt;
	block = AIR;
	if (std::holds_alternative<decals::TrafficKey>(key)) {
		switch (std::get<decals::TrafficKey>(key).sign) {
		case decals::TrafficSign::Stop:
			block = STREET_SIGN_STOP;
			break;
		case decals::TrafficSign::GiveWay:
			block = STREET_SIGN_YIELD;
			break;
		case decals::TrafficSign::NoEntry:
			block = STREET_SIGN_DO_NOT_ENTER;
			break;
		case decals::TrafficSign::Crossing:
			block = STREET_SIGN_PEDESTRIAN_CROSSING;
			break;
		case decals::TrafficSign::LevelCrossing:
			block = STREET_SIGN_RR_CROSSBUCK;
			break;
		default:
			break;
		}
	} else if (std::holds_alternative<decals::SpeedLimitKey>(key)) {
		const auto &speed = std::get<decals::SpeedLimitKey>(key);
		block = STREET_SIGN_SPEED_LIMIT;
		text = std::to_string(speed.value);
	} else if (std::holds_alternative<decals::RouteShieldKey>(key)) {
		const auto &route = std::get<decals::RouteShieldKey>(key);
		if (route.style == decals::ShieldStyle::Interstate)
			block = STREET_SIGN_US_INTERSTATE;
		else if (route.style == decals::ShieldStyle::White)
			block = STREET_SIGN_US_ROUTE;
		text = route.text;
	}
	if (block.id() == CONTENT_AIR)
		return std::nullopt;
	return NativeSign{block, std::move(text), false};
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
		int half_width, const decals::DecalKey &key, bool reverse,
		const BuildingFootprintBitmap &footprints)
{
	(void)kind;
	if (cells.size() < 3 || !editor.signage_context)
		return;
	const auto &ctx = *editor.signage_context;
	idx = std::clamp<std::size_t>(idx, 1, cells.size() - 2);
	const int x = cells[idx].first;
	const int z = cells[idx].second;
	if (!editor.owns(x, z) || !ctx.has(key))
		return;
	const int bx = cells[idx - 1].first, bz = cells[idx - 1].second;
	const int fx = cells[idx + 1].first, fz = cells[idx + 1].second;
	double dx = fx - bx, dz = fz - bz;
	const double len = std::hypot(dx, dz);
	if (len == 0.0)
		return;
	dx /= len;
	dz /= len;
	if (reverse) {
		dx = -dx;
		dz = -dz;
	}
	const double px = decals::drives_on_left(ctx.region) ? dz : -dz;
	const double pz = decals::drives_on_left(ctx.region) ? -dx : dx;
	for (int k = 0; k < 5; ++k) {
		const double offset = half_width + 2 + k;
		const int sx = x + static_cast<int>(std::round(px * offset));
		const int sz = z + static_cast<int>(std::round(pz * offset));
		if (!editor.owns(sx, sz) || ctx.carriageway.contains(sx, sz))
			continue;
		if (footprints.contains(sx, sz) || editor.is_lc_water(sx, sz))
			continue;
		const std::int8_t facing = facing_for_dir(-dx, -dz);
		if (place_post(editor, sx, sz, key, false, false, facing))
			return;
	}
}

} // anonymous namespace

IntersectionIndex build_intersection_index(
		const std::vector<ProcessedElement> &elements, double scale)
{
	struct Street
	{
		const ProcessedWay *way;
		std::string name;
		int half_width;
	};
	struct Hit
	{
		std::uint64_t way_id;
		std::string name;
		std::pair<double, double> direction;
		int half_width;
	};
	std::vector<Street> streets;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto highway = way.tags.get("highway");
		if (!named_street(highway) || way.tags.get("area") == "yes" ||
				(!way.tags.get("tunnel").empty() && way.tags.get("tunnel") != "no") ||
				way.tags.get("indoor") == "yes")
			continue;
		auto name = clean_street_name(way.tags.get("name"));
		if (name.empty() || !decals::font::supports(name))
			continue;
		streets.push_back({&way, std::move(name),
				arnis_highways::highway_block_range(highway, way.tags, scale)});
	}

	std::unordered_map<std::uint64_t, unsigned> counts;
	for (const auto &street : streets)
		for (const auto &node : street.way->nodes)
			++counts[node.id];
	std::unordered_map<std::uint64_t, std::vector<Hit>> by_node;
	for (const auto &street : streets) {
		for (std::size_t i = 0; i < street.way->nodes.size(); ++i) {
			const auto &node = street.way->nodes[i];
			if (counts[node.id] < 2)
				continue;
			if (auto direction = way_direction_at(street.way->nodes, i))
				by_node[node.id].push_back({street.way->id, street.name, *direction,
						street.half_width});
		}
	}

	IntersectionIndex result;
	for (auto &[node_id, hits] : by_node) {
		std::map<std::string, std::vector<const Hit *>> names;
		std::uint64_t owner = std::numeric_limits<std::uint64_t>::max();
		for (const auto &hit : hits) {
			names[hit.name].push_back(&hit);
			owner = std::min(owner, hit.way_id);
		}
		if (names.size() < 2)
			continue;
		IntersectionPost post;
		post.owner_way = owner;
		for (const auto &[name, same_name] : names) {
			if (post.blades.size() == 3)
				break;
			std::pair<double, double> sum{0.0, 0.0};
			int half_width = 0;
			for (const Hit *hit : same_name) {
				auto direction = hit->direction;
				if (direction.first * sum.first + direction.second * sum.second < 0.0) {
					direction.first = -direction.first;
					direction.second = -direction.second;
				}
				sum.first += direction.first;
				sum.second += direction.second;
				half_width = std::max(half_width, hit->half_width);
			}
			const double length = std::hypot(sum.first, sum.second);
			if (length > 0.0) {
				sum.first /= length;
				sum.second /= length;
			} else {
				sum = {1.0, 0.0};
			}
			post.blades.push_back({name, sum, half_width});
		}
		result.emplace(node_id, std::move(post));
	}
	return result;
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
	if (tags.get("oneway") == "yes" || tags.get("oneway") == "-1" ||
			tags.get("oneway") == "true")
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
		decals::SignRegion region, double scale)
{
	if (level == SignageLevel::None)
		return {};
	auto intersections = std::make_shared<IntersectionIndex>(
			build_intersection_index(elements, scale));
	std::set<decals::DecalKey> keys;
	for (const auto &element : elements) {
		if (element.is_way()) {
			const auto &way = element.as_way();
			if (level == SignageLevel::Full && way.tags.contains("building") &&
					!way.tags.contains("building:part")) {
				if (decals::pictograms::business_kind(way.tags)) {
					const auto name = clean_name(way.tags.get("name"));
					if (!name.empty() && decals::font::supports(name)) {
						keys.insert(decals::DecalKey::text(
								{decals::TextStyleKind::Fascia}, name, fascia_cols(name)));
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
			const auto &tags = node.tags;
			if (level == SignageLevel::Full &&
					(tags.contains("shop") || tags.contains("amenity") ||
						tags.contains("office") || tags.contains("tourism") ||
						tags.contains("leisure") || tags.contains("healthcare") ||
						tags.contains("craft")) &&
					decals::pictograms::business_kind(tags)) {
				const auto name = clean_name(tags.get("name"));
				if (!name.empty() && decals::font::supports(name))
					keys.insert(decals::DecalKey::text(
							{decals::TextStyleKind::Fascia}, name, fascia_cols(name)));
			}
			const auto number = tags.get("addr:housenumber");
			if (level == SignageLevel::Full && !number.empty() && number.size() <= 8 &&
					decals::font::supports(number))
				keys.insert(decals::DecalKey::text(
						{decals::TextStyleKind::HouseNumber}, number, 1));
			if (auto key = highway_node_sign(node.tags, level))
				keys.insert(*key);
			if (auto key = information_key(node))
				keys.insert(*key);
			if (auto key = furniture_pictogram(node.tags))
				keys.insert(*key);
			for (auto &key : advertising_keys(node.tags, node.id))
				keys.insert(std::move(key));
			if (tags.get("highway") == "bus_stop") {
				keys.insert(decals::PictogramKey{"bus_stop"});
				const auto name = clean_name(tags.get("name"));
				if (!name.empty() && decals::font::supports(name))
					keys.insert(decals::DecalKey::text(
							{decals::TextStyleKind::StopName}, name, 1));
			}
			const auto railway = tags.get("railway");
			const bool subway = (railway == "station" || railway == "halt") &&
					(tags.get("station") == "subway" || tags.get("subway") == "yes");
			if ((railway == "station" || railway == "halt") && !subway) {
				keys.insert(decals::PictogramKey{"train"});
				const auto name = clean_name(tags.get("name"));
				if (!name.empty() && decals::font::supports(name))
					keys.insert(decals::DecalKey::text(
							{decals::TextStyleKind::StationBoard}, name, 3));
			} else if (railway == "tram_stop") {
				keys.insert(decals::PictogramKey{"tram"});
				const auto name = clean_name(tags.get("name"));
				if (!name.empty() && decals::font::supports(name))
					keys.insert(decals::DecalKey::text(
							{decals::TextStyleKind::StopName}, name,
							name.size() <= 10 ? 1 : 2));
			}
			else if (railway == "subway_entrance")
				keys.insert(decals::PictogramKey{decals::metro_logo(region)});
			else if (subway)
				keys.insert(decals::PictogramKey{decals::metro_logo(region)});
			if (tags.get("historic") == "memorial" && tags.get("memorial") == "plaque") {
				const auto text = clean_name(!tags.get("inscription").empty()
						? tags.get("inscription") : tags.get("name"));
				if (!text.empty() && decals::font::supports(text))
					keys.insert(decals::DecalKey::text(
							{decals::TextStyleKind::Plaque}, text, 1));
			}
		}
	}
	const decals::TextStyle blade_style{
			decals::TextStyleKind::StreetName, decals::blade_style(region)};
	for (const auto &[node_id, post] : *intersections) {
		(void)node_id;
		for (const auto &blade : post.blades)
			keys.insert(decals::DecalKey::text(blade_style, blade.name, 1));
	}
	return keys.empty() ? std::shared_ptr<const decals::DecalRegistry>{}
						: std::make_shared<const decals::DecalRegistry>(
								  decals::DecalRegistry::from_keys(keys));
}

std::shared_ptr<const SignageContext> build_context(
		const std::vector<ProcessedElement> &elements, SignageLevel level,
		decals::SignRegion region, double scale, const RoadMaskBitmap &carriageway)
{
	if (level == SignageLevel::None)
		return {};
	auto registry = build_registry(elements, level, region, scale);
	if (!registry)
		return {};
	auto context = std::make_shared<SignageContext>();
	context->registry = std::move(registry);
	context->level = level;
	context->region = region;
	context->intersections = build_intersection_index(elements, scale);
	context->scale = scale;
	context->carriageway = carriageway;
	return context;
}

bool place_post(world_editor::WorldEditor &editor, int x, int z,
		const decals::DecalKey &key, bool all_faces, bool glow,
		std::int8_t selected_facing)
{
	if (!editor.signage_enabled() || !editor.owns(x, z) ||
			!editor.decal_registry->contains(key))
		return false;
	const int ground = editor.get_ground_level(x, z);
	for (int dy = 1; dy <= 3; ++dy)
		if (!editor.cell_open_at(x, ground + dy, z) ||
				editor.cell_has_frame(x, ground + dy, z))
			return false;
	const auto region = editor.signage_context
			? editor.signage_context->region : decals::SignRegion::Europe;
	if (!all_faces && block_definitions::STREETS_RRXING_AVAILABLE &&
			std::holds_alternative<decals::TrafficKey>(key) &&
			std::get<decals::TrafficKey>(key).sign ==
					decals::TrafficSign::LevelCrossing) {
		const auto param2 = streets_facedir(selected_facing);
		for (const auto [dy, source] :
				std::array<std::pair<int, Block>, 3>{{
						{1, block_definitions::STREETS_RRXING_BOTTOM},
						{2, block_definitions::STREETS_RRXING_MIDDLE},
						{3, block_definitions::STREETS_RRXING_TOP}}}) {
			auto block = source;
			block.setParam2(static_cast<std::uint8_t>(param2));
			editor.set_block_absolute(block, x, ground + dy, z);
		}
		return true;
	}
	editor.set_block_absolute(block_definitions::STREETS_POLE, x, ground + 1, z,
			std::nullopt, std::nullopt);
	editor.set_block_absolute(block_definitions::STREETS_POLE, x, ground + 2, z,
			std::nullopt, std::nullopt);
	const int head = ground + 3;
	// Prefer street_signs' purpose-built mesh and pole mount. Text-bearing signs
	// receive the same generated metadata as default/signs_lib wall signs.
	if (!all_faces)
		if (const auto native = native_street_sign(key, region);
				native && editor.place_sign_node(native->block, x, head, z,
						native->facedir ? streets_facedir(selected_facing)
										: selected_facing,
						native->text))
			return true;
	editor.set_block_absolute(block_definitions::LIGHT_GRAY_CONCRETE, x, head, z,
			std::nullopt, std::nullopt);
	bool placed = false;
	for (const std::int8_t facing : std::array<std::int8_t, 4>{2, 3, 4, 5}) {
		placed |= editor.place_decal_panel(
				x, head, z, all_faces ? facing : selected_facing, key, glow, false);
		if (!all_faces)
			break;
	}
	return placed;
}

void place_node_signage(world_editor::WorldEditor &editor, const ProcessedNode &node,
		SignageLevel level, decals::SignRegion region)
{
	(void)region;
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

std::optional<NameSign> make_name_sign(const tags_t &tags, decals::TextStyle style,
		std::uint8_t cols)
{
	auto raw = get_name(tags);
	if (!raw)
		return std::nullopt;
	const auto name = clean_name(*raw);
	if (name.empty())
		return std::nullopt;
	NameSign result;
	result.text = name;
	if (decals::font::supports(name))
		result.key = decals::DecalKey::text(style, name, cols);
	return result;
}

bool place_name_sign(WorldEditor &editor, int x, int y, int z,
		std::int8_t facing, const NameSign &name, bool require_hosts = true)
{
	if (name.key) {
		const auto [cols, rows] = name.key->dims();
		(void)rows;
		const auto [left_x, left_z] = WorldEditor::panel_left_anchor(
				x, z, facing, static_cast<int>(cols));
		return editor.place_decal_panel(
				left_x, y, left_z, facing, *name.key, false, require_hosts);
	}
	// place_text_sign writes the wall-sign node itself, while decal coordinates
	// name the backing block.  Offset to the exterior cell like Rust's
	// place_wall_sign so the sign does not collide with its host wall.
	switch (facing) {
	case 2: --z; break;
	case 3: ++z; break;
	case 4: --x; break;
	case 5: ++x; break;
	default: break;
	}
	return editor.place_text_sign(x, y, z, facing, name.text, true);
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

std::optional<NameSign> poi_name(const tags_t &tags, SignageLevel level)
{
	if (level != SignageLevel::Full)
		return std::nullopt;
	const auto amenity = tags.get("amenity");
	static const std::set<std::string> furniture{"recycling", "waste_basket",
			"waste_disposal", "vending_machine", "atm", "bench", "drinking_water",
			"fountain", "bicycle_parking", "shelter", "post_box", "parking_space"};
	if (furniture.contains(amenity) || tags.get("tourism") == "information" ||
			!decals::pictograms::business_kind(tags))
		return std::nullopt;
	auto raw = get_name(tags);
	if (!raw)
		return std::nullopt;
	const auto name = clean_name(*raw);
	return make_name_sign(tags, {decals::TextStyleKind::Fascia}, fascia_cols(name));
}

std::optional<decals::DecalKey> house_number(const tags_t &tags, SignageLevel level)
{
	if (level != SignageLevel::Full)
		return std::nullopt;
	auto num = get_house_number(tags);
	if (!num || num->empty() || num->size() > 8 || !decals::font::supports(*num))
		return std::nullopt;
	return decals::DecalKey::text(
			{decals::TextStyleKind::HouseNumber}, *num, 1);
}

void generate_building_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const std::optional<building_facade::FacadeAnchor> &anchor)
{
	if (!editor.signage_enabled() || !editor.signage_context || !anchor ||
			way.tags.contains("building:part") || !editor.owns(anchor->x, anchor->z))
		return;
	const auto level = editor.signage_context->level;
	const auto facing = WorldEditor::facing_for_normal(
			anchor->normal.first, anchor->normal.second);
	if (const auto name = poi_name(way.tags, level)) {
		if (!name->key || editor.signage_context->has(*name->key))
			for (int dy = 0; dy < 2; ++dy)
				if (place_name_sign(editor, anchor->x, anchor->fascia_y + dy,
						anchor->z, facing, *name))
					break;
	}
	if (const auto number = house_number(way.tags, level);
			number && editor.signage_context->has(*number)) {
		const auto [rx, rz] = right_dir(facing);
		const auto first = anchor->door
				? std::pair{anchor->door->first + rx, anchor->door->second + rz}
				: std::pair{anchor->x - rx * 2, anchor->z - rz * 2};
		for (const auto [x, z] : {first,
				 std::pair{first.first - rx * 2, first.second - rz * 2}})
			if (editor.place_decal_panel(
						x, anchor->number_y, z, facing, *number, false, true))
				break;
	}
}

std::optional<std::tuple<int, int, std::int8_t>> nearest_wall_from_inside(
		const WorldEditor &editor, int x, int z, int sign_y,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &road_mask)
{
	std::optional<std::pair<int, std::tuple<int, int, std::int8_t>>> best;
	for (const auto [dx, dz] : std::array<std::pair<int, int>, 4>{
				 std::pair{0, -1}, {0, 1}, {-1, 0}, {1, 0}}) {
		bool left_footprint = false;
		for (int k = 1; k <= 24; ++k) {
			const int cx = x + dx * k, cz = z + dz * k;
			if (!left_footprint) {
				if (footprints.contains(cx, cz))
					continue;
				left_footprint = true;
			}
			if (!editor.cell_open_at(cx, sign_y, cz))
				continue;
			const int hx = cx - dx, hz = cz - dz;
			if (editor.cell_open_at(hx, sign_y, hz))
				break;
			int score = k;
			bool road_near = false;
			for (int j = 1; j <= 8; ++j)
				road_near |= road_mask.contains(cx + dx * j, cz + dz * j);
			if (!road_near)
				score += 6;
			const auto facing = WorldEditor::facing_for_normal(dx, dz);
			if (!best || score < best->first)
				best = {score, std::tuple{hx, hz, facing}};
			break;
		}
	}
	return best ? std::optional{best->second} : std::nullopt;
}

void generate_node_facade_signage(world_editor::WorldEditor &editor,
		const ProcessedNode &node,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &road_mask)
{
	if (!editor.signage_enabled() || !editor.signage_context ||
			!editor.owns(node.x, node.z))
		return;
	const auto &ctx = *editor.signage_context;
	const auto &tags = node.tags;
	const bool poi = tags.contains("shop") || tags.contains("amenity") ||
			tags.contains("office") || tags.contains("tourism") ||
			tags.contains("leisure") || tags.contains("healthcare") ||
			tags.contains("craft");
	if (footprints.contains(node.x, node.z) && (poi || tags.contains("addr:housenumber"))) {
		const int row = editor.get_ground_level(node.x, node.z) + 3;
		if (const auto wall = nearest_wall_from_inside(
					editor, node.x, node.z, row, footprints, road_mask)) {
			const auto [hx, hz, facing] = *wall;
			const int wall_row = editor.get_ground_level(hx, hz) + 3;
			if (poi)
				if (const auto name = poi_name(tags, ctx.level);
						name && (!name->key || ctx.has(*name->key)))
					for (int dy = 0; dy < 2; ++dy)
						if (place_name_sign(
								editor, hx, wall_row + dy, hz, facing, *name))
							break;
			if (const auto number = house_number(tags, ctx.level);
					number && ctx.has(*number))
				editor.place_decal_panel(
						hx, editor.get_ground_level(hx, hz) + 2, hz,
						facing, *number, false, true);
		}
	}
}

void generate_node_signage(world_editor::WorldEditor &editor, const ProcessedNode &node,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &road_mask)
{
	(void)footprints;
	if (!editor.signage_enabled() || !editor.signage_context ||
			!editor.owns(node.x, node.z))
		return;
	const auto &ctx = *editor.signage_context;
	const auto &tags = node.tags;

	if (const auto key = highway_node_sign(tags, ctx.level); key && ctx.has(*key)) {
		int sx = node.x, sz = node.z;
		if (ctx.carriageway.contains(sx, sz)) {
			bool found = false;
			for (int radius = 1; radius <= 10 && !found; ++radius)
				for (int dx = -radius; dx <= radius && !found; ++dx)
					for (int dz = -radius; dz <= radius; ++dz) {
						if (std::max(std::abs(dx), std::abs(dz)) != radius)
							continue;
						const int tx = node.x + dx, tz = node.z + dz;
						if (!ctx.carriageway.contains(tx, tz) && editor.owns(tx, tz) &&
								!editor.is_lc_water(tx, tz)) {
							sx = tx;
							sz = tz;
							found = true;
							break;
						}
					}
			if (!found)
				return;
		}
		place_post(editor, sx, sz, *key, false, false,
				facing_for_dir(node.x - sx, node.z - sz));
	}

	const auto railway = tags.get("railway");
	if (railway == "level_crossing" || railway.empty())
		return;
	const bool subway = (railway == "station" || railway == "halt") &&
			(tags.get("station") == "subway" || tags.get("subway") == "yes");
	if (railway == "station" || railway == "halt") {
		const decals::DecalKey icon = decals::PictogramKey{
				subway ? decals::metro_logo(ctx.region) : "train"};
		if (!ctx.has(icon))
			return;
		if (subway) {
			place_post(editor, node.x, node.z, icon, true, true);
			return;
		}
		const int ground = editor.get_ground_level(node.x, node.z), beam = ground + 3;
		bool clear = true;
		for (const int dx : {-1, 1})
			for (int dy = 1; dy <= 3; ++dy)
				clear &= editor.cell_open_at(node.x + dx, ground + dy, node.z);
		if (!clear)
			return;
		for (const int dx : {-1, 1})
			for (int dy = 1; dy <= 2; ++dy)
				editor.set_block_absolute(block_definitions::STONE_BRICK_WALL,
						node.x + dx, ground + dy, node.z);
		for (int dx = -1; dx <= 1; ++dx)
			editor.set_block_absolute(block_definitions::LIGHT_GRAY_CONCRETE,
					node.x + dx, beam, node.z);
		const auto name = make_name_sign(tags,
				{decals::TextStyleKind::StationBoard}, 3);
		bool named = false;
		if (name && (!name->key || ctx.has(*name->key)))
			for (const std::int8_t facing : {std::int8_t(2), std::int8_t(3)})
				named |= place_name_sign(
						editor, node.x, beam, node.z, facing, *name, true);
		if (!named)
			for (const std::int8_t facing : {std::int8_t(2), std::int8_t(3),
					 std::int8_t(4), std::int8_t(5)})
				editor.place_decal(node.x, beam, node.z, facing, icon);
		return;
	}
	if (railway == "tram_stop" || railway == "subway_entrance") {
		const decals::DecalKey icon = decals::PictogramKey{
				railway == "tram_stop" ? "tram" : decals::metro_logo(ctx.region)};
		if (!ctx.has(icon) || ctx.carriageway.contains(node.x, node.z))
			return;
		place_post(editor, node.x, node.z, icon, true, railway == "subway_entrance");
		if (railway == "tram_stop") {
			const auto name = make_name_sign(tags, {decals::TextStyleKind::StopName},
					clean_name(tags.get("name")).size() <= 10 ? 1 : 2);
			if (name && (!name->key || ctx.has(*name->key)))
				place_name_sign(editor, node.x,
						editor.get_ground_level(node.x, node.z) + 4,
						node.z, 3, *name, false);
		}
	}
}

void generate_power_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const RoadMaskBitmap &road_mask)
{
	if (!editor.signage_enabled() || !editor.signage_context)
		return;

	auto sign = power_sign(way.tags);
	if (!sign)
		return;

	if (way.nodes.empty())
		return;

	const auto &node = way.nodes.front();
	int x = node.x, z = node.z;

	if (editor.signage_context->carriageway.contains(x, z))
		return;
	if (!editor.owns(x, z))
		return;

	std::int8_t facing = 3;
	if (const auto road = get_nearest_road_block(x, z, 12, road_mask))
		facing = facing_for_dir(road->first - x, road->second - z);
	if (place_post(editor, x, z, *sign, false, false, facing))
		editor.place_decal_panel(x, editor.get_ground_level(x, z) + 3, z,
				opposite(facing), *sign, false, false);
}

static void place_street_name_post(world_editor::WorldEditor &editor,
		const ProcessedNode &node, const std::pair<double, double> &own_direction,
		const IntersectionPost &post, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &road_mask)
{
	if (!editor.decal_registry || !editor.signage_context)
		return;
	int widest = 1, width_x = 0, width_z = 0;
	for (const auto &blade : post.blades) {
		widest = std::max(widest, blade.half_width);
		if (std::abs(blade.direction.second) > std::abs(blade.direction.first))
			width_x = std::max(width_x, blade.half_width);
		else
			width_z = std::max(width_z, blade.half_width);
	}
	if (!width_x) width_x = widest;
	if (!width_z) width_z = widest;
	auto cross = std::pair{-own_direction.second, own_direction.first};
	for (const auto &blade : post.blades) {
		const double dot = blade.direction.first * own_direction.first +
				blade.direction.second * own_direction.second;
		if (std::abs(dot) < 0.9) {
			cross = blade.direction;
			break;
		}
	}
	const int qx = -(own_direction.first + cross.first) >= 0.0 ? 1 : -1;
	const int qz = -(own_direction.second + cross.second) >= 0.0 ? 1 : -1;
	std::vector<std::pair<int, int>> offsets;
	for (int dx = 1; dx <= 9; ++dx)
		for (int dz = 1; dz <= 9; ++dz)
			offsets.emplace_back(dx, dz);
	std::sort(offsets.begin(), offsets.end(), [](const auto &a, const auto &b) {
		return a.first * a.first + a.second * a.second <
				b.first * b.first + b.second * b.second;
	});
	std::optional<std::pair<int, int>> position;
	for (const auto [sx, sz] : std::array<std::pair<int, int>, 4>{{
			 {qx, qz}, {qx, -qz}, {-qx, qz}, {-qx, -qz}}}) {
		for (const auto [dx, dz] : offsets) {
			const int x = node.x + sx * (width_x + dx);
			const int z = node.z + sz * (width_z + dz);
			if (!editor.owns(x, z) || road_mask.contains(x, z) ||
					footprints.contains(x, z) || editor.is_lc_water(x, z))
				continue;
			const int ground = editor.get_ground_level(x, z);
			bool clear = true;
			for (int dy = 1; dy <= static_cast<int>(post.blades.size()) + 2; ++dy)
				clear &= editor.cell_open_at(x, ground + dy, z) &&
						!editor.cell_has_frame(x, ground + dy, z);
			if (clear) {
				position = {x, z};
				break;
			}
		}
		if (position)
			break;
	}
	if (!position)
		return;
	const auto [x, z] = *position;
	const int ground = editor.get_ground_level(x, z);
	// street_signs:sign_basic is the native two-up intersection post described by
	// the mod itself. Use it for the common two-street case and retain the decal
	// stack below for three-way names or games without street_signs.
	if (block_definitions::STREET_SIGNS_AVAILABLE && post.blades.size() <= 2) {
		std::string text;
		for (const auto &blade : post.blades) {
			if (!text.empty())
				text.push_back('\n');
			text += blade.name;
		}
		const std::int8_t facedir =
				std::abs(own_direction.first) >= std::abs(own_direction.second) ? 0 : 1;
		if (editor.place_sign_node(block_definitions::STREET_SIGN_BASIC,
					x, ground + 1, z, facedir, text))
			return;
	}
	editor.set_block_absolute(block_definitions::STONE_BRICK_WALL, x, ground + 1, z,
			std::nullopt, std::nullopt);
	editor.set_block_absolute(block_definitions::STONE_BRICK_WALL, x, ground + 2, z,
			std::nullopt, std::nullopt);
	const decals::TextStyle style{
			decals::TextStyleKind::StreetName,
			decals::blade_style(editor.signage_context->region)};
	int row = 0;
	for (const auto &blade : post.blades) {
		const auto key = decals::DecalKey::text(style, blade.name, 1);
		if (!editor.decal_registry->contains(key))
			continue;
		const int y = ground + 3 + row++;
		editor.set_block_absolute(block_definitions::POLISHED_ANDESITE, x, y, z,
				std::nullopt, std::nullopt);
		const bool along_x = std::abs(blade.direction.first) >=
				std::abs(blade.direction.second);
		const std::int8_t first = along_x ? 2 : 4;
		editor.place_decal_panel(x, y, z, first, key, false, false);
		editor.place_decal_panel(x, y, z, opposite(first), key, false, false);
	}
}

void generate_highway_way_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const BuildingFootprintBitmap &footprints)
{
	// Highway signage: speed limit signs, route shields, cycleway signs, no-entry and periodic signs
	if (!editor.signage_enabled() || !editor.signage_context || way.nodes.size() < 2)
		return;
	const auto &ctx = *editor.signage_context;
	const auto &road_mask = ctx.carriageway;

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
	const bool oneway = way.tags.get("oneway") == "yes" ||
			way.tags.get("oneway") == "-1";
	const bool reversed = way.tags.get("oneway") == "-1";
	const int half_width =
			arnis_highways::highway_block_range(highway, way.tags, ctx.scale);

	// A single owner way places each shared-node post, preventing duplicate
	// blades when OSM splits the same intersection into several ways.
	{
		for (std::size_t i = 0; i < way.nodes.size(); ++i) {
			const auto found = ctx.intersections.find(way.nodes[i].id);
			if (found == ctx.intersections.end() ||
					found->second.owner_way != way.id)
				continue;
			if (auto direction = way_direction_at(way.nodes, i))
				place_street_name_post(editor, way.nodes[i], *direction, found->second,
						footprints, road_mask);
		}
	}

	// Build Bresenham line along the way
	std::vector<std::pair<int, int>> cells;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &from = way.nodes[i - 1];
		const auto &to = way.nodes[i];
		const auto line = bresenham::bresenham_line(from.x, 0, from.z, to.x, 0, to.z);
		for (const auto &pt : line)
			cells.emplace_back(std::get<0>(pt), std::get<2>(pt));
	}

	const auto signs = highway_way_signs(way.tags, ctx.region);
	if (!signs.speed.has_value() && !signs.shield.has_value() &&
			!signs.no_entry.has_value() && !signs.cycleway.has_value())
		return;
	if (cells.size() < 24)
		return;

	// Place cycleway signs (if cycleway, place just these and return)
	if (signs.cycleway.has_value()) {
		place_roadside_sign(editor, "cycleway signs", cells, 6, half_width,
				signs.cycleway.value(), false, footprints);
		return;
	}

	// Place speed limit signs at start and periodically (every 8 cells for short gaps)
	if (signs.speed.has_value()) {
		{
			place_roadside_sign(editor, "speed limits", cells, 8, half_width,
					signs.speed.value(), reversed, footprints);
			if (!oneway && cells.size() >= 60) {
				place_roadside_sign(editor, "speed limits", cells,
						cells.size() - 9, half_width, signs.speed.value(), true, footprints);
			}
		}
	}

	// No-entry where wrong-way traffic would enter a one-way street
	if (signs.no_entry.has_value()) {
		if (reversed) {
			place_roadside_sign(editor, "no-entry signs", cells, 4, half_width,
					signs.no_entry.value(), false, footprints);
		} else if (oneway) {
			place_roadside_sign(editor, "no-entry signs", cells,
					cells.size() - 5, half_width, signs.no_entry.value(), true, footprints);
		}
	}

	// Reassurance shields: thinned so a road split into many OSM ways is not carpeted
	if (signs.shield.has_value() && cells.size() >= 40) {
		if (cells.size() >= 120 || way.id % 3 == 0)
			place_roadside_sign(editor, "route shields", cells,
					std::min<std::size_t>(40, cells.size() / 2), half_width,
					signs.shield.value(), false, footprints);

		std::size_t pos = 250;
		while (pos + 30 < cells.size()) {
			place_roadside_sign(editor, "route shields", cells, pos, half_width,
					signs.shield.value(), false, footprints);
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

std::optional<decals::DecalKey> plaque_key(const tags_t &tags)
{
	if (tags.get("historic") != "memorial" || tags.get("memorial") != "plaque")
		return std::nullopt;
	const auto text = clean_name(!tags.get("inscription").empty()
			? tags.get("inscription") : tags.get("name"));
	if (text.empty() || !decals::font::supports(text))
		return std::nullopt;
	return decals::DecalKey::text({decals::TextStyleKind::Plaque}, text, 1);
}

bool generate_plaque(WorldEditor &editor, const ProcessedNode &node)
{
	if (!editor.signage_enabled() || !editor.signage_context)
		return false;
	const auto key = plaque_key(node.tags);
	if (!key || !editor.signage_context->has(*key))
		return false;
	const int x = node.x, z = node.z;
	const int y = editor.get_ground_level(x, z) + 2;
	for (const auto [dx, dz] : std::array<std::pair<int, int>, 4>{
				 std::pair{0, -1}, {0, 1}, {-1, 0}, {1, 0}}) {
		if (!editor.cell_open_at(x + dx, y, z + dz) &&
				editor.cell_open_at(x, y, z)) {
			const auto facing = WorldEditor::facing_for_normal(-dx, -dz);
			return editor.place_decal(x + dx, y, z + dz, facing, *key);
		}
	}
	return false;
}

void place_bus_stop_signs(WorldEditor &editor, const tags_t &tags,
		int x, int sign_y, int z)
{
	if (!editor.signage_enabled() || !editor.signage_context)
		return;
	const auto &ctx = *editor.signage_context;
	const decals::DecalKey icon = decals::PictogramKey{"bus_stop"};
	if (ctx.has(icon)) {
		editor.place_decal(x + 1, sign_y, z, 2, icon);
		editor.place_decal(x + 1, sign_y, z, 3, icon);
	}
	const auto name = make_name_sign(tags, {decals::TextStyleKind::StopName}, 1);
	if (name && (!name->key || ctx.has(*name->key)))
		for (const std::int8_t facing : {std::int8_t(2), std::int8_t(3)})
			place_name_sign(editor, x, sign_y, z, facing, *name, true);
}

} // namespace arnis::signage
