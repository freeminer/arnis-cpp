#include "uses.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>

namespace arnis::interior_uses
{
namespace
{
constexpr std::size_t CELLS_PER_UNIT = 36;
constexpr std::size_t HOST_SHARE_CELLS = 140;
constexpr int SAME_TENANT_DIST = 4;

std::string_view tag(const tags_t &tags, std::string_view key)
{
	const auto it = tags.find(std::string(key));
	return it == tags.end() ? std::string_view{} : std::string_view(it->second);
}
bool has_tag(const tags_t &tags, std::string_view key)
{
	return tags.find(std::string(key)) != tags.end();
}

Use simple(UseKind kind)
{
	return Use{kind};
}
Use shop(Goods goods)
{
	return Use{UseKind::Shop, goods};
}
Use food(Eatery eatery)
{
	Use use{UseKind::Food};
	use.eatery = eatery;
	return use;
}
Use worship(Faith faith)
{
	Use use{UseKind::Worship};
	use.faith = faith;
	return use;
}

Goods goods_for_shop(std::string_view value)
{
	if (value == "bakery" || value == "pastry" || value == "confectionery" ||
			value == "chocolate" || value == "deli" || value == "cheese" ||
			value == "coffee" || value == "tea")
		return Goods::Bakery;
	if (value == "butcher" || value == "seafood")
		return Goods::Butcher;
	if (value == "greengrocer" || value == "farm" || value == "health_food" ||
			value == "organic" || value == "frozen_food" || value == "pet")
		return Goods::Grocery;
	if (value == "books" || value == "stationery" || value == "music" ||
			value == "video" || value == "art" || value == "anime" ||
			value == "musical_instrument")
		return Goods::Books;
	if (value == "clothes" || value == "shoes" || value == "boutique" ||
			value == "fashion" || value == "fashion_accessories" || value == "bag" ||
			value == "leather" || value == "fabric" || value == "tailor" ||
			value == "sewing" || value == "second_hand" || value == "charity" ||
			value == "wool" || value == "curtain" || value == "sports" ||
			value == "outdoor" || value == "baby_goods")
		return Goods::Clothes;
	if (value == "electronics" || value == "computer" || value == "mobile_phone" ||
			value == "hifi" || value == "telecommunication" || value == "appliance" ||
			value == "camera" || value == "video_games" || value == "radiotechnics" ||
			value == "electrical" || value == "games")
		return Goods::Electronics;
	if (value == "hardware" || value == "doityourself" || value == "trade" ||
			value == "paint" || value == "tool_hire" || value == "building_materials" ||
			value == "agrarian" || value == "weapons" || value == "hunting" ||
			value == "fishing" || value == "locksmith")
		return Goods::Hardware;
	if (value == "chemist" || value == "cosmetics" || value == "perfumery" ||
			value == "medical_supply" || value == "hearing_aids" || value == "optician" ||
			value == "herbalist" || value == "drugstore" ||
			value == "nutrition_supplements")
		return Goods::Pharmacy;
	if (value == "florist" || value == "garden_centre" || value == "plant")
		return Goods::Florist;
	if (value == "jewelry" || value == "watches" || value == "gold_buyer" ||
			value == "pawnbroker")
		return Goods::Jewelry;
	if (value == "toys" || value == "gift" || value == "party" || value == "craft" ||
			value == "hobby" || value == "model" || value == "souvenir")
		return Goods::Toys;
	if (value == "furniture" || value == "interior_decoration" || value == "bed" ||
			value == "kitchen" || value == "lighting" || value == "houseware" ||
			value == "carpet" || value == "antiques" || value == "frame" ||
			value == "flooring" || value == "bathroom_furnishing" || value == "doors")
		return Goods::Furniture;
	if (value == "alcohol" || value == "beverages" || value == "wine")
		return Goods::Drinks;
	if (value == "hairdresser" || value == "beauty" || value == "massage" ||
			value == "tattoo" || value == "nails" || value == "hairdresser_supply")
		return Goods::Salon;
	return Goods::General;
}

Faith faith(const tags_t &tags)
{
	const auto religion = tag(tags, "religion");
	if (has_tag(tags, "religion")) {
		if (religion == "muslim")
			return Faith::Muslim;
		if (religion == "jewish")
			return Faith::Jewish;
		if (religion == "christian")
			return Faith::Christian;
		return Faith::Other;
	}
	const auto building = tag(tags, "building");
	if (building == "mosque")
		return Faith::Muslim;
	if (building == "synagogue")
		return Faith::Jewish;
	if (building == "temple" || building == "shrine")
		return Faith::Other;
	return Faith::Christian;
}

bool includes(std::initializer_list<std::string_view> values, std::string_view value)
{
	return std::find(values.begin(), values.end(), value) != values.end();
}
} // namespace

bool Use::is_street_tenant() const
{
	return kind == UseKind::Shop || kind == UseKind::Supermarket ||
		   kind == UseKind::Food || kind == UseKind::Bank || kind == UseKind::Workshop ||
		   kind == UseKind::Clinic || kind == UseKind::Gym;
}
bool Use::is_hall() const
{
	return kind == UseKind::Worship || kind == UseKind::SportsHall ||
		   kind == UseKind::Auditorium || kind == UseKind::Warehouse ||
		   kind == UseKind::Factory || kind == UseKind::Barn || kind == UseKind::Station;
}
bool Use::always_open() const
{
	return kind == UseKind::Worship || kind == UseKind::SportsHall ||
		   kind == UseKind::Auditorium;
}
bool Use::hosts_tenants() const
{
	return kind == UseKind::Home || kind == UseKind::Office || kind == UseKind::Hotel ||
		   kind == UseKind::Shop || kind == UseKind::Supermarket;
}
int Use::rank() const
{
	switch (kind) {
	case UseKind::Supermarket:
		return 9;
	case UseKind::Food:
		return 7;
	case UseKind::Shop:
		return 6;
	case UseKind::Bank:
		return 5;
	case UseKind::Clinic:
	case UseKind::Gym:
		return 4;
	case UseKind::Workshop:
		return 3;
	case UseKind::Office:
		return 2;
	default:
		return 1;
	}
}
bool Use::prefers_upper_floor() const
{
	return kind == UseKind::Office || kind == UseKind::Clinic;
}

std::optional<Use> use_from_tags(const tags_t &tags)
{
	const auto amenity = tag(tags, "amenity");
	if (has_tag(tags, "amenity")) {
		if (includes({"restaurant", "food_court"}, amenity))
			return food(Eatery::Restaurant);
		if (includes({"cafe", "ice_cream"}, amenity))
			return food(Eatery::Cafe);
		if (includes({"bar", "pub", "biergarten", "nightclub"}, amenity))
			return food(Eatery::Bar);
		if (amenity == "fast_food")
			return food(Eatery::FastFood);
		if (includes({"bank", "bureau_de_change", "money_transfer", "post_office"},
					amenity))
			return simple(UseKind::Bank);
		if (amenity == "pharmacy")
			return shop(Goods::Pharmacy);
		if (includes({"school", "college", "university", "language_school",
							 "music_school", "driving_school", "training", "prep_school"},
					amenity))
			return simple(UseKind::School);
		if (includes({"kindergarten", "childcare"}, amenity))
			return simple(UseKind::Kindergarten);
		if (amenity == "library")
			return simple(UseKind::Library);
		if (includes({"doctors", "dentist", "clinic", "veterinary"}, amenity))
			return simple(UseKind::Clinic);
		if (includes({"hospital", "nursing_home"}, amenity))
			return simple(UseKind::Hospital);
		if (includes({"place_of_worship", "monastery"}, amenity))
			return worship(faith(tags));
		if (includes({"cinema", "theatre", "arts_centre", "concert_hall", "events_venue",
							 "conference_centre", "community_centre"},
					amenity))
			return simple(UseKind::Auditorium);
		if (includes({"townhall", "courthouse", "police", "fire_station",
							 "public_building", "embassy", "coworking_space",
							 "social_facility"},
					amenity))
			return simple(UseKind::Office);
		if (amenity == "marketplace")
			return shop(Goods::Grocery);
		if (includes({"car_wash", "car_rental", "vehicle_inspection"}, amenity))
			return simple(UseKind::Workshop);
		if (amenity == "gym")
			return simple(UseKind::Gym);
		if (amenity == "bus_station")
			return simple(UseKind::Station);
	}
	const auto shop_tag = tag(tags, "shop");
	if (has_tag(tags, "shop")) {
		if (shop_tag == "no" || shop_tag == "vacant")
			return std::nullopt;
		if (includes({"supermarket", "wholesale", "hypermarket"}, shop_tag))
			return simple(UseKind::Supermarket);
		if (includes({"car", "car_repair", "car_parts", "bicycle", "motorcycle", "tyres",
							 "boat"},
					shop_tag))
			return simple(UseKind::Workshop);
		return shop(goods_for_shop(shop_tag));
	}
	const auto tourism = tag(tags, "tourism");
	if (includes({"hotel", "hostel", "guest_house", "motel", "apartment"}, tourism))
		return simple(UseKind::Hotel);
	if (tourism == "museum" || tourism == "gallery")
		return simple(UseKind::Museum);
	const auto leisure = tag(tags, "leisure");
	if (includes({"fitness_centre", "fitness_station"}, leisure))
		return simple(UseKind::Gym);
	if (includes({"sports_hall", "sports_centre", "dance", "bowling_alley"}, leisure))
		return simple(UseKind::SportsHall);
	const auto healthcare = tag(tags, "healthcare");
	if (has_tag(tags, "healthcare")) {
		if (healthcare == "hospital")
			return simple(UseKind::Hospital);
		if (healthcare == "pharmacy")
			return shop(Goods::Pharmacy);
		if (healthcare == "no")
			return std::nullopt;
		return simple(UseKind::Clinic);
	}
	const auto craft = tag(tags, "craft");
	if (has_tag(tags, "craft")) {
		if (craft == "bakery" || craft == "confectionery")
			return shop(Goods::Bakery);
		if (includes({"brewery", "distillery", "winery", "metal_construction"}, craft))
			return simple(UseKind::Factory);
		if (craft == "no")
			return std::nullopt;
		return simple(UseKind::Workshop);
	}
	const auto office = tag(tags, "office");
	if (has_tag(tags, "office") && office != "no")
		return simple(UseKind::Office);
	return std::nullopt;
}

std::optional<Use> use_from_building_type(std::string_view type, const tags_t &tags)
{
	if (includes({"house", "detached", "semidetached_house", "terrace", "bungalow",
						 "villa", "cabin", "residential", "apartments", "dormitory",
						 "static_caravan", "farm", "houseboat"},
				type))
		return simple(UseKind::Home);
	if (includes({"retail", "shop", "kiosk"}, type))
		return shop(Goods::General);
	if (type == "supermarket")
		return simple(UseKind::Supermarket);
	if (includes({"commercial", "office", "civic", "public", "government", "townhall",
						 "fire_station", "police"},
				type))
		return simple(UseKind::Office);
	if (type == "hotel")
		return simple(UseKind::Hotel);
	if (includes({"industrial", "factory", "manufacture"}, type))
		return simple(UseKind::Factory);
	if (includes({"warehouse", "hangar", "storage", "depot"}, type))
		return simple(UseKind::Warehouse);
	if (includes({"school", "college", "university"}, type))
		return simple(UseKind::School);
	if (type == "kindergarten")
		return simple(UseKind::Kindergarten);
	if (type == "hospital")
		return simple(UseKind::Hospital);
	if (includes({"church", "cathedral", "chapel", "mosque", "synagogue", "temple",
						 "religious", "shrine", "monastery"},
				type))
		return worship(faith(tags));
	if (includes({"sports_hall", "sports_centre", "gymnasium", "riding_hall"}, type))
		return simple(UseKind::SportsHall);
	if (type == "train_station" || type == "transportation")
		return simple(UseKind::Station);
	if (includes({"barn", "stable", "cowshed", "sty", "farm_auxiliary", "sheepfold"},
				type))
		return simple(UseKind::Barn);
	if (type == "museum")
		return simple(UseKind::Museum);
	if (type == "library")
		return simple(UseKind::Library);
	return std::nullopt;
}

std::optional<Use> use_from_area(const tags_t &tags)
{
	const auto amenity = tag(tags, "amenity");
	if (includes({"school", "college", "university"}, amenity))
		return simple(UseKind::School);
	if (includes({"kindergarten", "childcare"}, amenity))
		return simple(UseKind::Kindergarten);
	if (amenity == "hospital")
		return simple(UseKind::Hospital);
	if (amenity == "place_of_worship")
		return worship(faith(tags));
	if (amenity == "bus_station")
		return simple(UseKind::Station);
	if (tag(tags, "leisure") == "sports_centre")
		return simple(UseKind::SportsHall);
	if (tag(tags, "tourism") == "museum")
		return simple(UseKind::Museum);
	const auto landuse = tag(tags, "landuse");
	if (landuse == "industrial")
		return simple(UseKind::Factory);
	if (landuse == "farmyard")
		return simple(UseKind::Barn);
	if (landuse == "retail")
		return shop(Goods::General);
	return std::nullopt;
}

bool area_use_fits(Use use, std::size_t footprint)
{
	if (use.kind == UseKind::SportsHall)
		return footprint >= 300;
	if (use.kind == UseKind::Factory)
		return footprint >= 80;
	return true;
}

std::optional<int> parse_level(std::string_view value)
{
	const auto end = value.find_first_of(";,");
	value = value.substr(0, end);
	const auto first = value.find_first_not_of(" \t\n\r\f\v");
	if (first == std::string_view::npos)
		return std::nullopt;
	const auto last = value.find_last_not_of(" \t\n\r\f\v");
	try {
		std::size_t consumed = 0;
		const double level =
				std::stod(std::string(value.substr(first, last - first + 1)), &consumed);
		if (consumed != last - first + 1)
			return std::nullopt;
		// Rust's `f64::round() as i32` maps NaN to zero and saturates infinities
		// and out-of-range values; preserve those cast semantics for malformed tags.
		if (std::isnan(level))
			return 0;
		if (level <= std::numeric_limits<int>::min())
			return std::numeric_limits<int>::min();
		if (level >= std::numeric_limits<int>::max())
			return std::numeric_limits<int>::max();
		return static_cast<int>(std::lround(level));
	} catch (...) {
		return std::nullopt;
	}
}

InteriorPlan plan_interior(const PlanInputs &input)
{
	const std::size_t floors = std::max<std::size_t>(1, input.floors);
	const auto own = use_from_tags(input.tags);
	const auto tourism = tag(input.tags, "tourism");
	const std::optional<Use> lodging =
			includes({"hotel", "hostel", "guest_house", "motel"}, tourism)
					? std::optional<Use>(simple(UseKind::Hotel))
					: std::nullopt;
	auto typed = use_from_building_type(input.building_type, input.tags);
	if (!typed)
		typed = lodging;
	const auto area = !typed && input.area && area_use_fits(*input.area, input.footprint)
							  ? input.area
							  : std::nullopt;
	const auto base = typed ? typed : area;
	Use ground_default = simple(UseKind::Home), upper_default = ground_default;
	if (own && base && own->is_street_tenant() && *own != *base &&
			base->hosts_tenants()) {
		ground_default = *own;
		upper_default = *base;
	} else if (own && !base && own->is_street_tenant() && floors >= 2) {
		ground_default = *own;
		upper_default = simple(UseKind::Home);
	} else if (own) {
		ground_default = upper_default = *own;
	} else if (base) {
		ground_default = upper_default = *base;
	}
	const Use host = base ? *base : own ? *own : simple(UseKind::Home);
	std::vector<const Tenant *> tenants;
	for (const auto &tenant : input.tenants)
		if ((!typed || tenant.use != *typed) && (!own || tenant.use != *own))
			tenants.push_back(&tenant);
	const auto levels = tag(input.tags, "building:levels");
	bool levels_tagged = false;
	if (!levels.empty()) {
		try {
			const auto first = levels.find_first_not_of(" \t\n\r\f\v");
			const auto last = levels.find_last_not_of(" \t\n\r\f\v");
			if (first != std::string_view::npos) {
				std::size_t consumed = 0;
				const auto value = std::string(levels.substr(first, last - first + 1));
				const double parsed = std::stod(value, &consumed);
				levels_tagged = consumed == value.size() && parsed >= 2.0;
			}
		} catch (...) {
		}
	}
	const bool open_hall = tenants.empty() && ground_default == upper_default &&
						   ground_default.is_hall() &&
						   (ground_default.always_open() || !levels_tagged);
	if (open_hall)
		return {{{Unit{ground_default, input.center}}}, true};

	std::vector<std::vector<Unit>> per_floor(floors);
	const int top = static_cast<int>(floors) - 1;
	const bool height_mapped = input.tags.find("building:levels") != input.tags.end() ||
							   input.tags.find("height") != input.tags.end();
	auto floor_of = [&](std::optional<int> level) -> std::optional<std::size_t> {
		if (input.elevated && input.min_level == 0)
			return std::nullopt;
		if (level) {
			if (*level < input.min_level ||
					(height_mapped && *level - input.min_level > top))
				return std::nullopt;
			return static_cast<std::size_t>(std::min(*level - input.min_level, top));
		}
		return input.elevated ? std::nullopt : std::optional<std::size_t>(0);
	};
	auto push = [&](std::size_t floor, const Unit &unit) {
		const auto duplicate = std::find_if(per_floor[floor].begin(),
				per_floor[floor].end(), [&](const Unit &existing) {
					return existing.use == unit.use &&
						   std::abs(existing.anchor.first - unit.anchor.first) <=
								   SAME_TENANT_DIST &&
						   std::abs(existing.anchor.second - unit.anchor.second) <=
								   SAME_TENANT_DIST;
				});
		if (duplicate == per_floor[floor].end())
			per_floor[floor].push_back(unit);
	};
	std::stable_sort(
			tenants.begin(), tenants.end(), [](const Tenant *a, const Tenant *b) {
				return a->use.prefers_upper_floor() && !a->level &&
					   !(b->use.prefers_upper_floor() && !b->level);
			});
	for (const Tenant *tenant : tenants) {
		auto floor = floor_of(tenant->level);
		if (!floor)
			continue;
		if (!tenant->level && tenant->use.prefers_upper_floor() && floors >= 2 &&
				std::any_of(per_floor[0].begin(), per_floor[0].end(),
						[](const Unit &unit) { return unit.use.is_street_tenant(); }))
			*floor = 1;
		push(*floor, Unit{tenant->use, {tenant->x, tenant->z}});
	}
	const std::size_t max_units =
			std::max<std::size_t>(1, input.footprint / CELLS_PER_UNIT);
	for (std::size_t i = 0; i < per_floor.size(); ++i) {
		auto &units = per_floor[i];
		if (units.empty()) {
			units.push_back({i == 0 && !input.elevated ? ground_default : upper_default,
					input.center});
			continue;
		}
		const bool explicit_host = base.has_value() || floors >= 3;
		if (explicit_host && input.footprint >= HOST_SHARE_CELLS * (units.size() + 1) &&
				std::none_of(units.begin(), units.end(),
						[&](const Unit &unit) { return unit.use == host; }))
			units.push_back({host, input.center});
		if (units.size() > max_units) {
			std::stable_sort(
					units.begin(), units.end(), [](const Unit &a, const Unit &b) {
						return a.use.rank() > b.use.rank();
					});
			units.resize(max_units);
		}
	}
	return {std::move(per_floor), false};
}
} // namespace arnis::interior_uses
