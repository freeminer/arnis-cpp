#include "pictograms.h"
#include "../assets_root.h"
#include <unordered_map>
namespace arnis::decals::pictograms
{
const std::vector<std::string> &names()
{
	static const std::vector<std::string> value{"atm", "bus_stop", "hydrant",
			"information", "metro_m", "metro_s", "metro_u", "parking", "recycling",
			"train", "tram", "vending_machine"};
	return value;
}
std::optional<std::filesystem::path> asset(
		const std::string &name, const std::filesystem::path &root)
{
	if (std::find(names().begin(), names().end(), name) == names().end())
		return std::nullopt;
	const auto &directory = root.empty() ? assets::path("decorations/pictograms") : root;
	auto path = directory / (name + ".png");
	return std::filesystem::exists(path) ? std::optional{path} : std::nullopt;
}
std::optional<std::string> business_kind(const tags_t &tags)
{
	static const std::unordered_map<std::string, std::string> shops{
			{"supermarket", "supermarket"}, {"department_store", "supermarket"},
			{"mall", "supermarket"}, {"wholesale", "supermarket"},
			{"general", "supermarket"}, {"variety_store", "supermarket"},
			{"convenience", "convenience"}, {"grocery", "convenience"},
			{"frozen_food", "convenience"}, {"health_food", "convenience"},
			{"organic", "convenience"}, {"farm", "convenience"}, {"bakery", "bakery"},
			{"pastry", "bakery"}, {"butcher", "butcher"}, {"seafood", "butcher"},
			{"cheese", "butcher"}, {"deli", "butcher"}, {"greengrocer", "greengrocer"},
			{"confectionery", "sweets"}, {"chocolate", "sweets"}, {"candy", "sweets"},
			{"alcohol", "alcohol"}, {"wine", "alcohol"}, {"beverages", "alcohol"},
			{"brewing_supplies", "alcohol"}, {"coffee", "coffee_shop"},
			{"tea", "coffee_shop"}, {"clothes", "clothes"}, {"boutique", "clothes"},
			{"fashion", "clothes"}, {"fashion_accessories", "clothes"},
			{"tailor", "clothes"}, {"fabric", "clothes"}, {"second_hand", "clothes"},
			{"charity", "clothes"}, {"baby_goods", "clothes"}, {"sewing", "clothes"},
			{"leather", "clothes"}, {"shoes", "shoes"}, {"shoe_repair", "shoes"},
			{"books", "books"}, {"stationery", "books"}, {"newsagent", "books"},
			{"copyshop", "books"}, {"printing", "books"}, {"electronics", "electronics"},
			{"computer", "electronics"}, {"hifi", "electronics"},
			{"video", "electronics"}, {"video_games", "electronics"},
			{"appliance", "electronics"}, {"electrical", "electronics"},
			{"radiotechnics", "electronics"}, {"vacuum_cleaner", "electronics"},
			{"camera", "electronics"}, {"mobile_phone", "mobile_phone"},
			{"telecommunication", "mobile_phone"}, {"optician", "optician"},
			{"jewelry", "jewelry"}, {"watches", "jewelry"}, {"gold_buyer", "jewelry"},
			{"florist", "florist"}, {"garden_centre", "garden"}, {"agrarian", "garden"},
			{"landscaping", "garden"}, {"hardware", "hardware"},
			{"doityourself", "hardware"}, {"paint", "hardware"}, {"tiles", "hardware"},
			{"trade", "hardware"}, {"locksmith", "hardware"}, {"tool_hire", "hardware"},
			{"glaziery", "hardware"}, {"houseware", "hardware"},
			{"bathroom_furnishing", "hardware"}, {"flooring", "hardware"},
			{"kitchen", "hardware"}, {"lighting", "hardware"}, {"curtain", "hardware"},
			{"window_blind", "hardware"}, {"energy", "hardware"},
			{"fireplace", "hardware"}, {"security", "hardware"},
			{"furniture", "furniture"}, {"interior_decoration", "furniture"},
			{"bed", "furniture"}, {"antiques", "furniture"}, {"carpet", "furniture"},
			{"frame", "furniture"}, {"toys", "toys"}, {"gift", "gift"}, {"party", "gift"},
			{"craft", "gift"}, {"art", "gift"}, {"hobby", "gift"}, {"model", "gift"},
			{"collector", "gift"}, {"games", "gift"}, {"anime", "gift"}, {"pet", "pet"},
			{"pet_grooming", "pet"}, {"bicycle", "bicycle"}, {"e-bike", "bicycle"},
			{"scooter", "bicycle"}, {"motorcycle", "bicycle"},
			{"motorcycle_repair", "bicycle"}, {"sports", "sports"}, {"outdoor", "sports"},
			{"fishing", "sports"}, {"hunting", "sports"}, {"golf", "sports"},
			{"ski", "sports"}, {"swimming_pool", "sports"}, {"weapons", "sports"},
			{"military_surplus", "sports"}, {"boat", "sports"},
			{"hairdresser", "hairdresser"}, {"beauty", "hairdresser"},
			{"cosmetics", "hairdresser"}, {"perfumery", "hairdresser"},
			{"massage", "hairdresser"}, {"tattoo", "hairdresser"},
			{"hairdresser_supply", "hairdresser"}, {"nails", "hairdresser"},
			{"erotic", "hairdresser"}, {"laundry", "laundry"},
			{"dry_cleaning", "laundry"}, {"kiosk", "kiosk"}, {"tobacco", "kiosk"},
			{"e-cigarette", "kiosk"}, {"cannabis", "kiosk"}, {"lottery", "kiosk"},
			{"bookmaker", "kiosk"}, {"vacant", "kiosk"}, {"chemist", "chemist"},
			{"medical_supply", "chemist"}, {"hearing_aids", "chemist"},
			{"nutrition_supplements", "chemist"}, {"herbalist", "chemist"},
			{"music", "music"}, {"musical_instrument", "music"}, {"car", "car"},
			{"car_parts", "car"}, {"tyres", "car"}, {"caravan", "car"},
			{"trailer", "car"}, {"truck", "car"}, {"atv", "car"}, {"snowmobile", "car"},
			{"car_repair", "car_repair"}, {"photo", "photo"}, {"photo_studio", "photo"},
			{"travel_agency", "office"}, {"estate_agent", "office"},
			{"insurance", "office"}, {"money_lender", "office"}, {"pawnbroker", "office"},
			{"funeral_directors", "office"}, {"storage_rental", "office"},
			{"outpost", "office"}, {"ticket", "office"}, {"religion", "office"},
			{"rental", "office"}, {"laundry_service", "office"},
			{"dry_cleaning_service", "office"}, {"bag", "bag"}, {"boutique_bag", "bag"}};
	static const std::unordered_map<std::string, std::string> amenities{
			{"restaurant", "restaurant"}, {"food_court", "restaurant"},
			{"biergarten", "restaurant"}, {"canteen", "restaurant"}, {"cafe", "cafe"},
			{"internet_cafe", "cafe"}, {"bar", "bar"}, {"nightclub", "nightclub"},
			{"stripclub", "nightclub"}, {"hookah_lounge", "nightclub"},
			{"love_hotel", "nightclub"}, {"pub", "pub"}, {"fast_food", "fast_food"},
			{"ice_cream", "ice_cream"}, {"pharmacy", "pharmacy"}, {"dentist", "dentist"},
			{"doctors", "doctors"}, {"clinic", "doctors"}, {"nursing_home", "doctors"},
			{"social_facility", "doctors"}, {"healthcare", "doctors"},
			{"hospital", "hospital"}, {"veterinary", "veterinary"},
			{"animal_shelter", "veterinary"}, {"animal_boarding", "veterinary"},
			{"bank", "bank"}, {"bureau_de_change", "money_exchange"},
			{"money_transfer", "money_exchange"}, {"payment_centre", "money_exchange"},
			{"atm", "atm"}, {"post_office", "post"}, {"post_box", "post"},
			{"post_depot", "post"}, {"parcel_locker", "post"}, {"courier", "post"},
			{"police", "police"}, {"prison", "police"}, {"customs", "police"},
			{"ranger_station", "police"}, {"fire_station", "fire_station"},
			{"rescue_station", "fire_station"}, {"townhall", "townhall"},
			{"courthouse", "townhall"}, {"public_building", "townhall"},
			{"register_office", "townhall"}, {"archive", "townhall"},
			{"coworking_space", "office"}, {"conference_centre", "office"},
			{"exhibition_centre", "office"}, {"events_venue", "office"},
			{"studio", "office"}, {"public_bath", "office"}, {"school", "school"},
			{"college", "school"}, {"university", "school"},
			{"language_school", "school"}, {"music_school", "school"},
			{"prep_school", "school"}, {"research_institute", "school"},
			{"training", "school"}, {"kindergarten", "kindergarten"},
			{"childcare", "kindergarten"}, {"driving_school", "driving_school"},
			{"library", "library"}, {"public_bookcase", "library"},
			{"toy_library", "library"}, {"cinema", "cinema"}, {"theatre", "theatre"},
			{"arts_centre", "theatre"}, {"concert_hall", "theatre"},
			{"planetarium", "theatre"}, {"community_centre", "community"},
			{"social_centre", "community"}, {"youth_centre", "community"},
			{"place_of_mourning", "community"}, {"senior_centre", "community"},
			{"shelter_home", "community"}, {"place_of_worship", "place_of_worship"},
			{"monastery", "place_of_worship"}, {"crematorium", "place_of_worship"},
			{"toilets", "toilets"}, {"shower", "toilets"}, {"parking", "parking"},
			{"parking_entrance", "parking"}, {"motorcycle_parking", "parking"},
			{"charging_station", "charging_station"}, {"fuel", "fuel"},
			{"compressed_air", "fuel"}, {"car_wash", "car_wash"}, {"car_rental", "car"},
			{"car_sharing", "car"}, {"car_pooling", "car"}, {"vehicle_inspection", "car"},
			{"bicycle_rental", "bicycle"}, {"bicycle_repair_station", "bicycle"},
			{"taxi", "taxi"}, {"vending_machine", "vending_machine"},
			{"marketplace", "marketplace"}, {"casino", "casino"}, {"gambling", "casino"},
			{"embassy", "embassy"}, {"consulate", "embassy"},
			{"swimming_pool", "swimming"}, {"public_bath_house", "swimming"},
			{"gym", "gym"}, {"fitness_centre", "gym"}, {"dojo", "gym"}};
	static const std::unordered_map<std::string, std::string> tourism{{"hotel", "hotel"},
			{"hostel", "hotel"}, {"guest_house", "hotel"}, {"motel", "hotel"},
			{"apartment", "hotel"}, {"chalet", "hotel"}, {"alpine_hut", "hotel"},
			{"wilderness_hut", "hotel"}, {"camp_site", "hotel"},
			{"caravan_site", "hotel"}, {"museum", "museum"}, {"gallery", "museum"},
			{"aquarium", "museum"}, {"attraction", "attraction"},
			{"theme_park", "attraction"}, {"zoo", "attraction"},
			{"artwork", "attraction"}, {"viewpoint", "viewpoint"},
			{"information", "information"}};
	static const std::unordered_map<std::string, std::string> leisure{
			{"fitness_centre", "gym"}, {"sports_centre", "gym"}, {"sports_hall", "gym"},
			{"fitness_station", "gym"}, {"swimming_pool", "swimming"},
			{"water_park", "swimming"}, {"bowling_alley", "attraction"},
			{"amusement_arcade", "attraction"}, {"escape_game", "attraction"},
			{"adult_gaming_centre", "attraction"}, {"miniature_golf", "attraction"},
			{"trampoline_park", "attraction"}, {"dance", "attraction"}};
	const auto lookup = [](const auto &values,
								const std::string &key) -> std::optional<std::string> {
		auto it = values.find(key);
		return it == values.end() ? std::nullopt : std::optional(it->second);
	};
	if (const auto it = tags.find("shop"); it != tags.end()) {
		if (auto result = lookup(shops, it->second))
			return result;
		return "shop";
	}
	if (const auto it = tags.find("amenity"); it != tags.end())
		if (auto result = lookup(amenities, it->second))
			return result;
	if (const auto it = tags.find("tourism"); it != tags.end())
		if (auto result = lookup(tourism, it->second))
			return result;
	if (const auto it = tags.find("leisure"); it != tags.end())
		if (auto result = lookup(leisure, it->second))
			return result;
	if (const auto it = tags.find("healthcare"); it != tags.end()) {
		if (it->second == "dentist")
			return "dentist";
		if (it->second == "pharmacy")
			return "pharmacy";
		if (it->second == "hospital")
			return "hospital";
		return "doctors";
	}
	if (tags.contains("office"))
		return "office";
	if (tags.contains("craft"))
		return "hardware";
	return std::nullopt;
}
}
