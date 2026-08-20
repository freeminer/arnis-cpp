#include "pictograms.h"
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
	auto path = root / (name + ".png");
	return std::filesystem::exists(path) ? std::optional{path} : std::nullopt;
}
std::optional<std::string> business_kind(const tags_t &tags)
{
	static const std::unordered_map<std::string, std::string> amenity{
			{"restaurant", "restaurant"}, {"cafe", "cafe"}, {"bar", "bar"},
			{"pub", "pub"}, {"fast_food", "fast_food"}, {"pharmacy", "pharmacy"},
			{"dentist", "dentist"}, {"hospital", "hospital"}, {"bank", "bank"},
			{"atm", "atm"}, {"post_office", "post"}, {"police", "police"},
			{"fire_station", "fire_station"}, {"school", "school"},
			{"library", "library"}, {"cinema", "cinema"}, {"theatre", "theatre"},
			{"parking", "parking"}, {"fuel", "fuel"},
			{"vending_machine", "vending_machine"}};
	static const std::unordered_map<std::string, std::string> shop{
			{"supermarket", "supermarket"}, {"convenience", "convenience"},
			{"bakery", "bakery"}, {"butcher", "butcher"}, {"clothes", "clothes"},
			{"shoes", "shoes"}, {"books", "books"}, {"electronics", "electronics"},
			{"mobile_phone", "mobile_phone"}, {"optician", "optician"},
			{"jewelry", "jewelry"}, {"florist", "florist"}, {"hardware", "hardware"},
			{"furniture", "furniture"}, {"toys", "toys"}, {"gift", "gift"},
			{"pet", "pet"}, {"bicycle", "bicycle"}, {"hairdresser", "hairdresser"},
			{"laundry", "laundry"}, {"chemist", "chemist"}, {"car", "car"},
			{"car_repair", "car_repair"}};
	if (const auto value = tags.get("shop"); !value.empty()) {
		if (auto it = shop.find(value); it != shop.end())
			return it->second;
		return "shop";
	}
	if (const auto value = tags.get("amenity"); !value.empty())
		if (auto it = amenity.find(value); it != amenity.end())
			return it->second;
	if (const auto value = tags.get("tourism");
			value == "hotel" || value == "hostel" || value == "motel")
		return "hotel";
	if (tags.contains("office"))
		return "office";
	if (tags.contains("craft"))
		return "hardware";
	return std::nullopt;
}
}
