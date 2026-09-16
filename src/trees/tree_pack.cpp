#include "tree_pack.h"
#include "region.h"
#include <cmath>
#include <utility>
namespace arnis::trees
{
Habitat habitat_from_string(const std::string &s)
{
	return s == "conifer"	 ? Habitat::Conifer
		   : s == "wet"		 ? Habitat::Wet
		   : s == "dry"		 ? Habitat::Dry
		   : s == "tropical" ? Habitat::Tropical
							 : Habitat::Lowland;
}
RegionLibrary load_region_library(
		double lat, double lon, const std::filesystem::path &root)
{
	const auto realm = realm_for_latlon(lat, lon);
	TreePackSource source(realm, root);
	auto manifest = source.realm_path("region.json");
	if (!std::filesystem::exists(manifest))
		manifest = source.vanilla_path("region.json");
	return RegionLibrary::load(manifest);
}
std::string realm_for_latlon(double lat, double lon)
{
	struct B
	{
		const char *n;
		double a, b, c, d;
	};
	static constexpr B boxes[] = {{"fl", 8, 31, -90, -60}, {"ena", 8, 62, -100, -52},
			{"wna", 25, 72, -170, -100}, {"sam", -56, 14, -82, -34},
			{"eur", 34, 72, -25, 40}, {"afr", -36, 37, -19, 52},
			{"ind", -11, 29, 60, 155}, {"asn", 5, 75, 40, 155}, {"aus", -50, 0, 110, 180},
			{"aus", -50, 32, -180, -130}};
	for (const auto &x : boxes)
		if (lat >= x.a && lat <= x.b && lon >= x.c && lon <= x.d)
			return x.n;
	return "vanilla-plus";
}
bool exclude_palms_for_latitude(double lat)
{
	return std::abs(lat) > 35.0;
}
TreePackSource::TreePackSource(std::string r, std::filesystem::path root) :
		realm_(std::move(r)), root_(std::move(root))
{
}
TreePackSource TreePackSource::embedded(
		const std::string &realm, std::filesystem::path root)
{
	if (root.empty()) {
		root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
			   "assets" / "tree-packs";
	}
	return TreePackSource(realm, std::move(root));
}
std::string TreePackSource::realm_file(const std::string &r) const
{
	return realm_ + "/" + r;
}
std::string TreePackSource::vanilla_file(const std::string &r) const
{
	return "vanilla-plus/" + r;
}
std::filesystem::path TreePackSource::realm_path(const std::string &r) const
{
	return root_ / realm_ / r;
}
std::filesystem::path TreePackSource::vanilla_path(const std::string &r) const
{
	return root_ / "vanilla-plus" / r;
}
std::optional<std::filesystem::path> TreePackSource::realm_manifest() const
{
	const auto p = realm_path("region.json");
	return std::filesystem::is_regular_file(p) ? std::optional{p} : std::nullopt;
}
std::optional<std::filesystem::path> TreePackSource::vanilla_manifest() const
{
	const auto p = vanilla_path("region.json");
	return std::filesystem::is_regular_file(p) ? std::optional{p} : std::nullopt;
}
bool TreePackSource::has_realm_file(const std::string &r) const
{
	return std::filesystem::is_regular_file(realm_path(r));
}
bool TreePackSource::has_vanilla_file(const std::string &r) const
{
	return std::filesystem::is_regular_file(vanilla_path(r));
}
std::filesystem::path resolve_tree_asset(const TreePackSource &s, const std::string &r)
{
	if (s.has_realm_file(r))
		return s.realm_path(r);
	if (s.has_vanilla_file(r))
		return s.vanilla_path(r);
	return {};
}
RegionLibrary load_combined_region_library(
		double lat, double lon, const std::filesystem::path &root)
{
	TreePackSource s(realm_for_latlon(lat, lon), root);
	const auto realm = s.realm_path("region.json"),
			   vanilla = s.vanilla_path("region.json");
	if (std::filesystem::is_regular_file(realm) &&
			std::filesystem::is_regular_file(vanilla))
		return RegionLibrary::combine(realm, vanilla);
	if (std::filesystem::is_regular_file(realm))
		return RegionLibrary::load(realm);
	if (std::filesystem::is_regular_file(vanilla))
		return RegionLibrary::load(vanilla);
	return {};
}
}
