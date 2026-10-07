#pragma once

#include "../../../../../arnis_types.h"

#include <cstddef>
#include <initializer_list>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

namespace arnis::interior_uses
{
enum class Goods
{
	General,
	Bakery,
	Butcher,
	Grocery,
	Books,
	Clothes,
	Electronics,
	Hardware,
	Pharmacy,
	Florist,
	Jewelry,
	Toys,
	Furniture,
	Drinks,
	Salon
};
enum class Eatery
{
	Restaurant,
	Cafe,
	Bar,
	FastFood
};
enum class Faith
{
	Christian,
	Jewish,
	Muslim,
	Other
};
enum class UseKind
{
	Home,
	Shop,
	Supermarket,
	Food,
	Office,
	Bank,
	Workshop,
	School,
	Kindergarten,
	Library,
	Clinic,
	Hospital,
	Hotel,
	Museum,
	Station,
	Worship,
	SportsHall,
	Gym,
	Auditorium,
	Warehouse,
	Factory,
	Barn
};

struct Use
{
	UseKind kind{UseKind::Home};
	Goods goods{Goods::General};
	Eatery eatery{Eatery::Restaurant};
	Faith faith{Faith::Christian};
	friend bool operator==(const Use &, const Use &) = default;
	bool is_street_tenant() const;
	bool is_hall() const;
	bool always_open() const;
	bool hosts_tenants() const;
	int rank() const;
	bool prefers_upper_floor() const;
};

std::optional<Use> use_from_tags(const tags_t &tags);
std::optional<Use> use_from_building_type(
		std::string_view building_type, const tags_t &tags);
std::optional<Use> use_from_area(const tags_t &tags);
bool area_use_fits(Use use, std::size_t footprint);
std::optional<int> parse_level(std::string_view value);

struct Tenant
{
	Use use;
	int x{0};
	int z{0};
	std::optional<int> level;
};
// Walkable cell immediately inside an outer door and its inward direction.
struct Entry
{
	std::pair<int, int> cell;
	std::pair<int, int> inward;
};
struct Unit
{
	Use use;
	std::pair<int, int> anchor;
};
struct InteriorPlan
{
	std::vector<std::vector<Unit>> floors;
	bool open_hall{false};
};
struct PlanInputs
{
	const tags_t &tags;
	std::string_view building_type;
	std::size_t floors{1};
	int min_level{0};
	bool elevated{false};
	std::size_t footprint{0};
	std::pair<int, int> center{0, 0};
	const std::vector<Tenant> &tenants;
	std::optional<Use> area;
};

InteriorPlan plan_interior(const PlanInputs &input);
} // namespace arnis::interior_uses
