#include "choose.h"
#include <algorithm>
#include <cmath>
namespace arnis::building_facades
{
namespace
{
constexpr double NOMINAL = 3.1, STOREY = .35, PARTIAL = .30, NO_GROUND = .15,
				 WINDOW = .45;
long long floor_div(long long a, long long b)
{
	auto q = a / b, r = a % b;
	return r && ((r < 0) != (b < 0)) ? q - 1 : q;
}
std::uint64_t pair_hash(long long a, long long b)
{
	return hash64(hash64(std::uint64_t(a)) ^ (std::uint64_t(b) * 0x9E3779B97F4A7C15ULL));
}

std::vector<buildings::BuildingCategory> related(buildings::BuildingCategory c)
{
	using C = buildings::BuildingCategory;
	switch (c) {
	case C::Residential:
		return {C::House, C::TallBuilding, C::Hotel};
	case C::House:
		return {C::Residential, C::Farm};
	case C::Farm:
		return {C::House, C::Warehouse, C::Shed};
	case C::Commercial:
		return {C::Office, C::Residential};
	case C::Office:
		return {C::Commercial, C::TallBuilding};
	case C::Hotel:
		return {C::Residential, C::Office};
	case C::Industrial:
		return {C::Warehouse, C::Garage};
	case C::Warehouse:
		return {C::Industrial, C::Garage};
	case C::School:
		return {C::Office, C::Historic};
	case C::Hospital:
		return {C::Office, C::School};
	case C::Religious:
		return {C::Historic};
	case C::TallBuilding:
		return {C::Office, C::Residential};
	case C::GlassySkyscraper:
	case C::GlassCornerSkyscraper:
	case C::GridSkyscraper:
	case C::ContemporarySkyscraper:
	case C::ModernSkyscraper:
		return {C::TallBuilding, C::Office};
	case C::MasonrySkyscraper:
		return {C::TallBuilding, C::Historic, C::Office};
	case C::Historic:
		return {C::Religious, C::Residential};
	case C::Tower:
		return {C::Historic, C::TallBuilding};
	case C::Garage:
		return {C::Warehouse, C::Shed, C::Industrial};
	case C::Shed:
		return {C::Garage, C::Warehouse};
	case C::Greenhouse:
		return {C::Warehouse, C::Shed};
	case C::Default:
		return {C::Residential, C::Commercial, C::House};
	}
	return {};
}
}
}
namespace arnis::building_facades
{
std::uint64_t hash64(std::uint64_t x)
{
	x += 0x9E3779B97F4A7C15ULL;
	x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
	x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
	return x ^ (x >> 31);
}
double score(const Entry &e, double h)
{
	const auto sm = e.storey_m();
	return std::abs(std::log(std::max(.5, h) / e.metres_tall)) +
		   STOREY * std::abs(sm - NOMINAL) / NOMINAL +
		   PARTIAL * std::abs(h / sm - std::round(h / sm)) +
		   (e.has_ground_floor ? 0 : NO_GROUND);
}
std::vector<std::size_t> shortlist(
		const std::vector<Entry> &entries, buildings::BuildingCategory category, double h)
{
	std::vector<std::size_t> out;
	for (std::size_t i = 0; i < entries.size(); ++i)
		if (std::find(entries[i].categories.begin(), entries[i].categories.end(),
					category) != entries[i].categories.end())
			out.push_back(i);
	if (out.empty())
		for (const auto related_category : related(category))
			for (std::size_t i = 0; i < entries.size(); ++i)
				if (std::find(entries[i].categories.begin(), entries[i].categories.end(),
							related_category) != entries[i].categories.end() &&
						std::find(out.begin(), out.end(), i) == out.end())
					out.push_back(i);
	if (out.empty())
		for (std::size_t i = 0; i < entries.size(); ++i)
			if (std::find(entries[i].categories.begin(), entries[i].categories.end(),
						buildings::BuildingCategory::Default) !=
					entries[i].categories.end())
				out.push_back(i);
	if (out.empty())
		for (std::size_t i = 0; i < entries.size(); ++i)
			out.push_back(i);
	std::sort(out.begin(), out.end(), [&](auto a, auto b) {
		auto sa = score(entries[a], h), sb = score(entries[b], h);
		return sa == sb ? entries[a].file < entries[b].file : sa < sb;
	});
	if (!out.empty()) {
		auto cut = score(entries[out[0]], h) + WINDOW;
		out.erase(std::remove_if(out.begin(), out.end(),
						  [&](auto i) { return score(entries[i], h) > cut; }),
				out.end());
	}
	if (out.size() > 8)
		out.resize(8);
	return out;
}
std::size_t slot(int x, int z, std::size_t n)
{
	if (n < 2)
		return 0;
	auto cx = floor_div(x, 8), cz = floor_div(z, 8);
	auto rot = pair_hash(floor_div(cx, 3), floor_div(cz, 3)) % n;
	auto v = (cx + 2 * cz + static_cast<long long>(rot)) % static_cast<long long>(n);
	return std::size_t(v < 0 ? v + static_cast<long long>(n) : v);
}
std::optional<Choice> choose(const std::vector<Entry> &e, buildings::BuildingCategory c,
		double h, std::uint64_t id, int x, int z)
{
	auto list = shortlist(e, c, h);
	if (list.empty())
		return {};
	auto i = list[slot(x, z, list.size())];
	return Choice{i, double(hash64(id) % 16) / 16 * e[i].metres_wide};
}
}
