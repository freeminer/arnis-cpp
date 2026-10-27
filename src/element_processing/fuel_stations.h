#pragma once

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"

#include <chrono>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

namespace arnis::fuel_stations
{

struct Livery
{
	Block fascia, stripe, column;
	std::optional<Block> stripe2;
	Livery() = default;
	Livery(Block fascia_, Block stripe_, Block column_,
			std::optional<Block> stripe2_ = std::nullopt) :
			fascia(fascia_), stripe(stripe_), column(column_), stripe2(stripe2_)
	{
	}
};

/// Resolve a station's brand colors from its tags, or choose a seeded fallback.
Livery livery_for(const tags_t &tags, std::uint64_t seed);

struct Site
{
	std::uint64_t id{};
	double x{}, z{};
	std::vector<std::pair<double, double>> ring;
	Livery livery{};
	bool has_canopy{}, has_shop{}, building{};
};

class FuelIndex
{
public:
	std::vector<Site> sites;
	std::unordered_map<std::uint64_t, std::size_t> canopies;
	static FuelIndex build(const std::vector<ProcessedElement> &elements, double scale);
	const Site *site(std::uint64_t id) const;
	const Site *site_for_canopy(std::uint64_t way_id) const;
};

bool is_station_canopy(const WorldEditor &editor, const ProcessedWay &way);
bool generate_canopy(WorldEditor &editor, const ProcessedWay &way,
		const FloodFillCache &fills, const RoadMaskBitmap &roads, double scale,
		const std::optional<std::chrono::milliseconds> &timeout);
void generate_station(WorldEditor &editor, const ProcessedElement &element,
		const FloodFillCache &fills, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &roads, double scale,
		const std::optional<std::chrono::milliseconds> &timeout);

} // namespace arnis::fuel_stations
