#pragma once

#include "../floodfill_cache.h"

#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace arnis::element_processing::kerb_strips
{

struct StripCell
{
	std::pair<int, int> cell;
	std::pair<int, int> from;
};

class KerbStrips
{
public:
	const std::vector<StripCell> &of(std::uint64_t way) const;
	void add(std::uint64_t way, StripCell cell);
	std::vector<std::pair<int, int>> cells() const;

private:
	std::unordered_map<std::uint64_t, std::vector<StripCell>> by_way_;
};

class KerbWays
{
public:
	static KerbWays stamp(const std::vector<ProcessedElement> &elements,
			const WorldEditor &editor, const ::XZBBox &bbox, double scale);
	KerbStrips strips(const WorldEditor &editor, double scale,
			const CoordinateBitmap &roads, const CoordinateBitmap &footprints,
			const CoordinateBitmap &blocked) const;

private:
	struct Sidewalk
	{
		const ProcessedWay *way{};
		int half_width{};
	};
	CoordinateBitmap vehicles_;
	CoordinateBitmap walkways_;
	std::vector<Sidewalk> sidewalks_;
	explicit KerbWays(const ::XZBBox &bbox) : vehicles_(bbox), walkways_(bbox) {}
};

} // namespace arnis::element_processing::kerb_strips
