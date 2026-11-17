#include "fm_rail_passages.h"

#include "strict_parse.h"

#include <algorithm>
#include <cmath>

namespace arnis::railways
{

const RailPassages &RailPassages::none()
{
	static const RailPassages none{};
	return none;
}

RailThrough RailPassages::through(const std::vector<std::pair<int, int>> &area) const
{
	if (cells.is_empty())
		return RailThrough::None;
	if (cells.count_contained(area) == 0)
		return RailThrough::None;
	if (std::any_of(area.begin(), area.end(), [&](const auto &point) {
			return under.contains(point.first, point.second);
		}))
		return RailThrough::Under;
	if (track.count_contained(area) >= runs_in)
		return RailThrough::Into;
	return RailThrough::Clips;
}

RailPassages collect_rail_passages(const std::vector<ProcessedElement> &elements,
		const XZBBox &xzbbox, double scale,
		const BuildingFootprintBitmap &building_footprints)
{
	constexpr double TRAIN_HEADROOM_M = 5.5;
	constexpr double TRAIN_REACH_M = 2.0;
	constexpr double RUNS_IN_M = 8.0;
	const int reach = std::max(1, static_cast<int>(std::round(TRAIN_REACH_M * scale)));
	std::optional<CoordinateBitmap> cells, under, track;
	if (!building_footprints.is_empty()) {
		const auto near_building = [&](int x, int z) {
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz)
					if (building_footprints.contains(x + dx, z + dz))
						return true;
			return false;
		};
		for (const auto &element : elements) {
			if (!element.is_way())
				continue;
			const auto &way = element.as_way();
			const auto railway = way.tags.get("railway");
			if (!is_at_grade_track(way) || railway == "disused" || railway == "miniature")
				continue;
			const auto layer = strict_parse::i32(way.tags.get("layer"));
			const bool passes_under = way.tags.get("tunnel") == "building_passage" ||
									  way.tags.get("covered") == "yes" ||
									  (layer && *layer < 0);
			for (const auto &[cx, cz] : build_smoothed_centerline(way)) {
				if (building_footprints.contains(cx, cz)) {
					if (!track)
						track.emplace(xzbbox);
					track->set(cx, cz);
				}
				for (int dx = -reach; dx <= reach; ++dx)
					for (int dz = -reach; dz <= reach; ++dz) {
						const int x = cx + dx, z = cz + dz;
						if (!near_building(x, z))
							continue;
						if (!cells)
							cells.emplace(xzbbox);
						cells->set(x, z);
						if (passes_under) {
							if (!under)
								under.emplace(xzbbox);
							under->set(x, z);
						}
					}
			}
		}
	}
	RailPassages result;
	result.cells = cells ? std::move(*cells) : CoordinateBitmap::new_empty();
	result.under = under ? std::move(*under) : CoordinateBitmap::new_empty();
	result.track = track ? std::move(*track) : CoordinateBitmap::new_empty();
	result.height = std::max(static_cast<int>(std::ceil(TRAIN_HEADROOM_M * scale)), 4);
	result.runs_in = static_cast<std::size_t>(std::max(RUNS_IN_M * scale, 3.0));
	return result;
}

} // namespace arnis::railways
