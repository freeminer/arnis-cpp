#pragma once

#include "../coordinate_system/cartesian/xzbbox/xzbbox_enum.h"
#include "../water_probe.h"
#include "trains.h"

#include <memory>
#include <optional>
#include <vector>

namespace arnis::element_processing::ships
{
/// Mapped ferry crossings waiting for the finished water surface to be sampled.
class ShipPlan
{
public:
	static ShipPlan build(const std::vector<ProcessedElement> &elements, double scale,
			const cartesian::XZBBox &world);
	bool is_empty() const { return lines_.empty(); }
	std::shared_ptr<water_probe::WaterProbe> probe() const;
	std::vector<std::pair<trains::Route, trains::Service>> finish(
			const water_probe::WaterProbe &probe) const;

private:
	struct Line
	{
		std::uint64_t way{};
		bool cars{};
		std::optional<std::uint8_t> waterway_class;
		std::optional<std::uint32_t> minutes;
		trains::End ends[2]{trains::End::Buffer, trains::End::Buffer};
		std::vector<trains::RoutePoint> points;
	};
	std::vector<Line> lines_;
	std::vector<trains::RoutePoint> piers_;
	double scale_{1.0};
	double size_{1.0};
};
} // namespace arnis::element_processing::ships
