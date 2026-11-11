#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>
#include "coordinate_system/cartesian/xzpoint.h"

namespace arnis
{
namespace cartesian
{
struct XZVector;
class XZBBoxRect;
}

// Rasterized polygon outline used by the Rust XZBBoxShape equivalent. Each
// polygon contains an outer ring followed by zero or more holes.
class XZBBoxShape
{
public:
	using Point = std::pair<double, double>;
	using Ring = std::vector<Point>;
	using Polygon = std::vector<Ring>;
	using Polygons = std::vector<Polygon>;
	using Span = std::pair<std::int32_t, std::int32_t>;

	static std::optional<XZBBoxShape> create(const cartesian::XZBBoxRect &rect,
			Polygons polygons, std::string *error = nullptr);
	bool contains(std::int32_t x, std::int32_t z) const;
	bool covers(std::int32_t min_x, std::int32_t min_z, std::int32_t max_x,
			std::int32_t max_z) const;
	bool touches(std::int32_t min_x, std::int32_t min_z, std::int32_t max_x,
			std::int32_t max_z) const;
	XZBBoxShape cropped(const cartesian::XZBBoxRect &rect) const;
	std::uint64_t covered_columns() const;
	std::optional<cartesian::XZPoint> first_inside() const;
	std::optional<cartesian::XZPoint> nearest_inside(cartesian::XZPoint point) const;
	XZBBoxShape translated(
			std::int32_t dx, std::int32_t dz, const cartesian::XZBBoxRect &rect) const;
	const cartesian::XZBBoxRect &rect() const { return *rect_; }
	const Polygons &polygons() const { return mask_->polygons; }
	std::span<const Span> row(std::int32_t z) const;

private:
	struct Mask
	{
		Polygons polygons;
		std::int32_t min_z{};
		std::vector<std::uint32_t> offsets;
		std::vector<Span> spans;
	};
	std::shared_ptr<const cartesian::XZBBoxRect> rect_;
	std::shared_ptr<const Mask> mask_;
};
}
