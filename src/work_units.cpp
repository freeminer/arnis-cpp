#include "work_units.h"

#include "projection/projection_spec.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <numbers>
#include <numeric>
#include <sstream>
#include <string_view>
#include <tuple>
#include <utility>

namespace arnis::work_units
{
namespace
{
std::int64_t div_euclid(std::int64_t value, std::int64_t divisor)
{
	const auto quotient = value / divisor;
	const auto remainder = value % divisor;
	return remainder < 0 ? quotient - 1 : quotient;
}

std::uint64_t region_span(std::int32_t low, std::int32_t high)
{
	return static_cast<std::uint64_t>(
			div_euclid(high, REGION_BLOCKS) - div_euclid(low, REGION_BLOCKS) + 1);
}

struct Segment
{
	std::int32_t cell;
	std::int32_t low;
	std::int32_t high;
};

std::vector<Segment> segments(
		std::int32_t low, std::int32_t high, std::int32_t side, bool merge)
{
	const auto first = div_euclid(low, side);
	const auto last = div_euclid(high, side);
	std::vector<Segment> result;
	result.reserve(static_cast<std::size_t>(last - first + 1));
	for (auto cell = first; cell <= last; ++cell) {
		const auto cell_low = cell * side;
		const auto cell_high = cell_low + side - 1;
		result.push_back({static_cast<std::int32_t>(cell),
				static_cast<std::int32_t>(std::max<std::int64_t>(cell_low, low)),
				static_cast<std::int32_t>(std::min<std::int64_t>(cell_high, high))});
	}
	const auto thin = [](const Segment &s) {
		return std::int64_t(s.high) - s.low + 1 < SLIVER_BLOCKS;
	};
	if (merge && result.size() > 1 && thin(result.front())) {
		const auto first_segment = result.front();
		result.erase(result.begin());
		result.front().low = first_segment.low;
	}
	if (merge && result.size() > 1 && thin(result.back())) {
		const auto last_segment = result.back();
		result.pop_back();
		result.back().high = last_segment.high;
	}
	return result;
}

std::optional<WorkUnit> make_unit(const projection::WebMercatorProjection &projection,
		const Segment &x, const Segment &z, std::int32_t size)
{
	try {
		cartesian::XZBBoxRect rect{{x.low, z.low}, {x.high, z.high}};
		return WorkUnit{0, x.cell, z.cell, size, rect,
				projection::llbbox_for_rect(projection, rect),
				region_span(x.low, x.high) * region_span(z.low, z.high)};
	} catch (const std::exception &) {
		return std::nullopt;
	}
}

void order_centre_out(const cartesian::XZBBoxRect &world, std::vector<WorkUnit> &units)
{
	const std::int64_t cx = std::int64_t(world.min().x) + world.max().x;
	const std::int64_t cz = std::int64_t(world.min().z) + world.max().z;
	std::sort(units.begin(), units.end(), [&](const WorkUnit &a, const WorkUnit &b) {
		const std::int64_t ax = std::int64_t(a.rect.min().x) + a.rect.max().x - cx;
		const std::int64_t az = std::int64_t(a.rect.min().z) + a.rect.max().z - cz;
		const std::int64_t bx = std::int64_t(b.rect.min().x) + b.rect.max().x - cx;
		const std::int64_t bz = std::int64_t(b.rect.min().z) + b.rect.max().z - cz;
		return std::tuple{ax * ax + az * az, a.gz, a.gx} <
			   std::tuple{bx * bx + bz * bz, b.gz, b.gx};
	});
	for (std::size_t i = 0; i < units.size(); ++i)
		units[i].index = static_cast<std::uint32_t>(i);
}
} // namespace

WorkUnit::WorkUnit(std::uint32_t unit_index, std::int32_t cell_x, std::int32_t cell_z,
		std::int32_t region_size, cartesian::XZBBoxRect unit_rect,
		geographic::LLBBox geographic_bbox, std::uint64_t region_count) :
		index(unit_index), gx(cell_x), gz(cell_z), size(region_size),
		rect(std::move(unit_rect)), bbox(std::move(geographic_bbox)),
		regions(region_count)
{
}

std::uint64_t WorkUnit::chunks() const
{
	const auto width = std::int64_t(rect.max().x) - rect.min().x + 1;
	const auto height = std::int64_t(rect.max().z) - rect.min().z + 1;
	return width > 0 && height > 0 ? static_cast<std::uint64_t>(width / 16) *
											 static_cast<std::uint64_t>(height / 16)
								   : 0;
}

std::string WorkUnit::region_key() const
{
	return std::to_string(div_euclid(rect.min().x, REGION_BLOCKS)) + "," +
		   std::to_string(div_euclid(rect.min().z, REGION_BLOCKS)) + "," +
		   std::to_string(size);
}

bool plan_units(const projection::WebMercatorProjection &projection,
		const cartesian::XZBBoxRect &world, std::int32_t regions_per_side,
		std::vector<WorkUnit> &units, std::string *error, bool merge_slivers)
{
	units.clear();
	if (regions_per_side < 1 ||
			regions_per_side > std::numeric_limits<std::int32_t>::max() / REGION_BLOCKS) {
		if (error)
			*error = "a work unit is at least one region";
		return false;
	}
	const auto side = regions_per_side * REGION_BLOCKS;
	const auto xs = segments(world.min().x, world.max().x, side, merge_slivers);
	const auto zs = segments(world.min().z, world.max().z, side, merge_slivers);
	if (!xs.empty() && zs.size() > std::numeric_limits<std::size_t>::max() / xs.size()) {
		if (error)
			*error = "work-unit plan is too large";
		return false;
	}
	units.reserve(xs.size() * zs.size());
	for (const auto &z : zs) {
		for (const auto &x : xs) {
			auto unit = make_unit(projection, x, z, regions_per_side);
			if (!unit) {
				units.clear();
				if (error)
					*error = "could not project work-unit bounds";
				return false;
			}
			units.push_back(std::move(*unit));
		}
	}
	order_centre_out(world, units);
	if (error)
		error->clear();
	return true;
}

bool build_order(const std::vector<WorkUnit> &units, const PieceOrder &order,
		std::vector<std::size_t> &indices, std::string *error)
{
	indices.resize(units.size());
	std::iota(indices.begin(), indices.end(), 0);
	const auto &rect = [&](std::size_t i) -> const cartesian::XZBBoxRect & {
		return units[i].rect;
	};
	switch (order.mode) {
	case PieceOrderMode::Centre:
		break;
	case PieceOrderMode::West:
		std::stable_sort(indices.begin(), indices.end(), [&](auto a, auto b) {
			return std::tuple{rect(a).min().x, a} < std::tuple{rect(b).min().x, b};
		});
		break;
	case PieceOrderMode::East:
		std::stable_sort(indices.begin(), indices.end(), [&](auto a, auto b) {
			return std::tuple{-std::int64_t(rect(a).max().x), a} <
				   std::tuple{-std::int64_t(rect(b).max().x), b};
		});
		break;
	case PieceOrderMode::North:
		std::stable_sort(indices.begin(), indices.end(), [&](auto a, auto b) {
			return std::tuple{rect(a).min().z, a} < std::tuple{rect(b).min().z, b};
		});
		break;
	case PieceOrderMode::South:
		std::stable_sort(indices.begin(), indices.end(), [&](auto a, auto b) {
			return std::tuple{-std::int64_t(rect(a).max().z), a} <
				   std::tuple{-std::int64_t(rect(b).max().z), b};
		});
		break;
	case PieceOrderMode::List: {
		std::vector<std::size_t> listed;
		std::vector<bool> seen(units.size(), false);
		for (const auto number : order.list) {
			if (number == 0 || number > units.size()) {
				if (error)
					*error = "piece-order index must be from 1 to " +
							 std::to_string(units.size());
				return false;
			}
			const auto index = static_cast<std::size_t>(number - 1);
			if (!seen[index]) {
				seen[index] = true;
				listed.push_back(index);
			}
		}
		for (const auto index : indices)
			if (!seen[index])
				listed.push_back(index);
		indices = std::move(listed);
		break;
	}
	}
	if (error)
		error->clear();
	return true;
}

std::optional<Plan> plan_selection(const projection::WebMercatorProjection &projection,
		const geographic::LLBBox &selection, const PlannerOptions &options,
		std::string *error)
{
	const auto fail = [&](std::string message) -> std::optional<Plan> {
		if (error)
			*error = std::move(message);
		return std::nullopt;
	};
	cartesian::XZBBoxRect requested({0, 0}, {0, 0});
	try {
		requested = projection::snap_bbox_to_chunks(projection, selection).first;
	} catch (const std::exception &e) {
		return fail(std::string("could not snap selected bounds to chunks: ") + e.what());
	}
	const auto max_regions = options.fixed_regions.value_or(options.auto_max_regions);
	if (max_regions < 1 ||
			max_regions > std::numeric_limits<std::int32_t>::max() / REGION_BLOCKS ||
			options.margin_regions < 0 || options.max_pieces == 0)
		return fail("invalid work-unit size or margin");
	const std::int64_t side = std::int64_t(REGION_BLOCKS) * max_regions;
	const auto axis = [&](std::int32_t low, std::int32_t high) {
		std::int64_t first = low, last = high;
		if (options.snap_regions == SnapRegions::Cover) {
			first = div_euclid(low, side) * side;
			last = (div_euclid(high, side) + 1) * side - 1;
		} else if (options.snap_regions == SnapRegions::Fit) {
			auto a = div_euclid(std::int64_t(low) + side - 1, side);
			auto b = div_euclid(std::int64_t(high) + 1, side);
			if (b <= a) {
				const auto center = div_euclid(std::int64_t(low) + high, 2);
				a = div_euclid(center, side);
				b = a + 1;
			}
			first = a * side;
			last = b * side - 1;
		}
		return std::pair{first, last};
	};
	const auto [x0, x1] = axis(requested.min().x, requested.max().x);
	const auto [z0, z1] = axis(requested.min().z, requested.max().z);
	const std::int64_t margin = std::int64_t(options.margin_regions) * REGION_BLOCKS;
	const std::int64_t expanded[]{x0 - margin, z0 - margin, x1 + margin, z1 + margin};
	for (const auto coordinate : expanded)
		if (coordinate < std::numeric_limits<std::int32_t>::min() ||
				coordinate > std::numeric_limits<std::int32_t>::max())
			return fail("work-unit rectangle exceeds supported block coordinates");
	const cartesian::XZBBoxRect world({static_cast<std::int32_t>(expanded[0]),
											  static_cast<std::int32_t>(expanded[1])},
			{static_cast<std::int32_t>(expanded[2]),
					static_cast<std::int32_t>(expanded[3])});
	const bool merge = options.snap_regions == SnapRegions::Merge;
	std::optional<Mask> mask;
	if (options.mask_path) {
		mask = Mask::load(*options.mask_path, projection, world, error);
		if (!mask)
			return std::nullopt;
	}
	const auto coarse_side = std::int64_t(REGION_BLOCKS) * max_regions;
	const auto cells_x = div_euclid(world.max().x, coarse_side) -
						 div_euclid(world.min().x, coarse_side) + 1;
	const auto cells_z = div_euclid(world.max().z, coarse_side) -
						 div_euclid(world.min().z, coarse_side) + 1;
	if (cells_x > 0 && cells_z > 0 &&
			static_cast<std::uint64_t>(cells_x) * cells_z > options.max_pieces)
		return fail(
				"selection creates more coarse work cells than the configured plan limit");
	std::vector<WorkUnit> units;
	if (options.fixed_regions) {
		if (!plan_units(projection, world, *options.fixed_regions, units, error, merge))
			return std::nullopt;
	} else {
		std::vector<std::int32_t> leaves{max_regions};
		while (leaves.back() > 2 && leaves.back() % 2 == 0)
			leaves.push_back(leaves.back() / 2);
		std::reverse(leaves.begin(), leaves.end());
		for (const auto min_regions : leaves) {
			if (!mask && min_regions != max_regions)
				continue;
			if (!plan_auto_units(projection, world, max_regions, min_regions,
						mask ? &*mask : nullptr, units, error))
				return std::nullopt;
			if (units.size() <= options.max_pieces)
				break;
		}
	}
	if (mask) {
		std::erase_if(
				units, [&](const WorkUnit &unit) { return !mask->touches(unit.rect); });
		if (units.empty())
			return fail("--plan-mask: the polygon holds no piece of this job.");
		order_centre_out(world, units);
	}
	const auto size_name = options.fixed_regions ? std::to_string(*options.fixed_regions)
												 : std::string("auto");
	std::string name = std::to_string(world.min().x) + "_" +
					   std::to_string(world.min().z) + "_" +
					   std::to_string(world.max().x) + "_" +
					   std::to_string(world.max().z) + "_n" + size_name;
	if (!options.fixed_regions)
		name += std::to_string(max_regions);
	if (merge)
		name += "_merge";
	if (mask) {
		static constexpr char hex[] = "0123456789abcdef";
		std::string hash(16, '0');
		auto value = mask->hash();
		for (int i = 15; i >= 0; --i) {
			hash[static_cast<std::size_t>(i)] = hex[value & 0xf];
			value >>= 4;
		}
		name += "_m" + hash;
	}
	if (error)
		error->clear();
	return Plan{requested, world, std::move(units), std::move(name)};
}

std::optional<Mask> Mask::load(const std::filesystem::path &path,
		const projection::WebMercatorProjection &projection,
		const cartesian::XZBBoxRect &rect, std::string *error)
{
	std::ifstream input(path, std::ios::binary);
	if (!input) {
		if (error)
			*error = "--plan-mask " + path.string() + ": could not read file";
		return std::nullopt;
	}
	std::ostringstream content;
	content << input.rdbuf();
	const auto text = content.str();
	std::optional<LLShapes::Polygons> polygons = LLShapes::from_geojson(text);
	if (!polygons) {
		LLShapes::Polygon poly;
		LLShapes::Ring ring;
		std::istringstream lines(text);
		std::string line;
		while (std::getline(lines, line)) {
			const auto first = line.find_first_not_of(" \t\r\n\f\v");
			const auto last = line.find_last_not_of(" \t\r\n\f\v");
			const std::string_view trimmed =
					first == std::string::npos
							? std::string_view{}
							: std::string_view(line).substr(first, last - first + 1);
			std::istringstream fields(line);
			double lon = 0, lat = 0;
			if (fields >> lon >> lat) {
				ring.emplace_back(lat, lon);
			} else if (trimmed == "END" && ring.size() > 2) {
				poly.push_back(std::move(ring));
				ring.clear();
			} else {
				ring.clear();
			}
		}
		if (poly.empty()) {
			if (error)
				*error = "--plan-mask " + path.string() +
						 ": no polygon found (GeoJSON or .poly).";
			return std::nullopt;
		}
		polygons = LLShapes::Polygons{std::move(poly)};
	}
	auto shapes = LLShapes::create(*polygons, error);
	if (!shapes)
		return std::nullopt;
	XZBBoxShape::Polygons projected;
	projected.reserve(shapes->polygons().size());
	for (const auto &polygon : shapes->polygons()) {
		XZBBoxShape::Polygon projected_polygon;
		projected_polygon.reserve(polygon.size());
		for (const auto &ring : polygon) {
			XZBBoxShape::Ring projected_ring;
			projected_ring.reserve(ring.size());
			for (const auto &point : ring)
				projected_ring.emplace_back(projection.x_for_lon(point.lng()),
						projection.z_for_lat(point.lat()));
			projected_polygon.push_back(std::move(projected_ring));
		}
		projected.push_back(std::move(projected_polygon));
	}
	auto shape = XZBBoxShape::create(rect, std::move(projected), error);
	if (!shape) {
		if (error && error->empty())
			*error = "--plan-mask: the polygon holds no piece of this job";
		return std::nullopt;
	}
	std::uint64_t hash = 14695981039346656037ULL;
	for (const unsigned char byte : text) {
		hash ^= byte;
		hash *= 1099511628211ULL;
	}
	if (error)
		error->clear();
	return Mask{std::make_shared<const XZBBoxShape>(std::move(*shape)), hash};
}

bool Mask::touches(const cartesian::XZBBoxRect &rect) const
{
	return shape_ &&
		   shape_->touches(rect.min().x, rect.min().z, rect.max().x, rect.max().z);
}

bool Mask::covers(const cartesian::XZBBoxRect &rect) const
{
	return shape_ &&
		   shape_->covers(rect.min().x, rect.min().z, rect.max().x, rect.max().z);
}

bool plan_auto_units(const projection::WebMercatorProjection &projection,
		const cartesian::XZBBoxRect &world, std::int32_t max_regions,
		std::int32_t min_regions, const Mask *mask, std::vector<WorkUnit> &units,
		std::string *error)
{
	units.clear();
	if (min_regions < 1 || max_regions < min_regions ||
			max_regions > std::numeric_limits<std::int32_t>::max() / REGION_BLOCKS) {
		if (error)
			*error = "invalid auto work-unit region sizes";
		return false;
	}
	for (auto size = max_regions; size > min_regions;) {
		if (size <= 2 || size % 2 != 0 || size / 2 < min_regions) {
			if (error)
				*error = "auto work-unit sizes must follow successive halvings";
			return false;
		}
		size /= 2;
	}
	std::string failure;
	const auto split = [&](auto &&self, std::int32_t gx, std::int32_t gz,
							   std::int32_t size) -> bool {
		const std::int64_t side = std::int64_t(REGION_BLOCKS) * size;
		const std::int64_t x0 = std::int64_t(gx) * side;
		const std::int64_t z0 = std::int64_t(gz) * side;
		const std::int64_t x1 = x0 + side - 1;
		const std::int64_t z1 = z0 + side - 1;
		const auto clipped_x0 = std::max<std::int64_t>(x0, world.min().x);
		const auto clipped_z0 = std::max<std::int64_t>(z0, world.min().z);
		const auto clipped_x1 = std::min<std::int64_t>(x1, world.max().x);
		const auto clipped_z1 = std::min<std::int64_t>(z1, world.max().z);
		if (clipped_x0 > clipped_x1 || clipped_z0 > clipped_z1)
			return true;
		Segment sx{gx, static_cast<std::int32_t>(clipped_x0),
				static_cast<std::int32_t>(clipped_x1)};
		Segment sz{gz, static_cast<std::int32_t>(clipped_z0),
				static_cast<std::int32_t>(clipped_z1)};
		auto piece = make_unit(projection, sx, sz, size);
		if (!piece) {
			failure = "could not project auto work-unit bounds";
			return false;
		}
		if (mask && !mask->touches(piece->rect))
			return true;
		const bool whole_cell = clipped_x0 == x0 && clipped_z0 == z0 &&
								clipped_x1 == x1 && clipped_z1 == z1;
		if ((whole_cell && (!mask || mask->covers(piece->rect))) || size <= min_regions) {
			units.push_back(std::move(*piece));
			return true;
		}
		const auto child_size = size / 2;
		if (child_size < 1) {
			failure = "auto work-unit subdivision reached an invalid size";
			return false;
		}
		for (const auto [dx, dz] :
				{std::pair{0, 0}, std::pair{1, 0}, std::pair{0, 1}, std::pair{1, 1}}) {
			if (!self(self, gx * 2 + dx, gz * 2 + dz, child_size))
				return false;
		}
		return true;
	};
	const auto side = std::int64_t(REGION_BLOCKS) * max_regions;
	for (auto gz = div_euclid(world.min().z, side); gz <= div_euclid(world.max().z, side);
			++gz) {
		for (auto gx = div_euclid(world.min().x, side);
				gx <= div_euclid(world.max().x, side); ++gx) {
			if (!split(split, static_cast<std::int32_t>(gx),
						static_cast<std::int32_t>(gz), max_regions)) {
				units.clear();
				if (error)
					*error = failure;
				return false;
			}
		}
	}
	order_centre_out(world, units);
	if (error)
		error->clear();
	return true;
}

std::vector<geographic::LLBBox> room(const projection::WebMercatorProjection &projection,
		const cartesian::XZBBoxRect &rect, std::int32_t regions_per_side,
		double max_stretch, std::size_t cap)
{
	std::vector<geographic::LLBBox> result;
	if (!std::isfinite(max_stretch))
		return result;
	regions_per_side = std::max<std::int32_t>(1, regions_per_side);
	const std::int64_t side = std::int64_t(REGION_BLOCKS) * regions_per_side;
	const auto gx0 = div_euclid(rect.min().x, side);
	const auto gx1 = div_euclid(rect.max().x, side);
	const auto gz0 = div_euclid(rect.min().z, side);
	const auto gz1 = div_euclid(rect.max().z, side);
	const auto width = gx1 - gx0 + 1;
	const auto height = gz1 - gz0 + 1;
	const auto count = [&](std::int64_t ring) {
		return (width + 2 * ring) * (height + 2 * ring) - width * height;
	};
	auto ring = std::max(width, height);
	while (ring > 0 && (count(ring) < 0 || static_cast<std::uint64_t>(count(ring)) > cap))
		--ring;
	const auto origin_cos =
			std::cos(projection.origin_latitude_degrees() * std::numbers::pi / 180.0);
	const auto stretch = [&](double lat) {
		const auto denominator = std::cos(lat * std::numbers::pi / 180.0);
		return denominator == 0.0 ? std::numeric_limits<double>::infinity()
								  : std::abs(origin_cos / denominator - 1.0);
	};
	for (auto gz = gz0 - ring; gz <= gz1 + ring; ++gz) {
		for (auto gx = gx0 - ring; gx <= gx1 + ring; ++gx) {
			if (gx >= gx0 && gx <= gx1 && gz >= gz0 && gz <= gz1)
				continue;
			const auto x0 = gx * side, z0 = gz * side;
			const auto x1 = x0 + side - 1, z1 = z0 + side - 1;
			if (x0 < std::numeric_limits<std::int32_t>::min() ||
					x1 > std::numeric_limits<std::int32_t>::max() ||
					z0 < std::numeric_limits<std::int32_t>::min() ||
					z1 > std::numeric_limits<std::int32_t>::max())
				continue;
			try {
				const cartesian::XZBBoxRect cell(
						{static_cast<std::int32_t>(x0), static_cast<std::int32_t>(z0)},
						{static_cast<std::int32_t>(x1), static_cast<std::int32_t>(z1)});
				auto bbox = projection::llbbox_for_rect(projection, cell);
				if (std::max(stretch(bbox.min().lat()), stretch(bbox.max().lat())) <=
						max_stretch)
					result.push_back(std::move(bbox));
			} catch (const std::exception &) {
				// Cells beyond the projection's geographic domain cannot be extended into.
			}
		}
	}
	return result;
}

} // namespace arnis::work_units
