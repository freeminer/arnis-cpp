#include "fm_xz_bbox_shape.h"

#include "coordinate_system/cartesian/xzbbox/rectangle.h"
#include "coordinate_system/cartesian/xzpoint.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arnis
{
namespace
{
double snap(double value)
{
	const double nearest = std::round(value);
	return std::abs(value - nearest) < 1e-7 ? nearest : value;
}

void merge(std::vector<XZBBoxShape::Span> &row)
{
	std::sort(row.begin(), row.end());
	std::size_t write = 0;
	for (const auto &[lo, hi] : row) {
		if (write && static_cast<std::int64_t>(lo) <=
							 static_cast<std::int64_t>(row[write - 1].second) + 1) {
			row[write - 1].second = std::max(row[write - 1].second, hi);
		} else {
			row[write++] = {lo, hi};
		}
	}
	row.resize(write);
}

struct Edge
{
	double z0{}, z1{}, x0{}, x1{};
	double x_at(double z) const
	{
		if (z == z0)
			return x0;
		if (z == z1)
			return x1;
		return x0 + (z - z0) * (x1 - x0) / (z1 - z0);
	}
};

void sweep(const XZBBoxShape::Polygon &rings, const cartesian::XZBBoxRect &rect,
		std::vector<std::vector<XZBBoxShape::Span>> &cells,
		std::vector<std::uint32_t> &merged)
{
	const auto min_z = static_cast<std::int64_t>(rect.min().z);
	const auto max_z = static_cast<std::int64_t>(rect.max().z);
	const double min_x = rect.min().x, max_x = rect.max().x;
	std::vector<Edge> edges;
	std::vector<double> heights;
	auto add = [&](double row, double lo, double hi) {
		if (!std::isfinite(row) || row < min_z || row > max_z || !std::isfinite(lo) ||
				!std::isfinite(hi))
			return;
		const auto row_index = static_cast<std::int64_t>(row);
		// Match Rust's intersect-then-rasterize behavior. Clamping both endpoints
		// would collapse an entirely out-of-bounds span onto a border column.
		const double a = std::max(std::floor(std::min(lo, hi)), min_x);
		const double b = std::min(std::floor(std::max(lo, hi)), max_x);
		if (a > b)
			return;
		const auto index = static_cast<std::size_t>(row_index - min_z);
		auto &spans = cells[index];
		spans.emplace_back(static_cast<std::int32_t>(a), static_cast<std::int32_t>(b));
		if (spans.size() >= 64 && spans.size() >= 2 * merged[index]) {
			merge(spans);
			merged[index] = static_cast<std::uint32_t>(spans.size());
		}
	};
	for (const auto &ring : rings) {
		if (ring.size() < 2)
			continue;
		for (std::size_t i = 0; i < ring.size(); ++i) {
			const auto [xa, za] = ring[i];
			const auto [xb, zb] = ring[(i + 1) % ring.size()];
			heights.push_back(za);
			if (za == zb)
				add(std::floor(za), xa, xb);
			else if (za < zb)
				edges.push_back({za, zb, xa, xb});
			else
				edges.push_back({zb, za, xb, xa});
		}
	}
	if (edges.empty())
		return;
	const auto [lo, hi] = std::minmax_element(heights.begin(), heights.end());
	// Compare in floating point before converting. A finite outline may be far
	// outside the rectangle; converting its height directly to int64_t is
	// undefined in C++, while Rust's float-to-int cast saturates.
	const double first_height = std::max(std::ceil(*lo), static_cast<double>(min_z));
	const double last_height = std::min(std::floor(*hi), static_cast<double>(max_z + 1));
	if (first_height <= last_height) {
		const auto first = static_cast<std::int64_t>(first_height);
		const auto last = static_cast<std::int64_t>(last_height);
		for (auto z = first; z <= last; ++z)
			heights.push_back(static_cast<double>(z));
	}
	std::sort(heights.begin(), heights.end());
	heights.erase(std::unique(heights.begin(), heights.end()), heights.end());
	std::sort(edges.begin(), edges.end(),
			[](const Edge &a, const Edge &b) { return a.z0 < b.z0; });
	std::size_t next = 0;
	std::vector<Edge> active;
	std::vector<std::pair<double, Edge>> keyed;
	for (std::size_t i = 1; i < heights.size(); ++i) {
		const double a = heights[i - 1], b = heights[i];
		while (next < edges.size() && edges[next].z0 <= a)
			active.push_back(edges[next++]);
		active.erase(std::remove_if(active.begin(), active.end(),
							 [&](const Edge &edge) { return edge.z1 <= a; }),
				active.end());
		const double row = std::floor(a);
		if (row > max_z || b < min_z)
			continue;
		const bool top_row = std::floor(b) == b;
		const double middle = (a + b) * .5;
		keyed.clear();
		for (const auto &edge : active)
			keyed.emplace_back(edge.x_at(middle), edge);
		std::sort(keyed.begin(), keyed.end(),
				[](const auto &l, const auto &r) { return l.first < r.first; });
		active.clear();
		for (const auto &entry : keyed)
			active.push_back(entry.second);
		for (std::size_t edge = 1; edge < keyed.size(); edge += 2) {
			const auto &left = keyed[edge - 1].second;
			const auto &right = keyed[edge].second;
			const double la = left.x_at(a), lb = left.x_at(b);
			const double ra = right.x_at(a), rb = right.x_at(b);
			const double lo_x = std::min({la, lb, ra, rb});
			const double hi_x = std::max(
					{la, ra, top_row ? lb - 1e-9 : lb, top_row ? rb - 1e-9 : rb});
			add(row, lo_x, hi_x);
			if (top_row)
				add(b, left.x_at(b), right.x_at(b));
		}
	}
}
}

std::optional<XZBBoxShape> XZBBoxShape::create(
		const cartesian::XZBBoxRect &rect, Polygons polygons, std::string *error)
{
	auto fail = [&](const char *message) -> std::optional<XZBBoxShape> {
		if (error)
			*error = message;
		return std::nullopt;
	};
	if (rect.total_blocks_z() > 10'000'000)
		return fail("outline bounds contain too many rows");
	for (auto &polygon : polygons)
		for (auto &ring : polygon)
			for (auto &point : ring) {
				if (!std::isfinite(point.first) || !std::isfinite(point.second))
					return fail("The drawn area has a broken point.");
				point.first = snap(point.first);
				point.second = snap(point.second);
			}

	XZBBoxShape shape;
	shape.rect_ = std::make_shared<const cartesian::XZBBoxRect>(rect);
	auto mask = std::make_shared<Mask>();
	mask->polygons = std::move(polygons);
	mask->min_z = rect.min().z;
	const std::size_t rows = rect.total_blocks_z();
	std::vector<std::vector<Span>> row_spans(rows);
	std::vector<std::uint32_t> merged(rows);
	for (const auto &polygon : mask->polygons)
		sweep(polygon, rect, row_spans, merged);
	mask->offsets.reserve(rows + 1);
	mask->offsets.push_back(0);
	for (auto &row : row_spans) {
		merge(row);
		mask->spans.insert(mask->spans.end(), row.begin(), row.end());
		if (mask->spans.size() > std::numeric_limits<std::uint32_t>::max())
			return fail("outline contains too many raster spans");
		mask->offsets.push_back(static_cast<std::uint32_t>(mask->spans.size()));
	}
	if (mask->spans.empty())
		return fail("The drawn area holds no block.");
	shape.mask_ = std::move(mask);
	return shape;
}

std::span<const XZBBoxShape::Span> XZBBoxShape::row(std::int32_t z) const
{
	const auto index = static_cast<std::int64_t>(z) - mask_->min_z;
	if (index < 0 || static_cast<std::size_t>(index + 1) >= mask_->offsets.size())
		return {};
	const auto begin =
			mask_->spans.begin() + mask_->offsets[static_cast<std::size_t>(index)];
	const auto end =
			mask_->spans.begin() + mask_->offsets[static_cast<std::size_t>(index + 1)];
	return {begin, end};
}

bool XZBBoxShape::contains(std::int32_t x, std::int32_t z) const
{
	if (x < rect_->min().x || x > rect_->max().x || z < rect_->min().z ||
			z > rect_->max().z)
		return false;
	const auto spans = row(z);
	const auto it = std::lower_bound(spans.begin(), spans.end(), x,
			[](const Span &span, std::int32_t value) { return span.second < value; });
	return it != spans.end() && it->first <= x;
}

bool XZBBoxShape::covers(std::int32_t min_x, std::int32_t min_z, std::int32_t max_x,
		std::int32_t max_z) const
{
	if (min_x < rect_->min().x || min_z < rect_->min().z || max_x > rect_->max().x ||
			max_z > rect_->max().z)
		return false;
	for (std::int32_t z = min_z; z <= max_z; ++z) {
		const auto spans = row(z);
		const auto it = std::lower_bound(spans.begin(), spans.end(), min_x,
				[](const Span &span, std::int32_t value) { return span.second < value; });
		if (it == spans.end() || it->first > min_x || it->second < max_x)
			return false;
	}
	return true;
}

bool XZBBoxShape::touches(std::int32_t min_x, std::int32_t min_z, std::int32_t max_x,
		std::int32_t max_z) const
{
	min_x = std::max(min_x, rect_->min().x);
	max_x = std::min(max_x, rect_->max().x);
	min_z = std::max(min_z, rect_->min().z);
	max_z = std::min(max_z, rect_->max().z);
	if (min_x > max_x || min_z > max_z)
		return false;
	for (std::int32_t z = min_z; z <= max_z; ++z) {
		const auto spans = row(z);
		const auto it = std::lower_bound(spans.begin(), spans.end(), min_x,
				[](const Span &span, std::int32_t value) { return span.second < value; });
		if (it != spans.end() && it->first <= max_x)
			return true;
	}
	return false;
}

XZBBoxShape XZBBoxShape::cropped(const cartesian::XZBBoxRect &rect) const
{
	XZBBoxShape result = *this;
	result.rect_ = std::make_shared<const cartesian::XZBBoxRect>(rect);
	return result;
}

std::uint64_t XZBBoxShape::covered_columns() const
{
	std::uint64_t total = 0;
	for (std::int64_t z = rect_->min().z; z <= rect_->max().z; ++z)
		for (const auto &[lo, hi] : row(static_cast<std::int32_t>(z))) {
			const auto a = std::max(lo, rect_->min().x);
			const auto b = std::min(hi, rect_->max().x);
			if (a <= b)
				total += static_cast<std::uint64_t>(static_cast<std::int64_t>(b) - a + 1);
		}
	return total;
}

std::optional<cartesian::XZPoint> XZBBoxShape::first_inside() const
{
	for (int margin : {4, 1, 0}) {
		const int size = 2 * margin;
		for (std::int64_t z = rect_->min().z; z <= rect_->max().z; ++z) {
			for (const auto &[a, b] : row(static_cast<std::int32_t>(z))) {
				const auto lo = std::max(a, rect_->min().x);
				const auto hi = std::min(b, rect_->max().x);
				int attempts = 0;
				for (std::int64_t x = lo; x + size <= hi && attempts < 256;
						x += std::max(1, margin), ++attempts)
					if (covers(static_cast<std::int32_t>(x), static_cast<std::int32_t>(z),
								x + size, z + size))
						return cartesian::XZPoint{static_cast<int>(x + margin),
								static_cast<int>(z + margin)};
			}
		}
	}
	return std::nullopt;
}

std::optional<cartesian::XZPoint> XZBBoxShape::nearest_inside(cartesian::XZPoint p) const
{
	if (contains(p.x, p.z))
		return p;
	std::optional<std::pair<std::int64_t, cartesian::XZPoint>> best;
	for (std::int64_t z = rect_->min().z; z <= rect_->max().z; ++z) {
		const auto dz = z - p.z;
		if (best && dz * dz >= best->first && z > p.z)
			break;
		for (const auto &[a, b] : row(static_cast<std::int32_t>(z))) {
			const auto lo = std::max(a, rect_->min().x);
			const auto hi = std::min(b, rect_->max().x);
			if (lo > hi)
				continue;
			const auto x = std::clamp<std::int64_t>(p.x, lo, hi);
			const auto dx = x - p.x;
			const auto distance = dx * dx + dz * dz;
			if (!best || distance < best->first)
				best = std::make_pair(distance,
						cartesian::XZPoint{static_cast<int>(x), static_cast<int>(z)});
		}
	}
	return best ? std::optional<cartesian::XZPoint>(best->second) : std::nullopt;
}

XZBBoxShape XZBBoxShape::translated(
		std::int32_t dx, std::int32_t dz, const cartesian::XZBBoxRect &rect) const
{
	XZBBoxShape result = *this;
	result.rect_ = std::make_shared<const cartesian::XZBBoxRect>(rect);
	auto mask = std::make_shared<Mask>(*mask_);
	mask->min_z += dz;
	for (auto &polygon : mask->polygons)
		for (auto &ring : polygon)
			for (auto &point : ring) {
				point.first += dx;
				point.second += dz;
			}
	for (auto &span : mask->spans) {
		span.first += dx;
		span.second += dx;
	}
	result.mask_ = std::move(mask);
	return result;
}
}
