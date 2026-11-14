#include "area_layout.h"

#include "../clipping.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arnis::element_processing::area_layout
{
namespace
{
std::pair<int32_t, int32_t> snap(const std::pair<double, double> &axis)
{
	if (std::abs(axis.first) >= std::abs(axis.second))
		return {axis.first >= 0.0 ? 1 : -1, 0};
	return {0, axis.second >= 0.0 ? 1 : -1};
}
} // namespace

std::optional<Frame> Frame::of(const std::vector<ProcessedNode> &nodes)
{
	if (nodes.empty())
		return std::nullopt;
	double c = 0.0, s = 0.0;
	for (std::size_t i = 1; i < nodes.size(); ++i) {
		if (clipping::is_invented_node_id(nodes[i - 1].id) &&
				clipping::is_invented_node_id(nodes[i].id))
			continue;
		const double dx = double(nodes[i].x) - nodes[i - 1].x;
		const double dz = double(nodes[i].z) - nodes[i - 1].z;
		const double segment_length = std::hypot(dx, dz);
		if (segment_length >= 1.0) {
			const double angle = 4.0 * std::atan2(dz, dx);
			c += segment_length * std::cos(angle);
			s += segment_length * std::sin(angle);
		}
	}
	const double angle = std::atan2(s, c) / 4.0;
	const std::pair<double, double> u{std::cos(angle), std::sin(angle)};
	const std::pair<double, double> v{-u.second, u.first};
	const auto extent = [&](const std::pair<double, double> &axis) {
		std::pair<double, double> range{std::numeric_limits<double>::max(),
				std::numeric_limits<double>::lowest()};
		for (const auto &node : nodes) {
			const double p = axis.first * node.x + axis.second * node.z;
			range.first = std::min(range.first, p);
			range.second = std::max(range.second, p);
		}
		return range;
	};
	const auto eu = extent(u), ev = extent(v);
	if (eu.first > eu.second)
		return std::nullopt;
	if (eu.second - eu.first >= ev.second - ev.first)
		return Frame{u, v, eu, ev};
	return Frame{v, u, ev, eu};
}

double Frame::length() const
{
	return along_range.second - along_range.first;
}

double Frame::width() const
{
	return across_range.second - across_range.first;
}

std::pair<double, double> Frame::local(int32_t x, int32_t z) const
{
	return {along.first * x + along.second * z - along_range.first,
			across.first * x + across.second * z - across_range.first};
}

std::pair<int32_t, int32_t> Frame::column(double a, double c) const
{
	a += along_range.first;
	c += across_range.first;
	return {static_cast<int32_t>(std::lround(along.first * a + across.first * c)),
			static_cast<int32_t>(std::lround(along.second * a + across.second * c))};
}

std::array<std::pair<double, double>, 2> Frame::local_span(
		const std::array<int32_t, 4> &box) const
{
	std::array<std::pair<double, double>, 2> span{
			{{std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()},
					{std::numeric_limits<double>::max(),
							std::numeric_limits<double>::lowest()}}};
	for (const auto &[x, z] :
			std::array<std::pair<int32_t, int32_t>, 4>{{{box[0], box[1]},
					{box[2], box[1]}, {box[0], box[3]}, {box[2], box[3]}}}) {
		const auto point = local(x, z);
		span[0].first = std::min(span[0].first, point.first);
		span[0].second = std::max(span[0].second, point.first);
		span[1].first = std::min(span[1].first, point.second);
		span[1].second = std::max(span[1].second, point.second);
	}
	return span;
}

std::pair<int32_t, int32_t> Frame::along_axis() const
{
	return snap(along);
}

std::pair<int32_t, int32_t> Frame::across_axis() const
{
	return snap(across);
}

AreaMask::AreaMask(const std::vector<std::pair<int32_t, int32_t>> &cells,
		const std::function<bool(int32_t, int32_t)> &keep)
{
	if (cells.empty())
		return;
	min_x_ = min_z_ = std::numeric_limits<int32_t>::max();
	int32_t max_x = std::numeric_limits<int32_t>::lowest();
	int32_t max_z = std::numeric_limits<int32_t>::lowest();
	for (const auto &[x, z] : cells) {
		min_x_ = std::min(min_x_, x);
		min_z_ = std::min(min_z_, z);
		max_x = std::max(max_x, x);
		max_z = std::max(max_z, z);
	}
	width_ = static_cast<std::size_t>(std::int64_t(max_x) - min_x_ + 1);
	height_ = static_cast<std::size_t>(std::int64_t(max_z) - min_z_ + 1);
	if (height_ == 0 || width_ > std::numeric_limits<std::size_t>::max() / height_) {
		width_ = height_ = 0;
		return;
	}
	const std::size_t area = width_ * height_;
	bits_.assign(area / 64 + (area % 64 != 0), 0);
	for (const auto &[x, z] : cells) {
		if (!keep(x, z))
			continue;
		const auto dx = static_cast<std::size_t>(std::int64_t(x) - min_x_);
		const auto dz = static_cast<std::size_t>(std::int64_t(z) - min_z_);
		const std::size_t bit = dz * width_ + dx;
		bits_[bit >> 6] |= std::uint64_t{1} << (bit & 63);
	}
}

bool AreaMask::contains(int32_t x, int32_t z) const
{
	const auto dx = std::int64_t(x) - min_x_;
	const auto dz = std::int64_t(z) - min_z_;
	if (dx < 0 || dz < 0 || static_cast<std::uint64_t>(dx) >= width_ ||
			static_cast<std::uint64_t>(dz) >= height_)
		return false;
	const std::size_t bit =
			static_cast<std::size_t>(dz) * width_ + static_cast<std::size_t>(dx);
	return (bits_[bit >> 6] & (std::uint64_t{1} << (bit & 63))) != 0;
}

std::optional<Bounds> bounds(const std::vector<ProcessedNode> &nodes)
{
	if (nodes.empty())
		return std::nullopt;
	Bounds result{double(nodes.front().x), double(nodes.front().z),
			double(nodes.front().x), double(nodes.front().z)};
	for (const auto &node : nodes) {
		result[0] = std::min(result[0], double(node.x));
		result[1] = std::min(result[1], double(node.z));
		result[2] = std::max(result[2], double(node.x));
		result[3] = std::max(result[3], double(node.z));
	}
	return result;
}

bool inside(const Bounds &box, const std::pair<double, double> &point)
{
	return point.first >= box[0] && point.first <= box[2] && point.second >= box[1] &&
		   point.second <= box[3];
}

std::pair<double, double> centroid(const std::vector<ProcessedNode> &nodes)
{
	if (nodes.empty())
		return {std::numeric_limits<double>::quiet_NaN(),
				std::numeric_limits<double>::quiet_NaN()};
	double x = 0.0, z = 0.0;
	for (const auto &node : nodes) {
		x += node.x;
		z += node.z;
	}
	return {x / nodes.size(), z / nodes.size()};
}

bool point_in_rings(const std::vector<const std::vector<ProcessedNode> *> &rings,
		const std::pair<double, double> &point)
{
	bool inside_point = false;
	for (const auto *ring : rings) {
		if (!ring)
			continue;
		for (std::size_t i = 1; i < ring->size(); ++i) {
			const auto &a = (*ring)[i - 1];
			const auto &b = (*ring)[i];
			if ((a.z > point.second) != (b.z > point.second) &&
					point.first < a.x + (point.second - a.z) * (b.x - a.x) /
												  (double(b.z) - a.z))
				inside_point = !inside_point;
		}
	}
	return inside_point;
}

std::optional<std::pair<double, double>> interior_point(
		const std::vector<ProcessedNode> &nodes)
{
	const auto box = bounds(nodes);
	if (!box)
		return std::nullopt;
	const double z = (box->at(1) + box->at(3)) / 2.0 +
					 0.37 * std::min(box->at(3) - box->at(1), 1.0);
	std::vector<double> xs;
	for (std::size_t i = 1; i < nodes.size(); ++i) {
		const auto &a = nodes[i - 1];
		const auto &b = nodes[i];
		if ((a.z > z) != (b.z > z))
			xs.push_back(a.x + (z - a.z) * (double(b.x) - a.x) / (double(b.z) - a.z));
	}
	std::sort(xs.begin(), xs.end());
	if (xs.size() < 2)
		return std::nullopt;
	std::optional<std::pair<double, double>> widest;
	for (std::size_t i = 0; i + 1 < xs.size(); i += 2) {
		if (!widest || xs[i + 1] - xs[i] > widest->second - widest->first)
			widest = {xs[i], xs[i + 1]};
	}
	if (!widest)
		return std::nullopt;
	return std::pair<double, double>{(widest->first + widest->second) / 2.0, z};
}

std::uint64_t mix(std::uint64_t h)
{
	h ^= h >> 30;
	h *= 0xBF58476D1CE4E5B9ULL;
	h ^= h >> 27;
	h *= 0x94D049BB133111EBULL;
	return h ^ (h >> 31);
}

std::uint64_t cell_hash(std::uint64_t id, std::int64_t i, std::int64_t j)
{
	return mix(id ^ mix(static_cast<std::uint64_t>(i) ^
							mix(static_cast<std::uint64_t>(j) ^ 0x51EDULL)));
}

} // namespace arnis::element_processing::area_layout
