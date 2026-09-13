#pragma once

#include "pose.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace arnis::mapillary::sfm
{
inline constexpr double index_cell_m = 5.0;
inline constexpr const char *z_base_foot = "foot";
inline constexpr const char *z_base_pano = "pano";

struct Shot
{
	std::string id;
	std::array<double, 3> centre{};
	pose::Mat3 axes{};
	double capture_time{};
	std::string sequence;
	double compass{};
	std::array<double, 3> rotation{};
	std::string camera;
};

// A deterministic uniform plan-index.  Rust deliberately uses a grid rather
// than a kd-tree, because its neighbourhood radius is modest and stable.
class Cluster
{
	std::map<std::pair<std::int32_t, std::int32_t>, std::vector<std::size_t>> buckets_;
	static std::pair<std::int32_t, std::int32_t> cell(double x, double y)
	{
		return {std::int32_t(std::floor(x / index_cell_m)),
				std::int32_t(std::floor(y / index_cell_m))};
	}
	void rebuild_index()
	{
		buckets_.clear();
		for (std::size_t i = 0; i < points.size(); ++i)
			buckets_[cell(points[i][0], points[i][1])].push_back(i);
	}

public:
	std::string id;
	std::vector<std::array<double, 3>> points;
	std::vector<std::array<std::uint8_t, 3>> colors;
	std::vector<Shot> shots;

	Cluster() = default;
	explicit Cluster(std::vector<std::array<double, 3>> values) :
			points(std::move(values))
	{
		rebuild_index();
	}
	void set_points(std::vector<std::array<double, 3>> values)
	{
		points = std::move(values);
		rebuild_index();
	}
	const Shot *shot(const std::string &shot_id) const
	{
		auto it = std::find_if(shots.begin(), shots.end(),
				[&](const Shot &shot) { return shot.id == shot_id; });
		return it == shots.end() ? nullptr : &*it;
	}
	std::vector<std::size_t> near_indices(
			const std::array<double, 2> &xy, double radius) const
	{
		std::vector<std::size_t> result;
		if (!(radius > 0) || !std::isfinite(xy[0]) || !std::isfinite(xy[1]))
			return result;
		auto [x0, y0] = cell(xy[0] - radius, xy[1] - radius);
		auto [x1, y1] = cell(xy[0] + radius, xy[1] + radius);
		double r2 = radius * radius;
		for (std::int32_t x = x0; x <= x1; ++x)
			for (std::int32_t y = y0; y <= y1; ++y) {
				auto it = buckets_.find({x, y});
				if (it == buckets_.end())
					continue;
				for (auto i : it->second) {
					double dx = points[i][0] - xy[0], dy = points[i][1] - xy[1];
					if (dx * dx + dy * dy <= r2)
						result.push_back(i);
				}
			}
		std::sort(result.begin(), result.end());
		return result;
	}
	std::vector<std::array<double, 3>> near(
			const std::array<double, 2> &xy, double radius) const
	{
		auto indices = near_indices(xy, radius);
		std::vector<std::array<double, 3>> result;
		result.reserve(indices.size());
		for (auto i : indices)
			result.push_back(points[i]);
		return result;
	}
};

inline void shift_points(std::vector<std::array<double, 3>> &points,
		const std::array<double, 3> &shift,
		const std::array<double, 2> &registered_centre)
{
	double dx = shift[0], dy = shift[1], radians = shift[2] * .017453292519943295769;
	if (std::abs(radians) <= 1e-12) {
		for (auto &point : points) {
			point[0] += dx;
			point[1] += dy;
		}
		return;
	}
	double s = std::sin(radians), c = std::cos(radians);
	double raw_x = registered_centre[0] - dx, raw_y = registered_centre[1] - dy;
	for (auto &point : points) {
		double x = point[0] - raw_x, y = point[1] - raw_y;
		point[0] = c * x - s * y + raw_x + dx;
		point[1] = s * x + c * y + raw_y + dy;
	}
}

struct DepthMap
{
	unsigned width{}, height{};
	std::vector<float> depth;
	DepthMap(unsigned w, unsigned h) :
			width(w), height(h),
			depth(std::size_t(w) * h, std::numeric_limits<float>::infinity())
	{
	}
	float at(unsigned column, unsigned row) const
	{
		return depth[std::size_t(row) * width + column];
	}
	std::optional<float> sample(double u, double v) const
	{
		if (!width || !height || !std::isfinite(u) || !std::isfinite(v))
			return {};
		long column = long(u * width) % long(width);
		if (column < 0)
			column += width;
		long row = std::clamp(long(v * height), 0L, long(height) - 1);
		return at(unsigned(column), unsigned(row));
	}
};

inline float round_to_f16(float value)
{
	if (!std::isfinite(value) || value == 0)
		return value;
	float sign = value < 0 ? -1 : 1;
	double absolute = std::abs(double(value));
	if (absolute >= 65520)
		return sign * std::numeric_limits<float>::infinity();
	double exponent = std::max(-14.0, std::floor(std::log2(absolute)));
	double unit = std::exp2(exponent - 10), scaled = absolute / unit;
	double floor = std::floor(scaled), fraction = scaled - floor;
	double rounded = fraction > .5 || (fraction == .5 && std::int64_t(floor) % 2)
							 ? floor + 1
							 : floor;
	return float(sign * rounded * unit);
}

inline std::vector<float> minimum_filter_3x3(const std::vector<float> &source,
		unsigned width, unsigned height, bool wrap_columns)
{
	std::vector<float> rows(source.size(), std::numeric_limits<float>::infinity()),
			result(rows);
	if (!width || !height)
		return result;
	for (unsigned y = 0; y < height; ++y) {
		unsigned up = y ? y - 1 : 0, down = std::min(y + 1, height - 1);
		for (unsigned x = 0; x < width; ++x)
			rows[std::size_t(y) * width + x] =
					std::min({source[std::size_t(up) * width + x],
							source[std::size_t(y) * width + x],
							source[std::size_t(down) * width + x]});
	}
	for (unsigned y = 0; y < height; ++y)
		for (unsigned x = 0; x < width; ++x) {
			unsigned left = x ? x - 1 : (wrap_columns ? width - 1 : 0);
			unsigned right = x + 1 < width ? x + 1 : (wrap_columns ? 0 : width - 1);
			result[std::size_t(y) * width + x] =
					std::min({rows[std::size_t(y) * width + left],
							rows[std::size_t(y) * width + x],
							rows[std::size_t(y) * width + right]});
		}
	return result;
}

inline DepthMap depth_map(const Cluster &cluster, const Camera &camera, unsigned width,
		unsigned height, double radius_m, const std::array<double, 3> &shift = {})
{
	DepthMap result(width, height);
	if (!width || !height)
		return result;
	auto points = cluster.near(
			{camera.centre[0] - shift[0], camera.centre[1] - shift[1]}, radius_m);
	shift_points(points, shift, {camera.centre[0], camera.centre[1]});
	pose::Projector projector(camera);
	std::vector<std::pair<std::size_t, float>> hits;
	for (const auto &point : points) {
		auto projection = projector.project(point);
		if (!projection.valid() || (!projector.spherical() && !projection.inside_image()))
			continue;
		long column = long(projection.u * width) % long(width);
		if (column < 0)
			column += width;
		long row = std::clamp(long(projection.v * height), 0L, long(height) - 1);
		hits.emplace_back(
				std::size_t(row) * width + unsigned(column), float(projection.length_m));
	}
	std::sort(hits.begin(), hits.end(),
			[](const auto &a, const auto &b) { return a.second > b.second; });
	for (const auto &[cell, depth] : hits)
		result.depth[cell] = depth;
	result.depth = minimum_filter_3x3(result.depth, width, height, projector.spherical());
	for (auto &depth : result.depth)
		depth = round_to_f16(depth);
	return result;
}
} // namespace arnis::mapillary::sfm
