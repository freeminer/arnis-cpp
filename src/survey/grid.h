#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace arnis::survey::grid
{

struct Shape
{
	double area = 0.0;
	double length = 0.0;
	double width = 0.0;
};

inline std::pair<std::vector<std::uint32_t>, std::uint32_t> label(
		const std::vector<std::uint8_t> &mask, std::size_t width, std::size_t height,
		bool diagonal)
{
	std::vector<std::uint32_t> labels(width * height, 0);
	if (!width || !height || labels.size() != mask.size())
		return {std::move(labels), 0};
	std::uint32_t count = 0;
	std::vector<std::size_t> stack;
	for (std::size_t start = 0; start < mask.size(); ++start) {
		if (!mask[start] || labels[start])
			continue;
		++count;
		labels[start] = count;
		stack.push_back(start);
		while (!stack.empty()) {
			const auto index = stack.back();
			stack.pop_back();
			const auto x = static_cast<std::int64_t>(index % width);
			const auto z = static_cast<std::int64_t>(index / width);
			for (int dz = -1; dz <= 1; ++dz)
				for (int dx = -1; dx <= 1; ++dx) {
					if ((!dx && !dz) || (!diagonal && dx && dz))
						continue;
					const auto nx = x + dx, nz = z + dz;
					if (nx < 0 || nz < 0 || nx >= static_cast<std::int64_t>(width) ||
							nz >= static_cast<std::int64_t>(height))
						continue;
					const auto next = static_cast<std::size_t>(nz) * width +
									  static_cast<std::size_t>(nx);
					if (mask[next] && !labels[next]) {
						labels[next] = count;
						stack.push_back(next);
					}
				}
		}
	}
	return {std::move(labels), count};
}

inline std::vector<Shape> shapes(
		const std::vector<std::uint32_t> &labels, std::uint32_t count, std::size_t width)
{
	std::vector<std::array<double, 6>> sums(static_cast<std::size_t>(count) + 1);
	if (!width)
		return std::vector<Shape>(sums.size());
	for (std::size_t i = 0; i < labels.size(); ++i) {
		const auto label = labels[i];
		if (!label || label > count)
			continue;
		const double x = static_cast<double>(i % width);
		const double z = static_cast<double>(i / width);
		auto &sum = sums[label];
		sum[0] += 1;
		sum[1] += x;
		sum[2] += z;
		sum[3] += x * x;
		sum[4] += z * z;
		sum[5] += x * z;
	}
	std::vector<Shape> out;
	out.reserve(sums.size());
	for (const auto &sum : sums) {
		const double n = sum[0];
		if (n == 0) {
			out.emplace_back();
			continue;
		}
		const double mx = sum[1] / n, mz = sum[2] / n;
		const double sxx = sum[3] / n - mx * mx;
		const double szz = sum[4] / n - mz * mz;
		const double sxz = sum[5] / n - mx * mz;
		const double trace = sxx + szz;
		const double determinant = sxx * szz - sxz * sxz;
		const double lambda =
				trace / 2 + std::sqrt(std::max(0.0, trace * trace / 4 - determinant));
		const double length = std::sqrt(12 * std::max(0.0, lambda)) + 1;
		out.push_back({n, length, n / std::max(1.0, length)});
	}
	return out;
}

inline std::vector<std::uint8_t> dilate(const std::vector<std::uint8_t> &mask,
		std::size_t width, std::size_t height, std::size_t radius)
{
	if (!width || height > std::numeric_limits<std::size_t>::max() / width ||
			mask.size() != width * height)
		return {};
	std::vector<std::uint8_t> rows(mask.size(), 0), out(mask.size(), 0);
	for (std::size_t z = 0; z < height; ++z) {
		std::size_t last = 0;
		bool have_last = false;
		for (std::size_t x = 0; x < width; ++x) {
			if (mask[z * width + x]) {
				last = x;
				have_last = true;
			}
			rows[z * width + x] = have_last && x - last <= radius;
		}
		std::size_t next = 0;
		bool have_next = false;
		for (std::size_t x = width; x-- > 0;) {
			if (mask[z * width + x]) {
				next = x;
				have_next = true;
			}
			rows[z * width + x] |= have_next && next - x <= radius;
		}
	}
	for (std::size_t z = 0; z < height; ++z) {
		const auto z0 = z > radius ? z - radius : 0;
		const auto z1 = radius >= height - z ? height : z + radius + 1;
		for (std::size_t near = z0; near < z1; ++near)
			for (std::size_t x = 0; x < width; ++x)
				out[z * width + x] |= rows[near * width + x];
	}
	return out;
}

template <typename F>
void box_share(const std::vector<std::uint8_t> &mask, std::size_t width,
		std::size_t height, std::size_t radius, F &&visit)
{
	if (!width || height > std::numeric_limits<std::size_t>::max() / width ||
			mask.size() != width * height)
		return;
	std::vector<std::uint32_t> columns(width, 0), prefix(width + 1, 0);
	std::size_t low = 0, high = 0;
	for (std::size_t z = 0; z < height; ++z) {
		const auto z0 = z > radius ? z - radius : 0;
		const auto z1 = radius >= height - z ? height : z + radius + 1;
		while (high < z1) {
			const auto row = high++ * width;
			for (std::size_t x = 0; x < width; ++x)
				columns[x] += mask[row + x] != 0;
		}
		while (low < z0) {
			const auto row = low++ * width;
			for (std::size_t x = 0; x < width; ++x)
				columns[x] -= mask[row + x] != 0;
		}
		prefix[0] = 0;
		for (std::size_t x = 0; x < width; ++x)
			prefix[x + 1] = prefix[x] + columns[x];
		for (std::size_t x = 0; x < width; ++x) {
			const auto x0 = x > radius ? x - radius : 0;
			const auto x1 = radius >= width - x ? width : x + radius + 1;
			const double area = double((z1 - z0) * (x1 - x0));
			visit(z * width + x, static_cast<float>((prefix[x1] - prefix[x0]) / area));
		}
	}
}

inline std::vector<float> median3_within(const std::vector<float> &values,
		const std::vector<std::uint8_t> &keep, std::size_t width, std::size_t height)
{
	if (!width || height > std::numeric_limits<std::size_t>::max() / width ||
			values.size() != width * height || keep.size() != values.size())
		return {};
	std::vector<float> out;
	std::array<float, 9> buffer{};
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t x = 0; x < width; ++x) {
			if (!keep[z * width + x])
				continue;
			std::size_t count = 0;
			for (int dz = -1; dz <= 1; ++dz)
				for (int dx = -1; dx <= 1; ++dx) {
					const auto nx = static_cast<std::int64_t>(x) + dx;
					const auto nz = static_cast<std::int64_t>(z) + dz;
					if (nx < 0 || nz < 0 || nx >= static_cast<std::int64_t>(width) ||
							nz >= static_cast<std::int64_t>(height))
						continue;
					const auto index = static_cast<std::size_t>(nz) * width +
									   static_cast<std::size_t>(nx);
					if (keep[index])
						buffer[count++] = values[index];
				}
			std::sort(buffer.begin(), buffer.begin() + count);
			out.push_back(buffer[count / 2]);
		}
	return out;
}

inline std::vector<float> component_median(const std::vector<float> &values,
		const std::vector<std::uint32_t> &labels, std::uint32_t count)
{
	std::vector<std::vector<float>> groups(static_cast<std::size_t>(count) + 1);
	for (std::size_t i = 0; i < std::min(values.size(), labels.size()); ++i)
		if (labels[i] && labels[i] <= count)
			groups[labels[i]].push_back(values[i]);
	std::vector<float> out;
	out.reserve(groups.size());
	for (auto &group : groups) {
		if (group.empty()) {
			out.push_back(0.0f);
			continue;
		}
		std::sort(group.begin(), group.end());
		out.push_back(group[group.size() / 2]);
	}
	return out;
}

inline std::vector<std::pair<int, int>> hull(
		const std::vector<std::pair<int, int>> &cells)
{
	auto points = cells;
	std::sort(points.begin(), points.end());
	points.erase(std::unique(points.begin(), points.end()), points.end());
	if (points.size() < 3)
		return points;
	const auto cross = [](const auto &o, const auto &a, const auto &b) {
		return (std::int64_t(a.first) - o.first) * (std::int64_t(b.second) - o.second) -
			   (std::int64_t(a.second) - o.second) * (std::int64_t(b.first) - o.first);
	};
	std::vector<std::pair<int, int>> lower, upper;
	for (const auto &p : points) {
		while (lower.size() >= 2 && cross(lower[lower.size() - 2], lower.back(), p) <= 0)
			lower.pop_back();
		lower.push_back(p);
	}
	for (auto it = points.rbegin(); it != points.rend(); ++it) {
		while (upper.size() >= 2 &&
				cross(upper[upper.size() - 2], upper.back(), *it) <= 0)
			upper.pop_back();
		upper.push_back(*it);
	}
	lower.pop_back();
	upper.pop_back();
	lower.insert(lower.end(), upper.begin(), upper.end());
	return lower;
}

} // namespace arnis::survey::grid
