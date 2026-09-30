#pragma once
#include "confidence.h"
#include "project.h"
#include "imgops.h"
#include "visibility.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <map>
#include <tuple>
#include <optional>
#include <vector>
namespace arnis::mapillary
{
// Rust's exported facade grids encode the semantic class in PNG alpha while
// RGB carries the sampled wall colour.  Keep the mapping in the shared C++
// contract so importers and block/display consumers cannot silently disagree.
enum class FacadeClass : std::uint8_t
{
	Wall,
	Window,
	Door,
	Unknown,
	NoData,
};

inline FacadeClass facade_class_from_alpha(std::uint8_t alpha)
{
	switch (alpha) {
	case 255:
		return FacadeClass::Wall;
	case 192:
		return FacadeClass::Window;
	case 128:
		return FacadeClass::Door;
	case 64:
		return FacadeClass::Unknown;
	default:
		return FacadeClass::NoData;
	}
}

struct FacadeCell
{
	imgops::Rgb color{};
	FacadeClass type = FacadeClass::NoData;
};

inline std::optional<std::pair<imgops::Rgb, double>> dominant_color(
		const std::vector<imgops::Rgb> &cells)
{
	constexpr std::size_t L_BINS = 10, AB_BINS = 12;
	constexpr double AB_RANGE = .2, MIN_CELLS = 12, MIN_SHARE = .15;
	if (cells.size() < MIN_CELLS)
		return std::nullopt;
	std::map<std::tuple<std::size_t, std::size_t, std::size_t>, std::vector<imgops::Rgb>>
			bins;
	for (const auto &cell : cells) {
		const auto lab = imgops::srgb_to_oklab(cell);
		const auto quantize = [AB_RANGE](double value) {
			const double normalized =
					(std::clamp(value, -AB_RANGE, AB_RANGE) + AB_RANGE) /
					(2.0 * AB_RANGE);
			return std::min(AB_BINS - 1,
					static_cast<std::size_t>(normalized * static_cast<double>(AB_BINS)));
		};
		const auto key = std::make_tuple(
				std::min(L_BINS - 1,
						static_cast<std::size_t>(std::clamp(lab[0], 0.0, 1.0) * L_BINS)),
				quantize(lab[1]), quantize(lab[2]));
		bins[key].push_back(cell);
	}
	if (bins.empty())
		return std::nullopt;
	const auto best =
			std::max_element(bins.begin(), bins.end(), [](const auto &a, const auto &b) {
				if (a.second.size() != b.second.size())
					return a.second.size() < b.second.size();
				return a.first > b.first;
			});
	const double share = static_cast<double>(best->second.size()) / cells.size();
	if (share < MIN_SHARE)
		return std::nullopt;
	std::array<unsigned, 3> sum{};
	for (const auto &rgb : best->second)
		for (std::size_t i = 0; i < 3; ++i)
			sum[i] += rgb[i];
	imgops::Rgb result{};
	for (std::size_t i = 0; i < 3; ++i)
		result[i] = static_cast<std::uint8_t>(sum[i] / best->second.size());
	return std::make_pair(result, share);
}

struct WallSegment
{
	double ax{}, az{}, bx{}, bz{};
	double nx{}, nz{}, length{};
};

struct ScoredFacadeView
{
	std::size_t index{};
	projection::CameraPose pose{};
	double score{};
	double distance_m{};
};

inline std::vector<ScoredFacadeView> select_facade_views(const WallSegment &,
		const std::vector<projection::CameraPose> &, double, std::size_t limit = 4);

struct WallSamplePoint
{
	double x{}, y{}, z{};
	std::size_t row{}, column{};
};

struct FacadeCameraImage
{
	projection::CameraPose pose;
	const projection::Image *image = nullptr;
};

struct WallGrid
{
	std::size_t columns{}, rows{};
	std::vector<std::optional<imgops::Rgb>> cells;
};

struct WallSampleResult
{
	std::vector<imgops::Rgb> accepted;
	std::optional<WallGrid> grid;
	std::size_t no_view = 0;
	std::array<std::size_t, 4> rejected{};
};

inline std::vector<WallSamplePoint> wall_sample_points(
		const WallSegment &wall, const std::array<double, 2> &band, double scale)
{
	const double inset = wall.length * .12;
	const double usable = std::max(0.0, wall.length - 2.0 * inset);
	const auto clamp_count = [](double value, std::size_t maximum) {
		return std::clamp<std::size_t>(
				static_cast<std::size_t>(std::llround(value)), 1, maximum);
	};
	const std::size_t columns = clamp_count(usable, 64);
	const std::size_t rows = clamp_count(band[1] - band[0], 32);
	const double tx = (wall.bx - wall.ax) / wall.length;
	const double tz = (wall.bz - wall.az) / wall.length;
	const double epsilon = .05 * scale;
	std::vector<WallSamplePoint> result;
	result.reserve(columns * rows);
	for (std::size_t row = 0; row < rows; ++row) {
		const double t = (static_cast<double>(row) + .5) / rows;
		const double y = band[1] - t * (band[1] - band[0]);
		for (std::size_t column = 0; column < columns; ++column) {
			const double along =
					inset + (static_cast<double>(column) + .5) * usable / columns;
			result.push_back({wall.ax + tx * along + wall.nx * epsilon, y,
					wall.az + tz * along + wall.nz * epsilon, row, column});
		}
	}
	return result;
}

inline WallSampleResult sample_wall(const WallSegment &wall,
		const std::vector<FacadeCameraImage> &cameras, const std::array<double, 2> &band,
		double scale, bool keep_grid = true)
{
	std::vector<projection::CameraPose> poses;
	poses.reserve(cameras.size());
	for (const auto &camera : cameras)
		poses.push_back(camera.pose);
	const auto selected = select_facade_views(wall, poses, scale);
	const auto points = wall_sample_points(wall, band, scale);
	WallSampleResult result;
	if (keep_grid)
		result.grid = WallGrid{};
	if (result.grid) {
		const double inset = wall.length * .12;
		result.grid->columns = std::clamp<std::size_t>(
				static_cast<std::size_t>(
						std::llround(std::max(0.0, wall.length - 2 * inset))),
				1, 64);
		result.grid->rows = std::clamp<std::size_t>(
				static_cast<std::size_t>(std::llround(band[1] - band[0])), 1, 32);
		result.grid->cells.reserve(points.size());
	}
	for (const auto &point : points) {
		std::vector<imgops::Rgb> samples;
		for (const auto &view : selected) {
			if (view.index >= cameras.size() || !cameras[view.index].image) {
				++result.no_view;
				continue;
			}
			const auto uv = projection::project(view.pose, point.x, point.y, point.z);
			if (const auto raw = projection::sample_pixel(
						*cameras[view.index].image, uv[0], uv[1])) {
				projection::Reject reason{};
				if (const auto colour = projection::classify(*raw, &reason))
					samples.push_back(*colour);
				else
					++result.rejected[static_cast<std::size_t>(reason)];
			} else {
				++result.no_view;
			}
		}
		const auto cell = projection::median_color(samples);
		if (cell)
			result.accepted.push_back(*cell);
		if (result.grid)
			result.grid->cells.push_back(cell);
	}
	return result;
}

inline std::vector<ScoredFacadeView> select_facade_views(const WallSegment &wall,
		const std::vector<projection::CameraPose> &cameras, double scale,
		std::size_t limit)
{
	const double min_distance = 4.0 * scale;
	const double max_distance = 45.0 * scale;
	const double min_cos = std::cos(60.0 * 3.14159265358979323846 / 180.0);
	const double mx = (wall.ax + wall.bx) * .5, mz = (wall.az + wall.bz) * .5;
	std::vector<ScoredFacadeView> scored;
	for (std::size_t i = 0; i < cameras.size(); ++i) {
		const auto &camera = cameras[i];
		const double dx = camera.centre[0] - mx, dz = camera.centre[2] - mz;
		const double distance = std::hypot(dx, dz);
		if (distance < min_distance || distance > max_distance)
			continue;
		const double incidence = (dx * wall.nx + dz * wall.nz) / distance;
		if (incidence < min_cos)
			continue;
		scored.push_back(
				{i, camera, incidence / (1.0 + distance / (20.0 * scale)), distance});
	}
	std::stable_sort(scored.begin(), scored.end(),
			[](const auto &a, const auto &b) { return a.score > b.score; });
	if (scored.size() > limit)
		scored.resize(limit);
	return scored;
}

// Rust facade geometry: rebuild a closed footprint with one consistent
// outward normal per edge.  `scale` is blocks per metre.
inline std::vector<WallSegment> wall_segments(
		const std::vector<std::array<double, 2>> &points, double scale)
{
	if (points.size() < 3)
		return {};
	const std::size_t n =
			points.front() == points.back() ? points.size() - 1 : points.size();
	if (n < 3)
		return {};
	double area = 0.0;
	for (std::size_t i = 0; i < n; ++i) {
		const auto &a = points[i];
		const auto &b = points[(i + 1) % n];
		area += a[0] * b[1] - b[0] * a[1];
	}
	const bool clockwise = area > 0.0;
	const double min_length = 3.0 * scale;
	std::vector<WallSegment> result;
	result.reserve(n);
	for (std::size_t i = 0; i < n; ++i) {
		const auto &a = points[i];
		const auto &b = points[(i + 1) % n];
		const double dx = b[0] - a[0], dz = b[1] - a[1];
		const double length = std::hypot(dx, dz);
		if (length < min_length)
			continue;
		const double tx = dx / length, tz = dz / length;
		result.push_back({a[0], a[1], b[0], b[1], clockwise ? tz : -tz,
				clockwise ? -tx : tx, length});
	}
	return result;
}

inline double building_height_m(const std::map<std::string, std::string> &tags)
{
	if (const auto it = tags.find("height"); it != tags.end()) {
		std::string value = it->second;
		while (!value.empty() &&
				(std::isalpha(static_cast<unsigned char>(value.back())) ||
						std::isspace(static_cast<unsigned char>(value.back()))))
			value.pop_back();
		try {
			const double height = std::stod(value);
			if (height > 1.0 && height < 400.0)
				return height;
		} catch (...) {
		}
	}
	if (const auto it = tags.find("building:levels"); it != tags.end()) {
		try {
			const double levels = std::stod(it->second);
			if (levels >= 1.0 && levels < 150.0)
				return levels * 3.0;
		} catch (...) {
		}
	}
	return 9.0;
}

inline std::optional<std::array<double, 2>> facade_sample_band(
		double height_m, double scale)
{
	const double top = std::min(height_m - 1.0, 12.0);
	const double bottom = std::min(2.5, top - 1.0);
	if (top - bottom < 1.5)
		return std::nullopt;
	return std::array<double, 2>{bottom * scale, top * scale};
}

struct FacadeView
{
	std::size_t index{};
	ViewQuality quality{};
};

inline std::optional<FacadeView> select_facade_view(
		const std::vector<FacadeView> &views, double minimum = .10)
{
	std::optional<FacadeView> best;
	for (const auto &view : views)
		if (confidence(view.quality) >= minimum &&
				(!best || confidence(view.quality) > confidence(best->quality)))
			best = view;
	return best;
}

inline ViewQuality wall_view_quality(const Camera &camera, const Wall &wall,
		const Coverage &coverage, double distance_m, double sharpness, PoseSource pose)
{
	auto n = wall_outward(wall);
	double facing = 0;
	if (n) {
		double dx = camera.centre[0] - wall.a[0];
		double dy = camera.centre[1] - wall.a[1];
		double length = std::hypot(dx, dy);
		if (length > 1e-9)
			facing = std::max(0., (dx * (*n)[0] + dy * (*n)[1]) / length);
	}
	return {coverage.fraction(), facing, distance_m, sharpness, pose};
}
}
