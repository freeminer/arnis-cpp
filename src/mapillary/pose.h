#pragma once

#include "types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>

namespace arnis::mapillary::pose
{
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<Vec3, 3>;
inline constexpr double radial_limit_max = 6.0;
inline constexpr double radial_limit_safety = .9;

inline double dot(const Vec3 &a, const Vec3 &b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline double norm(const Vec3 &a)
{
	return std::sqrt(dot(a, a));
}
inline Vec3 scale(const Vec3 &a, double k)
{
	return {a[0] * k, a[1] * k, a[2] * k};
}
inline Vec3 add(const Vec3 &a, const Vec3 &b)
{
	return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
inline Vec3 sub(const Vec3 &a, const Vec3 &b)
{
	return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
inline Vec3 cross(const Vec3 &a, const Vec3 &b)
{
	return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
			a[0] * b[1] - a[1] * b[0]};
}

inline Mat3 axes_from_rotation(const Vec3 &rotation)
{
	double theta = norm(rotation);
	if (theta < 1e-12)
		return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
	Vec3 k = scale(rotation, 1.0 / theta);
	double s = std::sin(theta), c = std::cos(theta);
	double skew[3][3] = {{0, -k[2], k[1]}, {k[2], 0, -k[0]}, {-k[1], k[0], 0}};
	Mat3 result{};
	for (unsigned i = 0; i < 3; ++i)
		for (unsigned j = 0; j < 3; ++j)
			result[i][j] =
					(i == j ? 1.0 : 0.0) * c + s * skew[i][j] + (1.0 - c) * k[i] * k[j];
	return result;
}
inline Mat3 level_axes(double compass_deg, double roll_deg = 0, double pitch_deg = 0)
{
	double h = compass_deg * .017453292519943295769;
	double sh = std::sin(h), ch = std::cos(h);
	Vec3 forward{sh, ch, 0}, right{ch, -sh, 0}, down{0, 0, -1};
	double p = pitch_deg * .017453292519943295769;
	Vec3 f2 = sub(scale(forward, std::cos(p)), scale(down, std::sin(p)));
	Vec3 d2 = add(scale(down, std::cos(p)), scale(forward, std::sin(p)));
	double r = roll_deg * .017453292519943295769;
	return {add(scale(right, std::cos(r)), scale(d2, std::sin(r))),
			sub(scale(d2, std::cos(r)), scale(right, std::sin(r))), f2};
}
inline double heading_of(const Mat3 &axes)
{
	double heading = std::atan2(axes[2][0], axes[2][1]) * 57.295779513082320876;
	heading = std::fmod(heading, 360.0);
	return heading < 0.0 ? heading + 360.0 : heading;
}
inline std::pair<double, double> roll_pitch_of(const Mat3 &axes, double compass_deg)
{
	const auto level = level_axes(compass_deg);
	const Vec3 forward_local{
			dot(level[0], axes[2]), dot(level[1], axes[2]), dot(level[2], axes[2])};
	const Vec3 right_local{
			dot(level[0], axes[0]), dot(level[1], axes[0]), dot(level[2], axes[0])};
	const double pitch =
			std::asin(std::clamp(-forward_local[1], -1.0, 1.0)) * 57.295779513082320876;
	const Vec3 down{0.0, 1.0, 0.0};
	const auto right0_raw = cross(down, forward_local);
	const double right0_norm = norm(right0_raw);
	if (right0_norm < 1e-9)
		return {0.0, pitch};
	const auto right0 = scale(right0_raw, 1.0 / right0_norm);
	const auto down0 = cross(forward_local, right0);
	const double roll = std::atan2(dot(right_local, down0), dot(right_local, right0)) *
						57.295779513082320876;
	return {roll, pitch};
}
inline Mat3 rotate_axes_about_z(const Mat3 &axes, double theta_deg)
{
	const double angle = theta_deg * .017453292519943295769;
	const double s = std::sin(angle), c = std::cos(angle);
	const Mat3 rotation{{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}};
	Mat3 result{};
	for (unsigned i = 0; i < 3; ++i)
		for (unsigned j = 0; j < 3; ++j)
			result[i][j] = dot(axes[i], rotation[j]);
	return result;
}
inline double radial_limit(CameraModel model, const std::vector<double> &parameters)
{
	double k1 = 0, k2 = 0, k3 = 0;
	if (model == CameraModel::Brown && parameters.size() >= 9) {
		k1 = parameters[4];
		k2 = parameters[5];
		k3 = parameters[8];
	} else if (parameters.size() >= 2) {
		k1 = parameters[1];
		if (parameters.size() >= 3)
			k2 = parameters[2];
	}
	for (unsigned i = 0; i < 1201; ++i) {
		double r = radial_limit_max * i / 1200.0;
		double x = model == CameraModel::Fisheye ? std::atan(r) : r, x2 = x * x;
		if (1 + 3 * k1 * x2 + 5 * k2 * x2 * x2 + 7 * k3 * x2 * x2 * x2 <= 0)
			return radial_limit_safety * r;
	}
	return radial_limit_max;
}

struct Projection
{
	double u{std::numeric_limits<double>::quiet_NaN()};
	double v{std::numeric_limits<double>::quiet_NaN()};
	double length_m{};
	bool valid() const { return std::isfinite(u) && std::isfinite(v); }
	bool inside_image() const { return valid() && u >= 0 && u < 1 && v >= 0 && v < 1; }
};

class Projector
{
	Camera camera_;
	double radial_limit_squared_;
	double big_;
	std::pair<double, double> distort(double x, double y) const
	{
		const auto &p = camera_.parameters;
		double r2 = x * x + y * y;
		if (camera_.model == CameraModel::Brown && p.size() >= 9) {
			double d = 1 + p[4] * r2 + p[5] * r2 * r2 + p[8] * r2 * r2 * r2;
			return {p[0] * (x * d + 2 * p[6] * x * y + p[7] * (r2 + 2 * x * x)) + p[2],
					p[1] * (y * d + p[6] * (r2 + 2 * y * y) + 2 * p[7] * x * y) + p[3]};
		}
		double f = p.empty() ? (camera_.focal > 0 ? camera_.focal / big_ : .85) : p[0];
		double k1 = p.size() > 1 ? p[1] : 0, k2 = p.size() > 2 ? p[2] : 0;
		if (camera_.model == CameraModel::Fisheye) {
			double r = std::sqrt(r2), theta = std::atan(r), t2 = theta * theta;
			double scale = r > 1e-12 ? theta * (1 + k1 * t2 + k2 * t2 * t2) / r : 1;
			return {f * x * scale, f * y * scale};
		}
		double d = 1 + k1 * r2 + k2 * r2 * r2;
		return {f * x * d, f * y * d};
	}

public:
	explicit Projector(const Camera &camera) :
			camera_(camera),
			radial_limit_squared_(
					camera.model == CameraModel::Spherical
							? std::numeric_limits<double>::infinity()
							: std::pow(radial_limit(camera.model, camera.parameters), 2)),
			big_(std::max({camera.width, camera.height, 1.0}))
	{
	}
	bool spherical() const { return camera_.model == CameraModel::Spherical; }
	Vec3 to_camera(const Vec3 &point) const
	{
		Vec3 d = sub(point, camera_.centre);
		return {dot(d, camera_.axes[0]), dot(d, camera_.axes[1]),
				dot(d, camera_.axes[2])};
	}
	Projection project(const Vec3 &point) const
	{
		Vec3 c = to_camera(point);
		double length = norm(c);
		if (spherical())
			return {.5 + std::atan2(c[0], c[2]) / 6.2831853071795864769,
					.5 + std::asin(
								 std::clamp(c[1] / std::max(length, 1e-12), -1.0, 1.0)) /
									3.1415926535897932385,
					length};
		if (c[2] <= 1e-6)
			return {{}, {}, length};
		double x = c[0] / c[2], y = c[1] / c[2];
		if (x * x + y * y >= radial_limit_squared_)
			return {{}, {}, length};
		auto [xi, yi] = distort(x, y);
		return {.5 + xi * big_ / std::max(camera_.width, 1.0),
				.5 + yi * big_ / std::max(camera_.height, 1.0), length};
	}
	std::optional<std::array<float, 2>> pixel(const Vec3 &point) const
	{
		auto p = project(point);
		if (!p.valid() || !camera_.width || !camera_.height)
			return {};
		if (spherical()) {
			double x = std::fmod(p.u * camera_.width - .5, camera_.width);
			if (x < 0)
				x += camera_.width;
			return std::array<float, 2>{
					float(x), float(std::clamp(p.v * camera_.height - .5, 0.0,
									  camera_.height - 1))};
		}
		return p.inside_image()
					   ? std::optional<std::array<float, 2>>{{float(p.u * camera_.width -
																	  .5),
								 float(p.v * camera_.height - .5)}}
					   : std::nullopt;
	}
};

inline Projection project_depth(const Camera &camera, const Vec3 &point)
{
	return Projector(camera).project(point);
}

// Construct the initial camera state from Graph metadata.  SfM rotation is
// preferred when it passes the same finite/roll sanity gates as Rust; callers
// that later resolve sequence or cluster poses can replace axes and sources.
inline Camera camera_from_meta(const PanoMeta &meta, const Frame &frame,
		const Params &params, const std::optional<Vec3> &shot_rotation = std::nullopt)
{
	const auto xy = frame.to_enu(meta.lon, meta.lat);
	const double compass = std::fmod(meta.compass + 360.0, 360.0);
	const double height = params.height_prior(meta.camera_type);
	Camera camera;
	camera.pano_id = meta.id;
	camera.model = meta.camera_type;
	camera.centre = {xy[0], xy[1], meta.alt};
	camera.axes = level_axes(compass);
	camera.pose_source = PoseSource::Heading;
	camera.ground_z = meta.alt - height;
	camera.cam_height_m = height;
	camera.ground_source = GroundSource::Default;
	camera.cluster_id = meta.cluster_id;
	camera.compass_deg = compass;
	camera.pose_factor = pose_factor(PoseSource::Heading);
	camera.width = meta.width;
	camera.height = meta.height;
	camera.parameters = meta.camera_params;
	const auto try_rotation = [&](const std::optional<Vec3> &rotation) {
		if (!rotation)
			return false;
		const auto axes = axes_from_rotation(*rotation);
		for (const auto &axis : axes)
			if (!finite(axis))
				return false;
		const auto roll_pitch = roll_pitch_of(axes, heading_of(axes));
		if (meta.is_spherical() && std::abs(roll_pitch.first) > params.roll_max_deg)
			return false;
		camera.axes = axes;
		camera.roll_deg = roll_pitch.first;
		camera.pitch_deg = roll_pitch.second;
		camera.pose_source = PoseSource::Sfm;
		camera.pose_factor = pose_factor(PoseSource::Sfm);
		return true;
	};
	if (!try_rotation(meta.rotation))
		try_rotation(shot_rotation);
	return camera;
}

inline std::optional<std::pair<double, double>> sequence_roll_pitch(const PanoMeta &meta,
		const std::vector<const PanoMeta *> &sequence_metas, const Params &params,
		std::size_t *used = nullptr, double *deviation = nullptr)
{
	std::vector<double> rolls, pitches;
	for (const auto *candidate : sequence_metas) {
		if (!candidate || candidate->id == meta.id ||
				candidate->sequence != meta.sequence || !candidate->rotation ||
				std::abs(candidate->captured_at - meta.captured_at) >
						static_cast<std::int64_t>(params.seq_window_s * 1000.0))
			continue;
		const auto axes = axes_from_rotation(*candidate->rotation);
		const auto rp = roll_pitch_of(axes, heading_of(axes));
		if (std::abs(rp.first) <= params.roll_max_deg) {
			rolls.push_back(rp.first);
			pitches.push_back(rp.second);
		}
	}
	if (used)
		*used = rolls.size();
	if (rolls.size() < params.seq_min_panos)
		return std::nullopt;
	const double mean = std::accumulate(rolls.begin(), rolls.end(), 0.0) / rolls.size();
	double variance = 0.0;
	for (const double roll : rolls)
		variance += (roll - mean) * (roll - mean);
	if (deviation)
		*deviation = std::sqrt(variance / rolls.size());
	const auto median = [](std::vector<double> values) {
		std::sort(values.begin(), values.end());
		const auto middle = values.size() / 2;
		return values.size() % 2 ? values[middle]
								 : .5 * (values[middle - 1] + values[middle]);
	};
	return std::make_pair(median(rolls), median(pitches));
}

inline Camera camera_from_meta(const PanoMeta &meta, const Frame &frame,
		const std::vector<const PanoMeta *> &sequence_metas, const Params &params,
		const std::optional<Vec3> &shot_rotation = std::nullopt)
{
	Camera camera = camera_from_meta(meta, frame, params, shot_rotation);
	if (camera.pose_source == PoseSource::Sfm || !meta.is_spherical())
		return camera;
	double deviation = 0.0;
	if (const auto sequence = sequence_roll_pitch(
				meta, sequence_metas, params, nullptr, &deviation)) {
		const double spread = std::max(deviation, params.roll_sd_floor_deg);
		// Rust rejects an outlying panorama relative to the sequence median.
		const auto own_axes = meta.rotation ? axes_from_rotation(*meta.rotation)
											: level_axes(camera.compass_deg);
		const auto own = roll_pitch_of(own_axes, heading_of(own_axes));
		if (!meta.rotation ||
				std::abs(own.first - sequence->first) <= params.roll_sd_mult * spread) {
			camera.axes =
					level_axes(camera.compass_deg, sequence->first, sequence->second);
			camera.roll_deg = sequence->first;
			camera.pitch_deg = sequence->second;
			camera.pose_source = PoseSource::Sequence;
			camera.pose_factor = pose_factor(PoseSource::Sequence);
		}
	}
	return camera;
}
} // namespace arnis::mapillary::pose
