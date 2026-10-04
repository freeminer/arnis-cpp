#include "sfm.h"

#include "geometry.h"

#include <charconv>
#include <cmath>
#include <array>
#include <limits>
#include <tuple>
#include <zlib.h>

namespace arnis::mapillary::sfm
{
namespace
{
constexpr std::size_t inflated_cluster_limit = 256U * 1024U * 1024U;

std::optional<std::vector<std::uint8_t>> inflate_cluster(
		const std::vector<std::uint8_t> &bytes)
{
	if (bytes.empty() || bytes.size() > std::numeric_limits<uInt>::max())
		return {};
	z_stream stream{};
	stream.next_in = const_cast<Bytef *>(bytes.data());
	stream.avail_in = static_cast<uInt>(bytes.size());
	if (inflateInit(&stream) != Z_OK)
		return {};
	std::vector<std::uint8_t> output;
	std::array<std::uint8_t, 64 * 1024> chunk{};
	int status = Z_OK;
	while (status == Z_OK) {
		stream.next_out = chunk.data();
		stream.avail_out = static_cast<uInt>(chunk.size());
		status = inflate(&stream, Z_NO_FLUSH);
		const auto produced = chunk.size() - stream.avail_out;
		if (produced > inflated_cluster_limit - output.size()) {
			status = Z_MEM_ERROR;
			break;
		}
		output.insert(output.end(), chunk.begin(), chunk.begin() + produced);
	}
	inflateEnd(&stream);
	return status == Z_STREAM_END ? std::optional{std::move(output)} : std::nullopt;
}

std::optional<std::array<double, 3>> vec3(const nlohmann::json *value)
{
	if (!value || !value->is_array() || value->size() < 3)
		return std::nullopt;
	std::array<double, 3> out{};
	for (std::size_t i = 0; i < out.size(); ++i) {
		if (!(*value)[i].is_number())
			return std::nullopt;
		out[i] = (*value)[i].get<double>();
	}
	return out;
}

const nlohmann::json *member(const nlohmann::json &value, const char *key)
{
	if (!value.is_object())
		return nullptr;
	const auto it = value.find(key);
	return it == value.end() ? nullptr : &*it;
}

double number_or(const nlohmann::json *value, double fallback)
{
	return value && value->is_number() ? value->get<double>() : fallback;
}

std::string string_or(const nlohmann::json *value)
{
	return value && value->is_string() ? value->get<std::string>() : std::string{};
}

std::optional<std::array<double, 3>> topocentric_to_run(
		const std::array<double, 3> &point, const std::array<double, 3> &reference,
		const Frame &frame)
{
	return geodesy::topocentric_to_run(point, reference, frame);
}
} // namespace

std::optional<nlohmann::json> parse_cluster_bytes(
		const std::vector<std::uint8_t> &bytes, std::string *error)
{
	if (error)
		error->clear();
	const auto inflated = inflate_cluster(bytes);
	const auto &json_bytes = inflated ? *inflated : bytes;
	try {
		return nlohmann::json::parse(json_bytes.begin(), json_bytes.end());
	} catch (const std::exception &exception) {
		if (error)
			*error = std::string("cluster JSON: ") + exception.what();
		return {};
	}
}

std::optional<Cluster> load_cluster(const nlohmann::json &document,
		const std::string &cluster_id, const Frame &frame, std::string *error)
{
	auto fail = [&](const char *message) -> std::optional<Cluster> {
		if (error)
			*error = message;
		return std::nullopt;
	};
	if (error)
		error->clear();
	std::vector<const nlohmann::json *> reconstructions;
	if (document.is_array()) {
		for (const auto &entry : document)
			reconstructions.push_back(&entry);
	} else if (document.is_object()) {
		reconstructions.push_back(&document);
	} else {
		return fail("reconstruction is neither a list nor an object");
	}
	if (reconstructions.empty())
		return fail("reconstruction file is empty");

	const nlohmann::json *best = nullptr;
	std::size_t best_point_count = 0;
	for (const auto *reconstruction : reconstructions) {
		const auto *points = member(*reconstruction, "points");
		const std::size_t count = points && points->is_object() ? points->size() : 0;
		if (!best || count > best_point_count) {
			best = reconstruction;
			best_point_count = count;
		}
	}
	if (!best)
		return fail("reconstruction file is empty");
	const auto *reference_lla = member(*best, "reference_lla");
	if (!reference_lla)
		return fail("reconstruction has no reference_lla");
	const auto *longitude = member(*reference_lla, "longitude");
	const auto *latitude = member(*reference_lla, "latitude");
	if (!longitude || !longitude->is_number())
		return fail("reference_lla has no longitude");
	if (!latitude || !latitude->is_number())
		return fail("reference_lla has no latitude");
	const std::array<double, 3> reference{longitude->get<double>(),
			latitude->get<double>(), number_or(member(*reference_lla, "altitude"), 0.0)};

	Cluster cluster;
	cluster.id = cluster_id;
	cluster.ref_lla = reference;
	cluster.offset = frame.to_enu(reference[0], reference[1]);
	cluster.frame = frame;
	cluster.scale_ok = true;
	cluster.metric_ratio = std::numeric_limits<double>::quiet_NaN();

	if (const auto *shots = member(*best, "shots"); shots && shots->is_object()) {
		cluster.shots.reserve(shots->size());
		for (auto it = shots->begin(); it != shots->end(); ++it) {
			const auto rotation = vec3(member(it.value(), "rotation"));
			const auto translation = vec3(member(it.value(), "translation"));
			if (!rotation || !translation)
				continue;
			Shot shot;
			shot.id = it.key();
			shot.rotation = *rotation;
			shot.axes = pose::axes_from_rotation(*rotation);
			const auto &t = *translation;
			const std::array<double, 3> centre_topocentric{
					-(shot.axes[0][0] * t[0] + shot.axes[1][0] * t[1] +
							shot.axes[2][0] * t[2]),
					-(shot.axes[0][1] * t[0] + shot.axes[1][1] * t[1] +
							shot.axes[2][1] * t[2]),
					-(shot.axes[0][2] * t[0] + shot.axes[1][2] * t[1] +
							shot.axes[2][2] * t[2])};
			const auto centre = topocentric_to_run(centre_topocentric, reference, frame);
			if (!centre)
				continue;
			shot.centre = *centre;
			shot.capture_time = number_or(member(it.value(), "capture_time"), 0.0);
			shot.sequence = string_or(member(it.value(), "skey"));
			shot.camera = string_or(member(it.value(), "camera"));
			const auto *compass = member(it.value(), "compass");
			shot.compass = compass ? number_or(member(*compass, "angle"),
											 std::numeric_limits<double>::quiet_NaN())
								   : std::numeric_limits<double>::quiet_NaN();
			cluster.shots.push_back(std::move(shot));
		}
	}

	struct KeyedPoint
	{
		std::int64_t id;
		std::array<double, 3> point;
		std::array<std::uint8_t, 3> color;
	};
	std::vector<KeyedPoint> keyed;
	if (const auto *points = member(*best, "points"); points && points->is_object()) {
		keyed.reserve(points->size());
		for (auto it = points->begin(); it != points->end(); ++it) {
			const auto coordinates = vec3(member(it.value(), "coordinates"));
			if (!coordinates)
				continue;
			std::int64_t point_id = std::numeric_limits<std::int64_t>::max();
			const auto parsed = std::from_chars(
					it.key().data(), it.key().data() + it.key().size(), point_id);
			if (parsed.ec != std::errc{} ||
					parsed.ptr != it.key().data() + it.key().size())
				point_id = std::numeric_limits<std::int64_t>::max();
			const auto color = vec3(member(it.value(), "color"))
									   .value_or(std::array<double, 3>{0.0, 0.0, 0.0});
			std::array<std::uint8_t, 3> rgb{};
			for (std::size_t channel = 0; channel < 3; ++channel)
				rgb[channel] = static_cast<std::uint8_t>(
						std::clamp(std::round(color[channel]), 0.0, 255.0));
			keyed.push_back({point_id, *coordinates, rgb});
		}
	}
	std::stable_sort(keyed.begin(), keyed.end(),
			[](const auto &a, const auto &b) { return a.id < b.id; });
	std::vector<std::array<double, 3>> points;
	std::vector<std::array<std::uint8_t, 3>> colors;
	points.reserve(keyed.size());
	colors.reserve(keyed.size());
	for (const auto &entry : keyed) {
		const auto point = topocentric_to_run(entry.point, reference, frame);
		if (!point)
			continue;
		points.push_back(*point);
		colors.push_back(entry.color);
	}
	cluster.set_points(std::move(points));
	cluster.colors = std::move(colors);
	return cluster;
}
} // namespace arnis::mapillary::sfm
