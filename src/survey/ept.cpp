#include "ept.h"

#include "crs.h"

#include <algorithm>
#include <atomic>
#include <charconv>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <map>
#include <set>
#include <string_view>
#include <thread>

namespace arnis::survey::ept
{
namespace
{
constexpr double SPARSE = 0.1;
constexpr double PI = 3.14159265358979323846;

std::uint16_t read_u16(std::span<const std::uint8_t> data, std::size_t offset)
{
	return static_cast<std::uint16_t>(data[offset]) |
		   (static_cast<std::uint16_t>(data[offset + 1]) << 8);
}

std::uint32_t read_u32(std::span<const std::uint8_t> data, std::size_t offset)
{
	std::uint32_t value = 0;
	for (unsigned i = 0; i < 4; ++i)
		value |= static_cast<std::uint32_t>(data[offset + i]) << (i * 8);
	return value;
}

std::uint64_t read_u64(std::span<const std::uint8_t> data, std::size_t offset)
{
	std::uint64_t value = 0;
	for (unsigned i = 0; i < 8; ++i)
		value |= static_cast<std::uint64_t>(data[offset + i]) << (i * 8);
	return value;
}

double read_f64(std::span<const std::uint8_t> data, std::size_t offset)
{
	const auto bits = read_u64(data, offset);
	double value;
	std::memcpy(&value, &bits, sizeof(value));
	return value;
}
constexpr std::size_t MAX_HIERARCHY_REQUESTS = 100;

std::optional<std::int32_t> parse_i32(const std::string &text)
{
	if (text.empty())
		return std::nullopt;
	std::int32_t value = 0;
	const auto *begin = text.data();
	const auto *end = begin + text.size();
	const auto parsed = std::from_chars(begin, end, value);
	if (parsed.ec != std::errc{} || parsed.ptr != end)
		return std::nullopt;
	return value;
}

std::optional<std::int64_t> signed_count(const nlohmann::json &value)
{
	if (!value.is_number_integer() && !value.is_number_unsigned())
		return std::nullopt;
	try {
		if (value.is_number_unsigned()) {
			const auto number = value.get<std::uint64_t>();
			if (number >
					static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
				return std::nullopt;
			return static_cast<std::int64_t>(number);
		}
		return value.get<std::int64_t>();
	} catch (...) {
		return std::nullopt;
	}
}

Key parse_key(const std::string &text, bool &ok)
{
	Key key{};
	std::size_t begin = 0;
	for (std::size_t i = 0; i < key.size(); ++i) {
		const auto end = text.find('-', begin);
		const auto stop = end == std::string::npos ? text.size() : end;
		const auto part = parse_i32(text.substr(begin, stop - begin));
		if (!part || (i + 1 < key.size() && end == std::string::npos)) {
			ok = false;
			return {};
		}
		key[i] = *part;
		begin = stop + 1;
	}
	ok = true;
	return key;
}

std::string trim_ept_json(std::string url)
{
	constexpr std::string_view suffix = "ept.json";
	if (url.ends_with(suffix))
		url.resize(url.size() - suffix.size());
	while (!url.empty() && url.back() == '/')
		url.pop_back();
	return url;
}

} // namespace

std::string Node::key() const
{
	return std::to_string(level) + "-" + std::to_string(x) + "-" + std::to_string(y) +
		   "-" + std::to_string(z);
}

std::optional<std::string> parse_info(const nlohmann::json &json, Info &out)
{
	if (!json.is_object() || !json.contains("bounds") || !json["bounds"].is_array())
		return "ept.json without bounds";
	std::vector<double> bounds;
	for (const auto &value : json["bounds"]) {
		if (value.is_number())
			bounds.push_back(value.get<double>());
	}
	if (bounds.size() != 6)
		return "ept.json without bounds";
	if (!json.contains("dataType") || !json["dataType"].is_string() ||
			json["dataType"].get<std::string>() != "laszip")
		return "EPT data type " +
			   (json.contains("dataType") ? json["dataType"].dump() : "null");
	out.cube = {bounds[0], bounds[1], bounds[3], bounds[4]};
	return std::nullopt;
}

source::Rect node_rect(
		const Info &info, std::int32_t level, std::int32_t x, std::int32_t y)
{
	const auto bounded_level = std::clamp(level, 0, 30);
	const double size =
			(info.cube[2] - info.cube[0]) / static_cast<double>(1U << bounded_level);
	return {info.cube[0] + static_cast<double>(x) * size,
			info.cube[1] + static_cast<double>(y) * size,
			info.cube[0] + (static_cast<double>(x) + 1.0) * size,
			info.cube[1] + (static_cast<double>(y) + 1.0) * size};
}

bool meets(const source::Rect &a, const source::Rect &b)
{
	return a[0] < b[2] && a[2] > b[0] && a[1] < b[3] && a[3] > b[1];
}

Page parse_page(const Info &info, const nlohmann::json &json, const source::Rect &window)
{
	Page result;
	if (!json.is_object())
		return result;
	for (auto it = json.begin(); it != json.end(); ++it) {
		bool valid_key = false;
		const auto key = parse_key(it.key(), valid_key);
		if (!valid_key)
			continue;
		const auto [level, x, y, z] = key;
		if (level < 0 || level > copc::MAX_LEVEL ||
				!meets(node_rect(info, level, x, y), window))
			continue;
		const auto count = signed_count(it.value());
		if (!count)
			continue;
		if (*count == -1) {
			result.pages.push_back(key);
		} else if (*count > 0) {
			result.nodes.push_back({level, x, y, z, static_cast<std::uint64_t>(*count)});
		}
	}
	return result;
}

std::int32_t choose_level(const Info &info, const std::vector<Node> &nodes,
		double ground_area_factor, double target_density)
{
	std::map<std::int32_t, double> points_by_level;
	std::map<std::int32_t, std::set<std::pair<std::int32_t, std::int32_t>>> squares;
	std::int32_t deepest = 0;
	for (const auto &node : nodes) {
		points_by_level[node.level] += static_cast<double>(node.count);
		squares[node.level].emplace(node.x, node.y);
		deepest = std::max(deepest, node.level);
	}
	double density = 0.0;
	for (std::int32_t level = 0; level <= deepest; ++level) {
		const auto squares_at_level = squares.find(level);
		if (squares_at_level == squares.end())
			continue;
		const auto rect = node_rect(info, level, 0, 0);
		const double cell_area =
				(rect[2] - rect[0]) * (rect[3] - rect[1]) * ground_area_factor;
		const double area = cell_area * squares_at_level->second.size();
		if (area > 0.0)
			density += points_by_level[level] / area;
		else if (points_by_level[level] > 0.0)
			density = std::numeric_limits<double>::infinity();
		if (density >= target_density)
			return level;
	}
	return deepest;
}

std::vector<Node> select(const Info &info, const std::vector<Node> &nodes,
		std::int32_t level, double ground_area_factor)
{
	std::vector<Node> selected;
	for (const auto &node : nodes) {
		if (node.level > level)
			continue;
		const auto rect = node_rect(info, node.level, node.x, node.y);
		const double area =
				(rect[2] - rect[0]) * (rect[3] - rect[1]) * ground_area_factor;
		if (static_cast<double>(node.count) / area >= SPARSE)
			selected.push_back(node);
	}
	return selected;
}

std::string hierarchy_url(const std::string &base, const Key &key)
{
	return base + "/ept-hierarchy/" + std::to_string(key[0]) + "-" +
		   std::to_string(key[1]) + "-" + std::to_string(key[2]) + "-" +
		   std::to_string(key[3]) + ".json";
}

std::string data_url(const std::string &base, const Node &node)
{
	return base + "/ept-data/" + node.key() + ".laz";
}

std::variant<Decoded, std::string> decode(std::span<const std::uint8_t> data)
{
	if (data.size() < 227 || std::memcmp(data.data(), "LASF", 4) != 0)
		return std::string("not a LAS/LAZ file");
	const auto header_size = static_cast<std::size_t>(read_u16(data, 94));
	const auto point_offset = static_cast<std::size_t>(read_u32(data, 96));
	const auto vlr_count = read_u32(data, 100);
	const auto format_byte = data[104];
	const bool compressed = (format_byte & 0x80) != 0;
	const auto format = static_cast<std::uint8_t>(format_byte & 0x3f);
	const auto record_len = static_cast<std::size_t>(read_u16(data, 105));
	std::uint64_t point_count = read_u32(data, 107);
	if (point_count == 0 && data[25] >= 4 && data.size() >= 255)
		point_count = read_u64(data, 247);

	Decoded decoded;
	decoded.layout.format = format;
	decoded.layout.record_len = record_len;
	for (std::size_t axis = 0; axis < 3; ++axis) {
		decoded.layout.scale[axis] = read_f64(data, 131 + axis * 8);
		decoded.layout.offset[axis] = read_f64(data, 155 + axis * 8);
	}
	if (const auto error = decoded.layout.check())
		return *error;
	if (point_count > copc::MAX_NODE_POINTS ||
			point_count > copc::MAX_NODE_BYTES / record_len)
		return "unexpected point records (" + std::to_string(record_len) + " bytes, " +
			   std::to_string(point_count) + ")";

	std::size_t pos = header_size;
	bool has_laz_vlr = false;
	for (std::uint32_t i = 0; i < vlr_count; ++i) {
		if (pos > data.size() || 54 > data.size() - pos)
			return std::string("records past the end");
		const auto user = [&]() {
			std::string value(data.begin() + pos + 2, data.begin() + pos + 18);
			while (!value.empty() && value.back() == '\0')
				value.pop_back();
			return value;
		}();
		const auto record_id = static_cast<std::size_t>(read_u16(data, pos + 18));
		const auto length = static_cast<std::size_t>(read_u16(data, pos + 20));
		const auto payload = pos + 54;
		if (payload > data.size() || length > data.size() - payload)
			return std::string("records past the end");
		if (user == "laszip encoded" && record_id == 22204)
			has_laz_vlr = true;
		pos = payload + length;
	}
	if (has_laz_vlr)
		return std::string("LAZ point decompressor is unavailable");
	if (compressed)
		return std::string("LAZ without its record");
	if (point_offset > data.size() ||
			point_count > (data.size() - point_offset) / record_len)
		return std::string("short LAS file");
	const auto bytes = static_cast<std::size_t>(point_count) * record_len;
	decoded.records.assign(
			data.begin() + point_offset, data.begin() + point_offset + bytes);
	return decoded;
}

HierarchyResult read_hierarchy(
		const std::string &ept_json_url, const source::Rect &window)
{
	const auto base = trim_ept_json(ept_json_url);
	auto body = fetch::get_bytes(ept_json_url);
	if (auto *failure = std::get_if<fetch::Failure>(&body))
		return *failure;
	auto &bytes = std::get<std::vector<std::uint8_t>>(body);
	fetch::count_laser_bytes(bytes.size());
	const auto json = nlohmann::json::parse(bytes.begin(), bytes.end(), nullptr, false);
	if (json.is_discarded())
		return fetch::Failure{fetch::Failure::Kind::Unusable, "invalid ept.json"};
	Hierarchy result;
	if (const auto error = parse_info(json, result.info))
		return fetch::Failure{fetch::Failure::Kind::Unusable, *error};
	const auto [latitude, longitude] =
			crs::Crs{crs::Kind::WebMercator}.inverse(0.0, (window[1] + window[3]) / 2.0);
	(void)longitude;
	result.ground_factor = std::pow(std::cos(latitude * (PI / 180.0)), 2.0);

	std::vector<Key> pages{{0, 0, 0, 0}};
	result.requests = 1;
	while (!pages.empty()) {
		const auto key = pages.back();
		pages.pop_back();
		auto page_body = fetch::get_bytes(hierarchy_url(base, key));
		if (auto *failure = std::get_if<fetch::Failure>(&page_body))
			return failure->of_part();
		auto &page_bytes = std::get<std::vector<std::uint8_t>>(page_body);
		++result.requests;
		fetch::count_laser_bytes(page_bytes.size());
		const auto page_json = nlohmann::json::parse(
				page_bytes.begin(), page_bytes.end(), nullptr, false);
		if (page_json.is_discarded())
			return fetch::Failure{
					fetch::Failure::Kind::Other, "invalid EPT hierarchy page"};
		auto parsed = parse_page(result.info, page_json, window);
		result.nodes.insert(result.nodes.end(), parsed.nodes.begin(), parsed.nodes.end());
		for (const auto &child : parsed.pages)
			if (child != key)
				pages.push_back(child);
		if (result.requests > MAX_HIERARCHY_REQUESTS)
			return fetch::Failure{fetch::Failure::Kind::Other, "hierarchy too large"};
	}
	return result;
}

ReadResult read(const std::string &ept_json_url, const source::Rect &window,
		double target_density, std::size_t threads, const Visit &visit)
{
	const auto base = trim_ept_json(ept_json_url);
	auto hierarchy_result = read_hierarchy(ept_json_url, window);
	if (auto *failure = std::get_if<fetch::Failure>(&hierarchy_result))
		return *failure;
	auto &hierarchy = std::get<Hierarchy>(hierarchy_result);
	const auto level = choose_level(
			hierarchy.info, hierarchy.nodes, hierarchy.ground_factor, target_density);
	const auto chosen =
			select(hierarchy.info, hierarchy.nodes, level, hierarchy.ground_factor);
	copc::ReadStats stats{level, chosen.size(), hierarchy.requests + chosen.size(), 0, 0};
	if (chosen.empty())
		return stats;

	struct Outcome
	{
		std::uint64_t bytes = 0;
		std::uint64_t points = 0;
		std::optional<fetch::Failure> failure;
	};
	std::vector<Outcome> outcomes(chosen.size());
	std::atomic<std::size_t> next{0};
	const auto worker_count = std::min(std::max<std::size_t>(threads, 1), chosen.size());
	std::vector<std::jthread> workers;
	workers.reserve(worker_count);
	for (std::size_t worker = 0; worker < worker_count; ++worker) {
		workers.emplace_back([&] {
			for (;;) {
				const auto index = next.fetch_add(1, std::memory_order_relaxed);
				if (index >= chosen.size())
					return;
				auto &outcome = outcomes[index];
				auto allowed = fetch::within_laser_limit();
				if (const auto *failure = std::get_if<fetch::Failure>(&allowed)) {
					outcome.failure = *failure;
					continue;
				}
				if (!std::get<bool>(allowed)) {
					outcome.failure = fetch::Failure{
							fetch::Failure::Kind::Other, fetch::PAST_LIMIT};
					continue;
				}
				auto response = fetch::get_bytes(data_url(base, chosen[index]));
				if (auto *failure = std::get_if<fetch::Failure>(&response)) {
					outcome.failure = failure->of_part();
					continue;
				}
				auto &point_bytes = std::get<std::vector<std::uint8_t>>(response);
				fetch::count_laser_bytes(point_bytes.size());
				outcome.bytes = point_bytes.size();
				auto decoded_result = decode(point_bytes);
				if (auto *error = std::get_if<std::string>(&decoded_result)) {
					outcome.failure = fetch::Failure{fetch::Failure::Kind::Other, *error};
					continue;
				}
				auto &decoded = std::get<Decoded>(decoded_result);
				outcome.points = decoded.records.size() / decoded.layout.record_len;
				try {
					visit(decoded.layout, decoded.records);
				} catch (const std::exception &error) {
					outcome.failure =
							fetch::Failure{fetch::Failure::Kind::Other, error.what()};
				} catch (...) {
					outcome.failure = fetch::Failure{
							fetch::Failure::Kind::Other, "point visitor failed"};
				}
			}
		});
	}
	for (auto &worker : workers)
		worker.join();
	for (const auto &outcome : outcomes) {
		if (outcome.failure)
			return *outcome.failure;
		stats.bytes += outcome.bytes;
		stats.points += outcome.points;
	}
	return stats;
}
} // namespace arnis::survey::ept
