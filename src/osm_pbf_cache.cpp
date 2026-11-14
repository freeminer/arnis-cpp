#include "osm_pbf_cache.h"

#include "cache_root.h"
#include "osm_pbf.h"
#include "world_utils.h"

#include <nlohmann/json.hpp>
#include <zstd.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace arnis::osm_pbf
{
namespace
{
constexpr std::uint64_t MAX_BAKE_BYTES = 1024ULL * 1024 * 1024;

struct E7Box
{
	std::int64_t min_lat, min_lon, max_lat, max_lon;
	static E7Box from(const geographic::LLBBox &bbox, double pad_m)
	{
		const double middle = (bbox.min().lat() + bbox.max().lat()) * 0.5;
		const double dlat = pad_m / 111320.0;
		const double dlon =
				pad_m /
				(111320.0 * std::max(0.01, std::cos(middle * 0.017453292519943295)));
		return {static_cast<std::int64_t>(
						std::max(-90e7, std::floor((bbox.min().lat() - dlat) * 1e7))),
				static_cast<std::int64_t>(
						std::max(-180e7, std::floor((bbox.min().lng() - dlon) * 1e7))),
				static_cast<std::int64_t>(
						std::min(90e7, std::ceil((bbox.max().lat() + dlat) * 1e7))),
				static_cast<std::int64_t>(
						std::min(180e7, std::ceil((bbox.max().lng() + dlon) * 1e7)))};
	}
	bool contains(const E7Box &other) const
	{
		return min_lat <= other.min_lat && min_lon <= other.min_lon &&
			   max_lat >= other.max_lat && max_lon >= other.max_lon;
	}
	long double area() const
	{
		return static_cast<long double>(max_lat - min_lat) *
			   static_cast<long double>(max_lon - min_lon);
	}
	std::string filename() const
	{
		return std::to_string(min_lat) + "_" + std::to_string(min_lon) + "_" +
			   std::to_string(max_lat) + "_" + std::to_string(max_lon) + ".json.zst";
	}
	static std::optional<E7Box> from_filename(std::string_view name)
	{
		constexpr std::string_view suffix = ".json.zst";
		if (name.size() <= suffix.size() ||
				name.substr(name.size() - suffix.size()) != suffix)
			return std::nullopt;
		name.remove_suffix(suffix.size());
		std::array<std::int64_t, 4> values{};
		for (std::size_t i = 0; i < values.size(); ++i) {
			const auto delimiter = name.find('_');
			const auto token = name.substr(0, delimiter);
			if (token.empty())
				return std::nullopt;
			try {
				std::size_t used = 0;
				values[i] = std::stoll(std::string(token), &used);
				if (used != token.size())
					return std::nullopt;
			} catch (...) {
				return std::nullopt;
			}
			if (i + 1 != values.size()) {
				if (delimiter == std::string_view::npos)
					return std::nullopt;
				name.remove_prefix(delimiter + 1);
			} else if (delimiter != std::string_view::npos) {
				return std::nullopt;
			}
		}
		E7Box box{values[0], values[1], values[2], values[3]};
		if (box.min_lat > box.max_lat || box.min_lon > box.max_lon)
			return std::nullopt;
		return box;
	}
};

std::string cache_key(const std::filesystem::path &path)
{
	std::error_code ec;
	auto absolute = std::filesystem::weakly_canonical(path, ec);
	if (ec)
		absolute = std::filesystem::absolute(path, ec).lexically_normal();
	const auto size = std::filesystem::file_size(path, ec);
	const auto modified = std::filesystem::last_write_time(path, ec);
	const auto ticks = ec ? 0LL : modified.time_since_epoch().count();
	const auto source =
			absolute.string() + "|" + std::to_string(size) + "|" + std::to_string(ticks);
	std::uint64_t hash = 14695981039346656037ULL;
	for (const unsigned char c : source) {
		hash ^= c;
		hash *= 1099511628211ULL;
	}
	std::ostringstream out;
	out << std::hex << hash;
	return out.str();
}

std::shared_ptr<std::mutex> bake_mutex(const std::filesystem::path &path)
{
	static std::mutex mutex;
	static std::unordered_map<std::string, std::weak_ptr<std::mutex>> locks;
	const auto key = path.string();
	const std::lock_guard lock(mutex);
	auto &entry = locks[key];
	auto result = entry.lock();
	if (!result) {
		result = std::make_shared<std::mutex>();
		entry = result;
	}
	return result;
}

nlohmann::json to_json(const osm_parser::RawOsmDocument &document)
{
	nlohmann::json out{{"elements", nlohmann::json::array()}};
	for (const auto &node : document.nodes) {
		nlohmann::json e{{"type", "node"}, {"id", node.id}};
		if (node.lat)
			e["lat"] = *node.lat;
		if (node.lon)
			e["lon"] = *node.lon;
		if (!node.tags.empty())
			e["tags"] = node.tags;
		out["elements"].push_back(std::move(e));
	}
	for (const auto &way : document.ways) {
		nlohmann::json e{{"type", "way"}, {"id", way.id}, {"nodes", way.node_refs}};
		if (!way.tags.empty())
			e["tags"] = way.tags;
		out["elements"].push_back(std::move(e));
	}
	for (const auto &relation : document.relations) {
		nlohmann::json members = nlohmann::json::array();
		for (const auto &member : relation.members)
			members.push_back(
					{{"type", member.type}, {"ref", member.ref}, {"role", member.role}});
		nlohmann::json e{{"type", "relation"}, {"id", relation.id},
				{"members", std::move(members)}};
		if (!relation.tags.empty())
			e["tags"] = relation.tags;
		out["elements"].push_back(std::move(e));
	}
	if (document.bounds) {
		const auto &b = *document.bounds;
		out["bounds"] = {
				{"minlat", b[0]}, {"minlon", b[1]}, {"maxlat", b[2]}, {"maxlon", b[3]}};
	}
	return out;
}

std::optional<osm_parser::RawOsmDocument> load_bake(const std::filesystem::path &path)
{
	std::error_code ec;
	const auto compressed_size = std::filesystem::file_size(path, ec);
	if (ec || compressed_size > MAX_BAKE_BYTES)
		return std::nullopt;
	std::ifstream input(path, std::ios::binary);
	if (!input)
		return std::nullopt;
	std::vector<char> compressed(static_cast<std::size_t>(compressed_size));
	if (!compressed.empty())
		input.read(compressed.data(), static_cast<std::streamsize>(compressed.size()));
	if (!input && !compressed.empty())
		return std::nullopt;
	const auto frame_size =
			ZSTD_getFrameContentSize(compressed.data(), compressed.size());
	if (frame_size == ZSTD_CONTENTSIZE_ERROR || frame_size == ZSTD_CONTENTSIZE_UNKNOWN ||
			frame_size > MAX_BAKE_BYTES)
		return std::nullopt;
	std::string json(static_cast<std::size_t>(frame_size), '\0');
	const auto decoded = ZSTD_decompress(
			json.data(), json.size(), compressed.data(), compressed.size());
	if (ZSTD_isError(decoded) || decoded != json.size())
		return std::nullopt;
	std::istringstream stream(json);
	try {
		return osm_parser::parse_overpass_json(stream);
	} catch (...) {
		return std::nullopt;
	}
}

std::optional<std::filesystem::path> find_bake(
		const std::filesystem::path &directory, const E7Box &want)
{
	std::error_code ec;
	std::optional<std::pair<long double, std::filesystem::path>> best;
	for (std::filesystem::directory_iterator it(directory, ec), end; !ec && it != end;
			it.increment(ec)) {
		if (!it->is_regular_file(ec))
			continue;
		const auto name = it->path().filename().string();
		const auto box = E7Box::from_filename(name);
		if (!box || !box->contains(want))
			continue;
		const auto candidate = std::make_pair(box->area(), it->path());
		if (!best || candidate.first < best->first)
			best = candidate;
	}
	return best ? std::optional<std::filesystem::path>(best->second) : std::nullopt;
}
} // namespace

osm_parser::RawOsmDocument cached_cut_extract(
		const std::filesystem::path &pbf, const geographic::LLBBox &bbox, double pad_m)
{
	const auto want = E7Box::from(bbox, pad_m);
	const auto directory = cache::base_directory() / "osm-pbf" / "bakes" / cache_key(pbf);
	const auto exact_path = directory / want.filename();
	const auto lock = bake_mutex(exact_path);
	const std::lock_guard guard(*lock);
	if (const auto hit = find_bake(directory, want)) {
		if (auto document = load_bake(*hit))
			return std::move(*document);
		std::error_code ec;
		std::filesystem::remove(*hit, ec);
	}
	auto document = cut_extract(pbf, bbox, pad_m);
	if (document.nodes.empty() && document.ways.empty() && document.relations.empty())
		return document; // Empty coverage is not cached: the extract may have changed.
	const auto json = to_json(document).dump();
	const auto bound = ZSTD_compressBound(json.size());
	std::vector<std::uint8_t> compressed(bound);
	const auto written = ZSTD_compress(
			compressed.data(), compressed.size(), json.data(), json.size(), 3);
	if (ZSTD_isError(written) || written > MAX_BAKE_BYTES)
		return document;
	compressed.resize(written);
	std::error_code ec;
	std::filesystem::create_directories(directory, ec);
	if (!ec)
		world_utils::replace_file_atomically(exact_path, compressed);
	return document;
}
} // namespace arnis::osm_pbf
