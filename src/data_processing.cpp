#include "data_processing.h"
#include "element_processing/fm_highway_connectivity.h"
#include "fm_ecoregion_cache.h"
#include "element_processing/signage.h"
#include "element_processing/advtrains.h"
#include <sys/types.h>
#include <unordered_set>
#include <array>
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <utility>
#include <cstdlib>
#include <fstream>
#include <condition_variable>
#include <mutex>
#include <unordered_map>
#include <thread>
#include <tuple>

#include "../../arnis_adapter.h"
#include "assets_root.h"
#include "bresenham.h"
#include "cache_root.h"
#include "building_suppression.h"
#include "element_processing/historic.h"
#include "element_processing/power.h"
#include "element_processing/emergency.h"
#include "element_processing/advertising.h"
#include "element_processing/bridges.h"
#include "element_processing/highway_tunnels.h"
#include "element_processing/road_markings.h"
#include "element_processing/buildings.h"
#include "element_processing/tree.h"
#include "element_processing/subprocessor/interior/index.h"
#include "building_facades/registry.h"
#include "ground_decoration.h"
#include "mapillary/atlas.h"
#include "mapillary/facades.h"
#include "mapillary/pipeline.h"
#include "mapillary/fetch.h"
#include "net.h"
#include "httpfetch.h"
#include "floodfill_cache.h"
#include "ground_generation.h"
#include "ecoregion.h"
#include "land_cover/land_cover.h"
#include "ore_generation.h"
#include "caves_deepslate.h"
#include "caves_carver.h"
#include "caves_ores.h"
#include "caves_water.h"
#include "trees/region.h"
#include "water_depth.h"
#include "world_editor/floor_state.h"
#include "clipping.h"
#include "structures/boat.h"
#include "structures/starship.h"
#include "structures/helicopter.h"
#include "structures/jetbridge.h"
#include "structures/plane.h"
#include "tile.h"
//#include "models_3d/wikidata/osm_models.h"
#include "models_3d/wikidata/remote_provider.h"
#include "canopy/canopy.h"
#include "elevation/elevation.h"
#include "trees/mapped.h"
#include "models_3d/pipeline.h"
#include "models_3d/placement_executor.h"
#include "models_3d/custom/client.h"
#include "models_3d/three_dmr/client.h"
#include "landmarks.h"
#include "trees/engine.h"
#include "decals/render.h"
#include "map_item_palette.h"
#include "util/base64.h"
#include "util/png.h"

namespace arnis
{

namespace
{
std::shared_ptr<arnis::trees::RegionSelector> cached_region_selector(
		const std::filesystem::path &root, const std::string &realm, double latitude,
		double longitude, double scale, int ground_level, arnis::trees::TreeSize max_size,
		double blocks_per_meter, const std::vector<std::uint16_t> &ecoregion_ids,
		bool exclude_palms, bool wait_for_load = false)
{
	// Tree pack loading eagerly opens every referenced schematic. Keep it out
	// of the synchronous generation path so terrain and structures can proceed.
	using Selector = std::shared_ptr<arnis::trees::RegionSelector>;
	// Rust loads the pack at every scale, but only attaches area-specific mixes
	// when schematic trees can actually be placed. Tiny-scale worlds use the
	// compact procedural tree path, so decoding cross-pack communities there is
	// wasted work and must not alter the selected pack.
	const auto effective_ecoregion_ids =
			scale >= MICRO_TREE_MAX_SCALE ? ecoregion_ids : std::vector<std::uint16_t>{};
	struct CacheState
	{
		struct Job
		{
			Selector selector;
			bool complete{};
			std::uint64_t used{};
		};
		std::mutex mutex;
		std::condition_variable ready;
		std::unordered_map<std::string, std::shared_ptr<Job>> selectors;
		std::uint64_t clock{};
		unsigned active_loaders{};
		void trim()
		{
			// Keep four completed location-specific selectors. Active callers and
			// waiters retain their own references, so eviction cannot invalidate them.
			for (;;) {
				std::size_t completed = 0;
				auto oldest = selectors.end();
				for (auto it = selectors.begin(); it != selectors.end(); ++it) {
					if (!it->second->complete)
						continue;
					++completed;
					if (oldest == selectors.end() ||
							it->second->used < oldest->second->used)
						oldest = it;
				}
				if (completed <= 4)
					return;
				selectors.erase(oldest);
			}
		}
	};
	// The initializer is detached so process shutdown must not destroy its cache
	// while the tree pack is still being read.
	static CacheState *state = new CacheState;
	// A selector retains location-specific latitude rules and may resolve its
	// default realm from the coordinates. Keep selectors for distinct locations
	// separate even when their other loading parameters match.
	const auto key = root.string() + "|" + realm + "|" +
					 std::to_string(std::bit_cast<std::uint64_t>(latitude)) + "|" +
					 std::to_string(std::bit_cast<std::uint64_t>(longitude)) + "|" +
					 std::to_string(std::bit_cast<std::uint64_t>(scale)) + "|" +
					 std::to_string(ground_level) + "|" +
					 std::to_string(static_cast<int>(max_size)) + "|" +
					 std::to_string(std::bit_cast<std::uint64_t>(blocks_per_meter)) +
					 "|" + (exclude_palms ? "exclude-palms" : "include-palms") + [&] {
						 std::string suffix;
						 for (const auto id : effective_ecoregion_ids)
							 suffix += "|" + std::to_string(id);
						 return suffix;
					 }();
	std::shared_ptr<CacheState::Job> job;
	{
		std::unique_lock lock(state->mutex);
		if (const auto it = state->selectors.find(key); it != state->selectors.end()) {
			job = it->second;
			job->used = ++state->clock;
			if (job->complete)
				return job->selector;
			if (!wait_for_load)
				return {};
			state->ready.wait(lock, [&] { return job->complete; });
			return job->selector;
		}
		job = std::make_shared<CacheState::Job>();
		job->used = ++state->clock;
		state->selectors.emplace(key, job);
	}
	auto load = [root, realm, latitude, longitude, scale, ground_level, max_size,
						blocks_per_meter, effective_ecoregion_ids, exclude_palms, job] {
		{
			std::unique_lock lock(state->mutex);
			state->ready.wait(lock, [&] { return state->active_loaders < 2; });
			++state->active_loaders;
		}
		Selector result;
		try {
			auto loaded = arnis::trees::RegionSelector::load_for_location(latitude,
					longitude, root, scale, ground_level,
					arnis::trees::SizeFilter::up_to(max_size), blocks_per_meter,
					realm.empty() ? std::nullopt : std::optional<std::string>(realm),
					effective_ecoregion_ids, exclude_palms);
			if (loaded)
				result = std::make_shared<arnis::trees::RegionSelector>(
						std::move(*loaded));
		} catch (...) {
			// A missing or malformed regional pack disables this optional source.
		}
		{
			std::lock_guard lock(state->mutex);
			job->selector = std::move(result);
			job->complete = true;
			job->used = ++state->clock;
			--state->active_loaders;
			state->trim();
		}
		state->ready.notify_all();
	};
	try {
		std::thread(load).detach();
	} catch (...) {
		// Thread resource exhaustion must not leave a pending job/waiter stranded.
		load();
	}
	if (wait_for_load) {
		std::unique_lock lock(state->mutex);
		state->ready.wait(lock, [&] { return job->complete; });
		return job->selector;
	}
	return {};
}

struct AircraftStripSegment
{
	double ax, az, bx, bz, half;

	bool contains(double x, double z) const
	{
		const double dx = bx - ax, dz = bz - az;
		const double length_squared = dx * dx + dz * dz;
		const double t =
				length_squared > 0.0
						? std::clamp(((x - ax) * dx + (z - az) * dz) / length_squared,
								  0.0, 1.0)
						: 0.0;
		const double px = ax + t * dx - x, pz = az + t * dz - z;
		return px * px + pz * pz <= half * half;
	}
};

void append_aircraft_strip_segments(std::vector<AircraftStripSegment> &segments,
		const ProcessedWay &way, double scale)
{
	double half_m = 12.0;
	const auto width = way.tags.get("width");
	if (!width.empty()) {
		std::string raw = width;
		const auto first = raw.find_first_not_of(" \t\r\n");
		if (first != std::string::npos) {
			raw.erase(0, first);
			const auto last = raw.find_last_not_of(" \t\r\n");
			raw.erase(last + 1);
			while (!raw.empty() && raw.back() == 'm')
				raw.pop_back();
			const auto end = raw.find_last_not_of(" \t\r\n");
			raw.erase(end == std::string::npos ? 0 : end + 1);
			try {
				std::size_t parsed = 0;
				const double value = std::stod(raw, &parsed);
				if (parsed == raw.size() && std::isfinite(value) && value > 0.0)
					half_m = std::clamp(value * 0.5, 6.0, 40.0);
			} catch (...) {
				// Match Rust: malformed width falls back to the default strip width.
			}
		}
	}
	const double half = std::max(1.0, std::round(half_m * scale));
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &a = way.nodes[i - 1];
		const auto &b = way.nodes[i];
		segments.push_back({static_cast<double>(a.x), static_cast<double>(a.z),
				static_cast<double>(b.x), static_cast<double>(b.z), half});
	}
}

bool aircraft_area_contains(
		const std::vector<std::pair<int, int>> &ring, double x, double z)
{
	bool inside = false;
	for (std::size_t i = 1; i < ring.size(); ++i) {
		const auto [x1i, z1i] = ring[i - 1];
		const auto [x2i, z2i] = ring[i];
		const double x1 = x1i, z1 = z1i, x2 = x2i, z2 = z2i;
		if ((z1 > z) != (z2 > z) && x < x1 + (z - z1) * (x2 - x1) / (z2 - z1))
			inside = !inside;
	}
	return inside;
}

std::size_t drop_buildings_on_aircraft_pavement(
		const std::vector<ProcessedElement> &elements,
		std::vector<ProcessedElement> &filtered_elements, double scale)
{
	std::vector<AircraftStripSegment> runways, taxiways;
	std::vector<std::vector<std::pair<int, int>>> runway_areas, aprons;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const bool closed = way.nodes.size() >= 4 &&
							way.nodes.front().x == way.nodes.back().x &&
							way.nodes.front().z == way.nodes.back().z;
		const auto aeroway = way.tags.get("aeroway");
		const auto area_kind = way.tags.get("area:aeroway");
		auto ring = [&] {
			std::vector<std::pair<int, int>> points;
			points.reserve(way.nodes.size());
			for (const auto &node : way.nodes)
				points.emplace_back(node.x, node.z);
			return points;
		};
		if (area_kind == std::optional<std::string>("runway") && closed) {
			runway_areas.push_back(ring());
		} else if (area_kind == std::optional<std::string>("taxiway") && closed) {
			aprons.push_back(ring());
		} else if (aeroway == std::optional<std::string>("runway") && closed &&
				   way.tags.get("area") == std::optional<std::string>("yes")) {
			runway_areas.push_back(ring());
		} else if (aeroway == std::optional<std::string>("runway")) {
			append_aircraft_strip_segments(runways, way, scale);
		} else if (aeroway == std::optional<std::string>("taxiway")) {
			append_aircraft_strip_segments(taxiways, way, scale);
		} else if (aeroway == std::optional<std::string>("apron") && closed) {
			aprons.push_back(ring());
		}
	}
	if (runways.empty() && taxiways.empty() && runway_areas.empty() && aprons.empty())
		return 0;

	double min_x = std::numeric_limits<double>::max();
	double min_z = std::numeric_limits<double>::max();
	double max_x = std::numeric_limits<double>::lowest();
	double max_z = std::numeric_limits<double>::lowest();
	for (const auto &strip : runways) {
		min_x = std::min(min_x, std::min(strip.ax, strip.bx) - strip.half);
		max_x = std::max(max_x, std::max(strip.ax, strip.bx) + strip.half);
		min_z = std::min(min_z, std::min(strip.az, strip.bz) - strip.half);
		max_z = std::max(max_z, std::max(strip.az, strip.bz) + strip.half);
	}
	for (const auto &strip : taxiways) {
		min_x = std::min(min_x, std::min(strip.ax, strip.bx) - strip.half);
		max_x = std::max(max_x, std::max(strip.ax, strip.bx) + strip.half);
		min_z = std::min(min_z, std::min(strip.az, strip.bz) - strip.half);
		max_z = std::max(max_z, std::max(strip.az, strip.bz) + strip.half);
	}
	for (const auto *areas : {&runway_areas, &aprons})
		for (const auto &area : *areas)
			for (const auto &[x, z] : area) {
				min_x = std::min(min_x, static_cast<double>(x));
				max_x = std::max(max_x, static_cast<double>(x));
				min_z = std::min(min_z, static_cast<double>(z));
				max_z = std::max(max_z, static_cast<double>(z));
			}

	auto should_drop = [&](const ProcessedElement &element) {
		if (!element.is_way())
			return false;
		const auto &way = element.as_way();
		if (way.nodes.size() < 3 ||
				(!way.tags.contains("building") && !way.tags.contains("building:part")))
			return false;
		double cx = 0.0, cz = 0.0;
		for (const auto &node : way.nodes) {
			cx += node.x;
			cz += node.z;
		}
		cx /= way.nodes.size();
		cz /= way.nodes.size();
		if (cx < min_x || cx > max_x || cz < min_z || cz > max_z)
			return false;
		if (std::any_of(runways.begin(), runways.end(),
					[&](const auto &strip) { return strip.contains(cx, cz); }) ||
				std::any_of(
						runway_areas.begin(), runway_areas.end(), [&](const auto &ring) {
							return aircraft_area_contains(ring, cx, cz);
						}))
			return true;
		const bool traced_overture =
				way.tags.get("source") == std::optional<std::string>("overture_maps");
		return traced_overture &&
			   (std::any_of(taxiways.begin(), taxiways.end(), [&](const auto &strip) {
				   return strip.contains(cx, cz);
			   }) || std::any_of(aprons.begin(), aprons.end(), [&](const auto &ring) {
				   return aircraft_area_contains(ring, cx, cz);
			   }));
	};
	if (std::none_of(elements.begin(), elements.end(), should_drop))
		return 0;
	auto candidate_elements = elements;
	const auto before = candidate_elements.size();
	candidate_elements.erase(std::remove_if(candidate_elements.begin(),
									 candidate_elements.end(), should_drop),
			candidate_elements.end());
	const auto dropped = before - candidate_elements.size();
	filtered_elements = std::move(candidate_elements);
	return dropped;
}
}

std::size_t drop_buildings_on_aircraft_pavement(
		std::vector<ProcessedElement> &elements, double scale)
{
	std::vector<ProcessedElement> filtered;
	const auto dropped = drop_buildings_on_aircraft_pavement(elements, filtered, scale);
	if (dropped > 0)
		elements = std::move(filtered);
	return dropped;
}

// Helper functions for ground fill area detection
namespace
{

// Rust's cave module deliberately uses one fixed, world-independent seed so
// streamed/tiled generation produces the same cave geometry at every entry
// point.  Deriving this from the current bbox makes neighboring generation
// requests disagree at their seams.
constexpr std::int64_t CAVE_SEED = 0xCA7ECA7E;

std::optional<std::string> decal_texture(
		const decals::DecalRegistry &registry, int map_id)
{
	const auto tile = registry.tile(map_id);
	if (!tile)
		return std::nullopt;
	const auto &[key, tile_x, tile_y] = *tile;
	const auto canvas = decals::render(key);
	constexpr std::uint32_t size = decals::TILE;
	std::vector<std::uint8_t> rgba(std::size_t(size) * size * 4);
	for (std::uint32_t y = 0; y < size; ++y)
		for (std::uint32_t x = 0; x < size; ++x) {
			const auto color = canvas.get(int(tile_x * size + x), int(tile_y * size + y));
			const auto [r, g, b] = map_palette::map_color_rgb(color);
			const auto offset = (std::size_t(y) * size + x) * 4;
			rgba[offset] = r;
			rgba[offset + 1] = g;
			rgba[offset + 2] = b;
			rgba[offset + 3] = color == TRANSPARENT ? 0 : 255;
		}
	const auto png = encodePNG(rgba.data(), size, size, 6);
	return "[png:" + base64_encode(png);
}

} // anonymous namespace

bool should_stream_to_disk(std::size_t tile_count, bool fillground)
{
	if (const char *v = std::getenv("ARNIS_STREAM_TO_DISK")) {
		if (std::string(v) == "1")
			return true;
		if (std::string(v) == "0")
			return false;
	}
	constexpr std::uint64_t base_mb = 500;
	constexpr std::uint64_t base_region_mb = 26;
	constexpr int vanilla_span =
			world_editor::DEFAULT_MAX_Y - world_editor::DEFAULT_MIN_Y;
	const auto span = static_cast<std::uint64_t>(std::max(
			world_editor::world_max_y() - world_editor::terrain_floor_y(), vanilla_span));
	const std::uint64_t per_region_mb =
			fillground ? base_region_mb * span / static_cast<std::uint64_t>(vanilla_span)
					   : base_region_mb;
	std::uint64_t available_mb = 0;
	std::ifstream mem("/proc/meminfo");
	std::string key;
	std::uint64_t kb;
	while (mem >> key >> kb) {
		if (key == "MemAvailable:") {
			available_mb = kb / 1024;
			break;
		}
		std::string unit;
		std::getline(mem, unit);
	}
	return available_mb > 0 &&
		   (base_mb + per_region_mb * tile_count) * 100 > available_mb * 55;
}

bool should_use_parallel_tiles(std::size_t tile_count, bool java_format)
{
	// Rust only parallelizes Java region work once there are enough tiles to
	// amortize thread/editor setup; Bedrock and Luanti retain ordered writes.
	return java_format && tile_count >= 3;
}

GenerationFeatureFlags generation_features(
		bool java_format, bool luanti_format, bool map_item, bool map_preview)
{
	GenerationFeatureFlags f;
	f.map_item = map_item && java_format;
	f.map_preview = map_preview && !luanti_format;
	f.branding = java_format;
	f.map_decals = java_format;
	return f;
}

GenerationTilePolicy generation_tile_policy(
		std::size_t tile_count, bool java_format, bool fillground)
{
	return {should_use_parallel_tiles(tile_count, java_format),
			// Rust's eviction path is Java-region specific.  Bedrock and Luanti
			// keep their native writers ordered and must never inherit the Java
			// stream-to-disk decision.
			java_format && should_stream_to_disk(tile_count, fillground)};
}

using FillExpirations = std::unordered_map<std::size_t, std::vector<std::uint64_t>>;

FillExpirations fills_expiring_at(
		const std::unordered_map<std::uint64_t, std::size_t> &last_use)
{
	FillExpirations result;
	for (const auto &[way_id, index] : last_use)
		result[index].push_back(way_id);
	return result;
}

void release_finished_fills(
		FloodFillCache &cache, const FillExpirations &expiring_at, std::size_t index)
{
	if (const auto found = expiring_at.find(index); found != expiring_at.end())
		for (const auto way_id : found->second)
			cache.release(way_id);
}

std::unordered_map<std::uint64_t, std::size_t> compute_last_fill_use(
		const std::vector<ProcessedElement> &elements,
		const std::vector<std::pair<std::size_t, std::size_t>> &element_tasks,
		const std::unordered_map<std::uint64_t, std::uint64_t> &part_groups,
		const std::unordered_map<std::uint64_t, std::vector<std::uint64_t>>
				&group_members)
{
	std::unordered_map<std::uint64_t, std::size_t> result;
	auto record_last_use = [&](std::uint64_t id, std::size_t index) {
		auto [it, inserted] = result.emplace(id, index);
		if (!inserted)
			it->second = std::max(it->second, index);
	};
	auto record_with_siblings = [&](std::uint64_t id, std::size_t index) {
		record_last_use(id, index);
		const auto group = part_groups.find(id);
		const auto seed = group == part_groups.end() ? id : group->second;
		for (const auto key : {osm_parser::seed_without_hint(seed),
					 osm_parser::seed_without_hint(id)}) {
			if (const auto members = group_members.find(key);
					members != group_members.end())
				for (const auto sibling_id : members->second)
					record_last_use(sibling_id, index);
		}
	};
	for (std::size_t task_index = 0; task_index < element_tasks.size(); ++task_index) {
		const auto &element = elements[element_tasks[task_index].first];
		if (element.is_way()) {
			record_with_siblings(element.as_way().id, task_index);
		} else if (element.is_relation()) {
			record_with_siblings(element.as_relation().id, task_index);
			for (const auto &member : element.as_relation().members)
				record_with_siblings(member.way.id, task_index);
		}
	}
	return result;
}

namespace
{

bool is_closed_ring(const std::vector<ProcessedNode> &nodes)
{
	return nodes.size() >= 4 && nodes.front().x == nodes.back().x &&
		   nodes.front().z == nodes.back().z;
}

bool same_point(const ProcessedNode &a, const ProcessedNode &b)
{
	return a.x == b.x && a.z == b.z;
}

void stitch_way_segments(std::vector<std::vector<ProcessedNode>> &rings)
{
	bool changed = true;
	while (changed) {
		changed = false;
		for (std::size_t i = 0; i < rings.size() && !changed; ++i) {
			if (rings[i].empty() || is_closed_ring(rings[i]))
				continue;
			for (std::size_t j = i + 1; j < rings.size(); ++j) {
				if (rings[j].empty() || is_closed_ring(rings[j]))
					continue;
				if (same_point(rings[i].back(), rings[j].front())) {
					rings[i].insert(rings[i].end(), rings[j].begin() + 1, rings[j].end());
				} else if (same_point(rings[i].front(), rings[j].back())) {
					rings[j].insert(rings[j].end(), rings[i].begin() + 1, rings[i].end());
					rings[i] = std::move(rings[j]);
				} else if (same_point(rings[i].front(), rings[j].front())) {
					std::reverse(rings[j].begin(), rings[j].end());
					rings[j].insert(rings[j].end(), rings[i].begin() + 1, rings[i].end());
					rings[i] = std::move(rings[j]);
				} else if (same_point(rings[i].back(), rings[j].back())) {
					std::reverse(rings[j].begin(), rings[j].end());
					rings[i].insert(rings[i].end(), rings[j].begin() + 1, rings[j].end());
				} else {
					continue;
				}
				rings.erase(rings.begin() + static_cast<std::ptrdiff_t>(j));
				changed = true;
				break;
			}
		}
	}
}

bool tag_is(const tags_t &tags, const std::string &key, const std::string &value)
{
	const auto it = tags.find(key);
	return it != tags.end() && it->second == value;
}

double ring_area(const std::vector<ProcessedNode> &nodes)
{
	if (nodes.size() < 3)
		return 0.0;
	double twice_area = 0.0;
	for (size_t i = 0; i < nodes.size(); ++i) {
		const auto &a = nodes[i];
		const auto &b = nodes[(i + 1) % nodes.size()];
		twice_area += static_cast<double>(a.x) * b.z - static_cast<double>(b.x) * a.z;
	}
	return std::abs(twice_area * 0.5);
}

bool landuse_paints_ground(const tags_t &tags)
{
	const auto it = tags.find("landuse");
	return it == tags.end() ||
		   (it->second != "residential" && it->second != "commercial");
}

std::optional<double> ground_fill_area(const ProcessedElement &element)
{
	double area = 0.0;
	if (element.is_way()) {
		const auto &way = element.as_way();
		const auto &tags = way.tags;
		if (tags.contains("building") || tags.contains("building:part") ||
				tags.contains("highway"))
			return std::nullopt;
		bool fills = false;
		if (tags.contains("landuse"))
			fills = landuse_paints_ground(tags);
		else if (tags.contains("natural")) {
			const auto natural = tags.find("natural");
			fills = natural->second != "tree_row" && !tag_is(tags, "amenity", "fountain");
		} else if (tags.contains("amenity")) {
			fills = false;
		} else if (tags.contains("leisure")) {
			fills = true;
		} else {
			fills = tag_is(tags, "place", "square");
		}
		if (!fills)
			return std::nullopt;
		area = ring_area(way.nodes);
	} else if (element.is_relation()) {
		const auto &rel = element.as_relation();
		const auto &tags = rel.tags;
		if (tags.contains("building") || tags.contains("building:part") ||
				tag_is(tags, "type", "building") || tags.contains("water") ||
				tag_is(tags, "natural", "water") || tag_is(tags, "natural", "bay"))
			return std::nullopt;
		const bool fills = tags.contains("natural") ||
						   (tags.contains("landuse") && landuse_paints_ground(tags)) ||
						   tag_is(tags, "leisure", "park");
		if (!fills)
			return std::nullopt;
		for (const auto &member : rel.members) {
			if (member.role == ProcessedMemberRole::Outer)
				area += ring_area(member.way.nodes);
		}
	} else {
		return std::nullopt;
	}
	return area > 0.0 ? std::optional<double>(area) : std::nullopt;
}

void sort_ground_fill_areas(std::vector<ProcessedElement> &elements)
{
	std::vector<size_t> slots;
	std::vector<double> areas;
	for (size_t i = 0; i < elements.size(); ++i) {
		if (const auto area = ground_fill_area(elements[i])) {
			slots.push_back(i);
			areas.push_back(*area);
		}
	}
	const size_t count = slots.size();
	if (count < 2)
		return;

	// src[k] is the compact area index that belongs at slots[k].
	std::vector<size_t> src(count);
	std::iota(src.begin(), src.end(), 0);
	std::stable_sort(src.begin(), src.end(), [&](size_t a, size_t b) {
		// Match Rust's f64::total_cmp, including signed zeroes and NaNs.
		const auto total_order_key = [](double value) {
			const auto bits = std::bit_cast<std::uint64_t>(value);
			constexpr std::uint64_t sign = std::uint64_t{1} << 63;
			return (bits & sign) ? ~bits : (bits ^ sign);
		};
		const auto area_a = total_order_key(areas[a]);
		const auto area_b = total_order_key(areas[b]);
		if (area_a != area_b)
			return area_a < area_b;
		return elements[slots[a]].id() < elements[slots[b]].id();
	});

	// Invert into "where does compact element j belong", then apply in place.
	std::vector<size_t> dest(count);
	for (size_t k = 0; k < count; ++k)
		dest[src[k]] = k;
	for (size_t k = 0; k < count; ++k)
		while (dest[k] != k) {
			const size_t target = dest[k];
			std::swap(elements[slots[k]], elements[slots[target]]);
			std::swap(dest[k], dest[target]);
		}
}

bool is_water_polygon_way(const ProcessedWay &way)
{
	return tag_is(way.tags, "natural", "water") || tag_is(way.tags, "natural", "bay") ||
		   tag_is(way.tags, "waterway", "riverbank") ||
		   tag_is(way.tags, "landuse", "reservoir") || way.tags.contains("water");
}

bool is_water_relation(const ProcessedRelation &rel)
{
	return rel.tags.contains("water") || tag_is(rel.tags, "natural", "water") ||
		   tag_is(rel.tags, "natural", "bay") ||
		   tag_is(rel.tags, "waterway", "riverbank") ||
		   tag_is(rel.tags, "landuse", "reservoir");
}

int waterway_width(const ProcessedWay &way)
{
	const auto it_width = way.tags.find("width");
	if (it_width != way.tags.end()) {
		try {
			return std::max(1, static_cast<int>(std::round(std::stod(it_width->second))));
		} catch (...) {
		}
	}
	const auto it = way.tags.find("waterway");
	if (it == way.tags.end())
		return 1;
	if (it->second == "river")
		return 8;
	if (it->second == "canal")
		return 5;
	if (it->second == "stream" || it->second == "drain")
		return 2;
	return 3;
}

std::pair<std::size_t, std::size_t> grid_index_for(
		int x, int z, const XZBBox &xzbbox, std::size_t width, std::size_t height)
{
	const double xr = std::clamp(static_cast<double>(x - xzbbox.min_x()) /
										 static_cast<double>(std::max<int>(
												 1, xzbbox.max_x() - xzbbox.min_x())),
			0.0, 1.0);
	const double zr = std::clamp(static_cast<double>(z - xzbbox.min_z()) /
										 static_cast<double>(std::max<int>(
												 1, xzbbox.max_z() - xzbbox.min_z())),
			0.0, 1.0);
	const auto gx = std::min<std::size_t>(
			static_cast<std::size_t>(std::llround(xr * static_cast<double>(width - 1))),
			width - 1);
	const auto gz = std::min<std::size_t>(
			static_cast<std::size_t>(std::llround(zr * static_cast<double>(height - 1))),
			height - 1);
	return {gx, gz};
}

bool point_in_ring(double px, double pz, const std::vector<ProcessedNode> &ring)
{
	bool inside = false;
	for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
		const double xi = ring[i].x;
		const double zi = ring[i].z;
		const double xj = ring[j].x;
		const double zj = ring[j].z;
		if (((zi > pz) != (zj > pz)) && (px < (xj - xi) * (pz - zi) / (zj - zi) + xi))
			inside = !inside;
	}
	return inside;
}

void mark_grid_radius(
		land_cover::LandCoverData &lc, const XZBBox &xzbbox, int x, int z, int radius)
{
	if (lc.width == 0 || lc.height == 0)
		return;
	for (int dz = -radius; dz <= radius; ++dz) {
		for (int dx = -radius; dx <= radius; ++dx) {
			if (dx * dx + dz * dz > radius * radius)
				continue;
			const auto [gx, gz] =
					grid_index_for(x + dx, z + dz, xzbbox, lc.width, lc.height);
			lc.grid[gz][gx] = land_cover::LC_WATER;
		}
	}
}

void mark_polygon(land_cover::LandCoverData &lc, const XZBBox &xzbbox,
		const std::vector<ProcessedNode> &outer,
		const std::vector<std::vector<ProcessedNode>> &inners)
{
	if (!is_closed_ring(outer))
		return;
	int min_x = xzbbox.max_x();
	int max_x = xzbbox.min_x();
	int min_z = xzbbox.max_z();
	int max_z = xzbbox.min_z();
	for (const auto &n : outer) {
		min_x = std::min(min_x, n.x);
		max_x = std::max(max_x, n.x);
		min_z = std::min(min_z, n.z);
		max_z = std::max(max_z, n.z);
	}
	min_x = std::max(min_x, xzbbox.min_x());
	max_x = std::min(max_x, xzbbox.max_x());
	min_z = std::max(min_z, xzbbox.min_z());
	max_z = std::min(max_z, xzbbox.max_z());
	if (min_x > max_x || min_z > max_z)
		return;

	for (int z = min_z; z <= max_z; ++z) {
		for (int x = min_x; x <= max_x; ++x) {
			if (!point_in_ring(static_cast<double>(x) + 0.5, static_cast<double>(z) + 0.5,
						outer))
				continue;
			bool in_hole = false;
			for (const auto &inner : inners) {
				if (is_closed_ring(inner) &&
						point_in_ring(static_cast<double>(x) + 0.5,
								static_cast<double>(z) + 0.5, inner)) {
					in_hole = true;
					break;
				}
			}
			if (in_hole)
				continue;
			const auto [gx, gz] = grid_index_for(x, z, xzbbox, lc.width, lc.height);
			lc.grid[gz][gx] = land_cover::LC_WATER;
		}
	}
}

land_cover::LandCoverData build_osm_water_land_cover(
		const std::vector<ProcessedElement> &elements, const XZBBox &xzbbox)
{
	// Source selection is isolated here; ESAWorldCover can replace this provider
	// without changing bridge repair or ground-generation consumers.
	const auto width = static_cast<std::size_t>(xzbbox.max_x() - xzbbox.min_x() + 1);
	const auto height = static_cast<std::size_t>(xzbbox.max_z() - xzbbox.min_z() + 1);
	land_cover::LandCoverData lc;
	if (width == 0 || height == 0)
		return lc;
	lc.width = width;
	lc.height = height;
	lc.grid.assign(height, std::vector<uint8_t>(width, 0));

	for (const auto &element : elements) {
		if (element.is_way()) {
			const auto &way = element.as_way();
			const auto waterway = way.tags.find("waterway");
			if (is_water_polygon_way(way) && is_closed_ring(way.nodes)) {
				if (auto clipped = clipping::clip_water_ring_to_bbox(way.nodes, xzbbox))
					mark_polygon(lc, xzbbox, *clipped, {});
			} else if (waterway != way.tags.end() && !way.nodes.empty()) {
				const int radius = std::max(1, waterway_width(way) / 2);
				for (std::size_t i = 1; i < way.nodes.size(); ++i) {
					const auto points = bresenham_line(way.nodes[i - 1].x, 0,
							way.nodes[i - 1].z, way.nodes[i].x, 0, way.nodes[i].z);
					for (const auto &[x, y, z] : points) {
						(void)y;
						mark_grid_radius(lc, xzbbox, x, z, radius);
					}
				}
			}
		} else if (element.is_relation()) {
			const auto &rel = element.as_relation();
			if (!is_water_relation(rel))
				continue;
			std::vector<std::vector<ProcessedNode>> outers;
			std::vector<std::vector<ProcessedNode>> inners;
			for (const auto &member : rel.members) {
				if (member.way.nodes.empty())
					continue;
				if (member.role == ProcessedMemberRole::Inner)
					inners.push_back(member.way.nodes);
				else if (member.role == ProcessedMemberRole::Outer)
					outers.push_back(member.way.nodes);
			}
			stitch_way_segments(outers);
			stitch_way_segments(inners);
			for (const auto &outer : outers) {
				auto clipped = clipping::clip_water_ring_to_bbox(outer, xzbbox);
				if (clipped)
					mark_polygon(lc, xzbbox, *clipped, inners);
			}
		}
	}

	bool any = false;
	for (const auto &row : lc.grid) {
		if (std::find(row.begin(), row.end(), land_cover::LC_WATER) != row.end()) {
			any = true;
			break;
		}
	}
	if (!any) {
		lc.grid.clear();
		lc.width = 0;
		lc.height = 0;
	}
	return lc;
}

}

void prepare_elements_for_generation(std::vector<ProcessedElement> &elements)
{
	static constexpr std::array<const char *, 6> priority_tags = {
			"entrance", "building", "highway", "waterway", "water", "barrier"};
	std::vector<std::pair<std::size_t, std::size_t>> ranked;
	ranked.reserve(elements.size());
	for (std::size_t i = 0; i < elements.size(); ++i) {
		std::size_t priority = priority_tags.size();
		for (std::size_t p = 0; p < priority_tags.size(); ++p)
			if (elements[i].tags().contains(priority_tags[p])) {
				priority = p;
				break;
			}
		ranked.emplace_back(priority, i);
	}
	std::stable_sort(ranked.begin(), ranked.end(),
			[](const auto &a, const auto &b) { return a.first < b.first; });
	std::vector<ProcessedElement> ordered;
	ordered.reserve(elements.size());
	for (const auto &rank : ranked)
		ordered.push_back(std::move(elements[rank.second]));
	elements = std::move(ordered);
	sort_ground_fill_areas(elements);
}

// Forward declarations for all element processing functions
namespace buildings
{
void generate_building_from_relation(
		WorldEditor &editor, const ProcessedRelation &relation, const Args &args);
void generate_building_from_relation(WorldEditor &editor,
		const ProcessedRelation &relation, const Args &args,
		const FloodFillCache &flood_fill_cache, const XZBBox &xzbbox,
		const CoordinateBitmap &building_passages,
		const interior_uses::InteriorUseIndex *interior_index);
std::optional<building_facade::FacadeAnchor> generate_buildings(WorldEditor *editor,
		const ProcessedWay &element, const Args &args,
		const std::optional<int> &relation_levels);
std::optional<building_facade::FacadeAnchor> generate_buildings(WorldEditor *editor,
		const ProcessedWay &element, const Args &args,
		const std::optional<int> &relation_levels, const FloodFillCache &flood_fill_cache,
		const CoordinateBitmap &building_passages,
		const std::vector<HolePolygon> *hole_polygons,
		std::optional<std::uint64_t> style_seed, const CoordinateBitmap *road_mask,
		const CoordinateBitmap *building_footprints,
		const std::unordered_map<std::uint64_t, std::vector<std::uint64_t>>
				*group_members,
		const interior_uses::InteriorUseIndex *interior_index);
}

namespace highways
{
void generate_highways_internal(WorldEditor &editor, const ProcessedElement &element,
		const Args &args, const HighwayConnectivity &highway_connectivity,
		const std::optional<std::chrono::duration<double>> &floodfill_timeout,
		const RoadMaskBitmap &road_mask,
		const bridges::BridgeStructureMap &bridge_structures,
		const bridges::BridgeSurfaceMap &bridge_surface,
		const TunnelPortalMap &tunnel_portals,
		const road_markings::RoadMarkingIndex &markings,
		const CoordinateBitmap *tunnel_footprint);

void generate_highways(WorldEditor &editor, const ProcessedElement &element,
		const Args &args, const std::vector<ProcessedElement> &all_elements,
		const std::optional<std::chrono::duration<double>> &floodfill_timeout);
void generate_highways(WorldEditor &editor, const ProcessedElement &element,
		const Args &args, const std::vector<ProcessedElement> &all_elements,
		const std::optional<std::chrono::duration<double>> &floodfill_timeout,
		const RoadMaskBitmap &road_mask,
		const bridges::BridgeStructureMap &bridge_structures,
		const bridges::BridgeSurfaceMap &bridge_surface,
		const TunnelPortalMap &tunnel_portals,
		const CoordinateBitmap *tunnel_footprint = nullptr);
void generate_aeroway(WorldEditor &editor, const ProcessedWay &way, const Args &args);
void generate_aeroway(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const CoordinateBitmap &building_footprints);
void generate_helipad_node(WorldEditor &editor, const ProcessedNode &node,
		const Args &args, const CoordinateBitmap &building_footprints);
void generate_siding(WorldEditor &editor, const ProcessedWay &way);
void generate_siding(WorldEditor &editor, const ProcessedWay &way,
		const bridges::BridgeSurfaceMap &bridge_surface);
CoordinateBitmap collect_road_surface_coords(
		const std::vector<ProcessedElement> &elements, const WorldEditor &editor,
		const ::XZBBox &xzbbox, double scale);
CoordinateBitmap collect_carriageway_coords(const std::vector<ProcessedElement> &elements,
		const ::XZBBox &xzbbox, double scale);
CoordinateBitmap collect_building_passage_coords(
		const std::vector<ProcessedElement> &elements, const ::XZBBox &xzbbox,
		double scale);
}

namespace landuse
{
void generate_landuse(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const RoadMaskBitmap &road_mask, const bridges::BridgeSurfaceMap &bridge_surface);
void generate_landuse_from_relation(WorldEditor &editor, const ProcessedRelation &rel,
		const Args &args, FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const RoadMaskBitmap &road_mask, const bridges::BridgeSurfaceMap &bridge_surface);
void generate_place(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		FloodFillCache const &flood_fill_cache);
}

namespace natural
{
void generate_natural(WorldEditor &editor, const ProcessedElement &element,
		const Args &args, FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const bridges::BridgeSurfaceMap &bridge_surface);
void generate_natural_from_relation(WorldEditor &editor, const ProcessedRelation &rel,
		const Args &args, FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const bridges::BridgeSurfaceMap &bridge_surface);
}

namespace amenities
{
void generate_amenities(
		WorldEditor &editor, const ProcessedElement &element, const Args &args);
void generate_amenities(WorldEditor &editor, const ProcessedElement &element,
		const Args &args, const FloodFillCache &flood_fill_cache,
		const RoadMaskBitmap &road_mask);
}

namespace leisure
{
void generate_leisure(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const bridges::BridgeSurfaceMap &bridge_surface);
void generate_leisure_from_relation(WorldEditor &editor, const ProcessedRelation &rel,
		const Args &args, FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const bridges::BridgeSurfaceMap &bridge_surface);
}

namespace barriers
{
void generate_barriers(WorldEditor &editor, const ProcessedElement &element);
void generate_barriers(WorldEditor &editor, const ProcessedElement &element,
		const bridges::BridgeSurfaceMap &bridge_surface);
void generate_barrier_nodes(WorldEditor &editor, const ProcessedNode &node);
void generate_barrier_nodes(WorldEditor &editor, const ProcessedNode &node,
		const bridges::BridgeSurfaceMap &bridge_surface);
}

namespace waterways
{
void generate_waterways(WorldEditor &editor, const ProcessedWay &way);
}

namespace water_areas
{
StillWaterSurfaces prescan_still_surfaces(const std::vector<ProcessedElement> &elements,
		const Ground *ground, const XZBBox &xzbbox);
void generate_water_areas_from_relation(WorldEditor &editor, const ProcessedRelation &rel,
		const XZBBox &xzbbox, const water_depth::BigWaterField &bwf,
		const RoadMaskBitmap &road_mask, const RoadMaskBitmap *tunnel_footprint,
		const bridges::BridgeSurfaceMap &bridge_surface,
		std::optional<int> precomputed_surface);
void generate_water_area_from_way(WorldEditor &editor, const ProcessedWay &way,
		const water_depth::BigWaterField &bwf, const RoadMaskBitmap &road_mask,
		const RoadMaskBitmap *tunnel_footprint,
		const bridges::BridgeSurfaceMap &bridge_surface,
		std::optional<int> precomputed_surface);
}

namespace railways
{
using RailBridgeInternalEndpoints = std::vector<std::pair<int, int>>;
void generate_roller_coaster(WorldEditor &editor, const ProcessedWay &way);
void generate_railways(WorldEditor &editor, const ProcessedWay &element);
void generate_railways(WorldEditor &editor, const ProcessedWay &element,
		std::vector<std::pair<int, int>> &subway_points,
		const RailBridgeInternalEndpoints &rail_bridge_internal_endpoints,
		const bridge_styles::BridgeOutlineIndex &bridge_outlines,
		const CoordinateBitmap &road_mask, const CoordinateBitmap &building_footprints,
		const CoordinateBitmap &rail_mask,
		const bridges::BridgeStructureMap &bridge_structures,
		const bridges::BridgeSurfaceMap &bridge_surface);
RailBridgeInternalEndpoints collect_rail_bridge_internal_endpoints(
		const std::vector<ProcessedElement> &elements);
CoordinateBitmap collect_at_grade_rail_mask(
		const std::vector<ProcessedElement> &elements, const XZBBox &xzbbox);
void add_tunnel_footprint(const std::vector<ProcessedElement> &elements,
		const XZBBox &xzbbox, CoordinateBitmap &footprint);
void carve_subway_interior(
		WorldEditor &editor, const std::vector<std::pair<int, int>> &subway_points);
}

namespace tourisms
{
void generate_tourisms(
		WorldEditor &editor, const ProcessedNode &node, const RoadMaskBitmap &road_mask);
}

namespace man_made
{
void generate_man_made(
		WorldEditor &editor, const ProcessedElement &element, const Args &args);
void generate_man_made_nodes(WorldEditor &editor, const ProcessedNode &node);
void generate_man_made_nodes(
		WorldEditor &editor, const ProcessedNode &node, const Args &args);
}

namespace doors
{
void generate_doors(WorldEditor &editor, const ProcessedNode &rel);
}

namespace historic
{
void generate_pyramid(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const FloodFillCache &flood_fill_cache);
void generate_historic(WorldEditor &editor, const ProcessedNode &node);
}

namespace power
{
void generate_power(WorldEditor &editor, const ProcessedElement &element);
void generate_power_nodes(WorldEditor &editor, const ProcessedNode &node);
}

namespace emergency
{
void generate_emergency(WorldEditor &editor, const ProcessedNode &node);
}

namespace advertising
{
void generate_advertising(WorldEditor &editor, const ProcessedNode &node);
}

// Main generate_world function
bool generate_world(WorldEditor &editor,
		const std::vector<ProcessedElement> &input_elements, const Args &args_,
		FloodFillCache &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints, bool elements_prepared,
		const PreparedBuildingData *prepared_buildings, mapillary::FacadeJob *facade_job)
{
	Args effective_args = args_;
	// Rust's CLI and GUI both make cave generation imply underground fill.
	// Keep direct C++ library callers on the same path as well.
	if (effective_args.caves)
		effective_args.fillground = true;
	effective_args.apply_mode_defaults();
	effective_args.apply_body_defaults();
	// Rust no longer exposes a roof-generation toggle: roof handling is always
	// part of building generation. Keep the legacy C++ field for source
	// compatibility, but normalize it at the pipeline boundary.
	effective_args.roof = true;
	const Args &args = effective_args;
	if (!args.valid())
		return false;
	// Rust uses the same 3D toggle for bundled schematic props and model
	// placements.  Direct callers of generate_world do not pass through the
	// GenerationOptions adapter, so set the editor policy here as well.
	editor.set_place_schematics(args.use_3d);
	// Structure schematics and cave packs are distinct asset families. Keep
	// their roots separate so the installed Arnis assets directory can be
	// supplied independently from an optional user cave pack.
	editor.set_schematic_asset_root(assets::path("structures"));
	editor.set_cave_asset_root({});
	if (args.cave_asset_pack) {
		editor.set_cave_asset_root(*args.cave_asset_pack);
	} else if (args.caves) {
		const std::array<std::filesystem::path, 3> cave_pack_candidates = {
				assets::path("cave-pack"), std::filesystem::path("cave-pack"),
				std::filesystem::path(__FILE__).parent_path().parent_path() /
						"cave-pack"};
		for (const auto &candidate : cave_pack_candidates)
			if (std::filesystem::is_directory(candidate)) {
				editor.set_cave_asset_root(candidate);
				break;
			}
	}
	// Keep direct generation calls consistent with Rust's editor setup. Decals
	// are Java-only; the editor still receives the requested map-item policy,
	// while the format-specific exporter decides whether to emit it.
	const bool java_format = !args.bedrock && !args.luanti;
	// Voxy/Lod output also requires baked light data in the Rust pipeline,
	// even when the user did not explicitly request Java lighting.
	editor.set_bake_lighting(args.bake_lighting || args.voxy_lod);
	editor.set_start_with_map(args.map_item);
	editor.set_map_decals(java_format);
	editor.clear_facade_panels();
	mapillary::atlas::set_atlas_side(facade_atlas_side(args.facade_detail));
	std::optional<std::filesystem::path> facade_directory =
			args.building_facades_dir
					? std::optional<std::filesystem::path>(*args.building_facades_dir)
					: std::nullopt;
	if (!facade_directory && args.building_facades) {
		// Match Rust's executable/repository asset search instead of depending
		// on the caller's current working directory.
		const std::array<std::filesystem::path, 1> candidates = {
				assets::path("building-facades")};
		for (const auto &candidate : candidates)
			if (std::filesystem::is_directory(candidate)) {
				facade_directory = candidate;
				break;
			}
	}
	// Mapillary exports and preset building textures are different schemas.
	// Do not feed a Mapillary export directory to FacadeSet::load_directory;
	// Rust installs that export into its own facade store first.
	const bool facade_set_enabled = args.building_facades && java_format;
	building_facades::reset(
			facade_set_enabled, facade_directory, args.facade_px, args.scale);
	mapillary::facades::clear();
	// Rust treats an explicit off toggle as authoritative even when an export
	// directory was also supplied (the GUI can set both independently).
	if (args.mapillary_facades != std::optional<bool>(false) &&
			args.mapillary_facades_dir) {
		std::string facade_error;
		if (auto export_data = mapillary::facades::load_export(
					*args.mapillary_facades_dir, &facade_error))
			mapillary::facades::install_export(std::move(*export_data));
	}
	if (facade_job && (args.mapillary_facades == std::optional<bool>(false) ||
							  args.mapillary_facades_dir))
		facade_job->cancel();
	editor.reserve_ground_level_cache();
	world_editor::set_world_bounds(
			args.disable_height_limit && !args.bedrock ? -2032 : -64,
			args.disable_height_limit && !args.bedrock ? 2031 : 319);
	if (editor.ground) {
		editor.ground->set_celestial_body(args.body);
		editor.ground->set_extended_ceiling(args.disable_height_limit && !args.bedrock);
	}
	const int base_level = editor.ground ? editor.ground->base_level(args.ground_level)
										 : args.ground_level;
	int area_floor = base_level;
	if (args.one_world_run && args.disable_height_limit && editor.ground &&
			editor.ground->elevation_enabled) {
		std::optional<int> lowest;
		for (const auto &row : editor.ground->elevation_grid)
			for (const float height : row)
				if (std::isfinite(height)) {
					const double floored = std::floor(static_cast<double>(height));
					const int y = floored <= std::numeric_limits<int>::min()
										  ? std::numeric_limits<int>::min()
								  : floored >= std::numeric_limits<int>::max()
										  ? std::numeric_limits<int>::max()
										  : static_cast<int>(floored);
					lowest = lowest ? std::min(*lowest, y) : y;
				}
		if (lowest)
			area_floor = *lowest;
	}
	world_editor::set_terrain_floor_y(area_floor);
	world_editor::set_base_chunk_y(area_floor);
	// Match Rust's filler palette for out-of-bounds chunks on non-Earth bodies.
	const Block filler =
			args.body == CelestialBody::Moon
					? ANDESITE
					: (args.body == CelestialBody::Mars ? RED_TERRACOTTA : GRASS_BLOCK);
	world_editor::set_base_chunk_block_id(filler.id());
	static const std::vector<ProcessedElement> no_elements;
	std::vector<ProcessedElement> aircraft_filtered_elements;
	const std::vector<ProcessedElement> *elements_source = &input_elements;
	if (!args.skip_objects()) {
		const auto dropped = drop_buildings_on_aircraft_pavement(
				input_elements, aircraft_filtered_elements, args.scale);
		if (dropped > 0) {
			std::cout << "  Skipped " << dropped
					  << " building(s) on runways, taxiways and aprons\n";
			elements_source = &aircraft_filtered_elements;
		}
	}
	const auto &elements = args.skip_objects() ? no_elements : *elements_source;
	// Build the authored-trunk index once per generation.  Ground decoration
	// can then suppress only procedural crowns overlapping mapped trees, while
	// preserving deterministic results for streamed tiles.
	editor.mapped_trunks = std::make_shared<trees::mapped::MappedTrunks>(
			trees::mapped::MappedTrunks::collect(elements, args.scale));
	const auto geo = editor.geographic_bounds();
	const double centre_lat = (geo[0] + geo[1]) * .5, centre_lon = (geo[2] + geo[3]) * .5;
	if (editor.ground)
		editor.ground->set_snow_line_for_latitude(centre_lat);
	// Rust loads the global ecoregion raster for Earth runs and keeps it on
	// Ground so vegetation and climate decoration can query it per column.
	if (editor.ground && args.body == CelestialBody::Earth &&
			(!editor.ground->ecoregion_map() ||
					!editor.ground->ecoregion_map()->is_local())) {
		if (const auto &map = ecoregion::generation_map().map)
			editor.ground->set_ecoregion_map(std::move(*map));
	}
	editor.set_regional_tree_placer({});
	editor.set_mapped_regional_tree_placer({});
	editor.set_schematic_tree_pack_available(false);
	std::shared_ptr<trees::RegionSelector> shared_selector;
	const auto tree_pack_root = assets::path("tree-packs");
	std::optional<std::string> dominant_tree_realm;
	std::vector<std::uint16_t> area_ecoregion_ids;
	bool ecoregion_map_has_gaps = true;
	if (editor.ground && editor.ground->ecoregion_map()) {
		const auto &ecoregions = editor.ground->ecoregion_map();
		if (ecoregions->is_local()) {
			// Area-local maps are small and vary with the current bbox, so their
			// summary must be computed from this map.
			dominant_tree_realm = ecoregions->dominant_tree_pack();
			ecoregion_map_has_gaps = ecoregions->has_gaps();
			for (const auto &[id, count] : ecoregions->by_area()) {
				(void)count;
				if (ecoregion::tree_mix(id))
					area_ecoregion_ids.push_back(id);
			}
		} else {
			// The Freeminer chunk adapter may supply the immutable global raster.
			// Its summary is computed once by generation_map(); rescanning and
			// decompressing every source tile here stalls all EmergeThreads before
			// they reach object or terrain placement.
			const auto &summary = ecoregion::generation_map();
			dominant_tree_realm = summary.dominant_tree_realm;
			ecoregion_map_has_gaps = summary.has_gaps;
			for (const auto &[id, count] : summary.by_area) {
				(void)count;
				if (ecoregion::tree_mix(id))
					area_ecoregion_ids.push_back(id);
			}
		}
	}
	const double abs_centre_lat = std::abs(centre_lat);
	const bool unmapped_palms = ecoregion_map_has_gaps && abs_centre_lat <= 35.0;
	const bool any_area_ecoregion_allows_palms = std::any_of(
			area_ecoregion_ids.begin(), area_ecoregion_ids.end(), [&](std::uint16_t id) {
				const auto eco = ecoregion::lookup(id);
				return eco && ecoregion::palms_belong(*eco, abs_centre_lat);
			});
	const bool exclude_palms = !unmapped_palms && !any_area_ecoregion_allows_palms;
	if (!args.legacy_trees) {
		const auto selector = cached_region_selector(tree_pack_root,
				dominant_tree_realm.value_or(""), centre_lat, centre_lon, args.scale,
				base_level, args.max_tree_size,
				editor.ground ? editor.ground->elevation_blocks_per_meter : 0.0,
				area_ecoregion_ids, exclude_palms);
		if (selector)
			shared_selector = selector;
		if (shared_selector) {
			editor.set_tree_slot_spacing(shared_selector->base_spacing());
			editor.set_schematic_tree_pack_available(true);
		}
	}
	models_3d::RemoteModelProvider wikidata_provider(
			cache::provider_cache_root("wikidata"));
	models_3d::three_dmr::Client three_dmr_provider(cache::provider_cache_root("3dmr"));
	const auto model_asset_root = assets::path("models");
	models_3d::custom::Client custom_model_provider(
			cache::provider_cache_root("custom_models"), {}, model_asset_root);
	// Landmarks use their matched OSM feature's projected centre as the world
	// anchor.  This keeps the C++ mapgen host independent of Rust's geographic
	// projection plumbing while preserving the same suppression/late-placement
	// ordering.
	std::vector<landmarks::WorldAnchor> landmark_anchors;
	auto normalize_qid = [](std::string value) {
		const auto first = value.find_first_not_of(" \t\r\n");
		if (first == std::string::npos)
			return std::string{};
		const auto last = value.find_last_not_of(" \t\r\n");
		value = value.substr(first, last - first + 1);
		return value;
	};
	for (const auto &landmark : landmarks::catalogue()) {
		for (const auto &element : elements) {
			auto wikidata = element.tags().find("wikidata");
			const auto key = std::pair<std::string, std::uint64_t>{
					std::string(element.kind()), element.id()};
			const bool named_match = wikidata != element.tags().end() &&
									 normalize_qid(wikidata->second) == landmark.qid;
			const bool osm_match =
					std::find(landmark.osm_ids.begin(), landmark.osm_ids.end(), key) !=
					landmark.osm_ids.end();
			if (!named_match && !osm_match)
				continue;
			std::vector<std::pair<int, int>> points;
			if (element.is_node())
				points.emplace_back(element.as_node().x, element.as_node().z);
			else if (element.is_way())
				for (const auto &n : element.as_way().nodes)
					points.emplace_back(n.x, n.z);
			else
				for (const auto &m : element.as_relation().members)
					for (const auto &n : m.way.nodes)
						points.emplace_back(n.x, n.z);
			if (points.empty())
				continue;
			std::int64_t sx = 0, sz = 0;
			for (const auto &[x, z] : points) {
				sx += x;
				sz += z;
			}
			const auto count = static_cast<std::int64_t>(points.size());
			landmark_anchors.push_back({landmark.qid, static_cast<int>(sx / count),
					static_cast<int>(sz / count)});
			break;
		}
	}
	auto landmark_plan = landmarks::prescan(elements, landmark_anchors, args.scale);
	const auto model_pipeline =
			args.use_3d
					? std::optional<models_3d::Models3dPipeline>(
							  models_3d::Models3dPipeline::prescan_fetchable_models(
									  elements, args.scale,
									  [&](const std::string &key) {
										  return wikidata_provider.fetch(key).has_value();
									  },
									  [&]() {
										  return custom_model_provider
												  .fetch("stadium.glb")
												  .has_value();
									  },
									  0.0, landmark_plan.suppressed))
					: std::nullopt;
	// Match Rust's specificity ordering: smaller ground-cover polygons render
	// first, then larger areas fill the remaining uncovered cells.
	std::vector<ProcessedElement> ordered_elements;
	if (!elements_prepared) {
		ordered_elements = elements;
		prepare_elements_for_generation(ordered_elements);
	}
	const auto &render_elements = elements_prepared ? elements : ordered_elements;
	const auto plane_candidates =
			args.use_3d
					? structures::plane::prescan_placements(render_elements, args.scale)
					: std::vector<structures::plane::Placement>{};
	auto [min_x, min_z] = editor.get_min_coords();
	auto [max_x, max_z] = editor.get_max_coords();
	::XZBBox xzbbox(min_x, min_z, max_x, max_z);
	const int interior_pad = args.one_world_run ? one_world::CLIP_PAD_BLOCKS : 0;
	const ::XZBBox interior_clip_bbox(
			static_cast<int>(std::max<std::int64_t>(
					std::numeric_limits<int>::min(), std::int64_t(min_x) - interior_pad)),
			static_cast<int>(std::max<std::int64_t>(
					std::numeric_limits<int>::min(), std::int64_t(min_z) - interior_pad)),
			static_cast<int>(std::min<std::int64_t>(
					std::numeric_limits<int>::max(), std::int64_t(max_x) + interior_pad)),
			static_cast<int>(std::min<std::int64_t>(std::numeric_limits<int>::max(),
					std::int64_t(max_z) + interior_pad)));
	std::optional<interior_uses::InteriorUseIndex> interior_use_index;
	if (args.interior)
		interior_use_index.emplace(
				interior_uses::InteriorUseIndex::build(elements, interior_clip_bbox));
	// Rust waits for precomputation after terrain/OSM preparation, immediately
	// before its export is projected and buildings are placed. Explicit review
	// exports were installed above and remain authoritative.
	if (facade_job && args.mapillary_facades != std::optional<bool>(false) &&
			!args.mapillary_facades_dir) {
		if (const auto export_dir = facade_job->join()) {
			std::string facade_error;
			if (auto export_data =
							mapillary::facades::load_export(*export_dir, &facade_error))
				mapillary::facades::install_export(std::move(*export_data));
		}
	}
	mapillary::facades::set_displays_enabled(
			java_format && args.mapillary_facades != std::optional<bool>(false) &&
			args.mapillary_facade_mode == FacadeMode::Photos);
	mapillary::facades::project_export(elements, xzbbox, args.scale);
	building_facades::set_world_extent(
			xzbbox.min_x(), xzbbox.min_z(), xzbbox.max_x(), xzbbox.max_z());
	if (editor.ground && !editor.ground->has_land_cover()) {
		// Match Ground::new_enabled: land-cover samples use the same world/grid
		// dimensions as elevation, before applying OSM water/land overrides.
		const auto geographic = editor.geographic_bounds();
		const auto world_width = static_cast<std::size_t>(max_x - min_x + 1);
		const auto world_height = static_cast<std::size_t>(max_z - min_z + 1);
		const auto grid_dims =
				elevation::compute_grid_dims_for_world(world_width, world_height);
		const auto grid_width = std::get<2>(grid_dims);
		const auto grid_height = std::get<3>(grid_dims);
		auto land_cover = land_cover::fetch_land_cover_data(
				{geographic[0], geographic[2], geographic[1], geographic[3]}, grid_width,
				grid_height);
		if (land_cover.width == 0 || land_cover.height == 0)
			land_cover = build_osm_water_land_cover(elements, xzbbox);
		if (land_cover.width > 0 && land_cover.height > 0) {
			editor.ground->set_land_cover_data(
					std::move(land_cover), world_width, world_height);
		}
	}
	// Rust fetches canopy alongside land cover and only enables it for Earth.
	// Keep the acquisition optional for library hosts: a missing canopy tile
	// must not prevent terrain generation, while a successful raster is shared
	// by tree placement and the natural-surface pass.
	if (editor.ground && args.canopy_height && args.body == CelestialBody::Earth &&
			!editor.ground->has_canopy()) {
		const auto geographic = editor.geographic_bounds();
		const auto world_width = static_cast<std::size_t>(max_x - min_x + 1);
		const auto world_height = static_cast<std::size_t>(max_z - min_z + 1);
		const auto grid_dims =
				elevation::compute_grid_dims_for_world(world_width, world_height);
		const auto grid_width = std::get<2>(grid_dims);
		const auto grid_height = std::get<3>(grid_dims);
		if (auto canopy = canopy::fetch_canopy_data(cache::provider_cache_root("canopy"),
					geographic[0], geographic[2], geographic[1], geographic[3],
					grid_width, grid_height))
			editor.ground->set_canopy_data(std::move(*canopy), world_width, world_height);
	}
	if (editor.ground && editor.ground->has_land_cover()) {
		auto &land_cover = *editor.ground->land_cover;
		// These are elevation-mutating passes in Rust. Run them through Ground so
		// water snapping and bridge repair update the authoritative DEM, rather
		// than a rounded land-cover-sized sample that is discarded afterwards.
		editor.ground->apply_osm_water_override(elements, xzbbox);
		editor.ground->apply_osm_land_override(elements, xzbbox, args.scale);
		editor.ground->apply_bridge_land_cover_repair(elements, xzbbox, args.scale);
		editor.ground->mark_beaches();
		// Each override deliberately invalidates the interpolated shoreline
		// field. Rebuild it only after the complete Rust-equivalent override
		// sequence so the ground pass sees OSM-correct water and smooth coasts.
		land_cover.refresh_water_blend_grid();
	}
	auto road_mask_owner = std::make_shared<const RoadMaskBitmap>(
			highways::collect_road_surface_coords(elements, editor, xzbbox, args.scale));
	const auto &road_mask = *road_mask_owner;
	// Share the road bitmap when no additional paved area contributes cells.
	// The scope guard releases the generation mask on every exit path.
	auto sealed = flood_fill_cache.collect_sealed_surfaces(elements, road_mask);
	editor.set_sealed_surface(
			sealed ? std::make_shared<const CoordinateBitmap>(std::move(*sealed))
				   : road_mask_owner);
	struct SealedSurfaceScope
	{
		WorldEditor &editor;
		~SealedSurfaceScope() { editor.release_sealed_surface(); }
	} sealed_surface_scope{editor};
	if (editor.map_decals) {
		std::shared_ptr<const signage::SignageContext> context;
		if (signage_enabled(args.signage)) {
			auto signage_carriageway =
					highways::collect_carriageway_coords(elements, xzbbox, args.scale);
			context = signage::build_context(elements, args.signage,
					decals::detect_region(centre_lat, centre_lon), args.scale,
					signage_carriageway, editor.first_decal_map_id);
		}
		editor.set_signage_context(context);
		editor.set_decal_registry(context ? context->registry : nullptr);
	}
	if (editor.decal_registry && !editor.decal_frame_sink) {
		editor.set_decal_frame_sink([&editor](const WorldEditor::DecalFrame &frame) {
			const auto texture = decal_texture(*editor.decal_registry, frame.map_id);
			if (!texture)
				return false;
			Block node = block_definitions::DECAL_FRAME;
			editor.set_block_absolute(
					node, frame.x, frame.y, frame.z, std::nullopt, std::nullopt);
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc++11-narrowing"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
#endif
			return editor.mg &&
				   editor.mg->queueGeneratedDecal({frame.x, frame.y, frame.z}, *texture,
						   frame.map_id, frame.facing, frame.rotation, frame.glow);
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
		});
	}
	auto big_water_field = water_depth::compute_big_water_field(editor, xzbbox);
	// Pre-scan still water surfaces for consistent water levels across tiles
	StillWaterSurfaces still_surfaces;
	if (editor.ground)
		still_surfaces =
				water_areas::prescan_still_surfaces(elements, editor.ground, xzbbox);
	auto bridge_outlines = bridge_styles::BridgeOutlineIndex::build(elements);
	auto bridge_structures = bridges::BridgeStructureMap::build(
			elements, editor, bridge_outlines, args.scale);
	auto bridge_surface =
			bridges::BridgeSurfaceMap::build(elements, bridge_structures, args.scale);
	// Rust completes tree-pack loading before generation. Keep the C++ load
	// overlapped with earlier passes, but do not silently skip schematic trees
	// on this first generation merely because the async cache was still warming.
	if (!shared_selector && !args.legacy_trees) {
		shared_selector = cached_region_selector(tree_pack_root,
				dominant_tree_realm.value_or(""), centre_lat, centre_lon, args.scale,
				base_level, args.max_tree_size,
				editor.ground ? editor.ground->elevation_blocks_per_meter : 0.0,
				area_ecoregion_ids, exclude_palms, true);
		if (shared_selector)
			editor.set_tree_slot_spacing(shared_selector->base_spacing());
		if (shared_selector)
			editor.set_schematic_tree_pack_available(true);
	}
	// Rust switches to compact proportional trees before consulting schematic
	// packs at very small world scales. Keep the selector callbacks disabled in
	// that mode so both mapped and land-cover trees take the same micro path.
	if (shared_selector && args.scale >= MICRO_TREE_MAX_SCALE) {
		editor.set_regional_tree_placer(
				[&editor, shared_selector, &building_footprints, &bridge_surface](int x,
						int y, int z, std::uint8_t cover,
						std::optional<trees::Habitat> habitat_hint, bool tagged,
						bool wet_ground, bool allow_on_paved,
						bool density_decided) -> std::optional<bool> {
					trees::SlotRequest request;
					request.density_decided = density_decided;
					request.tagged = tagged;
					const auto [slot_x, slot_z] =
							trees::trunk_slot_s(x, z, shared_selector->base_spacing());
					const auto slot_cover =
							editor.ground ? editor.ground->cover_class(
													editor.ground_point(slot_x, slot_z))
										  : cover;
					request.wet_ground =
							editor.ground ? slot_cover == land_cover::LC_WETLAND ||
													slot_cover == land_cover::LC_MANGROVES
										  : wet_ground;
					request.eco = editor.ground
										  ? editor.ground->ecoregion_at(
													editor.ground_point(slot_x, slot_z))
										  : std::nullopt;
					request.want_size = editor.canopy_size_hint(slot_x, slot_z);
					if (request.eco) {
						const int beach_radius = std::clamp(
								static_cast<int>(std::lround(12.0 * editor.scale())), 2,
								12);
						for (const auto &[dx, dz] : std::array<std::pair<int, int>, 5>{
									 {{0, 0}, {beach_radius, 0}, {-beach_radius, 0},
											 {0, beach_radius}, {0, -beach_radius}}})
							if (editor.ground &&
									editor.ground->cover_class(editor.ground_point(
											slot_x + dx, slot_z + dz)) ==
											land_cover::LC_BEACH) {
								request.beach = true;
								break;
							}
					}
					if (!habitat_hint)
						return trees::place_selected_region_tree_for_cover(editor,
								*shared_selector, x, z, slot_cover, y, request,
								&building_footprints, &bridge_surface, allow_on_paved);
					return trees::place_selected_region_tree(editor, *shared_selector, x,
							z, *habitat_hint, y, request, &building_footprints,
							&bridge_surface, allow_on_paved);
				});
		editor.set_mapped_regional_tree_placer(
				[&editor, shared_selector, &building_footprints, &bridge_surface](int x,
						int y, int z, std::uint8_t cover,
						const trees::MappedRequest &request) -> std::optional<bool> {
					(void)cover;
					// Tree::create_mapped passes an absolute placement height, while
					// schematic placement expects an offset from local ground.
					const int ground_y = editor.get_ground_level(x, z);
					const int y_offset = y - ground_y;
					// Rust falls back to the requested absolute base when sampled
					// terrain elevation is unavailable.
					const int elevation = editor.terrain_level(x, z).value_or(y);
					return trees::place_selected_mapped_region_tree(editor,
							*shared_selector, x, z, elevation, y_offset, request,
							&building_footprints, &bridge_surface);
				});
	}
	struct RegionalTreeScope
	{
		WorldEditor &editor;
		~RegionalTreeScope()
		{
			editor.set_regional_tree_placer({});
			editor.set_mapped_regional_tree_placer({});
		}
	} regional_tree_scope{editor};
	auto building_passages =
			highways::collect_building_passage_coords(elements, xzbbox, args.scale);
	std::vector<std::pair<int, int>> subway_points;
	std::vector<highways::HighwayTunnelCell> highway_tunnel_cells;
	auto tunnel_internal_endpoints =
			highways::collect_tunnel_internal_endpoints(elements, xzbbox);
	auto tunnel_portals = highways::collect_tunnel_portals(
			elements, editor, bridge_structures, tunnel_internal_endpoints, args.scale);
	auto tunnel_footprint = highways::collect_tunnel_footprint(
			elements, editor, tunnel_internal_endpoints, xzbbox, args.scale);
	railways::advtrains::prepare_network(elements, editor);
	railways::add_tunnel_footprint(elements, xzbbox, tunnel_footprint);
	auto rail_bridge_internal_endpoints =
			railways::collect_rail_bridge_internal_endpoints(elements);
	// Centreline bitmap prevents catenary masts from being stamped into a
	// neighbouring parallel track, matching the Rust railway pass.
	auto rail_mask = railways::collect_at_grade_rail_mask(elements, xzbbox);

	// Both the parser-prepared path and direct library callers use the same
	// geographic suppression/grouping pass as Rust. Reconstructing a reduced
	// projected-space approximation here loses split relation rings and can
	// render duplicate building outlines.
	std::optional<PreparedBuildingData> computed_buildings;
	if (!prepared_buildings)
		computed_buildings = prepare_building_data(elements);
	const PreparedBuildingData &building_data =
			prepared_buildings ? *prepared_buildings : *computed_buildings;
	std::unordered_set<std::uint64_t> suppressed_building_outlines;
	auto building_part_groups = building_data.part_groups;
	for (const auto &[kind, id] : building_data.outline_suppression)
		if (kind == "way")
			suppressed_building_outlines.insert(id);
	const auto &suppressed_relations = building_data.suppressed_relations;
	std::unordered_map<std::uint64_t, std::vector<std::uint64_t>> building_group_members;
	// Rust groups siblings by the seed without packed facade hints.
	building_group_members.clear();
	for (const auto &[way_id, seed] : building_part_groups)
		building_group_members[osm_parser::seed_without_hint(seed)].push_back(way_id);
	for (auto it = building_group_members.begin(); it != building_group_members.end();) {
		if (it->second.size() < 2)
			it = building_group_members.erase(it);
		else {
			std::sort(it->second.begin(), it->second.end());
			++it;
		}
	}
	// A fill may be read by its way, by a relation containing it, and by every
	// sibling part's shared building/facade context. Tile ownership can schedule
	// one source element on multiple tiles, so compute expiry in task order and
	// keep each fill alive until its last tile has consumed it.
	const auto element_tiles = create_tiles(xzbbox, DEFAULT_TILE_SIZE);
	const bool tiled_element_dispatch =
			should_use_parallel_tiles(element_tiles.size(), java_format);
	std::vector<std::vector<std::size_t>> element_assignments;
	std::vector<std::pair<std::size_t, std::size_t>> element_tasks;
	if (tiled_element_dispatch) {
		element_assignments =
				assign_elements_to_tiles(render_elements, element_tiles, args.scale);
		std::size_t task_count = 0;
		for (const auto &assigned : element_assignments)
			task_count += assigned.size();
		element_tasks.reserve(task_count);
		for (std::size_t tile_index = 0; tile_index < element_assignments.size();
				++tile_index)
			for (const auto element_index : element_assignments[tile_index])
				element_tasks.emplace_back(element_index, tile_index);
	} else {
		element_tasks.reserve(render_elements.size());
		for (std::size_t element_index = 0; element_index < render_elements.size();
				++element_index)
			element_tasks.emplace_back(
					element_index, std::numeric_limits<std::size_t>::max());
	}
	auto last_fill_use = compute_last_fill_use(
			render_elements, element_tasks, building_part_groups, building_group_members);
	const auto expiring_fills = fills_expiring_at(last_fill_use);

	// All roads share the same immutable connectivity for this pass.
	std::optional<highways::HighwayConnectivity> highway_connectivity;
	const auto geo_bounds = editor.geographic_bounds();
	const auto marking_index =
			road_markings::RoadMarkingIndex::build(elements, args.scale,
					decals::detect_region((geo_bounds[0] + geo_bounds[1]) * .5,
							(geo_bounds[2] + geo_bounds[3]) * .5));
	// Rust finalizes each Java tile immediately after its elements. Keep the
	// same order on the shared C++ editor: strict bounds provide tile ownership,
	// while all generation inputs (ground, masks and relation fills) remain shared.
	const bool regional_ground_passes =
			tiled_element_dispatch && java_format && element_tiles.size() >= 3;
	auto begin_generation_tile = [&](std::size_t tile_index) {
		const auto &tile = element_tiles[tile_index];
		const int min_x = std::max(tile.min_x, xzbbox.min_x());
		const int max_x = std::min(tile.max_x - 1, xzbbox.max_x());
		const int min_z = std::max(tile.min_z, xzbbox.min_z());
		const int max_z = std::min(tile.max_z - 1, xzbbox.max_z());
		editor.set_strict_bounds(min_x, min_z, max_x, max_z);
		return editor.begin_tile(tile.min_x, tile.min_z, tile.max_x - 1, tile.max_z - 1);
	};
	auto finish_ground_tile = [&](std::size_t tile_index) {
		const auto &tile = element_tiles[tile_index];
		const int min_x = std::max(tile.min_x, xzbbox.min_x());
		const int max_x = std::min(tile.max_x - 1, xzbbox.max_x());
		const int min_z = std::max(tile.min_z, xzbbox.min_z());
		const int max_z = std::min(tile.max_z - 1, xzbbox.max_z());
		if (min_x > max_x || min_z > max_z)
			return editor.merge_tile(
					tile.min_x, tile.min_z, tile.max_x - 1, tile.max_z - 1);
		editor.set_strict_bounds(min_x, min_z, max_x, max_z);
		// These POI/address plates need their host building, and must precede
		// ground decoration just like other tile-owned element writes.
		if (editor.signage_enabled())
			for (const auto element_index : element_assignments[tile_index]) {
				const auto &element = render_elements[element_index];
				if (element.is_node())
					signage::generate_node_facade_signage(
							editor, element.as_node(), building_footprints, road_mask);
			}
		ground_generation::generate_ground_region(editor, args, xzbbox,
				building_footprints, min_x, max_x, min_z, max_z,
				tunnel_footprint.is_empty() ? nullptr : &tunnel_footprint,
				&bridge_surface);
		if (args.caves) {
			const caves::CaveRect cave_world{
					xzbbox.min_x(), xzbbox.max_x(), xzbbox.min_z(), xzbbox.max_z()};
			const caves::CaveRect cave_region{min_x, max_x, min_z, max_z};
			caves::apply_deepslate(editor, min_x, max_x, min_z, max_z);
			caves::CaveEllipsoids tile_ellipsoids;
			std::unordered_set<std::int64_t> cave_air;
			caves::carve_region(editor, cave_region, cave_world, CAVE_SEED,
					world_editor::terrain_floor_y(), &tile_ellipsoids, &cave_air);
			const caves::CaveShapeQuery cave_shape(editor, cave_world, CAVE_SEED,
					world_editor::terrain_floor_y(), tile_ellipsoids);
			std::unordered_set<std::int64_t> cave_basin_fluid;
			std::unordered_set<std::int64_t> cave_water_cells;
			caves::WaterPlan cave_water_plan;
			caves::generate_water_features(editor, cave_world, cave_region, CAVE_SEED,
					world_editor::terrain_floor_y(), args, cave_shape, &cave_basin_fluid,
					&cave_water_cells, &cave_air, &cave_water_plan);
			caves::stamp_schematics_region(editor, cave_region, cave_air, CAVE_SEED,
					world_editor::terrain_floor_y(), args, &cave_basin_fluid);
			caves::place_ores_region(
					editor, cave_region, CAVE_SEED, world_editor::terrain_floor_y());
			caves::decorate_region(editor, cave_region, cave_world, CAVE_SEED,
					world_editor::terrain_floor_y(), args, cave_shape, cave_water_plan,
					&cave_basin_fluid, &cave_water_cells, &cave_air);
		} else if (args.fillground) {
			ore_generation::generate_ores_region(
					editor, min_x, max_x, min_z, max_z, false);
		}
		water_depth::carve_lc_water_region(editor, big_water_field, road_mask,
				tunnel_footprint, min_x, max_x, min_z, max_z, &bridge_surface);
		return editor.merge_tile(tile.min_x, tile.min_z, tile.max_x - 1, tile.max_z - 1);
	};
	std::size_t active_tile = std::numeric_limits<std::size_t>::max();
	std::size_t next_ground_tile = 0;
	for (std::size_t task_index = 0; task_index < element_tasks.size(); ++task_index) {
		const auto [element_index, tile_index] = element_tasks[task_index];
		if (tile_index != active_tile) {
			if (regional_ground_passes &&
					active_tile != std::numeric_limits<std::size_t>::max()) {
				if (!finish_ground_tile(active_tile)) {
					std::cerr << "Failed to merge generated Arnis tile\n";
					editor.finish_tile_pass();
					return false;
				}
				next_ground_tile = active_tile + 1;
			}
			while (regional_ground_passes && next_ground_tile < tile_index) {
				if (!begin_generation_tile(next_ground_tile) ||
						!finish_ground_tile(next_ground_tile)) {
					std::cerr << "Failed to generate or merge empty Arnis tile\n";
					editor.finish_tile_pass();
					return false;
				}
				++next_ground_tile;
			}
			active_tile = tile_index;
			if (tiled_element_dispatch) {
				const auto &tile = element_tiles[tile_index];
				editor.set_strict_bounds(std::max(tile.min_x, xzbbox.min_x()),
						std::max(tile.min_z, xzbbox.min_z()),
						std::min(tile.max_x - 1, xzbbox.max_x()),
						std::min(tile.max_z - 1, xzbbox.max_z()));
				if (regional_ground_passes && !begin_generation_tile(tile_index)) {
					std::cerr << "Failed to begin Arnis tile generation\n";
					editor.finish_tile_pass();
					return false;
				}
			} else {
				editor.clear_strict_bounds();
			}
		}
		auto const &element = render_elements[element_index];
		// Keep per-element terrain adjustment, but start from the normalized
		// options used by the Rust generation pipeline rather than the raw caller
		// arguments (which may not yet include mode/body defaults).
		auto args = effective_args;
		if (element.is_relation() && suppressed_relations.count(element.id())) {
			release_finished_fills(flood_fill_cache, expiring_fills, task_index);
			continue;
		}
		if (building_data.outline_suppression.count(
					{std::string(element.kind()), element.id()})) {
			release_finished_fills(flood_fill_cache, expiring_fills, task_index);
			continue;
		}
		if (model_pipeline &&
				std::find(model_pipeline->suppressed().begin(),
						model_pipeline->suppressed().end(),
						std::pair<std::string, std::uint64_t>{std::string(element.kind()),
								element.id()}) != model_pipeline->suppressed().end()) {
			release_finished_fills(flood_fill_cache, expiring_fills, task_index);
			continue;
		}

		if (element.is_way()) {
			auto const &way = element.as_way();
			if (std::find(landmark_plan.suppressed.begin(),
						landmark_plan.suppressed.end(),
						std::pair<std::string, std::uint64_t>{"way", way.id}) !=
					landmark_plan.suppressed.end()) {
				release_finished_fills(flood_fill_cache, expiring_fills, task_index);
				continue;
			}
			if (editor.ground && !way.nodes.empty()) {
				args.ground_level = editor.ground->level(way.nodes.begin()->xz());
			}

			// Solar farms are commonly mapped as barrier=fence plus
			// power=generator.  The barrier handler would otherwise shadow the
			// generator, unlike the Rust dispatcher.
			if (way.tags.contains("barrier") && !way.tags.contains("building") &&
					way.tags.get("power") == std::optional<std::string>("generator")) {
				power::generate_power(editor, element, building_footprints,
						flood_fill_cache, args.timeout);
			}

			if (way.tags.contains("building") || way.tags.contains("building:part")) {
				// Skip building outlines that are suppressed by building relations with parts.
				// The individual building:part ways will render instead.
				if (suppressed_building_outlines.find(way.id) ==
						suppressed_building_outlines.end()) {
					auto group = building_part_groups.find(way.id);
					auto anchor = buildings::generate_buildings(&editor, way, args,
							std::optional<int>{}, flood_fill_cache, building_passages,
							nullptr,
							group == building_part_groups.end()
									? std::nullopt
									: std::optional<std::uint64_t>(group->second),
							&road_mask, &building_footprints, &building_group_members,
							interior_use_index ? &*interior_use_index : nullptr);
					if (editor.signage_enabled())
						signage::generate_building_signage(editor, way, anchor);
				}
			} else if (structures::jetbridge::claims(way)) {
				// Must precede highway: many jet bridges also have corridor/highway tags.
				structures::jetbridge::generate_jet_bridge(
						editor, way, building_footprints);
			} else if (way.tags.contains("highway")) {
				const bool tunnel_rendered =
						highways::renders_as_highway_tunnel(way) &&
						highways::generate_highway_tunnel_shell(editor, way, args,
								tunnel_internal_endpoints, tunnel_portals,
								highway_tunnel_cells);
				if (!tunnel_rendered) {
					if (!highway_connectivity)
						highway_connectivity =
								highways::build_highway_connectivity_map(elements);
					highways::generate_highways_internal(editor, element, args,
							*highway_connectivity, args.timeout, road_mask,
							bridge_structures, bridge_surface, tunnel_portals,
							marking_index,
							tunnel_footprint.is_empty() ? nullptr : &tunnel_footprint);
				}
				if (editor.signage_enabled())
					signage::generate_highway_way_signage(
							editor, way, building_footprints);
			} else if (way.tags.contains("landuse")) {
				landuse::generate_landuse(editor, way, args, flood_fill_cache,
						building_footprints, road_mask, bridge_surface);
			} else if (way.tags.contains("natural") &&
					   way.tags.get("amenity") !=
							   std::optional<std::string>(std::string("fountain"))) {
				natural::generate_natural(editor, element, args, flood_fill_cache,
						building_footprints, bridge_surface);
			} else if (way.tags.contains("amenity")) {
				amenities::generate_amenities(
						editor, element, args, flood_fill_cache, road_mask);
				if (editor.signage_enabled() && way.tags.get("amenity") == "parking") {
					signage::generate_parking_signage(editor, way, road_mask);
				}
			} else if (way.tags.contains("leisure")) {
				leisure::generate_leisure(editor, way, args, flood_fill_cache,
						building_footprints, bridge_surface);
			} else if (way.tags.contains("barrier")) {
				barriers::generate_barriers(editor, element, bridge_surface);
			} else if (way.tags.contains("waterway")) {
				auto it_val = way.tags.find("waterway");
				if (it_val != way.tags.end() &&
						(it_val->second == "dock" || it_val->second == "riverbank")) {
					// Rust treats dock and legacy riverbank polygons as water areas.
					std::optional<int> surface_level;
					if (still_surfaces.has("way", way.id))
						surface_level = still_surfaces.get("way", way.id);
					water_areas::generate_water_area_from_way(editor, way,
							big_water_field, road_mask,
							tunnel_footprint.is_empty() ? nullptr : &tunnel_footprint,
							bridge_surface, surface_level);
				} else {
					waterways::generate_waterways(editor, way);
				}
			} else if (way.tags.contains("railway")) {
				railways::generate_railways(editor, way, subway_points,
						rail_bridge_internal_endpoints, bridge_outlines, road_mask,
						building_footprints, rail_mask, bridge_structures,
						bridge_surface);
			} else if (way.tags.contains("roller_coaster")) {
				railways::generate_roller_coaster(editor, way);
			} else if (way.tags.contains("aeroway") ||
					   way.tags.contains("area:aeroway")) {
				highways::generate_aeroway(editor, way, args, building_footprints);
			} else if (way.tags.get("service") ==
					   std::optional<std::string>(std::string("siding"))) {
				highways::generate_siding(editor, way, bridge_surface);
			} else if (way.tags.get("tomb") ==
					   std::optional<std::string>(std::string("pyramid"))) {
				historic::generate_pyramid(editor, way, args, flood_fill_cache);
			} else if (way.tags.contains("man_made")) {
				man_made::generate_man_made(editor, element, args);
			} else if (way.tags.contains("power")) {
				power::generate_power(editor, element, building_footprints,
						flood_fill_cache, args.timeout);
				if (editor.signage_enabled() && signage::power_sign(way.tags))
					signage::generate_power_signage(editor, way, road_mask);
			} else if (way.tags.contains("place")) {
				landuse::generate_place(editor, way, args, flood_fill_cache);
			}
		} else if (element.is_node()) {
			auto const &node = element.as_node();
			if (std::find(landmark_plan.suppressed.begin(),
						landmark_plan.suppressed.end(),
						std::pair<std::string, std::uint64_t>{"node", node.id}) !=
					landmark_plan.suppressed.end()) {
				release_finished_fills(flood_fill_cache, expiring_fills, task_index);
				continue;
			}
			// Rust resolves node signage before the feature dispatcher.  Keep
			// this phase ordering so signage sees the same world state.
			if (editor.signage_enabled())
				signage::generate_node_signage(
						editor, node, building_footprints, road_mask);

			if (editor.ground)
				args.ground_level = editor.ground->level(node.xz());

			if (node.tags.contains("door") || node.tags.contains("entrance")) {
				doors::generate_doors(editor, node);
			} else if (node.tags.contains("natural") &&
					   node.tags.get("natural") ==
							   std::optional<std::string>(std::string("tree"))) {
				natural::generate_natural(editor, element, args, flood_fill_cache,
						building_footprints, bridge_surface);
			} else if (node.tags.contains("amenity")) {
				amenities::generate_amenities(
						editor, element, args, flood_fill_cache, road_mask);
			} else if (node.tags.contains("barrier")) {
				barriers::generate_barrier_nodes(editor, node, bridge_surface);
			} else if (node.tags.contains("highway")) {
				if (!highway_connectivity)
					highway_connectivity =
							highways::build_highway_connectivity_map(elements);
				highways::generate_highways_internal(editor, element, args,
						*highway_connectivity, args.timeout, road_mask, bridge_structures,
						bridge_surface, tunnel_portals, marking_index,
						tunnel_footprint.is_empty() ? nullptr : &tunnel_footprint);
			} else if (node.tags.get("aeroway") ==
					   std::optional<std::string>(std::string("helipad"))) {
				highways::generate_helipad_node(editor, node, args, building_footprints);
			} else if (node.tags.contains("tourism")) {
				tourisms::generate_tourisms(editor, node, road_mask);
			} else if (node.tags.contains("man_made")) {
				man_made::generate_man_made_nodes(editor, node, args);
			} else if (node.tags.contains("power")) {
				power::generate_power_nodes(editor, node);
			} else if (node.tags.contains("historic")) {
				historic::generate_historic(editor, node);
			} else if (node.tags.contains("emergency")) {
				emergency::generate_emergency(editor, node);
			} else if (node.tags.contains("advertising")) {
				advertising::generate_advertising(editor, node, road_mask);
			}
		} else if (element.is_relation()) {
			auto const &rel = element.as_relation();
			if (std::find(landmark_plan.suppressed.begin(),
						landmark_plan.suppressed.end(),
						std::pair<std::string, std::uint64_t>{"relation", rel.id}) !=
					landmark_plan.suppressed.end()) {
				release_finished_fills(flood_fill_cache, expiring_fills, task_index);
				continue;
			}

			if (editor.ground && !rel.members.empty() &&
					!rel.members.begin()->way.nodes.empty()) {
				args.ground_level = editor.ground->level(
						rel.members.begin()->way.nodes.begin()->xz());
			}

			bool is_building_relation =
					rel.tags.contains("building") || rel.tags.contains("building:part") ||
					(rel.tags.get("type") ==
							std::optional<std::string>(std::string("building")));

			if (is_building_relation) {
				buildings::generate_building_from_relation(editor, rel, args,
						flood_fill_cache, xzbbox, building_passages,
						interior_use_index ? &*interior_use_index : nullptr);
			} else if (rel.tags.contains("water") ||
					   rel.tags.get("natural") ==
							   std::optional<std::string>(std::string("water")) ||
					   rel.tags.get("natural") ==
							   std::optional<std::string>(std::string("bay"))) {
				std::optional<int> surface_level;
				if (still_surfaces.has("relation", rel.id))
					surface_level = still_surfaces.get("relation", rel.id);
				water_areas::generate_water_areas_from_relation(editor, rel, xzbbox,
						big_water_field, road_mask,
						tunnel_footprint.is_empty() ? nullptr : &tunnel_footprint,
						bridge_surface, surface_level);
			} else if (rel.tags.contains("natural")) {
				natural::generate_natural_from_relation(editor, rel, args,
						flood_fill_cache, building_footprints, bridge_surface);
			} else if (rel.tags.contains("landuse")) {
				landuse::generate_landuse_from_relation(editor, rel, args,
						flood_fill_cache, building_footprints, road_mask, bridge_surface);
			} else if (rel.tags.get("leisure") ==
					   std::optional<std::string>(std::string("park"))) {
				leisure::generate_leisure_from_relation(editor, rel, args,
						flood_fill_cache, building_footprints, bridge_surface);
			} else if (rel.tags.contains("man_made")) {
				man_made::generate_man_made(editor, ProcessedElement(rel), args);
			}
		}

		// Do this after processing: a relation may consume fills belonging to
		// member ways, so eviction before dispatch would cause a refill.
		release_finished_fills(flood_fill_cache, expiring_fills, task_index);
	}
	editor.clear_strict_bounds();
	// Tagged nodes precede ways in OSM input.  Defer address and POI facade
	// plates until every building wall exists; traffic/rail signs stay in the
	// normal node dispatcher because they do not depend on building hosts.
	if (editor.signage_enabled() && !regional_ground_passes) {
		if (tiled_element_dispatch) {
			for (std::size_t tile_index = 0; tile_index < element_tiles.size();
					++tile_index) {
				const auto &tile = element_tiles[tile_index];
				const int min_x = std::max(tile.min_x, xzbbox.min_x());
				const int max_x = std::min(tile.max_x - 1, xzbbox.max_x());
				const int min_z = std::max(tile.min_z, xzbbox.min_z());
				const int max_z = std::min(tile.max_z - 1, xzbbox.max_z());
				if (min_x > max_x || min_z > max_z)
					continue;
				editor.set_strict_bounds(min_x, min_z, max_x, max_z);
				for (const auto element_index : element_assignments[tile_index]) {
					const auto &element = render_elements[element_index];
					if (element.is_node())
						signage::generate_node_facade_signage(editor, element.as_node(),
								building_footprints, road_mask);
				}
			}
			editor.clear_strict_bounds();
		} else {
			for (const auto &element : render_elements)
				if (element.is_node())
					signage::generate_node_facade_signage(
							editor, element.as_node(), building_footprints, road_mask);
		}
	}

	if (regional_ground_passes) {
		if (active_tile != std::numeric_limits<std::size_t>::max()) {
			if (!finish_ground_tile(active_tile)) {
				std::cerr << "Failed to merge generated Arnis tile\n";
				editor.finish_tile_pass();
				return false;
			}
			next_ground_tile = active_tile + 1;
		}
		while (next_ground_tile < element_tiles.size()) {
			if (!begin_generation_tile(next_ground_tile) ||
					!finish_ground_tile(next_ground_tile)) {
				std::cerr << "Failed to generate or merge empty Arnis tile\n";
				editor.finish_tile_pass();
				return false;
			}
			++next_ground_tile;
		}
		editor.finish_tile_pass();
	}
	editor.clear_strict_bounds();
	caves::CaveEllipsoids cave_ellipsoids;
	if (!regional_ground_passes) {
		ground_generation::generate_ground_layer(editor, args, xzbbox,
				building_footprints,
				tunnel_footprint.is_empty() ? nullptr : &tunnel_footprint,
				&bridge_surface);
	}
	if (!regional_ground_passes) {
		if (args.caves) {
			// Rust establishes the deepslate host transition on the filled
			// terrain before any cave carving.  Applying it afterwards leaves
			// carved cells untouched (and changes the host set seen by cave
			// decoration), so keep this phase ahead of both cave passes.
			caves::apply_deepslate(editor, xzbbox.min_x(), xzbbox.max_x(), xzbbox.min_z(),
					xzbbox.max_z());
			std::unordered_set<std::int64_t> cave_air;
			caves::carve_region(editor,
					{xzbbox.min_x(), xzbbox.max_x(), xzbbox.min_z(), xzbbox.max_z()},
					{xzbbox.min_x(), xzbbox.max_x(), xzbbox.min_z(), xzbbox.max_z()},
					CAVE_SEED, world_editor::terrain_floor_y(), &cave_ellipsoids,
					&cave_air);
			std::unordered_set<std::int64_t> cave_basin_fluid;
			std::unordered_set<std::int64_t> cave_water_cells;
			caves::WaterPlan cave_water_plan;
			const caves::CaveRect cave_world{
					xzbbox.min_x(), xzbbox.max_x(), xzbbox.min_z(), xzbbox.max_z()};
			const caves::CaveShapeQuery cave_shape(editor, cave_world, CAVE_SEED,
					world_editor::terrain_floor_y(), cave_ellipsoids);
			caves::generate_water_features(editor, cave_world, cave_world, CAVE_SEED,
					world_editor::terrain_floor_y(), args, cave_shape, &cave_basin_fluid,
					&cave_water_cells, &cave_air, &cave_water_plan);
			caves::stamp_schematics_region(editor, cave_world, cave_air, CAVE_SEED,
					world_editor::terrain_floor_y(), args, &cave_basin_fluid);
			caves::place_ores_region(editor,
					{xzbbox.min_x(), xzbbox.max_x(), xzbbox.min_z(), xzbbox.max_z()},
					CAVE_SEED, world_editor::terrain_floor_y());
			caves::decorate_region(editor, cave_world, cave_world, CAVE_SEED,
					world_editor::terrain_floor_y(), args, cave_shape, cave_water_plan,
					&cave_basin_fluid, &cave_water_cells, &cave_air);
		} else if (args.fillground) {
			ore_generation::generate_ores(editor, xzbbox.min_x(), xzbbox.max_x(),
					xzbbox.min_z(), xzbbox.max_z());
		}
	}

	if (!regional_ground_passes)
		water_depth::carve_lc_water_pass(
				editor, big_water_field, road_mask, tunnel_footprint, &bridge_surface);
	// Rust releases ground-surface protection after the ground/ore/water
	// passes. Models and landmarks own their final footprints.
	editor.release_sealed_surface();
	// Tunnel interiors must be reopened after underground fill, but before
	// models: a late carve can cut holes into an already placed landmark.
	if (!subway_points.empty())
		railways::carve_subway_interior(editor, subway_points);
	if (!highway_tunnel_cells.empty())
		highways::carve_highway_tunnel_interior(editor, highway_tunnel_cells);
	if (args.caves)
		caves::seal_floating_fluid_region(editor,
				{xzbbox.min_x(), xzbbox.max_x(), xzbbox.min_z(), xzbbox.max_z()},
				world_editor::terrain_floor_y());
	if (model_pipeline) {
		models_3d::place_three_dmr_prescan(
				three_dmr_provider, editor, model_pipeline->three_dmr(), args.scale);
		models_3d::place_wikidata_prescan(
				wikidata_provider, editor, model_pipeline->wikidata(), args.scale);
		models_3d::place_custom_models(
				*model_pipeline, custom_model_provider, editor, args.scale);
	}
	// Rust places plane models after the other 3D providers and immediately
	// before landmarks; preserve that ordering for overlap and suppression.
	structures::plane::place_plane_placements(editor, plane_candidates);
	landmarks::place_all(editor, landmark_plan, args.scale);
	structures::scatter_boats(editor, min_x, min_z, max_x, max_z);

	railways::advtrains::finish_network(editor);
	// Resolve preset facade candidates only after all neighbouring building
	// blocks exist; party-wall panels are cropped from their finished rooflines.
	building_facades::finalize(editor);
	// Mark the completed generation for the format-specific persistence layer.
	// Java/Bedrock/Luanti writers consume these lifecycle requests when wired by
	// their respective WorldEditor backends.
	editor.request_flush();
	editor.request_save();
	if (!editor.finalize_persistence())
		return false;

	return true;
}

bool generate_world_with_options(WorldEditor &editor,
		const std::vector<ProcessedElement> &elements, Args args,
		FloodFillCache &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const GenerationOptions &options, bool elements_prepared)
{
	if (!valid_generation_options(options))
		return false;
	apply_generation_options(editor, options);
	args.ground_level = options.ground_level;
	args.bedrock = is_bedrock(options.format);
	args.luanti = is_luanti(options.format);
	args.overture_source = options.overture_source;
	args.world_name = options.world_name;
	if (!options.output_path.empty())
		args.path = options.output_path.string();
	args.scale = options.projection_scale;
	args.map_item = options.map_item;
	args.map_preview = options.map_preview;
	args.use_3d = options.use_3d;
	auto facade_job = options.facades;
	const auto geo = editor.geographic_bounds();
	const bool geographic_bounds_valid = std::isfinite(geo[0]) && std::isfinite(geo[1]) &&
										 std::isfinite(geo[2]) && std::isfinite(geo[3]) &&
										 geo[1] > geo[0] && geo[3] > geo[2];
	if (!facade_job && geographic_bounds_valid && args.mapillary_pipeline_on() &&
			!args.skip_objects() && !args.mapillary_probe) {
		const mapillary::BBox bbox{geo[0], geo[2], geo[1], geo[3]};
		mapillary::PipelineConfig config;
		config.bounds = {bbox.min_lon, bbox.min_lat, bbox.max_lon, bbox.max_lat};
		config.access_token = *args.mapillary_api_token();
		constexpr double pi = 3.14159265358979323846;
		constexpr double earth_radius_m = 6371000.0;
		const double mid_lat = 0.5 * (bbox.min_lat + bbox.max_lat);
		const double height_m =
				(bbox.max_lat - bbox.min_lat) * pi / 180.0 * earth_radius_m;
		const double width_m = (bbox.max_lon - bbox.min_lon) * pi / 180.0 *
							   earth_radius_m * std::cos(mid_lat * pi / 180.0);
		if (std::abs(width_m * height_m) > 100000.0)
			config.cache_only = "area is too large for a cold Mapillary facade run";
		auto overpass_fetcher =
				[](const retrieve_data::OverpassEndpoint &endpoint,
						const std::string &query) -> std::optional<std::string> {
			auto permit = net::request_permit();
			HTTPFetchRequest request;
			request.url = endpoint.url;
			request.method = HTTP_POST;
			request.fields.emplace("data", query);
			request.timeout = static_cast<long>(endpoint.timeout_seconds) * 1000L;
			request.connect_timeout = 10000L;
			request.useragent = retrieve_data::OSM_USER_AGENT;
			request.quiet = true;
			HTTPFetchResult response;
			httpfetch_sync(request, response);
			if (!response.succeeded || response.response_code < 200 ||
					response.response_code >= 300 ||
					response.data.size() > 64 * 1024 * 1024)
				return std::nullopt;
			return std::move(response.data);
		};
		config.osm_fetch =
				mapillary::osm_fetch_config(bbox, args, std::move(overpass_fetcher));
		mapillary::Client client;
		facade_job = std::make_shared<mapillary::FacadeJob>(
				mapillary::FacadeJob::start(std::move(client), std::move(config)));
	}
	return generate_world(editor, elements, args, flood_fill_cache, building_footprints,
			elements_prepared,
			options.prepared_buildings ? &*options.prepared_buildings : nullptr,
			facade_job.get());
}

}
