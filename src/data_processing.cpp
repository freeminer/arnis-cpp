#include "data_processing.h"
#include "biome.h"
#include "fm_datapack_disasters.h"
#include "fm_arnis_bench.h"
#include "element_processing/fm_highway_connectivity.h"
#include "fm_ecoregion_cache.h"
#include "element_processing/signage.h"
#include "element_processing/advtrains.h"
#include "world_style.h"
#include <sys/types.h>
#include <atomic>
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
#include <map>
#include <numeric>
#include <set>
#include <utility>
#include <cstdlib>
#include <fstream>
#include <condition_variable>
#include <mutex>
#include <unordered_map>
#include <thread>
#include <future>
#include <tuple>
#include <stdexcept>
#include <string_view>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#elif defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_host.h>
#include <sys/sysctl.h>
#endif

#include "../../arnis_adapter.h"
#include "../../arnis_ground.h"
#include "assets_root.h"
#include "retrieve_data.h"
#include "bresenham.h"
#include "cache_root.h"
#include "building_suppression.h"
#include "element_processing/historic.h"
#include "element_processing/relation_area.h"
#include "element_processing/aerialways.h"
#include "element_processing/attractions.h"
#include "element_processing/fuel_stations.h"
#include "element_processing/power.h"
#include "element_processing/emergency.h"
#include "element_processing/advertising.h"
#include "element_processing/bridges.h"
#include "element_processing/barriers.h"
#include "element_processing/way_segments.h"
#include "element_processing/highways.h"
#include "element_processing/highway_tunnels.h"
#include "element_processing/road_markings.h"
#include "element_processing/traffic_lights.h"
#include "datapack/extras.h"
#include "element_processing/cars.h"
#include "element_processing/trains.h"
#include "element_processing/rail_track.h"
#include "element_processing/buildings.h"
#include "element_processing/blueprint.h"
#include "element_processing/waterfront.h"
#include "element_processing/tree.h"
#include "element_processing/subprocessor/interior/index.h"
#include "building_facades/registry.h"
#include "ground_decoration.h"
#include "mapillary/atlas.h"
#include "mapillary/credits.h"
#include "mapillary/facades.h"
#include "mapillary/displays.h"
#include "mode_filter.h"
#include "mapillary/pipeline.h"
#include "mapillary/fetch.h"
#include "net.h"
#include "httpfetch.h"
#include "floodfill_cache.h"
#include "ground_generation.h"
#include "projection/projection_spec.h"
#include "ecoregion.h"
#include "land_cover/land_cover.h"
#include "ore_generation.h"
#include "caves_deepslate.h"
#include "caves_carver.h"
#include "caves_ores.h"
#include "caves_water.h"
#include "trees/region.h"
#include "water_depth.h"
#include "water_steps.h"
#include "world_editor/floor_state.h"
#include "progress.h"
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
#include "elevation/cache.h"
#include "one_world.h"
#include "coordinate_system/cartesian/xzbbox/xzbbox_enum.h"
#include "coordinate_system/geographic/llbbox.h"
#include "projection/web_mercator.h"
#include "trees/mapped.h"
#include "models_3d/pipeline.h"
#include "models_3d/placement_executor.h"
#include "models_3d/custom/client.h"
#include "models_3d/three_dmr/client.h"
#include "landmarks.h"
#include "deferred_placement.h"
#include "trees/engine.h"
#include "decals/render.h"
#include "map_item.h"
#include "map_item_palette.h"
#include "world_utils.h"
#include "util/base64.h"
#include "util/png.h"

namespace arnis
{

namespace
{
// Keep the manifest's per-area generator version aligned with the Rust package
// version in Cargo.toml. The C++ library is built independently of Cargo.
constexpr char ARNIS_PACKAGE_VERSION[] = "3.3.0";
one_world::ElevationAffine one_world_elevation(const elevation::ElevationAffine &affine)
{
	return {affine.min_height_m, affine.blocks_per_meter, affine.ground_level,
			affine.soft_top
					? std::optional<one_world::SoftTop>(one_world::SoftTop{
							  affine.soft_top->knee_m, affine.soft_top->width_blocks})
					: std::nullopt};
}

int area_floor_for(const Ground *ground, const Args &args)
{
	if (!ground)
		return args.ground_level;
	int floor = ground->base_level(args.ground_level);
	if (!args.one_world_run || !args.disable_height_limit || !ground->elevation_enabled)
		return floor;
	std::optional<int> lowest;
	for (const auto &row : ground->elevation_grid)
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
	return lowest.value_or(floor);
}

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
			if (loaded) {
				// Rust's TreePackSource::load reports the selected realm, community
				// counts and enabled size tiers when the pack is first loaded. The
				// cached C++ selector reports once at cache population, not on every
				// mapgen worker that reuses it.
				loaded->report();
				result = std::make_shared<arnis::trees::RegionSelector>(
						std::move(*loaded));
			}
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
		const auto &[x1i, z1i] = ring[i - 1];
		const auto &[x2i, z2i] = ring[i];
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
		const auto aeroway = way.tags.get("aeroway");
		const auto area_kind = highways::aeroway_area_kind(way);
		auto ring = [&] {
			std::vector<std::pair<int, int>> points;
			points.reserve(way.nodes.size());
			for (const auto &node : way.nodes)
				points.emplace_back(node.x, node.z);
			return points;
		};
		if (area_kind == highways::AerowayAreaKind::Runway) {
			runway_areas.push_back(ring());
		} else if (area_kind == highways::AerowayAreaKind::Apron) {
			aprons.push_back(ring());
		} else if (aeroway == std::optional<std::string>("runway") ||
				   aeroway == std::optional<std::string>("stopway")) {
			append_aircraft_strip_segments(runways, way, scale);
		} else if (aeroway == std::optional<std::string>("taxiway")) {
			append_aircraft_strip_segments(taxiways, way, scale);
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

#if defined(__APPLE__)
std::optional<std::uint64_t> macos_available_memory_mb()
{
	std::uint32_t level = 0;
	size_t level_size = sizeof(level);
	if (sysctlbyname("kern.memorystatus_level", &level, &level_size, nullptr, 0) != 0 ||
			level_size != sizeof(level) || level == 0 || level > 100)
		return std::nullopt;

	std::uint64_t total = 0;
	size_t total_size = sizeof(total);
	if (sysctlbyname("hw.memsize", &total, &total_size, nullptr, 0) != 0 ||
			total_size != sizeof(total))
		return std::nullopt;

	return (total / 100 * level) / (1024 * 1024);
}
#endif

std::uint64_t available_memory_mb()
{
#if defined(_WIN32)
	MEMORYSTATUSEX memory{};
	memory.dwLength = sizeof(memory);
	return GlobalMemoryStatusEx(&memory) ? memory.ullAvailPhys / (1024 * 1024) : 0;
#elif defined(__APPLE__)
	// Match Rust's kernel memory-pressure estimate. Mach's free/inactive counts
	// omit compressed pages, which can otherwise make the scheduler believe a
	// busy but healthy system has only a few hundred MB available.
	if (const auto available = macos_available_memory_mb())
		return *available;
	vm_statistics64_data_t statistics{};
	mach_msg_type_number_t count = HOST_VM_INFO64_COUNT;
	vm_size_t page_size = 0;
	const auto host = mach_host_self();
	const auto page_status = host_page_size(host, &page_size);
	const auto stats_status =
			page_status == KERN_SUCCESS && page_size > 0
					? host_statistics64(host, HOST_VM_INFO64,
							  reinterpret_cast<host_info64_t>(&statistics), &count)
					: KERN_FAILURE;
	mach_port_deallocate(mach_task_self(), host);
	if (page_status != KERN_SUCCESS || page_size == 0 || stats_status != KERN_SUCCESS)
		return 0;
	return (std::uint64_t(statistics.free_count) + statistics.inactive_count +
				   statistics.speculative_count) *
		   page_size / (1024 * 1024);
#elif defined(__linux__)
	std::ifstream mem("/proc/meminfo");
	std::string key;
	std::uint64_t kb;
	while (mem >> key >> kb) {
		if (key == "MemAvailable:")
			return kb / 1024;
		std::string unit;
		std::getline(mem, unit);
	}
	return 0;
#else
	return 0;
#endif
}

std::uint64_t per_region_estimate_mb(bool fillground)
{
	constexpr std::uint64_t BASE_REGION_MB = 26;
	constexpr int VANILLA_SPAN =
			world_editor::DEFAULT_MAX_Y - world_editor::DEFAULT_MIN_Y;
	const auto span = static_cast<std::uint64_t>(
			std::max(world_editor::terrain_top_y() - world_editor::terrain_floor_y(),
					VANILLA_SPAN));
	return fillground ? BASE_REGION_MB * span / static_cast<std::uint64_t>(VANILLA_SPAN)
					  : BASE_REGION_MB;
}

std::size_t tile_processing_threads(
		std::size_t pool_threads, std::uint64_t available_mb, bool fillground)
{
	pool_threads = std::max<std::size_t>(1, pool_threads);
	if (available_mb == 0)
		return pool_threads;
	constexpr std::uint64_t TILE_RAM_BUDGET_PERCENT = 20;
	constexpr std::uint64_t REGIONS_PER_TILE_ESTIMATE = 2;
	const auto per_tile_mb =
			std::max<std::uint64_t>(1, per_region_estimate_mb(fillground)) *
			REGIONS_PER_TILE_ESTIMATE;
	const auto tile_budget_mb = (available_mb / 100) * TILE_RAM_BUDGET_PERCENT +
								(available_mb % 100) * TILE_RAM_BUDGET_PERCENT / 100;
	const auto memory_bound = std::max<std::uint64_t>(1, tile_budget_mb / per_tile_mb);
	return std::min<std::size_t>(
			pool_threads, memory_bound > std::numeric_limits<std::size_t>::max()
								  ? std::numeric_limits<std::size_t>::max()
								  : static_cast<std::size_t>(memory_bound));
}

} // anonymous namespace

bool should_stream_to_disk(
		std::size_t tile_count, std::uint64_t available_mb, bool fillground)
{
	// Rust streams Java regions by default so peak RAM and output don't depend on
	// the host's current free-memory estimate. Keep the explicit opt-out for
	// callers that prefer to retain the complete world in memory.
	(void)tile_count;
	(void)available_mb;
	(void)fillground;
	const char *setting = std::getenv("ARNIS_STREAM_TO_DISK");
	return !setting || std::string_view(setting) != "0";
}

bool should_stream_to_disk(std::size_t tile_count, bool fillground)
{
	return should_stream_to_disk(tile_count, available_memory_mb(), fillground);
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
			cache.remove_way(way_id);
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
		for (const auto key : {osm_parser::group_key(seed), osm_parser::group_key(id)}) {
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
				record_with_siblings(member.way->id, task_index);
		}
	}
	return result;
}

struct TileFillReaders
{
	std::vector<std::vector<std::uint64_t>> tile_ids;
	std::unordered_map<std::uint64_t, std::size_t> remaining_readers;
};

TileFillReaders compute_tile_fill_readers(
		const std::vector<std::vector<std::size_t>> &tile_assignments,
		const std::vector<ProcessedElement> &elements,
		const std::unordered_map<std::uint64_t, std::uint64_t> &part_groups,
		const std::unordered_map<std::uint64_t, std::vector<std::uint64_t>>
				&group_members,
		const FloodFillCache &cache)
{
	TileFillReaders result;
	result.tile_ids.reserve(tile_assignments.size());
	for (const auto &assigned : tile_assignments) {
		std::vector<std::uint64_t> ids;
		auto add_with_siblings = [&](std::uint64_t id) {
			ids.push_back(id);
			const auto group = part_groups.find(id);
			const auto seed = group == part_groups.end() ? id : group->second;
			for (const auto key :
					{osm_parser::group_key(seed), osm_parser::group_key(id)})
				if (const auto members = group_members.find(key);
						members != group_members.end())
					ids.insert(ids.end(), members->second.begin(), members->second.end());
		};
		for (const auto element_index : assigned) {
			const auto &element = elements[element_index];
			if (element.is_way()) {
				add_with_siblings(element.as_way().id);
			} else if (element.is_relation()) {
				add_with_siblings(element.as_relation().id);
				for (const auto &member : element.as_relation().members)
					add_with_siblings(member.way->id);
			}
		}
		ids.erase(std::remove_if(ids.begin(), ids.end(),
						  [&](std::uint64_t id) { return !cache.contains(id); }),
				ids.end());
		std::sort(ids.begin(), ids.end());
		ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
		for (const auto id : ids)
			++result.remaining_readers[id];
		result.tile_ids.push_back(std::move(ids));
	}
	return result;
}

void release_tile_fills(const std::vector<std::uint64_t> &ids,
		std::unordered_map<std::uint64_t, std::size_t> &remaining_readers,
		FloodFillCache &cache)
{
	for (const auto id : ids) {
		const auto found = remaining_readers.find(id);
		if (found == remaining_readers.end() || found->second == 0)
			continue;
		if (--found->second == 0) {
			cache.release(id);
			remaining_readers.erase(found);
		}
	}
}

namespace
{

bool is_closed_ring(const std::vector<ProcessedNode> &nodes)
{
	return nodes.size() >= 4 && nodes.front().x == nodes.back().x &&
		   nodes.front().z == nodes.back().z;
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
	return element_processing::landuse_paints_ground(tags);
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
		const auto area_kind = element_processing::relation_area(tags);
		const bool draws_ground =
				area_kind == element_processing::RelationArea::Natural ||
				area_kind == element_processing::RelationArea::Park ||
				(area_kind == element_processing::RelationArea::Landuse &&
						element_processing::landuse_paints_ground(tags));
		if (!draws_ground)
			return std::nullopt;
		double outer_area = 0.0;
		double inner_area = 0.0;
		for (const auto &member : rel.members) {
			if (member.role == ProcessedMemberRole::Outer)
				outer_area += ring_area(member.way->nodes);
			else if (member.role == ProcessedMemberRole::Inner)
				inner_area += ring_area(member.way->nodes);
		}
		area = outer_area > inner_area ? outer_area - inner_area : outer_area;
	} else {
		return std::nullopt;
	}
	return area > 0.0 ? std::optional<double>(area) : std::nullopt;
}

std::vector<size_t> ground_fill_area_order(const std::vector<ProcessedElement> &elements)
{
	std::vector<size_t> order(elements.size());
	std::iota(order.begin(), order.end(), 0);
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
		return order;

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

	// At each ground-fill slot, select the source element in ascending-area order.
	for (size_t k = 0; k < count; ++k)
		order[slots[k]] = slots[src[k]];
	return order;
}

void sort_ground_fill_areas(std::vector<ProcessedElement> &elements)
{
	auto order = ground_fill_area_order(elements);
	// ground_fill_area_order maps each destination slot to the source element
	// that belongs there. To apply that permutation by swaps, invert it first:
	// Rust's sort_ground_fill_areas stores this source-to-destination mapping
	// (`dest`) and swaps only while an element has not reached its destination.
	std::vector<size_t> dest(order.size());
	for (size_t destination = 0; destination < order.size(); ++destination)
		dest[order[destination]] = destination;
	for (size_t k = 0; k < order.size(); ++k)
		while (dest[k] != k) {
			const size_t target = dest[k];
			std::swap(elements[k], elements[target]);
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
				if (member.way->nodes.empty())
					continue;
				if (member.role == ProcessedMemberRole::Inner)
					inners.push_back(member.way->nodes);
				else if (member.role == ProcessedMemberRole::Outer)
					outers.push_back(member.way->nodes);
			}
			merge_way_segments(outers);
			merge_way_segments(inners);
			for (const auto &[members, cutting] :
					element_processing::hole_groups(outers, inners)) {
				std::vector<std::vector<ProcessedNode>> group_inners;
				group_inners.reserve(cutting.size());
				for (const auto index : cutting)
					group_inners.push_back(inners[index]);
				for (const auto index : members)
					if (auto clipped = clipping::clip_water_ring_to_bbox(
								outers[index], xzbbox))
						mark_polygon(lc, xzbbox, *clipped, group_inners);
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

void sort_elements_by_priority(std::vector<ProcessedElement> &elements)
{
	// Match the Rust GUI dispatch order: landuse is applied after other
	// features; the remaining elements follow osm_parser::get_priority().
	std::stable_sort(elements.begin(), elements.end(),
			[](const ProcessedElement &a, const ProcessedElement &b) {
				constexpr std::size_t PRIORITY_COUNT = 6;
				const bool a_landuse = osm_parser::get_priority(a) >= PRIORITY_COUNT &&
									   a.tags().contains("landuse");
				const bool b_landuse = osm_parser::get_priority(b) >= PRIORITY_COUNT &&
									   b.tags().contains("landuse");
				if (a_landuse != b_landuse)
					return !a_landuse;
				return osm_parser::get_priority(a) < osm_parser::get_priority(b);
			});
}

void prepare_elements_for_generation(std::vector<ProcessedElement> &elements)
{
	sort_elements_by_priority(elements);
	// Nested ground areas then render smallest-first, matching Rust's later pass.
	sort_ground_fill_areas(elements);
}

std::shared_ptr<mapillary::FacadeJob> start_mapillary_facade_job(
		const Args &args, const std::array<double, 4> &geo)
{
	mapillary::credits::reset();
	const bool geographic_bounds_valid =
			std::all_of(geo.begin(), geo.end(),
					[](double value) { return std::isfinite(value); }) &&
			geo[1] > geo[0] && geo[3] > geo[2];
	if (!geographic_bounds_valid || !args.mapillary_pipeline_on() || args.skip_osm() ||
			args.mapillary_probe)
		return {};

	const mapillary::BBox bbox{geo[0], geo[2], geo[1], geo[3]};
	mapillary::PipelineConfig config;
	config.bounds = {bbox.min_lon, bbox.min_lat, bbox.max_lon, bbox.max_lat};
	config.access_token = *args.mapillary_api_token();
	constexpr double pi = 3.14159265358979323846;
	constexpr double earth_radius_m = 6371000.0;
	const double mid_lat = 0.5 * (bbox.min_lat + bbox.max_lat);
	const double height_m = (bbox.max_lat - bbox.min_lat) * pi / 180.0 * earth_radius_m;
	const double width_m = (bbox.max_lon - bbox.min_lon) * pi / 180.0 * earth_radius_m *
						   std::cos(mid_lat * pi / 180.0);
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
				response.response_code >= 300 || response.data.size() > 64 * 1024 * 1024)
			return std::nullopt;
		return std::move(response.data);
	};
	config.osm_fetch =
			mapillary::osm_fetch_config(bbox, args, std::move(overpass_fetcher));
	mapillary::Client client;
	return std::make_shared<mapillary::FacadeJob>(
			mapillary::FacadeJob::start(std::move(client), std::move(config)));
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
		const interior_uses::InteriorUseIndex *interior_index,
		const RoadMaskBitmap *road_mask,
		const BuildingFootprintBitmap *building_footprints,
		const std::unordered_map<std::uint64_t, std::vector<std::uint64_t>>
				*group_members);
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
		const FloodFillCache *flood_fill_cache,
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
void generate_aeroway(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const CoordinateBitmap &building_footprints,
		const FloodFillCache &flood_fill_cache);
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
void generate_roller_coaster(WorldEditor &editor, const ProcessedWay &way);
void generate_railways(WorldEditor &editor, const ProcessedWay &element);
void generate_railways(WorldEditor &editor, const ProcessedWay &element,
		std::vector<std::tuple<int, int, int, bool>> &rail_tunnel_points,
		const bridge_styles::BridgeOutlineIndex &bridge_outlines,
		const CoordinateBitmap &road_mask, const CoordinateBitmap &building_footprints,
		const CoordinateBitmap &rail_mask,
		const bridges::BridgeStructureMap &bridge_structures,
		const bridges::BridgeSurfaceMap &bridge_surface,
		const railways::RailJoins &rail_joins);
CoordinateBitmap collect_at_grade_rail_mask(
		const std::vector<ProcessedElement> &elements, const XZBBox &xzbbox);
void add_tunnel_footprint(const std::vector<ProcessedElement> &elements,
		const XZBBox &xzbbox, CoordinateBitmap &footprint);
std::vector<highways::TunnelLight> carve_rail_tunnel_interior(WorldEditor &editor,
		const std::vector<std::tuple<int, int, int, bool>> &rail_tunnel_points);
}

namespace tourisms
{
void generate_tourisms(
		WorldEditor &editor, const ProcessedNode &node, const RoadMaskBitmap &road_mask);
}

namespace man_made
{
bool is_summit_cross(const tags_t &tags);
std::size_t drop_doubled_summit_crosses(
		std::vector<ProcessedElement> &elements, double scale);
std::vector<std::pair<int, int>> mapped_crane_points(
		const std::vector<ProcessedElement> &elements);
void generate_summit_cross(
		WorldEditor &editor, const ProcessedNode &node, const Args &args);
void generate_man_made(WorldEditor &editor, const ProcessedElement &element,
		const Args &args, const FloodFillCache &flood_fill_cache,
		const CoordinateBitmap &building_footprints);
void generate_man_made_nodes(WorldEditor &editor, const ProcessedNode &node);
void generate_man_made_nodes(WorldEditor &editor, const ProcessedNode &node,
		const Args &args, const CoordinateBitmap &building_footprints);
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
		BuildingFootprintBitmap const &input_building_footprints, bool elements_prepared,
		const PreparedBuildingData *prepared_buildings, mapillary::FacadeJob *facade_job,
		std::optional<std::pair<int, int>> spawn_point)
{
	// Rust starts this best-effort sweep from both CLI and GUI entry points.
	// The C++ library has a single generation boundary, so do it here using
	// the caller-configured Arnis cache root; its daily stamp makes repeated
	// mapchunk/world requests cheap.
	elevation::spawn_throttled_cleanup(cache::base_directory());

	Args effective_args = args_;
	effective_args.apply_mode_defaults();
	effective_args.apply_body_defaults();
	effective_args.apply_feature_needs();
	// Rust no longer exposes a roof-generation toggle: roof handling is always
	// part of building generation. Keep the legacy C++ field for source
	// compatibility, but normalize it at the pipeline boundary.
	effective_args.roof = true;
	const Args &args = effective_args;
	Bench bench(args.benchmark);
	// Rust's Args::skip_osm() boundary replaces the parsed OSM payload with an
	// empty OsmData value. Enforce that here too, since library callers may pass
	// prepared elements, footprints, or facade work independently of the parser.
	static const std::vector<ProcessedElement> no_osm_elements;
	const bool skip_osm = args.skip_osm();
	const auto &generation_input = skip_osm ? no_osm_elements : input_elements;
	static const BuildingFootprintBitmap no_building_footprints;
	const auto &generation_footprints =
			skip_osm ? no_building_footprints : input_building_footprints;
	const PreparedBuildingData *generation_prepared_buildings =
			skip_osm ? nullptr : prepared_buildings;
	if (skip_osm && facade_job) {
		facade_job->cancel();
		facade_job = nullptr;
	}
	// Rust removes buildings covered by aircraft pavement before any pass that
	// derives building-dependent state (facade projection, tree masks, indices).
	// Keep one filtered view for the rest of this generation call.
	std::vector<ProcessedElement> aircraft_filtered_elements;
	std::size_t dropped_aircraft_buildings = drop_buildings_on_aircraft_pavement(
			generation_input, aircraft_filtered_elements, args.scale);
	const std::vector<ProcessedElement> *elements_source = &generation_input;
	if (dropped_aircraft_buildings > 0) {
		std::cout << "  Skipped " << dropped_aircraft_buildings
				  << " building(s) on runways, taxiways and aprons\n";
		elements_source = &aircraft_filtered_elements;
	}
	// A prestarted FacadeJob resets before its worker can record credits. Direct
	// generation without a job still starts with fresh attribution.
	if (!facade_job)
		mapillary::credits::reset();
	// The facade output/cache path is allowed to be new. Create it before the
	// side-effect-free Args validator checks the supplied directory, otherwise
	// validation rejects the path before the existing setup can create it.
	if (args.building_facades && args.building_facades_dir) {
		std::error_code create_error;
		std::filesystem::create_directories(*args.building_facades_dir, create_error);
		if (create_error) {
			const auto message = "cannot create building facade directory: " +
								 *args.building_facades_dir + ": " +
								 create_error.message();
			editor.report_generation_error(message);
			std::cerr << "Arnis generation error: " << message << '\n';
			return false;
		}
	}
	if (const auto validation_error = args.validation_error()) {
		editor.report_generation_error("options rejected: " + *validation_error);
		std::cerr << "Arnis generation options rejected: " << *validation_error << '\n';
		return false;
	}
	if (args.path && !args.path->empty()) {
		const auto geo = editor.geographic_bounds();
		const geographic::LLBBox bbox(geo[0], geo[2], geo[1], geo[3]);
		const WorldFormat format = args.bedrock	 ? WorldFormat::BedrockMcWorld
								   : args.luanti ? WorldFormat::LuantiWorld
												 : WorldFormat::JavaAnvil;
		world_utils::RunSizeError size_error;
		if (!world_utils::check_run_size(
					args, bbox, format, std::filesystem::path(*args.path), &size_error)) {
			editor.report_generation_error("run refused: " + size_error.detail);
			std::cerr << "Earth: Arnis generation refused: " << size_error.detail << '\n';
			return false;
		}
	}
	// Rust holds Minecraft's session.lock across every write into a One World.
	// Keep the guard alive until persistence and manifest recording both finish.
	std::unique_ptr<world_utils::SessionLock> one_world_lock;
	if (args.one_world_run && !args.one_world_run->session_lock) {
		std::string lock_error;
		one_world_lock = world_utils::SessionLock::acquire(
				args.one_world_run->world_dir, &lock_error);
		if (!one_world_lock) {
			const auto message = "cannot acquire One World session lock: " + lock_error;
			editor.report_generation_error(message);
			std::cerr << "Arnis generation error: " << message << '\n';
			return false;
		}
	}
	const bool java_world =
			editor.get_generation_format() == static_cast<int>(WorldFormat::JavaAnvil);
	const bool extending_world = args.one_world_run && args.one_world_run->extending;
	const auto java_world_path =
			args.one_world_run ? args.one_world_run->world_dir : editor.output_path;
	if (java_world && args.world_type == WorldType::Void && !extending_world &&
			!java_world_path.empty())
		world_utils::remove_untouched_template_region(java_world_path);
	// Rust uses the same 3D toggle for bundled schematic props and model
	// placements.  Direct callers of generate_world do not pass through the
	// GenerationOptions adapter, so set the editor policy here as well.
	editor.set_place_schematics(args.use_3d);
	// A One World extension keeps the first area's level.dat settings intact;
	// standalone/new worlds receive this run's requested values.
	if (!(args.one_world_run && args.one_world_run->extending))
		editor.set_game_settings(args.gamemode, args.world_time);
	editor.set_projection_info(projection::to_string(args.projection), args.scale);
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
			if (std::filesystem::is_regular_file(candidate / "cave_pack.json")) {
				editor.set_cave_asset_root(candidate);
				break;
			}
	}
	// Keep direct generation calls consistent with Rust's editor setup. Decals
	// are Java-only; the editor still receives the requested map-item policy,
	// while the format-specific exporter decides whether to emit it.
	const bool java_format = !args.bedrock && !args.luanti;
	// Rust only enables Voxy-driven baked lighting for Java Anvil worlds.
	// Keep the same format gate for direct C++ callers as well.
	editor.set_bake_lighting(args.bake_lighting || (args.voxy_lod && java_format));
	if (!(args.one_world_run && args.one_world_run->extending))
		editor.set_start_with_map(args.map_item);
	editor.set_map_decals(java_format);
	int first_decal_map_id = decals::DecalRegistry::FIRST_ID;
	if (java_format && args.one_world_run) {
		const auto &run = *args.one_world_run;
		const auto maps_dir =
				world_utils::WorldLayout::of(run.world_dir).maps_dir(run.world_dir);
		first_decal_map_id = map_item::next_map_id(maps_dir);
		if (!run.extending)
			first_decal_map_id += args.map_item ? 2 : 1;
	}
	editor.set_first_decal_map_id(first_decal_map_id);
	editor.clear_facade_panels();
	mapillary::atlas::set_atlas_side(facade_atlas_side(args.facade_detail));
	std::optional<std::filesystem::path> facade_directory =
			args.building_facades_dir
					? std::optional<std::filesystem::path>(*args.building_facades_dir)
					: std::nullopt;
	if (!facade_directory && args.building_facades) {
		// Rust's resolver accepts a replacement facade pack from the environment
		// before searching the packaged assets. Keep an invalid override from
		// masking the usable packaged set, matching manifest::resolve().
		if (const char *override_dir = std::getenv("ARNIS_BUILDING_FACADES_DIR");
				override_dir && *override_dir) {
			const std::filesystem::path candidate(override_dir);
			if (std::filesystem::is_regular_file(candidate / "manifest.json"))
				facade_directory = candidate;
		}
	}
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
	if (!skip_osm && args.mapillary_facades != std::optional<bool>(false) &&
			args.mapillary_facades_dir) {
		std::string facade_error;
		if (auto export_data = mapillary::facades::load_export(
					*args.mapillary_facades_dir, &facade_error))
			mapillary::facades::install_export(std::move(*export_data));
	}
	if (facade_job && (args.mapillary_facades == std::optional<bool>(false) ||
							  args.mapillary_facades_dir))
		facade_job->cancel();
	// Rust creates each WorldEditor with the area's XZ origin. C++ receives an
	// editor from its host, so establish the same frame here instead of relying
	// on the host to have called set_ground_origin(). Land-cover sampling,
	// including the bounded water pass, uses coordinates relative to this origin.
	const auto [ground_origin_x, ground_origin_z] = editor.get_min_coords();
	editor.set_ground_origin(ground_origin_x, ground_origin_z);
	editor.reserve_ground_level_cache();
	const bool extended_world_height = args.disable_height_limit ||
									   (editor.ground && editor.ground->extended_ceiling);
	if (extended_world_height && !args.disable_height_limit) {
		const auto geo = editor.geographic_bounds();
		const geographic::LLBBox bbox(geo[0], geo[2], geo[1], geo[3]);
		const auto world_path = args.one_world_run
										? args.one_world_run->world_dir
										: (args.path ? std::filesystem::path(*args.path)
													 : editor.output_path);
		if (!world_path.empty()) {
			std::string disk_error;
			if (!world_utils::check_disk_for_extended_height(args, bbox,
						static_cast<WorldFormat>(editor.get_generation_format()),
						world_path, &disk_error)) {
				editor.report_generation_error("run refused: " + disk_error);
				std::cerr << "Earth: Arnis generation refused: " << disk_error << '\n';
				return false;
			}
		}
	}
	Args world_height_args = args;
	world_height_args.disable_height_limit = extended_world_height;
	world_editor::set_world_bounds(
			extended_min_y_for(world_height_args), world_top_y_for(world_height_args));
	if (editor.ground) {
		editor.ground->set_celestial_body(args.body);
		editor.ground->set_winter(args.winter);
		editor.ground->set_extended_ceiling(
				extended_world_height &&
				extended_max_y_for(args) > world_editor::DEFAULT_MAX_Y);
	}
	int area_floor = area_floor_for(editor.ground, args);
	world_editor::set_terrain_floor_y(area_floor);
	world_editor::set_base_chunk_y(area_floor);
	// Match Rust's filler palette for out-of-bounds chunks on non-Earth bodies.
	const Block filler =
			args.body == CelestialBody::Moon
					? ANDESITE
					: (args.body == CelestialBody::Mars ? RED_TERRACOTTA : GRASS_BLOCK);
	world_editor::set_base_chunk_block_id(filler.id());
	static const std::vector<ProcessedElement> no_elements;
	const auto &source_elements =
			args.skip_objects_due_to_scale() ? no_elements : *elements_source;
	// Preserve the full priority-ordered element set for the Rust-equivalent
	// OSM water/land/bridge ground passes. Mode filtering affects object
	// placement, but Rust applies those ground overrides before filtering.
	std::vector<ProcessedElement> ordered_source_elements = source_elements;
	// Rust drops pier paths and duplicate mapped summit crosses before it
	// derives ground overrides, mode-specific objects, footprints, or indexes.
	// Apply both prepasses to the shared ordered input so every later view agrees.
	const auto dropped_pier_paths = waterfront::drop_paths_on_piers(
			ordered_source_elements, args.scale, flood_fill_cache, args.timeout);
	if (dropped_pier_paths > 0)
		std::cout << "  Replaced " << dropped_pier_paths
				  << " footpath(s) with mapped pier decks\n";
	man_made::drop_doubled_summit_crosses(ordered_source_elements, args.scale);
	if (!elements_prepared)
		sort_elements_by_priority(ordered_source_elements);
	// Prepare Rust-equivalent building suppression and part groups before any pass
	// builds masks, footprints, or indexes from OSM tags.
	std::optional<PreparedBuildingData> computed_buildings;
	if (!generation_prepared_buildings || dropped_aircraft_buildings > 0)
		computed_buildings = prepare_building_data(ordered_source_elements);
	const PreparedBuildingData &building_data =
			generation_prepared_buildings && dropped_aircraft_buildings == 0
					? *generation_prepared_buildings
					: *computed_buildings;
	const std::vector<ProcessedElement> &land_cover_elements = ordered_source_elements;
	std::vector<ProcessedElement> mode_filtered_elements = land_cover_elements;
	mode_filter::retain_for_mode(mode_filtered_elements, args.mode);
	// Prepared host input already has Rust priority ordering; direct callers now
	// retain that ordering through the full and mode-filtered views above.
	const auto &elements = mode_filtered_elements;
	// Build the authored-trunk index once per generation.  Ground decoration
	// can then suppress only procedural crowns overlapping mapped trees, while
	// preserving deterministic results for streamed tiles.
	auto mapped_trunks = trees::mapped::MappedTrunks::collect(elements, args.scale);
	if (editor.ground && editor.ground->has_canopy()) {
		const int crown_radius = crown_probe_blocks(args.scale);
		mapped_trunks.measure_typical_height(
				[&editor, crown_radius](int x, int z) -> std::optional<std::uint8_t> {
					std::optional<std::uint8_t> top;
					for (int dz = -crown_radius; dz <= crown_radius; ++dz)
						for (int dx = -crown_radius; dx <= crown_radius; ++dx)
							if (const auto height = editor.ground->canopy_height_m(
										editor.ground_point(x + dx, z + dz));
									height && *height >= canopy::CANOPY_MIN_M)
								top = std::max(top, height);
					return top;
				});
	}
	editor.mapped_trunks =
			std::make_shared<const trees::mapped::MappedTrunks>(std::move(mapped_trunks));
	editor.mapped_cranes = std::make_shared<const std::vector<std::pair<int, int>>>(
			man_made::mapped_crane_points(elements));
	const auto geo = editor.geographic_bounds();
	const double centre_lat = (geo[0] + geo[1]) * .5, centre_lon = (geo[2] + geo[3]) * .5;
	auto [min_x, min_z] = editor.get_min_coords();
	auto [max_x, max_z] = editor.get_max_coords();
	::XZBBox xzbbox(min_x, min_z, max_x, max_z);
	editor.set_generation_bounds(
			xzbbox.min_x(), xzbbox.min_z(), xzbbox.max_x(), xzbbox.max_z());
	// Rust filters aircraft-pavement buildings before deriving its footprint
	// bitmap. The Freeminer host normally supplies an already-filtered bitmap,
	// but direct library callers may pass one derived from the unfiltered input.
	// Rebuild only in that case so removed airport buildings do not continue to
	// suppress trees, ground decoration, or other footprint-aware features.
	std::optional<BuildingFootprintBitmap> filtered_building_footprints;
	if (dropped_aircraft_buildings > 0)
		filtered_building_footprints.emplace(
				flood_fill_cache.collect_building_footprints(elements, xzbbox));
	const BuildingFootprintBitmap &building_footprints =
			filtered_building_footprints ? *filtered_building_footprints
										 : generation_footprints;
	// Persist the elevation mapping before terrain or structures are written so
	// a later failed generation cannot leave chunks using an unrecorded mapping.
	if (args.one_world_run && editor.ground) {
		if (const auto affine = editor.ground->elevation_affine()) {
			std::string error;
			if (!one_world::remember_elevation(
						*args.one_world_run, one_world_elevation(*affine), &error)) {
				const auto message = "could not remember One World elevation: " + error;
				editor.report_generation_error(message);
				std::cerr << "Arnis generation error: " << message << '\n';
				return false;
			}
		}
	}
	if (editor.ground) {
		// Rust derives the snow threshold from the generated area's bbox center,
		// including for One World extensions.
		const geographic::LLBBox geographic_bbox(geo[0], geo[2], geo[1], geo[3]);
		const auto ground_frame = GroundFrame::from_args(args, geographic_bbox);
		const double snow_line_lat = ground_frame.anchor_lat(geographic_bbox);
		editor.ground->set_snow_line_for_latitude(snow_line_lat);
	}
	// Rust loads the global ecoregion raster as a source, then builds and keeps
	// a bbox-aligned EcoMap on Ground so vegetation can query it per column.
	// Freeminer may not have run Rust's Ground constructor, so seed the same
	// source map here before materializing the local map below.
	if (editor.ground && args.body == CelestialBody::Earth &&
			(!editor.ground->ecoregion_map() ||
					!editor.ground->ecoregion_map()->is_local())) {
		if (const auto &map = ecoregion::generation_map().map)
			editor.ground->set_ecoregion_map(std::move(*map));
	}
	editor.set_regional_tree_placer({});
	editor.set_mapped_regional_tree_placer({});
	editor.tree_crown_columns = {};
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
			// Freeminer may attach the immutable world raster instead of Rust's
			// area-local map. Build the same coarse summary over this generation
			// bbox; using a global summary makes every chunk load every realm's
			// communities and picks the first realm in the entire raster.
			const auto &source = ecoregion::generation_map().map;
			const auto width = std::int64_t(xzbbox.max_x()) - xzbbox.min_x() + 1;
			const auto height = std::int64_t(xzbbox.max_z()) - xzbbox.min_z() + 1;
			if (source && editor.mg && width > 0 && height > 0 &&
					width <= std::numeric_limits<std::int32_t>::max() &&
					height <= std::numeric_limits<std::int32_t>::max()) {
				const auto min_x = xzbbox.min_x();
				const auto min_z = xzbbox.min_z();
				auto area_map = ecoregion::EcoMap::build(static_cast<std::size_t>(width),
						static_cast<std::size_t>(height), {min_x, min_z},
						[&editor, min_x, min_z](
								double x, double z) -> std::pair<double, double> {
							const auto ll = editor.mg->pos_to_ll(
									static_cast<int>(std::floor(min_x + x)),
									static_cast<int>(std::floor(min_z + z)));
							return {static_cast<double>(ll.lat),
									static_cast<double>(ll.lon)};
						});
				if (area_map) {
					ecoregion_map_has_gaps = area_map->has_gaps();
					for (const auto &[id, count] : area_map->by_area()) {
						(void)count;
						if (!ecoregion::tree_mix(id))
							continue;
						if (!dominant_tree_realm)
							if (const auto mix = ecoregion::tree_mix(id))
								dominant_tree_realm = std::string(mix->first);
						area_ecoregion_ids.push_back(id);
					}
					// Rust Ground keeps the bbox-aligned EcoMap, not the global source
					// raster: Ground::ecoregion_at only indexes local maps. Retain the
					// same local map used for the area summary so vegetation and other
					// ground consumers can resolve per-cell ecoregions as well.
					editor.ground->set_ecoregion_map(std::move(*area_map));
				}
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
				area_floor, args.max_tree_size,
				editor.ground ? editor.ground->elevation_blocks_per_meter : 0.0,
				area_ecoregion_ids, exclude_palms);
		if (selector)
			shared_selector = selector;
		if (shared_selector) {
			editor.set_tree_slot_spacing(shared_selector->slot_spacing(true));
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
		// Rust anchors every catalogue entry from its canonical lat/lon, whether
		// or not the corresponding OSM feature survived clipping or was returned
		// by Overpass.  The Freeminer adapter already owns the authoritative
		// projection used to turn those same coordinates into ProcessedNodes.
		// Prefer it here so suppression and placement use the identical world
		// frame instead of a centroid that moves when a way is clipped.
		if (editor.mg) {
			const auto position =
					editor.mg->ll_to_pos({landmark.latitude, landmark.longitude});
			landmark_anchors.push_back({landmark.qid, static_cast<int>(position.X),
					static_cast<int>(position.Y)});
			continue;
		}
		// Standalone library callers without a mapgen projection retain the
		// geometry-based fallback; embedded Freeminer generation takes the
		// canonical geographic-anchor path above.
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
					for (const auto &n : m.way->nodes)
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
	// Rust disables both landmark suppression and placement when model objects
	// are off; otherwise a no-3D run could still discard the OSM feature that a
	// bundled schematic would have replaced.
	const auto [landmark_min_x, landmark_min_z] = editor.get_min_coords();
	const auto [landmark_max_x, landmark_max_z] = editor.get_max_coords();
	auto landmark_plan = args.use_3d && !args.skip_objects()
								 ? landmarks::prescan(elements, landmark_anchors,
										   {landmark_min_x, landmark_min_z,
												   landmark_max_x, landmark_max_z},
										   args.scale, args.rotation)
								 : landmarks::PrescanResult{};
	// Match Rust's Models3dPipeline::prescan: a Wikidata or stadium feature is
	// suppressed only after its model is fetchable. The providers memoize/cache
	// successful fetches, so placement below reuses these assets rather than
	// downloading them a second time.
	const auto model_pipeline =
			args.use_3d
					? std::optional<models_3d::Models3dPipeline>(
							  models_3d::Models3dPipeline::prescan_fetchable_models(
									  elements, args.scale,
									  [&](const std::string &qid) {
										  return wikidata_provider.fetch(qid).has_value();
									  },
									  [&] {
										  return custom_model_provider.fetch("stadium")
												  .has_value();
									  },
									  args.rotation, landmark_plan.suppressed))
					: std::nullopt;
	const auto plane_candidates =
			args.use_3d ? structures::plane::prescan_placements(elements, args.scale)
						: std::vector<structures::plane::Placement>{};
	const auto projection_spec = projection::ProjectionSpec::from_args(args);
	const cartesian::XZBBoxRect padded_clip =
			projection_spec.clip_bbox({{min_x, min_z}, {max_x, max_z}});
	const ::XZBBox clip_bbox(padded_clip.min().x, padded_clip.min().z,
			padded_clip.max().x, padded_clip.max().z);
	std::optional<interior_uses::InteriorUseIndex> interior_use_index;
	if (args.interior)
		interior_use_index.emplace(
				interior_uses::InteriorUseIndex::build(elements, clip_bbox));
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
	const auto facade_attribution_dir =
			args.mapillary_facades != std::optional<bool>(false) &&
							args.mapillary_facades_dir
					? std::optional<std::filesystem::path>(*args.mapillary_facades_dir)
					: std::nullopt;
	mapillary::facades::project_export(
			elements, xzbbox, args.scale, facade_attribution_dir);
	building_facades::set_world_extent(
			xzbbox.min_x(), xzbbox.min_z(), xzbbox.max_x(), xzbbox.max_z());
	const auto ground_geographic = editor.geographic_bounds();
	const geographic::LLBBox ground_bbox(ground_geographic[0], ground_geographic[2],
			ground_geographic[1], ground_geographic[3]);
	const auto ground_frame = GroundFrame::from_args(args, ground_bbox);
	const auto ground_fetch_plan = ground_frame.fetch_plan(ground_bbox, args.scale);
	const auto grid_width = std::get<2>(ground_fetch_plan.dims);
	const auto grid_height = std::get<3>(ground_fetch_plan.dims);
	const auto &[final_width, final_height] = ground_fetch_plan.final_dims;
	const auto &pad = ground_fetch_plan.pad_blocks;
	const land_cover::GeographicBounds ground_fetch_bounds{
			ground_fetch_plan.bbox.min().lat(), ground_fetch_plan.bbox.min().lng(),
			ground_fetch_plan.bbox.max().lat(), ground_fetch_plan.bbox.max().lng()};
	const bool fetch_land_cover = editor.ground && args.body == CelestialBody::Earth &&
								  !editor.ground->has_land_cover();
	const bool fetch_canopy = editor.ground && args.canopy_height &&
							  args.body == CelestialBody::Earth &&
							  !editor.ground->has_canopy();
	std::optional<std::future<land_cover::LandCoverData>> land_cover_job;
	std::optional<std::future<std::optional<canopy::CanopyData>>> canopy_job;
	// Rust starts the independent Earth rasters together. Keep the host adapter's
	// fetches overlapped too, then install each completed grid on this generation's
	// Ground before the elevation-dependent surface passes begin.
	if (fetch_land_cover) {
		const auto bounds = ground_fetch_bounds;
		const auto width = grid_width;
		const auto height = grid_height;
		land_cover_job.emplace(std::async(std::launch::async, [bounds, width, height] {
			return land_cover::fetch_land_cover_data(bounds, width, height);
		}));
	}
	if (fetch_canopy) {
		const std::array<double, 4> bounds{ground_fetch_plan.bbox.min().lat(),
				ground_fetch_plan.bbox.max().lat(), ground_fetch_plan.bbox.min().lng(),
				ground_fetch_plan.bbox.max().lng()};
		const auto width = grid_width;
		const auto height = grid_height;
		const auto cache_root = cache::provider_cache_root("canopy");
		canopy_job.emplace(
				std::async(std::launch::async, [bounds, width, height, cache_root] {
					return canopy::fetch_canopy_data(cache_root, bounds[0], bounds[2],
							bounds[1], bounds[3], width, height);
				}));
	}
	if (land_cover_job) {
		// Match Ground::new_enabled: land-cover samples use the same world/grid
		// dimensions as elevation. One World additionally fetches a padded,
		// Mercator-row-remapped source grid and crops to the generated rectangle.
		auto land_cover = land_cover_job->get();
		if (land_cover.width == 0 || land_cover.height == 0) {
			land_cover = build_osm_water_land_cover(elements, xzbbox);
		} else {
			if (ground_frame.mercator)
				land_cover.remap_rows_to_mercator(ground_fetch_plan.bbox.max().lat(),
						ground_fetch_plan.bbox.min().lat());
			if (pad > 0)
				land_cover.crop(pad, pad, final_width, final_height);
		}
		if (land_cover.width > 0 && land_cover.height > 0) {
			editor.ground->set_land_cover_data(
					std::move(land_cover), final_width, final_height);
		}
	}
	// Rust fetches canopy alongside land cover and only enables it for Earth.
	// Keep the acquisition optional for library hosts: a missing canopy tile
	// must not prevent terrain generation, while a successful raster is shared
	// by tree placement and the natural-surface pass.
	if (canopy_job) {
		if (auto canopy = canopy_job->get()) {
			if (ground_frame.mercator)
				canopy->remap_rows_to_mercator(ground_fetch_plan.bbox.max().lat(),
						ground_fetch_plan.bbox.min().lat());
			if (pad > 0)
				canopy->crop(pad, pad, final_width, final_height);
			editor.ground->set_canopy_data(std::move(*canopy), final_width, final_height);
		}
	}
	if (editor.ground && args.body == CelestialBody::Earth &&
			editor.ground->has_land_cover()) {
		// These are elevation-mutating passes in Rust. Run them through Ground so
		// water snapping and bridge repair update the authoritative DEM, rather
		// than a rounded land-cover-sized sample that is discarded afterwards.
		editor.ground->apply_osm_water_override(land_cover_elements, xzbbox);
		editor.ground->apply_osm_land_override(land_cover_elements, xzbbox, args.scale);
		if (args.debug)
			editor.ground->save_land_cover_debug_image("landcover_debug_post_osm_water");
		editor.ground->apply_bridge_land_cover_repair(
				land_cover_elements, xzbbox, args.scale);
		editor.ground->finish_land_cover(args.mode);
		// Rust applies OSM water/land and bridge repairs before area_floor_for;
		// these can lower cells in an extended One World, so refresh the global
		// filler/bedrock plane and the floor passed to tree placement here.
		area_floor = area_floor_for(editor.ground, args);
		world_editor::set_terrain_floor_y(area_floor);
		world_editor::set_base_chunk_y(area_floor);
		if (args.debug)
			editor.ground->save_land_cover_debug_image(
					"landcover_debug_post_bridge_repair");
	}
	// Rust materializes the lazy one-bit-per-world-column ESA water mask after
	// ground/shoreline preparation and before terrain workers start. This also
	// keeps the call harmless when a backend supplied no land-cover data.
	if (editor.ground)
		editor.ground->warm_water_blend();
	auto road_mask_owner = std::make_shared<const RoadMaskBitmap>(
			highways::collect_road_surface_coords(elements, editor, xzbbox, args.scale));
	const auto &road_mask = *road_mask_owner;
	const auto clear_corridors = std::make_shared<const CoordinateBitmap>(
			aerialways::clear_corridors(elements, xzbbox));
	editor.set_clear_corridors(clear_corridors);
	editor.set_fuel_index(std::make_shared<const fuel_stations::FuelIndex>(
			fuel_stations::FuelIndex::build(elements, args.scale)));
	auto barrier_clearance =
			barriers::RoadClearance::build(elements, editor, args.scale, road_mask_owner);
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
		editor.set_decal_frame_sink(
				[](WorldEditor &target, const WorldEditor::DecalFrame &frame) {
					if (!target.decal_registry)
						return false;
					const auto texture =
							decal_texture(*target.decal_registry, frame.map_id);
					if (!texture)
						return false;
					Block node = block_definitions::DECAL_FRAME;
					target.set_block_absolute(
							node, frame.x, frame.y, frame.z, std::nullopt, std::nullopt);
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc++11-narrowing"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
#endif
					return target.mg &&
						   target.mg->queueGeneratedDecal(target.active_tile_overlay,
								   {frame.x, frame.y, frame.z}, *texture, frame.map_id,
								   frame.facing, frame.rotation, frame.glow);
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
				},
				true);
	}
	auto big_water_field = water_depth::compute_big_water_field(editor, xzbbox);
	// Pre-scan still water surfaces for consistent water levels across tiles
	StillWaterSurfaces still_surfaces;
	if (editor.ground)
		still_surfaces =
				water_areas::prescan_still_surfaces(elements, editor.ground, xzbbox);
	progress::emit_gui_progress_update(19.5, "Processing data...");
	auto highway_connectivity = highways::build_highway_connectivity_map(elements);
	if (args.one_world_run)
		highway_connectivity.meet_at_edge_of(cartesian::XZBBox::rect_from_min_max(
				xzbbox.min_x(), xzbbox.min_z(), xzbbox.max_x(), xzbbox.max_z()));
	auto bridge_outlines = bridge_styles::BridgeOutlineIndex::build(elements);
	auto bridge_structures = bridges::BridgeStructureMap::build(
			elements, editor, highway_connectivity, bridge_outlines, args.scale);
	auto bridge_surface =
			bridges::BridgeSurfaceMap::build(elements, bridge_structures, args.scale);
	// Rust completes tree-pack loading before generation. Keep the C++ load
	// overlapped with earlier passes, but do not silently skip schematic trees
	// on this first generation merely because the async cache was still warming.
	if (!shared_selector && !args.legacy_trees) {
		shared_selector = cached_region_selector(tree_pack_root,
				dominant_tree_realm.value_or(""), centre_lat, centre_lon, args.scale,
				area_floor, args.max_tree_size,
				editor.ground ? editor.ground->elevation_blocks_per_meter : 0.0,
				area_ecoregion_ids, exclude_palms, true);
		if (shared_selector)
			editor.set_tree_slot_spacing(shared_selector->slot_spacing(true));
		if (shared_selector)
			editor.set_schematic_tree_pack_available(true);
	}
	// Rust switches to compact proportional trees before consulting schematic
	// packs at very small world scales. Keep the selector callbacks disabled in
	// that mode so both mapped and land-cover trees take the same micro path.
	if (shared_selector && args.scale >= MICRO_TREE_MAX_SCALE) {
		editor.tree_crown_columns = [shared_selector](int height) {
			return shared_selector->crown_columns(height);
		};
		editor.set_regional_tree_placer(
				[shared_selector, &building_footprints, &bridge_surface](
						WorldEditor &tile_editor, int x, int y, int z, std::uint8_t cover,
						std::optional<trees::Habitat> habitat_hint, bool tagged,
						bool wet_ground, bool allow_on_paved,
						bool density_decided) -> std::optional<bool> {
					trees::SlotRequest request;
					request.density_decided = density_decided;
					request.tagged = tagged;
					const auto [slot_x, slot_z] = trees::trunk_slot_s(
							x, z, shared_selector->slot_spacing(density_decided));
					const auto slot_cover =
							tile_editor.ground
									? tile_editor.ground->cover_class(
											  tile_editor.ground_point(slot_x, slot_z))
									: cover;
					request.wet_ground =
							tile_editor.ground
									? slot_cover == land_cover::LC_WETLAND ||
											  slot_cover == land_cover::LC_MANGROVES
									: wet_ground;
					request.eco =
							tile_editor.ground
									? tile_editor.ground->ecoregion_at(
											  tile_editor.ground_point(slot_x, slot_z))
									: std::nullopt;
					request.want_height =
							Tree::wanted_height_blocks(tile_editor, slot_x, slot_z);
					if (request.eco) {
						const int beach_radius = std::clamp(
								static_cast<int>(std::lround(12.0 * tile_editor.scale())),
								2, 12);
						for (const auto &[dx, dz] : std::array<std::pair<int, int>, 5>{
									 {{0, 0}, {beach_radius, 0}, {-beach_radius, 0},
											 {0, beach_radius}, {0, -beach_radius}}})
							if (tile_editor.ground &&
									tile_editor.ground->cover_class(
											tile_editor.ground_point(
													slot_x + dx, slot_z + dz)) ==
											land_cover::LC_BEACH) {
								request.beach = true;
								break;
							}
					}
					if (!habitat_hint)
						return trees::place_selected_region_tree_for_cover(tile_editor,
								*shared_selector, x, z, slot_cover, y, request,
								&building_footprints, &bridge_surface, allow_on_paved);
					return trees::place_selected_region_tree(tile_editor,
							*shared_selector, x, z, *habitat_hint, y, request,
							&building_footprints, &bridge_surface, allow_on_paved);
				},
				true);
		editor.set_mapped_regional_tree_placer(
				[shared_selector, &building_footprints, &bridge_surface](
						WorldEditor &tile_editor, int x, int y, int z, std::uint8_t cover,
						const trees::MappedRequest &request) -> std::optional<bool> {
					(void)cover;
					// Tree::create_mapped passes an absolute placement height, while
					// schematic placement expects an offset from local ground.
					const int ground_y = tile_editor.get_ground_level(x, z);
					const int y_offset = y - ground_y;
					// Rust falls back to the requested absolute base when sampled
					// terrain elevation is unavailable.
					const int elevation = tile_editor.terrain_level(x, z).value_or(y);
					return trees::place_selected_mapped_region_tree(tile_editor,
							*shared_selector, x, z, elevation, y_offset, request,
							&building_footprints, &bridge_surface);
				},
				true);
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
	// Rust also builds the junction/crossing index before it reorders ground
	// fills; only then does element dispatch begin.
	const auto geo_bounds = editor.geographic_bounds();
	auto marking_index = road_markings::RoadMarkingIndex::build(elements, args.scale,
			decals::detect_region((geo_bounds[0] + geo_bounds[1]) * .5,
					(geo_bounds[2] + geo_bounds[3]) * .5));
	if (args.traffic_lights && java_world) {
		editor.traffic_signal_index = highways::SignalIndex::build(
				elements, args.scale, marking_index.drives_on_left, road_mask);
	} else {
		editor.traffic_signal_index.reset();
	}
	if (args.moving_vehicles_enabled() && java_world) {
		auto surface_y = [&](int x, int z) {
			if (const auto deck = bridge_surface.nearby_deck_y(x, z, 0))
				return *deck;
			if (editor.ground)
				return editor.ground->level(editor.ground_point(x, z));
			return args.ground_level;
		};
		editor.car_network = std::make_shared<const highways::cars::CarNetwork>(
				highways::cars::CarNetwork::build(elements, args.scale,
						marking_index.drives_on_left, args.traffic_lights, xzbbox,
						surface_y));
	} else {
		editor.car_network.reset();
	}
	// Rust builds the backend-neutral train timetable before tile dispatch and
	// shares it with the editor so the library's consumer can animate or render
	// the scheduled rail services.
	if (args.moving_vehicles_enabled() && java_world) {
		auto network = trains::TrainRouteNetwork::build(elements, args.scale, xzbbox);
		if (!network.is_empty())
			editor.train_network =
					std::make_shared<const trains::TrainRouteNetwork>(std::move(network));
		else
			editor.train_network.reset();
		if (editor.train_network)
			editor.enable_rail_height_collection();
		else {
			editor.rail_heights.reset();
			editor.rail_heights_mutex.reset();
		}
	} else {
		editor.train_network.reset();
		editor.rail_heights.reset();
		editor.rail_heights_mutex.reset();
	}
	// Rust reorders only ground-fill elements here, after applying OSM ground
	// overrides and building the shared indexes. Keep the cached element vector
	// immutable and carry this order as indices to avoid a deep copy per chunk.
	auto dispatch_order = ground_fill_area_order(elements);
	std::vector<std::size_t> dispatch_rank(elements.size());
	for (std::size_t rank = 0; rank < dispatch_order.size(); ++rank)
		dispatch_rank[dispatch_order[rank]] = rank;
	struct ElementDispatchState
	{
		std::vector<std::tuple<int, int, int, bool>> rail_tunnel_points;
		std::vector<highways::HighwayTunnelCell> highway_tunnel_cells;
	};
	ElementDispatchState dispatch_state;
	auto tunnel_internal_endpoints =
			highways::collect_tunnel_internal_endpoints(elements, xzbbox);
	auto tunnel_portals = highways::collect_tunnel_portals(
			elements, editor, bridge_structures, tunnel_internal_endpoints, args.scale);
	auto tunnel_footprint = highways::collect_tunnel_footprint(
			elements, editor, tunnel_internal_endpoints, xzbbox, args.scale);
	auto rail_joins = railways::RailJoins::build(elements, railways::track_level);
	if (args.one_world_run) {
		const auto [min_x, min_z] = editor.get_min_coords();
		const auto [max_x, max_z] = editor.get_max_coords();
		rail_joins.meet_at_edge_of(
				cartesian::XZBBox::rect_from_min_max(min_x, min_z, max_x, max_z));
	}
	// fm: Share the resolved network with tile workers, including standalone callers.
	editor.native_rail_network = railways::advtrains::prepare_network(elements, editor);
	// ===
	railways::add_tunnel_footprint(elements, xzbbox, tunnel_footprint);
	// Centreline bitmap prevents catenary masts from being stamped into a
	// neighbouring parallel track, matching the Rust railway pass.
	auto rail_mask = railways::collect_at_grade_rail_mask(elements, xzbbox);

	// Both the parser-prepared path and direct library callers use the same
	// geographic suppression/grouping pass as Rust. Reconstructing a reduced
	// projected-space approximation here loses split relation rings and can
	// render duplicate building outlines.
	auto outline_suppression = mode_filter::outline_suppression_for_mode(
			building_data.outline_suppression, args.mode);
	std::unordered_set<std::uint64_t> suppressed_building_outlines;
	auto building_part_groups = building_data.part_groups;
	for (const auto &[kind, id] : outline_suppression)
		if (kind == "way")
			suppressed_building_outlines.insert(id);
	const auto &suppressed_relations = building_data.suppressed_relations;
	std::unordered_map<std::uint64_t, std::vector<std::uint64_t>> building_group_members;
	// Match Rust's grouping key, which clears packed facade, year and housing metadata.
	building_group_members.clear();
	for (const auto &[way_id, seed] : building_part_groups)
		building_group_members[osm_parser::group_key(seed)].push_back(way_id);
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
	auto element_tiles = create_tiles(xzbbox, DEFAULT_TILE_SIZE);
	const bool tiled_element_dispatch =
			should_use_parallel_tiles(element_tiles.size(), java_format);
	// Streaming is independent of scheduling policy: ARNIS_NO_BAND only switches
	// the eviction-safe row bands back to LPT order for diagnosis.
	const bool force_lpt_tile_order = std::getenv("ARNIS_NO_BAND") != nullptr;
	bool eviction_active = false;
	bool band_tile_order = false;
	std::uint64_t tile_available_memory_mb = 0;
	std::vector<std::vector<std::size_t>> element_assignments;
	std::vector<std::pair<std::size_t, std::size_t>> element_tasks;
	std::optional<TileFillReaders> tile_fill_reader_state;
	std::mutex tile_fill_release_mutex;
	FillExpirations expiring_fills;
	if (tiled_element_dispatch) {
		element_assignments =
				assign_elements_to_tiles(elements, element_tiles, args.scale);
		for (auto &assigned : element_assignments)
			std::stable_sort(
					assigned.begin(), assigned.end(), [&](std::size_t a, std::size_t b) {
						return dispatch_rank[a] < dispatch_rank[b];
					});
		// Rust samples available memory after the expensive tile/fill precompute.
		// Do the same here so streaming reflects the working set this pass has
		// actually allocated, not the more optimistic pre-assignment snapshot.
		tile_fill_reader_state = compute_tile_fill_readers(element_assignments, elements,
				building_part_groups, building_group_members, flood_fill_cache);
		tile_available_memory_mb = available_memory_mb();
		eviction_active =
				java_format && should_stream_to_disk(element_tiles.size(),
									   tile_available_memory_mb, args.fillground);
		band_tile_order = eviction_active && !force_lpt_tile_order;
		// Streaming normally uses row-major region bands to keep the eviction
		// frontier monotonic; ARNIS_NO_BAND selects LPT for ordering diagnostics.
		// The C++ Freeminer backend still processes overlays serially, but stable
		// tile/merge order keeps overlapping halo writes deterministic.
		std::vector<std::size_t> tile_order(element_tiles.size());
		std::iota(tile_order.begin(), tile_order.end(), 0);
		std::stable_sort(
				tile_order.begin(), tile_order.end(), [&](std::size_t a, std::size_t b) {
					if (band_tile_order) {
						const auto row_a = element_tiles[a].min_z >> 9;
						const auto row_b = element_tiles[b].min_z >> 9;
						if (row_a != row_b)
							return row_a < row_b;
					}
					return element_assignments[a].size() > element_assignments[b].size();
				});
		std::vector<TileBounds> ordered_tiles;
		std::vector<std::vector<std::size_t>> ordered_assignments;
		std::vector<std::vector<std::uint64_t>> ordered_fill_ids;
		ordered_tiles.reserve(element_tiles.size());
		ordered_assignments.reserve(element_assignments.size());
		ordered_fill_ids.reserve(tile_fill_reader_state->tile_ids.size());
		for (const auto index : tile_order) {
			ordered_tiles.push_back(std::move(element_tiles[index]));
			ordered_assignments.push_back(std::move(element_assignments[index]));
			ordered_fill_ids.push_back(
					std::move(tile_fill_reader_state->tile_ids[index]));
		}
		element_tiles = std::move(ordered_tiles);
		element_assignments = std::move(ordered_assignments);
		tile_fill_reader_state->tile_ids = std::move(ordered_fill_ids);
		std::size_t task_count = 0;
		for (const auto &assigned : element_assignments)
			task_count += assigned.size();
		element_tasks.reserve(task_count);
		for (std::size_t tile_index = 0; tile_index < element_assignments.size();
				++tile_index)
			for (const auto element_index : element_assignments[tile_index])
				element_tasks.emplace_back(element_index, tile_index);
	} else {
		element_tasks.reserve(elements.size());
		for (const auto element_index : dispatch_order)
			element_tasks.emplace_back(
					element_index, std::numeric_limits<std::size_t>::max());
	}
	if (!tiled_element_dispatch) {
		auto last_fill_use = compute_last_fill_use(
				elements, element_tasks, building_part_groups, building_group_members);
		expiring_fills = fills_expiring_at(last_fill_use);
	}
	auto release_task_fills = [&](std::size_t task_index) {
		if (!tile_fill_reader_state)
			release_finished_fills(flood_fill_cache, expiring_fills, task_index);
	};
	auto release_tile_fills_for = [&](std::size_t tile_index) {
		if (tile_fill_reader_state &&
				tile_index < tile_fill_reader_state->tile_ids.size()) {
			std::lock_guard<std::mutex> lock(tile_fill_release_mutex);
			release_tile_fills(tile_fill_reader_state->tile_ids[tile_index],
					tile_fill_reader_state->remaining_readers, flood_fill_cache);
		}
	};

	// All roads share the same immutable connectivity and marking index.
	// Rust builds Java tiles against independent editors, then merges results in
	// stable order in bounded batches. Only use worker editors when the host
	// supplies overlay-backed writes and backend callbacks are tile-safe.
	const bool regional_ground_passes =
			tiled_element_dispatch && java_format && element_tiles.size() >= 3;
	const std::size_t model_unit_count =
			model_pipeline ? model_pipeline->three_dmr().placements.size() +
									 model_pipeline->wikidata().placements.size() +
									 model_pipeline->stadium().placements.size()
						   : 0;
	const std::size_t plane_unit_count = plane_candidates.size();
	const std::size_t landmark_unit_count = landmark_plan.placements.size();
	const std::size_t plane_unit_begin = model_unit_count;
	const std::size_t landmark_unit_begin = plane_unit_begin + plane_unit_count;
	DeferredPlacements deferred_placements;
	const bool deferred_placements_active = eviction_active && regional_ground_passes;
	std::set<std::pair<int, int>> styled_regions;
	std::vector<std::pair<int, int>> tile_regions;
	std::set<std::pair<int, int>> real_regions;
	std::map<std::pair<int, int>, std::size_t> remaining_region_tiles;
	if (deferred_placements_active) {
		std::vector<std::vector<std::pair<int, int>>> unit_regions;
		if (model_pipeline) {
			auto regions = model_pipeline->placement_regions(args.scale);
			unit_regions.insert(unit_regions.end(),
					std::make_move_iterator(regions.begin()),
					std::make_move_iterator(regions.end()));
		}
		auto plane_regions = structures::plane::placement_regions(plane_candidates);
		unit_regions.insert(unit_regions.end(),
				std::make_move_iterator(plane_regions.begin()),
				std::make_move_iterator(plane_regions.end()));
		auto landmark_regions = landmarks::placement_regions(landmark_plan, args.scale);
		unit_regions.insert(unit_regions.end(),
				std::make_move_iterator(landmark_regions.begin()),
				std::make_move_iterator(landmark_regions.end()));
		tile_regions.reserve(element_tiles.size());
		for (const auto &tile : element_tiles) {
			const auto region = std::pair{
					tiles::region_floor(tile.min_x), tiles::region_floor(tile.min_z)};
			tile_regions.push_back(region);
			real_regions.insert(region);
		}
		// A region is not final when only its own tile has merged: neighboring
		// tile tasks can write into it through element/model halos. Track every
		// in-world tile that may contribute to each 3x3 region neighborhood, as
		// Rust's streaming coordinator does before releasing or styling regions.
		for (const auto &source : tile_regions)
			for (int dz = -1; dz <= 1; ++dz)
				for (int dx = -1; dx <= 1; ++dx) {
					const auto target = std::pair{source.first + dx, source.second + dz};
					if (real_regions.contains(target))
						++remaining_region_tiles[target];
				}
		deferred_placements = DeferredPlacements(
				std::move(unit_regions), [&real_regions](const auto region) {
					return real_regions.contains(region);
				});
	}
	std::set<std::pair<int, int>> deferred_write_regions;
	if (deferred_placements_active)
		for (const auto &region : deferred_placements.regions())
			deferred_write_regions.insert(region);
	// Winter and ruins styling reads road elevations after tiles merge. Preserve
	// those overrides for every style region, not only regions with deferred models.
	const bool keep_roads_for_style = world_style::is_active(args);
	auto execute_deferred_placements = [&](const std::vector<std::size_t> &units) {
		if (units.empty())
			return;
		std::vector<std::size_t> model_units, plane_units, landmark_units;
		for (const auto unit : units) {
			if (unit < model_unit_count)
				model_units.push_back(unit);
			else if (unit < landmark_unit_begin)
				plane_units.push_back(unit - plane_unit_begin);
			else if (unit < landmark_unit_begin + landmark_unit_count)
				landmark_units.push_back(unit - landmark_unit_begin);
		}
		if (model_pipeline && !model_units.empty())
			model_pipeline->place_selected(three_dmr_provider, wikidata_provider,
					custom_model_provider, editor, args.scale, model_units);
		if (!plane_units.empty()) {
			std::vector<structures::plane::Placement> selected;
			selected.reserve(plane_units.size());
			for (const auto index : plane_units)
				if (index < plane_candidates.size())
					selected.push_back(plane_candidates[index]);
			structures::plane::place_plane_placements(editor, selected);
		}
		for (const auto index : landmark_units)
			if (index < landmark_plan.placements.size())
				landmarks::place(editor, landmark_plan.placements[index], args.scale,
						args.rotation);
	};
	auto style_region_if_ready = [&](const std::pair<int, int> &region) {
		const auto remaining = remaining_region_tiles.find(region);
		if (!deferred_placements_active || remaining == remaining_region_tiles.end() ||
				remaining->second != 0 || deferred_placements.holds(region) ||
				!styled_regions.insert(region).second)
			return;
		editor.clear_strict_bounds();
		world_style::apply_region(
				editor, args, building_footprints, region.first, region.second);
		if (world_style::is_active(args) && !deferred_write_regions.contains(region))
			editor.release_road_surface_overrides(region.first, region.second);
	};
	water_steps::Borders water_borders;
	std::mutex water_borders_mutex;
	std::atomic<std::int64_t> water_steps_elapsed_ns{0};
	auto complete_deferred_tile = [&](const TileBounds &tile) {
		if (!deferred_placements_active)
			return true;
		if (!editor.flush_deferred_tile_merges())
			return false;
		// Deferred placements and regional styling intentionally escape the
		// just-merged tile's element bounds; the next tile reinstates its bounds.
		editor.clear_strict_bounds();
		const auto completed_region = std::pair{
				tiles::region_floor(tile.min_x), tiles::region_floor(tile.min_z)};
		std::vector<std::pair<int, int>> newly_ready;
		std::vector<std::pair<int, int>> flushable_regions;
		for (int dz = -1; dz <= 1; ++dz)
			for (int dx = -1; dx <= 1; ++dx) {
				const auto region = std::pair{
						completed_region.first + dx, completed_region.second + dz};
				auto remaining = remaining_region_tiles.find(region);
				if (remaining == remaining_region_tiles.end() || remaining->second == 0)
					continue;
				if (--remaining->second == 0) {
					if (!args.winter) {
						std::lock_guard<std::mutex> lock(water_borders_mutex);
						const auto water_steps_start = Bench::Clock::now();
						water_steps::shape_region(editor, water_borders, region,
								args.caves, [&](water_steps::Borders::Region r) {
									return real_regions.contains(r);
								});
						water_steps_elapsed_ns.fetch_add(
								std::chrono::duration_cast<std::chrono::nanoseconds>(
										Bench::Clock::now() - water_steps_start)
										.count(),
								std::memory_order_relaxed);
					}
					flushable_regions.push_back(region);
					newly_ready.push_back(region);
				}
			}
		// Release every newly-complete region before styling any of them. A model
		// spanning two regions may become placeable on this tile and still write
		// back into another region that reached zero in the same completion step.
		for (const auto &region : flushable_regions) {
			const auto waves = deferred_placements.complete(region);
			for (const auto &wave : waves) {
				for (const auto unit : wave) {
					const auto &regions = deferred_placements.regions_of(unit);
					newly_ready.insert(newly_ready.end(), regions.begin(), regions.end());
				}
				execute_deferred_placements(wave);
			}
		}
		std::sort(newly_ready.begin(), newly_ready.end());
		newly_ready.erase(
				std::unique(newly_ready.begin(), newly_ready.end()), newly_ready.end());
		for (const auto &region : newly_ready)
			style_region_if_ready(region);
		return true;
	};
	const bool parallel_tile_workers =
			regional_ground_passes && bool(editor.begin_tile_sink) &&
			bool(editor.merge_tile_sink) && editor.tile_worker_callbacks_safe();
	const auto pool_threads = static_cast<std::size_t>(
			std::max(1u, std::min(16u, std::thread::hardware_concurrency())));
	if (!tiled_element_dispatch)
		tile_available_memory_mb = available_memory_mb();
	const auto tile_batch_size = tile_processing_threads(
			pool_threads, tile_available_memory_mb, args.fillground);
	std::size_t deferred_tiles_in_batch = 0;
	if (regional_ground_passes)
		editor.set_defer_tile_merges(true);
	auto begin_generation_tile = [&](WorldEditor &tile_editor, std::size_t tile_index) {
		// fm: Async tile threads must bind the same immutable native rail network.
		railways::advtrains::use_network(tile_editor);
		// ===
		const auto &tile = element_tiles[tile_index];
		const int min_x = std::max(tile.min_x, xzbbox.min_x());
		const int max_x = std::min(tile.max_x - 1, xzbbox.max_x());
		const int min_z = std::max(tile.min_z, xzbbox.min_z());
		const int max_z = std::min(tile.max_z - 1, xzbbox.max_z());
		tile_editor.set_strict_bounds(min_x, min_z, max_x, max_z);
		const bool began = tile_editor.begin_tile(
				tile.min_x, tile.min_z, tile.max_x - 1, tile.max_z - 1);
		if (!began)
			tile_editor.report_generation_error("failed to begin tile overlay");
		return began;
	};
	auto merge_finished_tile = [&](WorldEditor &tile_editor, const TileBounds &tile) {
		const bool merged = tile_editor.merge_tile(
				tile.min_x, tile.min_z, tile.max_x - 1, tile.max_z - 1);
		if (!merged) {
			editor.report_generation_error("failed to finish tile overlay");
			return false;
		}
		if (&tile_editor != &editor &&
				!editor.merge_tile_worker(std::move(tile_editor), tile.min_x, tile.min_z,
						tile.max_x - 1, tile.max_z - 1,
						eviction_active && !keep_roads_for_style ? &deferred_write_regions
																 : nullptr)) {
			editor.report_generation_error("failed to collect tile worker result");
			return false;
		}
		if (deferred_placements_active) {
			deferred_tiles_in_batch = 0;
			if (!complete_deferred_tile(tile)) {
				editor.report_generation_error(
						"failed to flush completed placement region");
				return false;
			}
		} else if (++deferred_tiles_in_batch >= tile_batch_size) {
			deferred_tiles_in_batch = 0;
			if (!editor.flush_deferred_tile_merges()) {
				editor.report_generation_error("failed to merge tile overlay batch");
				return false;
			}
		}
		return true;
	};
	progress::emit_gui_progress_update(20.0, "Generating area...");
	const std::size_t generation_work_total =
			regional_ground_passes ? element_tiles.size() : element_tasks.size();
	std::atomic<std::size_t> completed_generation_tiles{0};
	std::atomic<int> last_generation_progress_bucket{regional_ground_passes ? 20 : 80};
	auto report_generation_work = [&](std::size_t completed) {
		if (generation_work_total == 0)
			return;
		const double pct = 20.0 + 50.0 * std::min(completed, generation_work_total) /
										  static_cast<double>(generation_work_total);
		const int bucket =
				static_cast<int>(std::floor(pct * (regional_ground_passes ? 1.0 : 4.0)));
		int previous = last_generation_progress_bucket.load(std::memory_order_relaxed);
		while (bucket > previous) {
			if (last_generation_progress_bucket.compare_exchange_weak(previous, bucket,
						std::memory_order_relaxed, std::memory_order_relaxed)) {
				progress::emit_gui_progress_update_ex(pct, "", eviction_active);
				break;
			}
		}
	};
	auto report_finished_tile = [&] {
		const auto completed =
				completed_generation_tiles.fetch_add(1, std::memory_order_relaxed) + 1;
		report_generation_work(completed);
	};
	auto finish_ground_tile = [&](WorldEditor &tile_editor,
									  ElementDispatchState &tile_state,
									  std::size_t tile_index,
									  bool return_worker_result = false) {
		WorldEditor &editor = tile_editor;
		const auto &tile = element_tiles[tile_index];
		const int min_x = std::max(tile.min_x, xzbbox.min_x());
		const int max_x = std::min(tile.max_x - 1, xzbbox.max_x());
		const int min_z = std::max(tile.min_z, xzbbox.min_z());
		const int max_z = std::min(tile.max_z - 1, xzbbox.max_z());
		if (min_x > max_x || min_z > max_z) {
			const bool merged = return_worker_result
										? tile_editor.merge_tile(tile.min_x, tile.min_z,
												  tile.max_x - 1, tile.max_z - 1)
										: merge_finished_tile(tile_editor, tile);
			if (merged && regional_ground_passes)
				report_finished_tile();
			return merged;
		}
		editor.set_strict_bounds(min_x, min_z, max_x, max_z);
		// These POI/address plates need their host building, and must precede
		// ground decoration just like other tile-owned element writes.
		if (editor.signage_enabled())
			for (const auto element_index : element_assignments[tile_index]) {
				const auto &element = elements[element_index];
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
			caves::CaveEllipsoids tile_ellipsoids;
			std::unordered_set<std::int64_t> cave_air;
			caves::carve_region(editor, cave_region, cave_world, CAVE_SEED,
					world_editor::terrain_floor_y(), &tile_ellipsoids, &cave_air);
			const caves::CaveShapeQuery cave_shape(editor, cave_world, cave_region,
					CAVE_SEED, world_editor::terrain_floor_y(), tile_ellipsoids);
			std::unordered_set<std::int64_t> cave_basin_fluid;
			std::unordered_set<std::int64_t> cave_water_cells;
			caves::WaterPlan cave_water_plan;
			caves::generate_water_features(editor, cave_world, cave_region, CAVE_SEED,
					world_editor::terrain_floor_y(), args, cave_shape, &cave_basin_fluid,
					&cave_water_cells, &cave_air, &cave_water_plan);
			caves::generate_deep_lava_sea(editor, cave_region,
					world_editor::terrain_floor_y(), cave_air, cave_water_cells,
					&cave_basin_fluid);
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
		highways::drop_lamps_in_water(editor);
		// Rust's eviction path cannot run these passes after regions have been
		// released. Match it by reopening tunnel interiors after the tile's ground,
		// ore and water writes, then sealing any newly exposed cave fluids before
		// this tile is merged. The editor's strict bounds keep the global tunnel
		// indexes from writing outside the tile being finalized.
		if (regional_ground_passes && eviction_active) {
			std::vector<highways::TunnelLight> tunnel_lights;
			if (!tile_state.rail_tunnel_points.empty())
				tunnel_lights = railways::carve_rail_tunnel_interior(
						editor, tile_state.rail_tunnel_points);
			if (!tile_state.highway_tunnel_cells.empty()) {
				auto road_lights = highways::carve_highway_tunnel_interior(
						editor, tile_state.highway_tunnel_cells);
				tunnel_lights.insert(
						tunnel_lights.end(), road_lights.begin(), road_lights.end());
			}
			highways::hang_tunnel_lights(editor, tunnel_lights);
			if (args.caves)
				caves::seal_floating_fluid_region(editor, {min_x, max_x, min_z, max_z},
						world_editor::terrain_floor_y());
		}
		if (deferred_placements_active && !args.winter) {
			std::lock_guard<std::mutex> lock(water_borders_mutex);
			water_steps::collect_region_candidates(
					editor, water_borders, min_x, max_x, min_z, max_z);
		}
		const bool merged = return_worker_result
									? tile_editor.merge_tile(tile.min_x, tile.min_z,
											  tile.max_x - 1, tile.max_z - 1)
									: merge_finished_tile(tile_editor, tile);
		if (merged && regional_ground_passes)
			report_finished_tile();
		return merged;
	};
	std::size_t active_tile = std::numeric_limits<std::size_t>::max();
	std::size_t next_ground_tile = 0;
	std::optional<WorldEditor> active_tile_worker;
	ElementDispatchState active_tile_dispatch_state;
	auto collect_tile_dispatch_state = [&](const ElementDispatchState &state,
											   const TileBounds &tile) {
		for (const auto &point : state.rail_tunnel_points)
			if (tile.contains(std::get<0>(point), std::get<1>(point)))
				dispatch_state.rail_tunnel_points.push_back(point);
		dispatch_state.highway_tunnel_cells.insert(
				dispatch_state.highway_tunnel_cells.end(),
				state.highway_tunnel_cells.begin(), state.highway_tunnel_cells.end());
	};
	auto process_assigned_element = [&](WorldEditor &target, ElementDispatchState &state,
											std::size_t task_index,
											std::size_t element_index) {
		WorldEditor &editor = target;
		const auto &element = elements[element_index];
		// Rust passes the same normalized generation options to every element;
		// terrain-aware handlers query their own coordinates rather than mutating
		// the shared flat-ground fallback from an element's first node.
		const Args &args = effective_args;
		if (args.mode != GenerationMode::Blueprint && element.is_relation() &&
				suppressed_relations.count(element.id())) {
			release_task_fills(task_index);
			return;
		}
		if (outline_suppression.count({std::string(element.kind()), element.id()})) {
			release_task_fills(task_index);
			return;
		}
		if (model_pipeline &&
				std::find(model_pipeline->suppressed().begin(),
						model_pipeline->suppressed().end(),
						std::pair<std::string, std::uint64_t>{std::string(element.kind()),
								element.id()}) != model_pipeline->suppressed().end()) {
			release_task_fills(task_index);
			return;
		}
		if (!landmark_plan.suppressed.empty() &&
				std::find(landmark_plan.suppressed.begin(),
						landmark_plan.suppressed.end(),
						std::pair<std::string, std::uint64_t>{std::string(element.kind()),
								element.id()}) != landmark_plan.suppressed.end()) {
			release_task_fills(task_index);
			return;
		}

		if (element.is_way()) {
			auto const &way = element.as_way();
			if (way.tags.contains("barrier") && !way.tags.contains("building") &&
					way.tags.get("power") == std::optional<std::string>("generator"))
				power::generate_power(editor, element, building_footprints,
						flood_fill_cache, args.timeout);

			if (way.tags.get("man_made") == std::optional<std::string>("pier")) {
				// Rust dispatches piers directly to the waterfront pass before the
				// generic man_made layer/level filtering and before building handling.
				// Keep that ordering here as piers may also carry building/path tags.
				waterfront::generate_pier(editor, element, flood_fill_cache, args);
			} else if (args.mode != GenerationMode::Blueprint &&
					   attractions::claims(way.tags) &&
					   attractions::generate_ride(editor, element, flood_fill_cache,
							   building_footprints, road_mask, args.scale,
							   args.timeout)) {
			} else if (args.mode != GenerationMode::Blueprint &&
					   fuel_stations::is_station_canopy(editor, way) &&
					   fuel_stations::generate_canopy(editor, way, flood_fill_cache,
							   road_mask, args.scale, args.timeout)) {
				if (way.tags.get("amenity") == std::optional<std::string>("fuel"))
					fuel_stations::generate_station(editor, element, flood_fill_cache,
							building_footprints, road_mask, args.scale, args.timeout);
			} else if (args.mode == GenerationMode::Blueprint &&
					   (way.tags.contains("building") ||
							   way.tags.contains("building:part"))) {
				blueprint::generate_outline(editor, way);
			} else if (way.tags.contains("building") ||
					   way.tags.contains("building:part")) {
				if (suppressed_building_outlines.find(way.id) ==
						suppressed_building_outlines.end()) {
					std::optional<ProcessedWay> facade_way;
					const ProcessedWay *render_way = &way;
					if (const auto inherited =
									building_data.inherited_facade_tags.find(way.id);
							inherited != building_data.inherited_facade_tags.end()) {
						facade_way = way;
						for (const auto &[key, value] : inherited->second)
							facade_way->tags.try_emplace(key, value);
						render_way = &*facade_way;
					}
					auto group = building_part_groups.find(way.id);
					auto anchor = buildings::generate_buildings(&editor, *render_way,
							args, std::optional<int>{}, flood_fill_cache,
							building_passages, nullptr,
							group == building_part_groups.end()
									? std::nullopt
									: std::optional<std::uint64_t>(group->second),
							&road_mask, &building_footprints, &building_group_members,
							interior_use_index ? &*interior_use_index : nullptr);
					if (editor.signage_enabled())
						signage::generate_building_signage(editor, *render_way, anchor);
					if (way.tags.get("amenity") == std::optional<std::string>("fuel"))
						fuel_stations::generate_station(editor, element, flood_fill_cache,
								building_footprints, road_mask, args.scale, args.timeout);
				}
			} else if (aerialways::is_lift(way.tags)) {
				aerialways::generate_aerialway(
						editor, way, building_footprints, road_mask);
			} else if (structures::jetbridge::claims(way)) {
				structures::jetbridge::generate_jet_bridge(
						editor, way, building_footprints);
			} else if (way.tags.contains("highway")) {
				if (editor.car_network)
					editor.car_network->place_way(editor, way.id);
				const bool tunnel_rendered =
						highways::renders_as_highway_tunnel(way) &&
						highways::generate_highway_tunnel_shell(editor, way, args,
								tunnel_internal_endpoints, tunnel_portals,
								state.highway_tunnel_cells);
				if (!tunnel_rendered) {
					highways::generate_highways_internal(editor, element, args,
							highway_connectivity, &flood_fill_cache, args.timeout,
							road_mask, bridge_structures, bridge_surface, tunnel_portals,
							marking_index,
							tunnel_footprint.is_empty() ? nullptr : &tunnel_footprint);
				}
				if (editor.signage_enabled())
					signage::generate_highway_way_signage(
							editor, way, building_footprints);
			} else if (way.tags.get("amenity") == std::optional<std::string>("fuel")) {
				fuel_stations::generate_station(editor, element, flood_fill_cache,
						building_footprints, road_mask, args.scale, args.timeout);
			} else if (way.tags.contains("landuse")) {
				landuse::generate_landuse(editor, way, args, flood_fill_cache,
						building_footprints, road_mask, bridge_surface);
			} else if (way.tags.contains("natural") &&
					   way.tags.get("amenity") !=
							   std::optional<std::string>(std::string("fountain"))) {
				natural::generate_natural(editor, element, args, flood_fill_cache,
						building_footprints, bridge_surface);
			} else if (way.tags.contains("amenity")) {
				if (way.tags.get("amenity") == std::optional<std::string>("fuel"))
					fuel_stations::generate_station(editor, element, flood_fill_cache,
							building_footprints, road_mask, args.scale, args.timeout);
				amenities::generate_amenities(
						editor, element, args, flood_fill_cache, road_mask);
				if (editor.signage_enabled() && way.tags.get("amenity") == "parking")
					signage::generate_parking_signage(editor, way, road_mask);
			} else if (way.tags.contains("leisure")) {
				leisure::generate_leisure(editor, way, args, flood_fill_cache,
						building_footprints, bridge_surface);
			} else if (way.tags.contains("barrier")) {
				barriers::generate_barriers(editor, element, bridge_surface,
						barrier_clearance, building_footprints);
			} else if (way.tags.contains("waterway")) {
				auto it_val = way.tags.find("waterway");
				if (it_val != way.tags.end() &&
						(it_val->second == "dock" || it_val->second == "riverbank")) {
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
				railways::generate_railways(editor, way, state.rail_tunnel_points,
						bridge_outlines, road_mask, building_footprints, rail_mask,
						bridge_structures, bridge_surface, rail_joins);
			} else if (way.tags.contains("roller_coaster")) {
				railways::generate_roller_coaster(editor, way);
			} else if (way.tags.contains("aeroway") ||
					   way.tags.contains("area:aeroway")) {
				highways::generate_aeroway(
						editor, way, args, building_footprints, flood_fill_cache);
			} else if (way.tags.get("service") ==
					   std::optional<std::string>(std::string("siding"))) {
				highways::generate_siding(editor, way, bridge_surface);
			} else if (way.tags.get("tomb") ==
					   std::optional<std::string>(std::string("pyramid"))) {
				historic::generate_pyramid(editor, way, args, flood_fill_cache);
			} else if (way.tags.contains("man_made")) {
				man_made::generate_man_made(
						editor, element, args, flood_fill_cache, building_footprints);
			} else if (way.tags.contains("power")) {
				power::generate_power(editor, element, building_footprints,
						flood_fill_cache, args.timeout);
				if (editor.signage_enabled())
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
				release_task_fills(task_index);
				return;
			}
			if (editor.signage_enabled())
				signage::generate_node_signage(
						editor, node, building_footprints, road_mask);

			if (node.tags.contains("door") || node.tags.contains("entrance")) {
				doors::generate_doors(editor, node);
			} else if (node.tags.contains("natural") &&
					   node.tags.get("natural") ==
							   std::optional<std::string>(std::string("tree"))) {
				natural::generate_natural(editor, element, args, flood_fill_cache,
						building_footprints, bridge_surface);
			} else if (attractions::claims(node.tags)) {
				attractions::generate_ride(editor, element, flood_fill_cache,
						building_footprints, road_mask, args.scale, args.timeout);
			} else if (man_made::is_summit_cross(node.tags)) {
				man_made::generate_summit_cross(editor, node, args);
			} else if (node.tags.get("amenity") == std::optional<std::string>("fuel")) {
				fuel_stations::generate_station(editor, element, flood_fill_cache,
						building_footprints, road_mask, args.scale, args.timeout);
			} else if (node.tags.contains("amenity")) {
				amenities::generate_amenities(
						editor, element, args, flood_fill_cache, road_mask);
			} else if (node.tags.contains("barrier")) {
				barriers::generate_barrier_nodes(editor, node, bridge_surface);
			} else if (node.tags.contains("highway") || highways::is_signal(node.tags)) {
				highways::generate_highways_internal(editor, element, args,
						highway_connectivity, &flood_fill_cache, args.timeout, road_mask,
						bridge_structures, bridge_surface, tunnel_portals, marking_index,
						tunnel_footprint.is_empty() ? nullptr : &tunnel_footprint);
			} else if (node.tags.get("aeroway") ==
					   std::optional<std::string>(std::string("helipad"))) {
				highways::generate_helipad_node(editor, node, args, building_footprints);
			} else if (node.tags.contains("tourism")) {
				tourisms::generate_tourisms(editor, node, road_mask);
			} else if (node.tags.contains("man_made")) {
				man_made::generate_man_made_nodes(
						editor, node, args, building_footprints);
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
				release_task_fills(task_index);
				return;
			}
			// Use the same ordered relation classification as Rust's process_element,
			// rather than maintaining a second chain of tag predicates here. This keeps
			// overlapping tags (for example building + natural) on one deterministic arm.
			if (const auto area_kind = element_processing::relation_area(rel.tags)) {
				switch (*area_kind) {
				case element_processing::RelationArea::Building:
					if (args.mode == GenerationMode::Blueprint) {
						blueprint::generate_relation_outline(editor, rel);
					} else {
						buildings::generate_building_from_relation(editor, rel, args,
								flood_fill_cache, clip_bbox, building_passages,
								interior_use_index ? &*interior_use_index : nullptr,
								&road_mask, &building_footprints, &building_group_members,
								&building_data.housing_relation_parts);
					}
					break;
				case element_processing::RelationArea::Water: {
					std::optional<int> surface_level;
					if (still_surfaces.has("relation", rel.id))
						surface_level = still_surfaces.get("relation", rel.id);
					water_areas::generate_water_areas_from_relation(editor, rel,
							clip_bbox, big_water_field, road_mask,
							tunnel_footprint.is_empty() ? nullptr : &tunnel_footprint,
							bridge_surface, surface_level);
					break;
				}
				case element_processing::RelationArea::Natural:
					natural::generate_natural_from_relation(editor, rel, args,
							flood_fill_cache, building_footprints, bridge_surface);
					break;
				case element_processing::RelationArea::Landuse:
					landuse::generate_landuse_from_relation(editor, rel, args,
							flood_fill_cache, building_footprints, road_mask,
							bridge_surface);
					break;
				case element_processing::RelationArea::Park:
					leisure::generate_leisure_from_relation(editor, rel, args,
							flood_fill_cache, building_footprints, bridge_surface);
					break;
				case element_processing::RelationArea::ManMade:
					man_made::generate_man_made(editor, ProcessedElement(rel), args,
							flood_fill_cache, building_footprints);
					break;
				}
			}
		}
		// Run after processing: relation processors may consume fills for their ways.
		release_task_fills(task_index);
	};
	auto process_element_with_context = [&](WorldEditor &target,
												ElementDispatchState &state,
												std::size_t task_index,
												std::size_t element_index) {
		try {
			process_assigned_element(target, state, task_index, element_index);
		} catch (const std::out_of_range &error) {
			const auto &element = elements[element_index];
			// Keep the failure visible but identify the OSM feature; the STL's
			// unordered_map::at message alone cannot point to the bad input.
			throw std::runtime_error("OSM " + std::string(element.kind()) + " " +
									 std::to_string(element.id()) + ": " + error.what());
		}
	};
	if (parallel_tile_workers) {
		using TileWorkerResult = std::pair<WorldEditor, ElementDispatchState>;
		bool tile_pipeline_ok = true;
		// Keep a bounded sliding window of tile jobs. Merge in the original
		// LPT/band order for deterministic first-writer semantics, but launch
		// the next tile as soon as its slot is freed so placement overlaps the
		// sequential merge instead of waiting for a whole batch to finish.
		const auto worker_count =
				std::min<std::size_t>(tile_batch_size, element_tiles.size());
		std::vector<std::future<TileWorkerResult>> workers(worker_count);
		auto launch_tile = [&](std::size_t slot, std::size_t tile_index) {
			// Snapshot the host editor on this thread: merging another completed
			// worker may mutate the host while the tile itself is being processed.
			auto tile_editor = editor.make_tile_worker();
			workers[slot] = std::async(std::launch::async,
					[&, tile_index, tile_editor = std::move(tile_editor)]() mutable {
						ElementDispatchState tile_state;
						if (!begin_generation_tile(tile_editor, tile_index))
							throw std::runtime_error("failed to begin Arnis tile worker");
						for (const auto element_index : element_assignments[tile_index])
							process_element_with_context(tile_editor, tile_state,
									element_index, element_index);
						// Rust releases a tile's cached fills as soon as its element pass
						// finishes, before ground work and before ordered merge can stall it.
						// The shared reader counts need the helper's lock across workers.
						release_tile_fills_for(tile_index);
						if (!finish_ground_tile(
									tile_editor, tile_state, tile_index, true))
							throw std::runtime_error(
									"failed to finish Arnis tile worker");
						return TileWorkerResult{
								std::move(tile_editor), std::move(tile_state)};
					});
		};
		try {
			for (std::size_t i = 0; i < worker_count; ++i)
				launch_tile(i, i);
			for (std::size_t tile_index = 0; tile_index < element_tiles.size();
					++tile_index) {
				const auto slot = tile_index % worker_count;
				if (!workers[slot].valid())
					throw std::runtime_error("missing Arnis tile worker result");
				auto [tile_editor, tile_state] = workers[slot].get();
				const auto &tile = element_tiles[tile_index];
				if (!editor.merge_tile_worker(std::move(tile_editor), tile.min_x,
							tile.min_z, tile.max_x - 1, tile.max_z - 1,
							eviction_active && !keep_roads_for_style
									? &deferred_write_regions
									: nullptr))
					throw std::runtime_error("failed to merge Arnis tile worker");
				collect_tile_dispatch_state(tile_state, tile);
				if (deferred_placements_active) {
					deferred_tiles_in_batch = 0;
					if (!complete_deferred_tile(tile))
						throw std::runtime_error(
								"failed to flush completed placement region");
				} else if (++deferred_tiles_in_batch >= tile_batch_size) {
					deferred_tiles_in_batch = 0;
					if (!editor.flush_deferred_tile_merges())
						throw std::runtime_error("failed to merge tile overlay batch");
				}
				// Match Rust's sliding window: only free a worker slot after its
				// result has crossed the ordered merge frontier.
				const auto next_tile = tile_index + worker_count;
				if (next_tile < element_tiles.size())
					launch_tile(slot, next_tile);
			}
		} catch (const std::exception &error) {
			editor.report_generation_error(error.what());
			std::cerr << "Arnis tile worker failed: " << error.what() << '\n';
			tile_pipeline_ok = false;
		} catch (...) {
			// Rust's AbortOnPanic guard wakes the tile pipeline when a worker
			// unwinds. Match that failure path for non-std C++ exceptions too, so
			// the cleanup below always discards partial overlays and releases the
			// generation pass instead of leaving a half-open tile transaction.
			constexpr const char *message = "unknown exception in Arnis tile worker";
			editor.report_generation_error(message);
			std::cerr << "Arnis tile worker failed: " << message << '\n';
			tile_pipeline_ok = false;
		}
		if (!tile_pipeline_ok) {
			for (auto &worker : workers)
				if (worker.valid())
					worker.wait();
			if (editor.mg)
				editor.mg->discardTileOverlay();
			editor.flush_deferred_tile_merges();
			editor.set_defer_tile_merges(false);
			editor.finish_tile_pass();
			return false;
		}
		next_ground_tile = element_tiles.size();
	} else {
		for (std::size_t task_index = 0; task_index < element_tasks.size();
				++task_index) {
			const auto [element_index, tile_index] = element_tasks[task_index];
			if (tile_index != active_tile) {
				if (active_tile != std::numeric_limits<std::size_t>::max())
					release_tile_fills_for(active_tile);
				if (regional_ground_passes &&
						active_tile != std::numeric_limits<std::size_t>::max()) {
					WorldEditor &tile_editor =
							active_tile_worker ? *active_tile_worker : editor;
					ElementDispatchState &tile_state =
							active_tile_worker ? active_tile_dispatch_state
											   : dispatch_state;
					if (!finish_ground_tile(tile_editor, tile_state, active_tile)) {
						std::cerr << "Failed to merge generated Arnis tile\n";
						editor.flush_deferred_tile_merges();
						editor.set_defer_tile_merges(false);
						editor.finish_tile_pass();
						return false;
					}
					if (active_tile_worker)
						collect_tile_dispatch_state(
								active_tile_dispatch_state, element_tiles[active_tile]);
					active_tile_worker.reset();
					next_ground_tile = active_tile + 1;
				}
				while (regional_ground_passes && next_ground_tile < tile_index) {
					bool finished = false;
					if (parallel_tile_workers) {
						auto empty_worker = editor.make_tile_worker();
						ElementDispatchState empty_state;
						finished =
								begin_generation_tile(empty_worker, next_ground_tile) &&
								finish_ground_tile(
										empty_worker, empty_state, next_ground_tile);
					} else {
						finished = begin_generation_tile(editor, next_ground_tile) &&
								   finish_ground_tile(
										   editor, dispatch_state, next_ground_tile);
					}
					if (!finished) {
						std::cerr << "Failed to generate or merge empty Arnis tile\n";
						editor.flush_deferred_tile_merges();
						editor.set_defer_tile_merges(false);
						editor.finish_tile_pass();
						return false;
					}
					++next_ground_tile;
				}
				active_tile = tile_index;
				if (tiled_element_dispatch) {
					const auto &tile = element_tiles[tile_index];
					if (regional_ground_passes) {
						bool began = false;
						if (parallel_tile_workers) {
							active_tile_worker.emplace(editor.make_tile_worker());
							active_tile_dispatch_state = {};
							began = begin_generation_tile(
									*active_tile_worker, tile_index);
						} else {
							began = begin_generation_tile(editor, tile_index);
						}
						if (!began) {
							std::cerr << "Failed to begin Arnis tile generation\n";
							editor.flush_deferred_tile_merges();
							editor.set_defer_tile_merges(false);
							editor.finish_tile_pass();
							return false;
						}
					} else {
						editor.set_strict_bounds(std::max(tile.min_x, xzbbox.min_x()),
								std::max(tile.min_z, xzbbox.min_z()),
								std::min(tile.max_x - 1, xzbbox.max_x()),
								std::min(tile.max_z - 1, xzbbox.max_z()));
					}
				} else {
					editor.clear_strict_bounds();
				}
			}
			if (active_tile_worker)
				process_element_with_context(*active_tile_worker,
						active_tile_dispatch_state, task_index, element_index);
			else
				process_element_with_context(
						editor, dispatch_state, task_index, element_index);
			if (!regional_ground_passes)
				report_generation_work(task_index + 1);
		}
		if (active_tile != std::numeric_limits<std::size_t>::max())
			release_tile_fills_for(active_tile);
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
					const auto &element = elements[element_index];
					if (element.is_node())
						signage::generate_node_facade_signage(editor, element.as_node(),
								building_footprints, road_mask);
				}
			}
			editor.clear_strict_bounds();
		} else {
			for (const auto &element : elements)
				if (element.is_node())
					signage::generate_node_facade_signage(
							editor, element.as_node(), building_footprints, road_mask);
		}
	}

	if (regional_ground_passes) {
		if (active_tile != std::numeric_limits<std::size_t>::max()) {
			WorldEditor &tile_editor = active_tile_worker ? *active_tile_worker : editor;
			ElementDispatchState &tile_state =
					active_tile_worker ? active_tile_dispatch_state : dispatch_state;
			if (!finish_ground_tile(tile_editor, tile_state, active_tile)) {
				std::cerr << "Failed to merge generated Arnis tile\n";
				editor.flush_deferred_tile_merges();
				editor.set_defer_tile_merges(false);
				editor.finish_tile_pass();
				return false;
			}
			if (active_tile_worker)
				collect_tile_dispatch_state(
						active_tile_dispatch_state, element_tiles[active_tile]);
			active_tile_worker.reset();
			next_ground_tile = active_tile + 1;
		}
		while (next_ground_tile < element_tiles.size()) {
			bool finished = false;
			if (parallel_tile_workers) {
				auto empty_worker = editor.make_tile_worker();
				ElementDispatchState empty_state;
				finished =
						begin_generation_tile(empty_worker, next_ground_tile) &&
						finish_ground_tile(empty_worker, empty_state, next_ground_tile);
			} else {
				finished = begin_generation_tile(editor, next_ground_tile) &&
						   finish_ground_tile(editor, dispatch_state, next_ground_tile);
			}
			if (!finished) {
				std::cerr << "Failed to generate or merge empty Arnis tile\n";
				editor.flush_deferred_tile_merges();
				editor.set_defer_tile_merges(false);
				editor.finish_tile_pass();
				return false;
			}
			++next_ground_tile;
		}
		if (!editor.flush_deferred_tile_merges()) {
			std::cerr << "Failed to merge generated Arnis tile batch\n";
			editor.set_defer_tile_merges(false);
			editor.finish_tile_pass();
			return false;
		}
		if (deferred_placements_active)
			execute_deferred_placements(deferred_placements.drain());
		editor.set_defer_tile_merges(false);
		editor.finish_tile_pass();
		editor.clear_strict_bounds();
	}
	progress::emit_gui_progress_update_ex(70.0, "", eviction_active);
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
			progress::emit_gui_progress_update(89.0, "Carving caves...");
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
			const caves::CaveShapeQuery cave_shape(editor, cave_world, cave_world,
					CAVE_SEED, world_editor::terrain_floor_y(), cave_ellipsoids);
			caves::generate_water_features(editor, cave_world, cave_world, CAVE_SEED,
					world_editor::terrain_floor_y(), args, cave_shape, &cave_basin_fluid,
					&cave_water_cells, &cave_air, &cave_water_plan);
			caves::generate_deep_lava_sea(editor, cave_world,
					world_editor::terrain_floor_y(), cave_air, cave_water_cells,
					&cave_basin_fluid);
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
					xzbbox.min_z(), xzbbox.max_z(), true);
		}
	}
	// Cave geometry is only needed while carving and decorating the cave pass;
	// release it before the later model/landmark work, as Rust does when that
	// phase's local state goes out of scope.
	cave_ellipsoids = {};

	if (!regional_ground_passes)
		water_depth::carve_lc_water_pass(
				editor, big_water_field, road_mask, tunnel_footprint, &bridge_surface);
	if (!regional_ground_passes)
		highways::drop_lamps_in_water(editor);
	// Keep the surface mask alive when the post-generation style pass needs to
	// distinguish roads/paving from natural ground; otherwise release it before
	// model placement, as Rust does after the ground/ore/water passes.
	const bool apply_world_style = world_style::is_active(args);
	if (!apply_world_style)
		editor.release_sealed_surface();
	// Tunnel interiors must be reopened after underground fill, but before
	// models: a late carve can cut holes into an already placed landmark.
	if (!(regional_ground_passes && eviction_active)) {
		std::vector<highways::TunnelLight> tunnel_lights;
		if (!dispatch_state.rail_tunnel_points.empty())
			tunnel_lights = railways::carve_rail_tunnel_interior(
					editor, dispatch_state.rail_tunnel_points);
		if (!dispatch_state.highway_tunnel_cells.empty()) {
			auto road_lights = highways::carve_highway_tunnel_interior(
					editor, dispatch_state.highway_tunnel_cells);
			tunnel_lights.insert(
					tunnel_lights.end(), road_lights.begin(), road_lights.end());
		}
		highways::hang_tunnel_lights(editor, tunnel_lights);
	}
	if (args.caves && !(regional_ground_passes && eviction_active))
		caves::seal_floating_fluid_region(editor,
				{xzbbox.min_x(), xzbbox.max_x(), xzbbox.min_z(), xzbbox.max_z()},
				world_editor::terrain_floor_y());
	// Match Rust's final terrain-water pass: cave fluids have settled, but
	// deferred models and world styling have not yet changed the surface.
	if (deferred_placements_active && args.benchmark)
		bench.report("water_steps", std::chrono::nanoseconds(water_steps_elapsed_ns.load(
											std::memory_order_relaxed)));
	if (java_world && !args.winter && !deferred_placements_active) {
		const auto water_steps_start = Bench::Clock::now();
		water_steps::shape_world(editor, args.caves);
		const auto elapsed = Bench::Clock::now() - water_steps_start;
		bench.report("water_steps", elapsed);
		bench.exclude(elapsed);
	}
	// Rust drops the generation-only indexes and masks before model placement.
	// Keep the same lifetime boundary here: these can scale with the full area,
	// but every consumer (including regional tree placement and tunnel carving)
	// has finished by this point.
	editor.set_regional_tree_placer({});
	editor.set_mapped_regional_tree_placer({});
	editor.tree_crown_columns = {};
	editor.mapped_trunks.reset();
	shared_selector.reset();
	// The barrier index shares the road bitmap. Release it first, then drop the
	// road mask and the building-interior index, whose last consumers were the
	// completed element/tile passes above.
	barrier_clearance = {};
	interior_use_index.reset();
	road_mask_owner.reset();
	editor.traffic_signal_index.reset();
	editor.car_network.reset();
	big_water_field = {};
	tunnel_footprint = {};
	rail_mask = {};
	building_passages = {};
	highway_connectivity = {};
	marking_index = {};
	still_surfaces = {};
	decltype(tunnel_internal_endpoints){}.swap(tunnel_internal_endpoints);
	tunnel_portals = {};
	bridge_outlines = {};
	bridge_structures = {};
	bridge_surface = {};
	decltype(dispatch_state.rail_tunnel_points){}.swap(dispatch_state.rail_tunnel_points);
	decltype(dispatch_state.highway_tunnel_cells){}.swap(
			dispatch_state.highway_tunnel_cells);
	filtered_building_footprints.reset();
	building_part_groups = {};
	building_group_members = {};
	std::vector<TileBounds>{}.swap(element_tiles);
	decltype(element_assignments){}.swap(element_assignments);
	decltype(element_tasks){}.swap(element_tasks);
	decltype(dispatch_order){}.swap(dispatch_order);
	decltype(dispatch_rank){}.swap(dispatch_rank);
	tile_fill_reader_state.reset();
	expiring_fills = {};
	// No element pass reads signage placement masks after this point. Drop the
	// large carriageway bitmap before 3D models and finalization, as Rust does.
	auto signage_context = editor.take_signage_context();
	if (signage_context)
		signage_context->release_placement_masks();
	editor.set_signage_context(std::move(signage_context));
	if (model_pipeline && !deferred_placements_active) {
		model_pipeline->place(three_dmr_provider, wikidata_provider,
				custom_model_provider, editor, args.scale);
	}
	// Rust places plane models after the other 3D providers and immediately
	// before landmarks; preserve that ordering for overlap and suppression.
	if (!deferred_placements_active) {
		structures::plane::place_plane_placements(editor, plane_candidates);
		landmarks::place_all(editor, landmark_plan, args.scale, args.rotation);
	}

	railways::advtrains::finish_network(editor);
	// Rust styles each completed region before facade panels are finalized or
	// flushed. Keep that ordering so generated panels remain visible over winter
	// snow and post-apocalyptic block substitutions.
	if (deferred_placements_active)
		world_style::apply_remaining(editor, args, building_footprints, styled_regions);
	else
		world_style::apply(editor, args, building_footprints);
	if (apply_world_style)
		editor.release_sealed_surface();
	// The final style pass is the last consumer of road elevations. Rust drops
	// them per region after styling; this non-streaming editor releases the
	// equivalent generation-only index at the same lifetime edge.
	editor.release_road_surface_overrides();
	// Resolve preset facade candidates only after all neighbouring building
	// blocks exist; party-wall panels are cropped from their finished rooflines.
	// The enabled registry preflights both sources together and flushes Mapillary
	// before writing preset panels, preserving Rust's display ordering.
	building_facades::finalize(editor);
	// Without a preset registry there was no joint preflight to flush photos.
	if (!building_facades::enabled())
		mapillary::facades::finalize_displays(editor);
	// Mark the completed generation for the format-specific persistence layer.
	// Java/Bedrock/Luanti writers consume these lifecycle requests when wired by
	// their respective WorldEditor backends.
	progress::emit_gui_progress_update(97.0, "Finalizing world...");
	editor.request_flush();
	editor.request_save();
	if (!editor.finalize_persistence()) {
		editor.report_generation_error("backend persistence finalization failed");
		std::cerr << "Arnis generation failed while finalizing backend persistence\n";
		return false;
	}
	if (java_world && !java_world_path.empty()) {
		std::string error;
		if (!mapillary::displays::write_world_resource_pack(editor, java_world_path,
					args.pause_menu && !args.one_world_run, &error))
			std::cerr << "Warning: Failed to write facade resource pack: " << error
					  << '\n';
		mapillary::facades::clear();
	}
	if (args.one_world_run) {
		const auto &run = *args.one_world_run;
		// Rust moves the Java spawn only when an extension includes an explicit
		// marker. Derive Y from the generated ground, not the prior world's level.
		if (run.extending && spawn_point &&
				editor.get_generation_format() ==
						static_cast<int>(WorldFormat::JavaAnvil) &&
				spawn_point->first >= xzbbox.min_x() &&
				spawn_point->first <= xzbbox.max_x() &&
				spawn_point->second >= xzbbox.min_z() &&
				spawn_point->second <= xzbbox.max_z()) {
			const auto ground_point =
					editor.ground_point(spawn_point->first, spawn_point->second);
			const int spawn_y = (editor.ground ? editor.ground->level(ground_point)
											   : args.ground_level) +
								3;
			if (world_utils::set_spawn_in_level_dat(run.world_dir, spawn_point->first,
						spawn_y, spawn_point->second)) {
				std::cout << "Spawn point moved to " << spawn_point->first << ", "
						  << spawn_y << ", " << spawn_point->second << ".\n";
			} else {
				std::cerr << "Warning: Failed to move the Java world spawn point\n";
			}
		}
		const auto preview_path = run.preview_path();
		std::error_code preview_error;
		const auto preview = std::filesystem::is_regular_file(preview_path, preview_error)
									 ? std::optional<std::filesystem::path>(preview_path)
									 : std::nullopt;
		const auto geographic_bounds = editor.geographic_bounds();
		try {
			const geographic::LLBBox llbbox(geographic_bounds[0], geographic_bounds[2],
					geographic_bounds[1], geographic_bounds[3]);
			const auto manifest_bbox = cartesian::XZBBox::rect_from_min_max(
					xzbbox.min_x(), xzbbox.min_z(), xzbbox.max_x(), xzbbox.max_z());
			std::string error;
			const auto affine =
					editor.ground ? editor.ground->elevation_affine() : std::nullopt;
			if (!one_world::record_area(run, llbbox, manifest_bbox,
						affine ? std::optional<one_world::ElevationAffine>(
										 one_world_elevation(*affine))
							   : std::nullopt,
						preview, ARNIS_PACKAGE_VERSION, &error)) {
				const auto message =
						"The area was written but could not be recorded: " + error;
				editor.report_generation_error(message);
				std::cerr << "Arnis generation error: " << message << '\n';
				return false;
			}
		} catch (const std::exception &error) {
			const auto message =
					std::string{"The area was written but could not be recorded: "} +
					error.what();
			editor.report_generation_error(message);
			std::cerr << "Arnis generation error: " << message << '\n';
			return false;
		}
	}
	if (args.one_world_run) {
		std::string error;
		if (!world_utils::touch_last_played(args.one_world_run->world_dir, &error))
			std::cerr << "Warning: Failed to update LastPlayed: " << error << '\n';
	}
	if (java_world && !extending_world && !java_world_path.empty()) {
		std::string error;
		if (!world_utils::apply_java_world_settings(java_world_path,
					java_game_type(args.gamemode), args.world_time,
					args.world_type == WorldType::Void, world_editor::base_chunk_y(),
					world_editor::min_y(), &error))
			std::cerr << "Warning: Failed to apply Java world settings: " << error
					  << '\n';
	}
	// Rust installs this pack whenever extended Java heights are selected. The
	// generated sections are otherwise outside the vanilla dimension's bounds
	// and are discarded or inaccessible when Minecraft opens the world.
	if (java_world && extended_world_height && !extending_world &&
			!java_world_path.empty()) {
		std::string error;
		if (!world_utils::install_tall_datapack(java_world_path, &error)) {
			const auto message = "failed to install tall-world datapack: " + error;
			editor.report_generation_error(message);
			std::cerr << "Arnis generation error: " << message << '\n';
			return false;
		}
	}
	if (java_world && !args.one_world_run && !java_world_path.empty()) {
		const auto red_amber_region =
				decals::detect_region((geo_bounds[0] + geo_bounds[1]) * .5,
						(geo_bounds[2] + geo_bounds[3]) * .5);
		const bool red_amber = red_amber_region == decals::SignRegion::Germanic ||
							   red_amber_region == decals::SignRegion::UkIreland;
		std::vector<datapack::discoveries::Place> extra_places;
		std::optional<std::string> settlement;
		if (args.discoveries || args.pause_menu || args.zombie_outbreak) {
			const auto discovery_bbox = cartesian::XZBBox::rect_from_min_max(
					xzbbox.min_x(), xzbbox.min_z(), xzbbox.max_x(), xzbbox.max_z());
			auto collected = datapack::discoveries::collect(elements, discovery_bbox);
			extra_places = std::move(collected.first);
			settlement = std::move(collected.second);
		}
		std::string area = std::string(celestial_body_display_name(args.body));
		if (is_earth(args.body)) {
			area.clear();
			if (const auto name = world_utils::level_name(java_world_path)) {
				if (const auto separator = name->find(": ");
						separator != std::string::npos) {
					area = name->substr(separator + 2);
					const auto first = area.find_first_not_of(" \t\r\n");
					if (first == std::string::npos) {
						area.clear();
					} else {
						const auto last = area.find_last_not_of(" \t\r\n");
						area = area.substr(first, last - first + 1);
					}
				}
			}
			if (area.empty()) {
				const double latitude = (geo_bounds[0] + geo_bounds[1]) * .5;
				const double longitude = (geo_bounds[2] + geo_bounds[3]) * .5;
				area = retrieve_data::fetch_area_name(latitude, longitude)
							   .value_or(settlement.value_or("this area"));
			}
		}
		std::string error;
		std::optional<datapack::SpawnDestination> extra_spawn;
		auto spawn_xz = spawn_point;
		if (!spawn_xz)
			spawn_xz = world_utils::level_spawn_xz(java_world_path);
		if (spawn_xz) {
			const auto point = editor.ground_point(spawn_xz->first, spawn_xz->second);
			const std::optional<int> spawn_ground_y =
					editor.ground ? std::optional<int>(editor.ground->level(point))
								  : std::optional<int>(args.ground_level);
			extra_spawn = datapack::SpawnDestination{
					spawn_xz->first, spawn_xz->second, spawn_ground_y};
		}
		bool disaster_wooded = false;
		std::vector<std::pair<std::string, std::uint64_t>> disaster_biomes;
		std::optional<int> disaster_highest_surface;
		std::optional<datapack::disasters::Surface> disaster_surface;
		std::optional<int> disaster_sea_y;
		if (args.disasters) {
			disaster_wooded = std::any_of(elements.begin(), elements.end(),
					[](const ProcessedElement &element) {
						const auto &tags = element.tags();
						const auto landuse = tags.find("landuse");
						const auto natural = tags.find("natural");
						return (landuse != tags.end() && landuse->second == "forest") ||
							   (natural != tags.end() &&
									   (natural->second == "wood" ||
											   natural->second == "scrub" ||
											   natural->second == "heath"));
					});
			if (!disaster_wooded && editor.ground && editor.ground->has_land_cover()) {
				constexpr int SIDE = 64;
				const auto width = std::int64_t(xzbbox.max_x()) - xzbbox.min_x();
				const auto height = std::int64_t(xzbbox.max_z()) - xzbbox.min_z();
				int tree_samples = 0;
				for (int iz = 0; iz < SIDE; ++iz) {
					for (int ix = 0; ix < SIDE; ++ix) {
						const int x = xzbbox.min_x() +
									  static_cast<int>(width * ix / (SIDE - 1));
						const int z = xzbbox.min_z() +
									  static_cast<int>(height * iz / (SIDE - 1));
						if (editor.ground->cover_class(editor.ground_point(x, z)) ==
								land_cover::LC_TREE_COVER)
							++tree_samples;
					}
				}
				disaster_wooded = double(tree_samples) / (SIDE * SIDE) >= 0.03;
			}
			if (args.body == CelestialBody::Earth && editor.ground) {
				disaster_surface.emplace(
						xzbbox.min_x(), xzbbox.min_z(), xzbbox.max_x(), xzbbox.max_z());
				std::unordered_map<content_t, datapack::disasters::SurfaceClass> classes;
				// ARNIS-PORT: datapack/disasters.rs Surface::record_chunk. WorldEditor
				// has no serialized Java-section callback, so collect the equivalent
				// committed surface from its written-cell sets at this boundary.
				auto record_surface = [&](const WorldEditor::WrittenCellSet &cells) {
					for (const auto &[x, y, z] : cells) {
						if (!disaster_surface->is_sample(x, z))
							continue;
						const auto block = editor.get_block_absolute(x, y, z);
						if (!block)
							continue;
						const auto content = block->getContent();
						auto found = classes.find(content);
						if (found == classes.end()) {
							const auto name = editor.block_name(*block);
							found = classes.emplace(content,
												   datapack::disasters::Surface::classify(
														   name.value_or(std::string{})))
											.first;
						}
						disaster_surface->observe(x, y, z, found->second);
					}
				};
				record_surface(editor.written_cells);
				record_surface(editor.tile_merged_written_cells);
				disaster_highest_surface = disaster_surface->highest();
				if (const auto affine = editor.ground->elevation_affine())
					disaster_sea_y =
							static_cast<int>(std::lround(affine->y_for_metres(0.0)));

				auto chunk_of = [](int coordinate) {
					return coordinate >= 0 ? coordinate / 16 : -((-coordinate + 15) / 16);
				};
				std::map<std::string, std::uint64_t> counts;
				const int min_cx = chunk_of(xzbbox.min_x());
				const int max_cx = chunk_of(xzbbox.max_x());
				const int min_cz = chunk_of(xzbbox.min_z());
				const int max_cz = chunk_of(xzbbox.max_z());
				for (int cz = min_cz; cz <= max_cz; ++cz) {
					for (int cx = min_cx; cx <= max_cx; ++cx) {
						for (const auto &name :
								biome::chunk_biome_names(cx, cz, editor.ground,
										centre_lat, ground_origin_x, ground_origin_z))
							++counts[name];
					}
				}
				disaster_biomes.assign(counts.begin(), counts.end());
			}
		}
		if (!datapack::install_runtime_extras(
					java_world_path, args, area, red_amber, editor.car_network.get(),
					editor.train_network.get(),
					[&editor](std::uint64_t way, int x, int z) {
						return editor.rail_height(way, x, z);
					},
					extra_places, extra_spawn, &error, disaster_wooded, disaster_biomes,
					disaster_highest_surface,
					disaster_surface ? &*disaster_surface : nullptr, disaster_sea_y)) {
			const auto message = "failed to install gameplay datapack: " + error;
			editor.report_generation_error(message);
			std::cerr << "Arnis generation error: " << message << '\n';
			return false;
		}
	}
	if (editor.signage_context) {
		std::cout << editor.signage_context->summary(args.debug) << '\n';
	}
	progress::emit_gui_progress_update(99.5, "Finalizing world...");

	return true;
}

bool generate_world_with_options(WorldEditor &editor,
		const std::vector<ProcessedElement> &elements, Args args,
		FloodFillCache &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const GenerationOptions &options, bool elements_prepared)
{
	if (!valid_generation_options(options, args))
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
	if (!facade_job)
		facade_job = start_mapillary_facade_job(args, geo);
	return generate_world(editor, elements, args, flood_fill_cache, building_footprints,
			elements_prepared,
			options.prepared_buildings ? &*options.prepared_buildings : nullptr,
			facade_job.get(), options.spawn_point);
}

}
