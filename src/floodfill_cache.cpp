#include "floodfill_cache.h"
#include "floodfill.h"
#include "clipping.h"
#include "element_processing/way_segments.h"
#include "element_processing/relation_area.h"
#include "bresenham.h"
#include "strict_parse.h"
#include <algorithm>
#include <span>
#include <atomic>
#include <charconv>
#include <thread>
#include <future>
#include <numeric>
#include <cmath>
#include <stdexcept>
#include <cstdlib>
#include <exception>
#include <semaphore>
#include <unordered_set>
#include <iterator>
#include <string_view>
#include <utility>

namespace arnis
{

FloodFillCache::FloodFillCache(FloodFillCache &&other) noexcept
{
	std::lock_guard<std::mutex> lock(other.way_cache_mutex);
	way_cache = std::move(other.way_cache);
	retain_entries_ = other.retain_entries_;
	timeout_ = other.timeout_;
}

FloodFillCache &FloodFillCache::operator=(FloodFillCache &&other) noexcept
{
	if (this == &other)
		return *this;
	std::scoped_lock lock(way_cache_mutex, other.way_cache_mutex);
	way_cache = std::move(other.way_cache);
	retain_entries_ = other.retain_entries_;
	timeout_ = other.timeout_;
	return *this;
}

namespace
{

std::atomic_size_t configured_workers{0};

constexpr std::size_t PREPASS_BATCH_CELLS = std::size_t{1} << 22;
constexpr std::size_t PREPASS_BATCH = 512;
constexpr std::ptrdiff_t MAX_PREPASS_WORKER_SLOTS = 256;

std::counting_semaphore<MAX_PREPASS_WORKER_SLOTS> &prepass_worker_slots()
{
	static std::counting_semaphore<MAX_PREPASS_WORKER_SLOTS> slots([] {
		const auto hardware = std::max(1u, std::thread::hardware_concurrency());
		return static_cast<std::ptrdiff_t>(
				std::min(hardware - 1, static_cast<unsigned>(MAX_PREPASS_WORKER_SLOTS)));
	}());
	return slots;
}

template <typename Function>
void parallel_for_indices(std::size_t count, Function function)
{
	if (count == 0)
		return;
	const auto configured = configured_workers.load(std::memory_order_relaxed);
	const auto workers = std::min(
			count, configured != 0 ? configured
								   : std::max(1u, std::thread::hardware_concurrency()));
	if (workers <= 1) {
		for (std::size_t i = 0; i < count; ++i)
			function(i);
		return;
	}

	std::atomic_size_t next{0};
	std::mutex error_mutex;
	std::exception_ptr error;
	auto &worker_slots = prepass_worker_slots();
	std::size_t acquired_slots = 0;
	while (acquired_slots + 1 < workers && worker_slots.try_acquire())
		++acquired_slots;
	auto work = [&] {
		while (true) {
			const auto i = next.fetch_add(1, std::memory_order_relaxed);
			if (i >= count)
				return;
			try {
				function(i);
			} catch (...) {
				{
					std::lock_guard lock(error_mutex);
					if (!error)
						error = std::current_exception();
				}
				return;
			}
		}
	};
	std::vector<std::thread> threads;
	std::size_t launched = 0;
	try {
		for (; launched < acquired_slots; ++launched)
			threads.emplace_back([&] {
				work();
				worker_slots.release();
			});
	} catch (...) {
		for (std::size_t i = launched; i < acquired_slots; ++i)
			worker_slots.release();
		work();
		for (auto &thread : threads)
			thread.join();
		throw;
	}
	work();
	for (auto &thread : threads)
		thread.join();
	if (error)
		std::rethrow_exception(error);
}

std::size_t thread_count_for_fraction(double cpu_fraction)
{
	if (const char *override_value = std::getenv("RAYON_NUM_THREADS")) {
		const std::string_view value(override_value);
		std::size_t requested = 0;
		const auto [end, error] =
				std::from_chars(value.data(), value.data() + value.size(), requested);
		if (error == std::errc{} && end == value.data() + value.size() && requested > 0)
			return requested;
	}
	cpu_fraction = std::clamp(cpu_fraction, 0.1, 1.0);
	const auto available = std::max(1u, std::thread::hardware_concurrency());
	return std::max<std::size_t>(
			1, static_cast<std::size_t>(static_cast<double>(available) * cpu_fraction));
}

FloodFillResult empty_flood_fill_result()
{
	static const auto empty =
			std::make_shared<const std::vector<std::pair<int32_t, int32_t>>>();
	return empty;
}

bool relation_fills_members(const ProcessedRelation &relation)
{
	const auto area = element_processing::relation_area(relation.tags);
	return area && element_processing::relation_area_fills_members(*area, relation.tags);
}

} // namespace

InnerRings::InnerRings(const ProcessedRelation &relation)
{
	for (const auto &member : relation.members) {
		if (member.role != ProcessedMemberRole::Inner || !member.way ||
				member.way->nodes.empty())
			continue;
		Entry entry{member.way.get(), member.way->nodes.front().x,
				member.way->nodes.front().z, member.way->nodes.front().x,
				member.way->nodes.front().z};
		for (const auto &node : member.way->nodes) {
			entry.min_x = std::min(entry.min_x, node.x);
			entry.min_z = std::min(entry.min_z, node.z);
			entry.max_x = std::max(entry.max_x, node.x);
			entry.max_z = std::max(entry.max_z, node.z);
		}
		rings.push_back(entry);
	}
}

bool in_hole(const Holes *holes, int32_t x, int32_t z)
{
	return holes && holes->contains(x, z);
}

bool edge_in_hole(const Holes *holes, int32_t x, int32_t z)
{
	return holes && holes->covers(x, z);
}

std::vector<std::pair<int32_t, int32_t>> without_holes(
		const std::vector<std::pair<int32_t, int32_t>> &cells, const Holes *holes)
{
	if (!holes)
		return cells;
	std::vector<std::pair<int32_t, int32_t>> result;
	result.reserve(cells.size());
	for (const auto &point : cells)
		if (!holes->contains(point.first, point.second))
			result.push_back(point);
	return result;
}

FloodFillResult without_holes(const FloodFillResult &cells, const Holes *holes)
{
	if (!cells)
		return empty_flood_fill_result();
	if (!holes)
		return cells;
	auto result = std::make_shared<std::vector<std::pair<int32_t, int32_t>>>();
	result->reserve(cells->size());
	for (const auto &point : *cells)
		if (!holes->contains(point.first, point.second))
			result->push_back(point);
	return result;
}

std::optional<Holes> FloodFillCache::holes_in(const ProcessedWay &outer,
		const InnerRings &inners,
		const std::optional<std::chrono::milliseconds> &timeout) const
{
	if (outer.nodes.empty() || inners.rings.empty())
		return std::nullopt;
	int32_t ox0 = outer.nodes.front().x, ox1 = ox0;
	int32_t oz0 = outer.nodes.front().z, oz1 = oz0;
	for (const auto &node : outer.nodes) {
		ox0 = std::min(ox0, node.x);
		ox1 = std::max(ox1, node.x);
		oz0 = std::min(oz0, node.z);
		oz1 = std::max(oz1, node.z);
	}
	// Keep borrowed node rings as Rust does instead of copying all member
	// coordinates just to classify which inner rings cut this outer.
	std::vector<std::span<const ProcessedNode>> inner_rings;
	std::vector<const ProcessedWay *> inner_ways;
	for (const auto &entry : inners.rings) {
		if (entry.min_x > ox1 || entry.max_x < ox0 || entry.min_z > oz1 ||
				entry.max_z < oz0)
			continue;
		inner_rings.emplace_back(entry.way->nodes);
		inner_ways.push_back(entry.way);
	}
	if (inner_rings.empty())
		return std::nullopt;
	const auto groups = element_processing::holes_by_outer(
			std::vector<std::span<const ProcessedNode>>{
					std::span<const ProcessedNode>(outer.nodes)},
			inner_rings);
	if (groups.empty() || groups.front().empty())
		return std::nullopt;
	int32_t x0 = ox1, z0 = oz1, x1 = ox0, z1 = oz0;
	for (const auto index : groups.front()) {
		const auto &entry = inner_ways[index]->nodes;
		for (const auto &node : entry) {
			x0 = std::min(x0, node.x);
			z0 = std::min(z0, node.z);
			x1 = std::max(x1, node.x);
			z1 = std::max(z1, node.z);
		}
	}
	x0 = std::max(x0, ox0);
	z0 = std::max(z0, oz0);
	x1 = std::min(x1, ox1);
	z1 = std::min(z1, oz1);
	if (x1 < x0 || z1 < z0)
		return std::nullopt;
	XZBBox bounds(x0, z0, x1, z1);
	Holes holes{CoordinateBitmap(bounds), CoordinateBitmap(bounds)};
	for (const auto index : groups.front()) {
		const auto *inner = inner_ways[index];
		for (const auto &[x, z] : *get_or_compute(*inner, timeout))
			holes.cells.set(x, z);
		for (std::size_t i = 1; i < inner->nodes.size(); ++i)
			for (const auto &[x, y, z] :
					bresenham_line(inner->nodes[i - 1].x, 0, inner->nodes[i - 1].z,
							inner->nodes[i].x, 0, inner->nodes[i].z)) {
				(void)y;
				holes.edges.set(x, z);
			}
	}
	if (holes.cells.is_empty() && holes.edges.is_empty())
		return std::nullopt;
	return holes;
}

namespace
{

template <typename Tags>
bool is_underground_building(const Tags &tags)
{
	if (const auto location = tags.find("location"); location != tags.end()) {
		if (location->second == "underground" || location->second == "subway")
			return true;
		if (location->second == "surface" || location->second == "overground" ||
				location->second == "roof")
			return false;
	}
	auto negative_tag = [&tags](const char *key) {
		auto it = tags.find(key);
		if (it == tags.end())
			return false;
		const auto level = strict_parse::i32(it->second);
		return level && *level < 0;
	};
	if (negative_tag("layer") || negative_tag("level"))
		return true;
	return tags.contains("building:levels:underground") &&
		   !tags.contains("building:levels");
}

template <typename Apply>
void for_relation_ring_cells(const ProcessedRelation &relation, ProcessedMemberRole role,
		const XZBBox &xzbbox, Apply apply)
{
	std::vector<std::vector<ProcessedNode>> rings;
	for (const auto &member : relation.members) {
		if (member.role == role)
			rings.push_back(member.way->nodes);
	}
	merge_way_segments(rings);

	for (auto &ring : rings) {
		ring = clipping::clip_way_to_bbox_owned(std::move(ring), xzbbox);
		if (ring.size() < 4)
			continue;
		// Segment matching tolerates nearby endpoints, but the rasterizer needs
		// an exactly closed coordinate ring after clipping, as in Rust.
		if (ring.front().x != ring.back().x || ring.front().z != ring.back().z) {
			if (std::abs(static_cast<int64_t>(ring.front().x) - ring.back().x) > 1 ||
					std::abs(static_cast<int64_t>(ring.front().z) - ring.back().z) > 1)
				continue;
			ring.push_back(ring.front());
		}
		std::vector<std::pair<int32_t, int32_t>> coords;
		coords.reserve(ring.size());
		for (const auto &node : ring)
			coords.emplace_back(node.x, node.z);
		for (const auto &[x, z] : flood_fill_area(coords, nullptr))
			apply(x, z);
	}
}

} // namespace

bool is_oversized_ring(const ProcessedWay &way)
{
	if (way.nodes.size() < 4 || way.nodes.front().x != way.nodes.back().x ||
			way.nodes.front().z != way.nodes.back().z)
		return false;
	int32_t min_x = std::numeric_limits<int32_t>::max();
	int32_t max_x = std::numeric_limits<int32_t>::min();
	int32_t min_z = std::numeric_limits<int32_t>::max();
	int32_t max_z = std::numeric_limits<int32_t>::min();
	for (const auto &node : way.nodes) {
		min_x = std::min(min_x, node.x);
		max_x = std::max(max_x, node.x);
		min_z = std::min(min_z, node.z);
		max_z = std::max(max_z, node.z);
	}
	const int64_t area = (static_cast<int64_t>(max_x) - min_x + 1) *
						 (static_cast<int64_t>(max_z) - min_z + 1);
	return area > floodfill::MAX_FLOOD_FILL_AREA;
}

CoordinateBitmap::CoordinateBitmap(const XZBBox &xzbbox)
{
	min_x_ = xzbbox.min_x();
	min_z_ = xzbbox.min_z();
	// Use int64_t to avoid overflow when world spans more than int32_t::MAX in either dimension
	width_ = static_cast<size_t>(
			static_cast<int64_t>(xzbbox.max_x()) - static_cast<int64_t>(min_x_) + 1);
	height_ = static_cast<size_t>(
			static_cast<int64_t>(xzbbox.max_z()) - static_cast<int64_t>(min_z_) + 1);

	// Match Rust's checked width*height calculation rather than allowing size_t
	// overflow to produce a truncated bitmap and out-of-bounds bit accesses.
	if (height_ != 0 && width_ > std::numeric_limits<size_t>::max() / height_)
		throw std::length_error("CoordinateBitmap: world size too large");
	// Calculate number of bytes needed (round up to nearest byte).
	size_t total_bits = width_ * height_;
	if (total_bits > std::numeric_limits<size_t>::max() - 7)
		throw std::length_error("CoordinateBitmap: bitmap size too large");
	size_t num_bytes = (total_bits + 7) / 8; // Ceiling division

	bits_.resize(num_bytes, 0);
	count_ = 0;
}

void CoordinateBitmap::set(int32_t x, int32_t z)
{
	// Use int64_t arithmetic to avoid overflow when coordinates span large ranges
	int64_t local_x = static_cast<int64_t>(x) - static_cast<int64_t>(min_x_);
	int64_t local_z = static_cast<int64_t>(z) - static_cast<int64_t>(min_z_);

	if (local_x < 0 || local_z < 0) {
		return;
	}

	size_t local_x_sz = static_cast<size_t>(local_x);
	size_t local_z_sz = static_cast<size_t>(local_z);

	if (local_x_sz >= width_ || local_z_sz >= height_) {
		return;
	}

	// Safe: bounds checks above ensure this won't overflow (max = total_bits - 1)
	size_t bit_index = local_z_sz * width_ + local_x_sz;
	size_t byte_index = bit_index / 8;
	size_t bit_offset = bit_index % 8;

	// Safety: bounds checks above ensure byte_index is always valid
	uint8_t mask = static_cast<uint8_t>(1U << bit_offset);
	// Only increment count if bit wasn't already set
	if ((bits_[byte_index] & mask) == 0) {
		bits_[byte_index] |= mask;
		++count_;
	}
}

void CoordinateBitmap::clear(int32_t x, int32_t z)
{
	const int64_t local_x = static_cast<int64_t>(x) - min_x_;
	const int64_t local_z = static_cast<int64_t>(z) - min_z_;
	if (local_x < 0 || local_z < 0 || static_cast<size_t>(local_x) >= width_ ||
			static_cast<size_t>(local_z) >= height_)
		return;
	const size_t bit_index =
			static_cast<size_t>(local_z) * width_ + static_cast<size_t>(local_x);
	const size_t byte_index = bit_index / 8;
	const uint8_t mask = static_cast<uint8_t>(1U << (bit_index % 8));
	if (bits_[byte_index] & mask) {
		bits_[byte_index] &= static_cast<uint8_t>(~mask);
		--count_;
	}
}

bool CoordinateBitmap::contains(int32_t x, int32_t z) const
{
	// Use int64_t arithmetic to avoid overflow when coordinates span large ranges
	int64_t local_x = static_cast<int64_t>(x) - static_cast<int64_t>(min_x_);
	int64_t local_z = static_cast<int64_t>(z) - static_cast<int64_t>(min_z_);

	if (local_x < 0 || local_z < 0) {
		return false;
	}

	size_t local_x_sz = static_cast<size_t>(local_x);
	size_t local_z_sz = static_cast<size_t>(local_z);

	if (local_x_sz >= width_ || local_z_sz >= height_) {
		return false;
	}

	// Safe: bounds checks above ensure this won't overflow (max = total_bits - 1)
	size_t bit_index = local_z_sz * width_ + local_x_sz;
	size_t byte_index = bit_index / 8;
	size_t bit_offset = bit_index % 8;

	// Safety: bounds checks above ensure byte_index is always valid
	return ((bits_[byte_index] >> bit_offset) & 1U) == 1U;
}

bool CoordinateBitmap::is_unset_in_bounds(int32_t x, int32_t z) const
{
	const auto lx = static_cast<int64_t>(x) - min_x_;
	const auto lz = static_cast<int64_t>(z) - min_z_;
	return lx >= 0 && lz >= 0 && static_cast<uint64_t>(lx) < width_ &&
		   static_cast<uint64_t>(lz) < height_ && !contains(x, z);
}

bool WorldEditor::surface_is_sealed(int x, int z) const
{
	return sealed_surface && sealed_surface->contains(x, z);
}

bool WorldEditor::nested_area_owns(int x, int z) const
{
	// The merged mask clears paving where a smaller planted area takes
	// precedence. Those cleared cells belong to the nested area, not this fill.
	return sealed_surface && !sealed_surface->contains(x, z);
}

std::optional<SealedSurfaceBitmap> FloodFillCache::collect_sealed_surfaces(
		const std::vector<ProcessedElement> &elements, const RoadMaskBitmap &roads) const
{
	static const std::unordered_set<std::string> sealed_leisure = {"pitch", "track",
			"playground", "recreation_ground", "schoolyard", "ice_rink", "water_park",
			"slipway", "outdoor_seating", "bathing_place", "fitness_station"};
	static const std::unordered_set<std::string> sealed_amenity = {"parking",
			"parking_space", "bicycle_parking", "motorcycle_parking", "fuel",
			"charging_station", "car_wash", "taxi", "marketplace"};
	static const std::unordered_set<std::string> planted_landuse = {"grass", "flowerbed",
			"meadow", "greenfield", "village_green", "forest", "orchard", "allotments",
			"plant_nursery"};

	std::vector<std::pair<uint64_t, bool>> sealed_ids;
	std::vector<uint64_t> planted_ids;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (!contains(way.id))
			continue;
		const auto leisure = way.tags.find("leisure");
		const auto amenity = way.tags.find("amenity");
		const auto landuse = way.tags.find("landuse");
		const auto natural = way.tags.find("natural");
		const auto highway = way.tags.find("highway");
		const auto area = way.tags.find("area");
		const bool leisure_paving =
				leisure != way.tags.end() && sealed_leisure.contains(leisure->second);
		const bool amenity_paving =
				amenity != way.tags.end() && sealed_amenity.contains(amenity->second);
		const bool highway_area = highway != way.tags.end() && area != way.tags.end() &&
								  area->second == "yes";
		if (leisure_paving || amenity_paving || highway_area) {
			const bool yields =
					highway_area || amenity_paving ||
					(leisure != way.tags.end() && leisure->second == "schoolyard");
			sealed_ids.emplace_back(way.id, yields);
		} else {
			bool is_planted = false;
			if (!way.tags.contains("building") && !way.tags.contains("building:part") &&
					highway == way.tags.end()) {
				if (landuse != way.tags.end()) {
					is_planted = planted_landuse.contains(landuse->second);
				} else if (natural != way.tags.end()) {
					is_planted =
							natural->second != "tree" && natural->second != "tree_row" &&
							!(amenity != way.tags.end() && amenity->second == "fountain");
				} else {
					is_planted =
							amenity == way.tags.end() && leisure != way.tags.end() &&
							(leisure->second == "park" || leisure->second == "garden");
				}
			}
			if (is_planted)
				planted_ids.push_back(way.id);
		}
	}
	struct Layer
	{
		std::size_t size;
		bool paved;
		uint64_t id;
	};
	std::vector<std::optional<std::size_t>> sealed_lens(sealed_ids.size());
	std::atomic_bool off_roads{false};
	parallel_for_indices(sealed_ids.size(), [&](std::size_t i) {
		const auto fill = transient_fill(sealed_ids[i].first);
		if (!fill || fill->empty())
			return;
		sealed_lens[i] = fill->size();
		// Once one sealed area contributes a cell outside the road mask, the
		// remaining fills need only report their sizes for layer ordering. Match
		// Rust's atomic early-out so large footprints do not all get rescanned.
		if (!off_roads.load(std::memory_order_relaxed) &&
				std::any_of(fill->begin(), fill->end(), [&roads](const auto &coord) {
					return roads.is_unset_in_bounds(coord.first, coord.second);
				}))
			off_roads.store(true, std::memory_order_relaxed);
	});
	if (!off_roads.load(std::memory_order_relaxed))
		return std::nullopt;

	std::vector<Layer> layers;
	layers.reserve(sealed_ids.size() + planted_ids.size());
	std::vector<Layer> managed;
	managed.reserve(sealed_ids.size());
	std::size_t largest_paving = 0;
	for (std::size_t i = 0; i < sealed_ids.size(); ++i) {
		if (!sealed_lens[i])
			continue;
		const auto &[id, yields] = sealed_ids[i];
		if (yields) {
			largest_paving = std::max(largest_paving, *sealed_lens[i]);
			layers.push_back({*sealed_lens[i], true, id});
		} else {
			managed.push_back({*sealed_lens[i], true, id});
		}
	}
	if (largest_paving > 0) {
		std::vector<std::optional<std::size_t>> planted_lens(planted_ids.size());
		parallel_for_indices(planted_ids.size(), [&](std::size_t i) {
			if (const auto fill = transient_fill(planted_ids[i]))
				planted_lens[i] = fill->size();
		});
		for (std::size_t i = 0; i < planted_ids.size(); ++i) {
			if (planted_lens[i] && *planted_lens[i] < largest_paving)
				layers.push_back({*planted_lens[i], false, planted_ids[i]});
		}
	}
	std::sort(layers.begin(), layers.end(), [](const Layer &a, const Layer &b) {
		if (a.size != b.size)
			return a.size > b.size;
		// Apply planted layers first at equal size, so paving takes precedence.
		return a.paved < b.paved;
	});
	layers.insert(layers.end(), managed.begin(), managed.end());
	auto mask = roads;
	for (std::size_t begin = 0; begin < layers.size();) {
		std::size_t end = begin;
		std::size_t cells = 0;
		while (end < layers.size()) {
			const auto size = layers[end].size;
			if (end > begin && size > PREPASS_BATCH_CELLS - cells)
				break;
			cells += size;
			++end;
			if (cells >= PREPASS_BATCH_CELLS)
				break;
		}
		std::vector<FloodFillResult> fills(end - begin);
		parallel_for_indices(fills.size(),
				[&](std::size_t i) { fills[i] = transient_fill(layers[begin + i].id); });
		for (std::size_t i = 0; i < fills.size(); ++i) {
			if (!fills[i])
				continue;
			const auto &layer = layers[begin + i];
			for (const auto &[x, z] : *fills[i]) {
				if (layer.paved)
					mask.set(x, z);
				else if (!roads.contains(x, z))
					mask.clear(x, z);
			}
		}
		begin = end;
	}
	return mask;
}

std::pair<size_t, size_t> CoordinateBitmap::count_in_range(int32_t min_x_range,
		int32_t min_z_range, int32_t max_x_range, int32_t max_z_range) const
{
	size_t urban_count = 0;
	size_t total_count = 0;

	for (int32_t z = min_z_range; z <= max_z_range; ++z) {
		// Calculate local z coordinate
		int64_t local_z = static_cast<int64_t>(z) - static_cast<int64_t>(min_z_);
		if (local_z < 0 || local_z >= static_cast<int64_t>(height_)) {
			// Row is out of bounds, still counts toward total
			total_count += static_cast<size_t>(static_cast<int64_t>(max_x_range) -
											   static_cast<int64_t>(min_x_range) + 1);
			continue;
		}
		size_t local_z_sz = static_cast<size_t>(local_z);

		// Calculate x range in local coordinates
		int64_t local_min_x =
				std::max(static_cast<int64_t>(min_x_range) - static_cast<int64_t>(min_x_),
						static_cast<int64_t>(0));
		int64_t local_max_x =
				std::min(static_cast<int64_t>(max_x_range) - static_cast<int64_t>(min_x_),
						static_cast<int64_t>(width_) - 1);

		// Count out-of-bounds x coordinates toward total
		int64_t x_start_offset =
				std::max(static_cast<int64_t>(min_x_) - static_cast<int64_t>(min_x_range),
						static_cast<int64_t>(0));
		int64_t x_end_offset = std::max(static_cast<int64_t>(max_x_range) -
												static_cast<int64_t>(min_x_) -
												(static_cast<int64_t>(width_) - 1),
				static_cast<int64_t>(0));
		total_count += static_cast<size_t>(x_start_offset + x_end_offset);

		if (local_min_x > local_max_x) {
			continue;
		}

		// Process this row
		size_t local_min_x_sz = static_cast<size_t>(local_min_x);
		size_t local_max_x_sz = static_cast<size_t>(local_max_x);
		size_t row_start_bit = local_z_sz * width_ + local_min_x_sz;
		size_t row_end_bit = local_z_sz * width_ + local_max_x_sz;
		size_t num_bits = row_end_bit - row_start_bit + 1;
		total_count += num_bits;

		// Count set bits using byte-wise popcount where possible
		size_t start_byte = row_start_bit / 8;
		size_t end_byte = row_end_bit / 8;
		size_t start_bit_in_byte = row_start_bit % 8;
		size_t end_bit_in_byte = row_end_bit % 8;

		if (start_byte == end_byte) {
			// All bits are in the same byte
			uint8_t byte_val = bits_[start_byte];
			// Create mask for bits from start_bit to end_bit (inclusive)
			size_t num_bits_in_mask = end_bit_in_byte - start_bit_in_byte + 1;
			uint8_t mask = (num_bits_in_mask >= 8)
								   ? 0xFFu
								   : static_cast<uint8_t>((1U << num_bits_in_mask) - 1);
			uint8_t masked = static_cast<uint8_t>((byte_val >> start_bit_in_byte) & mask);
			urban_count += static_cast<size_t>(__builtin_popcount(masked));
		} else {
			// First partial byte
			uint8_t first_byte = bits_[start_byte];
			uint8_t first_mask = static_cast<uint8_t>(
					~((1U << start_bit_in_byte) - 1)); // bits from start_bit to 7
			urban_count += static_cast<size_t>(
					__builtin_popcount(static_cast<uint8_t>(first_byte & first_mask)));

			// Full bytes in between
			for (size_t byte_idx = start_byte + 1; byte_idx < end_byte; ++byte_idx) {
				urban_count += static_cast<size_t>(__builtin_popcount(bits_[byte_idx]));
			}

			// Last partial byte
			uint8_t last_byte = bits_[end_byte];
			// Handle case where end_bit_in_byte is 7 (would overflow 1u8 << 8)
			uint8_t last_mask =
					(end_bit_in_byte >= 7)
							? 0xFFu
							: static_cast<uint8_t>((1U << (end_bit_in_byte + 1)) - 1);
			urban_count += static_cast<size_t>(
					__builtin_popcount(static_cast<uint8_t>(last_byte & last_mask)));
		}
	}

	return std::make_pair(urban_count, total_count);
}

bool FloodFillCache::way_needs_flood_fill(const ProcessedWay &way)
{
	auto it_building = way.tags.find("building");
	auto it_building_part = way.tags.find("building:part");
	auto it_landuse = way.tags.find("landuse");
	auto it_leisure = way.tags.find("leisure");
	auto it_amenity = way.tags.find("amenity");
	auto it_natural = way.tags.find("natural");
	auto it_highway = way.tags.find("highway");
	auto it_area = way.tags.find("area");
	auto it_tomb = way.tags.find("tomb");

	bool has_building = (it_building != way.tags.end());
	bool has_building_part = (it_building_part != way.tags.end());
	bool has_landuse = (it_landuse != way.tags.end());
	bool has_leisure = (it_leisure != way.tags.end());
	bool has_amenity = (it_amenity != way.tags.end());
	bool has_natural = (it_natural != way.tags.end());
	bool natural_not_tree = has_natural && it_natural->second != "tree" &&
							it_natural->second != "tree_row";
	bool has_highway = (it_highway != way.tags.end());
	bool has_area = (it_area != way.tags.end());
	bool area_yes = has_area && (it_area->second == "yes");
	bool has_tomb = (it_tomb != way.tags.end());
	bool tomb_pyramid = has_tomb && (it_tomb->second == "pyramid");
	auto it_power = way.tags.find("power");
	auto it_source = way.tags.find("generator:source");
	bool solar_generator = it_power != way.tags.end() &&
						   it_power->second == "generator" &&
						   it_source != way.tags.end() && it_source->second == "solar";

	const bool waterfront = is_waterfront(way.tags);
	const bool needed = has_building || has_building_part || has_landuse || has_leisure ||
						has_amenity || natural_not_tree || (has_highway && area_yes) ||
						tomb_pyramid || waterfront || solar_generator;
	return needed && !fill_goes_unread(way);
}

bool FloodFillCache::is_waterfront(
		const std::unordered_map<std::string, std::string> &tags)
{
	const auto it = tags.find("man_made");
	return it != tags.end() &&
		   (it->second == "pier" || it->second == "breakwater" || it->second == "groyne");
}

bool FloodFillCache::fill_goes_unread(const ProcessedWay &way)
{
	const auto &tags = way.tags;
	if (tags.contains("building") || tags.contains("building:part") ||
			tags.contains("highway") || tags.contains("power") || is_waterfront(tags))
		return false;
	static const std::unordered_set<std::string> sealed_leisure = {"pitch", "track",
			"playground", "recreation_ground", "schoolyard", "ice_rink", "water_park",
			"slipway", "outdoor_seating", "bathing_place", "fitness_station"};
	static const std::unordered_set<std::string> sealed_amenity = {"parking",
			"parking_space", "bicycle_parking", "motorcycle_parking", "fuel",
			"charging_station", "car_wash", "taxi", "marketplace"};
	const auto amenity = tags.find("amenity");
	const auto leisure = tags.find("leisure");
	const auto landuse = tags.find("landuse");
	if ((amenity != tags.end() && sealed_amenity.contains(amenity->second)) ||
			(leisure != tags.end() && sealed_leisure.contains(leisure->second)))
		return false;
	if (landuse != tags.end())
		return landuse->second == "residential" || landuse->second == "commercial";
	const auto natural = tags.find("natural");
	if (natural != tags.end() &&
			!(amenity != tags.end() && amenity->second == "fountain"))
		return false;
	return amenity != tags.end() && amenity->second != "bicycle_parking" &&
		   amenity->second != "shelter" && amenity->second != "fountain" &&
		   amenity->second != "parking";
}

std::optional<std::pair<int32_t, int32_t>> FloodFillCache::compute_centroid(
		const std::vector<std::pair<int32_t, int32_t>> &coords)
{
	if (coords.empty()) {
		return std::nullopt;
	}

	int64_t sum_x = 0;
	int64_t sum_z = 0;

	for (const auto &coord : coords) {
		sum_x += static_cast<int64_t>(coord.first);
		sum_z += static_cast<int64_t>(coord.second);
	}

	int64_t len = static_cast<int64_t>(coords.size());
	return std::make_optional(std::make_pair(
			static_cast<int32_t>(sum_x / len), static_cast<int32_t>(sum_z / len)));
}

FloodFillCache FloodFillCache::precompute(const std::vector<ProcessedElement> &elements,
		const std::optional<std::chrono::milliseconds> &timeout)
{
	// Collect all ways that need flood fill
	std::vector<const ProcessedWay *> ways_needing_fill;
	for (const auto &el : elements) {
		if (el.is_way()) {
			const ProcessedWay &way = el.as_way();
			if (way_needs_flood_fill(way)) {
				ways_needing_fill.push_back(&way);
			}
		}
	}
	// Relations that dispatch by filling their outer members (natural features,
	// ground-painting landuse, and parks) can otherwise re-run the same large
	// polygon fill once per relation/tile. Precompute each member way once, just
	// like standalone fillable ways. Keep this predicate aligned with
	// process_element's relation dispatch.
	std::unordered_set<uint64_t> seen;
	seen.reserve(ways_needing_fill.size());
	for (const auto *way : ways_needing_fill)
		seen.insert(way->id);
	for (const auto &el : elements) {
		if (!el.is_relation())
			continue;
		const auto &relation = el.as_relation();
		if (!relation_fills_members(relation))
			continue;
		for (const auto &member : relation.members)
			if ((member.role == ProcessedMemberRole::Outer ||
						member.role == ProcessedMemberRole::Inner) &&
					seen.insert(member.way->id).second)
				ways_needing_fill.push_back(member.way.get());
	}
	FloodFillCache cache;
	cache.timeout_ = timeout;
	for (const auto *way : ways_needing_fill) {
		std::vector<std::pair<int32_t, int32_t>> ring;
		ring.reserve(way->nodes.size());
		for (const auto &node : way->nodes)
			ring.emplace_back(node.x, node.z);
		cache.way_cache.emplace(way->id, std::make_shared<CachedFill>(std::move(ring)));
	}

	return cache;
}

FloodFillResult FloodFillCache::get_or_compute(const ProcessedWay &way,
		const std::optional<std::chrono::milliseconds> &timeout) const
{
	if (auto cached = get_cached(way.id))
		return cached;

	// Compute without holding the cache lock so different polygons can be
	// filled concurrently. Match Rust: cache misses (including synthetic or
	// combined relation ways) are computed on demand but are not inserted into
	// the long-lived cache. This keeps temporary fills from accumulating after
	// the precomputed OSM ways have been released.
	std::vector<std::pair<int32_t, int32_t>> polygon_coords;
	polygon_coords.reserve(way.nodes.size());
	for (const auto &n : way.nodes)
		polygon_coords.emplace_back(n.x, n.z);
	auto filled = flood_fill_area(polygon_coords, timeout);
	if (filled.empty())
		return empty_flood_fill_result();
	return std::make_shared<const std::vector<std::pair<int32_t, int32_t>>>(
			std::move(filled));
}

std::shared_ptr<const std::vector<std::pair<int32_t, int32_t>>>
FloodFillCache::get_cached(uint64_t way_id) const
{
	auto entry = find_cached_fill(way_id);
	if (!entry)
		return nullptr;
	std::lock_guard<std::mutex> lock(entry->mutex);
	if (entry->released)
		return nullptr;
	if (!entry->result) {
		auto filled = flood_fill_area(entry->ring, timeout_);
		entry->result = filled.empty()
								? empty_flood_fill_result()
								: std::make_shared<
										  const std::vector<std::pair<int32_t, int32_t>>>(
										  std::move(filled));
	}
	return entry->result;
}

FloodFillResult FloodFillCache::transient_fill(uint64_t way_id) const
{
	auto entry = find_cached_fill(way_id);
	if (!entry)
		return nullptr;
	{
		std::lock_guard<std::mutex> lock(entry->mutex);
		if (entry->result)
			return entry->result;
	}
	auto filled = flood_fill_area(entry->ring, timeout_);
	if (filled.empty())
		return empty_flood_fill_result();
	return std::make_shared<const std::vector<std::pair<int32_t, int32_t>>>(
			std::move(filled));
}

std::shared_ptr<FloodFillCache::CachedFill> FloodFillCache::find_cached_fill(
		uint64_t way_id) const
{
	std::lock_guard<std::mutex> lock(way_cache_mutex);
	const auto it = way_cache.find(way_id);
	return it == way_cache.end() ? nullptr : it->second;
}

bool FloodFillCache::contains(uint64_t way_id) const
{
	std::lock_guard<std::mutex> lock(way_cache_mutex);
	return way_cache.contains(way_id);
}

void FloodFillCache::release(uint64_t way_id) const
{
	if (retain_entries_)
		return;
	if (auto entry = find_cached_fill(way_id)) {
		std::lock_guard<std::mutex> lock(entry->mutex);
		entry->result.reset();
		entry->released = true;
	}
}

FloodFillResult FloodFillCache::get_or_compute_element(const ProcessedElement &element,
		const std::optional<std::chrono::milliseconds> &timeout) const
{

	if (element.is_way()) {
		return get_or_compute(element.as_way(), timeout);
	} else {
		return empty_flood_fill_result();
	}
}

BuildingFootprintBitmap FloodFillCache::collect_building_footprints(
		const std::vector<ProcessedElement> &elements, const XZBBox &xzbbox) const
{

	BuildingFootprintBitmap footprints(xzbbox);

	// Relations are applied first: merged outer rings mark the footprint and
	// merged inner rings cut courtyards back out. Standalone ways are applied
	// afterwards so buildings located inside a courtyard remain marked.
	for (const auto &element : elements) {
		if (element.is_relation()) {
			const ProcessedRelation &rel = element.as_relation();
			auto it_building = rel.tags.find("building");
			auto it_building_part = rel.tags.find("building:part");
			auto it_type = rel.tags.find("type");
			const bool is_building =
					it_building != rel.tags.end() || it_building_part != rel.tags.end() ||
					(it_type != rel.tags.end() && it_type->second == "building");
			if (!is_building || is_underground_building(rel.tags))
				continue;
			const auto collect_rings = [&](ProcessedMemberRole role) {
				std::vector<std::vector<ProcessedNode>> rings;
				for (const auto &member : rel.members)
					if (member.role == role && member.way)
						rings.push_back(member.way->nodes);
				merge_way_segments(rings);
				std::vector<std::vector<ProcessedNode>> clipped;
				for (auto &ring : rings) {
					ring = clipping::clip_way_to_bbox_owned(std::move(ring), xzbbox);
					if (ring.size() < 4)
						continue;
					if (ring.front().x != ring.back().x ||
							ring.front().z != ring.back().z) {
						if (std::abs(static_cast<int64_t>(ring.front().x) -
									 ring.back().x) > 1 ||
								std::abs(static_cast<int64_t>(ring.front().z) -
										 ring.back().z) > 1)
							continue;
						ring.push_back(ring.front());
					}
					clipped.push_back(std::move(ring));
				}
				return clipped;
			};
			const auto outers = collect_rings(ProcessedMemberRole::Outer);
			const auto courtyards = collect_rings(ProcessedMemberRole::Inner);
			for (const auto &group :
					element_processing::hole_groups(outers, courtyards)) {
				int32_t hole_min_x = xzbbox.max_x(), hole_min_z = xzbbox.max_z();
				int32_t hole_max_x = xzbbox.min_x(), hole_max_z = xzbbox.min_z();
				for (const auto i : group.second)
					for (const auto &node : courtyards[i]) {
						hole_min_x = std::min(hole_min_x, node.x);
						hole_min_z = std::min(hole_min_z, node.z);
						hole_max_x = std::max(hole_max_x, node.x);
						hole_max_z = std::max(hole_max_z, node.z);
					}
				std::optional<CoordinateBitmap> courtyard;
				if (!group.second.empty() && hole_min_x <= hole_max_x &&
						hole_min_z <= hole_max_z)
					courtyard.emplace(
							XZBBox(hole_min_x, hole_min_z, hole_max_x, hole_max_z));
				for (const auto i : group.second) {
					std::vector<std::pair<int32_t, int32_t>> coords;
					coords.reserve(courtyards[i].size());
					for (const auto &node : courtyards[i])
						coords.emplace_back(node.x, node.z);
					for (const auto &[x, z] : flood_fill_area(coords, nullptr))
						if (courtyard)
							courtyard->set(x, z);
				}
				for (const auto i : group.first) {
					std::vector<std::pair<int32_t, int32_t>> coords;
					coords.reserve(outers[i].size());
					for (const auto &node : outers[i])
						coords.emplace_back(node.x, node.z);
					for (const auto &[x, z] : flood_fill_area(coords, nullptr))
						if (!courtyard || !courtyard->contains(x, z))
							footprints.set(x, z);
				}
			}
		}
	}

	std::vector<uint64_t> building_ids;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const ProcessedWay &way = element.as_way();
		if ((way.tags.contains("building") || way.tags.contains("building:part")) &&
				!is_underground_building(way.tags))
			building_ids.push_back(way.id);
	}
	// Match the Rust prepass: compute bounded batches in parallel, then update the
	// bitmap serially. This avoids retaining every fill at once and keeps bitmap
	// writes race-free.
	for (std::size_t begin = 0; begin < building_ids.size(); begin += PREPASS_BATCH) {
		const std::size_t count = std::min(PREPASS_BATCH, building_ids.size() - begin);
		std::vector<FloodFillResult> fills(count);
		parallel_for_indices(count, [&](std::size_t i) {
			fills[i] = transient_fill(building_ids[begin + i]);
		});
		for (const auto &fill : fills)
			if (fill)
				for (const auto &[x, z] : *fill)
					footprints.set(x, z);
	}

	return footprints;
}

std::vector<std::pair<int32_t, int32_t>> FloodFillCache::collect_building_centroids(
		const std::vector<ProcessedElement> &elements) const
{

	std::vector<std::pair<int32_t, int32_t>> centroids;

	for (const auto &element : elements) {
		if (element.is_way()) {
			const ProcessedWay &way = element.as_way();
			auto it_building = way.tags.find("building");
			auto it_building_part = way.tags.find("building:part");

			if (it_building != way.tags.end() || it_building_part != way.tags.end()) {
				if (auto fill = transient_fill(way.id)) {
					auto centroid = compute_centroid(*fill);
					if (centroid.has_value()) {
						centroids.push_back(centroid.value());
					}
				}
			}
		} else if (element.is_relation()) {
			const ProcessedRelation &rel = element.as_relation();
			auto it_building = rel.tags.find("building");
			auto it_building_part = rel.tags.find("building:part");
			auto it_type = rel.tags.find("type");

			bool is_building =
					(it_building != rel.tags.end()) ||
					(it_building_part != rel.tags.end()) ||
					(it_type != rel.tags.end() && it_type->second == "building");

			if (is_building) {
				// For building relations, compute centroid from outer ways
				std::vector<std::pair<int32_t, int32_t>> all_coords;
				for (const auto &member : rel.members) {
					if (member.role == ProcessedMemberRole::Outer) {
						if (auto fill = transient_fill(member.way->id)) {
							all_coords.insert(
									all_coords.end(), fill->begin(), fill->end());
						}
					}
				}

				auto centroid = compute_centroid(all_coords);
				if (centroid.has_value()) {
					centroids.push_back(centroid.value());
				}
			}
		}
	}

	return centroids;
}

void FloodFillCache::remove_way(uint64_t way_id)
{
	if (retain_entries_)
		return;
	std::lock_guard<std::mutex> lock(way_cache_mutex);
	way_cache.erase(way_id);
}

void FloodFillCache::remove_relation_ways(const std::vector<uint64_t> &way_ids)
{
	if (retain_entries_)
		return;
	std::lock_guard<std::mutex> lock(way_cache_mutex);
	for (uint64_t id : way_ids) {
		way_cache.erase(id);
	}
}

void FloodFillCache::clear()
{
	std::lock_guard<std::mutex> lock(way_cache_mutex);
	way_cache.clear();
	way_cache.rehash(0);
}

void configure_thread_pool(double cpu_fraction)
{
	configured_workers.store(
			thread_count_for_fraction(cpu_fraction), std::memory_order_relaxed);
}

} // namespace arnis
