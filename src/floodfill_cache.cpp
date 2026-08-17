#include "floodfill_cache.h"
#include "floodfill.h"
#include <algorithm>
#include <thread>
#include <future>
#include <numeric>
#include <cmath>
#include <iterator>

namespace arnis
{

namespace
{

bool same_ring_point(const ProcessedNode &a, const ProcessedNode &b)
{
	return a.id == b.id || (a.x == b.x && a.z == b.z);
}

void merge_relation_segments(std::vector<std::vector<ProcessedNode>> &rings)
{
	bool changed = true;
	while (changed) {
		changed = false;
		for (size_t i = 0; i < rings.size() && !changed; ++i) {
			if (rings[i].empty())
				continue;
			for (size_t j = i + 1; j < rings.size(); ++j) {
				if (rings[j].empty())
					continue;
				auto &a = rings[i];
				auto &b = rings[j];
				if (same_ring_point(a.back(), b.front()))
					a.insert(a.end(), std::next(b.begin()), b.end());
				else if (same_ring_point(a.back(), b.back()))
					a.insert(a.end(), std::next(b.rbegin()), b.rend());
				else if (same_ring_point(a.front(), b.back()))
					a.insert(a.begin(), b.begin(), std::prev(b.end()));
				else if (same_ring_point(a.front(), b.front()))
					a.insert(a.begin(), std::next(b.rbegin()), b.rend());
				else
					continue;
				rings.erase(rings.begin() + static_cast<std::ptrdiff_t>(j));
				changed = true;
				break;
			}
		}
	}
}

template <typename Apply>
void for_relation_ring_cells(
		const ProcessedRelation &relation, ProcessedMemberRole role, Apply apply)
{
	std::vector<std::vector<ProcessedNode>> rings;
	for (const auto &member : relation.members) {
		if (member.role == role)
			rings.push_back(member.way.nodes);
	}
	merge_relation_segments(rings);

	for (auto &ring : rings) {
		if (ring.size() < 3)
			continue;
		if (!same_ring_point(ring.front(), ring.back())) {
			if (std::abs(ring.front().x - ring.back().x) > 1 ||
					std::abs(ring.front().z - ring.back().z) > 1)
				continue;
			ring.push_back(ring.front());
		}
		if (ring.size() < 4)
			continue;
		std::vector<std::pair<int32_t, int32_t>> coords;
		coords.reserve(ring.size());
		for (const auto &node : ring)
			coords.emplace_back(node.x, node.z);
		for (const auto &[x, z] : flood_fill_area(coords, std::nullopt))
			apply(x, z);
	}
}

} // namespace

CoordinateBitmap::CoordinateBitmap(const XZBBox &xzbbox)
{
	min_x_ = xzbbox.min_x();
	min_z_ = xzbbox.min_z();
	// Use int64_t to avoid overflow when world spans more than int32_t::MAX in either dimension
	width_ = static_cast<size_t>(
			static_cast<int64_t>(xzbbox.max_x()) - static_cast<int64_t>(min_x_) + 1);
	height_ = static_cast<size_t>(
			static_cast<int64_t>(xzbbox.max_z()) - static_cast<int64_t>(min_z_) + 1);

	// Calculate number of bytes needed (round up to nearest byte)
	size_t total_bits = width_ * height_;
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
	bool natural_not_tree = has_natural && (it_natural->second != "tree");
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

	return has_building || has_building_part || has_landuse || has_leisure ||
		   has_amenity || natural_not_tree || (has_highway && area_yes) || tomb_pyramid ||
		   solar_generator;
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

	// Compute all way flood fills (sequentially since we don't have parallel processing library)
	FloodFillCache cache;
	for (const ProcessedWay *way : ways_needing_fill) {
		std::vector<std::pair<int32_t, int32_t>> polygon_coords;
		polygon_coords.reserve(way->nodes.size());
		for (const auto &n : way->nodes) {
			polygon_coords.emplace_back(n.x, n.z);
		}

		std::vector<std::pair<int32_t, int32_t>> filled =
				flood_fill_area(polygon_coords, timeout);
		cache.way_cache[way->id] = std::move(filled);
	}

	return cache;
}

std::vector<std::pair<int32_t, int32_t>> FloodFillCache::get_or_compute(
		const ProcessedWay &way,
		const std::optional<std::chrono::milliseconds> &timeout) const
{

	auto it = way_cache.find(way.id);
	if (it != way_cache.end()) {
		// Return a copy as in Rust implementation
		return it->second;
	} else {
		// Fallback: compute on demand for synthetic/combined ways from relations
		std::vector<std::pair<int32_t, int32_t>> polygon_coords;
		polygon_coords.reserve(way.nodes.size());
		for (const auto &n : way.nodes) {
			polygon_coords.emplace_back(n.x, n.z);
		}
		return flood_fill_area(polygon_coords, timeout);
	}
}

std::vector<std::pair<int32_t, int32_t>> FloodFillCache::get_or_compute_element(
		const ProcessedElement &element,
		const std::optional<std::chrono::milliseconds> &timeout) const
{

	if (element.is_way()) {
		return get_or_compute(element.as_way(), timeout);
	} else {
		return std::vector<std::pair<int32_t, int32_t>>();
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
			if (!is_building)
				continue;
			for_relation_ring_cells(rel, ProcessedMemberRole::Outer,
					[&footprints](int32_t x, int32_t z) { footprints.set(x, z); });
			for_relation_ring_cells(rel, ProcessedMemberRole::Inner,
					[&footprints](int32_t x, int32_t z) { footprints.clear(x, z); });
		}
	}

	for (const auto &element : elements) {
		if (element.is_way()) {
			const ProcessedWay &way = element.as_way();
			auto it_building = way.tags.find("building");
			auto it_building_part = way.tags.find("building:part");

			if (it_building != way.tags.end() || it_building_part != way.tags.end()) {
				auto it = way_cache.find(way.id);
				if (it != way_cache.end()) {
					for (const auto &coord : it->second) {
						footprints.set(coord.first, coord.second);
					}
				}
			}
		}
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
				auto it = way_cache.find(way.id);
				if (it != way_cache.end()) {
					auto centroid = compute_centroid(it->second);
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
						auto it = way_cache.find(member.way.id);
						if (it != way_cache.end()) {
							all_coords.insert(all_coords.end(), it->second.begin(),
									it->second.end());
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
	way_cache.erase(way_id);
}

void FloodFillCache::remove_relation_ways(const std::vector<uint64_t> &way_ids)
{
	for (uint64_t id : way_ids) {
		way_cache.erase(id);
	}
}

void configure_thread_pool(double cpu_fraction)
{
	// Flood-fill work is currently synchronous; retain the Rust API's tuning
	// hook without pretending to configure a nonexistent global pool.
	(void)cpu_fraction;
}

} // namespace arnis
