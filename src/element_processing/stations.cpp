#include "stations.h"

#include "../bresenham.h"
#include "../floodfill.h"
#include "../world_editor/common.h"
#include "structure_kit.h"
#include "way_segments.h"
#include "rail_track.h"
#include "signage.h"
#include "highways.h"
#include "connected_blocks.h"
#include "../block_definitions.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <numbers>
#include <unordered_set>
#include <numeric>
#include <string_view>
#include <tuple>

namespace arnis::stations
{
namespace
{
constexpr std::array<const char *, 5> INACTIVE_RAILWAY_TAGS{
		"disused", "abandoned", "construction", "proposed", "razed"};

struct TrackCell
{
	std::uint64_t way{};
	int x{}, z{};
	bool tunnel{};
	std::pair<double, double> dir{};
};

struct Frame
{
	double cx{}, cz{}, ux{1.0}, uz{};
	double along(int x, int z) const { return (x - cx) * ux + (z - cz) * uz; }
	double across(int x, int z) const { return -(x - cx) * uz + (z - cz) * ux; }
};

Frame cell_frame(const std::vector<Xz> &cells)
{
	Frame frame;
	if (cells.empty())
		return frame;
	for (const auto &[x, z] : cells) {
		frame.cx += x;
		frame.cz += z;
	}
	frame.cx /= cells.size();
	frame.cz /= cells.size();
	double xx = 0.0, zz = 0.0, xz = 0.0;
	for (const auto &[x, z] : cells) {
		const double dx = x - frame.cx, dz = z - frame.cz;
		xx += dx * dx;
		zz += dz * dz;
		xz += dx * dz;
	}
	const double angle = 0.5 * std::atan2(2.0 * xz, xx - zz);
	frame.ux = std::cos(angle);
	frame.uz = std::sin(angle);
	return frame;
}

std::string_view tag(const tags_t &tags, std::string_view key)
{
	const auto found = tags.find(std::string(key));
	return found == tags.end() ? std::string_view{} : std::string_view(found->second);
}

bool yes(const tags_t &tags, std::string_view key)
{
	return tag(tags, key) == "yes";
}

std::optional<double> tag_number(std::string_view value)
{
	std::optional<double> lowest;
	std::size_t start = 0;
	while (start <= value.size()) {
		const auto end = value.find(';', start);
		auto item = value.substr(start,
				end == std::string_view::npos ? value.size() - start : end - start);
		while (!item.empty() && std::isspace(static_cast<unsigned char>(item.front())))
			item.remove_prefix(1);
		while (!item.empty() && std::isspace(static_cast<unsigned char>(item.back())))
			item.remove_suffix(1);
		if (!item.empty()) {
			std::size_t numeric_length = 0;
			while (numeric_length < item.size()) {
				const unsigned char c = static_cast<unsigned char>(item[numeric_length]);
				if (!(std::isdigit(c) || c == '.' || c == '-'))
					break;
				++numeric_length;
			}
			const std::string copy(item.substr(0, numeric_length));
			char *parsed_end = nullptr;
			const auto number = std::strtod(copy.c_str(), &parsed_end);
			if (!copy.empty() && parsed_end != copy.c_str() && *parsed_end == '\0' &&
					std::isfinite(number))
				lowest = lowest ? std::min(*lowest, number) : number;
		}
		if (end == std::string_view::npos)
			break;
		start = end + 1;
	}
	return lowest;
}

std::pair<int, int> bbox_of(const std::vector<Xz> &cells)
{
	int min_x = std::numeric_limits<int>::max();
	int min_z = std::numeric_limits<int>::max();
	int max_x = std::numeric_limits<int>::min();
	int max_z = std::numeric_limits<int>::min();
	for (const auto &[x, z] : cells) {
		min_x = std::min(min_x, x);
		min_z = std::min(min_z, z);
		max_x = std::max(max_x, x);
		max_z = std::max(max_z, z);
	}
	return {min_x, max_x};
}

std::vector<Xz> boundary_cells(const std::vector<Xz> &ring)
{
	std::vector<Xz> cells;
	for (std::size_t i = 1; i < ring.size(); ++i) {
		for (const auto &[x, y, z] : bresenham_line(ring[i - 1].first, 0,
					 ring[i - 1].second, ring[i].first, 0, ring[i].second))
			cells.emplace_back(x, z);
	}
	return cells;
}

std::vector<std::vector<ProcessedNode>> relation_rings(
		const ProcessedRelation &relation, ProcessedMemberRole role)
{
	std::vector<std::vector<ProcessedNode>> rings;
	for (const auto &member : relation.members) {
		if (member.role == role && member.way)
			rings.push_back(member.way->nodes);
	}
	merge_way_segments(rings);
	return rings;
}

std::vector<Xz> fill_rings(const std::vector<std::vector<ProcessedNode>> &rings,
		const StationTimeout &timeout)
{
	std::unordered_set<Xz, XzHash> cells;
	for (const auto &ring : rings) {
		if (ring.size() < 4)
			continue;
		std::vector<Xz> points;
		points.reserve(ring.size());
		for (const auto &node : ring)
			points.emplace_back(node.x, node.z);
		for (const auto &cell : ring_cells(points, timeout))
			cells.insert(cell);
	}
	return {cells.begin(), cells.end()};
}

struct HallFlight
{
	Xz foot{};
	Xz dir{};
	std::vector<int> offsets;
	std::vector<std::pair<Xz, int>> steps;
	std::vector<Xz> cells;
	std::vector<std::pair<Xz, int>> sides;
	Xz top{};

	Xz cell(int k, int l) const
	{
		const Xz side{-dir.second, dir.first};
		return {foot.first + dir.first * k + side.first * l,
				foot.second + dir.second * k + side.second * l};
	}
};

std::optional<HallFlight> fit_hall_flight(const TunnelHall &hall, Xz foot, Xz dir)
{
	const std::array<std::vector<int>, 4> widths{{{-1, 0, 1}, {0, 1}, {-1, 0}, {0}}};
	for (const auto &offsets : widths) {
		HallFlight flight;
		flight.foot = foot;
		flight.dir = dir;
		flight.offsets = offsets;
		const int lo = offsets.front(), hi = offsets.back();
		bool fits = true;
		for (int k = 0; k <= HALL_FLIGHT && fits; ++k)
			for (const int l : offsets) {
				const Xz c = flight.cell(k, l);
				if (!hall.platform.contains(c) || hall.platform_edge.contains(c)) {
					fits = false;
					break;
				}
			}
		for (int k = 1; k <= HALL_FLIGHT && fits; ++k)
			if (!hall.platform.contains(flight.cell(k, lo - 1)) ||
					!hall.platform.contains(flight.cell(k, hi + 1)))
				fits = false;
		if (!fits)
			continue;
		for (int k = 1; k <= HALL_FLIGHT; ++k) {
			for (const int l : offsets) {
				const Xz c = flight.cell(k, l);
				flight.steps.emplace_back(c, k);
				flight.cells.push_back(c);
			}
			flight.sides.emplace_back(flight.cell(k, lo - 1), k);
			flight.sides.emplace_back(flight.cell(k, hi + 1), k);
		}
		flight.top = flight.cell(HALL_FLIGHT + 1, 0);
		return flight;
	}
	return std::nullopt;
}

std::vector<Xz> station_l_path(Xz a, Xz b, bool x_first)
{
	const Xz corner = x_first ? Xz{b.first, a.second} : Xz{a.first, b.second};
	std::vector<Xz> out{a};
	for (const auto &[from, to] : {std::pair{a, corner}, std::pair{corner, b}}) {
		const int dx = (to.first > from.first) - (to.first < from.first);
		const int dz = (to.second > from.second) - (to.second < from.second);
		Xz c = from;
		while (c != to) {
			c = {c.first + dx, c.second + dz};
			out.push_back(c);
		}
	}
	return out;
}
} // namespace

std::size_t XzHash::operator()(const Xz &point) const noexcept
{
	auto value = (std::uint64_t(std::uint32_t(point.first)) << 32) |
				 std::uint32_t(point.second);
	value ^= value >> 30;
	value *= 0xbf58476d1ce4e5b9ULL;
	value ^= value >> 27;
	value *= 0x94d049bb133111ebULL;
	return static_cast<std::size_t>(value ^ (value >> 31));
}

std::size_t SparseCells::Hash::operator()(const Xz &point) const noexcept
{
	const auto value = (std::uint64_t(std::uint32_t(point.first)) << 32) |
					   std::uint32_t(point.second);
	auto mixed = value;
	mixed ^= mixed >> 30;
	mixed *= 0xbf58476d1ce4e5b9ULL;
	mixed ^= mixed >> 27;
	mixed *= 0x94d049bb133111ebULL;
	return static_cast<std::size_t>(mixed ^ (mixed >> 31));
}

std::size_t TrackBedIndex::Hash::operator()(const Key &key) const noexcept
{
	std::size_t value = std::hash<std::uint64_t>{}(key.way);
	value ^= XzHash{}({key.x, key.z}) + 0x9e3779b97f4a7c15ULL + (value << 6) +
			 (value >> 2);
	return value;
}

void TrackBedIndex::set(std::uint64_t way, int x, int z, int bed)
{
	beds_.insert_or_assign(Key{way, x, z}, bed);
}

std::optional<int> TrackBedIndex::get(std::uint64_t way, int x, int z) const
{
	const auto it = beds_.find(Key{way, x, z});
	return it == beds_.end() ? std::nullopt : std::optional<int>{it->second};
}

void TunnelPinIndex::set(std::uint64_t way, int x, int z, TunnelPin pin)
{
	const TrackBedIndex::Key key{way, x, z};
	pins_.insert_or_assign(key, pin);
}

void TunnelPinIndex::discard_inactive_halls()
{
	std::erase_if(halls_, [](const TunnelHall &hall) { return !hall.active; });
	dug_ = SparseCells{};
	for (const auto &hall : halls_)
		for (const auto &[x, z] : hall.footprint)
			dug_.set(x, z);
	for (const auto &entrance : entrances_)
		for (const auto &cell : entrance.cells)
			dug_.set(cell.x, cell.z);
}

std::optional<TunnelPin> TunnelPinIndex::get(std::uint64_t way, int x, int z) const
{
	const auto it = pins_.find(TrackBedIndex::Key{way, x, z});
	return it == pins_.end() ? std::nullopt : std::optional<TunnelPin>{it->second};
}

void TunnelPinIndex::set_joint_floor(int x, int z, int floor)
{
	const auto key = (std::uint64_t(std::uint32_t(x)) << 32) | std::uint32_t(z);
	auto [it, inserted] = joint_floors_.try_emplace(key, floor);
	if (!inserted)
		it->second = std::min(it->second, floor);
}

std::optional<int> TunnelPinIndex::joint_floor(int x, int z) const
{
	const auto key = (std::uint64_t(std::uint32_t(x)) << 32) | std::uint32_t(z);
	const auto it = joint_floors_.find(key);
	return it == joint_floors_.end() ? std::nullopt : std::optional<int>{it->second};
}

void SparseCells::set(int x, int z)
{
	const unsigned bit =
			(static_cast<unsigned>(z) & 15U) * 16U + (static_cast<unsigned>(x) & 15U);
	const auto block_x = x / 16 - (x % 16 < 0 ? 1 : 0);
	const auto block_z = z / 16 - (z % 16 < 0 ? 1 : 0);
	blocks_[{block_x, block_z}][bit >> 6] |= std::uint64_t{1} << (bit & 63U);
}

void SparseCells::merge(const SparseCells &other)
{
	for (const auto &[section, bits] : other.blocks_) {
		auto &target = blocks_[section];
		for (std::size_t i = 0; i < target.size(); ++i)
			target[i] |= bits[i];
	}
}

void SparseCells::set_square(int x, int z, int radius)
{
	if (radius < 0)
		return;
	const auto floor_section = [](int value) {
		return value / 16 - (value % 16 < 0 ? 1 : 0);
	};
	for (int zz = z - radius; zz <= z + radius; ++zz) {
		int x0 = x - radius;
		const int max_x = x + radius;
		while (x0 <= max_x) {
			const int x1 = std::min(max_x, floor_section(x0) * 16 + 15);
			const unsigned row = (static_cast<unsigned>(zz) & 15U) << 4;
			const unsigned lo = row + (static_cast<unsigned>(x0) & 15U);
			const unsigned hi = row + (static_cast<unsigned>(x1) & 15U);
			auto &block = blocks_[{floor_section(x0), floor_section(zz)}];
			for (unsigned bit = lo; bit <= hi; ++bit)
				block[bit >> 6] |= std::uint64_t{1} << (bit & 63U);
			x0 = x1 + 1;
		}
	}
}
bool SparseCells::contains(int x, int z) const
{
	const unsigned bit =
			(static_cast<unsigned>(z) & 15U) * 16U + (static_cast<unsigned>(x) & 15U);
	const auto block_x = x / 16 - (x % 16 < 0 ? 1 : 0);
	const auto block_z = z / 16 - (z % 16 < 0 ? 1 : 0);
	const auto found = blocks_.find({block_x, block_z});
	return found != blocks_.end() &&
		   (found->second[bit >> 6] & (std::uint64_t{1} << (bit & 63U))) != 0;
}

bool is_rail_platform(const tags_t &tags)
{
	if (tag(tags, "railway") == "platform")
		return true;
	if (tag(tags, "public_transport") != "platform")
		return false;
	for (const auto *mode : {"train", "subway", "light_rail", "tram", "monorail"})
		if (yes(tags, mode))
			return true;
	return false;
}

std::optional<int> depth_key(const tags_t &tags)
{
	for (const auto *key : {"layer", "level"}) {
		if (const auto value = tag_number(tag(tags, key)))
			return static_cast<int>(std::floor(std::clamp(*value, -16.0, 16.0)));
	}
	return std::nullopt;
}

bool tagged_surface(const tags_t &tags)
{
	const auto location = tag(tags, "location");
	const auto depth = depth_key(tags);
	return tag(tags, "tunnel") == "no" || location == "overground" ||
		   location == "surface" || (depth && *depth >= 0);
}

bool tagged_underground(const tags_t &tags)
{
	const auto tunnel = tag(tags, "tunnel");
	const auto location = tag(tags, "location");
	const auto depth = depth_key(tags);
	// Match Rust's `is_some_and(|value| value != "no")`: tag presence, rather
	// than a non-empty value, is what marks the platform as tunneled.
	return (tags.contains("tunnel") && tunnel != "no") || location == "underground" ||
		   (depth && *depth < 0);
}

bool is_low(const tags_t &tags)
{
	const auto raw_height = tag(tags, "height");
	auto height = tag_number(raw_height);
	if (height) {
		if (raw_height.find("cm") != std::string_view::npos)
			*height /= 100.0;
		return *height < 0.45;
	}
	return yes(tags, "tram") && !yes(tags, "train");
}

std::vector<Xz> ring_cells(const std::vector<Xz> &ring, const StationTimeout &timeout)
{
	if (ring.empty())
		return {};
	const auto [min_x, max_x] = bbox_of(ring);
	int min_z = std::numeric_limits<int>::max();
	int max_z = std::numeric_limits<int>::min();
	for (const auto &[x, z] : ring) {
		(void)x;
		min_z = std::min(min_z, z);
		max_z = std::max(max_z, z);
	}
	const auto width = static_cast<std::int64_t>(max_x) - min_x + 1;
	const auto height = static_cast<std::int64_t>(max_z) - min_z + 1;
	if (width <= 0 || height <= 0 || width > floodfill::MAX_FLOOD_FILL_AREA / height)
		return {};
	auto cells = floodfill::flood_fill_area(ring, timeout);
	const auto boundary = boundary_cells(ring);
	cells.insert(cells.end(), boundary.begin(), boundary.end());
	return cells;
}

std::vector<Xz> platform_cells(
		const ProcessedElement &element, const StationTimeout &timeout)
{
	std::vector<Xz> cells;
	if (element.is_way()) {
		const auto &way = element.as_way();
		std::vector<Xz> points;
		points.reserve(way.nodes.size());
		for (const auto &node : way.nodes)
			points.emplace_back(node.x, node.z);
		const bool closed = points.size() >= 4 && points.front() == points.back();
		if (closed) {
			cells = ring_cells(points, timeout);
		} else {
			const double half_width =
					tag_number(tag(way.tags, "width")).value_or(4.0) / 2.0;
			const int half = half_width <= 1.0 ? 1
							 : half_width >= 4.0
									 ? 4
									 : static_cast<int>(std::llround(half_width));
			std::unordered_set<Xz, XzHash> band;
			for (std::size_t i = 1; i < points.size(); ++i) {
				for (const auto &[x, y, z] :
						bresenham_line(points[i - 1].first, 0, points[i - 1].second,
								points[i].first, 0, points[i].second)) {
					(void)y;
					for (int dx = -half; dx <= half; ++dx)
						for (int dz = -half; dz <= half; ++dz)
							if (dx * dx + dz * dz <= half * half + half)
								band.emplace(x + dx, z + dz);
				}
				cells.assign(band.begin(), band.end());
			}
		}
	} else if (element.is_relation()) {
		const auto &relation = element.as_relation();
		const auto outer_rings = relation_rings(relation, ProcessedMemberRole::Outer);
		const auto inner_rings = relation_rings(relation, ProcessedMemberRole::Inner);
		const auto outer = fill_rings(outer_rings, timeout);
		const auto inner = fill_rings(inner_rings, timeout);
		std::unordered_set<Xz, XzHash> holes(inner.begin(), inner.end());
		for (const auto &cell : outer)
			if (!holes.contains(cell))
				cells.push_back(cell);
	}
	std::sort(cells.begin(), cells.end());
	cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
	if (cells.size() > MAX_PLATFORM_CELLS)
		cells.clear();
	return cells;
}

std::vector<Platform> collect_platforms(
		const std::vector<ProcessedElement> &elements, const StationTimeout &timeout)
{
	std::vector<Platform> platforms;
	for (const auto &element : elements) {
		const auto &tags = element.tags();
		if (element.is_node() || !is_rail_platform(tags))
			continue;
		const auto railway = tag(tags, "railway");
		if (std::find_if(INACTIVE_RAILWAY_TAGS.begin(), INACTIVE_RAILWAY_TAGS.end(),
					[railway](const char *value) { return railway == value; }) !=
				INACTIVE_RAILWAY_TAGS.end())
			continue;
		auto cells = platform_cells(element, timeout);
		if (cells.empty())
			continue;
		Platform platform;
		platform.id = element.id();
		platform.cells = std::move(cells);
		platform.min_x = platform.max_x = platform.cells.front().first;
		platform.min_z = platform.max_z = platform.cells.front().second;
		for (const auto &[x, z] : platform.cells) {
			platform.min_x = std::min(platform.min_x, x);
			platform.min_z = std::min(platform.min_z, z);
			platform.max_x = std::max(platform.max_x, x);
			platform.max_z = std::max(platform.max_z, z);
		}
		platform.low = is_low(tags);
		platform.covered = yes(tags, "covered");
		platform.shelter = yes(tags, "shelter");
		if (const auto name = tags.find("name"); name != tags.end())
			platform.name = name->second;
		if (const auto colour = tags.find("colour"); colour != tags.end())
			platform.colour = structure_kit::concrete_for_colour(colour->second);
		platform.depth = depth_key(tags);
		platform.underground = tagged_underground(tags);
		platform.surface = tagged_surface(tags);
		platforms.push_back(std::move(platform));
	}
	return platforms;
}

TrackBedIndex register_surface_platform_levels(WorldEditor &editor,
		const std::vector<Platform> &platforms,
		const std::vector<ProcessedElement> &elements,
		const bridges::BridgeStructureMap &bridge_structures, int min_x, int min_z,
		int max_x, int max_z, bool register_levels, std::vector<Platform> *surface_groups)
{
	if (surface_groups)
		surface_groups->clear();
	struct Rail
	{
		TrackCell cell;
		std::optional<int> deck;
		bool tunnel{};
	};
	std::unordered_map<Xz, std::vector<Rail>, XzHash> tracks_at;
	std::unordered_set<Xz, XzHash> nearby_tiles;
	for (const auto &platform : platforms)
		for (const auto &[x, z] : platform.cells)
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz)
					nearby_tiles.emplace((x >> 4) + dx, (z >> 4) + dz);
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto level = railways::track_level(way);
		if (!level)
			continue;
		const auto cells = railways::track_cells(way).cells;
		if (cells.empty())
			continue;
		const auto *deck = bridge_structures.rail_deck(way.id);
		for (std::size_t i = 0; i < cells.size(); ++i) {
			if (!nearby_tiles.contains({cells[i].first >> 4, cells[i].second >> 4}))
				continue;
			const auto a = cells[i > HEADING_CELLS ? i - HEADING_CELLS : 0];
			const auto b = cells[std::min(i + HEADING_CELLS, cells.size() - 1)];
			const double dx = b.first - a.first, dz = b.second - a.second;
			const double length = std::max(std::hypot(dx, dz), 1e-3);
			std::optional<int> deck_y;
			if (deck && !deck->ys.empty())
				deck_y = deck->ys[std::min(i, deck->ys.size() - 1)] + 1;
			tracks_at[cells[i]].push_back({{way.id, cells[i].first, cells[i].second,
												   false, {dx / length, dz / length}},
					deck_y, *level == railways::TrackLevel::Tunnel});
		}
	}
	std::vector<std::unordered_map<TrackBedIndex::Key, Rail, TrackBedIndex::Hash>>
			adjacent(platforms.size());
	std::vector<bool> underground(platforms.size(), false);
	for (std::size_t i = 0; i < platforms.size(); ++i) {
		const auto &platform = platforms[i];
		if (platform.cells.empty())
			continue;
		if (platform.underground) {
			underground[i] = true;
			continue;
		}
		const auto frame = cell_frame(platform.cells);
		double lo = std::numeric_limits<double>::infinity(), hi = -lo;
		double across_lo = std::numeric_limits<double>::infinity(),
			   across_hi = -across_lo;
		for (const auto &[x, z] : platform.cells) {
			lo = std::min(lo, frame.along(x, z));
			hi = std::max(hi, frame.along(x, z));
			across_lo = std::min(across_lo, frame.across(x, z));
			across_hi = std::max(across_hi, frame.across(x, z));
		}
		const bool headed = hi - lo > 2.0 * std::max(1.0, across_hi - across_lo);
		const double parallel = std::cos(PARALLEL_DEG * std::numbers::pi / 180.0);
		const std::unordered_set<Xz, XzHash> platform_cells(
				platform.cells.begin(), platform.cells.end());
		for (const auto &[x, z] : platform.cells) {
			const bool edge = !platform_cells.contains({x - 1, z}) ||
							  !platform_cells.contains({x + 1, z}) ||
							  !platform_cells.contains({x, z - 1}) ||
							  !platform_cells.contains({x, z + 1});
			const int radius = edge ? TUNNEL_TRACK_REACH : 0;
			for (int dx = -radius; dx <= radius; ++dx)
				for (int dz = -radius; dz <= radius; ++dz) {
					const auto found = tracks_at.find({x + dx, z + dz});
					if (found == tracks_at.end())
						continue;
					for (const auto &rail : found->second) {
						const int reach = rail.tunnel ? TUNNEL_TRACK_REACH : TRACK_REACH;
						if (std::max(std::abs(dx), std::abs(dz)) > reach ||
								(headed && std::abs(rail.cell.dir.first * frame.ux +
													rail.cell.dir.second * frame.uz) <
												   parallel))
							continue;
						adjacent[i].try_emplace(TrackBedIndex::Key{rail.cell.way,
														rail.cell.x, rail.cell.z},
								rail);
					}
				}
		}
		const auto tunnel_count = std::count_if(adjacent[i].begin(), adjacent[i].end(),
				[](const auto &entry) { return entry.second.tunnel; });
		underground[i] = platform.underground ||
						 (!platform.low && !platform.surface &&
								 tunnel_count * 2 >
										 static_cast<std::ptrdiff_t>(adjacent[i].size()));
		std::erase_if(adjacent[i],
				[&](const auto &entry) { return entry.second.tunnel != underground[i]; });
	}
	std::vector<std::size_t> parent(platforms.size());
	TrackBedIndex track_beds;
	std::iota(parent.begin(), parent.end(), 0);
	const auto root = [&](std::size_t value) {
		while (parent[value] != value) {
			parent[value] = parent[parent[value]];
			value = parent[value];
		}
		return value;
	};
	std::unordered_map<TrackBedIndex::Key, std::size_t, TrackBedIndex::Hash> owner;
	for (std::size_t i = 0; i < adjacent.size(); ++i)
		for (const auto &[key, rail] : adjacent[i]) {
			(void)rail;
			const auto [found, inserted] = owner.emplace(key, i);
			if (inserted || underground[i] != underground[found->second])
				continue;
			const auto a = root(i), b = root(found->second);
			if (a != b)
				parent[a] = b;
		}
	std::unordered_map<std::size_t, std::vector<std::size_t>> groups;
	for (std::size_t i = 0; i < platforms.size(); ++i)
		if (!underground[i])
			groups[root(i)].push_back(i);
	std::vector<std::vector<std::size_t>> ordered_groups;
	ordered_groups.reserve(groups.size());
	for (auto &[group_id, members] : groups) {
		(void)group_id;
		ordered_groups.push_back(std::move(members));
	}
	std::sort(ordered_groups.begin(), ordered_groups.end(),
			[](const auto &a, const auto &b) { return a.front() < b.front(); });
	for (const auto &members : ordered_groups) {
		std::unordered_set<Xz, XzHash> source_cells;
		std::unordered_map<TrackBedIndex::Key, Rail, TrackBedIndex::Hash> group_tracks;
		bool low = true;
		for (const auto i : members) {
			low = low && platforms[i].low;
			source_cells.insert(platforms[i].cells.begin(), platforms[i].cells.end());
			group_tracks.insert(adjacent[i].begin(), adjacent[i].end());
		}
		if (surface_groups) {
			const auto &first = platforms[members.front()];
			Platform group;
			group.id = first.id;
			group.cells.assign(source_cells.begin(), source_cells.end());
			std::sort(group.cells.begin(), group.cells.end());
			group.min_x = group.max_x = group.cells.front().first;
			group.min_z = group.max_z = group.cells.front().second;
			for (const auto &[x, z] : group.cells) {
				group.min_x = std::min(group.min_x, x);
				group.min_z = std::min(group.min_z, z);
				group.max_x = std::max(group.max_x, x);
				group.max_z = std::max(group.max_z, z);
			}
			group.low = low;
			group.covered = std::any_of(members.begin(), members.end(),
					[&](const auto i) { return platforms[i].covered; });
			group.shelter = std::any_of(members.begin(), members.end(),
					[&](const auto i) { return platforms[i].shelter; });
			group.name = first.name;
			group.colour = first.colour;
			group.depth = first.depth;
			group.surface = true;
			surface_groups->push_back(std::move(group));
		}
		std::unordered_set<Xz, XzHash> track_cells;
		std::vector<Rail> decks;
		std::vector<std::pair<TrackBedIndex::Key, Rail>> ordered_tracks(
				group_tracks.begin(), group_tracks.end());
		std::sort(ordered_tracks.begin(), ordered_tracks.end(),
				[](const auto &a, const auto &b) {
					return std::tie(a.first.way, a.first.x, a.first.z) <
						   std::tie(b.first.way, b.first.x, b.first.z);
				});
		for (const auto &[key, rail] : ordered_tracks) {
			(void)key;
			track_cells.emplace(rail.cell.x, rail.cell.z);
			if (rail.deck)
				decks.push_back(rail);
		}
		std::unordered_set<Xz, XzHash> bed_zone;
		for (const auto &[x, z] : track_cells)
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz)
					bed_zone.emplace(x + dx, z + dz);
		std::vector<Xz> cells;
		for (const auto &cell : source_cells)
			if (!bed_zone.contains(cell))
				cells.push_back(cell);
		std::sort(cells.begin(), cells.end());
		if (cells.empty())
			continue;
		const auto frame = cell_frame(cells);
		const bool elevated =
				!low && decks.size() * 2 > std::max<std::size_t>(1, group_tracks.size());
		struct Sample
		{
			double t, y;
		};
		std::vector<Sample> samples;
		if (elevated) {
			for (const auto &rail : decks)
				samples.push_back(
						{frame.along(rail.cell.x, rail.cell.z), double(*rail.deck)});
		} else {
			for (const auto &[x, z] : cells) {
				const int ground = editor.terrain_level(x, z).value_or(
						editor.get_ground_level(x, z));
				samples.push_back({frame.along(x, z), double(ground + 1)});
			}
		}
		const double mean_t =
				std::accumulate(samples.begin(), samples.end(), 0.0,
						[](double sum, const Sample &sample) { return sum + sample.t; }) /
				samples.size();
		const double mean_y =
				std::accumulate(samples.begin(), samples.end(), 0.0,
						[](double sum, const Sample &sample) { return sum + sample.y; }) /
				samples.size();
		double lo = std::numeric_limits<double>::infinity(), hi = -lo;
		double variance = 0.0, covariance = 0.0;
		for (const auto &sample : samples) {
			lo = std::min(lo, sample.y);
			hi = std::max(hi, sample.y);
			variance += (sample.t - mean_t) * (sample.t - mean_t);
			covariance += (sample.t - mean_t) * (sample.y - mean_y);
		}
		const double slope =
				hi - lo <= 2.0
						? 0.0
						: std::clamp(covariance / std::max(variance, 1.0), -0.12, 0.12);
		if (!low && !elevated) {
			for (const auto &[key, rail] : ordered_tracks) {
				if (rail.tunnel || rail.deck)
					continue;
				const auto y = static_cast<int>(std::lround(
						mean_y +
						slope * (frame.along(rail.cell.x, rail.cell.z) - mean_t)));
				track_beds.set(key.way, key.x, key.z, y - 1);
			}
		}
		for (const auto &[x, z] : cells)
			if (register_levels && x >= min_x && x <= max_x && z >= min_z && z <= max_z &&
					editor.in_bounds(x, z))
				editor.register_road_surface_y(x, z,
						static_cast<int>(std::lround(
								mean_y + slope * (frame.along(x, z) - mean_t))));
	}
	return track_beds;
}

void render_surface_platforms(WorldEditor &editor, const std::vector<Platform> &platforms,
		const std::vector<ProcessedElement> &elements,
		const bridges::BridgeStructureMap &bridge_structures,
		const CoordinateBitmap &road_mask, const CoordinateBitmap &building_footprints,
		int min_x, int min_z, int max_x, int max_z)
{
	static const std::vector<Block> rails{RAIL, RAIL_NORTH_SOUTH, RAIL_EAST_WEST,
			RAIL_ASCENDING_EAST, RAIL_ASCENDING_WEST, RAIL_ASCENDING_NORTH,
			RAIL_ASCENDING_SOUTH, RAIL_NORTH_EAST, RAIL_NORTH_WEST, RAIL_SOUTH_EAST,
			RAIL_SOUTH_WEST};
	static const std::vector<Block> loose{GRASS, TALL_GRASS_BOTTOM, TALL_GRASS_TOP, FERN,
			RED_FLOWER, YELLOW_FLOWER, BLUE_FLOWER, WHITE_FLOWER, DEAD_BUSH, SNOW_LAYER};
	struct RailCell
	{
		TrackCell track;
		std::optional<int> deck;
		bool tunnel{};
	};
	std::unordered_map<Xz, std::vector<RailCell>, XzHash> track_cells;
	std::unordered_map<Xz, std::vector<RailCell>, XzHash> all_track_cells;
	std::unordered_set<Xz, XzHash> nearby_tiles;
	for (const auto &platform : platforms)
		for (const auto &[x, z] : platform.cells)
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz)
					nearby_tiles.emplace((x >> 4) + dx, (z >> 4) + dz);
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto level = railways::track_level(way);
		if (!level)
			continue;
		const auto cells = railways::track_cells(way).cells;
		if (cells.empty())
			continue;
		const auto *deck = bridge_structures.rail_deck(way.id);
		for (std::size_t i = 0; i < cells.size(); ++i) {
			const auto a = cells[i > HEADING_CELLS ? i - HEADING_CELLS : 0];
			const auto b = cells[std::min(i + HEADING_CELLS, cells.size() - 1)];
			const double dx = b.first - a.first, dz = b.second - a.second;
			const double length = std::max(std::hypot(dx, dz), 1e-3);
			std::optional<int> deck_y;
			if (deck && !deck->ys.empty())
				deck_y = deck->ys[std::min(i, deck->ys.size() - 1)] + 1;
			RailCell rail{{way.id, cells[i].first, cells[i].second, false,
								  {dx / length, dz / length}},
					deck_y, *level == railways::TrackLevel::Tunnel};
			if (!nearby_tiles.contains({cells[i].first >> 4, cells[i].second >> 4}))
				continue;
			all_track_cells[cells[i]].push_back(rail);
			if (!rail.tunnel)
				track_cells[cells[i]].push_back(rail);
		}
	}
	const auto put = [&](Block block, int x, int y, int z) {
		editor.set_block_absolute(block, x, y, z, std::nullopt, rails);
	};
	const auto put_slab = [&](Block block, const char *type, int x, int y, int z) {
		BlockWithProperties slab{block, BlockWithProperties::Properties{{"type", type}}};
		editor.set_block_with_properties_absolute(slab, x, y, z, std::nullopt, rails);
	};
	const auto frame_of = [](const std::vector<Xz> &cells) { return cell_frame(cells); };
	const auto floor_ground = [&](int x, int z) { return editor.get_ground_level(x, z); };
	for (const auto &platform : platforms) {
		if (platform.cells.empty() || platform.min_x > max_x || platform.max_x < min_x ||
				platform.min_z > max_z || platform.max_z < min_z)
			continue;
		const auto frame = frame_of(platform.cells);
		double along_lo = std::numeric_limits<double>::infinity(), along_hi = -along_lo;
		double across_lo = std::numeric_limits<double>::infinity(),
			   across_hi = -across_lo;
		for (const auto &[x, z] : platform.cells) {
			along_lo = std::min(along_lo, frame.along(x, z));
			along_hi = std::max(along_hi, frame.along(x, z));
			across_lo = std::min(across_lo, frame.across(x, z));
			across_hi = std::max(across_hi, frame.across(x, z));
		}
		const bool headed =
				along_hi - along_lo > 2.0 * std::max(1.0, across_hi - across_lo);
		const double min_parallel = std::cos(PARALLEL_DEG * std::numbers::pi / 180.0);
		std::unordered_set<Xz, XzHash> source_cells(
				platform.cells.begin(), platform.cells.end());
		std::unordered_map<TrackBedIndex::Key, bool, TrackBedIndex::Hash> nearby_tracks;
		for (const auto &[x, z] : platform.cells) {
			const bool edge = !source_cells.contains({x - 1, z}) ||
							  !source_cells.contains({x + 1, z}) ||
							  !source_cells.contains({x, z - 1}) ||
							  !source_cells.contains({x, z + 1});
			const int radius = edge ? TUNNEL_TRACK_REACH : 0;
			for (int dx = -radius; dx <= radius; ++dx)
				for (int dz = -radius; dz <= radius; ++dz) {
					const auto found = all_track_cells.find({x + dx, z + dz});
					if (found == all_track_cells.end())
						continue;
					for (const auto &candidate : found->second) {
						const int reach =
								candidate.tunnel ? TUNNEL_TRACK_REACH : TRACK_REACH;
						if (std::max(std::abs(dx), std::abs(dz)) <= reach &&
								(!headed ||
										std::abs(candidate.track.dir.first * frame.ux +
												 candidate.track.dir.second * frame.uz) >=
												min_parallel))
							nearby_tracks.try_emplace(
									TrackBedIndex::Key{candidate.track.way,
											candidate.track.x, candidate.track.z},
									candidate.tunnel);
					}
				}
		}
		const auto tunnel_count = std::count_if(nearby_tracks.begin(),
				nearby_tracks.end(), [](const auto &entry) { return entry.second; });
		const bool underground =
				platform.underground ||
				(!platform.low && !platform.surface &&
						tunnel_count * 2 >
								static_cast<std::ptrdiff_t>(nearby_tracks.size()));
		if (underground)
			continue;
		std::unordered_map<TrackBedIndex::Key, RailCell, TrackBedIndex::Hash>
				adjacent_tracks;
		for (const auto &[x, z] : platform.cells) {
			const bool edge = !source_cells.contains({x - 1, z}) ||
							  !source_cells.contains({x + 1, z}) ||
							  !source_cells.contains({x, z - 1}) ||
							  !source_cells.contains({x, z + 1});
			const int reach = edge ? TUNNEL_TRACK_REACH : 0;
			for (int dx = -reach; dx <= reach; ++dx)
				for (int dz = -reach; dz <= reach; ++dz) {
					auto found = track_cells.find({x + dx, z + dz});
					if (found == track_cells.end())
						continue;
					for (const auto &candidate : found->second) {
						if (std::max(std::abs(dx), std::abs(dz)) <= TRACK_REACH &&
								(!headed ||
										std::abs(candidate.track.dir.first * frame.ux +
												 candidate.track.dir.second * frame.uz) >=
												std::cos(PARALLEL_DEG * std::numbers::pi /
														 180.0)))
							adjacent_tracks.try_emplace(
									TrackBedIndex::Key{candidate.track.way,
											candidate.track.x, candidate.track.z},
									candidate);
					}
				}
		}
		std::unordered_set<Xz, XzHash> adjacent_cells;
		std::vector<RailCell> adjacent_decks;
		for (const auto &[key, rail] : adjacent_tracks) {
			(void)key;
			adjacent_cells.emplace(rail.track.x, rail.track.z);
			if (rail.deck)
				adjacent_decks.push_back(rail);
		}
		const bool elevated =
				!platform.low && adjacent_decks.size() * 2 >
										 std::max<std::size_t>(1, adjacent_tracks.size());
		// Ballast is a corridor, not platform deck. Remove it before classifying edges.
		std::unordered_set<Xz, XzHash> bed_zone;
		for (const auto &[x, z] : adjacent_cells)
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz)
					bed_zone.emplace(x + dx, z + dz);
		std::vector<Xz> cells;
		for (const auto &cell : platform.cells)
			if (!bed_zone.contains(cell))
				cells.push_back(cell);
		if (cells.empty())
			continue;
		std::unordered_set<Xz, XzHash> deck_cells(cells.begin(), cells.end());
		struct Sample
		{
			double t, y;
		};
		std::vector<Sample> samples;
		if (elevated) {
			for (const auto &rail : adjacent_decks)
				samples.push_back(
						{frame.along(rail.track.x, rail.track.z), double(*rail.deck)});
		} else {
			for (const auto &[x, z] : cells)
				samples.push_back({frame.along(x, z), double(floor_ground(x, z) + 1)});
		}
		const double mean_t =
				std::accumulate(samples.begin(), samples.end(), 0.0,
						[](double sum, const Sample &s) { return sum + s.t; }) /
				samples.size();
		const double mean_y =
				std::accumulate(samples.begin(), samples.end(), 0.0,
						[](double sum, const Sample &s) { return sum + s.y; }) /
				samples.size();
		double lo = std::numeric_limits<double>::infinity(), hi = -lo;
		double variance = 0.0, covariance = 0.0;
		for (const auto &s : samples) {
			lo = std::min(lo, s.y);
			hi = std::max(hi, s.y);
			variance += (s.t - mean_t) * (s.t - mean_t);
			covariance += (s.t - mean_t) * (s.y - mean_y);
		}
		const double slope =
				hi - lo <= 2.0
						? 0.0
						: std::clamp(covariance / std::max(variance, 1.0), -0.12, 0.12);
		const auto level_at = [&](int x, int z) {
			return static_cast<int>(
					std::lround(mean_y + slope * (frame.along(x, z) - mean_t)));
		};
		bool building_roof = false;
		for (const auto &[x, z] : cells)
			building_roof += building_footprints.contains(x, z);
		const bool roofed_by_building = building_roof * 3 > cells.size();
		const bool covered = platform.covered && !roofed_by_building;
		double t_min = std::numeric_limits<double>::infinity(), t_max = -t_min;
		for (const auto &[x, z] : cells) {
			t_min = std::min(t_min, frame.along(x, z));
			t_max = std::max(t_max, frame.along(x, z));
		}
		const double t_mid = (t_min + t_max) * 0.5;
		const int ax =
				std::abs(frame.ux) >= std::abs(frame.uz) ? (frame.ux < 0 ? -1 : 1) : 0;
		const int az = ax ? 0 : (frame.uz < 0 ? -1 : 1);
		const int sx = -az, sz = ax;
		enum class DeckKind : std::uint8_t
		{
			Inner,
			Line,
			Edge,
			End
		};
		std::unordered_map<Xz, DeckKind, XzHash> deck_kinds;
		std::map<int, std::vector<Xz>> slices;
		for (const auto &[x, z] : cells) {
			const bool boundary = !deck_cells.contains({x - 1, z}) ||
								  !deck_cells.contains({x + 1, z}) ||
								  !deck_cells.contains({x, z - 1}) ||
								  !deck_cells.contains({x, z + 1});
			bool faces_track = false;
			for (const auto &[dx, dz] : {Xz{-1, 0}, Xz{1, 0}, Xz{0, -1}, Xz{0, 1}})
				faces_track |= bed_zone.contains({x + dx, z + dz});
			const double t = frame.along(x, z);
			const bool end =
					boundary && !faces_track && (t - t_min < 1.0 || t_max - t < 1.0);
			const bool edge = boundary && !end;
			bool line = false;
			if (!boundary)
				for (const auto &[dx, dz] : {Xz{-1, 0}, Xz{1, 0}, Xz{0, -1}, Xz{0, 1}}) {
					const int nx = x + dx, nz = z + dz;
					const bool next_boundary = !deck_cells.contains({nx - 1, nz}) ||
											   !deck_cells.contains({nx + 1, nz}) ||
											   !deck_cells.contains({nx, nz - 1}) ||
											   !deck_cells.contains({nx, nz + 1});
					bool next_track = false;
					for (const auto &[ex, ez] :
							{Xz{-1, 0}, Xz{1, 0}, Xz{0, -1}, Xz{0, 1}})
						next_track |= bed_zone.contains({nx + ex, nz + ez});
					line |= next_boundary && next_track;
				}
			const auto kind = end	 ? DeckKind::End
							  : edge ? DeckKind::Edge
							  : line ? DeckKind::Line
									 : DeckKind::Inner;
			deck_kinds.emplace(Xz{x, z}, kind);
			slices[static_cast<int>(std::lround(t - t_min))].emplace_back(x, z);
		}
		std::unordered_map<int, Xz> slice_centres;
		for (const auto &[slice, slice_cells] : slices) {
			double across_lo = std::numeric_limits<double>::infinity();
			double across_hi = -across_lo;
			for (const auto &[x, z] : slice_cells) {
				const double across = frame.across(x, z);
				across_lo = std::min(across_lo, across);
				across_hi = std::max(across_hi, across);
			}
			const double across_mid = (across_lo + across_hi) * 0.5;
			std::optional<Xz> centre;
			for (const auto &cell : slice_cells) {
				if (deck_kinds.at(cell) != DeckKind::Inner)
					continue;
				if (!centre ||
						std::abs(frame.across(cell.first, cell.second) - across_mid) <
								std::abs(frame.across(centre->first, centre->second) -
										 across_mid) ||
						(std::abs(frame.across(cell.first, cell.second) - across_mid) ==
										std::abs(frame.across(
														 centre->first, centre->second) -
												 across_mid) &&
								cell < *centre))
					centre = cell;
			}
			if (centre)
				slice_centres.emplace(slice, *centre);
		}
		if (!platform.low && !elevated)
			for (const auto &[x, z] : adjacent_cells) {
				if (road_mask.contains(x, z) || !editor.in_bounds(x, z))
					continue;
				bool beside_platform = false;
				for (const auto &[dx, dz] : {Xz{-1, 0}, Xz{1, 0}, Xz{0, -1}, Xz{0, 1}})
					beside_platform |= deck_cells.contains({x + dx, z + dz});
				if (!beside_platform || track_cells.contains({x, z}))
					continue;
				const int bed = level_at(x, z) - 1;
				editor.register_road_surface_y(x, z, bed);
				put(GRAVEL, x, bed, z);
				for (int y = bed + 1; y <= bed + 3; ++y)
					editor.set_block_absolute(AIR, x, y, z, std::nullopt, loose);
			}
		for (const auto &[x, z] : cells) {
			if (x < min_x || x > max_x || z < min_z || z > max_z ||
					!editor.in_bounds(x, z))
				continue;
			const auto kind = deck_kinds.at({x, z});
			const bool end = kind == DeckKind::End;
			const bool edge = kind == DeckKind::Edge;
			const bool line = kind == DeckKind::Line;
			bool faces_track = false;
			for (const auto &[dx, dz] : {Xz{-1, 0}, Xz{1, 0}, Xz{0, -1}, Xz{0, 1}})
				faces_track |= bed_zone.contains({x + dx, z + dz});
			const double t = frame.along(x, z);
			const int y =
					platform.low ? floor_ground(x, z) + 1
								 : editor.surface_override(x, z).value_or(level_at(x, z));
			const bool roof =
					covered ? t - t_min > 3.0 && t_max - t > 3.0
							: platform.shelter && !roofed_by_building &&
									  std::abs(t - t_mid) <= (platform.low ? 2.5 : 10.0);
			const auto slice = static_cast<int>(std::lround(t - t_min));
			const bool centre =
					slice_centres.contains(slice) && slice_centres.at(slice) == Xz{x, z};
			if (!platform.low)
				editor.register_road_surface_y(x, z, y);
			if (platform.low) {
				const int width = static_cast<int>(slices.at(slice).size());
				const bool furniture =
						centre &&
						((roof && slice % 8 == 0) || (width >= 5 && slice % 16 == 3) ||
								(width >= 4 && slice % 16 == 5) ||
								(roof && slice % 3 == 1));
				if (!furniture)
					put_slab(edge || end ? POLISHED_ANDESITE_SLAB
							 : line		 ? SMOOTH_QUARTZ_SLAB
										 : SMOOTH_STONE_SLAB,
							"bottom", x, y, z);
				if (roof) {
					put_slab(SMOOTH_STONE_SLAB, "bottom", x, y + 3, z);
				}
				continue;
			}
			const Block top = end	 ? SMOOTH_STONE
							  : edge ? POLISHED_ANDESITE
							  : line ? WHITE_CONCRETE
									 : SMOOTH_STONE;
			put(top, x, y, z);
			for (int dy = 1; dy <= 3; ++dy)
				editor.set_block_absolute(AIR, x, y + dy, z, std::nullopt, loose);
			if (roof && !edge && !end)
				put_slab(SMOOTH_STONE_SLAB, "bottom", x, y + 5, z);
			if (elevated) {
				put_slab(STONE_BRICK_SLAB, "top", x, y - 1, z);
				if (centre && slice % 6 == 3)
					for (int py = floor_ground(x, z) + 1; py < y - 1; ++py)
						put(STONE_BRICKS, x, py, z);
			}
			if (edge && !faces_track)
				if (elevated)
					editor.set_block_absolute(IRON_BARS, x, y + 1, z);
		}
		const auto facing_of = [](const Xz &direction) {
			if (direction.first == 0)
				return static_cast<std::int8_t>(direction.second < 0 ? 2 : 3);
			return static_cast<std::int8_t>(direction.first < 0 ? 4 : 5);
		};
		const auto stair_facing = [](std::int8_t facing) -> const char * {
			switch (facing) {
			case 2:
				return "north";
			case 3:
				return "south";
			case 4:
				return "west";
			default:
				return "east";
			}
		};
		const Xz side{-sz, sx};
		const int board_slice = static_cast<int>(std::lround(t_mid - t_min));
		for (const auto &[slice, centre] : slice_centres) {
			const auto &[x, z] = centre;
			if (x < min_x || x > max_x || z < min_z || z > max_z ||
					!editor.in_bounds(x, z))
				continue;
			const int width = static_cast<int>(slices.at(slice).size());
			const double t = t_min + slice;
			const bool roof =
					covered ? t - t_min > 3.0 && t_max - t > 3.0
							: platform.shelter && !roofed_by_building &&
									  std::abs(t - t_mid) <= (platform.low ? 2.5 : 10.0);
			const bool near_board =
					!platform.low && platform.name && std::abs(slice - board_slice) <= 1;
			const int surface_y =
					platform.low
							? floor_ground(x, z) + 1
							: editor.surface_override(x, z).value_or(level_at(x, z)) + 1;
			if (near_board) {
				if (slice == board_slice) {
					for (int dy = 0; dy < 3; ++dy)
						editor.set_block_absolute(STONE_BRICK_WALL, x, surface_y + dy, z);
					editor.set_block_absolute(LIGHT_GRAY_CONCRETE, x, surface_y + 3, z);
					signage::place_platform_name(
							editor, x, surface_y + 3, z, facing_of(side), *platform.name);
					signage::place_platform_name(editor, x, surface_y + 3, z,
							facing_of({-side.first, -side.second}), *platform.name);
				} else if (!roof) {
					for (int dy = 0; dy < 3; ++dy)
						editor.set_block_absolute(STONE_BRICK_WALL, x, surface_y + dy, z);
				}
			} else if (roof && slice % 8 == 0) {
				for (int dy = 0; dy < (platform.low ? 3 : 4); ++dy)
					editor.set_block_absolute(ANDESITE_WALL, x, surface_y + dy, z);
			} else if (roof && !platform.low && slice % 8 == 4) {
				BlockWithProperties lantern{
						LANTERN, BlockWithProperties::Properties{{"hanging", "true"}}};
				editor.set_block_with_properties_absolute(lantern, x, surface_y + 3, z);
			} else if (!roof && !platform.low && slice % 14 == 7) {
				for (int dy = 0; dy < 3; ++dy)
					editor.set_block_absolute(ANDESITE_WALL, x, surface_y + dy, z);
				editor.set_block_absolute(LANTERN, x, surface_y + 3, z);
			} else if (width >= 5 && slice % 16 == 3) {
				const auto a = facing_of(side),
						   b = facing_of({-side.first, -side.second});
				BlockWithProperties bench_b{OAK_STAIRS,
						BlockWithProperties::Properties{
								{"facing", stair_facing(b)}, {"half", "bottom"}}};
				BlockWithProperties bench_a{OAK_STAIRS,
						BlockWithProperties::Properties{
								{"facing", stair_facing(a)}, {"half", "bottom"}}};
				editor.set_block_with_properties_absolute(bench_b, x, surface_y, z);
				editor.set_block_with_properties_absolute(
						bench_a, x + side.first, surface_y, z + side.second);
			} else if (width >= 4 && slice % 16 == 5) {
				editor.set_block_absolute(CAULDRON, x, surface_y, z);
			} else if (platform.low && roof && slice % 3 == 1) {
				const auto a = facing_of(side);
				BlockWithProperties bench{OAK_STAIRS,
						BlockWithProperties::Properties{
								{"facing", stair_facing(a)}, {"half", "bottom"}}};
				editor.set_block_with_properties_absolute(bench, x, surface_y, z);
			}
		}
	}
}

SparseCells surface_track_tree_clearance(const std::vector<ProcessedElement> &elements)
{
	return surface_track_tree_clearance(elements, collect_platforms(elements));
}

SparseCells surface_track_tree_clearance(const std::vector<ProcessedElement> &elements,
		const std::vector<Platform> &platforms, const TunnelPinIndex *station_index)
{
	SparseCells result;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto level = railways::track_level(way);
		if (!level || *level != railways::TrackLevel::Surface)
			continue;
		const int reach = tag(way.tags, "railway") == "tram" ? 1 : TRACK_TREE_REACH;
		for (const auto &[x, z] : railways::track_cells(way).cells)
			result.set_square(x, z, reach);
	}
	// Rust's StationIndex::build also keeps trees off the station platform
	// surface, not just the railway corridor that feeds it.
	for (const auto &platform : platforms) {
		if (!station_index &&
				(platform.underground || (!platform.surface && !platform.low)))
			continue;
		for (const auto &[x, z] : platform.cells)
			if (!station_index || station_index->is_platform(x, z))
				result.set(x, z);
	}
	return result;
}

TrackBedIndex surface_track_beds(const std::vector<ProcessedElement> &elements,
		const std::vector<Platform> &platforms, const WorldEditor &editor,
		const bridges::BridgeStructureMap &bridge_structures)
{
	std::unordered_map<Xz, std::vector<TrackCell>, XzHash> tracks;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto level = railways::track_level(way);
		if (!level || *level != railways::TrackLevel::Surface ||
				bridge_structures.rail_deck(way.id))
			continue;
		const auto cells = railways::track_cells(way).cells;
		if (cells.empty())
			continue;
		for (std::size_t i = 0; i < cells.size(); ++i) {
			const auto a = cells[i > HEADING_CELLS ? i - HEADING_CELLS : 0];
			const auto b = cells[std::min(i + HEADING_CELLS, cells.size() - 1)];
			const double dx = b.first - a.first;
			const double dz = b.second - a.second;
			const double length = std::max(std::hypot(dx, dz), 1e-3);
			tracks[cells[i]].push_back({way.id, cells[i].first, cells[i].second, false,
					{dx / length, dz / length}});
		}
	}

	struct AccumulatedBed
	{
		long long sum{};
		int count{};
	};
	std::unordered_map<TrackBedIndex::Key, AccumulatedBed, TrackBedIndex::Hash>
			accumulated;
	for (const auto &platform : platforms) {
		if (platform.low || platform.underground || !platform.surface ||
				platform.cells.empty())
			continue;
		const auto frame = cell_frame(platform.cells);
		double along_min = std::numeric_limits<double>::infinity();
		double along_max = -along_min;
		double across_min = std::numeric_limits<double>::infinity();
		double across_max = -across_min;
		for (const auto &[x, z] : platform.cells) {
			const auto along = frame.along(x, z);
			const auto across = frame.across(x, z);
			along_min = std::min(along_min, along);
			along_max = std::max(along_max, along);
			across_min = std::min(across_min, across);
			across_max = std::max(across_max, across);
		}
		const bool headed =
				along_max - along_min > 2.0 * std::max(1.0, across_max - across_min);
		const double min_parallel = std::cos(PARALLEL_DEG * std::numbers::pi / 180.0);
		std::unordered_set<Xz, XzHash> platform_cells(
				platform.cells.begin(), platform.cells.end());
		std::unordered_map<TrackBedIndex::Key, TrackCell, TrackBedIndex::Hash> adjacent;
		for (const auto &[x, z] : platform.cells) {
			const bool edge = !platform_cells.contains({x - 1, z}) ||
							  !platform_cells.contains({x + 1, z}) ||
							  !platform_cells.contains({x, z - 1}) ||
							  !platform_cells.contains({x, z + 1});
			const int reach = edge ? TUNNEL_TRACK_REACH : 0;
			for (int dx = -reach; dx <= reach; ++dx) {
				for (int dz = -reach; dz <= reach; ++dz) {
					const auto found = tracks.find({x + dx, z + dz});
					if (found == tracks.end())
						continue;
					for (const auto &track : found->second) {
						if (std::max(std::abs(dx), std::abs(dz)) > TRACK_REACH)
							continue;
						if (headed &&
								std::abs(track.dir.first * frame.ux +
										 track.dir.second * frame.uz) < min_parallel)
							continue;
						adjacent.try_emplace(
								TrackBedIndex::Key{track.way, track.x, track.z}, track);
					}
				}
			}
		}
		if (adjacent.empty())
			continue;

		double mean_t = 0.0, mean_y = 0.0;
		std::vector<std::pair<double, double>> samples;
		samples.reserve(platform.cells.size());
		for (const auto &[x, z] : platform.cells) {
			const double t = frame.along(x, z);
			const double y = editor.get_ground_level(x, z) + 1.0;
			samples.emplace_back(t, y);
			mean_t += t;
			mean_y += y;
		}
		mean_t /= samples.size();
		mean_y /= samples.size();
		double lo = std::numeric_limits<double>::infinity();
		double hi = -lo;
		double variance = 0.0, covariance = 0.0;
		for (const auto &[t, y] : samples) {
			lo = std::min(lo, y);
			hi = std::max(hi, y);
			variance += (t - mean_t) * (t - mean_t);
			covariance += (t - mean_t) * (y - mean_y);
		}
		const double slope =
				hi - lo <= 2.0
						? 0.0
						: std::clamp(covariance / std::max(variance, 1.0), -0.12, 0.12);
		for (const auto &[key, track] : adjacent) {
			const int bed =
					static_cast<int>(std::llround(
							mean_y + slope * (frame.along(track.x, track.z) - mean_t))) -
					1;
			auto &entry = accumulated[key];
			entry.sum += bed;
			++entry.count;
		}
	}
	TrackBedIndex result;
	for (const auto &[key, entry] : accumulated)
		if (entry.count)
			result.set(key.way, key.x, key.z,
					static_cast<int>(std::llround(double(entry.sum) / entry.count)));
	return result;
}

TunnelPinIndex underground_track_pins(const std::vector<ProcessedElement> &elements,
		const std::vector<Platform> &platforms, const WorldEditor &editor,
		const railways::RailJoins &rail_joins, const XZBBox &bounds,
		const CoordinateBitmap &tunnel_footprint,
		const CoordinateBitmap &building_footprints, double scale, bool allow_underground)
{
	// Classify an untagged platform by the majority of its adjacent tracks, as
	// StationIndex does. Looking only for tunnels mistakes a surface stop above
	// a subway for an underground station.
	TunnelPinIndex result;
	if (platforms.empty())
		return result;
	// StationIndex indexes the 3x3 block halo around platforms rather than every
	// rail cell in a large extraction, which keeps this prescan bounded.
	std::unordered_set<Xz, XzHash> near;
	for (const auto &platform : platforms)
		for (const auto &[x, z] : platform.cells)
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz)
					near.emplace((x >> 4) + dx, (z >> 4) + dz);
	std::unordered_map<Xz, std::vector<TrackCell>, XzHash> tracks;
	std::unordered_map<std::uint64_t, std::vector<Xz>> tunnel_ways;
	std::unordered_map<std::uint64_t, std::vector<std::uint64_t>> tunnel_nodes;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto level = railways::track_level(way);
		if (!level)
			continue;
		const auto cells = railways::track_cells(way).cells;
		if (cells.empty())
			continue;
		if (*level == railways::TrackLevel::Tunnel &&
				std::any_of(cells.begin(), cells.end(), [&](const Xz &cell) {
					return near.contains({cell.first >> 4, cell.second >> 4});
				})) {
			tunnel_ways.try_emplace(way.id, cells);
			auto &nodes = tunnel_nodes[way.id];
			nodes.reserve(way.nodes.size());
			for (const auto &node : way.nodes)
				nodes.push_back(node.id);
		}
		for (std::size_t i = 0; i < cells.size(); ++i) {
			if (!near.contains({cells[i].first >> 4, cells[i].second >> 4}))
				continue;
			const auto a = cells[i > HEADING_CELLS ? i - HEADING_CELLS : 0];
			const auto b = cells[std::min(i + HEADING_CELLS, cells.size() - 1)];
			const double dx = b.first - a.first, dz = b.second - a.second;
			const double length = std::max(std::hypot(dx, dz), 1e-3);
			tracks[cells[i]].push_back({way.id, cells[i].first, cells[i].second,
					*level == railways::TrackLevel::Tunnel, {dx / length, dz / length}});
		}
	}
	const int lowest = world_editor::min_y() + 1;
	struct Candidate
	{
		std::unordered_set<TrackBedIndex::Key, TrackBedIndex::Hash> tracks;
		std::unordered_set<Xz, XzHash> core;
		std::unordered_set<Xz, XzHash> platform_cells;
		std::pair<double, double> heading{};
		int depth{-1};
		std::uint64_t seed{};
		std::optional<Block> accent;
		std::optional<std::string> name;
	};
	std::vector<Candidate> candidates;
	std::unordered_map<TrackBedIndex::Key, std::size_t, TrackBedIndex::Hash> owner;
	for (const auto &platform : platforms) {
		if (platform.cells.empty())
			continue;
		const auto frame = cell_frame(platform.cells);
		double along_min = std::numeric_limits<double>::infinity();
		double along_max = -along_min;
		double across_min = std::numeric_limits<double>::infinity();
		double across_max = -across_min;
		std::unordered_set<Xz, XzHash> platform_cells(
				platform.cells.begin(), platform.cells.end());
		for (const auto &[x, z] : platform.cells) {
			along_min = std::min(along_min, frame.along(x, z));
			along_max = std::max(along_max, frame.along(x, z));
			across_min = std::min(across_min, frame.across(x, z));
			across_max = std::max(across_max, frame.across(x, z));
		}
		const bool headed =
				along_max - along_min > 2.0 * std::max(1.0, across_max - across_min);
		std::unordered_map<TrackBedIndex::Key, TrackCell, TrackBedIndex::Hash> adjacent;
		for (const auto &[x, z] : platform.cells) {
			const bool edge = !platform_cells.contains({x - 1, z}) ||
							  !platform_cells.contains({x + 1, z}) ||
							  !platform_cells.contains({x, z - 1}) ||
							  !platform_cells.contains({x, z + 1});
			const int reach = edge ? TUNNEL_TRACK_REACH : 0;
			for (int dx = -reach; dx <= reach; ++dx)
				for (int dz = -reach; dz <= reach; ++dz) {
					const auto found = tracks.find({x + dx, z + dz});
					if (found == tracks.end())
						continue;
					for (const auto &track : found->second) {
						const int track_reach =
								track.tunnel ? TUNNEL_TRACK_REACH : TRACK_REACH;
						if (std::max(std::abs(dx), std::abs(dz)) <= track_reach &&
								(!headed || std::abs(track.dir.first * frame.ux +
													 track.dir.second * frame.uz) >=
													std::cos(PARALLEL_DEG *
															 std::numbers::pi / 180.0)))
							adjacent.try_emplace(
									TrackBedIndex::Key{track.way, track.x, track.z},
									track);
					}
				}
		}
		const auto tunnel_count = std::count_if(adjacent.begin(), adjacent.end(),
				[](const auto &entry) { return entry.second.tunnel; });
		const bool underground =
				platform.underground ? true
				: (platform.low || platform.surface)
						? false
						: tunnel_count * 2 > static_cast<std::ptrdiff_t>(adjacent.size());
		if (!underground) {
			std::unordered_set<Xz, XzHash> bed_zone;
			for (const auto &[key, track] : adjacent) {
				if (track.tunnel)
					continue;
				for (int dx = -1; dx <= 1; ++dx)
					for (int dz = -1; dz <= 1; ++dz)
						bed_zone.emplace(track.x + dx, track.z + dz);
			}
			for (const auto &cell : platform_cells)
				if (!bed_zone.contains(cell))
					result.set_platform_cell(cell.first, cell.second);
			continue;
		}
		if (!allow_underground)
			continue;
		std::erase_if(adjacent, [](const auto &entry) { return !entry.second.tunnel; });
		if (adjacent.empty())
			continue;
		std::unordered_set<Xz, XzHash> core = platform_cells;
		for (const auto &[key, track] : adjacent) {
			(void)track;
			core.emplace(track.x, track.z);
		}
		Candidate candidate;
		candidate.core = std::move(core);
		candidate.platform_cells = std::move(platform_cells);
		candidate.depth = platform.depth.value_or(-1);
		candidate.seed = platform.id;
		candidate.accent = platform.colour;
		candidate.name = platform.name;
		std::vector<Xz> track_cells;
		for (const auto &[key, track] : adjacent) {
			candidate.tracks.insert(key);
			track_cells.emplace_back(track.x, track.z);
		}
		std::sort(track_cells.begin(), track_cells.end());
		const auto track_frame = cell_frame(track_cells);
		candidate.heading = {track_frame.ux, track_frame.uz};
		candidates.push_back(std::move(candidate));
	}
	// Platforms served by a shared tunnel cell are one station. Resolve their
	// common floor together instead of letting whichever platform is visited
	// last overwrite the pin for that track.
	std::vector<std::size_t> parent(candidates.size());
	for (std::size_t i = 0; i < parent.size(); ++i)
		parent[i] = i;
	auto find_root = [&](std::size_t i) {
		std::size_t root = i;
		while (parent[root] != root)
			root = parent[root];
		while (parent[i] != i) {
			const auto next = parent[i];
			parent[i] = root;
			i = next;
		}
		return root;
	};
	for (std::size_t i = 0; i < candidates.size(); ++i)
		for (const auto &key : candidates[i].tracks) {
			const auto [it, inserted] = owner.emplace(key, i);
			if (!inserted) {
				const auto a = find_root(i), b = find_root(it->second);
				if (a != b)
					parent[std::max(a, b)] = std::min(a, b);
			}
		}
	const double parallel_limit = std::cos(PARALLEL_DEG * std::numbers::pi / 180.0);
	struct Group
	{
		std::unordered_set<TrackBedIndex::Key, TrackBedIndex::Hash> tracks;
		std::unordered_set<Xz, XzHash> core;
		std::unordered_set<Xz, XzHash> platform_cells;
		std::uint64_t seed{std::numeric_limits<std::uint64_t>::max()};
		int depth{16};
		std::optional<Block> accent;
		std::optional<std::string> name;
	};
	std::unordered_map<std::size_t, Group> groups;
	for (std::size_t i = 0; i < candidates.size(); ++i) {
		auto &group = groups[find_root(i)];
		group.tracks.insert(candidates[i].tracks.begin(), candidates[i].tracks.end());
		group.core.insert(candidates[i].core.begin(), candidates[i].core.end());
		group.platform_cells.insert(
				candidates[i].platform_cells.begin(), candidates[i].platform_cells.end());
		group.seed = std::min(group.seed, candidates[i].seed);
		group.depth = std::min(group.depth, candidates[i].depth);
		if (!group.accent && candidates[i].accent)
			group.accent = candidates[i].accent;
		if (!group.name && candidates[i].name)
			group.name = candidates[i].name;
	}
	std::vector<std::size_t> ordered_groups;
	ordered_groups.reserve(groups.size());
	for (const auto &[root, group] : groups) {
		(void)group;
		ordered_groups.push_back(root);
	}
	std::sort(ordered_groups.begin(), ordered_groups.end(),
			[&](std::size_t a, std::size_t b) {
				const auto &ga = groups.at(a);
				const auto &gb = groups.at(b);
				if (ga.depth != gb.depth)
					return ga.depth > gb.depth;
				return std::tie(ga.seed, a) < std::tie(gb.seed, b);
			});
	for (const auto root : ordered_groups) {
		const auto &group = groups.at(root);
		int surface = std::numeric_limits<int>::max();
		for (const auto &[x, z] : group.core) {
			for (int dx = -2; dx <= 2; ++dx)
				for (int dz = -2; dz <= 2; ++dz) {
					const int px = x + dx, pz = z + dz;
					const int ground = editor.terrain_level(px, pz).value_or(
							editor.get_ground_level(px, pz));
					const int local =
							editor.lowest_carved_bed(px, pz, 0).value_or(ground);
					surface = std::min(surface, std::min(ground, local));
				}
		}
		int floor = std::max(surface - HALL_DEPTH, lowest);

		// Rust expands each grouped platform/track core by two blocks to form the
		// shell footprint. Keep the set sparse: stations may span many chunks.
		std::unordered_set<Xz, XzHash> footprint;
		footprint.reserve(group.core.size() * 8);
		for (const auto &[x, z] : group.core)
			for (int dx = -2; dx <= 2; ++dx)
				for (int dz = -2; dz <= 2; ++dz)
					footprint.emplace(x + dx, z + dz);
		std::array<int, 4> footprint_box{std::numeric_limits<int>::max(),
				std::numeric_limits<int>::max(), std::numeric_limits<int>::min(),
				std::numeric_limits<int>::min()};
		for (const auto &[x, z] : footprint) {
			footprint_box[0] = std::min(footprint_box[0], x);
			footprint_box[1] = std::min(footprint_box[1], z);
			footprint_box[2] = std::max(footprint_box[2], x);
			footprint_box[3] = std::max(footprint_box[3], z);
		}
		std::unordered_set<Xz, XzHash> track_cells;
		track_cells.reserve(group.tracks.size());
		for (const auto &key : group.tracks)
			track_cells.emplace(key.x, key.z);
		std::vector<Xz> hall_tracks(track_cells.begin(), track_cells.end());
		std::sort(hall_tracks.begin(), hall_tracks.end());
		const auto heading = cell_frame(hall_tracks);
		std::unordered_set<Xz, XzHash> bed_zone;
		bed_zone.reserve(track_cells.size() * 9);
		for (const auto &[x, z] : track_cells)
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz)
					bed_zone.emplace(x + dx, z + dz);
		std::unordered_set<Xz, XzHash> interior;
		interior.reserve(group.core.size() * 9);
		for (const auto &[x, z] : group.core)
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz)
					interior.emplace(x + dx, z + dz);
		std::vector<Xz> core_cells(group.core.begin(), group.core.end());
		std::sort(core_cells.begin(), core_cells.end());
		const auto frame = cell_frame(core_cells);
		const auto along_step = std::abs(frame.ux) >= std::abs(frame.uz)
										? Xz{frame.ux < 0.0 ? -1 : 1, 0}
										: Xz{0, frame.uz < 0.0 ? -1 : 1};
		const Xz platform_side{-along_step.second, along_step.first};
		const auto facing_of = [](const Xz &direction) -> std::int8_t {
			if (direction.first == 0)
				return direction.second < 0 ? 2 : 3;
			return direction.first < 0 ? 4 : 5;
		};

		TunnelHall hall;
		hall.bbox = footprint_box;
		hall.heading = {heading.ux, heading.uz};
		hall.depth = group.depth;
		hall.interior = interior;
		hall.level_tracks = track_cells;
		hall.layout_frame = {frame.cx, frame.cz, frame.ux, frame.uz};
		for (const auto &key : group.tracks) {
			hall.ways.insert(key.way);
			hall.line_ways.insert(key.way);
		}
		for (const auto &cell : footprint) {
			const auto cell_tracks = tracks.find(cell);
			if (cell_tracks == tracks.end())
				continue;
			for (const auto &track : cell_tracks->second) {
				if (!track.tunnel)
					continue;
				hall.through_tracks.push_back({track.way, track.x, track.z, track.dir});
			}
		}
		std::sort(hall.through_tracks.begin(), hall.through_tracks.end(),
				[](const HallTrack &a, const HallTrack &b) {
					return std::tie(a.way, a.x, a.z) < std::tie(b.way, b.x, b.z);
				});
		hall.through_tracks.erase(
				std::unique(hall.through_tracks.begin(), hall.through_tracks.end(),
						[](const HallTrack &a, const HallTrack &b) {
							return a.way == b.way && a.x == b.x && a.z == b.z;
						}),
				hall.through_tracks.end());
		std::unordered_set<std::uint64_t> line_nodes;
		for (const auto way : hall.ways)
			if (const auto found = tunnel_nodes.find(way); found != tunnel_nodes.end())
				line_nodes.insert(found->second.begin(), found->second.end());
		std::vector<std::uint64_t> through_ways;
		for (const auto &track : hall.through_tracks)
			through_ways.push_back(track.way);
		std::sort(through_ways.begin(), through_ways.end());
		through_ways.erase(std::unique(through_ways.begin(), through_ways.end()),
				through_ways.end());
		bool added_line_way = true;
		while (added_line_way) {
			added_line_way = false;
			for (const auto way : through_ways) {
				if (hall.line_ways.contains(way))
					continue;
				const auto found = tunnel_nodes.find(way);
				if (found == tunnel_nodes.end() ||
						!std::any_of(found->second.begin(), found->second.end(),
								[&](std::uint64_t node) {
									return line_nodes.contains(node);
								}))
					continue;
				hall.line_ways.insert(way);
				line_nodes.insert(found->second.begin(), found->second.end());
				added_line_way = true;
			}
		}
		if (floor < lowest || surface - floor < HALL_MIN_DEPTH)
			continue;
		hall.floor = floor;
		std::uint64_t name_seed = group.seed;
		if (group.name) {
			name_seed = 0xcbf29ce484222325ULL;
			for (const unsigned char byte : *group.name)
				name_seed = (name_seed ^ byte) * 0x100000001b3ULL;
		}
		hall.style = static_cast<std::size_t>(structure_kit::mix(name_seed, 1) % 3);
		hall.accent =
				group.accent.value_or(std::array<Block, 8>{BLUE_CONCRETE, ORANGE_CONCRETE,
						CYAN_CONCRETE, GREEN_CONCRETE, RED_CONCRETE, YELLOW_CONCRETE,
						PURPLE_CONCRETE, LIGHT_BLUE_CONCRETE}[static_cast<std::size_t>(
						structure_kit::mix(name_seed, 2) % 8)]);
		hall.bbox = {std::numeric_limits<int>::max(), std::numeric_limits<int>::max(),
				std::numeric_limits<int>::min(), std::numeric_limits<int>::min()};
		hall.cells.reserve(footprint.size());
		for (const auto &[x, z] : footprint) {
			hall.bbox[0] = std::min(hall.bbox[0], x);
			hall.bbox[1] = std::min(hall.bbox[1], z);
			hall.bbox[2] = std::max(hall.bbox[2], x);
			hall.bbox[3] = std::max(hall.bbox[3], z);
			const bool on_bed = bed_zone.contains({x, z});
			const bool inside = interior.contains({x, z});
			const auto part =
					!inside	 ? (on_bed ? TunnelHallPart::Portal : TunnelHallPart::Wall)
					: on_bed ? TunnelHallPart::Trackbed
							 : TunnelHallPart::Platform;
			TunnelHallCell cell;
			cell.x = x;
			cell.z = z;
			cell.part = part;
			if (part == TunnelHallPart::Platform) {
				const auto faces_track = [&](int px, int pz) {
					return bed_zone.contains({px - 1, pz}) ||
						   bed_zone.contains({px + 1, pz}) ||
						   bed_zone.contains({px, pz - 1}) ||
						   bed_zone.contains({px, pz + 1});
				};
				if (faces_track(x, z))
					cell.deck = TunnelHallDeck::Edge;
				else if ((faces_track(x - 1, z) &&
								 group.platform_cells.contains({x - 1, z})) ||
						 (faces_track(x + 1, z) &&
								 group.platform_cells.contains({x + 1, z})) ||
						 (faces_track(x, z - 1) &&
								 group.platform_cells.contains({x, z - 1})) ||
						 (faces_track(x, z + 1) &&
								 group.platform_cells.contains({x, z + 1})))
					cell.deck = TunnelHallDeck::Line;
			}
			hall.cells.push_back(cell);
		}
		std::sort(hall.cells.begin(), hall.cells.end(), [](const auto &a, const auto &b) {
			return a.x < b.x || (a.x == b.x && a.z < b.z);
		});
		const double t0 = std::accumulate(hall.cells.begin(), hall.cells.end(),
				std::numeric_limits<double>::infinity(),
				[&](double value, const auto &cell) {
					return std::min(value, frame.along(cell.x, cell.z));
				});
		std::unordered_map<int, std::vector<std::size_t>> slices;
		for (std::size_t i = 0; i < hall.cells.size(); ++i) {
			const auto &cell = hall.cells[i];
			if (cell.part == TunnelHallPart::Platform)
				slices[static_cast<int>(std::lround(frame.along(cell.x, cell.z) - t0))]
						.push_back(i);
		}
		for (auto &[slice, positions] : slices) {
			std::sort(positions.begin(), positions.end(),
					[&](std::size_t a, std::size_t b) {
						return frame.across(hall.cells[a].x, hall.cells[a].z) <
							   frame.across(hall.cells[b].x, hall.cells[b].z);
					});
			std::size_t first = 0;
			while (first < positions.size()) {
				std::size_t last = first + 1;
				while (last < positions.size() &&
						frame.across(hall.cells[positions[last]].x,
								hall.cells[positions[last]].z) -
										frame.across(hall.cells[positions[last - 1]].x,
												hall.cells[positions[last - 1]].z) <
								1.6)
					++last;
				const std::size_t width = last - first;
				const std::size_t centre = positions[first + width / 2];
				const int t = slice;
				if ((t % 3 + 3) % 3 == 1) {
					if (width < 7)
						hall.cells[centre].light = true;
					else {
						hall.cells[positions[first + width / 2 - 2]].light = true;
						hall.cells[positions[first + width / 2 + 2]].light = true;
					}
				}
				if (width >= 6 && (t % 8 + 8) % 8 == 4)
					hall.cells[centre].prop = TunnelHallProp::Pillar;
				else if (width >= 5 && (t % 8 + 8) % 8 == 0 && width / 2 >= 1) {
					hall.cells[centre].prop = TunnelHallProp::Bench;
					hall.cells[centre].facing_a = facing_of(platform_side);
					hall.cells[centre].facing_b =
							facing_of({-platform_side.first, -platform_side.second});
				} else if (width >= 4 && (t % 16 + 16) % 16 == 2)
					hall.cells[centre].prop = TunnelHallProp::Bin;
				first = last;
			}
		}
		for (auto &cell : hall.cells) {
			if (cell.part != TunnelHallPart::Trackbed ||
					!track_cells.contains({cell.x, cell.z}))
				continue;
			const int t = static_cast<int>(std::lround(frame.along(cell.x, cell.z) - t0));
			cell.light = (t % 6 + 6) % 6 == 1;
		}
		hall.footprint = footprint;
		for (const auto &cell : hall.cells) {
			if (cell.part != TunnelHallPart::Platform)
				continue;
			hall.platform.emplace(cell.x, cell.z);
			if (cell.deck == TunnelHallDeck::Edge)
				hall.platform_edge.emplace(cell.x, cell.z);
		}
		result.add_hall(std::move(hall));
	}
	// Reconcile foreign lines only after all halls are known. Rust first builds
	// the way-to-own-halls index, so a crossing line's own hall can take priority
	// even when that hall is processed later in the deterministic order.
	auto &halls = result.halls();
	const auto parallel = [&](const TunnelHall &a, const TunnelHall &b) {
		return std::abs(a.heading.first * b.heading.first +
						a.heading.second * b.heading.second) >= parallel_limit;
	};
	const auto boxes_within = [](const TunnelHall &a, const TunnelHall &b, int reach) {
		return a.bbox[0] <= b.bbox[2] + reach && b.bbox[0] <= a.bbox[2] + reach &&
			   a.bbox[1] <= b.bbox[3] + reach && b.bbox[1] <= a.bbox[3] + reach;
	};
	std::vector<std::size_t> sibling_parent(halls.size());
	std::iota(sibling_parent.begin(), sibling_parent.end(), 0);
	auto sibling_root = [&](std::size_t i) {
		while (sibling_parent[i] != i) {
			sibling_parent[i] = sibling_parent[sibling_parent[i]];
			i = sibling_parent[i];
		}
		return i;
	};
	for (std::size_t i = 0; i < halls.size(); ++i)
		for (std::size_t j = i + 1; j < halls.size(); ++j)
			if (boxes_within(halls[i], halls[j], SIBLING_REACH) &&
					parallel(halls[i], halls[j]) &&
					std::abs(halls[i].depth - halls[j].depth) <= 1) {
				const auto a = sibling_root(i), b = sibling_root(j);
				if (a != b)
					sibling_parent[std::max(a, b)] = std::min(a, b);
			}
	const auto crosses = [&](const TunnelHall &a, const TunnelHall &b) {
		return std::any_of(a.through_tracks.begin(), a.through_tracks.end(),
				[&](const HallTrack &track) {
					if (!b.ways.contains(track.way) || a.ways.contains(track.way))
						return false;
					const double along = std::abs(track.dir.first * a.heading.first +
												  track.dir.second * a.heading.second);
					return along < parallel_limit && track.x >= b.bbox[0] - STACK_REACH &&
						   track.x <= b.bbox[2] + STACK_REACH &&
						   track.z >= b.bbox[1] - STACK_REACH &&
						   track.z <= b.bbox[3] + STACK_REACH;
				});
	};
	const auto stacked = [&](std::size_t a_index, std::size_t b_index) {
		if (sibling_root(a_index) == sibling_root(b_index))
			return false;
		const auto &a = halls[a_index];
		const auto &b = halls[b_index];
		if (!boxes_within(a, b, STACK_REACH))
			return false;
		const bool joined =
				std::any_of(b.ways.begin(), b.ways.end(),
						[&](std::uint64_t way) { return a.line_ways.contains(way); }) ||
				std::any_of(a.ways.begin(), a.ways.end(),
						[&](std::uint64_t way) { return b.line_ways.contains(way); });
		if (joined && parallel(a, b))
			return false;
		const auto &small =
				a.footprint.size() <= b.footprint.size() ? a.footprint : b.footprint;
		const auto &large =
				a.footprint.size() <= b.footprint.size() ? b.footprint : a.footprint;
		return std::any_of(small.begin(), small.end(),
					   [&](const Xz &cell) { return large.contains(cell); }) ||
			   crosses(a, b) || crosses(b, a);
	};
	// Rust reconciles sibling floors and crossing constraints to a fixed point.
	for (int pass = 0; pass < 4; ++pass) {
		std::vector<int> sibling_floor(halls.size(), std::numeric_limits<int>::max());
		for (std::size_t i = 0; i < halls.size(); ++i) {
			if (!halls[i].active)
				continue;
			const auto root = sibling_root(i);
			sibling_floor[root] = std::min(sibling_floor[root], halls[i].floor);
		}
		for (std::size_t i = 0; i < halls.size(); ++i)
			if (halls[i].active)
				halls[i].floor = std::min(halls[i].floor, sibling_floor[sibling_root(i)]);
		for (std::size_t k = 0; k < halls.size(); ++k)
			if (halls[k].active)
				for (std::size_t j = 0; j < k; ++j)
					if (halls[j].active && stacked(j, k) &&
							halls[k].floor > halls[j].floor - HALL_STACK) {
						const int lower = halls[j].floor - HALL_STACK;
						if (lower >= lowest)
							halls[k].floor = lower;
						else
							halls[k].active = false;
					}
	}
	result.discard_inactive_halls();
	// Erasing floors that could not fit changes hall indices; rebuild the same
	// transitive sibling sets for the ownership checks that follow.
	sibling_parent.assign(halls.size(), 0);
	std::iota(sibling_parent.begin(), sibling_parent.end(), 0);
	for (std::size_t i = 0; i < halls.size(); ++i)
		for (std::size_t j = i + 1; j < halls.size(); ++j)
			if (boxes_within(halls[i], halls[j], SIBLING_REACH) &&
					parallel(halls[i], halls[j]) &&
					std::abs(halls[i].depth - halls[j].depth) <= 1) {
				const auto a = sibling_root(i), b = sibling_root(j);
				if (a != b)
					sibling_parent[std::max(a, b)] = std::min(a, b);
			}
	for (const auto &hall : halls)
		for (const auto &track : hall.through_tracks)
			if (hall.ways.contains(track.way))
				result.set(track.way, track.x, track.z, TunnelPin{hall.floor, true});
	std::unordered_map<std::uint64_t, std::vector<std::size_t>> halls_of;
	for (std::size_t i = 0; i < halls.size(); ++i)
		for (const auto way : halls[i].ways)
			halls_of[way].push_back(i);
	for (std::size_t i = 0; i < halls.size(); ++i) {
		auto &hall = halls[i];
		std::map<std::uint64_t, std::vector<HallTrack>> foreign_tracks;
		for (const auto &track : hall.through_tracks)
			if (!hall.ways.contains(track.way))
				foreign_tracks[track.way].push_back(track);
		for (const auto &[way, cells] : foreign_tracks) {
			const auto own_it = halls_of.find(way);
			static const std::vector<std::size_t> no_own_halls;
			const auto &own = own_it == halls_of.end() ? no_own_halls : own_it->second;
			const bool level =
					hall.line_ways.contains(way) ||
					std::any_of(own.begin(), own.end(),
							[&](std::size_t j) {
								return halls[j].floor == hall.floor;
							}) ||
					(own.empty() &&
							std::count_if(cells.begin(), cells.end(),
									[&](const HallTrack &track) {
										const double along = std::abs(
												track.dir.first * hall.heading.first +
												track.dir.second * hall.heading.second);
										return along >= parallel_limit;
									}) * 2 >
									static_cast<std::ptrdiff_t>(cells.size()));
			const bool own_hall_above =
					!level && std::any_of(own.begin(), own.end(), [&](std::size_t j) {
						return halls[j].floor > hall.floor && stacked(j, i);
					});
			if (own_hall_above)
				continue;
			for (const auto &track : cells) {
				if (const auto existing = result.get(way, track.x, track.z);
						existing && existing->in_hall)
					continue;
				const int under = hall.floor - HALL_STACK;
				if (!level && under >= lowest) {
					const auto old = result.get(way, track.x, track.z);
					result.set(way, track.x, track.z,
							TunnelPin{old ? std::min(old->floor, under) : under, false});
				} else {
					hall.level_tracks.emplace(track.x, track.z);
					result.set(way, track.x, track.z, TunnelPin{hall.floor, true});
				}
			}
		}
	}
	// Foreign lines that stay at this hall's level are trackbeds too, not merely
	// rail pins: the Rust renderer dilates all level tracks before classifying
	// trackbed, portal and platform cells.
	for (auto &hall : halls) {
		std::unordered_set<Xz, XzHash> bed_zone;
		for (const auto &[x, z] : hall.level_tracks)
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz)
					bed_zone.emplace(x + dx, z + dz);
		std::unordered_set<Xz, XzHash> platform_cells;
		for (const auto &cell : hall.interior)
			if (!bed_zone.contains(cell))
				platform_cells.insert(cell);
		hall.platform.clear();
		hall.platform_edge.clear();
		const auto faces_track = [&](int x, int z) {
			return bed_zone.contains({x - 1, z}) || bed_zone.contains({x + 1, z}) ||
				   bed_zone.contains({x, z - 1}) || bed_zone.contains({x, z + 1});
		};
		for (auto &cell : hall.cells) {
			cell.prop = TunnelHallProp::None;
			cell.light = false;
			cell.facing_a = 0;
			cell.facing_b = 0;
			const Xz pos{cell.x, cell.z};
			if (!hall.interior.contains(pos))
				cell.part = bed_zone.contains(pos) ? TunnelHallPart::Portal
												   : TunnelHallPart::Wall;
			else if (bed_zone.contains(pos))
				cell.part = TunnelHallPart::Trackbed;
			else if (faces_track(cell.x, cell.z))
				cell.part = TunnelHallPart::Platform;
			else {
				bool line_deck = false;
				for (const auto &[dx, dz] :
						std::array<Xz, 4>{{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}})
					if (faces_track(cell.x + dx, cell.z + dz) &&
							platform_cells.contains({cell.x + dx, cell.z + dz})) {
						line_deck = true;
						break;
					}
				cell.part = TunnelHallPart::Platform;
				cell.deck = line_deck ? TunnelHallDeck::Line : TunnelHallDeck::Inner;
			}
			if (cell.part == TunnelHallPart::Platform) {
				hall.platform.emplace(pos);
				if (faces_track(cell.x, cell.z)) {
					cell.deck = TunnelHallDeck::Edge;
					hall.platform_edge.emplace(pos);
				}
			}
		}
		// Match Rust's final rendering pass: props and lighting are selected only
		// after all foreign tracks have contributed to the hall's bed zone.
		const auto [cx, cz, ux, uz] = hall.layout_frame;
		const auto along = [&](int x, int z) { return (x - cx) * ux + (z - cz) * uz; };
		const auto across = [&](int x, int z) { return -(x - cx) * uz + (z - cz) * ux; };
		double t0 = std::numeric_limits<double>::infinity();
		for (const auto &cell : hall.cells)
			t0 = std::min(t0, along(cell.x, cell.z));
		std::map<int, std::vector<std::size_t>> slices;
		for (std::size_t ci = 0; ci < hall.cells.size(); ++ci) {
			const auto &cell = hall.cells[ci];
			if (cell.part == TunnelHallPart::Platform)
				slices[static_cast<int>(std::lround(along(cell.x, cell.z) - t0))]
						.push_back(ci);
		}
		const int ax = std::abs(ux) >= std::abs(uz) ? (ux < 0.0 ? -1 : 1) : 0;
		const int az = std::abs(ux) >= std::abs(uz) ? 0 : (uz < 0.0 ? -1 : 1);
		const Xz platform_side{-az, ax};
		const auto facing_of = [](const Xz &direction) -> std::int8_t {
			if (direction.first == 0)
				return direction.second < 0 ? 2 : 3;
			return direction.first < 0 ? 4 : 5;
		};
		for (auto &[slice, positions] : slices) {
			std::sort(positions.begin(), positions.end(),
					[&](std::size_t a, std::size_t b) {
						return across(hall.cells[a].x, hall.cells[a].z) <
							   across(hall.cells[b].x, hall.cells[b].z);
					});
			std::size_t first = 0;
			while (first < positions.size()) {
				std::size_t last = first + 1;
				while (last < positions.size() &&
						across(hall.cells[positions[last]].x,
								hall.cells[positions[last]].z) -
										across(hall.cells[positions[last - 1]].x,
												hall.cells[positions[last - 1]].z) <
								1.6)
					++last;
				const std::size_t width = last - first;
				const std::size_t centre = positions[first + width / 2];
				const int t = slice;
				if ((t % 3 + 3) % 3 == 1) {
					if (width < 7)
						hall.cells[centre].light = true;
					else {
						hall.cells[positions[first + width / 2 - 2]].light = true;
						hall.cells[positions[first + width / 2 + 2]].light = true;
					}
				}
				if (width >= 6 && (t % 8 + 8) % 8 == 4)
					hall.cells[centre].prop = TunnelHallProp::Pillar;
				else if (width >= 5 && (t % 8 + 8) % 8 == 0 && width / 2 >= 1) {
					hall.cells[centre].prop = TunnelHallProp::Bench;
					hall.cells[centre].facing_a = facing_of(platform_side);
					hall.cells[centre].facing_b =
							facing_of({-platform_side.first, -platform_side.second});
				} else if (width >= 4 && (t % 16 + 16) % 16 == 2)
					hall.cells[centre].prop = TunnelHallProp::Bin;
				first = last;
			}
		}
		for (auto &cell : hall.cells) {
			if (cell.part != TunnelHallPart::Trackbed ||
					!hall.level_tracks.contains({cell.x, cell.z}))
				continue;
			const int t = static_cast<int>(std::lround(along(cell.x, cell.z) - t0));
			cell.light = (t % 6 + 6) % 6 == 1;
		}
	}
	// Route underground-platform entrances only after every hall has been
	// indexed, so competing routes can avoid halls above them and one another.
	if (!result.halls().empty()) {
		std::vector<const ProcessedNode *> entrances;
		for (const auto &element : elements)
			if (element.is_node()) {
				const auto &node = element.as_node();
				const auto railway = tag(node.tags, "railway");
				if (railway == "subway_entrance" || railway == "train_station_entrance")
					entrances.push_back(&node);
			}
		if (!entrances.empty()) {
			std::unordered_set<Xz, XzHash> buckets;
			for (const auto *node : entrances)
				for (int dx = -1; dx <= 1; ++dx)
					for (int dz = -1; dz <= 1; ++dz)
						buckets.emplace((node->x >> 5) + dx, (node->z >> 5) + dz);
			std::unordered_set<Xz, XzHash> carriageway, fenced;
			const auto near_entrance = [&](const ProcessedWay &way) {
				if (way.nodes.empty())
					return false;
				int min_x = way.nodes.front().x, max_x = min_x;
				int min_z = way.nodes.front().z, max_z = min_z;
				for (const auto &n : way.nodes) {
					min_x = std::min(min_x, n.x);
					max_x = std::max(max_x, n.x);
					min_z = std::min(min_z, n.z);
					max_z = std::max(max_z, n.z);
				}
				for (int bx = (min_x - 32) >> 5; bx <= (max_x + 32) >> 5; ++bx)
					for (int bz = (min_z - 32) >> 5; bz <= (max_z + 32) >> 5; ++bz)
						if (buckets.contains({bx, bz}))
							return true;
				return false;
			};
			for (const auto &element : elements) {
				if (!element.is_way())
					continue;
				const auto &way = element.as_way();
				const auto barrier = tag(way.tags, "barrier");
				if (!barrier.empty() && barrier != "kerb" && barrier != "ditch" &&
						near_entrance(way)) {
					for (std::size_t i = 1; i < way.nodes.size(); ++i)
						for (const auto &[x, y, z] :
								bresenham_line(way.nodes[i - 1].x, 0, way.nodes[i - 1].z,
										way.nodes[i].x, 0, way.nodes[i].z)) {
							(void)y;
							fenced.emplace(x, z);
						}
					continue;
				}
				const auto kind = highways::surface_highway_type(way);
				if (!kind || !highways::is_carriageway(*kind) || !near_entrance(way) ||
						((!tag(way.tags, "tunnel").empty() &&
								 tag(way.tags, "tunnel") != "no") ||
								(!tag(way.tags, "bridge").empty() &&
										tag(way.tags, "bridge") != "no")))
					continue;
				const int radius = highways::highway_block_range(*kind, way.tags, scale);
				for (std::size_t i = 1; i < way.nodes.size(); ++i)
					for (const auto &[x, y, z] : bresenham_line(way.nodes[i - 1].x, 0,
								 way.nodes[i - 1].z, way.nodes[i].x, 0, way.nodes[i].z)) {
						(void)y;
						if (!buckets.contains({x >> 5, z >> 5}))
							continue;
						for (int dx = -radius; dx <= radius; ++dx)
							for (int dz = -radius; dz <= radius; ++dz)
								carriageway.emplace(x + dx, z + dz);
					}
			}
			std::unordered_map<Xz, std::vector<std::size_t>, XzHash> hall_cover;
			for (std::size_t h = 0; h < result.halls().size(); ++h)
				for (const auto &cell : result.halls()[h].footprint)
					hall_cover[cell].push_back(h);
			const SparseCells tree_clearance =
					surface_track_tree_clearance(elements, platforms, &result);
			std::unordered_set<Xz, XzHash> on_platform;
			for (const auto &platform : platforms)
				for (const auto &cell : platform.cells)
					on_platform.insert(cell);
			std::unordered_set<Xz, XzHash> taken;
			const auto surface_at = [&](int x, int z) {
				const int ground = editor.terrain_level(x, z).value_or(
						editor.get_ground_level(x, z));
				return std::min(
						ground, editor.lowest_carved_bed(x, z, 0).value_or(ground));
			};
			const auto facing_of = [](Xz dir) -> std::int8_t {
				if (dir.first == 0)
					return dir.second < 0 ? 2 : 3;
				return dir.first < 0 ? 4 : 5;
			};
			for (const auto *node : entrances) {
				const Xz entry{node->x, node->z};
				if (!bounds.contains(entry.first, entry.second) ||
						building_footprints.contains(entry.first, entry.second))
					continue;
				std::size_t best_hall = result.halls().size();
				std::int64_t best_distance =
						std::int64_t(ENTRANCE_REACH) * ENTRANCE_REACH + 1;
				for (std::size_t h = 0; h < result.halls().size(); ++h) {
					const auto &hall = result.halls()[h];
					if (entry.first < hall.bbox[0] - ENTRANCE_REACH ||
							entry.first > hall.bbox[2] + ENTRANCE_REACH ||
							entry.second < hall.bbox[1] - ENTRANCE_REACH ||
							entry.second > hall.bbox[3] + ENTRANCE_REACH)
						continue;
					for (const auto &p : hall.platform) {
						const auto dx = std::int64_t(p.first) - entry.first;
						const auto dz = std::int64_t(p.second) - entry.second;
						const auto distance = dx * dx + dz * dz;
						if (distance < best_distance) {
							best_hall = h;
							best_distance = distance;
						}
					}
				}
				if (best_hall == result.halls().size() ||
						best_distance > std::int64_t(ENTRANCE_REACH) * ENTRANCE_REACH)
					continue;
				const auto &hall = result.halls()[best_hall];
				const int floor = hall.floor;
				const int walk = floor + HALL_HEIGHT;
				const int roof = walk + WALK_HEIGHT;
				// Rust memoizes these per entrance route. Candidate stairs and walks
				// probe the same columns many times; keep terrain/floor-state reads
				// bounded by the distinct cells rather than route combinations.
				std::unordered_map<Xz, int, XzHash> surface_cache;
				const auto surface = [&](Xz cell) {
					const auto [it, inserted] = surface_cache.try_emplace(cell, 0);
					if (inserted)
						it->second = surface_at(cell.first, cell.second);
					return it->second;
				};
				const int street = surface(entry);
				const int steps = street - walk - 1;
				if (steps < 1)
					continue;
				std::unordered_map<Xz, bool, XzHash> free_cache;
				const auto free = [&](Xz c) {
					if (const auto found = free_cache.find(c); found != free_cache.end())
						return found->second;
					bool available = bounds.contains(c.first, c.second) &&
									 !taken.contains(c) &&
									 (!tunnel_footprint.contains(c.first, c.second) ||
											 hall.footprint.contains(c));
					if (available) {
						if (const auto found = hall_cover.find(c);
								found != hall_cover.end())
							for (const auto other : found->second)
								if (other != best_hall &&
										result.halls()[other].floor >= floor) {
									available = false;
									break;
								}
					}
					free_cache.emplace(c, available);
					return available;
				};
				const auto frame_x =
						std::abs(hall.heading.first) >= std::abs(hall.heading.second)
								? (hall.heading.first < 0.0 ? -1 : 1)
								: 0;
				const Xz frame_dir = frame_x ? Xz{frame_x, 0}
											 : Xz{0, hall.heading.second < 0.0 ? -1 : 1};
				std::vector<Xz> landings(hall.platform.begin(), hall.platform.end());
				std::sort(landings.begin(), landings.end(), [&](Xz a, Xz b) {
					const auto dist = [&](Xz p) {
						const auto dx = std::int64_t(p.first) - entry.first;
						const auto dz = std::int64_t(p.second) - entry.second;
						return dx * dx + dz * dz;
					};
					return std::tuple{dist(a), a} < std::tuple{dist(b), b};
				});
				std::vector<HallFlight> flights;
				for (const auto &p : landings) {
					if (flights.size() >= LANDING_TRIES)
						break;
					for (const auto dir :
							{frame_dir, Xz{-frame_dir.first, -frame_dir.second}}) {
						auto candidate = fit_hall_flight(hall, p, dir);
						if (candidate &&
								std::all_of(candidate->cells.begin(),
										candidate->cells.end(), free) &&
								std::all_of(candidate->sides.begin(),
										candidate->sides.end(),
										[&](const auto &s) { return free(s.first); })) {
							flights.push_back(std::move(*candidate));
							break;
						}
					}
				}
				if (flights.empty())
					continue;
				struct StreetFlight
				{
					Xz e, q, c0;
				};
				std::vector<StreetFlight> street_flights;
				for (const Xz e : {Xz{1, 0}, Xz{-1, 0}, Xz{0, 1}, Xz{0, -1}})
					for (const int side : {1, -1}) {
						const Xz q{-e.second * side, e.first * side};
						const Xz c0{
								entry.first + 2 * q.first, entry.second + 2 * q.second};
						const auto at = [&](int j, int l) {
							return Xz{c0.first + e.first * j + q.first * l,
									c0.second + e.second * j + q.second * l};
						};
						bool fits = true;
						for (int j = 0; j <= steps + 1 && fits; ++j)
							for (int l = -2; l <= 2; ++l) {
								const Xz c = at(j, l);
								if (!free(c) ||
										building_footprints.contains(c.first, c.second) ||
										editor.is_lc_water(c.first, c.second) ||
										carriageway.contains(c)) {
									fits = false;
									break;
								}
								const int s = surface(c);
								const int step = std::clamp(j, 1, steps);
								const bool open = street - step + 4 > s - 1;
								if (open && tree_clearance.contains(c.first, c.second)) {
									fits = false;
									break;
								}
								if ((open && j <= steps &&
											(s - street < -1 || s - street > 1)) ||
										(!open && j >= 1 && s < street - step + 5)) {
									fits = false;
									break;
								}
							}
						bool entry_clear = fits;
						for (int l = -1; l <= 1 && entry_clear; ++l) {
							const Xz c = at(-1, l);
							entry_clear =
									bounds.contains(c.first, c.second) && free(c) &&
									!building_footprints.contains(c.first, c.second) &&
									!editor.is_lc_water(c.first, c.second) &&
									!fenced.contains(c) && !on_platform.contains(c) &&
									!taken.contains(c);
							if (entry_clear)
								entry_clear = surface(c) <= street;
						}
						if (entry_clear)
							street_flights.push_back({e, q, c0});
					}
				if (street_flights.empty())
					continue;
				struct CandidateRoute
				{
					int estimate;
					std::size_t flight, street;
					bool x_first;
				};
				std::vector<CandidateRoute> routes;
				for (std::size_t fi = 0; fi < flights.size(); ++fi)
					for (std::size_t si = 0; si < street_flights.size(); ++si) {
						const auto &sf = street_flights[si];
						const Xz bottom{sf.c0.first + sf.e.first * (steps + 1),
								sf.c0.second + sf.e.second * (steps + 1)};
						const auto &top = flights[fi].top;
						const int estimate = std::abs(bottom.first - top.first) +
											 std::abs(bottom.second - top.second);
						routes.push_back({estimate, fi, si, true});
						routes.push_back({estimate, fi, si, false});
					}
				std::sort(routes.begin(), routes.end(), [](const auto &a, const auto &b) {
					return std::tie(a.estimate, a.flight, a.street, a.x_first) <
						   std::tie(b.estimate, b.flight, b.street, b.x_first);
				});
				for (const auto &route : routes) {
					const auto &flight = flights[route.flight];
					const auto &sf = street_flights[route.street];
					const auto at = [&](int j, int l) {
						return Xz{sf.c0.first + sf.e.first * j + sf.q.first * l,
								sf.c0.second + sf.e.second * j + sf.q.second * l};
					};
					std::unordered_set<Xz, XzHash> street_cells;
					for (int j = 1; j <= steps; ++j)
						for (int l = -1; l <= 1; ++l)
							street_cells.insert(at(j, l));
					std::unordered_set<Xz, XzHash> hall_stair_cells(
							flight.cells.begin(), flight.cells.end());
					for (const auto &s : flight.sides)
						hall_stair_cells.insert(s.first);
					hall_stair_cells.insert(flight.top);
					bool overlap = false;
					for (int j = 0; j <= steps + 1 && !overlap; ++j)
						for (int l = -2; l <= 2; ++l)
							if (hall_stair_cells.contains(at(j, l))) {
								overlap = true;
								break;
							}
					if (overlap)
						continue;
					const Xz bottom = at(steps + 1, 0);
					auto path = station_l_path(bottom, flight.top, route.x_first);
					bool path_clear = true;
					for (std::size_t i = 1; i < path.size(); ++i)
						if (street_cells.contains(path[i]))
							path_clear = false;
					for (std::size_t i = 0; i + 1 < path.size(); ++i)
						if (std::find(flight.cells.begin(), flight.cells.end(),
									path[i]) != flight.cells.end())
							path_clear = false;
					if (!path_clear)
						continue;
					for (int dx = -2; dx <= 2 && path_clear; ++dx)
						for (int dz = -2; dz <= 2 && path_clear; ++dz)
							for (const auto &p : path) {
								const Xz c{p.first + dx, p.second + dz};
								if (!free(c) || surface(c) < roof + 1) {
									path_clear = false;
									break;
								}
							}
					if (!path_clear)
						continue;
					TunnelEntrance entrance;
					const int climb = facing_of({-sf.e.first, -sf.e.second});
					const auto roof_at = [&](int j, Xz c) -> std::optional<int> {
						const int y = street - j;
						if (y + 4 < surface(c))
							return y + 4;
						return std::nullopt;
					};
					const auto push = [&](int x, int z, TunnelEntrancePart part, int y,
											  int base, int top, bool has_top,
											  std::int8_t facing, int lo, int hi,
											  bool light) {
						entrance.cells.push_back({x, z, part, y, base, top, lo, hi,
								facing, has_top, light});
					};
					for (int l = -1; l <= 1; ++l) {
						const Xz c = at(0, l);
						push(c.first, c.second, TunnelEntrancePart::Stair, street, street,
								street, false, climb, 0, 0, false);
					}
					for (int j = 1; j <= steps; ++j) {
						for (int l = -1; l <= 1; ++l) {
							const Xz c = at(j, l);
							const auto r = roof_at(j, c);
							push(c.first, c.second, TunnelEntrancePart::Stair, street - j,
									walk, r.value_or(0), r.has_value(), climb, 0, 0,
									false);
						}
						for (const int l : {-2, 2}) {
							const Xz c = at(j, l);
							const auto r = roof_at(j, c);
							push(c.first, c.second, TunnelEntrancePart::Wall, 0, walk,
									r.value_or(0), r.has_value(), 0, 0, 0, false);
						}
						if (!roof_at(j, at(j, 0)) &&
								(j == steps || roof_at(j + 1, at(j + 1, 0)))) {
							const int next = j + 1;
							const int base = next > steps ? roof : street - next + 4;
							for (int l = -2; l <= 2; ++l) {
								const Xz c = at(next, l);
								push(c.first, c.second, TunnelEntrancePart::Wall, 0, base,
										0, false, 0, 0, 0, false);
							}
						}
					}
					std::unordered_set<Xz, XzHash> hall_cells(
							flight.cells.begin(), flight.cells.end());
					std::unordered_set<Xz, XzHash> inside;
					for (const auto &p : path)
						for (int dx = -1; dx <= 1; ++dx)
							for (int dz = -1; dz <= 1; ++dz) {
								const Xz c{p.first + dx, p.second + dz};
								if (!street_cells.contains(c))
									inside.insert(c);
							}
					std::vector<Xz> inside_ordered(inside.begin(), inside.end());
					std::sort(inside_ordered.begin(), inside_ordered.end());
					for (const auto &c : inside_ordered)
						push(c.first, c.second, TunnelEntrancePart::Walk, walk, 0, 0,
								false, 0, 0, 0,
								std::find(path.begin(), path.end(), c) != path.end() &&
										(std::find(path.begin(), path.end(), c) -
												path.begin()) %
														4 ==
												0);
					std::unordered_set<Xz, XzHash> walls;
					for (const auto &p : path)
						for (int dx = -2; dx <= 2; ++dx)
							for (int dz = -2; dz <= 2; ++dz) {
								const Xz c{p.first + dx, p.second + dz};
								if (!inside.contains(c) && !hall_cells.contains(c) &&
										!street_cells.contains(c))
									walls.insert(c);
							}
					for (const auto &side : flight.sides)
						if (!inside.contains(side.first) &&
								!street_cells.contains(side.first) &&
								!hall_cells.contains(side.first))
							walls.insert(side.first);
					std::vector<Xz> walls_ordered(walls.begin(), walls.end());
					std::sort(walls_ordered.begin(), walls_ordered.end());
					for (const auto &c : walls_ordered) {
						bool beside_street_flight = false;
						for (int j = 1; j <= steps && !beside_street_flight; ++j)
							beside_street_flight = c == at(j, -2) || c == at(j, 2);
						if (beside_street_flight)
							continue;
						push(c.first, c.second, TunnelEntrancePart::Wall, 0, walk, roof,
								true, 0, 0, 0, false);
					}
					// Keep the hall's furniture clear of the stair shaft and the first
					// two cells behind it, matching StationIndex::build_entrance.
					std::unordered_set<Xz, XzHash> hall_clear(
							flight.cells.begin(), flight.cells.end());
					for (const auto &side : flight.sides)
						hall_clear.insert(side.first);
					for (int k = 0; k <= 1; ++k)
						for (const int l : flight.offsets)
							hall_clear.insert({flight.foot.first - flight.dir.first * k +
													   (-flight.dir.second) * l,
									flight.foot.second - flight.dir.second * k +
											flight.dir.first * l});
					for (auto &hall_cell : result.halls()[best_hall].cells)
						if (hall_clear.contains({hall_cell.x, hall_cell.z}))
							hall_cell.prop = TunnelHallProp::None;
					for (const auto &[c, k] : flight.steps)
						push(c.first, c.second, TunnelEntrancePart::HallStair,
								floor + 1 + k, floor + 1, roof - 1, true,
								facing_of(flight.dir), 0, 0, false);
					for (const auto &[c, k] : flight.sides)
						if (k >= 2)
							push(c.first, c.second, TunnelEntrancePart::Glass, 0, 0, 0,
									false, 0, floor + 2, floor + 1 + k, false);
					entrance.style = hall.style;
					entrance.accent = hall.accent;
					entrance.bbox = {std::numeric_limits<int>::max(),
							std::numeric_limits<int>::max(),
							std::numeric_limits<int>::min(),
							std::numeric_limits<int>::min()};
					for (const auto &c : entrance.cells) {
						entrance.bbox[0] = std::min(entrance.bbox[0], c.x);
						entrance.bbox[1] = std::min(entrance.bbox[1], c.z);
						entrance.bbox[2] = std::max(entrance.bbox[2], c.x);
						entrance.bbox[3] = std::max(entrance.bbox[3], c.z);
						taken.emplace(c.x, c.z);
					}
					for (int j = 0; j <= steps + 1; ++j)
						for (int l = -2; l <= 2; ++l) {
							const auto c = at(j, l);
							taken.insert(c);
							result.keep_trees_off(c.first, c.second);
						}
					for (int l = -1; l <= 1; ++l) {
						const auto c = at(-1, l);
						taken.insert(c);
						result.keep_trees_off(c.first, c.second);
					}
					for (const auto &p : path)
						for (int dx = -2; dx <= 2; ++dx)
							for (int dz = -2; dz <= 2; ++dz)
								taken.emplace(p.first + dx, p.second + dz);
					for (const auto &c : flight.cells)
						taken.insert(c);
					for (const auto &s : flight.sides)
						taken.insert(s.first);
					result.add_entrance(std::move(entrance));
					break;
				}
			}
		}
	}
	// Lower a shared tunnel endpoint where the normal profile cannot climb back
	// from the deeper station floor. Surface connections remain untouched.
	for (const auto &[way, cells] : tunnel_ways) {
		const std::size_t n = cells.size();
		if (n < 2)
			continue;
		bool has_station_pin = false;
		for (const auto &[x, z] : cells)
			if (result.get(way, x, z)) {
				has_station_pin = true;
				break;
			}
		if (!has_station_pin)
			continue;
		const auto ends = rail_joins.ends(way);
		const auto straight = railways::straight_cells(cells,
				ends[0].beyond.empty() ? std::nullopt
									   : std::optional<Xz>{ends[0].beyond.front()},
				ends[1].beyond.empty() ? std::nullopt
									   : std::optional<Xz>{ends[1].beyond.front()});
		for (std::size_t e = 0; e < ends.size(); ++e) {
			const auto &end = ends[e];
			if (!end.shared || end.meets_surface)
				continue;
			std::size_t hall_pos = n;
			int hall_floor = 0;
			for (std::size_t pos = 0; pos < n; ++pos) {
				const auto i = e == 0 ? pos : n - 1 - pos;
				if (const auto pin = result.get(way, cells[i].first, cells[i].second)) {
					hall_pos = pos;
					hall_floor = pin->floor;
					break;
				}
			}
			if (hall_pos == n)
				continue;
			int climbs = 0;
			for (std::size_t pos = 1; pos < std::max<std::size_t>(hall_pos, 1); ++pos) {
				const auto i = e == 0 ? pos : n - 1 - pos;
				if (straight[i])
					++climbs;
			}
			const std::size_t endpoint = e == 0 ? 0 : n - 1;
			const auto [x, z] = cells[endpoint];
			const int ground =
					editor.terrain_level(x, z).value_or(editor.get_ground_level(x, z));
			const int surface =
					std::min(ground, editor.lowest_carved_bed(x, z, 2).value_or(ground));
			const int deep = std::max(
					std::min(surface, ground) - railways::RAIL_TUNNEL_FLOOR_DEPTH,
					world_editor::min_y() + 1);
			const int reach = hall_floor + climbs / 2;
			if (reach < deep)
				result.set_joint_floor(x, z, reach);
		}
	}
	return result;
}

void render_underground_halls(WorldEditor &editor, const TunnelPinIndex &index)
{
	static const std::vector<Block> rails{RAIL, RAIL_NORTH_SOUTH, RAIL_EAST_WEST,
			RAIL_ASCENDING_EAST, RAIL_ASCENDING_WEST, RAIL_ASCENDING_NORTH,
			RAIL_ASCENDING_SOUTH, RAIL_NORTH_EAST, RAIL_NORTH_WEST, RAIL_SOUTH_EAST,
			RAIL_SOUTH_WEST};
	static const std::array<std::array<Block, 5>, 3> styles{{
			{{SMOOTH_QUARTZ, WHITE_CONCRETE, SMOOTH_STONE, QUARTZ_BLOCK, GRAY_CONCRETE}},
			{{CALCITE, SMOOTH_STONE, POLISHED_ANDESITE, QUARTZ_BRICKS,
					POLISHED_DEEPSLATE}},
			{{WHITE_TERRACOTTA, LIGHT_GRAY_CONCRETE, SMOOTH_STONE, POLISHED_ANDESITE,
					GRAY_CONCRETE}},
	}};
	constexpr int hall_height = HALL_HEIGHT;
	constexpr int crust = 3;
	struct VoidRange
	{
		int x, z, lo, hi;
		Block wall;
	};
	std::vector<VoidRange> voids;
	const auto [x0, z0, x1, z1] = editor.bounds();
	const auto overlaps = [=](const std::array<int, 4> &box) {
		return box[0] <= x1 && box[2] >= x0 && box[1] <= z1 && box[3] >= z0;
	};
	const auto open_void = [&](int x, int z, int lo, int hi, Block wall) {
		if (lo > hi || !editor.in_bounds(x, z))
			return;
		voids.push_back({x, z, lo, hi, wall});
		for (int y = lo; y <= hi; ++y)
			editor.set_block_absolute(LIGHT, x, y, z, std::nullopt, rails);
	};
	for (const auto &hall : index.halls()) {
		if (!overlaps(hall.bbox))
			continue;
		const auto &style = styles[hall.style % styles.size()];
		const int roof = hall.floor + hall_height;
		std::unordered_set<Xz, XzHash> platform_cells;
		for (const auto &cell : hall.cells)
			if (cell.part == TunnelHallPart::Platform)
				platform_cells.emplace(cell.x, cell.z);
		for (const auto &cell : hall.cells) {
			const int x = cell.x, z = cell.z, floor = hall.floor;
			if (!editor.in_bounds(x, z))
				continue;
			const auto put = [&](Block block, int y) {
				editor.set_block_absolute(block, x, y, z, std::nullopt, rails);
			};
			switch (cell.part) {
			case TunnelHallPart::Wall:
				put(POLISHED_DEEPSLATE, floor);
				put(style[4], floor + 1);
				for (int y = floor + 2; y < roof; ++y)
					put(y == floor + 3 ? hall.accent : style[0], y);
				put(style[1], roof);
				break;
			case TunnelHallPart::Portal:
				put(POLISHED_DEEPSLATE, floor);
				open_void(x, z, floor + 1, floor + 4, style[0]);
				put(style[0], floor + 5);
				put(style[1], roof);
				break;
			case TunnelHallPart::Trackbed:
				put(POLISHED_DEEPSLATE, floor);
				open_void(x, z, floor + 1, roof - 1, style[0]);
				put(cell.light ? SEA_LANTERN : style[1], roof);
				break;
			case TunnelHallPart::Platform:
				put(POLISHED_DEEPSLATE, floor);
				put(cell.deck == TunnelHallDeck::Edge		? POLISHED_ANDESITE
						: cell.deck == TunnelHallDeck::Line ? WHITE_CONCRETE
															: style[2],
						floor + 1);
				open_void(x, z, floor + 2, roof - 1, style[0]);
				put(cell.light ? SEA_LANTERN : style[1], roof);
				break;
			}
		}
		const auto stair_facing = [](std::int8_t facing) -> const char * {
			switch (facing) {
			case 2:
				return "north";
			case 3:
				return "south";
			case 4:
				return "west";
			default:
				return "east";
			}
		};
		for (const auto &cell : hall.cells) {
			if (cell.part != TunnelHallPart::Platform ||
					!editor.in_bounds(cell.x, cell.z))
				continue;
			const int y = hall.floor + 2;
			switch (cell.prop) {
			case TunnelHallProp::Pillar:
				for (int py = y; py < roof; ++py)
					editor.set_block_absolute(
							style[3], cell.x, py, cell.z, std::nullopt, rails);
				break;
			case TunnelHallProp::Bench: {
				BlockWithProperties near{
						SPRUCE_STAIRS, BlockWithProperties::Properties{
											   {"facing", stair_facing(cell.facing_b)},
											   {"half", "bottom"}}};
				BlockWithProperties far{
						SPRUCE_STAIRS, BlockWithProperties::Properties{
											   {"facing", stair_facing(cell.facing_a)},
											   {"half", "bottom"}}};
				const Xz step = cell.facing_a == 2	 ? Xz{0, -1}
								: cell.facing_a == 3 ? Xz{0, 1}
								: cell.facing_a == 4 ? Xz{-1, 0}
													 : Xz{1, 0};
				editor.set_block_with_properties_absolute(
						near, cell.x, y, cell.z, std::nullopt, rails);
				if (platform_cells.contains({cell.x + step.first, cell.z + step.second}))
					editor.set_block_with_properties_absolute(far, cell.x + step.first, y,
							cell.z + step.second, std::nullopt, rails);
				break;
			}
			case TunnelHallProp::Bin:
				editor.set_block_absolute(
						CAULDRON, cell.x, y, cell.z, std::nullopt, rails);
				break;
			case TunnelHallProp::None:
				break;
			}
		}
	}
	const auto stair_facing = [](std::int8_t facing) -> const char * {
		switch (facing) {
		case 2:
			return "north";
		case 3:
			return "south";
		case 4:
			return "west";
		default:
			return "east";
		}
	};
	for (const auto &entrance : index.entrances()) {
		if (!overlaps(entrance.bbox))
			continue;
		const auto &style = styles[entrance.style % styles.size()];
		for (const auto &cell : entrance.cells) {
			const int x = cell.x, z = cell.z;
			if (!editor.in_bounds(x, z))
				continue;
			const auto put = [&](Block block, int y) {
				editor.set_block_absolute(block, x, y, z, std::nullopt, rails);
			};
			switch (cell.part) {
			case TunnelEntrancePart::Stair: {
				for (int y = cell.base; y < cell.y; ++y)
					put(style[0], y);
				BlockWithProperties stair{POLISHED_ANDESITE_STAIRS,
						BlockWithProperties::Properties{
								{"facing", stair_facing(cell.facing)},
								{"half", "bottom"}}};
				editor.set_block_with_properties_absolute(
						stair, x, cell.y, z, std::nullopt, rails);
				if (cell.has_top)
					put(style[1], cell.top);
				break;
			}
			case TunnelEntrancePart::Wall: {
				const int top = cell.has_top ? cell.top : editor.get_ground_level(x, z);
				for (int y = cell.base; y <= top; ++y)
					put(!cell.has_top && y == top ? SMOOTH_STONE
							: y == cell.base + 2  ? entrance.accent
												  : style[0],
							y);
				break;
			}
			case TunnelEntrancePart::Walk:
				put(style[2], cell.y);
				put(cell.light ? SEA_LANTERN : style[1], cell.y + WALK_HEIGHT);
				break;
			case TunnelEntrancePart::HallStair: {
				for (int y = cell.base; y < cell.y; ++y)
					put(style[2], y);
				BlockWithProperties stair{POLISHED_ANDESITE_STAIRS,
						BlockWithProperties::Properties{
								{"facing", stair_facing(cell.facing)},
								{"half", "bottom"}}};
				editor.set_block_with_properties_absolute(
						stair, x, cell.y, z, std::nullopt, rails);
				put(style[1], cell.top + 1);
				break;
			}
			case TunnelEntrancePart::Glass:
				break;
			}
		}
	}
	for (const auto &entrance : index.entrances()) {
		if (!overlaps(entrance.bbox))
			continue;
		const auto &style = styles[entrance.style % styles.size()];
		for (const auto &cell : entrance.cells) {
			const int x = cell.x, z = cell.z;
			if (!editor.in_bounds(x, z))
				continue;
			const auto clear = [&](int lo, int hi) { open_void(x, z, lo, hi, style[0]); };
			switch (cell.part) {
			case TunnelEntrancePart::Stair:
				if (cell.has_top)
					clear(cell.y + 1, cell.top - 1);
				else {
					clear(cell.y + 1, cell.top + 4);
				}
				break;
			case TunnelEntrancePart::Wall:
				if (!cell.has_top) {
					const int surface = editor.get_ground_level(x, z);
					clear(surface + 1, surface + 1);
					editor.set_block_absolute(
							IRON_BARS, x, surface + 1, z, std::nullopt, rails);
				}
				break;
			case TunnelEntrancePart::Walk:
				clear(cell.y + 1, cell.y + WALK_HEIGHT - 1);
				break;
			case TunnelEntrancePart::HallStair:
				clear(cell.y + 1, cell.top);
				break;
			case TunnelEntrancePart::Glass:
				clear(cell.lo, cell.hi);
				for (int y = cell.lo; y <= cell.hi; ++y)
					connected_blocks::place_connected(editor, GLASS_PANE, x, y, z);
				break;
			}
		}
	}
	// Rust seals the outside edge and caps of each open station volume up to
	// three blocks below the local surface. Keep Rust's LIGHT sentinel until after
	// sealing so later props and glass panes replace it and survive carving.
	constexpr std::array<Xz, 4> neighbours{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
	for (const auto &v : voids) {
		for (const auto &[dx, dz] : neighbours) {
			const int nx = v.x + dx, nz = v.z + dz;
			if (!editor.in_bounds(nx, nz))
				continue;
			const int top = std::min(v.hi, editor.get_ground_level(nx, nz) - crust);
			for (int y = v.lo; y <= top; ++y)
				editor.set_block_absolute(v.wall, nx, y, nz);
		}
		const int top = editor.get_ground_level(v.x, v.z) - crust;
		for (const int y : {v.lo - 1, v.hi + 1})
			if (y <= top)
				editor.set_block_absolute(v.wall, v.x, y, v.z);
	}
	for (const auto &v : voids)
		for (int y = v.lo; y <= v.hi; ++y)
			if (editor.get_block_absolute(v.x, y, v.z) == LIGHT)
				editor.set_block_absolute(AIR, v.x, y, v.z, std::nullopt, rails);
}

} // namespace arnis::stations
