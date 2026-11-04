#pragma once

#include "../../../arnis_adapter.h"
#include "bridges.h"

#include <cstddef>
#include <chrono>
#include <cstdint>
#include <array>
#include <optional>
#include <string>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arnis::railways
{
class RailJoins;
}

namespace arnis::stations
{

inline constexpr int TRACK_REACH = 3;
inline constexpr int TUNNEL_TRACK_REACH = 4;
inline constexpr int HALL_DEPTH = 11;
inline constexpr int HALL_HEIGHT = 6;
inline constexpr int HALL_MIN_DEPTH = HALL_HEIGHT + 2;
inline constexpr int HALL_STACK = 8;
inline constexpr int WALK_HEIGHT = 4;
inline constexpr std::size_t MAX_PLATFORM_CELLS = 40'000;
inline constexpr int TRACK_TREE_REACH = 2;
inline constexpr int ENTRANCE_REACH = 160;
inline constexpr std::size_t LANDING_TRIES = 48;
inline constexpr int HALL_FLIGHT = HALL_HEIGHT - 1;
inline constexpr int SIBLING_REACH = 24;
inline constexpr std::size_t HEADING_CELLS = 4;
inline constexpr int STACK_REACH = 48;
inline constexpr double PARALLEL_DEG = 25.0;

using Xz = std::pair<int, int>;
using StationTimeout = std::optional<std::chrono::milliseconds>;

struct HallTrack
{
	std::uint64_t way{};
	int x{}, z{};
	std::pair<double, double> dir{};
};

struct XzHash
{
	std::size_t operator()(const Xz &point) const noexcept;
};

// Sparse 16x16-cell bitset used for station clearances and excavation masks.
// Keeping only touched sections avoids allocating an area-sized raster for
// large railway corridors.
class SparseCells
{
public:
	void set(int x, int z);
	void set_square(int x, int z, int radius);
	void merge(const SparseCells &other);
	bool contains(int x, int z) const;
	bool empty() const { return blocks_.empty(); }
	std::vector<Xz> iter() const;

private:
	struct Hash
	{
		std::size_t operator()(const Xz &point) const noexcept;
	};
	std::unordered_map<Xz, std::array<std::uint64_t, 4>, Hash> blocks_;
};

struct TunnelPin
{
	int floor{};
	bool in_hall{};
};

enum class TunnelHallPart : std::uint8_t
{
	Trackbed,
	Platform,
	Wall,
	Portal
};

enum class TunnelHallDeck : std::uint8_t
{
	Inner,
	Edge,
	Line
};

enum class TunnelHallProp : std::uint8_t
{
	None,
	Pillar,
	Bench,
	Bin
};

struct TunnelHallCell
{
	int x{}, z{};
	TunnelHallPart part{TunnelHallPart::Trackbed};
	TunnelHallDeck deck{TunnelHallDeck::Inner};
	TunnelHallProp prop{TunnelHallProp::None};
	std::int8_t facing_a{}, facing_b{};
	bool light{};
};

struct TunnelHall
{
	std::array<int, 4> bbox{};
	std::vector<TunnelHallCell> cells;
	std::unordered_set<Xz, XzHash> footprint;
	std::unordered_set<Xz, XzHash> interior;
	std::unordered_set<Xz, XzHash> level_tracks;
	std::unordered_set<std::uint64_t> ways;
	std::unordered_set<std::uint64_t> line_ways;
	std::vector<HallTrack> through_tracks;
	std::unordered_set<Xz, XzHash> platform;
	std::unordered_set<Xz, XzHash> platform_edge;
	int floor{};
	int depth{-1};
	bool active{true};
	std::size_t style{};
	Block accent;
	std::pair<double, double> heading{};
	// Keep the layout frame used to place platform furniture and lights. Foreign
	// tracks can change the final platform/trackbed split after halls are joined.
	std::array<double, 4> layout_frame{}; // cx, cz, ux, uz
};

enum class TunnelEntrancePart : std::uint8_t
{
	Stair,
	Wall,
	Walk,
	HallStair,
	Glass
};

struct TunnelEntranceCell
{
	int x{}, z{};
	TunnelEntrancePart part{TunnelEntrancePart::Walk};
	int y{}, base{}, top{}, lo{}, hi{};
	std::int8_t facing{};
	bool has_top{}, light{};
};

struct TunnelEntrance
{
	std::array<int, 4> bbox{};
	std::vector<TunnelEntranceCell> cells;
	std::size_t style{};
	Block accent;
};

class TrackBedIndex
{
public:
	struct Key
	{
		std::uint64_t way{};
		int x{}, z{};
		bool operator==(const Key &) const = default;
	};
	struct Hash
	{
		std::size_t operator()(const Key &key) const noexcept;
	};
	void set(std::uint64_t way, int x, int z, int bed);
	std::optional<int> get(std::uint64_t way, int x, int z) const;
	bool empty() const { return beds_.empty(); }

private:
	std::unordered_map<Key, int, Hash> beds_;
};

class TunnelPinIndex
{
public:
	void set(std::uint64_t way, int x, int z, TunnelPin pin);
	std::optional<TunnelPin> get(std::uint64_t way, int x, int z) const;
	void set_joint_floor(int x, int z, int floor);
	std::optional<int> joint_floor(int x, int z) const;
	void add_hall(TunnelHall hall)
	{
		for (const auto &[x, z] : hall.footprint)
			dug_.set(x, z);
		halls_.push_back(std::move(hall));
	}
	void discard_inactive_halls();
	std::vector<TunnelHall> &halls() { return halls_; }
	const std::vector<TunnelHall> &halls() const { return halls_; }
	void add_entrance(TunnelEntrance entrance)
	{
		for (const auto &cell : entrance.cells)
			dug_.set(cell.x, cell.z);
		entrances_.push_back(std::move(entrance));
	}
	const std::vector<TunnelEntrance> &entrances() const { return entrances_; }
	void keep_trees_off(int x, int z) { entrance_tree_clearance_.set(x, z); }
	const SparseCells &entrance_tree_clearance() const
	{
		return entrance_tree_clearance_;
	}
	bool digs(int x, int z) const { return dug_.contains(x, z); }
	void set_platform_cell(int x, int z) { platform_cells_.set(x, z); }
	bool is_platform(int x, int z) const { return platform_cells_.contains(x, z); }
	bool empty() const
	{
		return pins_.empty() && joint_floors_.empty() && halls_.empty() &&
			   entrances_.empty() && dug_.empty() && platform_cells_.empty();
	}

private:
	std::unordered_map<TrackBedIndex::Key, TunnelPin, TrackBedIndex::Hash> pins_;
	std::unordered_map<std::uint64_t, int> joint_floors_;
	std::vector<TunnelHall> halls_;
	std::vector<TunnelEntrance> entrances_;
	SparseCells entrance_tree_clearance_;
	SparseCells dug_;
	SparseCells platform_cells_;
};

struct Platform
{
	std::uint64_t id{};
	std::vector<Xz> cells;
	int min_x{};
	int min_z{};
	int max_x{};
	int max_z{};
	bool low{};
	bool covered{};
	bool shelter{};
	std::optional<std::string> name;
	std::optional<Block> colour;
	std::optional<int> depth;
	bool underground{};
	bool surface{};
};

bool is_rail_platform(const tags_t &tags);
std::optional<int> depth_key(const tags_t &tags);
bool tagged_surface(const tags_t &tags);
bool tagged_underground(const tags_t &tags);
bool is_low(const tags_t &tags);
std::vector<Xz> ring_cells(
		const std::vector<Xz> &ring, const StationTimeout &timeout = std::nullopt);
std::vector<Xz> platform_cells(
		const ProcessedElement &element, const StationTimeout &timeout = std::nullopt);
std::vector<Platform> collect_platforms(const std::vector<ProcessedElement> &elements,
		const StationTimeout &timeout = std::nullopt);
// Render platform walking surfaces after the tile ground pass.
void render_surface_platforms(WorldEditor &editor, const std::vector<Platform> &platforms,
		const std::vector<ProcessedElement> &elements,
		const bridges::BridgeStructureMap &bridge_structures,
		const CoordinateBitmap &road_mask, const CoordinateBitmap &building_footprints,
		int min_x, int min_z, int max_x, int max_z);
TrackBedIndex register_surface_platform_levels(WorldEditor &editor,
		const std::vector<Platform> &platforms,
		const std::vector<ProcessedElement> &elements,
		const bridges::BridgeStructureMap &bridge_structures, int min_x, int min_z,
		int max_x, int max_z, bool register_levels = true,
		std::vector<Platform> *surface_groups = nullptr);
// Rust's StationIndex reserves surface railway corridors for the track itself
// before trees are considered, even when no platform is close by.
SparseCells surface_track_tree_clearance(const std::vector<ProcessedElement> &elements);
SparseCells surface_track_tree_clearance(const std::vector<ProcessedElement> &elements,
		const std::vector<Platform> &platforms,
		const TunnelPinIndex *station_index = nullptr);
TrackBedIndex surface_track_beds(const std::vector<ProcessedElement> &elements,
		const std::vector<Platform> &platforms, const WorldEditor &editor,
		const bridges::BridgeStructureMap &bridge_structures);
TunnelPinIndex underground_track_pins(const std::vector<ProcessedElement> &elements,
		const std::vector<Platform> &platforms, const WorldEditor &editor,
		const railways::RailJoins &rail_joins, const XZBBox &bounds,
		const CoordinateBitmap &tunnel_footprint,
		const CoordinateBitmap &building_footprints, double scale,
		bool allow_underground = true);
struct StationVoid
{
	int x, z, lo, hi;
};
// Render the hall/entrance shell and reserve open volumes before terrain/cave
// passes. Clear the returned reserved cells after those passes finish.
std::vector<StationVoid> render_underground_halls(
		WorldEditor &editor, const TunnelPinIndex &index);
void clear_underground_voids(WorldEditor &editor, const std::vector<StationVoid> &voids);

} // namespace arnis::stations
