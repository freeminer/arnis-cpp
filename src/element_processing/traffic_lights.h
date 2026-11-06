#pragma once

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

namespace arnis::world_editor
{
struct WorldEditor;
}

namespace arnis::highways
{

inline constexpr double UPSTREAM_M = 12.0;

char group_of(double direction_x, double direction_z);
bool signal_upstream(const std::vector<ProcessedNode> &nodes, std::size_t index, int step,
		double reach);

namespace cars
{
bool drivable(const tags_t &tags);
}

/// Drivable-road junctions and their approach signals, shared by physical
/// poles and the moving-car network.
class Junctions
{
public:
	static Junctions build(const std::vector<ProcessedElement> &elements, double scale);
	std::uint32_t arms(std::uint64_t id) const;
	bool is_junction(std::uint64_t id) const { return arms(id) >= 3; }
	int widest(std::uint64_t id) const;
	bool passed(
			const std::vector<ProcessedNode> &nodes, std::size_t index, int step) const;

private:
	double reach(std::uint64_t id) const;
	std::optional<std::pair<std::uint64_t, double>> nearest(
			const std::vector<ProcessedNode> &nodes, std::size_t index, int step) const;
	std::unordered_map<std::uint64_t, std::uint32_t> arms_;
	std::unordered_map<std::uint64_t, int> widest_;
	std::unordered_map<std::uint64_t, std::vector<std::uint64_t>> signals_;
	double upstream_ = UPSTREAM_M;
};

/// Area-wide traffic-signal geometry. Planning is separate from rendering so
/// backends can consume the same poles, approaches, and signal groups.
class SignalIndex
{
public:
	enum class Kind : std::uint8_t
	{
		Car,
		Bike,
		Pedestrian
	};

	static std::shared_ptr<const SignalIndex> build(
			const std::vector<ProcessedElement> &elements, double scale,
			bool drives_on_left, const RoadMaskBitmap &road_mask);

	bool place_node(world_editor::WorldEditor &editor, const ProcessedNode &node,
			const std::function<int(int, int)> &base_y) const;

private:
	struct Head
	{
		double facing_x{}, facing_z{};
		char group{'a'};
		Kind kind{Kind::Car};
	};
	struct Pole
	{
		int x{}, z{};
		int top{4};
		std::vector<Head> heads;
		bool has_mast{};
		int mast_x{}, mast_z{};
		Head mast_head{};
	};
	std::vector<Pole> poles_;
	std::unordered_map<std::uint64_t, std::vector<std::size_t>> by_node_;
};

bool is_vehicular(const std::string &highway);
bool is_signal(const tags_t &tags);

} // namespace arnis::highways
