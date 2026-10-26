#pragma once

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace arnis::world_editor
{
struct WorldEditor;
}

namespace arnis::highways::cars
{

struct Waypoint
{
	double x{}, y{}, z{};
	std::optional<std::uint8_t> stop;
	std::optional<std::int32_t> cap;
};

struct Turn
{
	std::size_t lane{}, entry{};
	bool crossing{};
};

struct Segment
{
	std::uint64_t way{};
	std::vector<Waypoint> points;
	std::vector<double> arcs;
	std::vector<Turn> next;
	bool into_junction{};
	bool turns_back{};
	// A short dead-end lane is retained in the network for connectivity but does
	// not carry cars when another way out of the junction exists.
	bool stub{};
	std::int32_t speed{};
	double density{};
};

enum class VehicleKind : std::uint8_t
{
	Sedan,
	Hatchback,
	Van
};

struct Car
{
	std::size_t segment{}, target{};
	std::array<double, 3> position{};
	VehicleKind kind{VehicleKind::Sedan};
	std::string colour;
	std::int32_t pace{1000};
};

/// Deterministic lane geometry and traffic samples for one generated area.
/// Backend writers may consume these records without relying on an exporter.
class CarNetwork
{
public:
	static CarNetwork build(const std::vector<ProcessedElement> &elements, double scale,
			bool drives_on_left, bool stop_at_signals, const ::XZBBox &world,
			const std::function<int(int, int)> &surface_y);
	void place_way(world_editor::WorldEditor &editor, std::uint64_t way) const;

	std::size_t car_count() const { return cars_.size(); }
	const std::vector<Segment> &segments() const { return segments_; }
	const std::vector<Car> &cars() const { return cars_; }
	const std::vector<std::size_t> &cars_for_way(std::uint64_t way) const;
	double size() const { return size_; }
	double scale() const { return scale_; }
	// Rust exposes these backend-neutral datapack payloads separately; retain
	// those entry points for library consumers while keeping the combined writer.
	std::string storage_function() const;
	std::vector<std::pair<std::string, std::string>> look_functions() const;
	std::vector<std::pair<std::string, std::string>> body_functions() const;
	std::vector<std::pair<std::string, std::string>> datapack_functions() const;

private:
	std::vector<Segment> segments_;
	std::vector<Car> cars_;
	std::unordered_map<std::uint64_t, std::vector<std::size_t>> by_way_;
	double size_{1.0};
	double scale_{1.0};
};

} // namespace arnis::highways::cars
