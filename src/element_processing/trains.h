#pragma once

#include "../floodfill_cache.h"
#include "../bedrock_addon.h"
#include "../../../arnis_adapter.h"
#include "train_bodies.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace arnis::cartesian
{
class XZBBox;
}

namespace arnis::trains
{

enum class Mode : std::uint8_t
{
	Tram,
	Metro,
	SBahn,
	Rail,
	Water
};

enum class End : std::uint8_t
{
	Edge,
	Buffer,
	Junction
};

enum class Shape : std::uint8_t
{
	Loop,
	Through,
	Shuttle
};

using TrainKind = train_bodies::Kind;
using CoachKind = train_bodies::CoachKind;
using Coach = train_bodies::Coach;
using Livery = train_bodies::Livery;

struct TrainDeparture
{
	std::size_t phase{};
	Livery livery;
};

struct Service
{
	std::size_t route{};
	std::vector<Coach> coaches;
	/// Distance behind the front carriage centre, in blocks.
	std::vector<double> offsets;
	/// Front carriage centre at each 0.1-second step, in thousandths of a block.
	std::vector<std::int32_t> timetable;
	/// Lap length in thousandths of a block; zero for a shuttle.
	std::int64_t period{};
	std::vector<TrainDeparture> trains;
};

/// One point along a route, in world block coordinates.
struct RoutePoint
{
	double x{}, z{};
};

/// Backend-neutral railway service path. Stops are point indices along the route.
struct Route
{
	Mode mode{Mode::Rail};
	/// Present for water routes, corresponding to the Rust `Mode::Water(Vessel)` payload.
	std::optional<ship_bodies::Vessel> water_vessel;
	Shape shape{Shape::Shuttle};
	End ends[2]{End::Buffer, End::Buffer};
	std::vector<RoutePoint> points;
	std::vector<std::uint64_t> ways;
	std::vector<double> speed_caps;
	std::vector<double> stops;
	/// Optional surface heights for waterborne routes, read from the completed world.
	/// Rail routes leave this empty and resolve their track from registered rail nodes.
	std::vector<double> surface_y;
	double surface_lift{0.0625};
	double length() const;
};

/// Build the single-vessel timetable used by a mapped ferry crossing.
std::optional<Service> ship_service(const Route &route, ship_bodies::Vessel vessel,
		bool low, std::uint64_t seed, double scale, double size,
		std::optional<double> headway_seconds = std::nullopt);

/// Topology, mapped stops and backend-neutral service schedules for railway,
/// metro, tram and light-rail ways. Rendering belongs to the consumer.
class TrainRouteNetwork
{
public:
	static TrainRouteNetwork build(const std::vector<ProcessedElement> &elements,
			double scale, const cartesian::XZBBox &world);
	const std::vector<Route> &routes() const { return routes_; }
	const std::vector<Service> &services() const { return services_; }
	double size() const { return size_; }
	std::size_t train_count() const;
	std::size_t ship_count() const;
	bool is_empty() const { return services_.empty(); }
	/// Append water services planned from the finished-world water probe. Each item owns one
	/// route and its backend-neutral schedule, just like a rail service.
	void add_ships(std::vector<std::pair<Route, Service>> ships);
	std::vector<std::pair<std::string, std::string>> datapack_functions(
			const std::function<std::optional<int>(std::uint64_t, int, int)> &rail_y)
			const;
	std::pair<nlohmann::json, std::vector<bedrock_addon::Body>> bedrock_data(
			const std::function<std::optional<int>(std::uint64_t, int, int)> &rail_y)
			const;

private:
	std::vector<Route> routes_;
	std::vector<Service> services_;
	double size_{1.0};
	std::size_t ship_services_{};
};

// Keep the Rust module's public type name available to library consumers;
// TrainRouteNetwork remains as the descriptive C++ name used by existing
// backend integrations.
using TrainNetwork = TrainRouteNetwork;

} // namespace arnis::trains
