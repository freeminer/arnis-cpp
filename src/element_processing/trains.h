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
	Rail
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
	Shape shape{Shape::Shuttle};
	End ends[2]{End::Buffer, End::Buffer};
	std::vector<RoutePoint> points;
	std::vector<std::uint64_t> ways;
	std::vector<double> speed_caps;
	std::vector<double> stops;
	double length() const;
};

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
	bool is_empty() const { return services_.empty(); }
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
};

} // namespace arnis::trains
