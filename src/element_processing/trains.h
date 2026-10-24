#pragma once

#include "../floodfill_cache.h"
#include "../../../arnis_adapter.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

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

enum class TrainKind : std::uint8_t
{
	Tram,
	Metro,
	SBahn,
	Regional,
	DoubleDeck,
	Intercity,
	Freight
};

enum class CoachKind : std::uint8_t
{
	TramCab,
	TramMid,
	TramShort,
	MetroCab,
	MetroMid,
	SBahnCab,
	SBahnMid,
	RegioCab,
	RegioMid,
	DeckCab,
	DeckMid,
	Loco,
	IceCab,
	IceMid,
	Container,
	Tank,
	Boxcar
};

struct Coach
{
	CoachKind kind{CoachKind::Loco};
	bool front{};
	std::uint8_t variant{};
};

struct Livery
{
	std::string body;
	std::string lower;
	std::string stripe;
	std::string door;
	std::string roof;
	std::string name;
	std::uint8_t seed{};
};

/// Compose the carriage consist and per-departure livery for a service.
/// departure_index selects a stable livery variant for repeated departures.
std::pair<std::vector<Coach>, Livery> compose(TrainKind kind, std::uint64_t seed,
		bool short_train, std::size_t departure_index = 0);

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
			double scale, const ::XZBBox &world);
	const std::vector<Route> &routes() const { return routes_; }
	const std::vector<Service> &services() const { return services_; }
	double size() const { return size_; }
	std::size_t train_count() const;
	bool is_empty() const { return services_.empty(); }

private:
	std::vector<Route> routes_;
	std::vector<Service> services_;
	double size_{1.0};
};

} // namespace arnis::trains

// The backend-neutral carriage geometry API is available with the route model.
#include "fm_train_bodies.h"

namespace arnis::train_bodies
{
/// Compose a consist and deterministic livery for a departure. Rust returns a
/// livery-producing closure; the C++ library takes its departure index here.
std::pair<std::vector<trains::Coach>, trains::Livery> compose(
		Kind kind, std::uint64_t seed, bool short_train, std::size_t departure_index = 0);
} // namespace arnis::train_bodies
