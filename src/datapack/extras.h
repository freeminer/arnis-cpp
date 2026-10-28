#pragma once

#include <filesystem>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "discoveries.h"

namespace arnis
{
struct Args;
enum class WorldFormat;
namespace highways::cars
{
class CarNetwork;
}
namespace trains
{
class TrainRouteNetwork;
}

namespace datapack
{
// Format-filtered gameplay features used while preparing generation state.
// Bedrock currently consumes only moving vehicles; Luanti has no gameplay
// extras. Keep this policy centralized like Rust's Extras.
struct Extras
{
	bool gravity{};
	bool traffic_lights{};
	bool discoveries{};
	bool pause_menu{};
	bool outbreak{};
	bool disasters{};
	bool cars{};
	bool trains{};

	static Extras from_args(const Args &args);
	Extras for_format(WorldFormat format) const;
	bool any() const;
	bool wants_places() const;
};

namespace disasters
{
class Surface;
}

struct SpawnDestination
{
	int x = 0;
	int z = 0;
	std::optional<int> ground_y;
};

struct Destination
{
	std::string name;
	std::string label;
	int x = 0;
	int z = 0;
	std::optional<int> ground_y;
};

std::vector<Destination> destinations_from(const std::vector<discoveries::Place> &places,
		const std::optional<SpawnDestination> &spawn);

// Install the Java-only runtime pieces which accompany generated world content.
// Returns false only when an enabled feature could not be installed.
bool install_runtime_extras(const std::filesystem::path &world, const Args &args,
		const std::string &area, bool red_amber,
		const highways::cars::CarNetwork *cars = nullptr,
		const trains::TrainRouteNetwork *trains = nullptr,
		const std::function<std::optional<int>(std::uint64_t, int, int)> &rail_y = {},
		const std::vector<discoveries::Place> &places = {},
		const std::optional<SpawnDestination> &spawn = std::nullopt,
		std::string *error = nullptr, bool wooded = false,
		const std::vector<std::pair<std::string, std::uint64_t>> &biomes = {},
		std::optional<int> highest_surface = std::nullopt,
		const disasters::Surface *surface = nullptr,
		std::optional<int> sea_y = std::nullopt);
} // namespace datapack
} // namespace arnis
