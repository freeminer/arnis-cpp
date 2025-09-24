#pragma once

#include <chrono>
#include <optional>
#include <string>

namespace arnis
{

struct Args
{
	// Bounding box of the area (min_lat,min_lng,max_lat,max_lng) (required)
	//LLBBox bbox{};

	// JSON file containing OSM data (optional)
	std::optional<std::string> file{std::nullopt};

	// JSON file to save OSM data to (optional)
	std::optional<std::string> save_json_file{std::nullopt};

	// Path to the Minecraft world (required)
	std::string path{};

	// Downloader method (requests/curl/wget) (optional)
	std::string downloader{std::string("requests")};

	// World scale to use, in blocks per meter
	double scale{1.0};

	// Ground level to use in the Minecraft world
	int ground_level{-62};

	// Enable terrain (optional)
	bool terrain{false};

	// Enable interior generation (optional)
	bool interior{true};

	// Enable roof generation (optional)
	bool roof{true};

	// Enable filling ground (optional)
	bool fillground{false};

	// Enable debug mode (optional)
	bool debug{false};

	// Set floodfill timeout (seconds) (optional)
	//std::optional<std::chrono::duration<double>> timeout{std::nullopt};
	const std::chrono::milliseconds timeout{200};
	std::chrono::milliseconds timeout_ref() const { return timeout; }

	// Spawn point coordinates (lat, lng)
	std::optional<std::pair<double, double>> spawn_point{std::nullopt};
};

}