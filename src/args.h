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

	// Output directory for the generated world (required for Java, optional for Bedrock).
	// Use --output-dir (or the deprecated --path alias) to specify where the world is created.
	std::optional<std::string> path{std::nullopt};

	// Generate a Bedrock Edition world (.mcworld) instead of Java Edition
	bool bedrock{false};

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

	// Enable city boundary ground generation (optional)
	// When enabled, detects building clusters and places stone ground in urban areas.
	// Isolated buildings in rural areas will keep grass around them.
	bool city_boundaries{true};

	// Enable debug mode (optional)
	bool debug{false};

	// Set floodfill timeout (seconds) (optional)
	std::optional<std::chrono::milliseconds> timeout{std::nullopt};
	std::chrono::milliseconds timeout_ref() const { 
		return timeout.value_or(std::chrono::milliseconds(3000)); 
	}
};

}
