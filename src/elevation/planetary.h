#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "../celestial.h"
#include "elevation.h"
#include "../coordinate_system/geographic/llbbox.h"

namespace arnis::elevation
{

using PlanetaryRangeReader = std::function<std::optional<std::vector<std::uint8_t>>(
		const std::string &url, std::uint64_t offset, std::uint64_t length)>;

// NASA PDS MOLA/LOLA terrain, in metres after the body's terrain gain.  The
// reader owns transport policy: callers may bind HTTP, a disk archive, or a
// test fixture without coupling celestial map generation to an exporter.
std::optional<ElevationData> fetch_planetary_elevation(CelestialBody,
		const geographic::LLBBox &, std::size_t width, std::size_t height,
		const PlanetaryRangeReader &, const std::filesystem::path &cache_directory = {});

double planetary_native_resolution_m(CelestialBody);

// Host adapter for the project HTTP layer. It accepts only exact 206 range
// replies, so a PDS server that ignores Range cannot trigger a full-raster
// download.
PlanetaryRangeReader http_planetary_range_reader();

} // namespace arnis::elevation
