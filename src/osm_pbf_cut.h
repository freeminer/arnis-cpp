#pragma once

#include "coordinate_system/geographic/llbbox.h"
#include "osm_parser.h"
#include <filesystem>
#include <string>

namespace arnis::osm_pbf
{
// Stream an extract in bounded-memory passes and return only the tagged OSM
// features relevant to the requested area, their member ways, and referenced
// nodes. The bbox is expanded by pad_m before clipping, matching Rust's bake.
osm_parser::RawOsmDocument cut_extract(
		const std::filesystem::path &path, const geographic::LLBBox &bbox, double pad_m);
}
