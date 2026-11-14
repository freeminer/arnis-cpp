#pragma once

#include "coordinate_system/geographic/llbbox.h"
#include "osm_parser.h"
#include <filesystem>

namespace arnis::osm_pbf
{
// Return a cached area cut when available, otherwise stream/cut the extract and
// atomically store the compressed result. Cache identity includes extract
// size and mtime so a refreshed -latest file invalidates older area bakes.
osm_parser::RawOsmDocument cached_cut_extract(
		const std::filesystem::path &pbf, const geographic::LLBBox &bbox, double pad_m);
}
