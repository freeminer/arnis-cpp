#pragma once

#include "crs.h"
#include "../coordinate_system/geographic/llbbox.h"
#include "../coordinate_system/geographic/llpoint.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

namespace arnis::survey::source
{
using Rect = std::array<double, 4>; // west, south, east, north in source metres
using Tile = std::pair<std::int32_t, std::int32_t>;
// A tile listed more than once may consist of multiple COPC/EPT files.
using TileUrls = std::vector<std::pair<Tile, std::string>>;

struct Sheet;

struct LaserClasses
{
	std::vector<std::uint8_t> ground;
	std::vector<std::uint8_t> building;
	std::vector<std::uint8_t> object;
	std::vector<std::uint8_t> vegetation;
	std::vector<std::uint8_t> footprint;
	std::vector<std::uint8_t> unsorted;
};

struct Locate
{
	enum class Kind
	{
		Urls,
		Lookup
	};
	using UrlsFn = std::function<std::vector<std::string>(std::int32_t, std::int32_t)>;
	using LookupFn = std::function<std::variant<TileUrls, std::string>(
			const Sheet &, const Rect &)>;

	Kind kind = Kind::Urls;
	UrlsFn urls;
	LookupFn lookup;
};

struct Laser
{
	enum class Kind
	{
		Whole,
		Copc,
		Ept
	};
	Kind kind = Kind::Whole;
	Locate locate;
	std::uint32_t tile_mb = 0;
	std::uint32_t mb_per_km2 = 0;
	bool own_grid = false;
};

struct Photo
{
	enum class Kind
	{
		TiledTiff,
		Wms,
		None,
		Sheets
	};
	using TileUrlFn = std::function<std::string(std::int32_t, std::int32_t)>;
	using SheetAtFn =
			std::function<std::optional<std::tuple<std::string, double, double>>(
					double, double)>;

	Kind kind = Kind::None;
	TileUrlFn tile_url;
	std::string url;
	std::string layer;
	SheetAtFn sheet_at;
	std::size_t pixels_per_metre = 0;
};

struct Grid
{
	enum class Kind
	{
		Fixed,
		Nad83Utm
	};
	Kind kind = Kind::Fixed;
	crs::Crs fixed;
};

// Geographic coverage plus the provider contract. Rings use (latitude,
// longitude) coordinates, matching the Rust source manifests.
struct Source
{
	std::string id;
	std::string name;
	Grid grid;
	std::vector<std::vector<geographic::LLPoint>> rings;
	Laser laser;
	Photo photo;
	std::pair<float, float> intensity_defaults{};
	LaserClasses classes;
	bool laser_plants = false;
	bool strict_pools = false;
	bool paving_brighter = false;
	std::string attribution;

	bool contains(double latitude, double longitude) const;
	double depth(double latitude, double longitude) const;
	bool covers(const geographic::LLBBox &bbox) const;
};

using Coverage = Source;

const Source &bavaria();
const Source &france();
const Source &netherlands();
const Source &switzerland();
const Source &nrw();
const Source &latvia();
const Source &estonia();
const Source &luxembourg();
const Source &usa();
const Source &canada();

// Select the source containing the bbox centre; on overlapping coverage choose
// the one whose centre is deeper inside. If none contains it, choose the first
// source reached by the Rust-compatible 250 m sampling lattice.
const Source *for_bbox(
		const geographic::LLBBox &bbox, const std::vector<Source> &sources);
const std::vector<Source> &sources();
const Source *for_bbox(const geographic::LLBBox &bbox);

struct Sheet
{
	const Source *source = nullptr;
	crs::Crs projection;
	bool zone_specific = false;

	static Sheet at(const Source &source, double longitude);
	std::string cache_dir() const;
	std::vector<std::pair<std::int32_t, std::int32_t>> tiles(const Rect &rect) const;
	Rect rect(const geographic::LLBBox &bbox, double margin_m) const;
};
} // namespace arnis::survey::source
