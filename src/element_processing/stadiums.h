#pragma once

#include "../../../arnis_types.h"
#include "../args.h"
#include "../block_definitions.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace arnis::world_editor
{
struct WorldEditor;
}

namespace arnis::element_processing::stadiums
{
inline constexpr std::size_t MAX_GRID_CELLS = 1'500'000;
using Key = std::pair<std::string, std::uint64_t>;

enum class Sport
{
	Soccer,
	AmericanFootball,
	Rugby,
	Baseball,
	Cricket,
	Athletics,
	Oval,
	Indoor
};

struct Bbox
{
	int min_x = 0, min_z = 0, max_x = 0, max_z = 0;
	Bbox united(const Bbox &) const;
	Bbox padded(int) const;
	bool contains(int x, int z) const;
};

struct Shape
{
	std::vector<std::vector<std::pair<int, int>>> rings;
	std::vector<bool> outer;
	Bbox bbox;
	std::pair<int, int> centroid{0, 0};
	double area = 0.0;
	bool has_hole = false;
	Shape filled() const;
};

struct Grid
{
	int x0 = 0, z0 = 0;
	std::size_t w = 0, h = 0;
	static Grid from_bbox(const Bbox &);
	std::size_t size() const;
	std::optional<std::size_t> index(int x, int z) const;
	std::pair<int, int> coordinate(std::size_t i) const;
	Bbox bbox() const;
};

struct Frame
{
	double cx = 0, cz = 0, ux = 1, uz = 0, half_len = 0, half_wid = 0;
};

struct SynthField
{
	ProcessedWay way;
	std::vector<std::pair<int, int>> cells;
};

struct Profile
{
	int lip = 0;
	float tier_len = 0, walkway = 0;
	int jump = 0;
	std::uint8_t tiers = 0;
	int cap = 0;
	std::tuple<int, std::uint8_t, bool> at(float distance) const;
};

struct Palette
{
	Block seat, accent, wall, rib;
};

struct Venue
{
	std::optional<std::string> name;
	Grid grid;
	std::vector<bool> stand, field;
	std::vector<std::uint16_t> front, back;
	std::pair<double, double> centre{0, 0};
	std::optional<Frame> field_frame;
	std::optional<SynthField> synth_field;
	Profile profile;
	float roof_cover = 0;
	std::optional<int> roof_top;
	float roof_min_depth = 0;
	int roof_clearance = 0, mast_above = 0;
	Palette palette;
	double front_radius = 1;
};

class StadiumPlan
{
public:
	std::vector<Venue> venues;
	std::set<Key> suppressed;
	std::size_t placement_count() const { return venues.size(); }
	std::vector<std::vector<std::pair<int, int>>> placement_regions() const;
};

std::optional<Sport> sport_of(const tags_t &);
double apron_m(Sport);
std::optional<std::pair<double, double>> field_m(Sport);
const char *pitch_tag(Sport);
bool usually_roofed(Sport);
bool is_venue(const tags_t &);
bool is_stand(const tags_t &);
bool is_indoor(const tags_t &);
bool is_field(const tags_t &);
bool is_building(const tags_t &);
bool is_roofish(const tags_t &);
std::optional<double> metres(const tags_t &, const std::string &key);
std::optional<Shape> shape_of(const ProcessedElement &);
void rasterize(const Grid &, const Shape &, std::vector<bool> &out);
std::vector<std::uint16_t> chamfer(const Grid &, const std::vector<bool> &seed);
std::vector<bool> enclosed(const Grid &, const std::vector<bool> &mask);
std::optional<Frame> frame_of(const std::vector<std::pair<int, int>> &cells);
void close_gaps(const Grid &, std::vector<bool> &stand, const std::vector<bool> &within,
		double radius);
void thin_out(const Grid &, std::vector<bool> &stand, std::uint16_t min_depth);
std::optional<SynthField> synthesize_field(const Grid &, const std::vector<bool> &venue,
		std::optional<Sport>, std::uint64_t id, double scale);
StadiumPlan prescan(const std::vector<ProcessedElement> &, const Args &,
		const std::set<Key> &already = {});
void place_stadiums(::arnis::world_editor::WorldEditor &, const StadiumPlan &,
		const std::vector<std::size_t> &indices);
} // namespace arnis::element_processing::stadiums
