#include "block_definitions.h"
#include "bresenham.h"
#include "../osm_parser.h"
#include "world_editor.h"
#include "waterways.h"
#include "bridges.h"
#include "../water_depth.h"
#include "../water_detail.h"
#include "field_texture.h"
#include "../strict_parse.h"
#include "../mapillary/geometry.h"
#include "../unicode_text.h"

#include <vector>
#include <string>
#include <unordered_map>
#include <optional>
#include <utility>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <stdexcept>
#include <cstdint>

#include "../../../arnis_adapter.h"
namespace arnis
{

namespace waterways
{

void generate_waterways(world_editor::WorldEditor &editor, const ProcessedWay &element,
		WaterDetail detail, double scale, const ChannelBed *bed);

int get_waterway_width(const std::string &waterway_type)
{
	if (waterway_type == "river")
		return 8;
	if (waterway_type == "canal")
		return 6;
	if (waterway_type == "stream")
		return 3;
	if (waterway_type == "fairway")
		return 12;
	if (waterway_type == "flowline")
		return 2;
	if (waterway_type == "brook")
		return 3;
	if (waterway_type == "ditch")
		return 1;
	if (waterway_type == "drain")
		return 1;
	return 4;
}

bool is_channel_waterway(const std::string &type)
{
	static const std::vector<std::string> excluded{"dam", "weir", "lock_gate",
			"waterfall", "rapids", "boatyard", "fuel", "dock", "riverbank", "water_point",
			"turning_point", "sluice_gate", "fish_pass", "security_lock", "milestone",
			"check_dam", "floating_barrier"};
	return std::find(excluded.begin(), excluded.end(), type) == excluded.end();
}

bool is_underground_waterway(const tags_t &tags)
{
	if (const auto it = tags.find("tunnel"); it != tags.end() && it->second != "no" &&
											 it->second != "0" && it->second != "false")
		return true;
	if (const auto it = tags.find("layer"); it != tags.end()) {
		const auto [begin, end] = unicode_text::trim_whitespace(it->second);
		const std::string_view value(it->second.data() + begin, end - begin);
		if (const auto layer = strict_parse::i32(value))
			return *layer < 0;
	}
	return false;
}

int waterway_width(const std::string &type, const tags_t &tags, double scale)
{
	double metres = static_cast<double>(get_waterway_width(type));
	if (const auto it = tags.find("width"); it != tags.end()) {
		const auto separator = it->second.find(';');
		const auto first = it->second.substr(0, separator);
		if (const auto width = mapillary::geodesy::parse_length_m(first);
				width && std::isfinite(*width) && *width > 0.0)
			metres = *width;
	}
	// Rust's positive-infinity-to-integer cast saturates, so the following
	// maximum-width check handles +inf as a capped channel. NaN and non-positive
	// scales fall back to one block, matching the Rust cast followed by clamp.
	if (std::isnan(scale) || scale <= 0.0)
		return 1;
	const double blocks = std::round(metres * scale);
	if (blocks <= 1.0)
		return 1;
	if (blocks >= MAX_WATERWAY_WIDTH)
		return MAX_WATERWAY_WIDTH;
	return static_cast<int>(blocks);
}

int capped_waterway_width(
		const std::string &type, const tags_t &tags, double scale, std::optional<int> cap)
{
	const int width = waterway_width(type, tags, scale);
	if (!cap || *cap <= 0)
		return width;
	if (get_waterway_width(type) <= 3)
		return std::min({width, *cap, 3});
	return std::min(width, *cap);
}

std::pair<int, int> channel_reach(int width)
{
	width = std::max(1, width);
	return {(width - 1) / 2, width / 2};
}

void channel_walk(const std::vector<ProcessedNode> &nodes, int width,
		const CoordinateBitmap *roads_at_grade, const XZBBox &area,
		const std::function<void(const ChannelCentre &, int, int)> &cell)
{
	const auto [before, after] = channel_reach(width);
	const int reach = std::max(before, after) + 1;
	std::optional<std::pair<int, int>> previous;
	for (std::size_t segment = 0; segment + 1 < nodes.size(); ++segment) {
		const auto &a = nodes[segment];
		const auto &b = nodes[segment + 1];
		if (std::int64_t(std::max(a.x, b.x)) + reach < area.min_x() ||
				std::int64_t(std::min(a.x, b.x)) - reach > area.max_x() ||
				std::int64_t(std::max(a.z, b.z)) + reach < area.min_z() ||
				std::int64_t(std::min(a.z, b.z)) - reach > area.max_z()) {
			previous.reset();
			continue;
		}
		const auto line = bresenham_line(a.x, 0, a.z, b.x, 0, b.z);
		const std::size_t steps = line.empty() ? 0 : line.size() - 1;
		for (std::size_t step = 0; step < line.size(); ++step) {
			const int x = std::get<0>(line[step]);
			const int z = std::get<2>(line[step]);
			if (roads_at_grade && roads_at_grade->contains(x, z)) {
				previous.reset();
				continue;
			}
			const ChannelCentre centre{x, z, segment, step, steps};
			if (width <= 1 && previous && previous->first != x && previous->second != z)
				cell(centre, previous->first, z);
			for (int cx = x - before; cx <= x + after; ++cx)
				for (int cz = z - before; cz <= z + after; ++cz)
					cell(centre, cx, cz);
			previous = std::pair{x, z};
		}
	}
}

void fill_channel_cell(
		WorldEditor &editor, int x, int z, int target_water_y, const ChannelBed *bed)
{
	const int BANK_TOLERANCE = 2;
	const int ground_y = editor.get_ground_level(x, z);
	std::optional<int> water_y;
	if (ground_y <= target_water_y)
		water_y = target_water_y;
	else if (ground_y <= target_water_y + BANK_TOLERANCE &&
			 !editor.block_exists_absolute(x, ground_y, z))
		water_y = ground_y;
	if (!water_y)
		return;
	editor.set_block_absolute(WATER, x, *water_y, z, std::nullopt, std::nullopt);
	const bool under_deck = bed && bed->piped_under_roads &&
							bed->piped_under_roads->deck_clears(x, z, *water_y) &&
							!bed->piped_under_roads->over_grade_way(x, z);
	if (bed &&
			(bed->field.is_river(x, z) ||
					(!editor.is_lc_water(x, z) && editor.is_mapped_water(x, z))) &&
			(!bed->road_mask.contains(x, z) || under_deck) &&
			!water_depth::at_bore(bed->field, bed->tunnel_footprint, x, z) &&
			editor.check_for_block_absolute(x, *water_y, z,
					std::optional<std::vector<Block>>(std::vector<Block>{WATER}),
					std::nullopt))
		water_depth::carve_water_column(editor, x, z, *water_y, bed->field.depth_at(x, z),
				bed->road_mask, bed->field);
	editor.set_block_absolute(AIR, x, *water_y + 1, z,
			std::optional<std::vector<Block>>(
					std::vector<Block>(field_texture::PLOT_PLANTS.begin(),
							field_texture::PLOT_PLANTS.end())),
			std::nullopt);
	editor.set_block_absolute(AIR, x, *water_y + 2, z,
			std::optional<std::vector<Block>>(std::vector<Block>{SUNFLOWER_UPPER}),
			std::nullopt);
}

void generate_waterways(world_editor::WorldEditor &editor, const ProcessedWay &element)
{
	generate_waterways(editor, element, WaterDetail::Default, 1.0);
}

void generate_waterways(world_editor::WorldEditor &editor, const ProcessedWay &element,
		WaterDetail detail, double scale)
{
	generate_waterways(editor, element, detail, scale, nullptr);
}

void generate_waterways(world_editor::WorldEditor &editor, const ProcessedWay &element,
		WaterDetail detail, double scale, const ChannelBed &bed)
{
	generate_waterways(editor, element, detail, scale, &bed);
}

void generate_waterways(world_editor::WorldEditor &editor, const ProcessedWay &element,
		WaterDetail detail, double scale, const ChannelBed *bed)
{
	auto it = element.tags.find("waterway");
	if (it == element.tags.end()) {
		return;
	}
	if (!is_channel_waterway(it->second) || is_underground_waterway(element.tags))
		return;

	const int cap = water_detail::channel_width_cap(detail, scale);
	const int channel_width = capped_waterway_width(it->second, element.tags, scale,
			cap > 0 ? std::optional<int>(cap) : std::nullopt);

	if (element.nodes.size() < 2)
		return;
	const auto [min_x, max_x] = std::minmax_element(element.nodes.begin(),
			element.nodes.end(), [](const auto &a, const auto &b) { return a.x < b.x; });
	const auto [min_z, max_z] = std::minmax_element(element.nodes.begin(),
			element.nodes.end(), [](const auto &a, const auto &b) { return a.z < b.z; });
	const XZBBox bounds(min_x->x, min_z->z, max_x->x, max_z->z);
	std::optional<std::tuple<std::size_t, int, int>> endpoints;
	std::optional<std::pair<ChannelCentre, int>> surface;
	channel_walk(element.nodes, channel_width, bed ? bed->roads_at_grade : nullptr,
			bounds, [&](const ChannelCentre &centre, int x, int z) {
				int water_y;
				if (surface && surface->first == centre) {
					water_y = surface->second;
				} else {
					if (!endpoints || std::get<0>(*endpoints) != centre.segment) {
						const auto a = element.nodes[centre.segment].xz();
						const auto b = element.nodes[centre.segment + 1].xz();
						endpoints = std::make_tuple(centre.segment,
								editor.get_water_level(a.x, a.z),
								editor.get_water_level(b.x, b.z));
					}
					const auto [segment, y0, y1] = *endpoints;
					(void)segment;
					const auto last = static_cast<std::int64_t>(centre.steps);
					const auto dy = static_cast<std::int64_t>(y1) - y0;
					const auto num = dy * static_cast<std::int64_t>(centre.step);
					const int ramped =
							last == 0 ? std::min(y0, y1)
									  : static_cast<int>(
												y0 +
												(2 * num + last * ((dy > 0) - (dy < 0))) /
														(2 * last));
					water_y =
							std::min(ramped, editor.get_water_level(centre.x, centre.z));
					surface = std::make_pair(centre, water_y);
				}
				if (bed && bed->piped_under_roads && bed->road_mask.contains(x, z) &&
						(!bed->piped_under_roads->deck_clears(x, z, water_y) ||
								bed->piped_under_roads->over_grade_way(x, z)))
					return;
				fill_channel_cell(editor, x, z, water_y, bed);
			});
}

}
}
