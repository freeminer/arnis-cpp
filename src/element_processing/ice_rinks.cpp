#include "ice_rinks.h"

#include "area_layout.h"
#include "connected_blocks.h"
#include "../element_processing/surfaces.h"

#include <algorithm>
#include <cmath>
#include <string_view>

namespace arnis::ice_rinks
{
namespace
{
struct Rink
{
	element_processing::area_layout::Frame frame;
	element_processing::area_layout::AreaMask mask;
	double corner{};

	bool contains(int32_t x, int32_t z) const
	{
		if (!mask.contains(x, z))
			return false;
		if (corner > 0.0) {
			const auto [a, c] = frame.local(x, z);
			const double da = std::max({corner - a, a - (frame.length() - corner), 0.0});
			const double dc = std::max({corner - c, c - (frame.width() - corner), 0.0});
			if (std::hypot(da, dc) > corner)
				return false;
		}
		return true;
	}
};

bool plays_hockey(const ProcessedWay &way)
{
	const auto found = way.tags.find("sport");
	if (found == way.tags.end())
		return false;
	std::string_view value = found->second;
	while (!value.empty()) {
		const size_t separator = value.find(';');
		std::string_view part = value.substr(0, separator);
		const size_t first = part.find_first_not_of(" \t\r\n");
		if (first != std::string_view::npos) {
			part.remove_prefix(first);
			const size_t last = part.find_last_not_of(" \t\r\n");
			part = part.substr(0, last + 1);
			if (part == "ice_hockey")
				return true;
		}
		if (separator == std::string_view::npos)
			break;
		value.remove_prefix(separator + 1);
	}
	return false;
}

std::optional<Block> marking(
		double a, double c, double length, double width, double scale)
{
	const auto near = [](double value, double at) { return std::abs(value - at) < 0.5; };
	const double goal_line = 0.055 * length;
	const double mid_a = length / 2.0, mid_c = width / 2.0;
	const double radius = std::min({4.57 * scale, width * .2, .08 * length});
	const auto ring = [&](double ca, double cc) {
		const double distance = std::hypot(a - ca, c - cc);
		return std::abs(distance - radius) < .5 || distance < .6;
	};
	if (near(a, mid_a) || ring(mid_a, mid_c))
		return near(a, mid_a) && !ring(mid_a, mid_c) ? RED_CONCRETE : BLUE_CONCRETE;
	if (near(a, .375 * length) || near(a, .625 * length))
		return BLUE_CONCRETE;
	for (const double end : {goal_line, length - goal_line}) {
		const double inward = end < mid_a ? a - end : end - a;
		const double crease = std::max(1.83 * scale, 1.5);
		if (inward >= 0.0 && inward < crease && std::hypot(a - end, c - mid_c) < crease)
			return LIGHT_BLUE_CONCRETE;
		if (near(a, end))
			return RED_CONCRETE;
		const double faceoff_a = end < mid_a ? end + 6.1 * scale : end - 6.1 * scale;
		for (const double faceoff_c : {mid_c - 6.7 * scale, mid_c + 6.7 * scale})
			if (ring(faceoff_a, faceoff_c))
				return RED_CONCRETE;
	}
	return std::nullopt;
}

void goal(WorldEditor &editor, int32_t x, int32_t z, double a, double c, double length,
		double width, double scale)
{
	const double goal_line = .055 * length;
	const double half = std::max(.9 * scale, 1.0);
	if (std::abs(c - width / 2.0) > half + .5)
		return;
	for (const double end : {goal_line, length - goal_line}) {
		const double behind = end < length / 2.0 ? end - a : a - end;
		if (behind >= .5 && behind < 1.5) {
			const bool post = std::abs(c - width / 2.0) > half - .5;
			editor.set_block(post ? NETHER_BRICK_FENCE : WHITE_WOOL, x, 1, z);
		}
	}
}
} // namespace

void generate_ice_rink(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int32_t, int32_t>> &fill, const Holes *holes)
{
	const auto frame = element_processing::area_layout::Frame::of(way.nodes);
	if (!frame)
		return;
	const double scale = std::clamp(editor.scale(), .1, 4.0);
	const bool boxy = way.nodes.size() <= 6;
	const double corner = boxy ? std::min(8.5 * scale, frame->width() / 3.0) : 0.0;
	const element_processing::area_layout::AreaMask mask(
			fill, [&](int32_t x, int32_t z) { return !in_hole(holes, x, z); });
	const Rink rink{*frame, mask, corner};
	const double length = frame->length(), width = frame->width();
	Block surface = PACKED_ICE;
	if (const auto found = way.tags.find("surface"); found != way.tags.end()) {
		const auto blocks = surfaces::get_blocks_for_surface(found->second);
		if (!blocks.empty())
			surface = blocks.front();
	}
	std::size_t rink_cells = 0;
	for (const auto &[x, z] : fill)
		rink_cells += !in_hole(holes, x, z);
	const bool rectangular = double(rink_cells) >= .85 * length * width;
	const bool hockey = plays_hockey(way) && rectangular && length >= 30.0 * scale &&
						width >= 15.0 * scale;
	static const std::optional<std::vector<Block>> grass{std::vector<Block>{GRASS_BLOCK}};
	for (const auto &[x, z] : fill) {
		if (in_hole(holes, x, z) || !editor.pos_ok(x, z) || !rink.contains(x, z))
			continue;
		const bool board = !rink.contains(x + 1, z) || !rink.contains(x - 1, z) ||
						   !rink.contains(x, z + 1) || !rink.contains(x, z - 1);
		const auto [a, c] = frame->local(x, z);
		const auto line =
				hockey && !board ? marking(a, c, length, width, scale) : std::nullopt;
		editor.set_block(line.value_or(surface), x, 0, z, grass, std::nullopt);
		if (!editor.owns(x, z))
			continue;
		if (board) {
			editor.set_block(WHITE_CONCRETE, x, 1, z);
			if (hockey) {
				const auto board_at = [&](int32_t nx, int32_t nz) {
					return rink.contains(nx, nz) &&
						   (!rink.contains(nx + 1, nz) || !rink.contains(nx - 1, nz) ||
								   !rink.contains(nx, nz + 1) ||
								   !rink.contains(nx, nz - 1));
				};
				const auto pane = connected_blocks::connected_fence(GLASS_PANE,
						board_at(x, z - 1), board_at(x, z + 1), board_at(x + 1, z),
						board_at(x - 1, z));
				editor.set_block_with_properties_absolute(pane, x,
						editor.get_absolute_y(x, 2, z), z, std::nullopt, std::nullopt);
			}
		} else if (hockey) {
			goal(editor, x, z, a, c, length, width, scale);
		}
	}
}
} // namespace arnis::ice_rinks
