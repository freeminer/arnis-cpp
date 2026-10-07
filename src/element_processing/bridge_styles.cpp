#include "bridge_styles.h"
#include "../assets_root.h"
#include "../structures/schem_decoder.h"
#include "bridge_modules.h"
#include "bridges.h"
#include "connected_blocks.h"
#include <fstream>
#include <cmath>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "../bresenham.h"
#include "irrlichttypes.h"

namespace arnis::bridge_styles
{

bool sweep_bridge_schematic(WorldEditor &editor, const bridges::BridgeSurfaceMap &surface,
		const std::vector<BridgePathSample> &path, std::optional<std::size_t> module_idx,
		const std::filesystem::path &asset_root)
{
	if (!module_idx)
		return false;
	const auto *module = bridge_modules::module_at(*module_idx, asset_root);
	if (!module)
		return false;
	std::vector<bridge_modules::BridgePathSample> module_path;
	module_path.reserve(path.size());
	for (const auto &sample : path)
		module_path.push_back(
				{sample.x, sample.y, sample.z, sample.perp_x, sample.perp_z});
	bridge_modules::sweep_module(&editor, surface, module_path, *module);
	return true;
}
namespace
{

constexpr std::size_t BEAM_PILLAR_INTERVAL = 8;
constexpr std::size_t WIDE_BEAM_PIER_INTERVAL = 12;
constexpr std::size_t BOARDWALK_POST_INTERVAL = 4;
constexpr std::size_t ARCH_SPAN = 20;
constexpr float ARCH_RISE_FRACTION = 0.85f;
constexpr int PIER_FOOTING_MIN_HEIGHT = 5;
constexpr int TRUSS_TOP_HEIGHT = 5;
constexpr std::size_t TRUSS_DIAGONAL_PERIOD = 8;
constexpr std::size_t TRUSS_POST_INTERVAL = 4;
constexpr std::size_t TRUSS_PIER_INTERVAL = 32;
constexpr std::size_t CABLE_FALLBACK_PIER_INTERVAL = 24;
constexpr std::size_t TRUSS_PORTAL_INTERVAL = 8;
constexpr int SUSPENSION_TOWER_BASE_HEIGHT = 8;
constexpr std::size_t SUSPENSION_TOWER_HEIGHT_DIVISOR = 6;
constexpr int SUSPENSION_TOWER_MAX_HEIGHT = 32;
constexpr std::size_t SUSPENSION_HANGER_INTERVAL = 4;
constexpr float SUSPENSION_TOWER_INSET_FRAC = 0.12f;
constexpr std::size_t SUSPENSION_MIN_LENGTH = 18;
constexpr std::size_t SUSPENSION_INTER_PYLON_SPACING = 1000;
constexpr std::size_t SUSPENSION_MAX_PYLONS = 5;
constexpr int CABLE_STAYED_TOWER_BASE_HEIGHT = 12;
constexpr std::size_t CABLE_STAYED_TOWER_HEIGHT_DIVISOR = 5;
constexpr int CABLE_STAYED_TOWER_MAX_HEIGHT = 40;
constexpr std::size_t CABLE_STAYED_ANCHOR_INTERVAL = 14;
constexpr std::size_t CABLE_STAYED_MIN_LENGTH = 18;
constexpr std::size_t CABLE_STAYED_MIN_GAP = 4;
constexpr std::size_t CABLE_STAYED_TWIN_PYLON_LENGTH = 100;
constexpr int COVERED_WALL_HEIGHT = 4;
constexpr std::size_t COVERED_WINDOW_INTERVAL = 4;
constexpr std::size_t COVERED_END_CLEAR = 1;

BridgeStyle resolve_bridge_style_from_pair(const std::optional<std::string> &structure,
		const std::optional<std::string> &bridge)
{
	if (structure) {
		if (*structure == "arch")
			return BridgeStyle::Arch;
		if (*structure == "truss")
			return BridgeStyle::Truss;
		if (*structure == "suspension" || *structure == "simple-suspension")
			return BridgeStyle::Suspension;
		if (*structure == "cable-stayed" || *structure == "cable_stayed")
			return BridgeStyle::CableStayed;
		if (*structure == "beam")
			return BridgeStyle::Beam;
	}
	if (bridge) {
		if (*bridge == "covered")
			return BridgeStyle::Covered;
		if (*bridge == "boardwalk")
			return BridgeStyle::Boardwalk;
		if (*bridge == "cable-stayed" || *bridge == "cable_stayed")
			return BridgeStyle::CableStayed;
		if (*bridge == "suspension" || *bridge == "suspension_bridge")
			return BridgeStyle::Suspension;
		if (*bridge == "truss")
			return BridgeStyle::Truss;
	}
	return BridgeStyle::Beam;
}

std::pair<int, int> centroid_xz(const ProcessedWay &way)
{
	if (way.nodes.empty())
		return {0, 0};
	long long sx = 0;
	long long sz = 0;
	for (const auto &node : way.nodes) {
		sx += node.x;
		sz += node.z;
	}
	const auto n = static_cast<long long>(way.nodes.size());
	return {static_cast<int>(sx / n), static_cast<int>(sz / n)};
}

bool point_in_polygon(int x, int z, const std::vector<std::pair<int, int>> &poly)
{
	const std::size_t n = poly.size();
	if (n < 3)
		return false;
	const double xf = static_cast<double>(x);
	const double zf = static_cast<double>(z);
	bool inside = false;
	std::size_t j = n - 1;
	for (std::size_t i = 0; i < n; ++i) {
		const double xi = static_cast<double>(poly[i].first);
		const double zi = static_cast<double>(poly[i].second);
		const double xj = static_cast<double>(poly[j].first);
		const double zj = static_cast<double>(poly[j].second);
		const bool intersects =
				((zi > zf) != (zj > zf)) &&
				xf < (xj - xi) * (zf - zi) /
										(zj - zi +
												std::numeric_limits<double>::epsilon()) +
								xi;
		if (intersects)
			inside = !inside;
		j = i;
	}
	return inside;
}

std::pair<std::size_t, std::size_t> arch_segment(std::size_t tds, std::size_t total)
{
	if (total < 2)
		return {0, total};
	const std::size_t n_arches =
			std::max<std::size_t>(1, (total + ARCH_SPAN / 2) / ARCH_SPAN);
	const std::size_t arch_idx = (tds * n_arches) / total;
	const std::size_t arch_start = (total * arch_idx) / n_arches;
	const std::size_t arch_end = (total * (arch_idx + 1)) / n_arches;
	return {arch_start, arch_end - arch_start};
}

float arch_local_t(std::size_t tds, std::size_t total)
{
	const auto [start, span] = arch_segment(tds, total);
	if (span <= 1)
		return 0.0f;
	return static_cast<float>(tds - start) / static_cast<float>(span - 1);
}

void place_pillar(
		WorldEditor &editor, int x, int deck_y, int z, Block body, bool with_base)
{
	const int ground_y = editor.get_ground_level(x, z);
	if (deck_y - ground_y <= 2)
		return;
	for (int y = ground_y + 1; y < deck_y; ++y)
		editor.set_block_absolute(body, x, y, z, std::nullopt, std::nullopt);
	editor.register_support_column(x, z, body);
	if (with_base && deck_y - ground_y >= PIER_FOOTING_MIN_HEIGHT)
		for (int bx = -1; bx <= 1; ++bx)
			for (int bz = -1; bz <= 1; ++bz)
				editor.set_block_absolute(
						body, x + bx, ground_y, z + bz, std::nullopt, std::nullopt);
}

void place_arch_spandrel_cell(WorldEditor &editor, int set_x, int cell_y, int set_z,
		int centerline_ground_y, std::size_t tds, std::size_t total, bool use_absolute_y)
{
	const int dist_to_deck = std::max(0, cell_y - 2 - centerline_ground_y);
	if (dist_to_deck <= 0)
		return;
	const int max_rise =
			static_cast<int>(static_cast<float>(dist_to_deck) * ARCH_RISE_FRACTION);
	const float t = arch_local_t(tds, total);
	const int rise_at_cell =
			static_cast<int>(static_cast<float>(max_rise) * 4.0f * t * (1.0f - t));
	const int arch_under_y = centerline_ground_y + rise_at_cell;
	const int fill_top = cell_y - 2;
	if (arch_under_y > fill_top)
		return;
	for (int fy = arch_under_y; fy <= fill_top; ++fy) {
		if (use_absolute_y)
			editor.set_block_absolute(STONE_BRICKS, set_x, fy, set_z, std::nullopt,
					std::optional<std::vector<Block>>{std::vector<Block>{WATER}});
		else
			editor.set_block(STONE_BRICKS, set_x, fy, set_z, std::nullopt,
					std::optional<std::vector<Block>>{std::vector<Block>{WATER}});
	}
	if (use_absolute_y && arch_under_y <= centerline_ground_y + 1)
		editor.register_support_column(set_x, set_z, STONE_BRICKS);
}

std::pair<std::pair<int, int>, std::pair<int, int>> side_offsets(
		int cx, int cz, float px, float pz, int block_range)
{
	const float rail_dist =
			static_cast<float>(block_range) * (std::abs(px) + std::abs(pz)) + 1.0f;
	const int lx = static_cast<int>(std::round(static_cast<float>(cx) + px * rail_dist));
	const int lz = static_cast<int>(std::round(static_cast<float>(cz) + pz * rail_dist));
	const int rx = static_cast<int>(std::round(static_cast<float>(cx) - px * rail_dist));
	const int rz = static_cast<int>(std::round(static_cast<float>(cz) - pz * rail_dist));
	return {{lx, lz}, {rx, rz}};
}

struct OpenSides
{
	bool left = false;
	bool right = false;
};

bool side_faces_open(const bridges::BridgeSurfaceMap &surface, std::pair<int, int> edge,
		float outward_x, float outward_z, int deck_y)
{
	for (int k = 0; k <= 2; ++k) {
		const int x = static_cast<int>(std::lround(edge.first + outward_x * k));
		const int z = static_cast<int>(std::lround(edge.second + outward_z * k));
		if (surface.deck_near(x, z, deck_y, 2))
			return false;
	}
	return true;
}

OpenSides open_sides(const bridges::BridgeSurfaceMap &surface,
		const std::vector<BridgePathSample> &path, int block_range)
{
	if (path.empty())
		return {};
	std::size_t left_count = 0;
	std::size_t right_count = 0;
	for (const auto &sample : path) {
		const auto [left, right] = side_offsets(
				sample.x, sample.z, sample.perp_x, sample.perp_z, block_range);
		left_count +=
				side_faces_open(surface, left, sample.perp_x, sample.perp_z, sample.y);
		right_count +=
				side_faces_open(surface, right, -sample.perp_x, -sample.perp_z, sample.y);
	}
	return {left_count * 2 >= path.size(), right_count * 2 >= path.size()};
}

struct SideRuns
{
	std::array<std::optional<std::pair<int, int>>, 2> previous{};

	std::vector<std::pair<int, int>> step(std::size_t side, std::pair<int, int> cell)
	{
		std::vector<std::pair<int, int>> cells;
		if (previous[side] && *previous[side] != cell)
			cells = connected_blocks::stair_steps(*previous[side], cell);
		else
			cells.push_back(cell);
		previous[side] = cell;
		return cells;
	}
};

std::vector<std::pair<int, int>> span_cells(std::pair<int, int> a, std::pair<int, int> b)
{
	std::vector<std::pair<int, int>> line;
	for (const auto &[x, y, z] :
			bresenham_line(a.first, 0, a.second, b.first, 0, b.second)) {
		(void)y;
		line.emplace_back(x, z);
	}
	return connected_blocks::four_connected_line(line);
}

int suspension_tower_height(std::size_t total)
{
	const int extra = static_cast<int>(total / SUSPENSION_TOWER_HEIGHT_DIVISOR);
	return std::min(SUSPENSION_TOWER_BASE_HEIGHT + extra, SUSPENSION_TOWER_MAX_HEIGHT);
}

std::size_t suspension_pylon_count(std::size_t total)
{
	return std::min<std::size_t>(
			2 + total / SUSPENSION_INTER_PYLON_SPACING, SUSPENSION_MAX_PYLONS);
}

std::vector<std::size_t> default_pylons_impl(BridgeStyle style, std::size_t total,
		bool start_is_boundary, bool end_is_boundary)
{
	if (style == BridgeStyle::Suspension && start_is_boundary && end_is_boundary) {
		if (total < SUSPENSION_MIN_LENGTH)
			return {};
		const std::size_t inset = std::max<std::size_t>(
				static_cast<std::size_t>(
						static_cast<float>(total) * SUSPENSION_TOWER_INSET_FRAC),
				2);
		if (inset * 2 + 2 > total)
			return {};
		const std::size_t count = suspension_pylon_count(total);
		const std::size_t first = inset;
		const std::size_t last = total - 1 - inset;
		std::vector<std::size_t> points;
		points.reserve(count);
		for (std::size_t i = 0; i < count; ++i)
			points.push_back(
					first + (last - first) * i / std::max<std::size_t>(1, count - 1));
		return points;
	}
	if (style == BridgeStyle::CableStayed && total >= CABLE_STAYED_MIN_LENGTH) {
		if (total >= CABLE_STAYED_TWIN_PYLON_LENGTH)
			return {total / 3, (2 * total) / 3};
		return {total / 2};
	}
	return {};
}

int cable_stayed_tower_height(std::size_t total)
{
	const int extra = static_cast<int>(total / CABLE_STAYED_TOWER_HEIGHT_DIVISOR);
	return std::min(
			CABLE_STAYED_TOWER_BASE_HEIGHT + extra, CABLE_STAYED_TOWER_MAX_HEIGHT);
}

void place_pylon_crossbeam(WorldEditor &editor, std::pair<int, int> left,
		std::pair<int, int> right, int top_y);

void place_pylon(WorldEditor &editor, std::pair<int, int> base,
		std::pair<float, float> outward, int deck_y, int height)
{
	const std::pair<float, float> along{outward.second, -outward.first};
	const int top_y = deck_y + height;
	std::vector<std::pair<int, int>> cells;
	for (const auto [a, b] :
			std::array<std::pair<float, float>, 4>{{{0, 0}, {1, 0}, {0, 1}, {1, 1}}}) {
		const std::pair<int, int> cell{
				static_cast<int>(
						std::round(base.first + outward.first * a + along.first * b)),
				static_cast<int>(
						std::round(base.second + outward.second * a + along.second * b))};
		if (std::find(cells.begin(), cells.end(), cell) == cells.end())
			cells.push_back(cell);
	}
	for (const auto &[x, z] : cells) {
		const int base_y = std::min(editor.get_ground_level(x, z), deck_y);
		for (int y = base_y + 1; y <= top_y; ++y)
			editor.set_block_absolute(SMOOTH_STONE, x, y, z, std::nullopt, std::nullopt);
		editor.register_support_column(x, z, SMOOTH_STONE);
	}
}

void place_pylon_pair(WorldEditor &editor, std::pair<int, int> center,
		std::pair<float, float> perp, int block_range, int deck_y, int height,
		OpenSides sides)
{
	const auto [left, right] = side_offsets(
			center.first, center.second, perp.first, perp.second, block_range);
	if (sides.left)
		place_pylon(editor, left, perp, deck_y, height);
	if (sides.right)
		place_pylon(editor, right, {-perp.first, -perp.second}, deck_y, height);
	if (sides.left && sides.right)
		place_pylon_crossbeam(editor, left, right, deck_y + height);
}

void place_pylon_crossbeam(WorldEditor &editor, std::pair<int, int> left,
		std::pair<int, int> right, int top_y)
{
	for (const auto &[cx, cz] : span_cells(left, right))
		editor.set_block_absolute(
				SMOOTH_STONE, cx, top_y, cz, std::nullopt, std::nullopt);
}

void draw_cable(WorldEditor &editor, int x1, int y1, int z1, int x2, int y2, int z2)
{
	const int dx = x2 - x1;
	const int dz = z2 - z1;
	const Block chain = std::abs(dx) >= std::abs(dz) ? CHAIN_X : CHAIN_Z;
	std::optional<std::tuple<int, int, int>> prev;
	for (const auto &[cx, cy, cz] : bresenham_line(x1, y1, z1, x2, y2, z2)) {
		editor.set_block_absolute(chain, cx, cy, cz, std::nullopt, std::nullopt);
		if (prev) {
			const auto [px, py, pz] = *prev;
			const int axes_changed = (cx != px) + (cy != py) + (cz != pz);
			if (axes_changed >= 2) {
				editor.set_block_absolute(chain, cx, py, cz, std::nullopt, std::nullopt);
				if (axes_changed == 3)
					editor.set_block_absolute(
							chain, cx, py, pz, std::nullopt, std::nullopt);
			}
		}
		prev = std::make_tuple(cx, cy, cz);
	}
}

void decorate_truss(WorldEditor &editor, const std::vector<BridgePathSample> &path,
		int block_range, bool start_is_boundary, bool end_is_boundary, OpenSides sides)
{
	const std::size_t last = path.size() - 1;
	SideRuns runs;
	for (std::size_t tds = 0; tds < path.size(); ++tds) {
		const auto &s = path[tds];
		const auto [left, right] =
				side_offsets(s.x, s.z, s.perp_x, s.perp_z, block_range);
		const bool open_end =
				(tds == 0 && start_is_boundary) || (tds == last && end_is_boundary);
		const int top_y = s.y + 1 + TRUSS_TOP_HEIGHT;
		const std::size_t p = tds % TRUSS_DIAGONAL_PERIOD;
		const std::size_t half = TRUSS_DIAGONAL_PERIOD / 2;
		const int dh = static_cast<int>(p <= half ? p : TRUSS_DIAGONAL_PERIOD - p);
		const int diag_y = s.y + 1 + std::min(dh, TRUSS_TOP_HEIGHT);
		const std::array<std::pair<bool, std::pair<int, int>>, 2> edges{
				{{sides.left, left}, {sides.right, right}}};
		for (std::size_t side = 0; side < edges.size(); ++side) {
			if (!edges[side].first)
				continue;
			const auto &[sx, sz] = edges[side].second;
			const auto run = runs.step(side, {sx, sz});
			if (open_end)
				continue;
			for (const auto &[rx, rz] : run) {
				editor.set_block_absolute(
						IRON_BLOCK, rx, s.y + 1, rz, std::nullopt, std::nullopt);
				editor.set_block_absolute(
						IRON_BLOCK, rx, top_y, rz, std::nullopt, std::nullopt);
			}
			if (tds % TRUSS_POST_INTERVAL == 0)
				for (int h = 1; h <= TRUSS_TOP_HEIGHT; ++h)
					editor.set_block_absolute(
							IRON_BLOCK, sx, s.y + 1 + h, sz, std::nullopt, std::nullopt);
			editor.set_block_absolute(
					IRON_BLOCK, sx, diag_y, sz, std::nullopt, std::nullopt);
		}
		if (!open_end && sides.left && sides.right && tds % TRUSS_PORTAL_INTERVAL == 0)
			for (const auto &[bx, bz] : span_cells(left, right))
				editor.set_block_absolute(
						IRON_BLOCK, bx, top_y, bz, std::nullopt, std::nullopt);
	}
}

void decorate_suspension(WorldEditor &editor, const std::vector<BridgePathSample> &path,
		int block_range, const std::vector<std::size_t> &pylons, OpenSides sides)
{
	const std::size_t total = path.size();
	if (pylons.empty())
		return;
	const std::size_t last_idx = total - 1;
	const int height = suspension_tower_height(total);

	for (const auto &p : pylons) {
		const auto &s = path[p];
		place_pylon_pair(editor, {s.x, s.z}, {s.perp_x, s.perp_z}, block_range, s.y,
				height, sides);
	}

	const float dip = static_cast<float>(height - 2);
	for (std::size_t wi = 1; wi < pylons.size(); ++wi) {
		const std::size_t a = pylons[wi - 1];
		const std::size_t b = pylons[wi];
		const float span_len = static_cast<float>(b - a);
		if (span_len < 1.0f)
			continue;
		const int top_a = path[a].y + height;
		const int top_b = path[b].y + height;
		SideRuns runs;
		for (std::size_t tds = a; tds <= b; ++tds) {
			const auto &s = path[tds];
			const auto [left, right] =
					side_offsets(s.x, s.z, s.perp_x, s.perp_z, block_range);
			const float t = static_cast<float>(tds - a) / span_len;
			const float base_y =
					static_cast<float>(top_a) + static_cast<float>(top_b - top_a) * t;
			const int cable_y =
					static_cast<int>(std::round(base_y - dip * 4.0f * t * (1.0f - t)));
			const Block chain =
					std::abs(s.perp_x) > std::abs(s.perp_z) ? CHAIN_Z : CHAIN_X;
			const std::array<std::pair<bool, std::pair<int, int>>, 2> edges{
					{{sides.left, left}, {sides.right, right}}};
			for (std::size_t side = 0; side < edges.size(); ++side) {
				if (!edges[side].first)
					continue;
				for (const auto &[rx, rz] : runs.step(side, edges[side].second))
					editor.set_block_absolute(
							chain, rx, cable_y, rz, std::nullopt, std::nullopt);
			}
			if ((tds - a) % SUSPENSION_HANGER_INTERVAL == 0 && tds != a && tds != b) {
				for (int hy = s.y + 2; hy < cable_y; ++hy) {
					if (sides.left)
						connected_blocks::place_connected(
								editor, IRON_BARS, left.first, hy, left.second);
					if (sides.right)
						connected_blocks::place_connected(
								editor, IRON_BARS, right.first, hy, right.second);
				}
			}
		}
	}

	const std::size_t first_p = pylons.front();
	const std::size_t last_p = pylons.back();
	const auto [lf, rf] = side_offsets(path[first_p].x, path[first_p].z,
			path[first_p].perp_x, path[first_p].perp_z, block_range);
	const auto [ls, rs] = side_offsets(
			path[0].x, path[0].z, path[0].perp_x, path[0].perp_z, block_range);
	if (sides.left)
		draw_cable(editor, lf.first, path[first_p].y + height, lf.second, ls.first,
				path[0].y + 1, ls.second);
	if (sides.right)
		draw_cable(editor, rf.first, path[first_p].y + height, rf.second, rs.first,
				path[0].y + 1, rs.second);

	const auto [ll, rl] = side_offsets(path[last_p].x, path[last_p].z,
			path[last_p].perp_x, path[last_p].perp_z, block_range);
	const auto [le, re] = side_offsets(path[last_idx].x, path[last_idx].z,
			path[last_idx].perp_x, path[last_idx].perp_z, block_range);
	if (sides.left)
		draw_cable(editor, ll.first, path[last_p].y + height, ll.second, le.first,
				path[last_idx].y + 1, le.second);
	if (sides.right)
		draw_cable(editor, rl.first, path[last_p].y + height, rl.second, re.first,
				path[last_idx].y + 1, re.second);
}

void decorate_cable_stayed(WorldEditor &editor, const std::vector<BridgePathSample> &path,
		int block_range, const std::vector<std::size_t> &pylons, OpenSides sides)
{
	const std::size_t total = path.size();
	if (total < CABLE_STAYED_MIN_LENGTH || pylons.empty())
		return;
	const int height = cable_stayed_tower_height(total);
	const std::size_t last_idx = total - 1;

	for (std::size_t idx = 0; idx < pylons.size(); ++idx) {
		const std::size_t t_tds = pylons[idx];
		const auto &tower = path[t_tds];
		const auto [left_t, right_t] =
				side_offsets(tower.x, tower.z, tower.perp_x, tower.perp_z, block_range);
		const int top_y = tower.y + height;
		place_pylon_pair(editor, {tower.x, tower.z}, {tower.perp_x, tower.perp_z},
				block_range, tower.y, height, sides);

		const std::size_t anchor_lo = idx == 0 ? 0 : (pylons[idx - 1] + t_tds) / 2;
		const std::size_t anchor_hi =
				idx + 1 == pylons.size() ? total : (t_tds + pylons[idx + 1]) / 2;
		std::vector<std::size_t> anchors;
		for (std::size_t distance = CABLE_STAYED_ANCHOR_INTERVAL; distance < total;
				distance += CABLE_STAYED_ANCHOR_INTERVAL) {
			if (t_tds >= distance)
				anchors.push_back(t_tds - distance);
			if (distance < total - t_tds)
				anchors.push_back(t_tds + distance);
		}
		for (const auto tds : anchors) {
			const std::size_t gap = tds > t_tds ? tds - t_tds : t_tds - tds;
			if (tds < anchor_lo || tds >= anchor_hi || tds == 0 || tds >= last_idx ||
					gap < CABLE_STAYED_MIN_GAP)
				continue;
			const auto &anchor = path[tds];
			const auto [left_a, right_a] = side_offsets(
					anchor.x, anchor.z, anchor.perp_x, anchor.perp_z, block_range);
			if (sides.left)
				draw_cable(editor, left_t.first, top_y, left_t.second, left_a.first,
						anchor.y + 1, left_a.second);
			if (sides.right)
				draw_cable(editor, right_t.first, top_y, right_t.second, right_a.first,
						anchor.y + 1, right_a.second);
		}
	}
}

void decorate_covered(WorldEditor &editor, const std::vector<BridgePathSample> &path,
		int block_range, bool start_is_boundary, bool end_is_boundary, OpenSides sides)
{
	if (path.size() < 4)
		return;
	const std::size_t last = path.size() - 1;
	SideRuns runs;
	for (std::size_t tds = 0; tds < path.size(); ++tds) {
		const auto &s = path[tds];
		const auto [left, right] =
				side_offsets(s.x, s.z, s.perp_x, s.perp_z, block_range);
		const bool open_end = (start_is_boundary && tds < COVERED_END_CLEAR) ||
							  (end_is_boundary && tds + COVERED_END_CLEAR > last);
		const std::array<std::pair<bool, std::pair<int, int>>, 2> edges{
				{{sides.left, left}, {sides.right, right}}};
		for (std::size_t side = 0; side < edges.size(); ++side) {
			if (!edges[side].first)
				continue;
			const auto run = runs.step(side, edges[side].second);
			if (open_end)
				continue;
			for (const auto &[wx, wz] : run)
				for (int h = 1; h <= COVERED_WALL_HEIGHT; ++h) {
					const bool window = h == 2 && wx == edges[side].second.first &&
										wz == edges[side].second.second &&
										tds % COVERED_WINDOW_INTERVAL == 0;
					const Block b = window ? GLASS : DARK_OAK_PLANKS;
					editor.set_block_absolute(
							b, wx, s.y + h, wz, std::nullopt, std::nullopt);
				}
		}
		if (open_end)
			continue;
		const int roof_y = s.y + COVERED_WALL_HEIGHT + 1;
		for (const auto &[rx, rz] : span_cells(left, right))
			editor.set_block_absolute(
					DARK_OAK_PLANKS, rx, roof_y, rz, std::nullopt, std::nullopt);
	}
}

}

std::vector<std::size_t> default_pylons(BridgeStyle style, std::size_t total,
		bool start_is_boundary, bool end_is_boundary)
{
	return default_pylons_impl(style, total, start_is_boundary, end_is_boundary);
}

Block foundation_block(BridgeStyle style)
{
	return style == BridgeStyle::Boardwalk ? OAK_PLANKS : STONE_BRICKS;
}

Block rail_block(BridgeStyle style)
{
	return style == BridgeStyle::Boardwalk ? OAK_FENCE : LIGHT_GRAY_CONCRETE;
}

std::size_t pillar_interval(BridgeStyle style)
{
	return pillar_interval(style, 0);
}

std::size_t pillar_interval(BridgeStyle style, int half_width)
{
	switch (style) {
	case BridgeStyle::Boardwalk:
		return BOARDWALK_POST_INTERVAL;
	case BridgeStyle::Arch:
		return 0;
	case BridgeStyle::Truss:
		return TRUSS_PIER_INTERVAL;
	case BridgeStyle::Suspension:
	case BridgeStyle::CableStayed:
		return CABLE_FALLBACK_PIER_INTERVAL;
	case BridgeStyle::Beam:
	case BridgeStyle::Covered:
		return half_width <= 2 ? BEAM_PILLAR_INTERVAL : WIDE_BEAM_PIER_INTERVAL;
	}
	return BEAM_PILLAR_INTERVAL;
}

bool has_side_railing(BridgeStyle style)
{
	return style != BridgeStyle::Boardwalk;
}

std::optional<Block> parapet_block(BridgeStyle style)
{
	switch (style) {
	case BridgeStyle::Boardwalk:
	case BridgeStyle::Covered:
		return std::nullopt;
	case BridgeStyle::Truss:
	case BridgeStyle::Suspension:
	case BridgeStyle::CableStayed:
		return IRON_BARS;
	default:
		return BRICK_WALL;
	}
}

Block rail_foundation_block(BridgeStyle style)
{
	return style == BridgeStyle::Boardwalk ? OAK_PLANKS : STONE_BRICKS;
}

BridgeOutlineIndex BridgeOutlineIndex::build(
		const std::vector<ProcessedElement> &elements)
{
	BridgeOutlineIndex index;
	for (const auto &elem : elements) {
		if (!elem.is_way())
			continue;
		const auto &way = elem.as_way();
		if (way.tags.get("man_made") != "bridge")
			continue;
		std::optional<std::string> structure;
		std::optional<std::string> bridge;
		if (const auto it = way.tags.find("bridge:structure"); it != way.tags.end())
			structure = it->second;
		if (const auto it = way.tags.find("bridge"); it != way.tags.end())
			bridge = it->second;
		const bool has_style =
				structure.has_value() ||
				(bridge && (*bridge == "covered" || *bridge == "boardwalk" ||
								   *bridge == "cable-stayed" ||
								   *bridge == "cable_stayed" || *bridge == "suspension" ||
								   *bridge == "suspension_bridge" || *bridge == "truss"));
		if (!has_style || way.nodes.size() < 3)
			continue;

		OutlineEntry entry;
		entry.structure = structure;
		entry.bridge = bridge;
		entry.bbox_min_x = std::numeric_limits<int>::max();
		entry.bbox_min_z = std::numeric_limits<int>::max();
		entry.bbox_max_x = std::numeric_limits<int>::min();
		entry.bbox_max_z = std::numeric_limits<int>::min();
		for (const auto &node : way.nodes) {
			entry.nodes.emplace_back(node.x, node.z);
			entry.bbox_min_x = std::min(entry.bbox_min_x, node.x);
			entry.bbox_max_x = std::max(entry.bbox_max_x, node.x);
			entry.bbox_min_z = std::min(entry.bbox_min_z, node.z);
			entry.bbox_max_z = std::max(entry.bbox_max_z, node.z);
		}
		if (!entry.nodes.empty() && entry.nodes.front() != entry.nodes.back())
			entry.nodes.push_back(entry.nodes.front());
		index.entries_.push_back(std::move(entry));
	}
	return index;
}

std::optional<BridgeStyle> BridgeOutlineIndex::style_for_way(
		const ProcessedWay &way) const
{
	if (entries_.empty() || way.nodes.empty())
		return std::nullopt;
	const auto [cx, cz] = centroid_xz(way);
	for (const auto &entry : entries_) {
		if (cx < entry.bbox_min_x || cx > entry.bbox_max_x || cz < entry.bbox_min_z ||
				cz > entry.bbox_max_z)
			continue;
		if (!point_in_polygon(cx, cz, entry.nodes))
			continue;
		const auto style = resolve_bridge_style_from_pair(entry.structure, entry.bridge);
		if (style != BridgeStyle::Beam)
			return style;
	}
	return std::nullopt;
}

BridgeStyle resolve_bridge_style(const std::unordered_map<std::string, std::string> &tags)
{
	std::optional<std::string> structure;
	std::optional<std::string> bridge;
	if (const auto it = tags.find("bridge:structure"); it != tags.end())
		structure = it->second;
	if (const auto it = tags.find("bridge"); it != tags.end())
		bridge = it->second;
	return resolve_bridge_style_from_pair(structure, bridge);
}

BridgeStyle resolve_bridge_style_with_outline(
		const ProcessedWay &way, const BridgeOutlineIndex &outlines)
{
	const auto direct = resolve_bridge_style(way.tags);
	if (direct != BridgeStyle::Beam)
		return direct;
	return outlines.style_for_way(way).value_or(BridgeStyle::Beam);
}

void place_bridge_support_below_deck(WorldEditor &editor, BridgeStyle style, int set_x,
		int cell_y, int set_z, int centerline_ground_y, std::size_t tds,
		std::size_t total, bool use_absolute_y, bool is_centerline,
		bool is_pillar_position)
{
	if (style == BridgeStyle::Arch) {
		place_arch_spandrel_cell(editor, set_x, cell_y, set_z, centerline_ground_y, tds,
				total, use_absolute_y);
		if (is_centerline) {
			const auto [start, span] = arch_segment(tds, total);
			if (tds == start || tds + 1 == start + span)
				place_pillar(editor, set_x, cell_y, set_z, STONE_BRICKS, true);
		}
		return;
	}
	if (style == BridgeStyle::Boardwalk) {
		if (is_centerline && is_pillar_position)
			place_pillar(editor, set_x, cell_y, set_z, OAK_LOG, false);
		return;
	}
	if (is_centerline && is_pillar_position)
		place_pillar(editor, set_x, cell_y, set_z, STONE_BRICKS, true);
}

void place_pier_bent(WorldEditor &editor, const bridges::BridgeSurfaceMap &surface,
		BridgeStyle style, int x, int z, int deck_y, std::pair<float, float> perp,
		int half_width)
{
	const auto at = [x, z, perp](int offset) {
		return std::pair{static_cast<int>(std::lround(x + perp.first * offset)),
				static_cast<int>(std::lround(z + perp.second * offset))};
	};
	Block body = STONE_BRICKS;
	bool footing = true;
	std::vector<std::pair<int, int>> cells;
	if (style == BridgeStyle::Boardwalk) {
		body = OAK_LOG;
		footing = false;
		if (half_width >= 1) {
			cells.push_back(at(-half_width));
			cells.push_back(at(half_width));
		} else {
			cells.push_back(at(0));
		}
	} else if (style == BridgeStyle::Truss) {
		cells = connected_blocks::cross_cells(x, z, perp, -half_width, half_width);
	} else {
		const int edge = half_width - 1;
		if (half_width <= 2)
			cells.push_back(at(0));
		else if (half_width <= 4) {
			cells.push_back(at(-edge));
			cells.push_back(at(edge));
		} else {
			cells.push_back(at(-edge));
			cells.push_back(at(0));
			cells.push_back(at(edge));
		}
	}
	for (std::size_t i = 0; i < cells.size(); ++i) {
		if (std::find(cells.begin(), cells.begin() + i, cells[i]) != cells.begin() + i)
			continue;
		if (!surface.support_blocked(cells[i].first, cells[i].second, deck_y))
			place_pillar(editor, cells[i].first, deck_y, cells[i].second, body, footing);
	}
}

void decorate_bridge_above_deck(WorldEditor &editor, BridgeStyle style,
		const bridges::BridgeSurfaceMap &surface,
		const std::vector<BridgePathSample> &path, int block_range,
		bool start_is_boundary, bool end_is_boundary,
		const std::vector<std::pair<int, int>> *pylon_points)
{
	if (path.size() < 4)
		return;
	std::vector<std::size_t> pylons;
	if (pylon_points) {
		for (const auto &[px, pz] : *pylon_points) {
			std::size_t nearest = 0;
			long long distance = std::numeric_limits<long long>::max();
			for (std::size_t i = 0; i < path.size(); ++i) {
				const long long dx = static_cast<long long>(path[i].x) - px;
				const long long dz = static_cast<long long>(path[i].z) - pz;
				const long long d = dx * dx + dz * dz;
				if (d < distance) {
					distance = d;
					nearest = i;
				}
			}
			pylons.push_back(nearest);
		}
	} else {
		pylons = default_pylons(style, path.size(), start_is_boundary, end_is_boundary);
	}
	std::sort(pylons.begin(), pylons.end());
	pylons.erase(std::unique(pylons.begin(), pylons.end()), pylons.end());
	const OpenSides sides = open_sides(surface, path, block_range);
	switch (style) {
	case BridgeStyle::Truss:
		decorate_truss(
				editor, path, block_range, start_is_boundary, end_is_boundary, sides);
		break;
	case BridgeStyle::Suspension:
		decorate_suspension(editor, path, block_range, pylons, sides);
		break;
	case BridgeStyle::CableStayed:
		decorate_cable_stayed(editor, path, block_range, pylons, sides);
		break;
	case BridgeStyle::Covered:
		decorate_covered(
				editor, path, block_range, start_is_boundary, end_is_boundary, sides);
		break;
	default:
		break;
	}
}
}
