#include "bridge_styles.h"
#include "../structures/schem_decoder.h"
#include "bridge_modules.h"
#include <fstream>
#include <cmath>

#include <algorithm>
#include <cmath>
#include <limits>

#include "../bresenham.h"
#include "irrlichttypes.h"

namespace arnis::bridge_styles
{

bool sweep_bridge_schematic(WorldEditor &editor,
		const std::vector<BridgePathSample> &path, int block_range,
		const std::filesystem::path &asset_root)
{
	const auto selected = bridge_modules::pick_module_index(block_range, path.size());
	if (!selected)
		return false;
	const int index = static_cast<int>(*selected) + 1;
	int street_y = index == 1 ? 8 : (index == 2 ? 16 : (index == 3 ? 2 : 3));
	std::filesystem::path root = asset_root;
	if (root.empty())
		root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
			   "assets/structures";
	std::ifstream in(root / ("bridge_segment_" + std::to_string(index) + ".schem"),
			std::ios::binary);
	if (!in)
		return false;
	std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)), {});
	auto doc = structures::decode_sponge_schem(bytes);
	if (doc.width <= 0 || doc.length <= 0)
		return false;
	for (const auto &v : doc.voxels) {
		if (v.x < 0 || v.x >= doc.width || v.z < 0 || v.z >= doc.length || v.y < 0)
			continue;
		const auto block = structures::resolve_schem_block(v.block);
		if (block == block_definitions::AIR)
			continue;
		const auto &sample = path[static_cast<std::size_t>(v.x) % path.size()];
		const float offset =
				static_cast<float>(v.z) - static_cast<float>(doc.length - 1) * 0.5f;
		const int x = static_cast<int>(std::lround(sample.x + sample.perp_x * offset));
		const int z = static_cast<int>(std::lround(sample.z + sample.perp_z * offset));
		const int y = sample.y + v.y - street_y;
		if (editor.mg && editor.pos_ok(x, z))
			editor.set_block_absolute(block, x, y, z);
	}
	// Long vehicular modules carry pillar feet; extend solid shaft columns to terrain.
	if (index <= 2) {
		for (int vz = 0; vz < doc.length; ++vz) {
			int lowest = doc.height;
			Block shaft = block_definitions::STONE;
			for (const auto &v : doc.voxels)
				if (v.z == vz && v.y < lowest && v.y <= -3) {
					const Block candidate = structures::resolve_schem_block(v.block);
					if (candidate == block_definitions::STONE ||
							candidate == block_definitions::STONE_BRICKS ||
							candidate == block_definitions::ANDESITE ||
							candidate == block_definitions::COBBLESTONE ||
							candidate == block_definitions::SANDSTONE) {
						lowest = v.y;
						shaft = candidate;
					}
				}
			if (lowest == doc.height)
				continue;
			const auto &sample = path[0];
			const float offset =
					static_cast<float>(vz) - static_cast<float>(doc.length - 1) * 0.5f;
			const int bx =
					static_cast<int>(std::lround(sample.x + sample.perp_x * offset));
			const int bz =
					static_cast<int>(std::lround(sample.z + sample.perp_z * offset));
			const int bottom = sample.y + lowest - street_y;
			for (int yy = editor.get_ground_level(bx, bz);
					yy < bottom && yy < editor.get_ground_level(bx, bz) + 48; ++yy)
				editor.set_block_absolute(shaft, bx, yy, bz);
		}
	}
	return true;
}
namespace
{

constexpr std::size_t BEAM_PILLAR_INTERVAL = 8;
constexpr std::size_t BOARDWALK_POST_INTERVAL = 4;
constexpr std::size_t ARCH_SPAN = 20;
constexpr float ARCH_RISE_FRACTION = 0.85f;
constexpr int TRUSS_TOP_HEIGHT = 5;
constexpr std::size_t TRUSS_DIAGONAL_PERIOD = 8;
constexpr std::size_t TRUSS_POST_INTERVAL = 4;
constexpr std::size_t TRUSS_PORTAL_INTERVAL = 8;
constexpr int SUSPENSION_TOWER_BASE_HEIGHT = 8;
constexpr std::size_t SUSPENSION_TOWER_HEIGHT_DIVISOR = 6;
constexpr int SUSPENSION_TOWER_MAX_HEIGHT = 32;
constexpr std::size_t SUSPENSION_HANGER_INTERVAL = 4;
constexpr float SUSPENSION_TOWER_INSET_FRAC = 0.12f;
constexpr std::size_t SUSPENSION_MIN_LENGTH = 18;
constexpr std::size_t SUSPENSION_INTER_PYLON_SPACING = 100;
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
	if (deck_y <= ground_y)
		return;
	for (int y = ground_y + 1; y < deck_y; ++y)
		editor.set_block_absolute(body, x, y, z, std::nullopt, std::nullopt);
	if (!with_base)
		return;
	for (int bx = -1; bx <= 1; ++bx) {
		for (int bz = -1; bz <= 1; ++bz)
			editor.set_block_absolute(
					body, x + bx, ground_y, z + bz, std::nullopt, std::nullopt);
	}
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

int cable_stayed_tower_height(std::size_t total)
{
	const int extra = static_cast<int>(total / CABLE_STAYED_TOWER_HEIGHT_DIVISOR);
	return std::min(
			CABLE_STAYED_TOWER_BASE_HEIGHT + extra, CABLE_STAYED_TOWER_MAX_HEIGHT);
}

void place_pylon(WorldEditor &editor, int x, int z, int deck_y, int height)
{
	const int ground_y = editor.get_ground_level(x, z);
	const int base_y = std::min(ground_y, deck_y);
	const int top_y = deck_y + height;
	for (int y = base_y + 1; y <= top_y; ++y)
		editor.set_block_absolute(SMOOTH_STONE, x, y, z, std::nullopt, std::nullopt);
}

void place_pylon_crossbeam(WorldEditor &editor, std::pair<int, int> left,
		std::pair<int, int> right, int top_y)
{
	for (const auto &[cx, cy, cz] : bresenham_line(
				 left.first, top_y, left.second, right.first, top_y, right.second))
		editor.set_block_absolute(SMOOTH_STONE, cx, cy, cz, std::nullopt, std::nullopt);
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
		int block_range, bool start_is_boundary, bool end_is_boundary)
{
	const std::size_t last = path.size() - 1;
	for (std::size_t tds = 0; tds < path.size(); ++tds) {
		const auto &s = path[tds];
		if ((tds == 0 && start_is_boundary) || (tds == last && end_is_boundary))
			continue;
		const auto [left, right] =
				side_offsets(s.x, s.z, s.perp_x, s.perp_z, block_range);
		const int top_y = s.y + 1 + TRUSS_TOP_HEIGHT;
		editor.set_block_absolute(
				IRON_BLOCK, left.first, s.y + 1, left.second, std::nullopt, std::nullopt);
		editor.set_block_absolute(IRON_BLOCK, right.first, s.y + 1, right.second,
				std::nullopt, std::nullopt);
		editor.set_block_absolute(
				IRON_BLOCK, left.first, top_y, left.second, std::nullopt, std::nullopt);
		editor.set_block_absolute(
				IRON_BLOCK, right.first, top_y, right.second, std::nullopt, std::nullopt);
		if (tds % TRUSS_POST_INTERVAL == 0) {
			for (int h = 1; h <= TRUSS_TOP_HEIGHT; ++h) {
				editor.set_block_absolute(IRON_BLOCK, left.first, s.y + 1 + h,
						left.second, std::nullopt, std::nullopt);
				editor.set_block_absolute(IRON_BLOCK, right.first, s.y + 1 + h,
						right.second, std::nullopt, std::nullopt);
			}
		}
		const std::size_t p = tds % TRUSS_DIAGONAL_PERIOD;
		const std::size_t half = TRUSS_DIAGONAL_PERIOD / 2;
		const int dh = static_cast<int>(p <= half ? p : TRUSS_DIAGONAL_PERIOD - p);
		const int diag_y = s.y + 1 + std::min(dh, TRUSS_TOP_HEIGHT);
		editor.set_block_absolute(
				IRON_BLOCK, left.first, diag_y, left.second, std::nullopt, std::nullopt);
		editor.set_block_absolute(IRON_BLOCK, right.first, diag_y, right.second,
				std::nullopt, std::nullopt);
		if (tds % TRUSS_PORTAL_INTERVAL == 0) {
			for (const auto &[bx, by, bz] : bresenham_line(left.first, top_y, left.second,
						 right.first, top_y, right.second))
				editor.set_block_absolute(
						IRON_BLOCK, bx, by, bz, std::nullopt, std::nullopt);
		}
	}
}

void decorate_suspension(WorldEditor &editor, const std::vector<BridgePathSample> &path,
		int block_range, bool start_is_boundary, bool end_is_boundary)
{
	if (!start_is_boundary || !end_is_boundary)
		return;
	const std::size_t total = path.size();
	if (total < SUSPENSION_MIN_LENGTH)
		return;
	const std::size_t last_idx = total - 1;
	const std::size_t inset =
			std::max<std::size_t>(static_cast<std::size_t>(static_cast<float>(total) *
														   SUSPENSION_TOWER_INSET_FRAC),
					2);
	if (inset * 2 + 2 > total)
		return;
	const int height = suspension_tower_height(total);
	const std::size_t n_pylons = suspension_pylon_count(total);
	const std::size_t first = inset;
	const std::size_t last = last_idx - inset;
	std::vector<std::size_t> pylons;
	for (std::size_t i = 0; i < n_pylons; ++i)
		pylons.push_back(
				first + (last - first) * i / std::max<std::size_t>(1, n_pylons - 1));

	for (const auto p : pylons) {
		const auto &s = path[p];
		const auto [left, right] =
				side_offsets(s.x, s.z, s.perp_x, s.perp_z, block_range);
		place_pylon(editor, left.first, left.second, s.y, height);
		place_pylon(editor, right.first, right.second, s.y, height);
		place_pylon_crossbeam(editor, left, right, s.y + height);
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
			editor.set_block_absolute(
					chain, left.first, cable_y, left.second, std::nullopt, std::nullopt);
			editor.set_block_absolute(chain, right.first, cable_y, right.second,
					std::nullopt, std::nullopt);
			if ((tds - a) % SUSPENSION_HANGER_INTERVAL == 0 && tds != a && tds != b) {
				for (int hy = s.y + 2; hy < cable_y; ++hy) {
					editor.set_block_absolute(IRON_BARS, left.first, hy, left.second,
							std::nullopt, std::nullopt);
					editor.set_block_absolute(IRON_BARS, right.first, hy, right.second,
							std::nullopt, std::nullopt);
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
	draw_cable(editor, lf.first, path[first_p].y + height, lf.second, ls.first,
			path[0].y + 1, ls.second);
	draw_cable(editor, rf.first, path[first_p].y + height, rf.second, rs.first,
			path[0].y + 1, rs.second);

	const auto [ll, rl] = side_offsets(path[last_p].x, path[last_p].z,
			path[last_p].perp_x, path[last_p].perp_z, block_range);
	const auto [le, re] = side_offsets(path[last_idx].x, path[last_idx].z,
			path[last_idx].perp_x, path[last_idx].perp_z, block_range);
	draw_cable(editor, ll.first, path[last_p].y + height, ll.second, le.first,
			path[last_idx].y + 1, le.second);
	draw_cable(editor, rl.first, path[last_p].y + height, rl.second, re.first,
			path[last_idx].y + 1, re.second);
}

void decorate_cable_stayed(
		WorldEditor &editor, const std::vector<BridgePathSample> &path, int block_range)
{
	const std::size_t total = path.size();
	if (total < CABLE_STAYED_MIN_LENGTH)
		return;
	const int height = cable_stayed_tower_height(total);
	const std::vector<std::size_t> pylons =
			total >= CABLE_STAYED_TWIN_PYLON_LENGTH
					? std::vector<std::size_t>{total / 3, (2 * total) / 3}
					: std::vector<std::size_t>{total / 2};
	const std::size_t split = total / 2;
	const std::size_t last_idx = total - 1;

	for (std::size_t idx = 0; idx < pylons.size(); ++idx) {
		const std::size_t t_tds = pylons[idx];
		const auto &tower = path[t_tds];
		const auto [left_t, right_t] =
				side_offsets(tower.x, tower.z, tower.perp_x, tower.perp_z, block_range);
		const int top_y = tower.y + height;
		place_pylon(editor, left_t.first, left_t.second, tower.y, height);
		place_pylon(editor, right_t.first, right_t.second, tower.y, height);
		place_pylon_crossbeam(editor, left_t, right_t, top_y);

		const std::size_t anchor_lo = pylons.size() == 1 ? 0 : (idx == 0 ? 0 : split);
		const std::size_t anchor_hi =
				pylons.size() == 1 ? total : (idx == 0 ? split : total);
		for (std::size_t tds = anchor_lo + CABLE_STAYED_ANCHOR_INTERVAL; tds < anchor_hi;
				tds += CABLE_STAYED_ANCHOR_INTERVAL) {
			const std::size_t gap = tds > t_tds ? tds - t_tds : t_tds - tds;
			if (gap < CABLE_STAYED_MIN_GAP || tds == 0 || tds == last_idx)
				continue;
			const auto &anchor = path[tds];
			const auto [left_a, right_a] = side_offsets(
					anchor.x, anchor.z, anchor.perp_x, anchor.perp_z, block_range);
			draw_cable(editor, left_t.first, top_y, left_t.second, left_a.first,
					anchor.y + 1, left_a.second);
			draw_cable(editor, right_t.first, top_y, right_t.second, right_a.first,
					anchor.y + 1, right_a.second);
		}
	}
}

void decorate_covered(WorldEditor &editor, const std::vector<BridgePathSample> &path,
		int block_range, bool start_is_boundary, bool end_is_boundary)
{
	if (path.size() < 4)
		return;
	const std::size_t last = path.size() - 1;
	for (std::size_t tds = 0; tds < path.size(); ++tds) {
		const auto &s = path[tds];
		if (start_is_boundary && tds < COVERED_END_CLEAR)
			continue;
		if (end_is_boundary && tds + COVERED_END_CLEAR > last)
			continue;
		const auto [left, right] =
				side_offsets(s.x, s.z, s.perp_x, s.perp_z, block_range);
		for (int h = 1; h <= COVERED_WALL_HEIGHT; ++h) {
			const Block b = h == 2 && tds % COVERED_WINDOW_INTERVAL == 0
									? GLASS
									: DARK_OAK_PLANKS;
			editor.set_block_absolute(
					b, left.first, s.y + h, left.second, std::nullopt, std::nullopt);
			editor.set_block_absolute(
					b, right.first, s.y + h, right.second, std::nullopt, std::nullopt);
		}
		const int roof_y = s.y + COVERED_WALL_HEIGHT + 1;
		const int extent = block_range + 1;
		for (int offset = -extent; offset <= extent; ++offset) {
			const int rx = static_cast<int>(std::round(
					static_cast<float>(s.x) + s.perp_x * static_cast<float>(offset)));
			const int rz = static_cast<int>(std::round(
					static_cast<float>(s.z) + s.perp_z * static_cast<float>(offset)));
			editor.set_block_absolute(
					DARK_OAK_PLANKS, rx, roof_y, rz, std::nullopt, std::nullopt);
		}
	}
}

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
	return style == BridgeStyle::Boardwalk ? BOARDWALK_POST_INTERVAL
										   : BEAM_PILLAR_INTERVAL;
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

void decorate_bridge_above_deck(WorldEditor &editor, BridgeStyle style,
		const std::vector<BridgePathSample> &path, int block_range,
		bool start_is_boundary, bool end_is_boundary)
{
	if (path.size() < 4)
		return;
	switch (style) {
	case BridgeStyle::Truss:
		decorate_truss(editor, path, block_range, start_is_boundary, end_is_boundary);
		break;
	case BridgeStyle::Suspension:
		decorate_suspension(
				editor, path, block_range, start_is_boundary, end_is_boundary);
		break;
	case BridgeStyle::CableStayed:
		decorate_cable_stayed(editor, path, block_range);
		break;
	case BridgeStyle::Covered:
		decorate_covered(editor, path, block_range, start_is_boundary, end_is_boundary);
		break;
	default:
		break;
	}
}

}
