#include "tree.h"
#include "../trees/mapped.h"
#include "../deterministic_rng.h"
#include "../land_cover/land_cover.h"
#include "../trees/region.h"
#include <algorithm>
#include <initializer_list>
#include <limits>

namespace arnis
{

// Rust's leaf_gap_at/leaf_hash helpers: keep canopy gaps deterministic and
// independent of the tree generation traversal order.
static std::uint64_t leaf_hash(int x, int y, int z)
{
	return std::uint64_t(std::int64_t(x) * 73856093LL) ^
		   std::uint64_t(std::int64_t(y) * 19349663LL) ^
		   std::uint64_t(std::int64_t(z) * 83492791LL);
}

static bool leaf_gap_at(std::uint64_t hash)
{
	return hash % 100 < 4;
}

// Additional leaves fill patterns for new tree types
static const std::array<std::pair<Coord, Coord>, 5> DARK_OAK_LEAVES_FILL = {{
		{{-1, 3, 0}, {-1, 6, 0}},
		{{1, 3, 0}, {1, 6, 0}},
		{{0, 3, -1}, {0, 6, -1}},
		{{0, 3, 1}, {0, 6, 1}},
		{{0, 6, 0}, {0, 7, 0}},
}};

static const std::array<std::pair<Coord, Coord>, 5> JUNGLE_LEAVES_FILL = {{
		{{-1, 7, 0}, {-1, 11, 0}},
		{{1, 7, 0}, {1, 11, 0}},
		{{0, 7, -1}, {0, 11, -1}},
		{{0, 7, 1}, {0, 11, 1}},
		{{0, 11, 0}, {0, 12, 0}},
}};

static const std::array<std::pair<Coord, Coord>, 5> ACACIA_LEAVES_FILL = {{
		{{-1, 5, 0}, {-1, 8, 0}},
		{{1, 5, 0}, {1, 8, 0}},
		{{0, 5, -1}, {0, 8, -1}},
		{{0, 5, 1}, {0, 8, 1}},
		{{0, 8, 0}, {0, 9, 0}},
}};

static const std::array<std::pair<Coord, Coord>, 5> CHERRY_LEAVES_FILL = {{
		{{-1, 4, 0}, {-1, 9, 0}},
		{{1, 4, 0}, {1, 9, 0}},
		{{0, 4, -1}, {0, 9, -1}},
		{{0, 4, 1}, {0, 9, 1}},
		{{0, 9, 0}, {0, 10, 0}},
}};

static const std::array<std::pair<Coord, Coord>, 5> TALL_OAK_LEAVES_FILL = {{
		{{-1, 8, 0}, {-1, 12, 0}},
		{{1, 8, 0}, {1, 12, 0}},
		{{0, 8, -1}, {0, 12, -1}},
		{{0, 8, 1}, {0, 12, 1}},
		{{0, 12, 0}, {0, 13, 0}},
}};

static const std::array<std::pair<Coord, Coord>, 5> PINE_LEAVES_FILL = {{
		{{-1, 5, 0}, {-1, 12, 0}},
		{{0, 5, -1}, {0, 12, -1}},
		{{1, 5, 0}, {1, 12, 0}},
		{{0, 5, 1}, {0, 12, 1}},
		{{0, 13, 0}, {0, 13, 0}},
}};

template <int SideLow, int SideHigh, int CrownLow, int CrownHigh>
constexpr std::array<std::pair<Coord, Coord>, 5> cross_fill()
{
	return {{{{-1, SideLow, 0}, {-1, SideHigh, 0}}, {{1, SideLow, 0}, {1, SideHigh, 0}},
			{{0, SideLow, -1}, {0, SideHigh, -1}}, {{0, SideLow, 1}, {0, SideHigh, 1}},
			{{0, CrownLow, 0}, {0, CrownHigh, 0}}}};
}

static constexpr auto OAK_TALL_SLIM_FILL = cross_fill<6, 11, 11, 12>();
static constexpr auto OAK_BUSHY_FILL = cross_fill<3, 7, 7, 8>();
static constexpr auto OAK_COMPACT_FILL = cross_fill<2, 5, 5, 6>();
static constexpr auto SPRUCE_TOWERING_FILL = cross_fill<4, 13, 14, 14>();
static constexpr auto SPRUCE_SQUAT_FILL = cross_fill<2, 7, 8, 8>();
static constexpr auto BIRCH_TALL_FILL = cross_fill<5, 10, 10, 11>();
static constexpr auto BIRCH_CLUSTER_FILL = cross_fill<3, 5, 5, 6>();
static constexpr auto DARK_OAK_TALL_BUSHY_FILL = cross_fill<4, 9, 9, 10>();
static constexpr auto DARK_OAK_STUNTED_FILL = cross_fill<2, 4, 4, 5>();
static constexpr auto JUNGLE_BROAD_FILL = cross_fill<8, 12, 12, 13>();
static constexpr auto ACACIA_TALL_FILL = cross_fill<7, 10, 10, 10>();
static constexpr auto CHERRY_WEEPING_FILL = cross_fill<3, 8, 8, 9>();
static constexpr auto TALL_OAK_GIANT_FILL = cross_fill<9, 14, 14, 15>();
static constexpr auto PINE_TALL_FILL = cross_fill<6, 15, 16, 16>();

// Rust's wetland and understorey types.  Luanti has no separate mangrove or
// willow blocks in the default game, so their geometry is retained while the
// closest native log/leaf materials are used.
static const std::array<std::pair<Coord, Coord>, 3> BUSH_LEAVES_FILL = {{
		{{-1, 0, -1}, {1, 1, 1}},
		{{-1, 2, 0}, {1, 2, 0}},
		{{0, 2, -1}, {0, 2, 1}},
}};
static const std::array<std::pair<Coord, Coord>, 5> WILLOW_LEAVES_FILL = {{
		{{-1, 4, 0}, {-1, 7, 0}},
		{{1, 4, 0}, {1, 7, 0}},
		{{0, 4, -1}, {0, 7, -1}},
		{{0, 4, 1}, {0, 7, 1}},
		{{0, 7, 0}, {0, 8, 0}},
}};
static const std::array<std::pair<Coord, Coord>, 5> MANGROVE_LEAVES_FILL = {{
		{{-1, 5, 0}, {-1, 10, 0}},
		{{1, 5, 0}, {1, 10, 0}},
		{{0, 5, -1}, {0, 10, -1}},
		{{0, 5, 1}, {0, 10, 1}},
		{{0, 10, 0}, {0, 11, 0}},
}};

constexpr int MAX_CANOPY_RADIUS = 3;

Tree Tree::get_tree(TreeType kind)
{
	switch (kind) {
	case TreeType::Oak: {
		Tree t;
		t.log_block = OAK_LOG;
		t.log_height = 8;
		t.leaves_block = OAK_LEAVES;
		t.leaves_fill = std::span<const std::pair<Coord, Coord>>(OAK_LEAVES_FILL);
		t.round_ranges[0].reserve(6);
		for (int v = 8; v >= 3; --v)
			t.round_ranges[0].push_back(v);
		t.round_ranges[1].reserve(4);
		for (int v = 7; v >= 4; --v)
			t.round_ranges[1].push_back(v);
		t.round_ranges[2].reserve(2);
		for (int v = 6; v >= 5; --v)
			t.round_ranges[2].push_back(v);
		t.branch_chance = .30f;
		return t;
	}

	case TreeType::Spruce: {
		Tree t;
		t.log_block = SPRUCE_LOG;
		t.log_height = 9;
		t.leaves_block = SPRUCE_LEAVES;
		t.leaves_fill = std::span<const std::pair<Coord, Coord>>(SPRUCE_LEAVES_FILL);
		t.round_ranges[0] = std::vector<int>{9, 7, 6, 4, 3};
		t.round_ranges[1] = std::vector<int>{6, 3};
		t.round_ranges[2] = std::vector<int>{};
		return t;
	}

	case TreeType::Birch: {
		Tree t;
		t.log_block = BIRCH_LOG;
		t.log_height = 6;
		t.leaves_block = BIRCH_LEAVES;
		t.leaves_fill = std::span<const std::pair<Coord, Coord>>(BIRCH_LEAVES_FILL);
		t.round_ranges[0].reserve(5);
		for (int v = 6; v >= 2; --v)
			t.round_ranges[0].push_back(v);
		t.round_ranges[1].reserve(3);
		for (int v = 2; v <= 4; ++v)
			t.round_ranges[1].push_back(v);
		t.branch_chance = .20f;
		t.round_ranges[2] = std::vector<int>{};
		return t;
	}

	case TreeType::DarkOak: {
		Tree t;
		t.log_block = DARK_OAK_LOG;
		t.log_height = 5;
		t.leaves_block = DARK_OAK_LEAVES;
		t.leaves_fill = std::span<const std::pair<Coord, Coord>>(DARK_OAK_LEAVES_FILL);
		// All 3 round patterns used for maximum width
		t.round_ranges[0].reserve(4);
		for (int v = 6; v >= 3; --v)
			t.round_ranges[0].push_back(v);
		t.round_ranges[1].reserve(3);
		for (int v = 5; v >= 3; --v)
			t.round_ranges[1].push_back(v);
		t.round_ranges[2].reserve(2);
		for (int v = 5; v >= 4; --v)
			t.round_ranges[2].push_back(v);
		t.branch_chance = .40f;
		return t;
	}

	case TreeType::Jungle: {
		Tree t;
		t.log_block = JUNGLE_LOG;
		t.log_height = 10;
		t.leaves_block = JUNGLE_LEAVES;
		t.leaves_fill = std::span<const std::pair<Coord, Coord>>(JUNGLE_LEAVES_FILL);
		// Canopy only near the top of the tree
		t.round_ranges[0].reserve(5);
		for (int v = 11; v >= 7; --v)
			t.round_ranges[0].push_back(v);
		t.round_ranges[1].reserve(3);
		for (int v = 10; v >= 8; --v)
			t.round_ranges[1].push_back(v);
		t.round_ranges[2] = std::vector<int>{};
		t.branch_chance = .50f;
		return t;
	}

	case TreeType::Acacia: {
		Tree t;
		t.log_block = ACACIA_LOG;
		t.log_height = 6;
		t.leaves_block = ACACIA_LEAVES;
		t.leaves_fill = std::span<const std::pair<Coord, Coord>>(ACACIA_LEAVES_FILL);
		// Inner rounds reach higher → gentle dome, outer stays low → wide brim
		t.round_ranges[0].reserve(4);
		for (int v = 8; v >= 5; --v)
			t.round_ranges[0].push_back(v);
		t.round_ranges[1].reserve(3);
		for (int v = 7; v >= 5; --v)
			t.round_ranges[1].push_back(v);
		t.round_ranges[2].reserve(2);
		for (int v = 7; v >= 6; --v)
			t.round_ranges[2].push_back(v);
		t.branch_chance = .35f;
		return t;
	}

	case TreeType::Cherry: {
		Tree t;
		t.log_block = CHERRY_LOG;
		t.log_height = 7;
		t.leaves_block = CHERRY_LEAVES;
		t.leaves_fill = std::span<const std::pair<Coord, Coord>>(CHERRY_LEAVES_FILL);
		for (int v = 9; v >= 4; --v)
			t.round_ranges[0].push_back(v);
		for (int v = 8; v >= 5; --v)
			t.round_ranges[1].push_back(v);
		for (int v = 7; v >= 6; --v)
			t.round_ranges[2].push_back(v);
		t.branch_chance = .30f;
		return t;
	}

	case TreeType::TallOak: {
		Tree t;
		t.log_block = OAK_LOG;
		t.log_height = 11;
		t.leaves_block = OAK_LEAVES;
		t.leaves_fill = std::span<const std::pair<Coord, Coord>>(TALL_OAK_LEAVES_FILL);
		for (int v = 12; v >= 8; --v)
			t.round_ranges[0].push_back(v);
		for (int v = 11; v >= 9; --v)
			t.round_ranges[1].push_back(v);
		t.round_ranges[2] = std::vector<int>{10};
		t.branch_chance = .40f;
		return t;
	}

	case TreeType::Pine: {
		Tree t;
		t.log_block = SPRUCE_LOG;
		t.log_height = 12;
		t.leaves_block = SPRUCE_LEAVES;
		t.leaves_fill = std::span<const std::pair<Coord, Coord>>(PINE_LEAVES_FILL);
		t.round_ranges[0] = std::vector<int>{11, 9, 7, 5};
		t.round_ranges[1] = std::vector<int>{8, 5};
		t.round_ranges[2] = std::vector<int>{};
		return t;
	}
	case TreeType::Bush:
	case TreeType::AzaleaBush: {
		Tree t;
		t.log_block = OAK_LOG;
		t.log_height = 0;
		t.leaves_block = kind == TreeType::AzaleaBush ? CHERRY_LEAVES : OAK_LEAVES;
		t.leaves_fill = std::span<const std::pair<Coord, Coord>>(BUSH_LEAVES_FILL);
		return t;
	}
	case TreeType::Willow: {
		Tree t;
		t.log_block = OAK_LOG;
		t.log_height = 5;
		t.leaves_block = OAK_LEAVES;
		t.leaves_fill = std::span<const std::pair<Coord, Coord>>(WILLOW_LEAVES_FILL);
		t.round_ranges[0] = {7, 6, 5};
		t.round_ranges[1] = {6};
		t.drooping = true;
		return t;
	}
	case TreeType::FloweringOak: {
		Tree t = get_tree(TreeType::Oak);
		t.accent_block = CHERRY_LEAVES;
		t.accent_chance = 18;
		t.branch_chance = .40f;
		return t;
	}
	case TreeType::Mangrove: {
		Tree t;
		t.log_block = MANGROVE_LOG;
		t.log_height = 8;
		t.leaves_block = MANGROVE_LEAVES;
		t.leaves_fill = std::span<const std::pair<Coord, Coord>>(MANGROVE_LEAVES_FILL);
		t.round_ranges[0] = {10, 9, 8, 7, 6, 5};
		t.round_ranges[1] = {9, 8, 7, 6};
		t.round_ranges[2] = {8, 7};
		t.branch_chance = .55f;
		return t;
	}
	}
	// fallback (should not happen)
	return Tree{};
}

Tree Tree::get_tree_variant(TreeType kind, std::uint32_t variant_idx)
{
	Tree t = get_tree(kind);
	const auto set_shape =
			[&](int height, const auto &fill, std::initializer_list<int> first,
					std::initializer_list<int> second, std::initializer_list<int> third,
					float branch, bool drooping = false) {
				t.log_height = height;
				t.leaves_fill = std::span<const std::pair<Coord, Coord>>(fill);
				t.round_ranges[0] = first;
				t.round_ranges[1] = second;
				t.round_ranges[2] = third;
				t.branch_chance = branch;
				t.drooping = drooping;
			};
	switch (kind) {
	case TreeType::Oak:
		switch (variant_idx % 5U) {
		case 0:
			set_shape(8, OAK_LEAVES_FILL, {8, 7, 6, 5, 4, 3}, {7, 6, 5, 4}, {6, 5}, .30f);
			break;
		case 1:
			set_shape(10, OAK_TALL_SLIM_FILL, {11, 10, 9, 8, 7}, {10, 9, 8}, {}, .40f);
			break;
		case 2:
			set_shape(6, OAK_BUSHY_FILL, {7, 6, 5, 4, 3}, {6, 5, 4}, {5, 4}, .20f);
			break;
		case 3:
			set_shape(5, OAK_COMPACT_FILL, {5, 4, 3, 2}, {4, 3}, {}, 0.0f);
			break;
		case 4:
			set_shape(8, OAK_LEAVES_FILL, {8, 7, 6, 5, 4, 3}, {7, 6, 5, 4}, {6, 5}, 1.0f);
			break;
		default:
			break;
		}
		break;
	case TreeType::Spruce:
		switch (variant_idx % 3U) {
		case 0:
			set_shape(9, SPRUCE_LEAVES_FILL, {9, 7, 6, 4, 3}, {6, 3}, {}, 0.0f);
			break;
		case 1:
			set_shape(12, SPRUCE_TOWERING_FILL, {12, 10, 8, 6, 4}, {9, 6, 4}, {}, 0.0f);
			break;
		default:
			set_shape(6, SPRUCE_SQUAT_FILL, {6, 4, 3}, {4, 2}, {3}, 0.0f);
			break;
		}
		break;
	case TreeType::Birch:
		switch (variant_idx % 3U) {
		case 0:
			set_shape(6, BIRCH_LEAVES_FILL, {6, 5, 4, 3, 2}, {2, 3, 4}, {}, .20f);
			break;
		case 1:
			set_shape(9, BIRCH_TALL_FILL, {9, 8, 7, 6, 5}, {8, 7, 6}, {}, .25f);
			break;
		default:
			set_shape(4, BIRCH_CLUSTER_FILL, {4, 3, 2}, {3}, {}, 0.0f);
			break;
		}
		break;
	case TreeType::DarkOak:
		switch (variant_idx % 3U) {
		case 0:
			set_shape(5, DARK_OAK_LEAVES_FILL, {6, 5, 4, 3}, {5, 4, 3}, {5, 4}, .40f);
			break;
		case 1:
			set_shape(8, DARK_OAK_TALL_BUSHY_FILL, {9, 8, 7, 6, 5, 4}, {8, 7, 6, 5},
					{7, 6}, .50f);
			break;
		default:
			set_shape(3, DARK_OAK_STUNTED_FILL, {3, 2}, {2}, {}, 0.0f);
			break;
		}
		break;
	case TreeType::Jungle:
		if (variant_idx & 1U)
			set_shape(11, JUNGLE_BROAD_FILL, {12, 11, 10, 9, 8}, {11, 10, 9}, {10}, .60f);
		else
			set_shape(10, JUNGLE_LEAVES_FILL, {11, 10, 9, 8, 7}, {10, 9, 8}, {}, .50f);
		break;
	case TreeType::Acacia:
		if (variant_idx & 1U)
			set_shape(8, ACACIA_TALL_FILL, {10, 9, 8, 7}, {9, 8}, {9}, .45f);
		else
			set_shape(6, ACACIA_LEAVES_FILL, {8, 7, 6, 5}, {7, 6, 5}, {7, 6}, .35f);
		break;
	case TreeType::Cherry:
		if (variant_idx & 1U)
			set_shape(6, CHERRY_WEEPING_FILL, {8, 7, 6, 5, 4, 3}, {7, 6, 5, 4}, {6, 5},
					0.0f, true);
		else
			set_shape(7, CHERRY_LEAVES_FILL, {9, 8, 7, 6, 5, 4}, {8, 7, 6, 5}, {7, 6},
					.30f);
		break;
	case TreeType::TallOak:
		if (variant_idx & 1U)
			set_shape(13, TALL_OAK_GIANT_FILL, {14, 13, 12, 11, 10, 9}, {13, 12, 11, 10},
					{12, 11}, .60f);
		else
			set_shape(11, TALL_OAK_LEAVES_FILL, {12, 11, 10, 9, 8}, {11, 10, 9}, {10},
					.40f);
		break;
	case TreeType::Pine:
		if (variant_idx & 1U)
			set_shape(15, PINE_TALL_FILL, {14, 12, 10, 8, 6}, {11, 7}, {}, 0.0f);
		else
			set_shape(12, PINE_LEAVES_FILL, {11, 9, 7, 5}, {8, 5}, {}, 0.0f);
		break;
	case TreeType::Willow:
		set_shape(5, WILLOW_LEAVES_FILL, {6, 5, 4}, {5, 4}, {5}, 0.0f, true);
		break;
	case TreeType::Mangrove:
		set_shape(
				8, MANGROVE_LEAVES_FILL, {10, 9, 8, 7, 6, 5}, {9, 8, 7, 6}, {8, 7}, .55f);
		break;
	default:
		break;
	}
	return t;
}

namespace
{
trees::Habitat habitat_for_tree_type(TreeType type)
{
	switch (type) {
	case TreeType::Spruce:
	case TreeType::Pine:
		return trees::Habitat::Conifer;
	case TreeType::Willow:
	case TreeType::Mangrove:
		return trees::Habitat::Wet;
	case TreeType::Acacia:
		return trees::Habitat::Dry;
	default:
		return trees::Habitat::Lowland;
	}
}

TreeType selected_tree_type(int x, int z)
{
	auto rng = coord_rng(x, z, 0);
	// Preserve Rust's weighted species selection.  The coordinate-seeded RNG
	// keeps results stable across region/tile boundaries.
	const int pick = 1 + static_cast<int>(rng.uniform(100));

	TreeType chosen = TreeType::Mangrove;
	if (pick <= 20)
		chosen = TreeType::Oak;
	else if (pick <= 32)
		chosen = TreeType::Spruce;
	else if (pick <= 44)
		chosen = TreeType::Birch;
	else if (pick <= 50)
		chosen = TreeType::DarkOak;
	else if (pick <= 56)
		chosen = TreeType::Jungle;
	else if (pick <= 62)
		chosen = TreeType::Acacia;
	else if (pick <= 64)
		chosen = TreeType::Cherry;
	else if (pick <= 70)
		chosen = TreeType::TallOak;
	else if (pick <= 77)
		chosen = TreeType::Pine;
	else if (pick <= 84)
		chosen = TreeType::Bush;
	else if (pick <= 88)
		chosen = TreeType::AzaleaBush;
	else if (pick <= 92)
		chosen = TreeType::Willow;
	else if (pick <= 98)
		chosen = TreeType::FloweringOak;

	return chosen;
}
}

void Tree::create(WorldEditor &editor, const Coord &pos,
		const BuildingFootprintBitmap *building_footprints,
		const bridges::BridgeSurfaceMap *bridge_surface)
{
	create_of_type(editor, pos, selected_tree_type(pos.x, pos.z), building_footprints,
			bridge_surface, false, std::nullopt, false, true, false);
}

void Tree::create_from_canopy(WorldEditor &editor, const Coord &pos,
		const BuildingFootprintBitmap *building_footprints,
		const bridges::BridgeSurfaceMap *bridge_surface)
{
	create_of_type(editor, pos, selected_tree_type(pos.x, pos.z), building_footprints,
			bridge_surface, false, std::nullopt, false, true, true);
}

void Tree::create_mapped(WorldEditor &editor, const Coord &pos,
		const trees::mapped::MappedTree &mapped,
		const BuildingFootprintBitmap *building_footprints,
		const bridges::BridgeSurfaceMap *bridge_surface)
{
	// Keep the mapped-tree selector request in one place, matching Rust's
	// create_mapped path. Explicit OSM placement is allowed on paving, but the
	// region selector still rejects water, footprints and bridge decks.
	if (building_footprints && building_footprints->contains(pos.x, pos.z))
		return;
	if (!editor.owns(pos.x, pos.z) ||
			(bridge_surface && bridge_surface->contains(pos.x, pos.z)) ||
			editor.check_for_block(
					pos.x, 0, pos.z, std::optional<std::vector<Block>>({WATER})))
		return;

	trees::MappedRequest request;
	request.habitat = habitat_for_tree_type(mapped.kind);
	if (!mapped.genus.empty())
		request.genus = mapped.genus;
	request.conifer = mapped.conifer;
	if (mapped.height_m > 0.0)
		request.want_size = trees::size_for_height(
				static_cast<int>(std::lround(mapped.height_m * editor.scale())));
	if (editor.ground)
		request.eco = editor.ground->ecoregion_at(editor.ground_point(pos.x, pos.z));
	if (request.eco) {
		const int radius =
				std::clamp(static_cast<int>(std::lround(12.0 * editor.scale())), 2, 12);
		for (const auto &[dx, dz] : std::array<std::pair<int, int>, 5>{
					 {{0, 0}, {radius, 0}, {-radius, 0}, {0, radius}, {0, -radius}}})
			if (editor.ground->cover_class(editor.ground_point(pos.x + dx, pos.z + dz)) ==
					land_cover::LC_BEACH) {
				request.beach = true;
				break;
			}
	}
	if (editor.place_mapped_regional_tree(pos.x,
					  editor.get_absolute_y(pos.x, pos.y, pos.z), pos.z, 0, request)
					.has_value())
		return;

	create_of_type(editor, pos, mapped.kind, building_footprints, bridge_surface, true,
			mapped.height_m > 0.0 ? std::optional<double>(mapped.height_m) : std::nullopt,
			false, false, true);
}

std::vector<Block> Tree::get_building_wall_blocks()
{
	return std::vector<Block>{
			BLACKSTONE,
			BLACK_TERRACOTTA,
			BRICK,
			BROWN_CONCRETE,
			BROWN_TERRACOTTA,
			DEEPSLATE_BRICKS,
			END_STONE_BRICKS,
			GRAY_CONCRETE,
			GRAY_TERRACOTTA,
			LIGHT_BLUE_TERRACOTTA,
			LIGHT_GRAY_CONCRETE,
			MUD_BRICKS,
			NETHER_BRICK,
			NETHERITE_BLOCK,
			POLISHED_ANDESITE,
			POLISHED_BLACKSTONE,
			POLISHED_BLACKSTONE_BRICKS,
			POLISHED_DEEPSLATE,
			POLISHED_GRANITE,
			QUARTZ_BLOCK,
			QUARTZ_BRICKS,
			SANDSTONE,
			SMOOTH_SANDSTONE,
			SMOOTH_STONE,
			STONE_BRICKS,
			WHITE_CONCRETE,
			WHITE_TERRACOTTA,
			ORANGE_TERRACOTTA,
			GREEN_STAINED_HARDENED_CLAY,
			BLUE_TERRACOTTA,
			YELLOW_TERRACOTTA,
			BLACK_CONCRETE,
			GRAY_CONCRETE_POWDER,
			CYAN_TERRACOTTA,
			WHITE_CONCRETE,
			GRAY_CONCRETE,
			LIGHT_GRAY_CONCRETE,
			BROWN_CONCRETE,
			RED_CONCRETE,
			ORANGE_TERRACOTTA,
			YELLOW_CONCRETE,
			LIME_CONCRETE,
			GREEN_STAINED_HARDENED_CLAY,
			CYAN_CONCRETE,
			LIGHT_BLUE_CONCRETE,
			BLUE_CONCRETE,
			PURPLE_CONCRETE,
			MAGENTA_CONCRETE,
			RED_TERRACOTTA,
	};
}

std::vector<Block> Tree::get_building_floor_blocks()
{
	return std::vector<Block>{
			GRAY_CONCRETE,
			LIGHT_GRAY_CONCRETE,
			WHITE_CONCRETE,
			SMOOTH_STONE,
			POLISHED_ANDESITE,
			STONE_BRICKS,
	};
}

std::vector<Block> Tree::get_structural_blocks()
{
	return std::vector<Block>{
			// Fences
			OAK_FENCE,
			// Walls
			COBBLESTONE_WALL,
			ANDESITE_WALL,
			STONE_BRICK_WALL,
			// Stairs
			OAK_STAIRS,
			// Slabs
			OAK_SLAB,
			STONE_BLOCK_SLAB,
			STONE_BRICK_SLAB,
			// Rails
			RAIL,
			RAIL_NORTH_SOUTH,
			RAIL_EAST_WEST,
			RAIL_ASCENDING_EAST,
			RAIL_ASCENDING_WEST,
			RAIL_ASCENDING_NORTH,
			RAIL_ASCENDING_SOUTH,
			RAIL_NORTH_EAST,
			RAIL_NORTH_WEST,
			RAIL_SOUTH_EAST,
			RAIL_SOUTH_WEST,
			// Doors and trapdoors
			OAK_DOOR,
			DARK_OAK_DOOR_LOWER,
			DARK_OAK_DOOR_UPPER,
			OAK_TRAPDOOR,
			// Ladders
			LADDER,
	};
}

std::vector<Block> Tree::get_functional_blocks()
{
	return std::vector<Block>{
			// Furniture and functional blocks
			CHEST,
			CRAFTING_TABLE,
			FURNACE,
			ANVIL,
			BREWING_STAND,
			NOTE_BLOCK,
			BOOKSHELF,
			CAULDRON,
			// Beds
			RED_BED_NORTH_HEAD,
			RED_BED_NORTH_FOOT,
			RED_BED_EAST_HEAD,
			RED_BED_EAST_FOOT,
			RED_BED_SOUTH_HEAD,
			RED_BED_SOUTH_FOOT,
			RED_BED_WEST_HEAD,
			RED_BED_WEST_FOOT,
			// Pressure plates and signs
			OAK_PRESSURE_PLATE,
			SIGN,
			// Glass blocks (windows)
			GLASS,
			WHITE_STAINED_GLASS,
			GRAY_STAINED_GLASS,
			LIGHT_GRAY_STAINED_GLASS,
			BROWN_STAINED_GLASS,
			CYAN_STAINED_GLASS,
			BLUE_STAINED_GLASS,
			LIGHT_BLUE_STAINED_GLASS,
			TINTED_GLASS,
			// Carpets
			WHITE_CARPET,
			RED_CARPET,
			// Other structural/building blocks
			IRON_BARS,
			IRON_BLOCK,
			SCAFFOLDING,
			BEDROCK,
	};
}

bool Tree::canopy_might_intersect_building(
		int x, int z, const BuildingFootprintBitmap *building_footprints)
{
	if (!building_footprints)
		return false;
	for (int check_x = x - MAX_CANOPY_RADIUS; check_x <= x + MAX_CANOPY_RADIUS; ++check_x)
		for (int check_z = z - MAX_CANOPY_RADIUS; check_z <= z + MAX_CANOPY_RADIUS;
				++check_z)
			if (building_footprints->contains(check_x, check_z))
				return true;
	return false;
}

void Tree::create_of_type(WorldEditor &editor, const Coord &pos, TreeType tree_type,
		const BuildingFootprintBitmap *building_footprints,
		const bridges::BridgeSurfaceMap *bridge_surface, bool allow_on_paved,
		std::optional<double> mapped_height_m, bool from_tags, bool use_region_pack,
		bool density_decided)
{
	// Tile halos may discover the same tree as their owner. Rust assigns tree
	// placement to the tile containing the trunk so edge columns are generated once.
	if (!editor.owns(pos.x, pos.z))
		return;

	// Skip if this coordinate is inside a building
	if (building_footprints != nullptr) {
		if (building_footprints->contains(pos.x, pos.z)) {
			return;
		}
	}

	// A trunk rooted below a bridge would grow through its deck.
	if (bridge_surface && bridge_surface->contains(pos.x, pos.z))
		return;
	if (!allow_on_paved && editor.surface_is_sealed(pos.x, pos.z))
		return;

	const std::optional<std::vector<Block>> protected_surface_blocks(
			allow_on_paved ? std::vector<Block>{WATER}
						   : std::vector<Block>{
									 BLACK_CONCRETE,
									 GRAY_CONCRETE_POWDER,
									 CYAN_TERRACOTTA,
									 GRAY_CONCRETE,
									 LIGHT_GRAY_CONCRETE,
									 DIRT_PATH,
									 SMOOTH_STONE,
									 WATER,
							 });
	if (editor.check_for_block(pos.x, 0, pos.z, protected_surface_blocks))
		return;
	const auto cover =
			editor.ground ? editor.ground->cover_class(editor.ground_point(pos.x, pos.z))
						  : std::uint8_t{0};
	const bool wet_ground =
			cover == land_cover::LC_WETLAND || cover == land_cover::LC_MANGROVES;
	if (use_region_pack && editor.scale() >= MICRO_TREE_MAX_SCALE) {
		const auto handled = editor.place_regional_tree(pos.x,
				editor.get_absolute_y(pos.x, pos.y, pos.z), pos.z, cover,
				habitat_for_tree_type(tree_type), from_tags, wet_ground, allow_on_paved,
				density_decided);
		// A present false result is still handled: Rust's active pack may choose
		// a clearing, which must not fall through to a procedural tree.
		if (handled.has_value())
			return;
	}

	std::vector<Block> blacklist;
	auto bw = get_building_wall_blocks();
	blacklist.insert(blacklist.end(), bw.begin(), bw.end());
	auto bf = get_building_floor_blocks();
	blacklist.insert(blacklist.end(), bf.begin(), bf.end());
	auto sb = get_structural_blocks();
	blacklist.insert(blacklist.end(), sb.begin(), sb.end());
	auto fb = get_functional_blocks();
	blacklist.insert(blacklist.end(), fb.begin(), fb.end());
	blacklist.push_back(WATER);

	auto shape_rng = coord_rng(pos.x, pos.z, 1);
	const std::uint32_t variant_idx = shape_rng();
	Tree tree = get_tree_variant(tree_type, variant_idx);
	const bool check_canopy_collision =
			canopy_might_intersect_building(pos.x, pos.z, building_footprints);
	const int base_y = editor.get_absolute_y(pos.x, pos.y, pos.z);
	// Fixed-size tree models dominate country-scale terrain. Keep the same
	// species palette but scale a nominal 25 m tree into a compact shrub.
	if (editor.scale() < MICRO_TREE_MAX_SCALE) {
		const int height = std::clamp(
				static_cast<int>(
						std::lround(mapped_height_m.value_or(25.0) * editor.scale())),
				1, 8);
		const int trunk = std::max(0, height - 1);
		for (int dy = 0; dy < trunk; ++dy)
			editor.set_block_absolute(tree.log_block, pos.x, base_y + dy, pos.z,
					std::nullopt, std::optional<const std::vector<Block>>(blacklist));
		const int top = base_y + trunk;
		editor.set_block_absolute(tree.leaves_block, pos.x, top, pos.z, std::nullopt,
				std::optional<const std::vector<Block>>(blacklist));
		if (height >= 3)
			for (const auto &[dx, dz] : std::array<std::pair<int, int>, 4>{
						 {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}})
				editor.set_block_absolute(tree.leaves_block, pos.x + dx, top, pos.z + dz,
						std::nullopt, std::optional<const std::vector<Block>>(blacklist));
		if (height >= 5)
			editor.set_block_absolute(tree.leaves_block, pos.x, top + 1, pos.z,
					std::nullopt, std::optional<const std::vector<Block>>(blacklist));
		return;
	}
	int canopy_top = 0;
	for (const auto &range : tree.leaves_fill)
		canopy_top = std::max(canopy_top, range.second.y);
	// Rust jitters the trunk from the independent shape seed, then clamps it
	// below the foliage cap. Bushes have no trunk; all other species retain at
	// least a two-block trunk unless the foliage geometry itself is shorter.
	const int height_jitter = int((variant_idx >> 8) & 0x3U) - 1;
	const int min_trunk = tree.log_height == 0 ? 0 : 2;
	const int trunk_cap = std::max(canopy_top - 1, min_trunk);
	const int trunk_height =
			std::clamp(tree.log_height + height_jitter, min_trunk, trunk_cap);

	// Snapshot roof heights before foliage is emitted.  A footprint with no
	// stamped building block deliberately culls the full column, matching the
	// Rust conservative fallback.
	std::array<int, (MAX_CANOPY_RADIUS * 2 + 1) * (MAX_CANOPY_RADIUS * 2 + 1)> roof_tops;
	roof_tops.fill(std::numeric_limits<int>::min());
	auto roof_index = [](int dx, int dz) {
		constexpr int span = MAX_CANOPY_RADIUS * 2 + 1;
		return (dx + MAX_CANOPY_RADIUS) * span + (dz + MAX_CANOPY_RADIUS);
	};
	if (check_canopy_collision && building_footprints)
		for (int dx = -MAX_CANOPY_RADIUS; dx <= MAX_CANOPY_RADIUS; ++dx)
			for (int dz = -MAX_CANOPY_RADIUS; dz <= MAX_CANOPY_RADIUS; ++dz)
				if (building_footprints->contains(pos.x + dx, pos.z + dz))
					roof_tops[roof_index(dx, dz)] =
							editor.highest_block_between(pos.x + dx, pos.z + dz, base_y,
										  base_y + canopy_top)
									.value_or(base_y + canopy_top);
	auto blocked_by_roof = [&](int x, int y, int z) {
		if (!check_canopy_collision || !building_footprints)
			return false;
		const int dx = x - pos.x, dz = z - pos.z;
		if (std::abs(dx) > MAX_CANOPY_RADIUS || std::abs(dz) > MAX_CANOPY_RADIUS)
			return building_footprints->contains(x, z);
		return y <= roof_tops[roof_index(dx, dz)];
	};
	auto place_leaf = [&](int x, int y, int z, bool apex = false, bool surface = false) {
		if (blocked_by_roof(x, y, z))
			return;
		// Rust suppresses ordinary canopy leaves directly above water; the apex
		// cap is deliberately exempt so a short trunk is never left exposed.
		if (!apex && editor.check_for_block_type_absolute(x, y - 1, z, WATER))
			return;
		// Rust's position hash leaves small deterministic organic gaps.  The
		// apex is exempt so a short trunk is never visibly exposed.
		const std::uint64_t h = leaf_hash(x, y, z);
		if (!apex && leaf_gap_at(h))
			return;
		Block block = tree.leaves_block;
		if (surface && tree.accent_block &&
				(h * 2654435761ULL) % 100 < tree.accent_chance)
			block = *tree.accent_block;
		editor.set_block_absolute(block, x, y, z, std::nullopt, std::nullopt);
	};

	// Build the trunk without replacing structural/building blocks.
	if (tree.log_height > 0)
		for (int y = base_y; y <= base_y + trunk_height; ++y)
			editor.set_block_absolute(tree.log_block, pos.x, y, pos.z, std::nullopt,
					std::optional<const std::vector<Block>>(blacklist));

	// Fill in the leaves
	for (const auto &pr : tree.leaves_fill) {
		const Coord &a = pr.first;
		const Coord &b = pr.second;
		if (check_canopy_collision) {
			for (int leaf_x = pos.x + a.x; leaf_x <= pos.x + b.x; ++leaf_x) {
				for (int leaf_y = base_y + a.y; leaf_y <= base_y + b.y; ++leaf_y) {
					for (int leaf_z = pos.z + a.z; leaf_z <= pos.z + b.z; ++leaf_z) {
						place_leaf(leaf_x, leaf_y, leaf_z);
					}
				}
			}
		} else {
			for (int leaf_x = pos.x + a.x; leaf_x <= pos.x + b.x; ++leaf_x)
				for (int leaf_y = base_y + a.y; leaf_y <= base_y + b.y; ++leaf_y)
					for (int leaf_z = pos.z + a.z; leaf_z <= pos.z + b.z; ++leaf_z)
						place_leaf(leaf_x, leaf_y, leaf_z);
		}
	}
	place_leaf(pos.x, base_y + canopy_top, pos.z, true);

	// Do the three rounds
	for (std::size_t idx = 0; idx < tree.round_ranges.size(); ++idx) {
		const std::vector<int> &range = tree.round_ranges[idx];
		std::span<const Coord> pattern = ROUND_PATTERNS[idx];
		const bool surface = std::none_of(tree.round_ranges.begin() + idx + 1,
				tree.round_ranges.end(), [](const auto &r) { return !r.empty(); });
		for (int offset : range) {
			if (check_canopy_collision) {
				for (const Coord &d : pattern) {
					int leaf_x = pos.x + d.x;
					int leaf_z = pos.z + d.z;
					place_leaf(leaf_x, base_y + offset + d.y, leaf_z, false, surface);
				}
			} else
				for (const Coord &d : pattern)
					place_leaf(pos.x + d.x, base_y + offset + d.y, pos.z + d.z, false,
							surface);
		}
	}

	const float branch_roll = float((variant_idx >> 16) & 0xffU) / 255.0f;
	if (branch_roll < tree.branch_chance && trunk_height >= 5) {
		const std::array<std::pair<int, int>, 4> dirs{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
		const auto [dx, dz] = dirs[(variant_idx >> 24) & 3U];
		const int branch_y = base_y + trunk_height - 2 - int((variant_idx >> 12) & 1U);
		for (int step = 1; step <= 2; ++step)
			editor.set_block_absolute(tree.log_block, pos.x + dx * step, branch_y,
					pos.z + dz * step, std::nullopt,
					std::optional<const std::vector<Block>>(blacklist));
		for (int bx = -1; bx <= 1; ++bx)
			for (int by = -1; by <= 1; ++by)
				for (int bz = -1; bz <= 1; ++bz)
					if (std::abs(bx) + std::abs(by) + std::abs(bz) <= 2)
						place_leaf(pos.x + dx * 2 + bx, branch_y + by,
								pos.z + dz * 2 + bz, false, true);
	}
	if (tree.drooping) {
		const std::array<std::pair<int, int>, 8> dirs{
				{{2, 0}, {-2, 0}, {0, 2}, {0, -2}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}}};
		for (std::size_t i = 0; i < dirs.size(); ++i) {
			const auto [dx, dz] = dirs[i];
			const int length = 2 + int((variant_idx >> (i * 2)) & 3U);
			for (int n = 0; n < length; ++n)
				place_leaf(pos.x + dx, base_y + 5 - n, pos.z + dz);
		}
	}
}

}
