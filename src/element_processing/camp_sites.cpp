#include "camp_sites.h"

#include "area_layout.h"
#include "../land_cover/land_cover.h"
#include "../structures/structures.h"
#include "world_editor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>

namespace arnis::camp_sites
{
namespace
{
constexpr std::string_view PITCHES_TAG = "arnis:pitches";

bool tag_is(const tags_t &tags, std::string_view key, std::string_view value)
{
	return tags.get(std::string(key)) == value;
}

bool ground_is_protected(WorldEditor &editor, int x, int z)
{
	static const std::optional<std::vector<Block>> protected_blocks{std::vector<Block>{
			BLACK_CONCRETE, GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA, GRAY_CONCRETE,
			LIGHT_GRAY_CONCRETE, WHITE_CONCRETE, DIRT_PATH, SMOOTH_STONE, WATER}};
	return editor.surface_is_sealed(x, z) ||
		   editor.check_for_block(x, 0, z, protected_blocks);
}

struct Stay
{
	enum Kind : std::uint8_t
	{
		Dome,
		Ridge,
		Caravan,
		Motorhome
	} kind{};

	static std::optional<Stay> pick(std::uint64_t h, const tags_t &tags)
	{
		const auto no = [&](std::string_view key) { return tag_is(tags, key, "no"); };
		const bool caravan_site = tag_is(tags, "tourism", "caravan_site");
		const auto choice = h % 100;
		Stay stay;
		if (caravan_site) {
			if (choice <= 44)
				stay.kind = Motorhome;
			else if (choice <= 79)
				stay.kind = Caravan;
			else
				return std::nullopt;
		} else if (choice <= 21) {
			stay.kind = Dome;
		} else if (choice <= 35) {
			stay.kind = Ridge;
		} else if (choice <= 62) {
			stay.kind = Caravan;
		} else if (choice <= 79) {
			stay.kind = Motorhome;
		} else {
			return std::nullopt;
		}
		const bool opted_out = stay.kind == Dome || stay.kind == Ridge ? no("tents")
							   : stay.kind == Caravan				   ? no("caravans")
																	   : no("motorhome");
		return opted_out ? std::nullopt : std::optional<Stay>(stay);
	}

	std::array<int, 4> extent() const
	{
		switch (kind) {
		case Dome:
			return {-1, 1, -1, 1};
		case Ridge:
			return {-1, 1, -1, 2};
		case Caravan:
			return {-1, 1, -3, 3};
		case Motorhome:
			return {-2, 2, -4, 4};
		}
		return {};
	}
	bool operator==(Kind other) const { return kind == other; }
};

void draw(WorldEditor &editor, Stay stay, std::uint64_t h,
		const std::function<std::pair<int, int>(int, int)> &at, std::pair<int, int> dir)
{
	const auto put = [&](int u, int t, int y, Block block) {
		const auto [x, z] = at(u, t);
		editor.set_block(block, x, y, z);
	};
	static const std::array<Block, 7> TENTS{GREEN_WOOL, BLUE_WOOL, ORANGE_WOOL, RED_WOOL,
			YELLOW_WOOL, CYAN_WOOL, GRAY_WOOL};
	const Block cloth = TENTS[h % TENTS.size()];
	const std::uint8_t rot = dir.first != 0 ? 1 : 0;
	switch (stay.kind) {
	case Stay::Dome:
		for (int u = -1; u <= 1; ++u) {
			for (int t = -1; t <= 1; ++t) {
				if (u != 0 || t != -1)
					put(u, t, 1, cloth);
				if (u == 0 || t == 0)
					put(u, t, 2, cloth);
			}
		}
		break;
	case Stay::Ridge:
		for (int t = -1; t <= 2; ++t) {
			put(-1, t, 1, cloth);
			put(1, t, 1, cloth);
			put(0, t, 2, cloth);
		}
		put(0, 2, 1, cloth);
		break;
	case Stay::Caravan:
		for (int u = -1; u <= 1; ++u) {
			for (int t = -2; t <= 2; ++t) {
				const bool side = u != 0 || std::abs(t) == 2;
				put(u, t, 1, WHITE_CONCRETE);
				const bool window = side && (std::abs(t) == 2 || (u != 0 && t == 0));
				put(u, t, 2, window ? GLASS : WHITE_CONCRETE);
				put(u, t, 3, SMOOTH_QUARTZ_SLAB);
			}
		}
		put(0, -3, 1, IRON_BARS);
		{
			const auto [cx, cz] = at(4, 0);
			structures::car::place_vehicle(editor, cx, cz, rot, false, h >> 4);
		}
		break;
	case Stay::Motorhome: {
		const auto [cx, cz] = at(0, 0);
		structures::car::place_vehicle(editor, cx, cz, rot, true, h >> 4);
		break;
	}
	}
}

struct Site
{
	std::uint64_t id{};
	element_processing::area_layout::Frame frame;
	element_processing::area_layout::AreaMask mask;
	double width{}, depth{}, lane{};
	bool lanes{};

	double period() const { return lane + 2.0 * depth; }

	std::optional<std::array<std::int64_t, 3>> pitch_of(int x, int z) const
	{
		const auto [a, c] = frame.local(x, z);
		const double p = period();
		const auto band = static_cast<std::int64_t>(std::floor(c / p));
		const double cc = c - band * p;
		if (cc < lane)
			return std::nullopt;
		const std::int64_t i = static_cast<std::int64_t>(std::floor(a / width));
		const std::int64_t side = cc < lane + depth ? 0 : 1;
		return std::array<std::int64_t, 3>{band, i, side};
	}

	bool on_lane(int x, int z) const
	{
		const auto [a, c] = frame.local(x, z);
		(void)a;
		const auto p = period();
		const auto band = std::floor(c / p);
		return c - band * p < lane;
	}

	bool free(const WorldEditor &editor, const RoadMaskBitmap &road_mask,
			const BuildingFootprintBitmap &footprints, int x, int z) const
	{
		return mask.contains(x, z) && !road_mask.contains(x, z) &&
			   !footprints.contains(x, z) && !editor.surface_is_sealed(x, z) &&
			   !editor.is_lc_water(x, z);
	}

	void place(WorldEditor &editor, const ProcessedWay &way,
			const RoadMaskBitmap &road_mask,
			const BuildingFootprintBitmap &footprints) const
	{
		const auto [min_x, min_z] = editor.get_min_coords();
		const auto [max_x, max_z] = editor.get_max_coords();
		const auto spans = frame.local_span({min_x, min_z, max_x, max_z});
		const double a0 = std::max(0.0, spans[0].first);
		const double a1 = std::min(frame.length(), spans[0].second);
		const double c0 = std::max(0.0, spans[1].first);
		const double c1 = std::min(frame.width(), spans[1].second);
		if (a0 > a1 || c0 > c1)
			return;
		const double p = period();
		const auto back = frame.across_axis();
		const auto band0 = static_cast<std::int64_t>(std::floor(c0 / p)) - 1;
		const auto band1 = static_cast<std::int64_t>(std::ceil(c1 / p));
		const auto i0 = static_cast<std::int64_t>(std::floor(a0 / width)) - 1;
		const auto i1 = static_cast<std::int64_t>(std::ceil(a1 / width));
		for (auto band = band0; band <= band1; ++band) {
			for (auto i = i0; i <= i1; ++i) {
				for (std::int64_t side = 0; side < 2; ++side) {
					const auto h = element_processing::area_layout::cell_hash(
							id ^ static_cast<std::uint64_t>(side), band, i);
					const double c_mid = side == 0 ? band * p + lane + depth / 2.0
												   : band * p + p - depth / 2.0;
					const auto dir = side == 0 ? back
											   : std::pair<int32_t, int32_t>{
														 -back.first, -back.second};
					auto centre = frame.column((i + 0.5) * width, c_mid);
					const auto post = frame.column(i * width + 0.5,
							side == 0 ? band * p + lane + 0.5 : band * p + p - 0.5);
					if (editor.owns(post.first, post.second) &&
							free(editor, road_mask, footprints, post.first,
									post.second)) {
						editor.set_block(OAK_FENCE, post.first, 1, post.second);
						if (h % 3 == 0)
							editor.set_block(LANTERN, post.first, 2, post.second);
					}
					const auto stay = Stay::pick(h >> 8, way.tags);
					if (!stay)
						continue;
					const std::pair<int32_t, int32_t> side_dir{-dir.second, dir.first};
					if (*stay == Stay::Caravan)
						centre = {centre.first - 2 * side_dir.first,
								centre.second - 2 * side_dir.second};
					if (!editor.owns(centre.first, centre.second))
						continue;
					const auto at = [&](int u, int t) {
						return std::pair<int, int>{
								centre.first + u * side_dir.first + t * dir.first,
								centre.second + u * side_dir.second + t * dir.second};
					};
					const auto bounds = stay->extent();
					const auto own = std::array<std::int64_t, 3>{band, i, side};
					bool fits = true;
					for (int u = bounds[0] - 1; fits && u <= bounds[1] + 1; ++u) {
						for (int t = bounds[2] - 1; t <= bounds[3] + 1; ++t) {
							const auto [x, z] = at(u, t);
							const bool inner = u >= bounds[0] && u <= bounds[1] &&
											   t >= bounds[2] && t <= bounds[3];
							if (!free(editor, road_mask, footprints, x, z) ||
									(inner && pitch_of(x, z) != own)) {
								fits = false;
								break;
							}
						}
					}
					if (fits)
						draw(editor, *stay, h >> 16, at, dir);
				}
			}
		}
	}
};
} // namespace

bool claims(const tags_t &tags)
{
	const auto tourism = tags.get("tourism");
	return tourism == "camp_site" || tourism == "caravan_site";
}

bool is_pitch(const tags_t &tags)
{
	return tags.get("tourism") == "camp_pitch";
}

void prepare(std::vector<ProcessedElement> &elements)
{
	std::vector<std::pair<double, double>> pitches;
	for (const auto &element : elements) {
		if (element.is_node() && is_pitch(element.as_node().tags)) {
			pitches.emplace_back(element.as_node().x, element.as_node().z);
		} else if (element.is_way() && is_pitch(element.as_way().tags)) {
			if (const auto point = element_processing::area_layout::interior_point(
						element.as_way().nodes))
				pitches.push_back(*point);
		}
	}
	if (pitches.empty())
		return;
	for (auto &element : elements) {
		std::vector<const std::vector<ProcessedNode> *> rings;
		tags_t *tags = nullptr;
		if (element.is_way()) {
			auto &way = std::get<ProcessedWay>(element);
			tags = &way.tags;
			rings.push_back(&way.nodes);
		} else if (element.is_relation()) {
			auto &relation = std::get<ProcessedRelation>(element);
			tags = &relation.tags;
			for (const auto &member : relation.members)
				if (member.role == ProcessedMemberRole::Outer && member.way)
					rings.push_back(&member.way->nodes);
		}
		if (!tags || !claims(*tags))
			continue;
		std::vector<element_processing::area_layout::Bounds> boxes;
		for (const auto *ring : rings)
			if (const auto box = element_processing::area_layout::bounds(*ring))
				boxes.push_back(*box);
		const bool has_mapped_pitch =
				std::any_of(pitches.begin(), pitches.end(), [&](const auto &point) {
					return std::any_of(boxes.begin(), boxes.end(), [&](const auto &box) {
						return element_processing::area_layout::inside(box, point);
					}) && element_processing::area_layout::point_in_rings(rings, point);
				});
		if (has_mapped_pitch)
			tags->insert_or_assign(std::string(PITCHES_TAG), "yes");
	}
}

void generate_camp_site(WorldEditor &editor, const ProcessedWay &way,
		const FloodFillCache &flood_fill_cache,
		const BuildingFootprintBitmap &building_footprints,
		const RoadMaskBitmap &road_mask,
		const std::optional<std::chrono::milliseconds> &timeout)
{
	camp_area(editor, way, flood_fill_cache, building_footprints, road_mask, timeout,
			nullptr);
}

void camp_area(WorldEditor &editor, const ProcessedWay &way,
		const FloodFillCache &flood_fill_cache,
		const BuildingFootprintBitmap &building_footprints,
		const RoadMaskBitmap &road_mask,
		const std::optional<std::chrono::milliseconds> &timeout, const Holes *holes)
{
	const auto fill = flood_fill_cache.get_or_compute(way, timeout);
	const auto frame = element_processing::area_layout::Frame::of(way.nodes);
	if (!frame)
		return;
	bool paths = false;
	std::size_t built = 0;
	for (const auto &[x, z] : *fill) {
		paths = paths || road_mask.contains(x, z);
		built += building_footprints.contains(x, z) ? 1 : 0;
	}
	const element_processing::area_layout::AreaMask mask(
			*fill, [&](int32_t x, int32_t z) { return !in_hole(holes, x, z); });
	const bool laid_out =
			!way.tags.contains(std::string(PITCHES_TAG)) && built * 12 < fill->size();
	const double scale = std::clamp(editor.scale(), 0.25, 4.0);
	Site site{way.id, *frame, mask, std::max(9.0 * scale, 5.0),
			std::max(10.0 * scale, 5.0), std::max(4.0 * scale, 2.0), laid_out && !paths};
	for (const auto &[x, z] : *fill) {
		if (in_hole(holes, x, z) || !editor.in_bounds(x, z) ||
				ground_is_protected(editor, x, z))
			continue;
		const bool lane = site.lanes && site.on_lane(x, z);
		editor.set_block(lane ? GRAVEL : GRASS_BLOCK, x, 0, z);
	}
	if (laid_out)
		site.place(editor, way, road_mask, building_footprints);
}

void generate_pitch_node(WorldEditor &editor, const ProcessedNode &node)
{
	if (!editor.owns(node.x, node.z))
		return;
	const auto h = land_cover::coord_hash(node.x ^ 0x0CA3, node.z);
	const std::pair<int, int> dir = h % 2 == 0 ? std::pair{0, 1} : std::pair{1, 0};
	const std::pair<int, int> side{-dir.second, dir.first};
	const auto at = [&](int u, int t) {
		return std::pair<int, int>{node.x + u * side.first + t * dir.first,
				node.z + u * side.second + t * dir.second};
	};
	std::vector<Stay> stays;
	if (!tag_is(node.tags, "tents", "no"))
		stays.insert(stays.end(), {{Stay::Dome}, {Stay::Ridge}});
	if (!tag_is(node.tags, "motorhome", "no"))
		stays.push_back({Stay::Motorhome});
	if (stays.empty())
		return;
	const auto &stay = stays[((h >> 8) % 3) % stays.size()];
	draw(editor, stay, h >> 12, at, dir);
}

} // namespace arnis::camp_sites
