#include "golf.h"

#include "../block_definitions.h"
#include "../ground_decoration.h"
#include "../land_cover/land_cover.h"
#include "sport_pitches.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <set>

namespace arnis::golf
{
namespace
{
enum class Turf
{
	Green,
	Fairway,
	Tee,
	Rough,
	Bunker,
	Range
};

std::optional<Turf> turf(const tags_t &tags)
{
	const auto it = tags.find("golf");
	if (it == tags.end())
		return std::nullopt;
	if (it->second == "green")
		return Turf::Green;
	if (it->second == "fairway")
		return Turf::Fairway;
	if (it->second == "tee")
		return Turf::Tee;
	if (it->second == "rough")
		return Turf::Rough;
	if (it->second == "bunker")
		return Turf::Bunker;
	if (it->second == "driving_range")
		return Turf::Range;
	return std::nullopt;
}

Block turf_block(Turf kind, const tags_t &tags)
{
	if (kind == Turf::Green)
		return MOSS_BLOCK;
	if (kind == Turf::Bunker) {
		const auto surface = tags.find("surface");
		return surface != tags.end() && (surface->second == "gravel" ||
												surface->second == "fine_gravel")
					   ? GRAVEL
					   : SAND;
	}
	return GRASS_BLOCK;
}

void tee_markers(WorldEditor &editor, const ProcessedWay &way,
		const std::set<std::pair<int, int>> &cells)
{
	if (cells.size() < 12)
		return;
	double cx = 0.0, cz = 0.0;
	for (const auto &[x, z] : cells) {
		cx += x;
		cz += z;
	}
	cx /= cells.size();
	cz /= cells.size();
	const auto direction = sport_pitches::longest_edge_dir(way);
	if (!direction)
		return;
	const auto &[ux, uz] = *direction;
	const auto normal = std::pair{-uz, ux};
	Block marker = BLUE_WOOL;
	if (const auto it = way.tags.find("tee"); it != way.tags.end()) {
		if (it->second == "championship" || it->second == "black")
			marker = BLACK_WOOL;
		else if (it->second == "men" || it->second == "white")
			marker = WHITE_WOOL;
		else if (it->second == "women" || it->second == "ladies" || it->second == "red")
			marker = RED_WOOL;
		else if (it->second == "senior" || it->second == "yellow" || it->second == "gold")
			marker = YELLOW_WOOL;
	}
	for (const double side : {-2.0, 2.0}) {
		const int x = static_cast<int>(std::lround(cx + normal.first * side));
		const int z = static_cast<int>(std::lround(cz + normal.second * side));
		if (cells.contains({x, z}))
			editor.set_block(marker, x, 1, z);
	}
}

void golf_area(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const FloodFillCache &fills, const Holes *holes)
{
	const auto kind = turf(way.tags);
	if (!kind)
		return;
	const Block block = turf_block(*kind, way.tags);
	const auto fill = fills.get_or_compute(way, args.timeout);
	std::set<std::pair<int, int>> cells;
	for (const auto &[x, z] : *fill)
		if (!in_hole(holes, x, z))
			cells.emplace(x, z);
	static const std::vector<Block> over{GRASS_BLOCK, DIRT, COARSE_DIRT, PODZOL};
	static const std::vector<Block> moss{MOSS_BLOCK};
	const auto &plants = ground_decoration::loose_plant_blocks();
	for (const auto &[x, z] : cells) {
		editor.set_block(block, x, 0, z, over);
		if (*kind != Turf::Rough)
			editor.set_block(AIR, x, 1, z, plants);
	}
	if (*kind == Turf::Green) {
		constexpr std::array<std::pair<int, int>, 4> neighbours{
				{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
		for (const auto &[x, z] : cells) {
			const bool edge =
					std::any_of(neighbours.begin(), neighbours.end(), [&](const auto &d) {
						return !cells.contains({x + d.first, z + d.second});
					});
			if (edge)
				editor.set_block(GRASS_BLOCK, x, 0, z, moss);
		}
	}
	if (*kind == Turf::Tee)
		tee_markers(editor, way, cells);
}
} // namespace

bool claims(const tags_t &tags)
{
	if (!turf(tags) || tags.contains("water") || tags.contains("building") ||
			tags.contains("building:part") || tags.contains("highway"))
		return false;
	const auto natural = tags.find("natural");
	return natural == tags.end() ||
		   (natural->second != "water" && natural->second != "wetland");
}

bool is_managed(const tags_t &tags)
{
	return claims(tags) && turf(tags) != Turf::Rough;
}

bool is_hole(const tags_t &tags)
{
	const auto it = tags.find("golf");
	return it != tags.end() && it->second == "hole";
}

void generate_golf(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const FloodFillCache &fills)
{
	golf_area(editor, way, args, fills, nullptr);
}

void generate_golf_from_relation(WorldEditor &editor, const ProcessedRelation &relation,
		const Args &args, const FloodFillCache &fills)
{
	const InnerRings inners(relation);
	for (const auto &member : relation.members) {
		if (member.role != ProcessedMemberRole::Outer || !member.way)
			continue;
		const auto holes = fills.holes_in(*member.way, inners, args.timeout);
		ProcessedWay way{member.way->id, member.way->nodes, relation.tags,
				member.way->source_node_refs};
		golf_area(editor, way, args, fills, holes ? &*holes : nullptr);
	}
}

void generate_hole_flag(WorldEditor &editor, const ProcessedWay &way)
{
	if (way.nodes.size() < 2)
		return;
	const auto &first = way.nodes.front();
	const auto &last = way.nodes.back();
	if (first.x == last.x && first.z == last.z)
		return;
	for (int y = 1; y <= 3; ++y)
		editor.set_block(END_ROD, last.x, y, last.z);
	const auto &prev = way.nodes[way.nodes.size() - 2];
	const int dx = last.x - prev.x, dz = last.z - prev.z;
	const auto forward = std::abs(dx) >= std::abs(dz) ? std::pair{dx < 0   ? -1
																  : dx > 0 ? 1
																		   : 0,
																0}
													  : std::pair{0, dz < 0	  ? -1
																	 : dz > 0 ? 1
																			  : 0};
	static const std::array<Block, 3> flags{RED_WOOL, YELLOW_WOOL, WHITE_WOOL};
	const Block flag = flags[land_cover::coord_hash(last.x, last.z) % flags.size()];
	editor.set_block(flag, last.x + forward.first, 3, last.z + forward.second);
}
} // namespace arnis::golf
