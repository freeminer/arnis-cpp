#include "structures.h"
#include "../assets_root.h"
#include "../land_cover/land_cover.h"
#include "../strict_parse.h"
#include "../trees/schematic.h"
#include "schem_decoder.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <string_view>

namespace arnis::structures::crane
{
namespace
{
bool site_has_mapped_crane(
		const WorldEditor &editor, const std::vector<std::pair<int, int>> &cells)
{
	if (!editor.mapped_cranes || cells.empty())
		return false;
	const auto [min_x, max_x] = std::minmax_element(cells.begin(), cells.end(),
			[](const auto &a, const auto &b) { return a.first < b.first; });
	const auto [min_z, max_z] = std::minmax_element(cells.begin(), cells.end(),
			[](const auto &a, const auto &b) { return a.second < b.second; });
	return std::any_of(editor.mapped_cranes->begin(), editor.mapped_cranes->end(),
			[&](const auto &point) {
				return point.first >= min_x->first && point.first <= max_x->first &&
					   point.second >= min_z->second && point.second <= max_z->second &&
					   std::find(cells.begin(), cells.end(), point) != cells.end();
			});
}

std::shared_ptr<const StructureSchematic> crane_schematic(WorldEditor &editor)
{
	const auto root = editor.get_schematic_asset_root().empty()
							  ? assets::path("structures")
							  : editor.get_schematic_asset_root();
	return try_load_structure_cached(root / "crane.schem");
}

std::pair<int, int> jib_direction(const StructureSchematic &schem, unsigned rotation)
{
	const auto [ax, az] = trees::rotate_xz(
			schem.anchor_x, schem.anchor_z, schem.width, schem.length, rotation);
	long long farthest = -1;
	std::pair<int, int> best{0, 1};
	for (const auto &voxel : schem.voxels) {
		const auto [vx, vz] =
				trees::rotate_xz(voxel.x, voxel.z, schem.width, schem.length, rotation);
		const int dx = vx - ax, dz = vz - az;
		const long long d =
				static_cast<long long>(dx) * dx + static_cast<long long>(dz) * dz;
		// Rust Iterator::max_by_key keeps the later item on ties.
		if (d >= farthest) {
			farthest = d;
			best = {dx, dz};
		}
	}
	if (std::abs(best.first) >= std::abs(best.second))
		return {best.first < 0 ? -1 : 1, 0};
	return {0, best.second < 0 ? -1 : 1};
}

unsigned quarter_for_bearing(const StructureSchematic &schem, double degrees)
{
	const double radians = degrees * std::acos(-1.0) / 180.0;
	const double want_x = std::sin(radians), want_z = -std::cos(radians);
	unsigned best_rotation = 0;
	double best_dot = -std::numeric_limits<double>::infinity();
	for (unsigned rotation = 0; rotation < 4; ++rotation) {
		const auto [dx, dz] = jib_direction(schem, rotation);
		const double dot = dx * want_x + dz * want_z;
		// Match Rust Iterator::max_by: equal bearings select the later quarter turn.
		if (dot >= best_dot) {
			best_dot = dot;
			best_rotation = rotation;
		}
	}
	return best_rotation;
}
} // namespace

void maybe_place_crane(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells)
{
	if (!editor.place_schematics() || cells.size() < 1500)
		return;
	auto schematic = crane_schematic(editor);
	if (!schematic)
		return;
	long long sx = 0, sz = 0;
	for (const auto &[x, z] : cells) {
		sx += x;
		sz += z;
	}
	const int cx = sx / static_cast<long long>(cells.size());
	const int cz = sz / static_cast<long long>(cells.size());
	const auto it = std::min_element(
			cells.begin(), cells.end(), [cx, cz](const auto &a, const auto &b) {
				const auto distance = [cx, cz](const auto &p) {
					const long long dx = static_cast<long long>(p.first) - cx;
					const long long dz = static_cast<long long>(p.second) - cz;
					return dx * dx + dz * dz;
				};
				return distance(a) < distance(b);
			});
	const auto &[x, z] = *it;
	if (site_has_mapped_crane(editor, cells))
		return;
	const auto h = land_cover::coord_hash(x, z);
	if (editor.is_lc_water(x, z) || h % 100 >= 60)
		return;
	place_structure(editor, *schematic, x, editor.get_absolute_y(x, 1, z), z,
			(unsigned)((h >> 8) & 3));
}

void place_mapped(WorldEditor &editor, int x, int z, const tags_t &tags,
		const CoordinateBitmap &building_footprints)
{
	if (!editor.place_schematics())
		return;
	const auto kind = tags.find("crane:type");
	const auto indoor = tags.find("indoor");
	if ((kind != tags.end() &&
				(kind->second == "travel_lift" || kind->second == "bridge_crane" ||
						kind->second == "overhead_crane" ||
						kind->second == "jib_crane")) ||
			(indoor != tags.end() && indoor->second == "yes") ||
			building_footprints.contains(x, z))
		return;
	const auto schematic = crane_schematic(editor);
	if (!schematic)
		return;
	const auto wet = [&](int px, int pz) {
		return editor.is_lc_water(px, pz) ||
			   editor.check_for_block_absolute(px, editor.get_ground_level(px, pz), pz,
					   std::optional<std::vector<Block>>{std::vector<Block>{WATER}},
					   std::nullopt);
	};
	auto water_toward = [&](unsigned rotation) {
		const auto [dx, dz] = jib_direction(*schematic, rotation);
		unsigned count = 0;
		for (int distance = 8; distance <= 40; distance += 4)
			count += wet(x + dx * distance, z + dz * distance);
		return count;
	};
	unsigned rotation = 0, best_water = 0;
	for (unsigned candidate = 0; candidate < 4; ++candidate) {
		const unsigned count = water_toward(candidate);
		// Rust Iterator::max_by_key keeps the later rotation when water counts tie.
		if (count >= best_water) {
			best_water = count;
			rotation = candidate;
		}
	}
	if (!best_water) {
		const auto direction = tags.find("direction");
		if (direction != tags.end()) {
			std::string_view bearing = direction->second;
			while (!bearing.empty() &&
					std::isspace(static_cast<unsigned char>(bearing.front())))
				bearing.remove_prefix(1);
			while (!bearing.empty() &&
					std::isspace(static_cast<unsigned char>(bearing.back())))
				bearing.remove_suffix(1);
			if (const auto degrees = strict_parse::f64(bearing))
				rotation = quarter_for_bearing(*schematic, *degrees);
			else
				rotation = static_cast<unsigned>(land_cover::coord_hash(x, z) & 3);
		} else {
			rotation = static_cast<unsigned>(land_cover::coord_hash(x, z) & 3);
		}
	}
	place_structure(editor, *schematic, x, editor.get_absolute_y(x, 1, z), z, rotation);
}
}
