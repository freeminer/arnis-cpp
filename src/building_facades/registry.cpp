#include "registry.h"

#include "../../../arnis_world_editor.h"
#include "../element_processing/buildings.h"
#include "manifest.h"
#include "../mapillary/facades.h"
#include "../mapillary/displays.h"
#include "../osm_parser.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

namespace arnis::building_facades
{
namespace
{
constexpr std::size_t MIN_RUN_CELLS = 2;
constexpr std::size_t MAX_PARTY_STRETCHES = 16;
constexpr int MIN_WALL_BLOCKS = 2;
constexpr int PARTY_ROOF_STEP = 2;

struct PendingRun
{
	std::uint64_t way_id = 0;
	std::uint64_t group = 0;
	std::size_t entry = 0;
	Choice choice;
	std::vector<std::pair<int, int>> cells;
	std::pair<int, int> dir;
	std::pair<int, int> normal;
	int base_y = 0;
	int height = 0;
	bool party = false;
};

struct GroupPick
{
	double wall_height_m = 0.0;
	std::uint64_t element_id = 0;
	Choice choice;
	std::size_t members = 0;
};

struct State
{
	std::mutex mutex;
	bool enabled = false;
	double scale = 1.0;
	std::uint32_t ppm = 16;
	std::filesystem::path directory;
	std::optional<FacadeSet> set;
	std::unordered_set<std::uint64_t> claimed;
	std::unordered_map<std::uint64_t, GroupPick> groups;
	std::optional<std::array<int, 4>> extent;
	std::vector<PendingRun> pending;
};
State &state()
{
	static State value;
	return value;
}

std::vector<std::pair<int, int>> line(int x0, int z0, int x1, int z1)
{
	std::vector<std::pair<int, int>> result;
	const int dx = std::abs(x1 - x0), dz = std::abs(z1 - z0);
	const int sx = x0 < x1 ? 1 : -1, sz = z0 < z1 ? 1 : -1;
	int err = dx - dz;
	for (;;) {
		result.emplace_back(x0, z0);
		if (x0 == x1 && z0 == z1)
			break;
		const int e2 = 2 * err;
		if (e2 > -dz) {
			err -= dz;
			x0 += sx;
		}
		if (e2 < dx) {
			err += dx;
			z0 += sz;
		}
	}
	return result;
}

}

void reset(bool want, const std::optional<std::filesystem::path> &directory,
		std::uint32_t pixels_per_metre, double scale)
{
	auto &s = state();
	std::lock_guard lock(s.mutex);
	s.enabled = false;
	s.claimed.clear();
	s.groups.clear();
	s.pending.clear();
	s.extent.reset();
	s.set.reset();
	s.scale = std::max(0.001, scale);
	s.ppm = pixels_per_metre ? pixels_per_metre : 16;
	if (!want || !directory)
		return;
	s.directory = *directory;
	s.set = FacadeSet::load_directory(s.directory);
	s.enabled = s.set && !s.set->entries().empty();
}

void set_world_extent(int min_x, int min_z, int max_x, int max_z)
{
	auto &s = state();
	std::lock_guard lock(s.mutex);
	s.extent = std::array<int, 4>{min_x, min_z, max_x, max_z};
}

bool enabled()
{
	auto &s = state();
	std::lock_guard lock(s.mutex);
	return s.enabled;
}

std::size_t finalize(world_editor::WorldEditor &editor)
{
	std::vector<PendingRun> pending;
	std::filesystem::path directory;
	std::optional<FacadeSet> set;
	std::unordered_map<std::uint64_t, GroupPick> groups;
	std::uint32_t ppm = 16;
	double scale = 1.0;
	{
		auto &s = state();
		std::lock_guard lock(s.mutex);
		if (!s.enabled || !s.set)
			return 0;
		pending.swap(s.pending);
		groups.swap(s.groups);
		directory = s.directory;
		set = s.set;
		ppm = s.ppm;
		scale = s.scale;
	}
	std::unordered_map<std::size_t, RgbImage> images;
	std::size_t placed = 0;
	for (auto &run : pending) {
		const auto group = groups.find(run.group);
		const Choice choice = group == groups.end() ? run.choice : group->second.choice;
		if (run.cells.size() < MIN_RUN_CELLS || choice.entry >= set->entries().size())
			continue;
		// Match mapillary::displays::wall_is_visible: a facade hidden entirely
		// below the terrain surface has no visible wall to carry a panel. Check
		// the wall's highest row against each column's effective ground height;
		// one exposed column is enough to keep the candidate.
		const int top = run.base_y + run.height;
		const bool wall_visible =
				std::any_of(run.cells.begin(), run.cells.end(), [&](const auto &cell) {
					return top > editor.get_absolute_y(cell.first, 0, cell.second) + 1;
				});
		if (!wall_visible)
			continue;
		auto image = images.find(choice.entry);
		if (image == images.end()) {
			auto loaded = load_image(directory, set->entries()[choice.entry], ppm);
			if (!loaded)
				continue;
			image = images.emplace(choice.entry, std::move(*loaded)).first;
		}
		const auto normal = mapillary::displays::outward_normal(
				run.dir, run.normal.first, run.normal.second);
		if (!normal)
			continue;
		const auto [right_x, right_z] = mapillary::displays::right_of(*normal);
		std::sort(run.cells.begin(), run.cells.end(), [&](const auto &a, const auto &b) {
			return a.first * right_x + a.second * right_z <
				   b.first * right_x + b.second * right_z;
		});
		std::vector<double> positions;
		positions.reserve(run.cells.size());
		for (const auto &[x, z] : run.cells)
			positions.push_back(x * right_x + z * right_z);
		const double step = mapillary::displays::cell_step(run.dir);
		const double left = positions.front() - step * 0.5;
		const double width =
				(positions.back() - positions.front() + step) / std::max(0.001, scale);
		const double height = static_cast<double>(run.height) / std::max(0.001, scale);
		const auto &entry = set->entries()[choice.entry];
		const Fit fit = Fit::make(entry, ppm, width, height, choice.phase_m);
		std::vector<int> hidden(run.cells.size(), 0);
		if (run.party) {
			for (std::size_t i = 0; i < run.cells.size(); ++i) {
				const auto [x, z] = run.cells[i];
				for (int y = run.base_y + run.height - 1; y >= run.base_y; --y) {
					bool occupied = false;
					for (int distance = 1; distance <= 2; ++distance)
						occupied |=
								editor.get_block_absolute(x + run.normal.first * distance,
											  y, z + run.normal.second * distance)
										.has_value();
					if (occupied) {
						hidden[i] = y + 1 - run.base_y;
						break;
					}
				}
			}
		}
		const int piece_cells = std::max(
				1, static_cast<int>(
						   std::lround(entry.metres_wide * std::max(0.001, scale))));
		std::size_t begin = 0;
		std::size_t stretches = 0;
		while (begin < run.cells.size()) {
			int low = hidden[begin], high = hidden[begin];
			std::size_t end = begin + 1;
			while (end < run.cells.size() &&
					std::max(high, hidden[end]) - std::min(low, hidden[end]) <=
							PARTY_ROOF_STEP) {
				low = std::min(low, hidden[end]);
				high = std::max(high, hidden[end]);
				++end;
			}
			const int rows_hidden = high;
			if (rows_hidden < run.height && end - begin >= MIN_RUN_CELLS &&
					stretches < MAX_PARTY_STRETCHES) {
				++stretches;
				for (std::size_t piece = begin; piece < end; piece += piece_cells) {
					const std::size_t last = std::min(end, piece + piece_cells);
					const double x0 = (positions[piece] - step * 0.5 - left) /
									  std::max(0.001, scale);
					const double x1 = (positions[last - 1] + step * 0.5 - left) /
									  std::max(0.001, scale);
					const double y0 =
							static_cast<double>(rows_hidden) / std::max(0.001, scale);
					const double y1 = height;
					const auto panel = gather_region(fit, image->second, x0, x1, y0, y1);
					if (!panel)
						continue;
					const auto &[x, z] = run.cells[piece];
					placed += submit_panel(editor, x + run.normal.first,
							run.base_y + rows_hidden, z + run.normal.second,
							world_editor::WorldEditor::facing_for_normal(
									run.normal.first, run.normal.second),
							*panel);
				}
			}
			begin = end;
		}
	}
	return placed;
}

building_facade::PointSet collect(world_editor::WorldEditor &editor,
		const std::vector<ProcessedNode> &nodes, std::uint64_t way_id,
		std::uint64_t group_seed, buildings::BuildingCategory category,
		const building_facade::FacadePlan &plan,
		const CoordinateBitmap *building_footprints,
		const building_facade::PointSet &own_fill, bool ground_level, int base_y,
		int building_height, double scale)
{
	if (nodes.size() < 3 || building_height < MIN_WALL_BLOCKS)
		return {};
	if (!editor.map_decals_enabled())
		return {};
	std::optional<FacadeSet> set;
	std::optional<std::array<int, 4>> extent;
	bool first = false;
	{
		auto &s = state();
		std::lock_guard lock(s.mutex);
		if (!s.enabled || !s.set)
			return {};
		first = s.claimed.insert(way_id).second;
		set = s.set;
		extent = s.extent;
	}
	int anchor_x = nodes.front().x, anchor_z = nodes.front().z;
	for (const auto &node : nodes) {
		anchor_x = std::min(anchor_x, node.x);
		anchor_z = std::min(anchor_z, node.z);
	}
	const auto choice = choose(set->entries(), category,
			static_cast<double>(building_height) / std::max(0.001, scale), way_id,
			anchor_x, anchor_z);
	if (!choice)
		return {};
	std::vector<PendingRun> collected_runs;
	for (std::size_t i = 0; i < plan.segments.size() && i + 1 < nodes.size(); ++i) {
		const auto &segment = plan.segments[i];
		if (!segment)
			continue;
		auto cells = line(nodes[i].x, nodes[i].z, nodes[i + 1].x, nodes[i + 1].z);
		if (cells.empty())
			continue;
		const auto inside = [&](const auto &cell) {
			const int x = cell.first + segment->normal.first;
			const int z = cell.second + segment->normal.second;
			return !extent || (x >= (*extent)[0] && z >= (*extent)[1] &&
									  x <= (*extent)[2] && z <= (*extent)[3]);
		};
		// Retain separate street/party runs: party-wall visibility is evaluated
		// against the completed neighbouring buildings in finalize().
		for (std::size_t first = 0; first < cells.size();) {
			while (first < cells.size() &&
					(!inside(cells[first]) ||
							mapillary::facades::photo_column(
									cells[first].first, cells[first].second, way_id)))
				++first;
			if (first == cells.size())
				break;
			std::size_t open_end = first + 1;
			const auto is_party = [&](const auto &cell) {
				if (!ground_level || !building_footprints)
					return false;
				for (int distance = 1; distance <= 2; ++distance) {
					const std::pair neighbor{
							cell.first + segment->normal.first * distance,
							cell.second + segment->normal.second * distance};
					if (building_footprints->contains(neighbor.first, neighbor.second) &&
							!own_fill.contains(neighbor))
						return true;
				}
				return false;
			};
			const bool party = is_party(cells[first]);
			while (open_end < cells.size() && inside(cells[open_end]) &&
					!mapillary::facades::photo_column(
							cells[open_end].first, cells[open_end].second, way_id) &&
					is_party(cells[open_end]) == party)
				++open_end;
			if (open_end - first < MIN_RUN_CELLS) {
				first = open_end;
				continue;
			}
			PendingRun pending_run;
			pending_run.way_id = way_id;
			pending_run.group = osm_parser::seed_without_hint(group_seed);
			pending_run.entry = choice->entry;
			pending_run.choice = *choice;
			pending_run.cells.assign(cells.begin() + first, cells.begin() + open_end);
			pending_run.dir = {nodes[i + 1].x - nodes[i].x, nodes[i + 1].z - nodes[i].z};
			pending_run.normal = segment->normal;
			pending_run.base_y = base_y;
			pending_run.height = building_height;
			pending_run.party = party;
			collected_runs.push_back(std::move(pending_run));
			first = open_end;
		}
	}
	if (collected_runs.empty())
		return {};
	building_facade::PointSet shell;
	for (const auto &run : collected_runs)
		shell.insert(run.cells.begin(), run.cells.end());
	// Every tile that walks this building needs the same flat-shell mask, while
	// only the first visit records image work for finalization.
	if (!first)
		return shell;
	const auto group_id = osm_parser::seed_without_hint(group_seed);
	const double wall_height_m =
			static_cast<double>(building_height) / std::max(0.001, scale);
	{
		auto &s = state();
		std::lock_guard lock(s.mutex);
		s.pending.insert(s.pending.end(), std::make_move_iterator(collected_runs.begin()),
				std::make_move_iterator(collected_runs.end()));
		auto [group, inserted] = s.groups.try_emplace(
				group_id, GroupPick{wall_height_m, way_id, *choice, 1});
		if (!inserted) {
			++group->second.members;
			if (wall_height_m > group->second.wall_height_m ||
					(wall_height_m == group->second.wall_height_m &&
							way_id < group->second.element_id)) {
				group->second.wall_height_m = wall_height_m;
				group->second.element_id = way_id;
				group->second.choice = *choice;
			}
		}
	}
	return shell;
}
} // namespace arnis::building_facades
