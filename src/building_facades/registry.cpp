#include "registry.h"

#include "../../../arnis_world_editor.h"
#include "../element_processing/buildings.h"
#include "manifest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <unordered_set>

namespace arnis::building_facades
{
namespace
{
struct State
{
	std::mutex mutex;
	bool enabled = false;
	double scale = 1.0;
	std::uint32_t ppm = 16;
	std::filesystem::path directory;
	std::optional<FacadeSet> set;
	std::unordered_set<std::uint64_t> claimed;
	std::optional<std::array<int, 4>> extent;
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

void collect(world_editor::WorldEditor &editor, const std::vector<ProcessedNode> &nodes,
		std::uint64_t way_id, buildings::BuildingCategory category,
		const building_facade::FacadePlan &plan, int base_y, int building_height,
		double scale)
{
	if (nodes.size() < 3 || building_height < 4)
		return;
	if (!editor.map_decals_enabled())
		return;
	std::filesystem::path directory;
	std::optional<FacadeSet> set;
	std::uint32_t ppm = 16;
	std::optional<std::array<int, 4>> extent;
	{
		auto &s = state();
		std::lock_guard lock(s.mutex);
		if (!s.enabled || !s.set || !s.claimed.insert(way_id).second)
			return;
		ppm = s.ppm;
		directory = s.directory;
		set = s.set;
		extent = s.extent;
	}
	const auto choice = choose(set->entries(), category,
			static_cast<double>(building_height) / std::max(0.001, scale), way_id,
			nodes.front().x, nodes.front().z);
	if (!choice)
		return;
	const auto &entry = set->entries()[choice->entry];
	const auto image = load_image(directory, entry, ppm);
	if (!image)
		return;
	for (std::size_t i = 0; i < plan.segments.size() && i + 1 < nodes.size(); ++i) {
		const auto &segment = plan.segments[i];
		if (!segment || segment->facade_class != building_facade::FacadeClass::Street)
			continue;
		auto cells = line(nodes[i].x, nodes[i].z, nodes[i + 1].x, nodes[i + 1].z);
		if (cells.empty())
			continue;
		const double height =
				static_cast<double>(building_height) / std::max(0.001, scale);
		const double run_width =
				static_cast<double>(cells.size()) / std::max(0.001, scale);
		const Fit fit = Fit::make(entry, ppm, run_width, height, choice->phase_m);
		const int piece_cells = std::max(
				1, static_cast<int>(
						   std::lround(entry.metres_wide * std::max(0.001, scale))));
		for (std::size_t first = 0; first < cells.size(); first += piece_cells) {
			const std::size_t last = std::min(cells.size(), first + piece_cells);
			const double x0 = static_cast<double>(first) / std::max(0.001, scale);
			const double x1 = static_cast<double>(last) / std::max(0.001, scale);
			const auto panel = gather_region(fit, *image, x0, x1, 0.0, height);
			if (!panel)
				continue;
			const auto &anchor = cells[first];
			const auto &end = cells[last - 1];
			if (extent) {
				const auto inside = [&](const auto &cell, int outward_x, int outward_z) {
					const int x = cell.first + outward_x;
					const int z = cell.second + outward_z;
					return x >= (*extent)[0] && z >= (*extent)[1] && x <= (*extent)[2] &&
						   z <= (*extent)[3];
				};
				if (!inside(anchor, segment->normal.first, segment->normal.second) ||
						!inside(end, segment->normal.first, segment->normal.second))
					continue;
			}
			submit_panel(editor, anchor.first + segment->normal.first, base_y,
					anchor.second + segment->normal.second,
					world_editor::WorldEditor::facing_for_normal(
							segment->normal.first, segment->normal.second),
					*panel);
		}
	}
}
} // namespace arnis::building_facades
