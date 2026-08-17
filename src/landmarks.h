#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace arnis { class ProcessedElement; }
namespace arnis::world_editor { struct WorldEditor; }

namespace arnis::landmarks {

// Static metadata mirrors landmarks.rs.  The schematic path is deliberately
// kept alongside the placement data so a library client can decide when and
// how to materialise the model.
struct Landmark {
	const char *name;
	const char *qid;
	std::vector<std::pair<std::string, std::int64_t>> osm_ids;
	const char *schematic_path;
	double latitude, longitude;
	double anchor_x, anchor_z;
	int ground_y, ground_offset;
	std::vector<std::string> interior_markers;
	int ground_overlap;
	double suppress_half_x, suppress_half_z;
	int reach_m;
};

// Projection is owned by the map host.  Supplying resolved world anchors
// preserves the Rust prescan's separation between coordinate transformation
// and landmark matching, without tying this library to a particular exporter.
struct WorldAnchor { const char *qid; int x, z; };
struct Placement { const Landmark *landmark=nullptr; int world_x=0, world_z=0; };
struct PrescanResult {
	std::vector<Placement> placements;
	std::vector<std::pair<std::string, std::int64_t>> suppressed;
	std::vector<std::pair<int, int>> deferred_regions;
};

const std::vector<Landmark> &catalogue();
PrescanResult prescan(const std::vector<ProcessedElement> &elements,
		const std::vector<WorldAnchor> &anchors, double scale,
		const std::vector<std::pair<std::string, std::int64_t>> &already_suppressed = {});
// Stamp one resolved landmark after ground generation.  Placement intentionally
// uses the generic Sponge decoder rather than an exporter-specific writer.
bool place(world_editor::WorldEditor &, const Placement &, double scale,
		unsigned rotation_quarters = 0);
std::size_t place_all(world_editor::WorldEditor &, const PrescanResult &, double scale,
		unsigned rotation_quarters = 0);

} // namespace arnis::landmarks
