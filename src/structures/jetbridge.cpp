#include "jetbridge.h"
#include "schem_decoder.h"

#include <cmath>
#include <fstream>

namespace arnis::structures::jetbridge
{
namespace
{
constexpr double TERMINAL_X = 2.0, TERMINAL_Z = 0.0;
constexpr double CAB_X = 11.0, CAB_Z = 14.0;
constexpr double MINIMUM_LENGTH = 12.0;
constexpr int FOOTPRINT_PROBE = 4;

std::optional<SchemDocument> load_asset()
{
	const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() /
					  "assets/structures/jetbridge.schem";
	std::ifstream stream(path, std::ios::binary);
	if (!stream)
		return std::nullopt;
	std::vector<std::uint8_t> bytes(
			(std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
	try {
		return decode_sponge_schem(bytes);
	} catch (...) {
		return std::nullopt;
	}
}

std::pair<const ProcessedNode *, const ProcessedNode *> terminal_end(
		const ProcessedNode &first, const ProcessedNode &last,
		const BuildingFootprintBitmap &footprints)
{
	const bool first_on = footprints.contains(first.x, first.z);
	const bool last_on = footprints.contains(last.x, last.z);
	if (first_on != last_on)
		return first_on ? std::pair{&first, &last} : std::pair{&last, &first};
	auto nearby = [&](const ProcessedNode &node) {
		std::size_t hits = 0;
		for (int dz = -FOOTPRINT_PROBE; dz <= FOOTPRINT_PROBE; ++dz)
			for (int dx = -FOOTPRINT_PROBE; dx <= FOOTPRINT_PROBE; ++dx)
				hits += footprints.contains(node.x + dx, node.z + dz);
		return hits;
	};
	return nearby(last) > nearby(first) ? std::pair{&last, &first}
										: std::pair{&first, &last};
}

}

bool claims(const ProcessedWay &way)
{
	if (way.tags.get("aeroway") != std::optional<std::string>("jet_bridge") ||
			way.tags.get("area") == std::optional<std::string>("yes") ||
			way.tags.get("indoor") == std::optional<std::string>("yes") ||
			way.nodes.size() < 2)
		return false;
	return way.nodes.front().id != way.nodes.back().id;
}

void generate_jet_bridge(WorldEditor &editor, const ProcessedWay &way,
		const BuildingFootprintBitmap &building_footprints)
{
	if (!editor.place_schematics() || !claims(way))
		return;
	const auto document = load_asset();
	if (!document)
		return;
	const auto [terminal, apron] =
			terminal_end(way.nodes.front(), way.nodes.back(), building_footprints);
	const double dx = double(apron->x - terminal->x), dz = double(apron->z - terminal->z);
	if (std::hypot(dx, dz) < MINIMUM_LENGTH)
		return;
	const double yaw =
			std::atan2(dz, dx) - std::atan2(CAB_Z - TERMINAL_Z, CAB_X - TERMINAL_X);
	const double s = std::sin(yaw), c = std::cos(yaw);
	const double ux = double(document->width - 1) * .5 - TERMINAL_X;
	const double uz = double(document->length - 1) * .5 - TERMINAL_Z;
	const int base_x = int(std::lround(double(terminal->x) + ux * c - uz * s));
	const int base_z = int(std::lround(double(terminal->z) + ux * s + uz * c));
	if (editor.is_lc_water(base_x, base_z))
		return;
	place_schem_document_yaw(editor, *document, base_x,
			editor.get_absolute_y(base_x, 1, base_z), base_z, yaw * 180.0 / M_PI);
}
}
