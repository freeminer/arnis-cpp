#include "structures.h"
#include "../land_cover/land_cover.h"
#include "schem_decoder.h"
#include <array>
#include <fstream>
#include <iterator>

namespace arnis::structures::car
{
void maybe_place_car(WorldEditor &editor, int cx, int cz, uint8_t rot_base)
{
	if (!editor.place_schematics())
		return;
	const auto h = land_cover::coord_hash(cx, cz);
	if (h % 100 >= 50)
		return;
	static constexpr const char *models[] = {"car_fedex", "car_hotrod_white",
			"car_hotrod_blue", "car_police", "car_uhaul", "car_workvan",
			"car_camper", "car_pickup", "car_suv", "car_sedan"};
	const auto file = editor.get_schematic_asset_root() / (std::string(models[(h >> 8) % 10]) + ".schem");
	// Rust rotates models whose footprint is wider than it is long so their
	// canonical axis follows the parking-space direction. Decode dimensions
	// once per process; placement itself remains the cheap cached-file path.
	static std::array<int, 10> alignments{};
	static bool dimensions_loaded = false;
	if (!dimensions_loaded) {
		for (std::size_t i = 0; i < std::size(models); ++i) {
			try {
				std::ifstream input(editor.get_schematic_asset_root() /
						(std::string(models[i]) + ".schem"), std::ios::binary);
				std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)), {});
				if (!bytes.empty()) {
					const auto doc = decode_sponge_schem(bytes);
					alignments[i] = doc.width > doc.length ? 1 : 0;
				}
			} catch (...) {
				alignments[i] = 0;
			}
		}
		dimensions_loaded = true;
	}
	const unsigned model_index = static_cast<unsigned>((h >> 8) % 10);
	const unsigned align = static_cast<unsigned>(alignments[model_index]);
	const auto flip = ((h >> 16) & 1) ? 2 : 0;
	place_schem_file_anchored(editor, file, cx, editor.get_absolute_y(cx, 1, cz), cz,
			(rot_base + align + flip) & 3, SchemAnchor::Centered);
}
}
