#include "structures.h"
#include "../assets_root.h"
#include "../land_cover/land_cover.h"
#include "schem_decoder.h"
#include <algorithm>
#include <array>
#include <fstream>
#include <iterator>
#include <limits>
#include <mutex>

namespace arnis::structures::car
{
namespace
{
void place_car(WorldEditor &editor, int cx, int cz, int base_y, uint8_t rot_base,
		std::uint64_t h, int max_height)
{
	if (!editor.place_schematics())
		return;
	if (h % 100 >= 50)
		return;
	static constexpr const char *models[] = {"car_fedex", "car_hotrod_white",
			"car_hotrod_blue", "car_police", "car_uhaul", "car_workvan", "car_camper",
			"car_pickup", "car_suv", "car_sedan"};
	// Rust rotates models whose footprint is wider than it is long so their
	// canonical axis follows the parking-space direction. Decode and retain each
	// model once so both selection and repeated placement use Rust-like caching.
	static std::array<StructureSchematic, 10> cars{};
	static std::array<int, 10> alignments{};
	static std::array<int, 10> heights{};
	static std::array<bool, 10> available{};
	static std::once_flag dimensions_loaded;
	std::call_once(dimensions_loaded, [&] {
		const auto root = editor.get_schematic_asset_root().empty()
								  ? assets::path("structures")
								  : editor.get_schematic_asset_root();
		for (std::size_t i = 0; i < std::size(models); ++i) {
			try {
				std::ifstream input(
						root / (std::string(models[i]) + ".schem"), std::ios::binary);
				std::vector<std::uint8_t> bytes(
						(std::istreambuf_iterator<char>(input)), {});
				if (!bytes.empty()) {
					auto schem = load_structure(bytes);
					alignments[i] = schem.width > schem.length ? 1 : 0;
					for (const auto &voxel : schem.voxels)
						heights[i] = std::max(heights[i], voxel.y + 1);
					available[i] = !schem.voxels.empty();
					if (available[i])
						cars[i] = schem.centered();
				}
			} catch (...) {
				alignments[i] = 0;
			}
		}
	});
	std::array<unsigned, 10> pool{};
	unsigned pool_size = 0;
	for (unsigned i = 0; i < available.size(); ++i)
		if (available[i] && heights[i] <= max_height)
			pool[pool_size++] = i;
	if (pool_size == 0)
		return;
	const unsigned model_index = pool[static_cast<unsigned>((h >> 8) % pool_size)];
	const unsigned align = static_cast<unsigned>(alignments[model_index]);
	const auto flip = ((h >> 16) & 1) ? 2 : 0;
	place_structure(
			editor, cars[model_index], cx, base_y, cz, (rot_base + align + flip) & 3);
}
} // namespace

void maybe_place_car(WorldEditor &editor, int cx, int cz, uint8_t rot_base)
{
	place_car(editor, cx, cz, editor.get_absolute_y(cx, 1, cz), rot_base,
			land_cover::coord_hash(cx, cz), std::numeric_limits<int>::max());
}

void maybe_place_car_on_deck(
		WorldEditor &editor, int cx, int cz, int base_y, uint8_t rot_base, int max_height)
{
	const auto x = static_cast<std::uint32_t>(cx) ^
				   (static_cast<std::uint32_t>(base_y) * 0x2F1Bu);
	const auto h = land_cover::coord_hash(static_cast<std::int32_t>(x), cz);
	place_car(editor, cx, cz, base_y, rot_base, h, max_height);
}
}
