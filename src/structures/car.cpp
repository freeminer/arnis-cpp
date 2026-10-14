#include "structures.h"
#include "../assets_root.h"
#include "../land_cover/land_cover.h"
#include "schem_decoder.h"
#include <algorithm>
#include <array>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace arnis::structures::car
{
namespace
{
struct CarFleet
{
	std::array<StructureSchematic, 10> cars{};
	std::array<int, 10> alignments{};
	std::array<int, 10> heights{};
	std::array<bool, 10> available{};
};

std::shared_ptr<const CarFleet> load_fleet(const std::filesystem::path &root)
{
	static std::mutex cache_mutex;
	static std::unordered_map<std::string, std::shared_ptr<const CarFleet>> cache;
	const auto key = std::filesystem::absolute(root).lexically_normal().string();
	std::lock_guard lock(cache_mutex);
	if (const auto found = cache.find(key); found != cache.end())
		return found->second;

	static constexpr const char *models[] = {"car_fedex", "car_hotrod_white",
			"car_hotrod_blue", "car_police", "car_uhaul", "car_workvan", "car_camper",
			"car_pickup", "car_suv", "car_sedan"};
	auto fleet = std::make_shared<CarFleet>();
	for (std::size_t i = 0; i < std::size(models); ++i) {
		const auto schematic =
				try_load_structure_cached(root / (std::string(models[i]) + ".schem"));
		if (!schematic)
			continue;
		fleet->alignments[i] = schematic->width > schematic->length ? 1 : 0;
		for (const auto &voxel : schematic->voxels)
			fleet->heights[i] = std::max(fleet->heights[i], voxel.y + 1);
		fleet->available[i] = !schematic->voxels.empty();
		if (fleet->available[i])
			fleet->cars[i] = schematic->centered();
	}
	return cache.emplace(key, std::move(fleet)).first->second;
}

void place_car(WorldEditor &editor, int cx, int cz, int base_y, uint8_t rot_base,
		std::uint64_t h, int max_height)
{
	if (!editor.place_schematics())
		return;
	if (h % 100 >= 50)
		return;
	const auto root = editor.get_schematic_asset_root().empty()
							  ? assets::path("structures")
							  : editor.get_schematic_asset_root();
	const auto fleet = load_fleet(root);
	std::array<unsigned, 10> pool{};
	unsigned pool_size = 0;
	for (unsigned i = 0; i < fleet->available.size(); ++i)
		if (fleet->available[i] && fleet->heights[i] <= max_height)
			pool[pool_size++] = i;
	if (pool_size == 0)
		return;
	const unsigned model_index = pool[static_cast<unsigned>((h >> 8) % pool_size)];
	const unsigned align = static_cast<unsigned>(fleet->alignments[model_index]);
	const auto flip = ((h >> 16) & 1) ? 2 : 0;
	place_structure(editor, fleet->cars[model_index], cx, base_y, cz,
			(rot_base + align + flip) & 3);
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
