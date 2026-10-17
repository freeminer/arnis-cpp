#include "landmarks.h"
#include "assets_root.h"
#include "../../arnis_adapter.h"
#include "structures/schem_decoder.h"
#include "world_editor/floor_state.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <tuple>

namespace arnis::landmarks
{
namespace
{
using Key = std::pair<std::string, std::uint64_t>;

bool contains(const std::vector<Key> &keys, const Key &key)
{
	return std::find(keys.begin(), keys.end(), key) != keys.end();
}

std::string tag(const ProcessedElement &element, const char *name)
{
	auto it = element.tags().find(name);
	return it == element.tags().end() ? std::string{} : it->second;
}

std::string trim(std::string value)
{
	const auto first = value.find_first_not_of(" \t\r\n");
	if (first == std::string::npos)
		return {};
	const auto last = value.find_last_not_of(" \t\r\n");
	return value.substr(first, last - first + 1);
}

std::vector<std::pair<int, int>> points(const ProcessedElement &element)
{
	std::vector<std::pair<int, int>> result;
	if (element.is_node())
		result.push_back({element.as_node().x, element.as_node().z});
	else if (element.is_way())
		for (const auto &node : element.as_way().nodes)
			result.push_back({node.x, node.z});
	else if (element.is_relation())
		for (const auto &member : element.as_relation().members)
			for (const auto &node : member.way->nodes)
				result.push_back({node.x, node.z});
	return result;
}

bool replaceable(const ProcessedElement &element)
{
	const auto tags = element.tags();
	if (tags.find("building") != tags.end() || tags.find("building:part") != tags.end())
		return true;
	const auto leisure = tag(element, "leisure");
	return leisure == "stadium" || leisure == "pitch" || leisure == "track" ||
		   leisure == "sports_centre";
}

std::vector<std::pair<int, int>> regions_around(int x, int z, int radius)
{
	// Rust's region-key conversion floors across zero.  A signed right shift
	// truncates differently on some hosts (and is implementation-defined), so
	// keep the landmark residency set correct for western/southern coordinates.
	auto region_floor = [](int value) {
		const auto wide = static_cast<long long>(value);
		return static_cast<int>(wide >= 0 ? wide / 512 : -((-wide + 511) / 512));
	};
	std::vector<std::pair<int, int>> out;
	for (int rz = region_floor(z - radius); rz <= region_floor(z + radius); ++rz)
		for (int rx = region_floor(x - radius); rx <= region_floor(x + radius); ++rx)
			out.push_back({rx, rz});
	return out;
}

std::pair<double, double> rotate(double x, double z, double degrees)
{
	constexpr double pi = 3.14159265358979323846;
	const double radians = degrees * pi / 180.0;
	const double cosine = std::cos(radians);
	const double sine = std::sin(radians);
	return {x * cosine - z * sine, x * sine + z * cosine};
}

std::filesystem::path asset_path(const Landmark &landmark)
{
	return assets::path(landmark.schematic_path);
}

int median_base_y(WorldEditor &editor, const Placement &placement, double scale,
		double rotation_degrees)
{
	std::vector<int> levels;
	levels.reserve(289);
	const auto &landmark = *placement.landmark;
	for (int ix = 0; ix <= 16; ++ix)
		for (int iz = 0; iz <= 16; ++iz) {
			const double mx = landmark.anchor_x +
							  landmark.suppress_half_x * (2.0 * ix / 16.0 - 1.0);
			const double mz = landmark.anchor_z +
							  landmark.suppress_half_z * (2.0 * iz / 16.0 - 1.0);
			auto [dx, dz] = rotate((mx - landmark.anchor_x) * scale,
					(mz - landmark.anchor_z) * scale, rotation_degrees);
			levels.push_back(
					editor.get_ground_level(int(std::lround(placement.world_x + dx)),
							int(std::lround(placement.world_z + dz))));
		}
	std::nth_element(levels.begin(), levels.begin() + levels.size() / 2, levels.end());
	return std::max(levels[levels.size() / 2] + 1 + landmark.ground_offset,
			world_editor::min_y() + 2);
}

using ModelVoxel = structures::ColumnVoxel;

struct LandmarkModel
{
	std::shared_ptr<const structures::PalettizedSchematic> schematic;
	// Sorted indices make columns contiguous without duplicating the compact
	// palette-indexed voxel records held by the shared asset cache.
	std::vector<std::uint32_t> voxel_order;
	std::vector<std::pair<std::uint32_t, std::uint32_t>> columns;
	std::vector<std::uint8_t> excavated;
	int min_x = 0, max_x = -1, min_z = 0, max_z = -1, max_y = 0;

	std::size_t slot(int x, int z) const
	{
		return static_cast<std::size_t>(x - min_x) +
			   static_cast<std::size_t>(z - min_z) * (max_x - min_x + 1);
	}
	std::pair<const std::uint32_t *, std::size_t> column(int x, int z) const
	{
		if (!schematic || x < min_x || x > max_x || z < min_z || z > max_z)
			return {nullptr, 0};
		const auto [start, count] = columns[slot(x, z)];
		if (!count)
			return {nullptr, 0};
		return {voxel_order.data() + start, static_cast<std::size_t>(count)};
	}
	const ModelVoxel &voxel(std::uint32_t order_index) const
	{
		return schematic->voxels[voxel_order[order_index]];
	}
};

LandmarkModel make_model(std::shared_ptr<const structures::PalettizedSchematic> schematic,
		const Landmark &landmark)
{
	LandmarkModel model;
	model.schematic = std::move(schematic);
	if (!model.schematic || model.schematic->voxels.empty())
		return model;
	const auto &voxels = model.schematic->voxels;
	model.min_x = model.max_x = voxels.front().x;
	model.min_z = model.max_z = voxels.front().z;
	for (const auto &voxel : voxels) {
		model.min_x = std::min(model.min_x, int(voxel.x));
		model.max_x = std::max(model.max_x, int(voxel.x));
		model.min_z = std::min(model.min_z, int(voxel.z));
		model.max_z = std::max(model.max_z, int(voxel.z));
		model.max_y = std::max(model.max_y, int(voxel.y));
	}
	const auto width = static_cast<std::size_t>(model.max_x - model.min_x + 1);
	const auto length = static_cast<std::size_t>(model.max_z - model.min_z + 1);
	const auto cell_count = width * length;
	model.columns.assign(cell_count, {0, 0});
	model.voxel_order.resize(voxels.size());
	std::iota(model.voxel_order.begin(), model.voxel_order.end(), 0);
	std::sort(model.voxel_order.begin(), model.voxel_order.end(), [&](auto a, auto b) {
		const auto &lhs = voxels[a];
		const auto &rhs = voxels[b];
		return std::tie(lhs.x, lhs.z, lhs.y) < std::tie(rhs.x, rhs.z, rhs.y);
	});
	std::vector<std::uint8_t> marker(cell_count, 0);
	std::vector<Block> marker_blocks;
	for (const auto &name : landmark.interior_markers)
		marker_blocks.push_back(structures::resolve_schem_block(name));
	for (std::size_t start = 0; start < model.voxel_order.size();) {
		const auto &first = model.voxel(static_cast<std::uint32_t>(start));
		const auto x = first.x;
		const auto z = first.z;
		std::size_t end = start + 1;
		while (end < model.voxel_order.size() &&
				model.voxel(static_cast<std::uint32_t>(end)).x == x &&
				model.voxel(static_cast<std::uint32_t>(end)).z == z)
			++end;
		const auto index = model.slot(x, z);
		model.columns[index] = {static_cast<std::uint32_t>(start),
				static_cast<std::uint32_t>(end - start)};
		for (std::size_t i = start; i < end; ++i) {
			const auto &voxel = model.voxel(static_cast<std::uint32_t>(i));
			if (std::find(marker_blocks.begin(), marker_blocks.end(),
						model.schematic->palette[voxel.palette_slot].block) !=
					marker_blocks.end())
				marker[index] = 1;
		}
		start = end;
	}

	// Flood the non-marker cells from the model boundary; enclosed space is the
	// landmark interior. This mirrors Rust's marker-ring excavation semantics.
	if (marker_blocks.empty()) {
		model.excavated.assign(cell_count, 1);
	} else {
		std::vector<std::uint8_t> outside(cell_count, 0);
		std::vector<std::size_t> stack;
		stack.reserve(cell_count);
		auto push = [&](std::size_t index) {
			if (!marker[index] && !outside[index]) {
				outside[index] = 1;
				stack.push_back(index);
			}
		};
		for (std::size_t x = 0; x < width; ++x) {
			push(x);
			push((length - 1) * width + x);
		}
		for (std::size_t z = 0; z < length; ++z) {
			push(z * width);
			push(z * width + width - 1);
		}
		while (!stack.empty()) {
			const auto index = stack.back();
			stack.pop_back();
			const auto x = index % width, z = index / width;
			if (x > 0)
				push(index - 1);
			if (x + 1 < width)
				push(index + 1);
			if (z > 0)
				push(index - width);
			if (z + 1 < length)
				push(index + width);
		}
		model.excavated.resize(cell_count);
		for (std::size_t i = 0; i < cell_count; ++i)
			model.excavated[i] = !outside[i];
		const auto radius =
				static_cast<std::size_t>(std::max(0, landmark.ground_overlap));
		if (radius) {
			std::vector<std::uint8_t> eroded(cell_count, 0), second(cell_count, 0);
			for (std::size_t z = 0; z < length; ++z)
				for (std::size_t x = radius; x + radius < width; ++x) {
					bool inside = true;
					for (std::size_t k = x - radius; k <= x + radius && inside; ++k)
						inside = model.excavated[z * width + k] != 0;
					eroded[z * width + x] = inside;
				}
			for (std::size_t z = radius; z + radius < length; ++z)
				for (std::size_t x = 0; x < width; ++x) {
					bool inside = true;
					for (std::size_t k = z - radius; k <= z + radius && inside; ++k)
						inside = eroded[k * width + x] != 0;
					second[z * width + x] = inside;
				}
			model.excavated.swap(second);
		}
	}
	return model;
}
} // namespace

const std::vector<Landmark> &catalogue()
{
	static const std::vector<Landmark> entries = {
			{"Olympiastadion M\xC3\xBCnchen", "Q131610", {{"way", 419656920}},
					"assets/structures/landmarks/olympiastadion_munich.schem", 48.1731012,
					11.5464833, 224.5, 238.0, 12, -20,
					{"green_wool", "green_stained_hardened_clay"}, 10, 135., 145., 240},
			{"Olympiahalle", "Q48849", {{"way", 303099272}},
					"assets/structures/landmarks/olympiahalle_munich.schem", 48.1749058,
					11.5500308, 115., 130., 0, -9, {"white_concrete"}, 0, 95., 75., 160},
			{"Olympia-Schwimmhalle", "Q3882013", {{"way", 227012665}},
					"assets/structures/landmarks/olympia_schwimmhalle_munich.schem",
					48.1735723, 11.5514797, 99., 126.5, 0, -6, {"glass"}, 0, 50., 70.,
					160},
			{"Olympiaturm", "Q599148", {{"way", 164084344}},
					"assets/structures/landmarks/olympiaturm_munich.schem", 48.1744095,
					11.5537401, 11.5, 13.5, 0, 0, {}, 0, 22., 22., 30},
	};
	return entries;
}

PrescanResult prescan(const std::vector<ProcessedElement> &elements,
		const std::vector<WorldAnchor> &anchors, const WorldBounds &world_bounds,
		double scale, double rotation_degrees, const std::vector<Key> &already_suppressed)
{
	PrescanResult result;
	if (scale <= 0.0 || !std::isfinite(scale) || !std::isfinite(rotation_degrees))
		return result;
	constexpr double pi = 3.14159265358979323846;
	const double rotation = rotation_degrees * pi / 180.0;
	const double sin_rotation = std::sin(rotation);
	const double cos_rotation = std::cos(rotation);
	for (const auto &landmark : catalogue()) {
		auto anchor = std::find_if(
				anchors.begin(), anchors.end(), [&](const WorldAnchor &value) {
					return std::string(value.qid ? value.qid : "") == landmark.qid;
				});
		if (anchor == anchors.end())
			continue;
		const int radius = static_cast<int>(std::ceil(landmark.reach_m * scale));
		if (anchor->x + radius < world_bounds.min_x ||
				anchor->x - radius > world_bounds.max_x ||
				anchor->z + radius < world_bounds.min_z ||
				anchor->z - radius > world_bounds.max_z)
			continue;
		const Placement placement{&landmark, anchor->x, anchor->z};
		result.placements.push_back(placement);
		auto regions = regions_around(anchor->x, anchor->z, radius);
		result.deferred_regions.insert(
				result.deferred_regions.end(), regions.begin(), regions.end());
		for (const auto &element : elements) {
			const Key key{std::string(element.kind()), element.id()};
			if (contains(already_suppressed, key) || contains(result.suppressed, key))
				continue;
			const bool is_landmark = trim(tag(element, "wikidata")) == landmark.qid ||
									 contains(landmark.osm_ids, key);
			if (is_landmark) {
				result.suppressed.push_back(key);
				continue;
			}
			if (!replaceable(element))
				continue;
			const auto geometry = points(element);
			if (geometry.empty())
				continue;
			long long sum_x = 0, sum_z = 0;
			for (const auto &[x, z] : geometry) {
				sum_x += x;
				sum_z += z;
			}
			const double dx =
					(sum_x / static_cast<double>(geometry.size()) - anchor->x) / scale;
			const double dz =
					(sum_z / static_cast<double>(geometry.size()) - anchor->z) / scale;
			// Rust maps world-space feature centres back into the schematic's
			// coordinate frame before testing its replacement rectangle. This
			// inverse rotation is essential for rotated Arnis maps; an axis-aligned
			// world-space test can otherwise miss the building the model covers.
			const double model_dx = dx * cos_rotation + dz * sin_rotation;
			const double model_dz = -dx * sin_rotation + dz * cos_rotation;
			if (std::abs(model_dx) <= landmark.suppress_half_x &&
					std::abs(model_dz) <= landmark.suppress_half_z)
				result.suppressed.push_back(key);
		}
	}
	std::sort(result.suppressed.begin(), result.suppressed.end());
	result.suppressed.erase(
			std::unique(result.suppressed.begin(), result.suppressed.end()),
			result.suppressed.end());
	std::sort(result.deferred_regions.begin(), result.deferred_regions.end());
	result.deferred_regions.erase(
			std::unique(result.deferred_regions.begin(), result.deferred_regions.end()),
			result.deferred_regions.end());
	return result;
}

bool place(WorldEditor &editor, const Placement &placement, double scale,
		double rotation_degrees)
{
	if (!placement.landmark || scale <= 0.0 || !std::isfinite(scale) ||
			!std::isfinite(rotation_degrees) || !editor.place_schematics())
		return false;
	std::shared_ptr<const structures::PalettizedSchematic> schematic;
	try {
		schematic = structures::load_palettized_cached(asset_path(*placement.landmark));
	} catch (...) {
		return false;
	}
	if (!schematic)
		return false;
	const auto model = make_model(std::move(schematic), *placement.landmark);
	if (!model.schematic || model.schematic->voxels.empty() || model.columns.empty())
		return false;
	const int base = median_base_y(editor, placement, scale, rotation_degrees);
	const auto [min_x, min_z] = editor.get_min_coords();
	const auto [max_x, max_z] = editor.get_max_coords();
	auto to_world = [&](double mx, double mz) {
		return rotate((mx - placement.landmark->anchor_x) * scale,
				(mz - placement.landmark->anchor_z) * scale, rotation_degrees);
	};
	auto to_model = [&](double wx, double wz) {
		const double dx = (wx - placement.world_x) / scale;
		const double dz = (wz - placement.world_z) / scale;
		constexpr double pi = 3.14159265358979323846;
		const double radians = rotation_degrees * pi / 180.0;
		const double cosine = std::cos(radians), sine = std::sin(radians);
		return std::make_pair(placement.landmark->anchor_x + dx * cosine + dz * sine,
				placement.landmark->anchor_z - dx * sine + dz * cosine);
	};
	int world_min_x = std::numeric_limits<int>::max();
	int world_max_x = std::numeric_limits<int>::min();
	int world_min_z = std::numeric_limits<int>::max();
	int world_max_z = std::numeric_limits<int>::min();
	for (const auto &[mx, mz] :
			{std::pair{model.min_x, model.min_z}, std::pair{model.max_x + 1, model.min_z},
					std::pair{model.min_x, model.max_z + 1},
					std::pair{model.max_x + 1, model.max_z + 1}}) {
		auto [dx, dz] = to_world(mx, mz);
		world_min_x = std::min(
				world_min_x, static_cast<int>(std::floor(placement.world_x + dx)));
		world_max_x = std::max(
				world_max_x, static_cast<int>(std::ceil(placement.world_x + dx)));
		world_min_z = std::min(
				world_min_z, static_cast<int>(std::floor(placement.world_z + dz)));
		world_max_z = std::max(
				world_max_z, static_cast<int>(std::ceil(placement.world_z + dz)));
	}
	world_min_x = std::max(world_min_x, min_x);
	world_max_x = std::min(world_max_x, max_x);
	world_min_z = std::max(world_min_z, min_z);
	world_max_z = std::min(world_max_z, max_z);
	const auto [writable_min_y, world_ceiling] = editor.writable_y_bounds();
	const int world_floor = std::max(writable_min_y, world_editor::min_y() + 1);
	const int clear_top =
			base + static_cast<int>(std::lround(
						   (model.max_y - placement.landmark->ground_y + 1) * scale));
	const int foundation_floor = std::max(world_floor, base - 64);
	const unsigned property_rotation =
			static_cast<unsigned>((std::llround(rotation_degrees / 90.0) % 4 + 4) % 4);
	std::vector<BlockWithProperties> rotated_palette;
	rotated_palette.reserve(model.schematic->palette.size());
	for (const auto &block : model.schematic->palette) {
		auto properties = block.properties;
		if (property_rotation && properties)
			properties = BlockWithProperties::intern_properties(
					structures::rotate_schem_properties(*properties, property_rotation));
		rotated_palette.emplace_back(block.block, std::move(properties));
	}
	const std::optional<std::vector<Block>> empty_replacements{std::vector<Block>{}};
	std::size_t placed = 0;
	for (int wz = world_min_z; wz <= world_max_z; ++wz)
		for (int wx = world_min_x; wx <= world_max_x; ++wx) {
			auto [mx, mz] = to_model(wx + 0.5, wz + 0.5);
			const int model_x = static_cast<int>(std::floor(mx));
			const int model_z = static_cast<int>(std::floor(mz));
			auto [column, count] = model.column(model_x, model_z);
			if (!column || !count)
				continue;
			const int column_low = model.schematic->voxels[column[0]].y;
			const int column_bottom =
					base + static_cast<int>(std::lround(
								   (column_low - placement.landmark->ground_y) * scale));
			const int column_top = std::max(column_bottom + 1,
					base + static_cast<int>(std::lround(
								   (model.schematic->voxels[column[count - 1]].y + 1 -
										   placement.landmark->ground_y) *
								   scale)));
			const auto index = model.slot(model_x, model_z);
			const int terrain = editor.get_ground_level(wx, wz);
			const int clear_from = model.excavated[index]
										   ? std::min(column_bottom, terrain + 1)
								   : column_top > terrain ? terrain + 1
														  : clear_top;
			for (int y = std::max(clear_from, world_floor);
					y < clear_top && y <= world_ceiling; ++y)
				if (editor.get_block_absolute(wx, y, wz))
					editor.set_block_absolute(
							AIR, wx, y, wz, std::nullopt, empty_replacements);

			if (column_low <= placement.landmark->ground_y + 2) {
				for (int y = column_bottom - 1;
						y >= foundation_floor && y >= world_floor &&
						!editor.get_block_absolute(wx, y, wz);
						--y)
					editor.set_block_absolute(
							STONE, wx, y, wz, std::nullopt, empty_replacements);
			}
			for (std::size_t i = 0; i < count; ++i) {
				const auto &voxel = model.schematic->voxels[column[i]];
				const int y0 =
						base + static_cast<int>(std::lround(
									   (voxel.y - placement.landmark->ground_y) * scale));
				const int y1 = std::max(y0 + 1,
						base + static_cast<int>(std::lround(
									   (voxel.y + 1 - placement.landmark->ground_y) *
									   scale)));
				const auto &block = rotated_palette[voxel.palette_slot];
				for (int y = std::max(y0, world_floor); y < y1 && y <= world_ceiling;
						++y) {
					editor.set_block_with_properties_absolute(
							block, wx, y, wz, std::nullopt, empty_replacements);
					++placed;
				}
			}
		}
	return placed > 0;
}

std::size_t place_all(WorldEditor &editor, const PrescanResult &prescan, double scale,
		double rotation_degrees)
{
	std::size_t placed = 0;
	for (const auto &placement : prescan.placements)
		placed += place(editor, placement, scale, rotation_degrees) ? 1 : 0;
	return placed;
}
} // namespace arnis::landmarks
