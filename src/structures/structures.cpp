#include "structures.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <unordered_set>

#include "../block_definitions.h"
#include "../land_cover/land_cover.h"
#include "schem_decoder.h"

namespace arnis::structures
{
namespace boat { void scatter_boats(WorldEditor &, int, int, int, int); }
namespace helicopter { void maybe_place_helicopter(WorldEditor &, int, int); }
namespace starship { void place_on_launch_mount(WorldEditor &, const ProcessedWay &); }
namespace tombstone { void maybe_place(WorldEditor &, int, int, const RoadMaskBitmap &); }
const std::vector<StructureAsset> &rust_structure_assets()
{
	static const std::vector<StructureAsset> a = {{"car", "car.schem", 0, 0, 0, 0},
			{"boat", "boat.schem", 0, 0, 0, 0},
			{"crane", "crane.schem", 0, 0, 0, 0},
			{"excavator", "excavator.schem", 0, 0, 0, 0},
			{"fountain", "fountain.schem", 0, 0, 0, 0},
			{"helicopter", "helicopter.schem", 0, 0, 0, 0},
			{"lighthouse", "lighthouse.schem", 0, 0, 0, 0},
			{"playground", "playground.schem", 0, 0, 0, 0},
			{"starship", "starship.schem", 0, 0, 0, 0},
			{"tombstone", "tombstone.schem", 0, 0, 0, 0},
			{"tractor", "tractor.schem", 0, 0, 0, 0},
			{"windturbine", "windturbine.schem", 0, 0, 0, 0}};
	return a;
}
const StructureAsset *find_structure_asset(const std::string &name)
{
	for (const auto &a : rust_structure_assets())
		if (name == a.name)
			return &a;
	return nullptr;
}
const StructureAsset *find_structure_asset_ci(const std::string &name)
{
	std::string n = name;
	std::transform(n.begin(), n.end(), n.begin(),
			[](unsigned char c) { return char(std::tolower(c)); });
	for (const auto &a : rust_structure_assets()) {
		std::string k = a.name;
		std::transform(k.begin(), k.end(), k.begin(),
				[](unsigned char c) { return char(std::tolower(c)); });
		if (n == k)
			return &a;
	}
	return nullptr;
}
const StructureAsset *find_structure_asset_alias(const std::string &name)
{
	if (const auto *a = find_structure_asset_ci(name))
		return a;
	for (const auto &a : rust_structure_assets()) {
		if (name == a.schematic)
			return &a;
	}
	return nullptr;
}
std::string canonical_structure_name(const std::string &n)
{
	if (const auto *a = find_structure_asset_alias(n))
		return a->name;
	return {};
}
std::filesystem::path structure_asset_path(const WorldEditor &e, const std::string &n)
{
	const auto *a = find_structure_asset_alias(n);
	if (!a)
		return {};
	return e.get_schematic_asset_root() / a->schematic;
}
bool structure_asset_available(const WorldEditor &e, const std::string &n)
{
	// These names are logical Rust modules, not one-file schematic assets.
	// Their placement functions select/parse their real bundled variants and
	// fall back procedurally, so a fictional `car.schem` must not make the
	// entire structure pipeline appear unavailable.
	if (structure_has_procedural_generator(canonical_structure_name(n)))
		return true;
	const auto p = structure_asset_path(e, n);
	return !p.empty() && std::filesystem::is_regular_file(p);
}
std::vector<std::string> missing_structure_assets(const WorldEditor &e)
{
	std::vector<std::string> out;
	for (const auto &a : rust_structure_assets())
		if (!structure_asset_available(e, a.name))
			out.emplace_back(a.name);
	return out;
}
std::optional<std::tuple<int, int, int>> structure_dimensions(
		const WorldEditor &e, const std::string &n)
{
	const auto p = structure_asset_path(e, n);
	if (p.empty() || !std::filesystem::is_regular_file(p))
		return std::nullopt;
	std::ifstream in(p, std::ios::binary);
	std::vector<std::uint8_t> b((std::istreambuf_iterator<char>(in)), {});
	try {
		const auto d = decode_sponge_schem(b);
		return std::tuple<int, int, int>{d.width, d.height, d.length};
	} catch (...) {
		return std::nullopt;
	}
}
std::vector<std::pair<std::string, std::tuple<int, int, int>>>
available_structure_dimensions(const WorldEditor &e)
{
	std::vector<std::pair<std::string, std::tuple<int, int, int>>> out;
	for (const auto &a : rust_structure_assets())
		if (auto d = structure_dimensions(e, a.name))
			out.emplace_back(a.name, *d);
	return out;
}
std::vector<std::string> invalid_structure_assets(const WorldEditor &e)
{
	std::vector<std::string> out;
	for (const auto &a : rust_structure_assets()) {
		if (structure_has_procedural_generator(a.name))
			continue;
		const auto p = structure_asset_path(e, a.name);
		if (std::filesystem::is_regular_file(p)) {
			std::ifstream in(p, std::ios::binary);
			std::vector<std::uint8_t> b((std::istreambuf_iterator<char>(in)), {});
			try {
				decode_sponge_schem(b);
			} catch (...) {
				out.emplace_back(a.name);
			}
		}
	}
	return out;
}
StructureAudit audit_structures(const WorldEditor &e)
{
	StructureAudit r;
	r.total = rust_structure_assets().size();
	for (const auto &a : rust_structure_assets()) {
		if (structure_has_procedural_generator(a.name)) {
			++r.available;
			continue;
		}
		const auto p = structure_asset_path(e, a.name);
		if (!std::filesystem::is_regular_file(p)) {
			++r.missing;
			continue;
		}
		std::ifstream in(p, std::ios::binary);
		std::vector<std::uint8_t> b((std::istreambuf_iterator<char>(in)), {});
		try {
			decode_sponge_schem(b);
			++r.available;
		} catch (...) {
			++r.invalid;
		}
	}
	return r;
}
std::string format_structure_audit(const StructureAudit &a)
{
	return "total=" + std::to_string(a.total) +
		   " available=" + std::to_string(a.available) +
		   " missing=" + std::to_string(a.missing) +
		   " invalid=" + std::to_string(a.invalid);
}
bool audit_usable(const StructureAudit &a)
{
	return a.total > 0 && a.invalid == 0;
}
std::vector<PlacementResult> place_structures_checked(WorldEditor &e,
		const std::vector<std::string> &n, int x, int z, unsigned r, std::size_t a)
{
	if (!audit_usable(audit_structures(e)))
		return {};
	return place_structures_auto(e, n, x, z, r, a);
}
CheckedPlacements place_structures_with_audit(WorldEditor &e,
		const std::vector<std::string> &n, int x, int z, unsigned r, std::size_t a)
{
	CheckedPlacements out;
	out.audit = audit_structures(e);
	if (audit_usable(out.audit))
		out.results = place_structures_auto(e, n, x, z, r, a);
	out.stats = placement_stats(out.results);
	return out;
}
std::string structure_registry_fingerprint()
{
	std::string s;
	for (const auto &a : rust_structure_assets()) {
		s += a.name;
		s += ':';
		s += a.schematic;
		s += ';';
	}
	return std::to_string(std::hash<std::string>{}(s));
}
std::string structure_registry_manifest()
{
	std::string s;
	for (const auto &a : rust_structure_assets()) {
		if (!s.empty())
			s += '\n';
		s += a.name;
		s += '=';
		s += a.schematic;
	}
	return s;
}
bool structure_registry_valid()
{
	std::unordered_set<std::string> names, files;
	for (const auto &a : rust_structure_assets()) {
		if (!a.name || !a.schematic || !*a.name || !*a.schematic ||
				!names.insert(a.name).second || !files.insert(a.schematic).second)
			return false;
	}
	return true;
}
bool structure_has_procedural_generator(const std::string &n)
{
	return n == "boat" || n == "fountain" || n == "helicopter" ||
		   n == "lighthouse" || n == "windturbine" ||
		   n == "playground" || n == "excavator" || n == "tractor" || n == "crane" ||
		   n == "car" || n == "starship" || n == "tombstone";
}
std::vector<StructureAsset> procedural_structure_assets()
{
	std::vector<StructureAsset> out;
	for (const auto &a : rust_structure_assets())
		if (structure_has_procedural_generator(a.name))
			out.push_back(a);
	return out;
}
std::vector<StructureAsset> schematic_only_structure_assets()
{
	std::vector<StructureAsset> out;
	for (const auto &a : rust_structure_assets())
		if (!structure_has_procedural_generator(a.name))
			out.push_back(a);
	return out;
}
StructureBackend structure_backend(const WorldEditor &e, const std::string &n)
{
	const auto *a = find_structure_asset_ci(n);
	if (!a)
		return StructureBackend::Missing;
	const std::string canonical = a->name;
	if (structure_has_procedural_generator(canonical))
		return StructureBackend::Procedural;
	return structure_asset_available(e, canonical) ? StructureBackend::Schematic
												   : StructureBackend::Missing;
}
PlacementResult place_structure_auto(
		WorldEditor &e, const std::string &n, int x, int z, unsigned r, std::size_t a)
{
	const auto *asset = find_structure_asset_alias(n);
	if (!asset)
		return {};
	switch (structure_backend(e, asset->name)) {
	case StructureBackend::Procedural:
		return place_named_structure_rotated(e, asset->name, x, z, r, a);
	case StructureBackend::Schematic:
		return place_named_structure_rotated(e, asset->name, x, z, r, a);
	default:
		return {};
	}
}
std::vector<PlacementResult> place_structures_auto(WorldEditor &e,
		const std::vector<std::string> &names, int x, int z, unsigned r, std::size_t a)
{
	const auto normalized = normalize_structure_names(names);
	std::vector<PlacementResult> out;
	out.reserve(normalized.size());
	for (const auto &n : normalized)
		out.push_back(place_structure_auto(e, n, x, z, r, a));
	return out;
}
PlacementStats placement_stats(const std::vector<PlacementResult> &v)
{
	PlacementStats s;
	for (const auto &r : v) {
		if (r.recognized)
			++s.recognized;
		if (r.placed)
			++s.placed;
		else
			++s.missing;
	}
	return s;
}
std::vector<std::string> normalize_structure_names(const std::vector<std::string> &v)
{
	std::vector<std::string> out;
	std::unordered_set<std::string> seen;
	for (const auto &n : v) {
		const auto c = canonical_structure_name(n);
		if (!c.empty() && seen.insert(c).second)
			out.push_back(c);
	}
	return out;
}
std::vector<std::string> valid_structure_names()
{
	std::vector<std::string> out;
	for (const auto &a : rust_structure_assets())
		out.emplace_back(a.name);
	return out;
}
bool place_named_structure(
		WorldEditor &e, const std::string &name, int x, int z, std::size_t area)
{
	std::vector<std::pair<int, int>> cells{{x, z}};
	if (name == "fountain") {
		fountain::place(e, x, z, area);
		return true;
	}
	if (name == "lighthouse") {
		lighthouse::place(e, x, z);
		return true;
	}
	if (name == "windturbine") {
		windturbine::place(e, x, z);
		return true;
	}
	if (name == "playground") {
		playground::scatter_playgrounds(e, cells);
		return true;
	}
	if (name == "excavator") {
		excavator::scatter_excavators(e, cells);
		return true;
	}
	if (name == "tractor") {
		tractor::maybe_place_tractor(e, cells);
		return true;
	}
	if (name == "crane") {
		crane::maybe_place_crane(e, cells);
		return true;
	}
	if (name == "car") {
		car::maybe_place_car(e, x, z, 0);
		return true;
	}
	if (name == "helicopter") {
		helicopter::maybe_place_helicopter(e, x, z);
		return true;
	}
	if (name == "boat") {
		boat::scatter_boats(e, x - 1, z - 1, x + 1, z + 1);
		return true;
	}
	const auto p = structure_asset_path(e, name);
	return !p.empty() && place_schem_file(e, p, x, e.get_ground_level(x, z), z);
}
PlacementResult place_named_structure_result(
		WorldEditor &e, const std::string &n, int x, int z, std::size_t a)
{
	const auto *asset = find_structure_asset_alias(n);
	if (!asset)
		return {};
	return {true, place_named_structure(e, asset->name, x, z, a), asset->schematic};
}
PlacementResult place_named_structure_rotated(WorldEditor &e, const std::string &n, int x,
		int z, unsigned rotation, std::size_t area)
{
	const auto *canonical = find_structure_asset_alias(n);
	if (!canonical)
		return {};
	const std::string name = canonical->name;
	if (name == "car") {
		car::maybe_place_car(e, x, z, std::uint8_t(rotation & 3));
		return {true, true, "car.schem"};
	}
	if (name == "boat") {
		boat::scatter_boats(e, x - 1, z - 1, x + 1, z + 1);
		return {true, true, "boat.schem"};
	}
	if (name == "helicopter") {
		helicopter::maybe_place_helicopter(e, x, z);
		return {true, true, "helicopter.schem"};
	}
	if (name == "fountain") {
		fountain::place(e, x, z, area);
		return {true, true, "fountain"};
	}
	if (name == "lighthouse") {
		lighthouse::place(e, x, z);
		return {true, true, "lighthouse.schem"};
	}
	if (name == "windturbine") {
		windturbine::place(e, x, z);
		return {true, true, "windturbine.schem"};
	}
	if (name == "playground") {
		playground::scatter_playgrounds(e, {{x, z}});
		return {true, true, "playground"};
	}
	if (name == "excavator") {
		excavator::scatter_excavators(e, {{x, z}});
		return {true, true, "excavator.schem"};
	}
	if (name == "tractor") {
		tractor::maybe_place_tractor(e, {{x, z}});
		return {true, true, "tractor.schem"};
	}
	if (name == "crane") {
		crane::maybe_place_crane(e, {{x, z}});
		return {true, true, "crane.schem"};
	}
	const auto p = structure_asset_path(e, name);
	const bool ok = !p.empty() && place_schem_file_rotated(e, p, x,
										  e.get_ground_level(x, z), z, rotation & 3);
	return {true, ok, canonical->schematic};
}
namespace
{

using arnis::land_cover::coord_hash;

std::pair<int, int> nearest_cell_to_centroid(
		const std::vector<std::pair<int, int>> &cells)
{
	if (cells.empty())
		return {0, 0};

	long long sx = 0;
	long long sz = 0;
	for (const auto &[x, z] : cells) {
		sx += x;
		sz += z;
	}
	const int cx = static_cast<int>(sx / static_cast<long long>(cells.size()));
	const int cz = static_cast<int>(sz / static_cast<long long>(cells.size()));

	return *std::min_element(
			cells.begin(), cells.end(), [cx, cz](const auto &a, const auto &b) {
				const long long adx = static_cast<long long>(a.first - cx);
				const long long adz = static_cast<long long>(a.second - cz);
				const long long bdx = static_cast<long long>(b.first - cx);
				const long long bdz = static_cast<long long>(b.second - cz);
				return adx * adx + adz * adz < bdx * bdx + bdz * bdz;
			});
}

std::pair<int, int> rotated(int dx, int dz, uint8_t rot)
{
	switch (rot & 3) {
	case 1:
		return {-dz, dx};
	case 2:
		return {-dx, -dz};
	case 3:
		return {dz, -dx};
	default:
		return {dx, dz};
	}
}

void block(WorldEditor &editor, Block b, int x, int y, int z)
{
	editor.set_block_absolute(b, x, y, z);
}

void rel(WorldEditor &editor, Block b, int x, int z, int base_y, int dx, int dy, int dz,
		uint8_t rot)
{
	const auto [rx, rz] = rotated(dx, dz, rot);
	block(editor, b, x + rx, base_y + dy, z + rz);
}

void pad(WorldEditor &editor, Block b, int x, int z, int base_y, int radius)
{
	for (int dx = -radius; dx <= radius; ++dx) {
		for (int dz = -radius; dz <= radius; ++dz)
			block(editor, b, x + dx, base_y - 1, z + dz);
	}
}

void simple_fountain(WorldEditor &editor, int x, int z, std::size_t area_cells)
{
	const bool large = area_cells >= 300;
	const int base_y = editor.get_absolute_y(x, 1, z);
	const int radius = large ? 4 : 2;
	pad(editor, STONE_BRICKS, x, z, base_y, radius);
	for (int dx = -radius; dx <= radius; ++dx) {
		for (int dz = -radius; dz <= radius; ++dz) {
			const bool edge = std::abs(dx) == radius || std::abs(dz) == radius;
			if (edge)
				block(editor, POLISHED_ANDESITE, x + dx, base_y, z + dz);
			else
				block(editor, WATER, x + dx, base_y, z + dz);
		}
	}
	for (int y = 1; y <= (large ? 4 : 2); ++y)
		block(editor, WATER, x, base_y + y, z);
	block(editor, SEA_LANTERN, x, base_y, z);
}

void simple_playground(WorldEditor &editor, int x, int z, uint8_t rot)
{
	const int base_y = editor.get_absolute_y(x, 1, z);
	pad(editor, SAND, x, z, base_y, 5);

	for (int dz : {-2, 2}) {
		for (int y = 0; y <= 3; ++y) {
			rel(editor, OAK_FENCE, x, z, base_y, -2, y, dz, rot);
			rel(editor, OAK_FENCE, x, z, base_y, 2, y, dz, rot);
		}
	}
	for (int dx = -2; dx <= 2; ++dx) {
		rel(editor, OAK_PLANKS, x, z, base_y, dx, 4, -2, rot);
		rel(editor, OAK_PLANKS, x, z, base_y, dx, 4, 2, rot);
	}
	rel(editor, CHAIN, x, z, base_y, -1, 3, 0, rot);
	rel(editor, CHAIN, x, z, base_y, 1, 3, 0, rot);
	rel(editor, OAK_SLAB, x, z, base_y, 0, 1, 0, rot);

	for (int step = 0; step < 4; ++step)
		rel(editor, STONE_BLOCK_SLAB, x, z, base_y, 4 + step, step, -1, rot);
	for (int y = 0; y <= 3; ++y)
		rel(editor, LADDER, x, z, base_y, 4, y, -2, rot);
}

void simple_lighthouse(WorldEditor &editor, int x, int z, uint8_t rot)
{
	const int base_y = editor.get_absolute_y(x, 1, z);
	pad(editor, STONE_BRICKS, x, z, base_y, 2);
	for (int y = 0; y < 12; ++y) {
		const int radius = y < 8 ? 2 : 1;
		for (int dx = -radius; dx <= radius; ++dx) {
			for (int dz = -radius; dz <= radius; ++dz) {
				if (std::abs(dx) == radius || std::abs(dz) == radius)
					rel(editor, (y / 2) % 2 == 0 ? WHITE_CONCRETE : RED_CONCRETE, x, z,
							base_y, dx, y, dz, rot);
			}
		}
	}
	for (int dx = -2; dx <= 2; ++dx) {
		for (int dz = -2; dz <= 2; ++dz)
			rel(editor, GLASS, x, z, base_y, dx, 12, dz, rot);
	}
	rel(editor, SEA_LANTERN, x, z, base_y, 0, 13, 0, rot);
	rel(editor, LIGHTNING_ROD, x, z, base_y, 0, 14, 0, rot);
}

void simple_crane(WorldEditor &editor, int x, int z, uint8_t rot)
{
	const int base_y = editor.get_absolute_y(x, 1, z);
	for (int y = 0; y < 18; ++y)
		rel(editor, SCAFFOLDING, x, z, base_y, 0, y, 0, rot);
	for (int dx = -10; dx <= 14; ++dx)
		rel(editor, YELLOW_CONCRETE, x, z, base_y, dx, 17, 0, rot);
	for (int y = 10; y <= 16; ++y)
		rel(editor, CHAIN, x, z, base_y, 10, y, 0, rot);
	rel(editor, IRON_BLOCK, x, z, base_y, 10, 9, 0, rot);
}

void simple_excavator(WorldEditor &editor, int x, int z, uint8_t rot)
{
	const int base_y = editor.get_absolute_y(x, 1, z);
	for (int dx = -2; dx <= 2; ++dx) {
		rel(editor, BLACK_CONCRETE, x, z, base_y, dx, 0, -1, rot);
		rel(editor, BLACK_CONCRETE, x, z, base_y, dx, 0, 1, rot);
	}
	for (int dx = -1; dx <= 1; ++dx) {
		for (int dz = -1; dz <= 1; ++dz)
			rel(editor, YELLOW_CONCRETE, x, z, base_y, dx, 1, dz, rot);
	}
	for (int dx = 2; dx <= 5; ++dx)
		rel(editor, YELLOW_CONCRETE, x, z, base_y, dx, 2, 0, rot);
	rel(editor, IRON_BLOCK, x, z, base_y, 6, 1, 0, rot);
}

void simple_tractor(WorldEditor &editor, int x, int z, uint8_t rot)
{
	const int base_y = editor.get_absolute_y(x, 1, z);
	for (int dz = -1; dz <= 1; ++dz) {
		rel(editor, RED_CONCRETE, x, z, base_y, 0, 1, dz, rot);
		rel(editor, RED_CONCRETE, x, z, base_y, 1, 1, dz, rot);
	}
	rel(editor, GLASS, x, z, base_y, -1, 2, 0, rot);
	rel(editor, BLACK_CONCRETE, x, z, base_y, -1, 0, -1, rot);
	rel(editor, BLACK_CONCRETE, x, z, base_y, -1, 0, 1, rot);
	rel(editor, BLACK_CONCRETE, x, z, base_y, 2, 0, -1, rot);
	rel(editor, BLACK_CONCRETE, x, z, base_y, 2, 0, 1, rot);
}

void simple_boat(WorldEditor &editor, int x, int z, uint8_t rot)
{
	const int base_y = editor.get_water_level(x, z);
	for (int dx = -4; dx <= 4; ++dx) {
		const int half = 2 - (std::abs(dx) / 3);
		for (int dz = -half; dz <= half; ++dz)
			rel(editor, OAK_PLANKS, x, z, base_y, dx, 0, dz, rot);
	}
	for (int dx = -2; dx <= 2; ++dx)
		rel(editor, WHITE_WOOL, x, z, base_y, dx, 3, 0, rot);
	for (int y = 1; y <= 4; ++y)
		rel(editor, OAK_FENCE, x, z, base_y, 0, y, 0, rot);
}

void simple_car(WorldEditor &editor, int x, int z, uint8_t rot, uint64_t variant)
{
	const int y = editor.get_absolute_y(x, 1, z);
	const std::array<Block, 6> colours{RED_CONCRETE, BLUE_CONCRETE, WHITE_CONCRETE,
			GRAY_CONCRETE, YELLOW_CONCRETE, ORANGE_CONCRETE};
	const Block body = colours[variant % colours.size()];
	for (int dx = -2; dx <= 2; ++dx)
		for (int dz = -1; dz <= 1; ++dz)
			rel(editor, body, x, z, y, dx, 1, dz, rot);
	for (int dx = -1; dx <= 1; ++dx)
		for (int dz = -1; dz <= 1; ++dz)
			rel(editor, dx == 0 ? GLASS : body, x, z, y, dx, 2, dz, rot);
	for (int dx : {-2, 2})
		for (int dz : {-1, 1})
			rel(editor, BLACK_CONCRETE, x, z, y, dx, 0, dz, rot);
}

void simple_helicopter(WorldEditor &editor, int x, int z, uint8_t rot)
{
	const int y = editor.get_absolute_y(x, 1, z);
	for (int dx = -3; dx <= 3; ++dx)
		rel(editor, WHITE_CONCRETE, x, z, y, dx, 1, 0, rot);
	for (int dx = -1; dx <= 1; ++dx)
		for (int dz = -1; dz <= 1; ++dz)
			rel(editor, dx == 1 ? GLASS : WHITE_CONCRETE, x, z, y, dx, 2, dz, rot);
	for (int d = -5; d <= 5; ++d) {
		rel(editor, IRON_BLOCK, x, z, y, d, 4, 0, rot);
		rel(editor, IRON_BLOCK, x, z, y, 0, 4, d, rot);
	}
}

void simple_windturbine(WorldEditor &editor, int x, int z, uint8_t rot)
{
	const int y = editor.get_absolute_y(x, 1, z);
	for (int dy = 0; dy < 28; ++dy)
		block(editor, WHITE_CONCRETE, x, y + dy, z);
	for (int d = -9; d <= 9; ++d)
		rel(editor, WHITE_CONCRETE, x, z, y, 0, 27 + d, d / 3, rot);
	block(editor, IRON_BLOCK, x, y + 27, z);
}

}

#if 0 // Structure generators are split into one C++ file per Rust module.
namespace fountain
{
void place(WorldEditor &editor, int x, int z, std::size_t area_cells)
{
	if (!editor.place_schematics())
		return;
	const auto h = coord_hash(x, z);
	// Rust reserves fountain4 for broad OSM polygons; ordinary fountains pick
	// deterministically among only the three small variants.
	const auto variant = area_cells >= 300 ? 4 : int((h >> 4) % 3) + 1;
	// Keep Rust's asset-only contract: an unavailable schematic means no
	// fountain, rather than silently substituting a different procedural shape.
	place_named_schem(editor, "fountain" + std::to_string(variant), x,
			editor.get_absolute_y(x, 1, z), z, (h >> 8) & 3);
}
}

namespace playground
{
void scatter_playgrounds(
		WorldEditor &editor, const std::vector<std::pair<int, int>> &cells)
{
	if (!editor.place_schematics())
		return;
	const std::size_t n = cells.size();
	if (n < 120)
		return;
	const std::size_t target = std::clamp<std::size_t>(n / 500, 1, 4);
	std::vector<std::pair<int, int>> placed;
	for (std::uint32_t t = 0; placed.size() < target && t < target * 8; ++t) {
		const auto h = coord_hash(static_cast<int>(t) + 1, static_cast<int>(n));
		const auto [ax, az] = cells[h % n];
		if (editor.is_lc_water(ax, az))
			continue;
		const bool too_close =
				std::any_of(placed.begin(), placed.end(), [ax, az](const auto &p) {
					return std::abs(p.first - ax) < 16 && std::abs(p.second - az) < 16;
				});
		if (too_close)
			continue;
		const int selected = static_cast<int>((h >> 5) % 3) + 1;
		const int y = editor.get_absolute_y(ax, 1, az);
		const unsigned rotation = (h >> 7) & 3;
		bool placed_schematic = place_named_schem(editor,
				"playground" + std::to_string(selected), ax, y, az, rotation, &SAND);
		for (int fallback = 1; !placed_schematic && fallback <= 3; ++fallback)
			if (fallback != selected)
				placed_schematic = place_named_schem(editor,
						"playground" + std::to_string(fallback), ax, y, az, rotation, &SAND);
		if (!placed_schematic)
			continue;
		placed.emplace_back(ax, az);
	}
}
}

namespace lighthouse
{
void place(WorldEditor &editor, int x, int z)
{
	if (!editor.place_schematics())
		return;
	const auto h = coord_hash(x, z);
	const auto file =
			std::filesystem::path(__FILE__).parent_path().parent_path() /
			"assets/structures/lighthouse.schem";
	place_schem_file_rotated(editor, file, x, editor.get_absolute_y(x, 1, z), z, h & 3);
}
}

namespace crane
{
void maybe_place_crane(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells)
{
	if (!editor.place_schematics())
		return;
	if (cells.size() < 1500)
		return;
	const auto [ax, az] = nearest_cell_to_centroid(cells);
	if (editor.is_lc_water(ax, az))
		return;
	const auto h = coord_hash(ax, az);
	if (h % 100 >= 60)
		return;
	place_named_schem(
			editor, "crane", ax, editor.get_absolute_y(ax, 1, az), az, (h >> 8) & 3);
}
}

namespace excavator
{
void scatter_excavators(
		WorldEditor &editor, const std::vector<std::pair<int, int>> &cells)
{
	if (!editor.place_schematics())
		return;
	const std::size_t n = cells.size();
	if (n < 1500)
		return;
	const std::size_t target = std::clamp<std::size_t>(n / 2000, 1, 6);
	std::vector<std::pair<int, int>> placed;
	for (std::uint32_t t = 0; placed.size() < target && t < target * 8; ++t) {
		const auto h = coord_hash(static_cast<int>(t) + 1, static_cast<int>(n));
		const auto [ax, az] = cells[h % n];
		if (editor.is_lc_water(ax, az))
			continue;
		const bool too_close =
				std::any_of(placed.begin(), placed.end(), [ax, az](const auto &p) {
					return std::abs(p.first - ax) < 24 && std::abs(p.second - az) < 24;
				});
		if (too_close)
			continue;
		place_named_schem(editor, "excavator", ax, editor.get_absolute_y(ax, 1, az), az,
				(h >> 5) & 3);
		placed.emplace_back(ax, az);
	}
}
}

namespace tractor
{
void maybe_place_tractor(
		WorldEditor &editor, const std::vector<std::pair<int, int>> &cells)
{
	if (!editor.place_schematics())
		return;
	const std::size_t n = cells.size();
	if (n < 600)
		return;
	const auto h =
			coord_hash(cells.front().first, cells.front().second ^ static_cast<int>(n));
	if (h % 100 >= 30)
		return;
	const auto [ax, az] = cells[h % n];
	if (editor.is_lc_water(ax, az))
		return;
	place_named_schem(
			editor, "tractor", ax, editor.get_absolute_y(ax, 1, az), az, (h >> 8) & 3);
}
}

#endif

namespace boat
{
void scatter_boats(WorldEditor &editor, int min_x, int min_z, int max_x, int max_z)
{
	constexpr int spacing = 400;
	constexpr int max_boats = 200;
	int count = 0;
	for (int gz = min_z - ((min_z % spacing) + spacing) % spacing; gz <= max_z;
			gz += spacing) {
		for (int gx = min_x - ((min_x % spacing) + spacing) % spacing; gx <= max_x;
				gx += spacing) {
			if (count >= max_boats)
				return;
			const auto h = coord_hash(gx, gz);
			if (h % 100 >= 45)
				continue;
			const int ax = gx + static_cast<int>(h % 7);
			const int az = gz + static_cast<int>((h >> 3) % 7);
			if (editor.is_lc_water(ax, az) && editor.water_distance(ax, az) == 0) {
				simple_boat(editor, ax, az, static_cast<uint8_t>((h >> 5) & 3));
				++count;
			}
		}
	}
}
}

#if 0
namespace car
{
void maybe_place_car(WorldEditor &editor, int cx, int cz, uint8_t rot_base)
{
	if (!editor.place_schematics())
		return;
	const auto h = coord_hash(cx, cz);
	if (h % 100 >= 50)
		return;
	const uint8_t flip = ((h >> 16) & 1) ? 2 : 0;
	static const std::array<const char *, 10> models{{"car_camper", "car_fedex",
			"car_hotrod_blue", "car_hotrod_white", "car_pickup", "car_police",
			"car_sedan", "car_suv", "car_uhaul", "car_workvan"}};
	const auto model = models[(h >> 8) % models.size()];
	const auto file =
			std::filesystem::path(__FILE__).parent_path().parent_path() /
			"assets/structures" / (std::string(model) + ".schem");
	unsigned align = 0;
	if (std::ifstream in(file, std::ios::binary); in) {
		std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)), {});
		try {
			const auto doc = decode_sponge_schem(bytes);
			align = doc.width > doc.length ? 1 : 0;
		} catch (...) {
		}
	}
	place_schem_file_anchored(editor, file, cx, editor.get_absolute_y(cx, 1, cz), cz,
			(rot_base + align + flip) & 3, SchemAnchor::Centered);
}
}

#endif

namespace helicopter
{
void maybe_place_helicopter(WorldEditor &editor, int cx, int cz)
{
	const auto h = coord_hash(cx, cz);
	if (h % 100 >= 60 || editor.is_lc_water(cx, cz))
		return;
	simple_helicopter(editor, cx, cz, static_cast<uint8_t>((h >> 8) & 3));
}
}

namespace starship
{
void place_on_launch_mount(WorldEditor &editor, const ProcessedWay &ring)
{
	if (ring.nodes.empty())
		return;
	int64_t sx = 0, sz = 0;
	for (const auto &node : ring.nodes) {
		sx += node.x;
		sz += node.z;
	}
	const int x = static_cast<int>(sx / static_cast<int64_t>(ring.nodes.size()));
	const int z = static_cast<int>(sz / static_cast<int64_t>(ring.nodes.size()));
	const int y = editor.get_absolute_y(x, 1, z);
	for (int dy = 0; dy < 48; ++dy) {
		const int radius = dy < 8 ? 2 : 1;
		for (int dx = -radius; dx <= radius; ++dx)
			for (int dz = -radius; dz <= radius; ++dz)
				if (dx * dx + dz * dz <= radius * radius)
					block(editor, dy < 5 ? BLACKSTONE : LIGHT_GRAY_CONCRETE, x + dx,
							y + dy, z + dz);
	}
}
}

#if 0
namespace windturbine
{
void place(WorldEditor &editor, int x, int z)
{
	if (!editor.place_schematics())
		return;
	const auto h = coord_hash(x, z);
	const auto file =
			std::filesystem::path(__FILE__).parent_path().parent_path() /
			"assets/structures/windturbine.schem";
	place_schem_file_anchored(editor, file, x, editor.get_absolute_y(x, 1, z), z, h & 3,
			SchemAnchor::BaseCentroid);
}
}

#endif

}
