#include "building_construction.h"

#include "../block_definitions.h"
#include "../land_cover/land_cover.h"
#include "../structures/structures.h"
#include "../../../arnis_world_editor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <array>
#include <optional>
#include <unordered_set>

namespace arnis::building_construction
{
namespace
{
using Cell = std::pair<int, int>;
struct CellHash
{
	std::size_t operator()(const Cell &p) const noexcept
	{
		return std::hash<int>{}(p.first) ^ (std::hash<int>{}(p.second) << 1);
	}
};
constexpr double HOUSE_M2 = 160.0;
constexpr int HOUSE_STOREYS = 3;
constexpr double CRANE_M2 = 600.0;
constexpr int CRANE_STOREYS = 4;
constexpr int CRANE_CLEARANCE = 55;
constexpr int MIN_STOREY = 3;

enum class Cladding
{
	Masonry,
	Curtain,
};

int euclidean_mod(int value, int divisor)
{
	const int remainder = value % divisor;
	return remainder < 0 ? remainder + divisor : remainder;
}

Cladding cladding(const ProcessedWay &way)
{
	const std::string future = way.tags.contains("construction")
									   ? way.tags.get("construction")
									   : way.tags.get("construction:building");
	if (future == "office" || future == "commercial" || future == "retail" ||
			future == "hotel" || future == "hospital" || future == "university")
		return Cladding::Curtain;
	return Cladding::Masonry;
}

std::uint64_t hash(std::uint64_t id, std::uint64_t salt)
{
	return land_cover::coord_hash(static_cast<int>(id ^ (salt * 0x9E3779B9ULL)),
			static_cast<int>((id >> 32) ^ salt));
}

struct Plan
{
	const std::vector<Cell> &cells;
	std::unordered_set<Cell, CellHash> set;
	int min_x = 0, min_z = 0, span = 6;
	explicit Plan(const std::vector<Cell> &c, double scale) : cells(c)
	{
		set.insert(c.begin(), c.end());
		if (!c.empty()) {
			min_x = std::min_element(c.begin(), c.end())->first;
			min_z = std::min_element(
					c.begin(), c.end(), [](const Cell &a, const Cell &b) {
						return a.second < b.second;
					})->second;
		}
		span = std::clamp(static_cast<int>(std::lround(6.0 * scale)), 3, 8);
	}
	bool inside(int x, int z) const { return set.contains({x, z}); }
	bool edge(int x, int z) const
	{
		return !inside(x + 1, z) || !inside(x - 1, z) || !inside(x, z + 1) ||
			   !inside(x, z - 1);
	}
	bool column(int x, int z) const
	{
		const int gx = x - min_x, gz = z - min_z;
		if (edge(x, z)) {
			const bool corner = (!inside(x + 1, z) || !inside(x - 1, z)) &&
								(!inside(x, z + 1) || !inside(x, z - 1));
			return corner || (gx + gz) % span == 0;
		}
		if (gx % span || gz % span)
			return false;
		for (int dz = -1; dz <= 1; ++dz)
			for (int dx = -1; dx <= 1; ++dx)
				if (edge(x + dx, z + dz) && inside(x + dx, z + dz))
					return false;
		return true;
	}
	Cell middle() const
	{
		if (cells.empty())
			return {};
		long long sx = 0, sz = 0;
		for (const auto &[x, z] : cells) {
			sx += x;
			sz += z;
		}
		const int cx = static_cast<int>(sx / cells.size());
		const int cz = static_cast<int>(sz / cells.size());
		return *std::min_element(
				cells.begin(), cells.end(), [&](const Cell &a, const Cell &b) {
					auto d = [&](const Cell &p) {
						const long long dx = p.first - cx, dz = p.second - cz;
						return dx * dx + dz * dz;
					};
					return d(a) < d(b);
				});
	}
};
} // namespace

Progress Progress::of(std::uint64_t id, int height, int storey)
{
	Progress p;
	p.storey = std::max(3, storey);
	p.planned = std::max(1, height / p.storey);
	const double share = .4 + static_cast<double>(hash(id, 1) % 46) / 100.0;
	p.built = std::clamp(static_cast<int>(std::lround(p.planned * share)), 1, p.planned);
	return p;
}

void generate(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<Cell> &cells, int base_y, Progress progress, bool terrain,
		double scale, const CoordinateBitmap &footprints,
		const CoordinateBitmap &passages, int passage_height)
{
	if (cells.empty() || scale <= 0.0)
		return;
	const Plan plan(cells, scale);
	const auto slab_y = [&](int k) { return base_y + k * progress.storey; };
	const double area = cells.size() / (scale * scale);
	const bool house = area <= HOUSE_M2 && progress.planned <= HOUSE_STOREYS;
	const auto open = [&](int x, int z, int y) {
		return passages.contains(x, z) && y <= base_y + passage_height;
	};
	for (const auto &[x, z] : cells) {
		if (passages.contains(x, z))
			continue;
		if (terrain)
			if (const auto ground = editor.terrain_level(x, z))
				for (int y = *ground; y < base_y; ++y)
					editor.set_block_absolute(GRAY_CONCRETE, x, y, z);
		editor.set_block_absolute(LIGHT_GRAY_CONCRETE, x, base_y, z);
	}
	for (int k = 1; k <= progress.built; ++k)
		for (const auto &[x, z] : cells)
			if (!open(x, z, slab_y(k)))
				editor.set_block_absolute(LIGHT_GRAY_CONCRETE, x, slab_y(k), z);

	const auto [mx, mz] = plan.middle();
	const int core_size = std::clamp(static_cast<int>(std::lround(4.0 * scale)), 3, 6);
	const int core_x = mx - core_size / 2, core_z = mz - core_size / 2;
	bool fits_core = true;
	for (int z = core_z - 1; z < core_z + core_size + 1 && fits_core; ++z)
		for (int x = core_x - 1; x < core_x + core_size + 1; ++x)
			fits_core &= plan.inside(x, z);
	const bool has_core = !house && fits_core;
	const auto in_core = [&](int x, int z) {
		return has_core && x >= core_x && x < core_x + core_size && z >= core_z &&
			   z < core_z + core_size;
	};
	for (const auto &[x, z] : cells) {
		if (house || in_core(x, z) || !plan.column(x, z))
			continue;
		for (int y = base_y + 1; y < slab_y(progress.built); ++y)
			if (!open(x, z, y))
				editor.set_block_absolute(GRAY_CONCRETE, x, y, z);
	}
	const int top = slab_y(progress.built);
	if (house && !progress.topped_out()) {
		for (const auto &[x, z] : cells) {
			if (!plan.edge(x, z))
				continue;
			const int stretch_sum = x + z;
			const int stretch = stretch_sum / 4 - static_cast<int>(stretch_sum % 4 < 0);
			const int rise =
					1 +
					static_cast<int>(land_cover::coord_hash(
											 stretch ^ static_cast<int>(way.id), 0x51) %
									 (progress.storey - 1));
			for (int y = top + 1; y <= top + rise; ++y)
				editor.set_block_absolute(LIGHT_GRAY_CONCRETE, x, y, z);
		}
	} else if (!progress.topped_out()) {
		const auto direction = hash(way.id, 2) % 4;
		const int dx = direction == 0 ? 1 : direction == 1 ? -1 : 0;
		const int dz = direction == 2 ? 1 : direction == 3 ? -1 : 0;
		int lo = std::numeric_limits<int>::max(), hi = std::numeric_limits<int>::min();
		for (const auto &[x, z] : cells) {
			const int v = x * dx + z * dz;
			lo = std::min(lo, v);
			hi = std::max(hi, v);
		}
		const int formed =
				lo + (hi - lo) * static_cast<int>(20 + hash(way.id, 3) % 30) / 100;
		for (const auto &[x, z] : cells) {
			if (plan.column(x, z) && !in_core(x, z)) {
				const int rise =
						1 + static_cast<int>(land_cover::coord_hash(
													 x ^ static_cast<int>(way.id), z) %
											 (progress.storey - 1));
				for (int y = top + 1; y <= top + rise; ++y)
					editor.set_block_absolute(GRAY_CONCRETE, x, y, z);
				for (int y = top + rise + 1; y <= top + rise + 2; ++y)
					editor.set_block_absolute(IRON_BARS, x, y, z);
			} else if (x * dx + z * dz <= formed && !in_core(x, z)) {
				editor.set_block_absolute(SPRUCE_PLANKS, x, top + progress.storey, z);
				if ((x - plan.min_x) % 3 == 0 && (z - plan.min_z) % 3 == 0)
					for (int y = top + 1; y < top + progress.storey; ++y)
						editor.set_block_absolute(SPRUCE_FENCE, x, y, z);
			}
		}
	} else {
		for (const auto &[x, z] : cells)
			if (plan.edge(x, z))
				editor.set_block_absolute(GRAY_CONCRETE, x, top + 1, z);
	}
	const int core_top = progress.topped_out() ? top + MIN_STOREY : top + progress.storey;
	if (has_core) {
		const int door = core_x + core_size / 2;
		for (int x = core_x; x < core_x + core_size; ++x)
			for (int z = core_z; z < core_z + core_size; ++z) {
				if (x != core_x && x != core_x + core_size - 1 && z != core_z &&
						z != core_z + core_size - 1)
					continue;
				for (int y = base_y + 1; y <= core_top; ++y) {
					const int level = (y - base_y) % progress.storey;
					if (z == core_z && x == door && (level == 1 || level == 2))
						continue;
					editor.set_block_absolute(GRAY_CONCRETE, x, y, z);
				}
			}
	}
	const int clad =
			house ? progress.built
				  : static_cast<int>(std::lround(
							(progress.built - 1) * (hash(way.id, 4) % 70) / 100.0));
	const auto kind = cladding(way);
	const double started = static_cast<double>(hash(way.id, 5) % 360);
	const auto [center_x, center_z] = plan.middle();
	for (int k = 1; k <= clad; ++k)
		for (const auto &[x, z] : cells) {
			if (!plan.edge(x, z))
				continue;
			if (k == clad && !house && clad < progress.built) {
				const double angle = std::atan2(static_cast<double>(z - center_z),
											 static_cast<double>(x - center_x)) *
											 180.0 / std::acos(-1.0) +
									 180.0;
				double sweep = std::fmod(angle - started, 360.0);
				if (sweep < 0.0)
					sweep += 360.0;
				if (sweep > 200.0)
					continue;
			}
			for (int y = slab_y(k - 1) + 1; y < slab_y(k); ++y) {
				if (open(x, z, y))
					continue;
				const int along = x + z, row = y - slab_y(k - 1);
				const int along_mod_3 = euclidean_mod(along, 3);
				if (kind == Cladding::Curtain) {
					editor.set_block_absolute(
							along_mod_3 == 0 ? GRAY_CONCRETE : LIGHT_BLUE_STAINED_GLASS,
							x, y, z);
				} else {
					const bool window =
							along_mod_3 != 0 && row >= 2 && row < progress.storey;
					if (window)
						continue;
					editor.set_block_absolute(
							house ? LIGHT_GRAY_CONCRETE : WHITE_TERRACOTTA, x, y, z);
				}
			}
		}
	if (progress.built > 0) {
		const BlockWithProperties scaffold{
				SCAFFOLDING, BlockWithProperties::Properties{{"distance", "0"},
									 {"bottom", "false"}, {"waterlogged", "false"}}};
		const std::array<Cell, 4> faces{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
		const auto first = static_cast<std::size_t>(hash(way.id, 6) % faces.size());
		const std::size_t count = hash(way.id, 7) % 2 == 0 ? 2 : 1;
		const int scaffold_top =
				house ? top + (progress.topped_out() ? 1 : 2)
					  : std::min(slab_y(clad) + progress.storey + 1, top + 2);
		std::unordered_set<Cell, CellHash> stood;
		for (std::size_t f = 0; f < count; ++f) {
			const auto n = faces[(first + 2 * f) % faces.size()];
			const Cell tangent{-n.second, n.first};
			std::vector<Cell> face;
			for (const auto &[x, z] : cells)
				if (!plan.inside(x + n.first, z + n.second))
					face.emplace_back(x, z);
			if (face.empty())
				continue;
			const auto along = [&](const Cell &p) {
				return p.first * tangent.first + p.second * tangent.second;
			};
			int lo = along(face.front()), hi = lo;
			for (const auto &p : face) {
				lo = std::min(lo, along(p));
				hi = std::max(hi, along(p));
			}
			const int length = hi - lo + 1;
			const int run = std::max(
					3, length * (40 + static_cast<int>(hash(way.id, 10 + f) % 41)) / 100);
			const int available = std::max(1, length - run + 1);
			const int start = lo + static_cast<int>(hash(way.id, 20 + f) % available);
			for (const auto &[x, z] : face) {
				if (along({x, z}) < start || along({x, z}) >= start + run)
					continue;
				const int sx = x + n.first, sz = z + n.second;
				if (footprints.contains(sx, sz) || passages.contains(sx, sz) ||
						editor.surface_is_sealed(sx, sz) || editor.is_lc_water(sx, sz) ||
						!stood.insert({sx, sz}).second)
					continue;
				for (int y = editor.get_absolute_y(sx, 0, sz) + 1; y <= scaffold_top; ++y)
					editor.set_block_with_properties_absolute(
							scaffold, sx, y, sz, std::nullopt, std::nullopt);
			}
		}
	}
	if (top - base_y <= CRANE_CLEARANCE) {
		auto crane = structures::crane::mapped_crane_on(editor, cells);
		if (!crane && !house && area >= CRANE_M2 && progress.planned >= CRANE_STOREYS &&
				hash(way.id, 8) % 2 == 0) {
			Cell own = has_core ? Cell{core_x + core_size, core_z + core_size}
								: Cell{mx, mz};
			if (plan.inside(own.first, own.second))
				crane = own;
		}
		if (crane)
			structures::crane::place_on_site(
					editor, crane->first, crane->second, base_y + 1, hash(way.id, 9));
	}
}
} // namespace arnis::building_construction
