#include "caves_ores.h"

#include "block_definitions.h"
#include "caves_rng.h"
#include "world_editor/floor_state.h"
#include "../../arnis_world_editor.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <optional>
#include <vector>

namespace arnis::caves
{
namespace
{
constexpr double PI = 3.14159265358979323846;

enum class CountKind
{
	Attempts,
	Rarity
};
enum class Distribution
{
	Uniform,
	Trapezoid
};

struct Ore
{
	Block stone;
	Block deep;
	int size;
	CountKind count_kind;
	int count;
	bool surface_relative;
	Distribution distribution;
	int y_min;
	int y_max;
	double discard;
};

const std::vector<Block> &replaceable_rock()
{
	static const std::vector<Block> blocks{STONE, DEEPSLATE, TUFF, COBBLED_DEEPSLATE,
			GRANITE, DIORITE, ANDESITE, GRAVEL, DIRT};
	return blocks;
}

const std::vector<Ore> &ore_table()
{
	static const std::vector<Ore> ores = [] {
		std::vector<Ore> result;
		auto fixed = [&](Block stone, Block deep, int size, int count,
							 Distribution distribution, int ymin, int ymax,
							 double discard) {
			result.push_back({stone, deep, size, CountKind::Attempts, count, false,
					distribution, ymin, ymax, discard});
		};
		auto rare = [&](Block stone, Block deep, int size, int rarity,
							Distribution distribution, int ymin, int ymax,
							double discard) {
			result.push_back({stone, deep, size, CountKind::Rarity, rarity, false,
					distribution, ymin, ymax, discard});
		};
		auto relative = [&](Block block, int size, CountKind kind, int count,
								Distribution distribution, int ymin, int ymax) {
			result.push_back({block, block, size, kind, count, true, distribution, ymin,
					ymax, 0.0});
		};
		using D = Distribution;
		using C = CountKind;

		// Surface-relative stone-family blobs fill real underground columns even
		// where terrain sits far below vanilla's nominal sea level.
		for (const auto block : {ANDESITE, DIORITE, GRANITE}) {
			relative(block, 64, C::Attempts, 3, D::Uniform, 2, 72);
			relative(block, 112, C::Rarity, 3, D::Uniform, 2, 80);
			relative(block, 160, C::Rarity, 8, D::Uniform, 2, 80);
		}
		relative(DIRT, 33, C::Attempts, 7, D::Uniform, 2, 90);
		relative(GRAVEL, 33, C::Attempts, 10, D::Uniform, 2, 110);
		fixed(TUFF, TUFF, 64, 2, D::Uniform, -64, 0, 0.0);
		rare(TUFF, TUFF, 112, 3, D::Uniform, -64, 0, 0.0);
		rare(TUFF, TUFF, 160, 8, D::Uniform, -64, 0, 0.0);
		fixed(GRAVEL, GRAVEL, 33, 6, D::Uniform, -64, 0, 0.0);

		fixed(COAL_ORE, DEEPSLATE_COAL_ORE, 17, 30, D::Uniform, 136, 256, 0.0);
		fixed(COAL_ORE, DEEPSLATE_COAL_ORE, 17, 20, D::Trapezoid, 0, 192, 0.5);
		fixed(IRON_ORE, DEEPSLATE_IRON_ORE, 9, 90, D::Trapezoid, 80, 256, 0.0);
		fixed(IRON_ORE, DEEPSLATE_IRON_ORE, 9, 10, D::Trapezoid, -24, 56, 0.0);
		fixed(IRON_ORE, DEEPSLATE_IRON_ORE, 4, 10, D::Uniform, -64, 72, 0.0);
		fixed(COPPER_ORE, DEEPSLATE_COPPER_ORE, 10, 16, D::Trapezoid, -16, 112, 0.0);
		fixed(COPPER_ORE, DEEPSLATE_COPPER_ORE, 20, 16, D::Trapezoid, -16, 112, 0.0);
		fixed(GOLD_ORE, DEEPSLATE_GOLD_ORE, 9, 4, D::Trapezoid, -64, 32, 0.5);
		fixed(REDSTONE_ORE, DEEPSLATE_REDSTONE_ORE, 8, 4, D::Uniform, -64, 15, 0.0);
		fixed(REDSTONE_ORE, DEEPSLATE_REDSTONE_ORE, 8, 8, D::Trapezoid, -96, -32, 0.0);
		fixed(LAPIS_ORE, DEEPSLATE_LAPIS_ORE, 7, 2, D::Trapezoid, -32, 32, 0.0);
		fixed(LAPIS_ORE, DEEPSLATE_LAPIS_ORE, 7, 4, D::Uniform, -64, 64, 1.0);
		fixed(DIAMOND_ORE, DEEPSLATE_DIAMOND_ORE, 4, 7, D::Trapezoid, -64, 16, 0.5);
		fixed(DIAMOND_ORE, DEEPSLATE_DIAMOND_ORE, 8, 2, D::Uniform, -64, -4, 0.5);
		rare(DIAMOND_ORE, DEEPSLATE_DIAMOND_ORE, 12, 9, D::Trapezoid, -64, 16, 0.7);
		fixed(DIAMOND_ORE, DEEPSLATE_DIAMOND_ORE, 8, 4, D::Trapezoid, -64, 16, 1.0);
		return result;
	}();
	return ores;
}

XoroRandom chunk_rng(std::int64_t seed, int cx, int cz)
{
	const auto state = std::bit_cast<std::uint64_t>(seed) ^
					   (std::uint64_t(std::int64_t(cx)) * 0x1F1F3C71ULL) ^
					   (std::uint64_t(std::int64_t(cz)) * 0x9E3779B1ULL) ^ 0x0EE50EE5ULL;
	return XoroRandom::from_seed(std::bit_cast<std::int64_t>(state));
}

int sample_y(Distribution distribution, int low, int high, XoroRandom &random)
{
	const int range = high - low;
	if (range <= 0)
		return low;
	if (distribution == Distribution::Uniform)
		return low + random.next_int(range + 1);
	return low + (random.next_int(range + 1) + random.next_int(range + 1)) / 2;
}

unsigned air_neighbors(const world_editor::WorldEditor &editor, int x, int y, int z)
{
	unsigned count = 0;
	for (const auto &[dx, dy, dz] : {std::array<int, 3>{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
				 {0, -1, 0}, {0, 0, 1}, {0, 0, -1}})
		count += !editor.block_exists_absolute(x + dx, y + dy, z + dz);
	return count;
}

void place_blob(world_editor::WorldEditor &editor, int cx, int cy, int cz, const Ore &ore,
		XoroRandom &random, const CaveRect &region)
{
	const double angle = random.next_float() * PI;
	const double half_length = ore.size / 8.0;
	const double axis_x = std::sin(angle);
	const double axis_z = std::cos(angle);
	int placed = 0;
	const auto &replaceable = replaceable_rock();
	const std::optional<std::vector<Block>> replaceable_filter{replaceable};
	const std::optional<std::vector<Block>> deep_filter{
			std::vector<Block>{DEEPSLATE, COBBLED_DEEPSLATE}};
	for (int i = 0; i < ore.size; ++i) {
		const double t = double(i) / ore.size;
		const double px = cx + axis_x * half_length * (t * 2.0 - 1.0);
		const double pz = cz + axis_z * half_length * (t * 2.0 - 1.0);
		const int py = cy + random.next_int(3) - 2;
		const double radius = (std::sin(PI * t) + 1.0) * (ore.size / 16.0) * 0.5 + 0.5;
		const int integer_radius = static_cast<int>(std::ceil(radius));
		const double radius_squared = radius * radius;
		for (int dx = -integer_radius; dx <= integer_radius; ++dx)
			for (int dy = -integer_radius; dy <= integer_radius; ++dy)
				for (int dz = -integer_radius; dz <= integer_radius; ++dz) {
					if (double(dx * dx + dy * dy + dz * dz) > radius_squared)
						continue;
					const int bx = static_cast<int>(std::round(px)) + dx;
					const int by = py + dy;
					const int bz = static_cast<int>(std::round(pz)) + dz;
					if (!region.contains(bx, bz) ||
							!editor.block_exists_absolute(bx, by, bz))
						continue;
					const auto exposed = air_neighbors(editor, bx, by, bz);
					if (exposed >= 3)
						continue;
					if (ore.discard > 0.0 && exposed >= 1 &&
							random.next_float() < ore.discard)
						continue;
					const bool deep = editor.check_for_block_absolute(
							bx, by, bz, deep_filter, std::nullopt);
					const Block block = deep ? ore.deep : ore.stone;
					editor.set_block_absolute(
							block, bx, by, bz, replaceable_filter, std::nullopt);
					if (++placed >= ore.size)
						return;
				}
	}
}

int floor_div_16(int value)
{
	const int quotient = value / 16;
	const int remainder = value % 16;
	return remainder < 0 ? quotient - 1 : quotient;
}
} // namespace

void place_ores_region(world_editor::WorldEditor &editor, const CaveRect &region,
		std::int64_t seed, int floor_y)
{
	if (region.min_x > region.max_x || region.min_z > region.max_z)
		return;
	const int cx0 = floor_div_16(region.min_x) - 1;
	const int cx1 = floor_div_16(region.max_x) + 1;
	const int cz0 = floor_div_16(region.min_z) - 1;
	const int cz1 = floor_div_16(region.max_z) + 1;
	const int shift = floor_y - (-64);
	for (int cx = cx0; cx <= cx1; ++cx)
		for (int cz = cz0; cz <= cz1; ++cz) {
			auto random = chunk_rng(seed, cx, cz);
			for (const auto &ore : ore_table()) {
				int attempts = ore.count;
				if (ore.count_kind == CountKind::Rarity)
					attempts = random.next_int(ore.count) == 0 ? 1 : 0;
				for (int attempt = 0; attempt < attempts; ++attempt) {
					const int x = cx * 16 + random.next_int(16);
					const int z = cz * 16 + random.next_int(16);
					const int ground_y = editor.get_ground_level(x, z);
					const int y =
							ore.surface_relative
									? ground_y - sample_y(ore.distribution, ore.y_min,
														 ore.y_max, random)
									: sample_y(ore.distribution, ore.y_min, ore.y_max,
											  random) +
											  shift;
					if (y > ground_y - 2 || !editor.block_exists_absolute(x, y, z))
						continue;
					place_blob(editor, x, y, z, ore, random, region);
				}
			}
		}
}
} // namespace arnis::caves
