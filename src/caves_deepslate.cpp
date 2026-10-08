#include "caves_deepslate.h"
#include "block_definitions.h"
#include "land_cover/land_cover.h"
#include "world_editor/floor_state.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace arnis::caves
{
namespace
{
std::uint64_t hash3(int x, int y, int z, std::uint64_t seed)
{
	auto mix = [](int v, std::uint64_t k) {
		auto h = (std::uint64_t(std::uint32_t(v)) + 0x9E3779B97F4A7C15ULL) * k;
		return h ^ (h >> 29);
	};
	auto h = seed ^ mix(x, 0x9E3779B97F4A7C15ULL) ^ mix(y, 0xC2B2AE3D27D4EB4FULL) ^
			 mix(z, 0x165667B19E3779F9ULL);
	h *= 0x9E3779B97F4A7C15ULL;
	return h ^ (h >> 29);
}
double noise(int x, int y, int z)
{
	constexpr int p = 24;
	auto floor = [](int v) {
		const int quotient = v / p;
		const int remainder = v % p;
		return remainder < 0 ? quotient - 1 : quotient;
	};
	int x0 = floor(x), y0 = floor(y), z0 = floor(z);
	double fx = double(x - x0 * p) / p, fy = double(y - y0 * p) / p,
		   fz = double(z - z0 * p) / p;
	auto sm = [](double t) { return t * t * (3 - 2 * t); };
	fx = sm(fx);
	fy = sm(fy);
	fz = sm(fz);
	auto c = [&](int a, int b, int d) {
		return double(hash3(a, b, d, 0xDEE95147) % 100000) / 100000.;
	};
	auto l = [](double a, double b, double t) { return a + (b - a) * t; };
	double a = l(l(c(x0, y0, z0), c(x0 + 1, y0, z0), fx),
			l(c(x0, y0 + 1, z0), c(x0 + 1, y0 + 1, z0), fx), fy);
	double b = l(l(c(x0, y0, z0 + 1), c(x0 + 1, y0, z0 + 1), fx),
			l(c(x0, y0 + 1, z0 + 1), c(x0 + 1, y0 + 1, z0 + 1), fx), fy);
	return l(a, b, fz);
}
}
void apply_deepslate(
		world_editor::WorldEditor &e, int min_x, int max_x, int min_z, int max_z)
{
	const int floor = world_editor::terrain_floor_y(), band_floor = floor + 5;
	for (int x = min_x; x <= max_x; ++x)
		for (int z = min_z; z <= max_z; ++z) {
			const int ceiling = e.get_ground_level(x, z) - 10,
					  band = ceiling - band_floor;
			if (band < 8)
				continue;
			const int line = band_floor + band * 30 / 100 +
							 int(std::round((noise(x, 0, z) - .5) * 4));
			const int transition = std::max(3, band * 5 / 100);
			for (int y = floor + 1; y <= line + transition; ++y)
				if (e.check_for_block_absolute(
							x, y, z, std::optional<std::vector<Block>>{{STONE}})) {
					const auto frac = std::uint64_t(std::max(0, line + transition - y)) *
									  100 / std::uint64_t(std::max(1, transition));
					if (y <= line || hash3(x, y, z, 0xDEE95147 ^ 0x6A6A) % 100 < frac)
						e.set_block_absolute(DEEPSLATE, x, y, z,
								std::optional<std::vector<Block>>{{STONE}});
				}
		}
}
}
