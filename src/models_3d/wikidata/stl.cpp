#include "stl.h"
#include <cstring>
#include <stdexcept>
#include <limits>
#include <cmath>
#include <string>
namespace arnis::models_3d
{
static std::uint32_t read_le_u32(const std::uint8_t *p)
{
	return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
		   (static_cast<std::uint32_t>(p[2]) << 16) |
		   (static_cast<std::uint32_t>(p[3]) << 24);
}

static bool looks_like_binary_with_solid_header(const std::vector<std::uint8_t> &bytes)
{
	if (bytes.size() < 84)
		return false;
	const std::uint64_t count = read_le_u32(bytes.data() + 80);
	const std::uint64_t expected = 84 + 50 * count;
	return expected == bytes.size() ||
		   (expected <= bytes.size() && expected + 4 >= bytes.size());
}

static float f(const std::uint8_t *p)
{
	const std::uint32_t bits = read_le_u32(p);
	float v;
	std::memcpy(&v, &bits, sizeof(v));
	return v;
}
std::vector<Triangle> parse_binary_stl(const std::vector<std::uint8_t> &b)
{
	if (b.size() < 84)
		throw std::runtime_error("STL too short for header + count");
	// Binary STL can legitimately have a header beginning with "solid"; only
	// reject it when its declared binary layout does not fit the payload.
	const bool solid = b.size() >= 5 && std::memcmp(b.data(), "solid", 5) == 0;
	if (solid && !looks_like_binary_with_solid_header(b))
		throw std::runtime_error("ASCII STL not supported");
	const std::uint32_t n = read_le_u32(b.data() + 80);
	const std::uint64_t body_bytes = 50ull * n;
	const std::uint64_t expected = 84ull + body_bytes;
	if (b.size() < expected)
		throw std::runtime_error("STL truncated: header says " + std::to_string(n) +
								 " triangles, body has " + std::to_string(b.size() - 84) +
								 " bytes (need " + std::to_string(body_bytes) + ")");
	std::vector<Triangle> o;
	o.reserve(n);
	for (std::uint32_t i = 0; i < n; ++i) {
		const auto *p = b.data() + 84 + i * 50 + 12;
		Triangle t{};
		for (auto &v : t)
			for (float &c : v) {
				c = f(p);
				p += 4;
			}
		o.push_back(t);
	}
	return o;
}
std::optional<std::pair<std::array<float, 3>, std::array<float, 3>>> stl_bbox(
		const std::vector<Triangle> &ts)
{
	std::array<float, 3> lo{INFINITY, INFINITY, INFINITY},
			hi{-INFINITY, -INFINITY, -INFINITY};
	bool any = false;
	for (auto &t : ts)
		for (auto &v : t) {
			if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2]))
				continue;
			for (int i = 0; i < 3; ++i) {
				lo[i] = std::min(lo[i], v[i]);
				hi[i] = std::max(hi[i], v[i]);
			}
			any = true;
		}
	if (!any)
		return std::nullopt;
	return std::pair{lo, hi};
}
}
