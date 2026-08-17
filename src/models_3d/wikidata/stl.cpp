#include "stl.h"
#include <cstring>
#include <stdexcept>
#include <limits>
#include <cmath>
namespace arnis::models_3d
{
static float f(const std::uint8_t *p)
{
	float v;
	std::memcpy(&v, p, 4);
	return v;
}
std::vector<Triangle> parse_binary_stl(const std::vector<std::uint8_t> &b)
{
	if (b.size() < 84)
		throw std::runtime_error("STL too short");
	std::uint32_t n = b[80] | b[81] << 8 | b[82] << 16 | b[83] << 24;
	const std::uint64_t expected = 84ull + 50ull * n;
	// Binary STL can legitimately have a header beginning with "solid"; only
	// reject it when its declared binary layout does not fit the payload.
	const bool solid = b.size() >= 5 && std::memcmp(b.data(), "solid", 5) == 0;
	if (solid && expected != b.size() && expected + 4 < b.size())
		throw std::runtime_error("ASCII STL not supported");
	if (b.size() < expected)
		throw std::runtime_error("STL truncated");
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
std::pair<std::array<float, 3>, std::array<float, 3>> stl_bbox(
		const std::vector<Triangle> &ts)
{
	std::array<float, 3> lo{INFINITY, INFINITY, INFINITY},
			hi{-INFINITY, -INFINITY, -INFINITY};
	bool any = false;
	for (auto &t : ts)
		for (auto &v : t)
			if (std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]))
				for (int i = 0; i < 3; ++i) {
					lo[i] = std::min(lo[i], v[i]);
					hi[i] = std::max(hi[i], v[i]);
					any = true;
				}
	if (!any)
		throw std::runtime_error("STL has no finite vertices");
	return {lo, hi};
}
}
