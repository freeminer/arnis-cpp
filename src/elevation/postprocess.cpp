#include "postprocess.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
namespace arnis::elevation
{
void repair_terrain_anomalies(std::vector<std::vector<double>> &h)
{
	if (h.size() < 5 || h[0].size() < 5)
		return;
	const int H = h.size(), W = h[0].size();
	for (int pass = 0; pass < 10; ++pass) {
		auto s = h;
		std::size_t changed = 0;
		for (int y = 2; y < H - 2; ++y)
			for (int x = 2; x < W - 2; ++x) {
				double c = s[y][x];
				if (!std::isfinite(c))
					continue;
				std::vector<double> n;
				for (int dy = -2; dy <= 2; ++dy)
					for (int dx = -2; dx <= 2; ++dx)
						if (dx || dy)
							if (std::isfinite(s[y + dy][x + dx]))
								n.push_back(s[y + dy][x + dx]);
				if (n.size() < 8)
					continue;
				std::nth_element(n.begin(), n.begin() + n.size() / 2, n.end());
				double m = n[n.size() / 2];
				std::vector<double> d;
				for (double v : n)
					d.push_back(std::abs(v - m));
				std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
				double mad = d[d.size() / 2];
				if (std::abs(c - m) > 6.0 && std::abs(c - m) > 3.0 * std::max(1.0, mad)) {
					h[y][x] = m;
					++changed;
				}
			}
		if (!changed)
			break;
	}
}
std::tuple<std::vector<std::vector<double>>, double, double> scale_to_minecraft(
		const std::vector<std::vector<double>> &in, double scale, int ground,
		bool disable, int extended)
{
	double lo = 1e300, hi = -1e300;
	for (auto &r : in)
		for (double v : r)
			if (std::isfinite(v)) {
				lo = std::min(lo, v);
				hi = std::max(hi, v);
			}
	if (!(lo <= hi) || !std::isfinite(lo)) {
		lo = 0;
		hi = 0;
	}
	double range = hi - lo;
	int maxy = disable ? extended : 319;
	double avail = std::max(0.0, double(maxy - 15 - ground));
	double factor = range > 0 ? std::min(scale, avail / range) : 0;
	std::vector<std::vector<double>> out = in;
	for (auto &r : out)
		for (double &v : r)
			v = std::isfinite(v) ? std::clamp(ground + (v - lo) * factor, double(-64),
										   double(maxy - 15))
								 : ground;
	return {std::move(out), lo, factor};
}
}
