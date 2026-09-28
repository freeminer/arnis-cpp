#include "shoreline.h"
#include "land_cover.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <new>
#include <stdexcept>
#include <tuple>

namespace arnis::land_cover
{
namespace
{
using Pt = std::pair<int, int>;
using FPt = std::pair<double, double>;
constexpr double MIDPOINT_TOLERANCE_PX = 0.72;
constexpr double CORNER_TOLERANCE_PX = 1.0;

bool collinear_axis(Pt a, Pt b, Pt c)
{
	return (a.first == b.first && b.first == c.first) ||
		   (a.second == b.second && b.second == c.second);
}
Pt side_start(int x, int y, int side)
{
	return {x + (side == 1 || side == 2), y + (side == 2 || side == 3)};
}

// Four-connected water regions, including holes. Prefer the same pixel at
// diagonal touches, exactly as shoreline.rs, so touching ponds stay separate.
std::optional<std::vector<std::vector<Pt>>> trace_boundaries(
		const std::vector<std::uint8_t> &water, int w, int h)
{
	const auto is_water = [&](int x, int y) {
		return x >= 0 && y >= 0 && x < w && y < h &&
			   water[std::size_t(y) * w + x] == LC_WATER;
	};
	const auto boundary = [&](int x, int y, int s) {
		constexpr int dx[] = {0, 1, 0, -1}, dy[] = {-1, 0, 1, 0};
		return is_water(x, y) && !is_water(x + dx[s], y + dy[s]);
	};
	const auto edge_id = [&](int x, int y, int s) {
		return (std::size_t(y) * w + x) * 4 + s;
	};
	std::vector<bool> visited(std::size_t(w) * h * 4);
	std::vector<std::vector<Pt>> rings;
	for (int y = 0; y < h; ++y)
		for (int x = 0; x < w; ++x)
			for (int s = 0; s < 4; ++s) {
				if (!boundary(x, y, s) || visited[edge_id(x, y, s)])
					continue;
				int cx = x, cy = y, cs = s;
				std::vector<Pt> ring;
				std::size_t steps = 0;
				while (!visited[edge_id(cx, cy, cs)]) {
					visited[edge_id(cx, cy, cs)] = true;
					const auto p = side_start(cx, cy, cs);
					if (ring.size() >= 2 &&
							collinear_axis(ring[ring.size() - 2], ring.back(), p))
						ring.back() = p;
					else
						ring.push_back(p);
					const int ns = (cs + 1) % 4;
					if (boundary(cx, cy, ns))
						cs = ns;
					else {
						const auto [vx, vy] = side_start(cx, cy, ns);
						std::optional<std::tuple<int, int, int>> found;
						for (const auto &[qx, qy, qs] :
								std::array<std::tuple<int, int, int>, 4>{
										{{vx - 1, vy - 1, 2}, {vx, vy - 1, 3},
												{vx - 1, vy, 1}, {vx, vy, 0}}}) {
							if ((qx == cx && qy == cy) || !boundary(qx, qy, qs))
								continue;
							if (found)
								return std::nullopt;
							found = std::tuple{qx, qy, qs};
						}
						if (!found)
							return std::nullopt;
						std::tie(cx, cy, cs) = *found;
					}
					if (++steps > visited.size() + 4)
						return std::nullopt;
				}
				if (std::tuple{cx, cy, cs} != std::tuple{x, y, s})
					return std::nullopt;
				while (ring.size() >= 3) {
					if (collinear_axis(ring.back(), ring[0], ring[1]))
						ring.erase(ring.begin());
					else if (collinear_axis(ring[ring.size() - 2], ring.back(), ring[0]))
						ring.pop_back();
					else
						break;
				}
				if (ring.size() >= 3)
					rings.push_back(std::move(ring));
			}
	return rings;
}

struct FitLine
{
	double cx, cy, dx, dy;
	static FitLine through(FPt a, FPt b)
	{
		const double dx = b.first - a.first, dy = b.second - a.second,
					 len = std::hypot(dx, dy);
		return {a.first, a.second, len > 0 ? dx / len : 1.0, len > 0 ? dy / len : 0.0};
	}
	static FitLine fit_run(const std::vector<Pt> &pts)
	{
		const auto fallback = through(pts.front(), pts.back());
		if (pts.size() < 3)
			return fallback;
		double sum = 0, cx = 0, cy = 0;
		std::vector<std::tuple<double, double, double>> mids;
		for (std::size_t i = 1; i < pts.size(); ++i) {
			const auto [ax, ay] = pts[i - 1];
			const auto [bx, by] = pts[i];
			const double w = std::hypot(double(bx) - ax, double(by) - ay);
			const double x = (double(ax) + bx) * 0.5, y = (double(ay) + by) * 0.5;
			sum += w;
			cx += w * x;
			cy += w * y;
			mids.emplace_back(x, y, w);
		}
		if (sum <= 0)
			return fallback;
		cx /= sum;
		cy /= sum;
		double sxx = 0, syy = 0, sxy = 0;
		for (const auto &[x, y, w] : mids) {
			sxx += w * (x - cx) * (x - cx);
			syy += w * (y - cy) * (y - cy);
			sxy += w * (x - cx) * (y - cy);
		}
		if (sxx + syy <= 1e-12)
			return fallback;
		const double theta = 0.5 * std::atan2(2 * sxy, sxx - syy);
		return {cx, cy, std::cos(theta), std::sin(theta)};
	}
	double residual(FPt p) const
	{
		return std::abs((p.first - cx) * dy - (p.second - cy) * dx);
	}
	FPt project(FPt p) const
	{
		const double t = (p.first - cx) * dx + (p.second - cy) * dy;
		return {cx + t * dx, cy + t * dy};
	}
	FPt join(const FitLine &next, FPt orig) const
	{
		const double cross = dx * next.dy - dy * next.dx;
		if (std::abs(cross) >= 0.087) {
			const double t =
					((next.cx - cx) * next.dy - (next.cy - cy) * next.dx) / cross;
			const FPt p{cx + t * dx, cy + t * dy};
			if (std::hypot(p.first - orig.first, p.second - orig.second) <= 1.0)
				return p;
		}
		if (residual(orig) > MIDPOINT_TOLERANCE_PX ||
				next.residual(orig) > MIDPOINT_TOLERANCE_PX)
			return orig;
		const auto a = project(orig), b = next.project(orig);
		return {(a.first + b.first) * 0.5, (a.second + b.second) * 0.5};
	}
};

std::optional<std::size_t> split_point(
		const std::vector<Pt> &pts, const FitLine *against = nullptr)
{
	const auto n = pts.size();
	if (n < 3)
		return std::nullopt;
	const auto line = against ? *against : FitLine::fit_run(pts);
	double worst_mid = 0, worst_corner = -1;
	std::size_t worst_edge = 0, split = 1;
	std::vector<double> residuals;
	for (const auto &p : pts)
		residuals.push_back(line.residual(p));
	for (std::size_t e = 0; e + 1 < n; ++e) {
		const double r = line.residual({(double(pts[e].first) + pts[e + 1].first) * 0.5,
				(double(pts[e].second) + pts[e + 1].second) * 0.5});
		if (r > worst_mid) {
			worst_mid = r;
			worst_edge = e;
		}
	}
	for (std::size_t i = 1; i + 1 < n; ++i)
		if (residuals[i] > worst_corner) {
			worst_corner = residuals[i];
			split = i;
		}
	if (worst_corner <= CORNER_TOLERANCE_PX) {
		if (worst_mid <= MIDPOINT_TOLERANCE_PX)
			return std::nullopt;
		const auto u = worst_edge, v = u + 1;
		if (u >= 1 && v <= n - 2)
			split = residuals[u] >= residuals[v] ? u : v;
		else
			split = u >= 1 ? u : v;
	}
	worst_corner = residuals[split];
	const auto flank = [&](std::size_t i) {
		const auto a = pts[i - 1], b = pts[i], c = pts[i + 1];
		return std::abs(std::int64_t(a.first) - b.first) +
			   std::abs(std::int64_t(a.second) - b.second) +
			   std::abs(std::int64_t(c.first) - b.first) +
			   std::abs(std::int64_t(c.second) - b.second);
	};
	auto best = flank(split);
	const auto lo = split > 2 ? split - 2 : 1, hi = std::min(split + 2, n - 2);
	for (auto cand = lo; cand <= hi; ++cand)
		if (cand != split && residuals[cand] >= worst_corner - 0.75 &&
				flank(cand) > best) {
			best = flank(cand);
			split = cand;
		}
	return split;
}

std::vector<Pt> cyclic_points(const std::vector<Pt> &ring, std::size_t i, std::size_t j)
{
	std::vector<Pt> out{ring[i]};
	do {
		i = (i + 1) % ring.size();
		out.push_back(ring[i]);
	} while (i != j);
	return out;
}

std::vector<FPt> simplify_ring(const std::vector<Pt> &ring)
{
	const auto n = ring.size();
	const auto exact = [&] { return std::vector<FPt>(ring.begin(), ring.end()); };
	if (n <= 4)
		return exact();
	std::size_t k = 1;
	double best = -1;
	for (std::size_t i = 1; i < n; ++i) {
		const double dx = double(ring[i].first) - ring[0].first,
					 dy = double(ring[i].second) - ring[0].second;
		if (dx * dx + dy * dy > best) {
			best = dx * dx + dy * dy;
			k = i;
		}
	}
	std::vector<bool> keep(n);
	keep[0] = keep[k] = true;
	std::vector<std::pair<std::size_t, std::size_t>> stack{{0, k}, {k, n}};
	while (!stack.empty()) {
		const auto [i, j] = stack.back();
		stack.pop_back();
		if (j <= i + 1)
			continue;
		if (auto m = split_point(cyclic_points(ring, i, j % n))) {
			keep[i + *m] = true;
			stack.emplace_back(i, i + *m);
			stack.emplace_back(i + *m, j);
		}
	}
	const auto kept_indices = [&] {
		std::vector<std::size_t> out;
		for (std::size_t i = 0; i < n; ++i)
			if (keep[i])
				out.push_back(i);
		return out;
	};
	for (auto seed : {k, std::size_t{0}}) {
		const auto kept = kept_indices();
		if (kept.size() <= 3)
			break;
		const auto pos = std::find(kept.begin(), kept.end(), seed) - kept.begin();
		if (!split_point(cyclic_points(ring, kept[(pos + kept.size() - 1) % kept.size()],
					kept[(pos + 1) % kept.size()])))
			keep[seed] = false;
	}
	for (;;) {
		const auto kept = kept_indices();
		const auto m = kept.size();
		if (m < 3)
			return exact();
		std::vector<std::vector<Pt>> runs;
		std::vector<FitLine> lines;
		for (std::size_t s = 0; s < m; ++s) {
			runs.push_back(cyclic_points(ring, kept[s], kept[(s + 1) % m]));
			lines.push_back(FitLine::fit_run(runs.back()));
		}
		std::vector<FPt> placed;
		for (std::size_t s = 0; s < m; ++s)
			placed.push_back(lines[(s + m - 1) % m].join(lines[s], ring[kept[s]]));
		bool split_any = false;
		for (std::size_t s = 0; s < m; ++s) {
			const auto rendered = FitLine::through(placed[s], placed[(s + 1) % m]);
			if (auto local = split_point(runs[s], &rendered)) {
				keep[(kept[s] + *local) % n] = true;
				split_any = true;
			}
		}
		if (!split_any)
			return placed;
	}
}
}

bool reconstruct_water_shoreline(LandCoverData &data)
try {
	const auto &bbox = data.source_bounds;
	const auto gw = data.width, gh = data.height;
	if (gw < 2 || gh < 2 || data.source_tiles.empty() || !(bbox.max_lng > bbox.min_lng) ||
			!(bbox.max_lat > bbox.min_lat))
		return false;
	// Join raw windows before tracing, avoiding artificial COG tile-edge shores.
	double ppd = 0;
	for (const auto &tile : data.source_tiles) {
		if (!tile.valid())
			continue;
		const double dx =
				tile.min_lng != tile.max_lng ? tile.max_lng - tile.min_lng : 3.0;
		const double dy =
				tile.min_lat != tile.max_lat ? tile.max_lat - tile.min_lat : 3.0;
		const double p =
				tile.pixels_per_degree > 0 ? tile.pixels_per_degree : tile.width / dx;
		if (!std::isfinite(p) || p <= 0 || std::abs(tile.height - dy * p) > 1e-5 ||
				std::abs(tile.width - dx * p) > 1e-5 || (ppd && std::abs(p - ppd) > 1e-5))
			return false;
		ppd = p;
	}
	if (!ppd)
		return false;
	const double sx = (gw - 1) / ((bbox.max_lng - bbox.min_lng) * ppd);
	const double sy = (gh - 1) / ((bbox.max_lat - bbox.min_lat) * ppd);
	if (!std::isfinite(sx) || !std::isfinite(sy) || std::max(sx, sy) < 2.0)
		return false;
	const double ox = (bbox.min_lng + 180.0) * ppd, oy = (90.0 - bbox.max_lat) * ppd;
	const double x0 = std::floor(ox), y0 = std::floor(oy);
	const double wd = std::floor((bbox.max_lng + 180.0) * ppd) - x0 + 1;
	const double hd = std::floor((90.0 - bbox.min_lat) * ppd) - y0 + 1;
	if (!std::isfinite(wd) || !std::isfinite(hd) || wd < 1 || hd < 1 ||
			wd > std::numeric_limits<int>::max() ||
			hd > std::numeric_limits<int>::max() ||
			wd * hd > double(std::uint64_t{1} << 31))
		return false;
	const int w = int(wd), h = int(hd);
	std::vector<std::uint8_t> raster(std::size_t(w) * h);
	for (int y = 0; y < h; ++y)
		for (int x = 0; x < w; ++x)
			for (const auto &tile : data.source_tiles) {
				const auto cls = tile.sample(
						90.0 - (y0 + y + 0.5) / ppd, (x0 + x + 0.5) / ppd - 180.0);
				if (cls) {
					raster[std::size_t(y) * w + x] = cls;
					break;
				}
			}
	const auto rings = trace_boundaries(raster, w, h);
	if (!rings || rings->empty())
		return false;
	std::vector<std::vector<double>> rows(gh);
	const auto span = [](double v, std::size_t limit) {
		return std::size_t(std::clamp(std::ceil(v), 0.0, double(limit)));
	};
	for (const auto &ring : *rings) {
		const auto simplified = simplify_ring(ring);
		for (std::size_t i = 0; i < simplified.size(); ++i) {
			const auto a = simplified[i], b = simplified[(i + 1) % simplified.size()];
			const double ay = (a.second + y0 - oy) * sy, by = (b.second + y0 - oy) * sy;
			if (ay == by)
				continue;
			const double ax = (a.first + x0 - ox) * sx, bx = (b.first + x0 - ox) * sx;
			for (auto z = span(std::min(ay, by), gh); z < span(std::max(ay, by), gh); ++z)
				rows[z].push_back(ax + (double(z) - ay) / (by - ay) * (bx - ax));
		}
	}
	std::vector<bool> mask(gw * gh);
	std::size_t before = 0, after = 0;
	if (data.grid.size() < gh)
		return false;
	for (std::size_t z = 0; z < gh; ++z) {
		if (data.grid[z].size() < gw)
			return false;
		auto &xs = rows[z];
		std::sort(xs.begin(), xs.end());
		for (std::size_t i = 0; i + 1 < xs.size(); i += 2)
			for (auto x = span(xs[i], gw); x < span(xs[i + 1], gw); ++x)
				mask[z * gw + x] = true;
		for (std::size_t x = 0; x < gw; ++x) {
			before += data.grid[z][x] == LC_WATER;
			after += mask[z * gw + x];
		}
	}
	if (std::abs(double(after) - double(before)) > 0.10 * before + 4 * sx * sy)
		return false;
	const int radius = int(std::clamp(std::ceil(1.5 * std::max(sx, sy)), 2.0, 64.0));
	// Defer all writes so nearest-land choices see the original classification.
	std::vector<std::tuple<std::size_t, std::size_t, std::uint8_t>> changes;
	for (std::size_t z = 0; z < gh; ++z)
		for (std::size_t x = 0; x < gw; ++x) {
			const bool water = mask[z * gw + x];
			if (water == (data.grid[z][x] == LC_WATER))
				continue;
			if (water) {
				changes.emplace_back(x, z, LC_WATER);
				continue;
			}
			bool found = false;
			for (int r = 1; r <= radius && !found; ++r)
				for (int dz = -r; dz <= r && !found; ++dz)
					for (int dx = -r; dx <= r; dx += std::abs(dz) == r ? 1 : 2 * r) {
						const auto nx = std::int64_t(x) + dx, nz = std::int64_t(z) + dz;
						if (nx < 0 || nz < 0 || nx >= std::int64_t(gw) ||
								nz >= std::int64_t(gh))
							continue;
						const auto cls = data.grid[nz][nx];
						if (cls && cls != LC_WATER) {
							changes.emplace_back(x, z, cls);
							found = true;
							break;
						}
					}
		}
	for (const auto &[x, z, cls] : changes)
		data.grid[z][x] = cls;
	return !changes.empty();
} catch (const std::bad_alloc &) {
	// All writes are deferred until allocation and reconstruction succeed.
	return false;
} catch (const std::length_error &) {
	return false;
}
}
