#include "provider.h"
#include "providers.h"
#include "elevation.h"
#include "postprocess.h"
#include <cmath>
#include <utility>
#include <fstream>
#include <cstdint>
#include <algorithm>
#include <limits>
namespace arnis::elevation
{
std::optional<std::vector<std::int16_t>> read_hgt(const Tile &t)
{
	std::ifstream f(t.file, std::ios::binary);
	if (!f)
		return std::nullopt;
	f.seekg(0, std::ios::end);
	auto n = f.tellg();
	if (n <= 0 || n % 2)
		return std::nullopt;
	f.seekg(0);
	std::vector<std::int16_t> v(std::size_t(n) / 2);
	for (auto &x : v) {
		unsigned char b[2];
		f.read(reinterpret_cast<char *>(b), 2);
		if (!f)
			return std::nullopt;
		x = std::int16_t((std::uint16_t(b[0]) << 8) | b[1]);
	}
	return v;
}
double sample_hgt(const std::vector<std::int16_t> &v, std::size_t side, double lat,
		double lon, const Tile &t)
{
	if (side < 2 || v.size() < side * side)
		return 0.0;
	double fx = (lon - t.lon) * (side - 1), fy = (t.lat + 1 - lat) * (side - 1);
	fx = std::clamp(fx, 0.0, double(side - 1));
	fy = std::clamp(fy, 0.0, double(side - 1));
	auto at = [&](std::size_t x, std::size_t y) {
		auto q = v[y * side + x];
		return q == -32768 ? 0.0 : double(q);
	};
	std::size_t x = std::min<std::size_t>(side - 2, std::size_t(fx)),
				y = std::min<std::size_t>(side - 2, std::size_t(fy));
	double ax = fx - x, ay = fy - y;
	return (1 - ay) * ((1 - ax) * at(x, y) + ax * at(x + 1, y)) +
		   ay * ((1 - ax) * at(x, y + 1) + ax * at(x + 1, y + 1));
}
std::optional<double> sample_tile(const Tile &t, double lat, double lon)
{
	auto v = read_hgt(t);
	if (!v)
		return std::nullopt;
	const auto side = std::size_t(std::llround(std::sqrt(double(v->size()))));
	if (side * side != v->size())
		return std::nullopt;
	return sample_hgt(*v, side, lat, lon, t);
}
std::optional<SampleResult> sample_with_source(CachedProvider &p, double lat, double lon)
{
	auto t = p.fetch_tile(lat, lon);
	if (!t)
		return std::nullopt;
	auto h = sample_tile(t->tile, lat, lon);
	return h ? std::optional<SampleResult>(SampleResult{*h, t->source}) : std::nullopt;
}
SourceGrid build_source_grid(CachedProvider &p, double a, double b, double c, double d,
		std::size_t w, std::size_t h)
{
	SourceGrid out;
	out.data.width = w;
	out.data.height = h;
	out.data.heights.assign(h, std::vector<double>(w));
	out.sources.assign(h, std::vector<providers::Source>(w, providers::Source::None));
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x) {
			double lat = a + (c - a) * (h == 1 ? 0.0 : double(y) / (h - 1)),
				   lon = b + (d - b) * (w == 1 ? 0.0 : double(x) / (w - 1));
			if (auto s = sample_with_source(p, lat, lon)) {
				out.data.heights[y][x] = s->height;
				out.sources[y][x] = s->source;
			}
		}
	return out;
}
std::array<std::size_t, 5> source_counts(const SourceGrid &g)
{
	std::array<std::size_t, 5> n{};
	for (const auto &r : g.sources)
		for (auto s : r)
			n[std::size_t(s)]++;
	return n;
}
double source_coverage(const SourceGrid &g)
{
	if (!g.data.width || !g.data.height)
		return 0.0;
	auto n = source_counts(g);
	return 1.0 - double(n[std::size_t(providers::Source::None)]) /
						 double(g.data.width * g.data.height);
}
bool acceptable_source_coverage(const SourceGrid &g, double minimum)
{
	return minimum <= 1.0 && source_coverage(g) >= std::max(0.0, minimum);
}
void fill_source_gaps(SourceGrid &g, double fallback)
{
	for (std::size_t y = 0; y < g.data.height; ++y)
		for (std::size_t x = 0; x < g.data.width; ++x)
			if (g.sources[y][x] == providers::Source::None) {
				double sum = 0;
				int n = 0;
				for (int dy = -1; dy <= 1; ++dy)
					for (int dx = -1; dx <= 1; ++dx) {
						std::size_t xx = std::clamp<int>(
											int(x) + dx, 0, int(g.data.width) - 1),
									yy = std::clamp<int>(
											int(y) + dy, 0, int(g.data.height) - 1);
						if (g.sources[yy][xx] != providers::Source::None) {
							sum += g.data.heights[yy][xx];
							++n;
						}
					}
				g.data.heights[y][x] = n ? sum / n : fallback;
			}
}
void process_source_grid(SourceGrid &g, double sigma, double fallback)
{
	fill_source_gaps(g, fallback);
	g.data = smooth_grid(g.data, sigma);
}
void normalize_source_grid(SourceGrid &g, double sea, double scale, double lo, double hi)
{
	normalize_grid(g.data, sea, scale, lo, hi);
}
SourceGrid resample_source_grid(const SourceGrid &in, std::size_t w, std::size_t h)
{
	SourceGrid out;
	out.data = resample_grid(in.data, w, h);
	out.sources.assign(h, std::vector<providers::Source>(w, providers::Source::None));
	if (!w || !h || !in.data.width || !in.data.height)
		return out;
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x) {
			std::size_t ix = std::min(in.data.width - 1,
								std::size_t((w == 1 ? 0.0
													: double(x) * (in.data.width - 1) /
															  (w - 1)))),
						iy = std::min(in.data.height - 1,
								std::size_t((h == 1 ? 0.0
													: double(y) * (in.data.height - 1) /
															  (h - 1))));
			out.sources[y][x] = in.sources[iy][ix];
		}
	return out;
}
std::optional<double> sample_tiles(const std::vector<Tile> &tiles, double lat, double lon)
{
	if (tiles.empty())
		return std::nullopt;
	double wrapped = std::fmod(lon + 180.0, 360.0);
	if (wrapped < 0)
		wrapped += 360.0;
	wrapped -= 180.0;
	for (const auto &t : tiles)
		if (lat >= t.lat && lat <= t.lat + 1 && wrapped >= t.lon && wrapped <= t.lon + 1)
			if (auto h = sample_tile(t, lat, wrapped))
				return h;
	return std::nullopt;
}
ElevationData build_grid(const std::vector<Tile> &tiles, double a, double b, double c,
		double d, std::size_t w, std::size_t h)
{
	ElevationData out;
	out.width = w;
	out.height = h;
	out.heights.assign(h, std::vector<double>(w, 0.0));
	if (!w || !h)
		return out;
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x) {
			double lat = a + (c - a) * (h == 1 ? 0.0 : double(y) / double(h - 1)),
				   lon = b + (d - b) * (w == 1 ? 0.0 : double(x) / double(w - 1));
			if (auto v = sample_tiles(tiles, lat, lon))
				out.heights[y][x] = *v;
		}
	return out;
}
ElevationData build_processed_grid(const std::vector<Tile> &tiles, double a, double b,
		double c, double d, std::size_t w, std::size_t h)
{
	auto out = build_grid(tiles, a, b, c, d, w, h);
	fill_nan_values(out.heights);
	filter_elevation_outliers(out.heights);
	// Match the Rust elevation pipeline's final spatial anomaly repair.  Keep
	// this after interpolation/outlier removal so the neighbourhood statistics
	// see a complete finite grid.
	repair_terrain_anomalies(out.heights);
	return out;
}
void normalize_grid(ElevationData &g, double sea, double scale, double lo, double hi)
{
	if (scale <= 0.0)
		scale = 1.0;
	for (auto &row : g.heights)
		for (auto &v : row) {
			v = (v - sea) * scale + sea;
			v = std::clamp(v, lo, hi);
		}
}
ElevationData resample_grid(const ElevationData &in, std::size_t w, std::size_t h)
{
	ElevationData out;
	out.width = w;
	out.height = h;
	out.heights.assign(h, std::vector<double>(w, 0.0));
	if (!w || !h || !in.width || !in.height)
		return out;
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x) {
			double fx = (w == 1 ? 0.0 : double(x) * (in.width - 1) / double(w - 1)),
				   fy = (h == 1 ? 0.0 : double(y) * (in.height - 1) / double(h - 1));
			auto ix = std::min(in.width - 1, std::size_t(fx)),
				 iy = std::min(in.height - 1, std::size_t(fy));
			out.heights[y][x] = in.heights[iy][ix];
		}
	return out;
}
ElevationData smooth_grid(const ElevationData &in, double sigma)
{
	ElevationData out = in;
	if (sigma <= 0.0 || in.heights.empty())
		return out;
	out.heights = gaussian_blur_grid(in.heights, sigma);
	return out;
}
std::vector<std::vector<double>> gradient_grid(const ElevationData &g)
{
	std::vector<std::vector<double>> out(g.height, std::vector<double>(g.width));
	if (!g.width || !g.height)
		return out;
	for (std::size_t y = 0; y < g.height; ++y)
		for (std::size_t x = 0; x < g.width; ++x) {
			auto at = [&](std::size_t xx, std::size_t yy) { return g.heights[yy][xx]; };
			std::size_t xl = x ? x - 1 : x, xr = x + 1 < g.width ? x + 1 : x,
						yt = y ? y - 1 : y, yb = y + 1 < g.height ? y + 1 : y;
			out[y][x] = std::hypot(
					(at(xr, y) - at(xl, y)) * 0.5, (at(x, yb) - at(x, yt)) * 0.5);
		}
	return out;
}
std::pair<double, double> elevation_range(const ElevationData &g)
{
	if (g.heights.empty())
		return {0.0, 0.0};
	double lo = std::numeric_limits<double>::infinity(), hi = -lo;
	for (const auto &r : g.heights)
		for (double v : r) {
			lo = std::min(lo, v);
			hi = std::max(hi, v);
		}
	return {lo, hi};
}
ElevationData slope_normalized(const ElevationData &g, double maxs)
{
	ElevationData out = g;
	auto grad = gradient_grid(g);
	if (maxs <= 0)
		maxs = 1;
	for (std::size_t y = 0; y < g.height; ++y)
		for (std::size_t x = 0; x < g.width; ++x)
			out.heights[y][x] = std::clamp(grad[y][x] / maxs, 0.0, 1.0);
	return out;
}
std::vector<std::vector<TerrainClass>> classify_terrain(
		const ElevationData &g, double rolling, double steep, double cliff)
{
	std::vector<std::vector<TerrainClass>> out(
			g.height, std::vector<TerrainClass>(g.width, TerrainClass::Flat));
	auto grad = gradient_grid(g);
	for (std::size_t y = 0; y < g.height; ++y)
		for (std::size_t x = 0; x < g.width; ++x) {
			double v = grad[y][x];
			out[y][x] = v >= cliff	   ? TerrainClass::Cliff
						: v >= steep   ? TerrainClass::Steep
						: v >= rolling ? TerrainClass::Rolling
									   : TerrainClass::Flat;
		}
	return out;
}
std::vector<std::vector<std::uint8_t>> basin_mask(const ElevationData &g, double drop)
{
	std::vector<std::vector<std::uint8_t>> out(
			g.height, std::vector<std::uint8_t>(g.width));
	for (std::size_t y = 0; y < g.height; ++y)
		for (std::size_t x = 0; x < g.width; ++x) {
			double n = g.heights[y][x];
			for (int dy = -1; dy <= 1; ++dy)
				for (int dx = -1; dx <= 1; ++dx) {
					std::size_t xx = std::clamp<int>(int(x) + dx, 0, int(g.width) - 1),
								yy = std::clamp<int>(int(y) + dy, 0, int(g.height) - 1);
					n = std::min(n, g.heights[yy][xx]);
				}
			out[y][x] = g.heights[y][x] - n >= drop;
		}
	return out;
}
std::vector<std::vector<std::uint8_t>> ridge_mask(const ElevationData &g, double rise)
{
	std::vector<std::vector<std::uint8_t>> out(
			g.height, std::vector<std::uint8_t>(g.width));
	for (std::size_t y = 0; y < g.height; ++y)
		for (std::size_t x = 0; x < g.width; ++x) {
			double n = g.heights[y][x];
			for (int dy = -1; dy <= 1; ++dy)
				for (int dx = -1; dx <= 1; ++dx) {
					std::size_t xx = std::clamp<int>(int(x) + dx, 0, int(g.width) - 1),
								yy = std::clamp<int>(int(y) + dy, 0, int(g.height) - 1);
					n = std::max(n, g.heights[yy][xx]);
				}
			out[y][x] = n - g.heights[y][x] >= rise;
		}
	return out;
}
std::vector<std::pair<std::size_t, std::size_t>> local_minima(
		const ElevationData &g, double p)
{
	std::vector<std::pair<std::size_t, std::size_t>> out;
	for (std::size_t y = 1; y + 1 < g.height; ++y)
		for (std::size_t x = 1; x + 1 < g.width; ++x) {
			double v = g.heights[y][x], n = v;
			for (int dy = -1; dy <= 1; ++dy)
				for (int dx = -1; dx <= 1; ++dx)
					n = std::min(n, g.heights[y + dy][x + dx]);
			if (v == n) {
				double mx = -v;
				for (int dy = -1; dy <= 1; ++dy)
					for (int dx = -1; dx <= 1; ++dx)
						mx = std::max(mx, g.heights[y + dy][x + dx]);
				if (mx - v >= p)
					out.emplace_back(x, y);
			}
		}
	return out;
}
std::vector<std::pair<std::size_t, std::size_t>> local_maxima(
		const ElevationData &g, double p)
{
	std::vector<std::pair<std::size_t, std::size_t>> out;
	for (std::size_t y = 1; y + 1 < g.height; ++y)
		for (std::size_t x = 1; x + 1 < g.width; ++x) {
			double v = g.heights[y][x], n = v;
			for (int dy = -1; dy <= 1; ++dy)
				for (int dx = -1; dx <= 1; ++dx)
					n = std::max(n, g.heights[y + dy][x + dx]);
			if (v == n) {
				double mn = v;
				for (int dy = -1; dy <= 1; ++dy)
					for (int dx = -1; dx <= 1; ++dx)
						mn = std::min(mn, g.heights[y + dy][x + dx]);
				if (v - mn >= p)
					out.emplace_back(x, y);
			}
		}
	return out;
}
std::vector<std::vector<int>> watershed_labels(const ElevationData &g, double p)
{
	auto seeds = local_minima(g, p);
	std::vector<std::vector<int>> out(g.height, std::vector<int>(g.width, -1));
	for (std::size_t y = 0; y < g.height; ++y)
		for (std::size_t x = 0; x < g.width; ++x) {
			double best = std::numeric_limits<double>::infinity();
			int bi = -1;
			for (std::size_t i = 0; i < seeds.size(); ++i) {
				double dx = double(x) - seeds[i].first, dy = double(y) - seeds[i].second,
					   d = dx * dx + dy * dy;
				if (d < best) {
					best = d;
					bi = int(i);
				}
			}
			out[y][x] = bi;
		}
	return out;
}
std::vector<std::vector<std::pair<std::size_t, std::size_t>>> contours(
		const ElevationData &g, double step)
{
	std::vector<std::vector<std::pair<std::size_t, std::size_t>>> out;
	if (step <= 0 || g.heights.empty())
		return out;
	auto r = elevation_range(g);
	for (double level = std::ceil(r.first / step) * step; level <= r.second;
			level += step) {
		std::vector<std::pair<std::size_t, std::size_t>> line;
		for (std::size_t y = 0; y < g.height; ++y)
			for (std::size_t x = 0; x < g.width; ++x)
				if (std::abs(g.heights[y][x] - level) < step * 0.05)
					line.emplace_back(x, y);
		if (!line.empty())
			out.push_back(std::move(line));
	}
	return out;
}
std::vector<std::int16_t> encode_heightmap(
		const ElevationData &g, double scale, double off)
{
	if (scale == 0)
		scale = 1;
	std::vector<std::int16_t> out;
	out.reserve(g.width * g.height);
	for (const auto &r : g.heights)
		for (double v : r)
			out.push_back(std::int16_t(
					std::clamp(std::llround((v - off) * scale), -32768ll, 32767ll)));
	return out;
}
ElevationData decode_heightmap(const std::vector<std::int16_t> &v, std::size_t w,
		std::size_t h, double scale, double off)
{
	if (scale == 0)
		scale = 1;
	ElevationData g;
	g.width = w;
	g.height = h;
	g.heights.assign(h, std::vector<double>(w));
	for (std::size_t i = 0; i < std::min(v.size(), w * h); ++i)
		g.heights[i / w][i % w] = double(v[i]) / scale + off;
	return g;
}
std::optional<Tile> CachedProvider::tile_for(double lat, double lon)
{
	const int la = int(std::floor(lat)), lo = int(std::floor(lon));
	char ns = la >= 0 ? 'N' : 'S', ew = lo >= 0 ? 'E' : 'W';
	auto p = root_ / (std::string(1, ns) + std::to_string(std::abs(la)) +
							 std::string(1, ew) + std::to_string(std::abs(lo)) + ".hgt");
	if (!std::filesystem::is_regular_file(p)) {
		if (!providers::fetch_with_fallback(la, lo, p))
			return std::nullopt;
	}
	return Tile{la, lo, p};
}
providers::Source CachedProvider::source_for(double lat, double lon) const
{
	const int la = int(std::floor(lat)), lo = int(std::floor(lon));
	char ns = la >= 0 ? 'N' : 'S', ew = lo >= 0 ? 'E' : 'W';
	auto p = root_ / (std::string(1, ns) + std::to_string(std::abs(la)) +
							 std::string(1, ew) + std::to_string(std::abs(lo)) + ".hgt");
	return providers::select_source(la, lo, p);
}
std::optional<TileResult> CachedProvider::fetch_tile(double lat, double lon)
{
	auto t = tile_for(lat, lon);
	if (!t)
		return std::nullopt;
	return TileResult{*t, source_for(lat, lon)};
}
std::vector<Tile> tiles_for_bbox(
		double a, double b, double c, double d, const std::filesystem::path &root)
{
	std::vector<Tile> out;
	for (int la = int(std::floor(a)); la <= int(std::floor(c)); ++la)
		for (int lo = int(std::floor(b)); lo <= int(std::floor(d)); ++lo) {
			CachedProvider p(root);
			if (auto t = p.tile_for(la + 0.5, lo + 0.5))
				out.push_back(*t);
		}
	return out;
}
}
