#include "providers.h"
#include "../../../http.h"
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <optional>
#include <fstream>
#include <cmath>
#include <limits>
namespace arnis::elevation::providers
{
static unsigned g_retries = 2;
static bool g_aws = true, g_mapterhorn = true, g_usgs = true;
static ProviderStats g_stats;
static std::optional<double> g_fixed;
static bool g_fixed_enabled = true;
bool bboxes_overlap(const GeoBBox &a, const GeoBBox &b)
{
	return a.min_lat <= b.max_lat && a.max_lat >= b.min_lat && a.min_lon <= b.max_lon &&
		   a.max_lon >= b.min_lon;
}
bool usgs_3dep_covers(const GeoBBox &bbox)
{
	static constexpr GeoBBox coverage[] = {{24, -125, 50, -66}, {51, -180, 72, -129},
			{18.5, -161, 22.5, -154}, {17.5, -68, 18.7, -64}, {13.2, 144.5, 13.7, 145}};
	return std::any_of(std::begin(coverage), std::end(coverage),
			[&](const GeoBBox &area) { return bboxes_overlap(area, bbox); });
}
std::vector<Source> select_sources(const GeoBBox &bbox, SourceMode mode)
{
	if (mode == SourceMode::AwsOnly)
		return {Source::AWS};
	if (mode == SourceMode::GlobalOnly)
		return {Source::Mapterhorn, Source::AWS};
	std::vector<Source> out;
	if (g_usgs && usgs_3dep_covers(bbox))
		out.push_back(Source::USGS);
	if (g_mapterhorn)
		out.push_back(Source::Mapterhorn);
	if (g_aws)
		out.push_back(Source::AWS);
	if (out.empty() && g_fixed && g_fixed_enabled)
		out.push_back(Source::Fixed);
	return out;
}
double lon_to_mercator_x(double lon)
{
	return lon * 3.14159265358979323846 / 180. * MERCATOR_RADIUS_M;
}
double lat_to_mercator_y(double lat)
{
	const auto clamped = std::clamp(lat, -MERCATOR_LAT_LIMIT, MERCATOR_LAT_LIMIT);
	const auto r = clamped * 3.14159265358979323846 / 180.;
	return MERCATOR_RADIUS_M * std::log(std::tan(3.14159265358979323846 / 4. + r / 2.));
}
double mercator_x_to_lon(double x)
{
	return x / MERCATOR_RADIUS_M * 180. / 3.14159265358979323846;
}
double mercator_y_to_lat(double y)
{
	return (2. * std::atan(std::exp(y / MERCATOR_RADIUS_M)) -
				   3.14159265358979323846 / 2.) *
		   180. / 3.14159265358979323846;
}
double FixedTileKey::span_m() const
{
	return 512. * std::max(1, level_m);
}
double meters_per_pixel(UsgsResolution r)
{
	switch (r) {
	case UsgsResolution::M1:
		return 1.;
	case UsgsResolution::M3:
		return 3.4359738368;
	case UsgsResolution::M10:
		return 10.3079215104;
	case UsgsResolution::M30:
		return 30.9220809814;
	}
	return 30.9220809814;
}
const char *resolution_id(UsgsResolution r)
{
	switch (r) {
	case UsgsResolution::M1:
		return "r1";
	case UsgsResolution::M3:
		return "r3";
	case UsgsResolution::M10:
		return "r10";
	case UsgsResolution::M30:
		return "r30";
	}
	return "r30";
}
double UsgsFixedTileKey::span_m() const
{
	return 512. * meters_per_pixel(level);
}
double UsgsFixedTileKey::min_mx() const
{
	return -MERCATOR_LIMIT + tile_x * span_m();
}
double UsgsFixedTileKey::max_mx() const
{
	return min_mx() + span_m();
}
double UsgsFixedTileKey::max_my() const
{
	return MERCATOR_LIMIT - tile_y * span_m();
}
double UsgsFixedTileKey::min_my() const
{
	return max_my() - span_m();
}
std::filesystem::path UsgsFixedTileKey::cache_path(
		const std::filesystem::path &root) const
{
	return root / resolution_id(level) / std::to_string(tile_y) /
		   (std::to_string(tile_x) + ".tiff");
}
double FixedTileKey::min_mx() const
{
	return -MERCATOR_LIMIT + tile_x * span_m();
}
double FixedTileKey::max_mx() const
{
	return min_mx() + span_m();
}
double FixedTileKey::max_my() const
{
	return MERCATOR_LIMIT - tile_y * span_m();
}
double FixedTileKey::min_my() const
{
	return max_my() - span_m();
}
std::filesystem::path FixedTileKey::cache_path(const std::filesystem::path &root) const
{
	return root / ("r" + std::to_string(level_m)) / std::to_string(tile_y) /
		   (std::to_string(tile_x) + ".tiff");
}
FixedTileKey fixed_tile_for_mercator(int level, double mx, double my)
{
	level = std::max(1, level);
	const double span = 512. * level;
	const int maximum = std::max(0, int(std::ceil(2. * MERCATOR_LIMIT / span)) - 1);
	return {level, std::clamp(int(std::floor((mx + MERCATOR_LIMIT) / span)), 0, maximum),
			std::clamp(int(std::floor((MERCATOR_LIMIT - my) / span)), 0, maximum)};
}
FixedTileKey fixed_tile_for_mercator(UsgsResolution level, double mx, double my)
{
	// FixedTileKey deliberately carries an integer because it is also used by
	// generic clients.  Its level_m stores a rounded cache-compatible m/px;
	// the USGS request plan itself keeps the precise enum through its URL/key.
	return fixed_tile_for_mercator(int(std::lround(meters_per_pixel(level))), mx, my);
}
UsgsFixedTileKey usgs_tile_for_mercator(UsgsResolution level, double mx, double my)
{
	const double span = 512. * meters_per_pixel(level);
	const int maximum = std::max(0, int(std::ceil(2. * MERCATOR_LIMIT / span)) - 1);
	return {level, std::clamp(int(std::floor((mx + MERCATOR_LIMIT) / span)), 0, maximum),
			std::clamp(int(std::floor((MERCATOR_LIMIT - my) / span)), 0, maximum)};
}
std::vector<FixedTileKey> covering_fixed_tiles(const GeoBBox &bbox, int level)
{
	level = std::max(1, level);
	const auto swx = lon_to_mercator_x(bbox.min_lon),
			   ney = lat_to_mercator_y(bbox.max_lat),
			   nex = lon_to_mercator_x(bbox.max_lon),
			   swy = lat_to_mercator_y(bbox.min_lat);
	const auto first = fixed_tile_for_mercator(level, swx, ney),
			   last = fixed_tile_for_mercator(level, nex, swy);
	std::vector<FixedTileKey> out;
	out.reserve(std::size_t(last.tile_x - first.tile_x + 1) *
				std::size_t(last.tile_y - first.tile_y + 1));
	for (int y = first.tile_y; y <= last.tile_y; ++y)
		for (int x = first.tile_x; x <= last.tile_x; ++x)
			out.push_back({level, x, y});
	return out;
}
std::vector<FixedTileKey> covering_fixed_tiles(const GeoBBox &bbox, UsgsResolution level)
{
	return covering_fixed_tiles(bbox, int(std::lround(meters_per_pixel(level))));
}
std::vector<UsgsFixedTileKey> covering_usgs_tiles(
		const GeoBBox &bbox, UsgsResolution level)
{
	const auto first = usgs_tile_for_mercator(
			level, lon_to_mercator_x(bbox.min_lon), lat_to_mercator_y(bbox.max_lat));
	const auto last = usgs_tile_for_mercator(
			level, lon_to_mercator_x(bbox.max_lon), lat_to_mercator_y(bbox.min_lat));
	std::vector<UsgsFixedTileKey> out;
	out.reserve(std::size_t(last.tile_x - first.tile_x + 1) *
				std::size_t(last.tile_y - first.tile_y + 1));
	for (int y = first.tile_y; y <= last.tile_y; ++y)
		for (int x = first.tile_x; x <= last.tile_x; ++x)
			out.push_back({level, x, y});
	return out;
}
UsgsResolution choose_usgs_resolution(
		const GeoBBox &bbox, std::size_t width, std::size_t height)
{
	const double mid = (bbox.min_lat + bbox.max_lat) * .5 * 3.14159265358979323846 / 180.;
	const double w = std::abs(bbox.max_lon - bbox.min_lon) * 3.14159265358979323846 /
					 180. * MERCATOR_RADIUS_M * std::max(1e-6, std::abs(std::cos(mid)));
	const double h = std::abs(bbox.max_lat - bbox.min_lat) * 3.14159265358979323846 /
					 180. * MERCATOR_RADIUS_M;
	const double cell = std::min(w / double(std::max<std::size_t>(1, width - 1)),
			h / double(std::max<std::size_t>(1, height - 1)));
	for (const auto r : {UsgsResolution::M1, UsgsResolution::M3, UsgsResolution::M10,
				 UsgsResolution::M30})
		if (!std::isfinite(cell) || cell <= 0. || meters_per_pixel(r) * 1.5 >= cell)
			return r;
	return UsgsResolution::M30;
}
std::string usgs_3dep_tile_url(const FixedTileKey &key)
{
	std::ostringstream s;
	s << "https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer/exportImage"
	  << "?bbox=" << std::fixed << std::setprecision(6) << key.min_mx() << ','
	  << key.min_my() << ',' << key.max_mx() << ',' << key.max_my()
	  << "&bboxSR=3857&imageSR=3857&size=512,512&format=tiff&pixelType=F32"
	  << "&interpolation=RSP_BilinearInterpolation&f=image";
	return s.str();
}
std::string usgs_3dep_tile_url(const UsgsFixedTileKey &key)
{
	std::ostringstream s;
	s << "https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer/exportImage"
	  << "?bbox=" << std::fixed << std::setprecision(6) << key.min_mx() << ','
	  << key.min_my() << ',' << key.max_mx() << ',' << key.max_my()
	  << "&bboxSR=3857&imageSR=3857&size=512,512&format=tiff&pixelType=F32"
	  << "&interpolation=RSP_BilinearInterpolation&f=image";
	return s.str();
}
XyzTileKey xyz_tile_at(double lat, double lon, unsigned zoom)
{
	const auto n = std::ldexp(1.0, int(zoom));
	const auto maximum = std::max(0, int(n) - 1);
	const auto clamped = std::clamp(lat, -MERCATOR_LAT_LIMIT, MERCATOR_LAT_LIMIT);
	const auto r = clamped * 3.14159265358979323846 / 180.;
	const int x = std::clamp(int(std::floor((lon + 180.) / 360. * n)), 0, maximum);
	const int y = std::clamp(
			int(std::floor(
					(1. - std::asinh(std::tan(r)) / 3.14159265358979323846) / 2. * n)),
			0, maximum);
	return {zoom, unsigned(x), unsigned(y)};
}
std::vector<XyzTileKey> covering_xyz_tiles(const GeoBBox &bbox, unsigned zoom)
{
	const auto a = xyz_tile_at(bbox.min_lat, bbox.min_lon, zoom),
			   b = xyz_tile_at(bbox.max_lat, bbox.max_lon, zoom);
	const auto x0 = std::min(a.x, b.x), x1 = std::max(a.x, b.x), y0 = std::min(a.y, b.y),
			   y1 = std::max(a.y, b.y);
	std::vector<XyzTileKey> out;
	out.reserve(std::size_t(x1 - x0 + 1) * std::size_t(y1 - y0 + 1));
	for (unsigned x = x0; x <= x1; ++x)
		for (unsigned y = y0; y <= y1; ++y)
			out.push_back({zoom, x, y});
	return out;
}
std::optional<XyzTileKey> xyz_parent(XyzTileKey key)
{
	if (!key.zoom)
		return std::nullopt;
	return XyzTileKey{key.zoom - 1, key.x / 2, key.y / 2};
}
std::string aws_terrarium_tile_url(const XyzTileKey &key)
{
	return "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/" +
		   std::to_string(key.zoom) + "/" + std::to_string(key.x) + "/" +
		   std::to_string(key.y) + ".png";
}
std::string mapterhorn_tile_url(const XyzTileKey &key)
{
	return "https://tiles.mapterhorn.com/" + std::to_string(key.zoom) + "/" +
		   std::to_string(key.x) + "/" + std::to_string(key.y) + ".webp";
}
unsigned choose_mapterhorn_zoom(const GeoBBox &bbox, std::size_t width,
		std::size_t height, unsigned min_zoom, unsigned max_zoom, std::size_t budget)
{
	const double lat = (bbox.min_lat + bbox.max_lat) * .5 * 3.14159265358979323846 / 180.;
	const double metres_x =
			std::abs(lon_to_mercator_x(bbox.max_lon) - lon_to_mercator_x(bbox.min_lon)) *
			std::max(.000001, std::cos(lat));
	const double metres_y =
			std::abs(lat_to_mercator_y(bbox.max_lat) - lat_to_mercator_y(bbox.min_lat)) *
			std::max(.000001, std::cos(lat));
	const double cell = std::max(
			.05, std::min(metres_x / double(std::max<std::size_t>(1, width - 1)),
						 metres_y / double(std::max<std::size_t>(1, height - 1))));
	const double need = 40075016.686 * std::max(.000001, std::abs(std::cos(lat))) /
						(512. * cell * 1.2);
	unsigned zoom = need <= 1 ? 0
							  : unsigned(std::clamp<int>(int(std::ceil(std::log2(need))),
										0, int(max_zoom)));
	while (zoom > min_zoom && covering_xyz_tiles(bbox, zoom).size() > budget)
		--zoom;
	return std::clamp(zoom, min_zoom, max_zoom);
}
std::optional<double> terrarium_height(std::array<std::uint8_t, 3> rgb)
{
	return double(rgb[0]) * 256. + double(rgb[1]) + double(rgb[2]) / 256. - 32768.;
}
std::optional<double> sample_terrarium_pixel(const RgbRaster &tile, int x, int y)
{
	if (x < 0 || y < 0 || std::size_t(x) >= tile.width || std::size_t(y) >= tile.height ||
			tile.pixels.size() < tile.width * tile.height)
		return std::nullopt;
	return terrarium_height(tile.pixels[std::size_t(y) * tile.width + std::size_t(x)]);
}
std::optional<double> sample_terrarium_bilinear(const RgbRaster &tile, double x, double y)
{
	if (tile.width < 2 || tile.height < 2 || !std::isfinite(x) || !std::isfinite(y))
		return std::nullopt;
	x = std::clamp(x, 0., double(tile.width - 1));
	y = std::clamp(y, 0., double(tile.height - 1));
	const int x0 = std::min<int>(tile.width - 2, int(std::floor(x))),
			  y0 = std::min<int>(tile.height - 2, int(std::floor(y)));
	const auto a = sample_terrarium_pixel(tile, x0, y0),
			   b = sample_terrarium_pixel(tile, x0 + 1, y0),
			   c = sample_terrarium_pixel(tile, x0, y0 + 1),
			   d = sample_terrarium_pixel(tile, x0 + 1, y0 + 1);
	if (!a || !b || !c || !d)
		return std::nullopt;
	const double dx = x - x0, dy = y - y0;
	return (1 - dy) * ((1 - dx) * *a + dx * *b) + dy * ((1 - dx) * *c + dx * *d);
}
double blend_finite_samples(
		double v00, double v10, double v01, double v11, double dx, double dy)
{
	const std::array<double, 4> values{{v00, v10, v01, v11}};
	const std::array<double, 4> weights{
			{(1. - dx) * (1. - dy), dx * (1. - dy), (1. - dx) * dy, dx * dy}};
	double sum = 0., weight = 0.;
	for (std::size_t i = 0; i < values.size(); ++i)
		if (std::isfinite(values[i])) {
			sum += values[i] * weights[i];
			weight += weights[i];
		}
	return weight > 0. ? sum / weight : std::numeric_limits<double>::quiet_NaN();
}
std::optional<double> sample_fixed_tile_bilinear(
		const std::vector<std::vector<double>> &tile, const FixedTileKey &key, double mx,
		double my)
{
	if (tile.empty() || tile.front().empty())
		return std::nullopt;
	const auto height = tile.size(), width = tile.front().size();
	if (width < 2 || height < 2 || !std::isfinite(mx) || !std::isfinite(my))
		return std::nullopt;
	const double x = std::clamp(
			(mx - key.min_mx()) / std::max(1, key.level_m), 0., double(width - 1));
	const double y = std::clamp(
			(key.max_my() - my) / std::max(1, key.level_m), 0., double(height - 1));
	const auto x0 = std::min(width - 2, std::size_t(std::floor(x)));
	const auto y0 = std::min(height - 2, std::size_t(std::floor(y)));
	const double result = blend_finite_samples(tile[y0][x0], tile[y0][x0 + 1],
			tile[y0 + 1][x0], tile[y0 + 1][x0 + 1], x - x0, y - y0);
	return std::isfinite(result) ? std::optional<double>(result) : std::nullopt;
}
std::optional<double> sample_usgs_tile_bilinear(
		const std::vector<std::vector<double>> &tile, const UsgsFixedTileKey &key,
		double mx, double my)
{
	if (tile.empty() || tile.front().empty())
		return std::nullopt;
	const auto height = tile.size(), width = tile.front().size();
	if (width < 2 || height < 2)
		return std::nullopt;
	const double mpp = meters_per_pixel(key.level);
	const double x = std::clamp((mx - key.min_mx()) / mpp, 0., double(width - 1));
	const double y = std::clamp((key.max_my() - my) / mpp, 0., double(height - 1));
	const auto x0 = std::min(width - 2, std::size_t(std::floor(x)));
	const auto y0 = std::min(height - 2, std::size_t(std::floor(y)));
	const double result = blend_finite_samples(tile[y0][x0], tile[y0][x0 + 1],
			tile[y0 + 1][x0], tile[y0 + 1][x0 + 1], x - x0, y - y0);
	return std::isfinite(result) ? std::optional<double>(result) : std::nullopt;
}
std::vector<std::vector<double>> resample_raster_nearest(
		const std::vector<double> &source, std::size_t source_width,
		std::size_t target_width, std::size_t target_height, double nodata)
{
	std::vector<std::vector<double>> out(target_height,
			std::vector<double>(target_width, std::numeric_limits<double>::quiet_NaN()));
	if (!source_width || source.empty() || !target_width || !target_height)
		return out;
	const auto source_height = source.size() / source_width;
	if (!source_height)
		return out;
	for (std::size_t y = 0; y < target_height; ++y) {
		const auto sy = std::min(source_height - 1,
				std::size_t(double(y) / std::max<std::size_t>(1, target_height - 1) *
							(source_height - 1)));
		for (std::size_t x = 0; x < target_width; ++x) {
			const auto sx = std::min(source_width - 1,
					std::size_t(double(x) / std::max<std::size_t>(1, target_width - 1) *
								(source_width - 1)));
			const auto value = source[sy * source_width + sx];
			if (std::isfinite(value) && value > nodata && value < 100000.)
				out[y][x] = value;
		}
	}
	return out;
}
void set_download_retries(unsigned r)
{
	g_retries = std::min(8u, r);
}
void enable_aws(bool v)
{
	g_aws = v;
}
void enable_mapterhorn(bool v)
{
	g_mapterhorn = v;
}
void enable_usgs(bool v)
{
	g_usgs = v;
}
ProviderConfig config()
{
	return {g_aws, g_usgs, g_mapterhorn, g_retries};
}
Source select_source(int lat, int lon, const std::filesystem::path &file)
{
	if (std::filesystem::exists(file))
		return Source::AWS;
	const auto sources =
			select_sources({double(lat), double(lon), double(lat + 1), double(lon + 1)});
	return sources.empty() ? Source::None : sources.front();
}
const char *source_name(Source s)
{
	switch (s) {
	case Source::AWS:
		return "aws";
	case Source::USGS:
		return "usgs";
	case Source::Mapterhorn:
		return "mapterhorn";
	case Source::Fixed:
		return "fixed";
	default:
		return "none";
	}
}
ProviderStats stats()
{
	return g_stats;
}
void reset_stats()
{
	g_stats = {};
}
void set_fixed_height(std::optional<double> h)
{
	g_fixed = h;
}
void enable_fixed(bool v)
{
	g_fixed_enabled = v;
}
void reset_config()
{
	g_retries = 2;
	g_aws = g_usgs = g_mapterhorn = g_fixed_enabled = true;
	g_fixed.reset();
}
std::string aws_terrain_url(int lat, int lon)
{
	std::ostringstream s;
	s << "https://s3.amazonaws.com/elevation-tiles-prod/skadi/" << (lat >= 0 ? 'N' : 'S')
	  << std::setw(2) << std::setfill('0') << std::abs(lat) << "/"
	  << (lat >= 0 ? 'N' : 'S') << std::setw(2) << std::setfill('0') << std::abs(lat)
	  << (lon >= 0 ? 'E' : 'W') << std::setw(3) << std::setfill('0') << std::abs(lon)
	  << ".hgt.gz";
	return s.str();
}
std::string usgs_3dep_url(double lat, double lon)
{
	return "https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer/exportImage?f=image&bbox=" +
		   std::to_string(lon) + "," + std::to_string(lat) + "," +
		   std::to_string(lon + 1) + "," + std::to_string(lat + 1);
}
std::string mapterhorn_url(double lat, double lon)
{
	return "https://mapterhorn.com/tiles/" + std::to_string(int(lat)) + "/" +
		   std::to_string(int(lon)) + ".hgt";
}
bool download_tile(const std::string &url, const std::filesystem::path &file)
{
	std::filesystem::create_directories(file.parent_path());
	auto tmp = file;
	tmp += ".tmp";
	std::error_code ec;
	std::filesystem::remove(tmp, ec);
	if (!http_to_file(url, tmp.string()))
		return false;
	auto n = std::filesystem::file_size(tmp, ec);
	if (ec || n < 1024) {
		std::filesystem::remove(tmp);
		return false;
	}
	std::filesystem::rename(tmp, file, ec);
	if (ec) {
		std::filesystem::remove(tmp);
		return false;
	}
	return true;
}
bool fetch_with_fallback(int lat, int lon, const std::filesystem::path &file)
{
	if (std::filesystem::exists(file))
		return true;
	if (g_fixed && g_fixed_enabled) {
		std::filesystem::create_directories(file.parent_path());
		std::ofstream f(file, std::ios::binary);
		if (f) {
			std::vector<std::int16_t> v(3601 * 3601, std::int16_t(*g_fixed));
			for (auto x : v) {
				unsigned char b[2] = {
						static_cast<unsigned char>((std::uint16_t(x) >> 8) & 255),
						static_cast<unsigned char>(std::uint16_t(x) & 255)};
				f.write(reinterpret_cast<char *>(b), 2);
			}
			return bool(f);
		}
	}
	std::vector<std::pair<std::string, int>> urls;
	if (g_aws)
		urls.push_back({aws_terrain_url(lat, lon), 0});
	if (g_usgs)
		urls.push_back({usgs_3dep_url(lat, lon), 1});
	if (g_mapterhorn)
		urls.push_back({mapterhorn_url(lat, lon), 2});
	for (const auto &[u, kind] : urls)
		for (unsigned i = 0; i <= g_retries; ++i) {
			++g_stats.attempts;
			if (download_tile(u, file)) {
				++g_stats.successes;
				if (kind == 0)
					++g_stats.aws_successes;
				else if (kind == 1)
					++g_stats.usgs_successes;
				else
					++g_stats.mapterhorn_successes;
				return true;
			}
		}
	return false;
}
}
