#include "land_cover.h"
#include "../grid_ops.h"
#include "bridge_repair.h"
#include "cog.h"
#include "shoreline.h"
#include "../../http.h"
#include <filesystem>
#include "../../arnis_adapter.h"
#include "../element_processing/bridges.h"
#include "../element_processing/way_segments.h"
#include "../element_processing/waterways.h"
#include "../bresenham.h"
#include "../clipping.h"
#include <queue>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <deque>
#include <array>
#include <limits>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <utility>
#include <unordered_map>

namespace arnis::highways
{
int highway_block_range(const std::string &highway_type,
		const std::unordered_map<std::string, std::string> &tags, double scale);
}

namespace arnis::land_cover
{
namespace
{
constexpr int BRIDGE_STAMP_MARGIN_CELLS = 8;

int scaled_block_range(int raw, double scale)
{
	return scale < 1.0 ? std::max(1, static_cast<int>(std::floor(raw * scale))) : raw;
}

int bridge_block_range_world(const ProcessedWay &way, double scale)
{
	const auto highway = way.tags.get("highway");
	if (!highway.empty())
		return highways::highway_block_range(highway, way.tags, scale);
	if (way.tags.find("railway") != way.tags.end()) {
		int tracks = 1;
		const auto tracks_tag = way.tags.get("tracks");
		if (!tracks_tag.empty()) {
			int parsed_tracks = 0;
			const auto *begin = tracks_tag.data();
			const auto *end = begin + tracks_tag.size();
			const auto parsed = std::from_chars(begin, end, parsed_tracks);
			if (parsed.ec == std::errc{} && parsed.ptr == end)
				tracks = std::max(1, parsed_tracks);
		}
		return scaled_block_range(1 + tracks - 1, scale);
	}
	return scaled_block_range(1, scale);
}

int world_to_grid(int relative, double scale)
{
	return static_cast<int>(std::lround(std::max(relative, 0) * scale));
}

double cells_per_meter(const GeographicBounds &bbox, std::size_t grid_width)
{
	constexpr double earth_radius_m = 6371000.0;
	constexpr double pi = 3.14159265358979323846;
	const double latitude = (bbox.min_lat + bbox.max_lat) * 0.5 * pi / 180.0;
	const double delta = (bbox.max_lng - bbox.min_lng) * pi / 180.0;
	const double sine = std::cos(latitude) * std::sin(delta * 0.5);
	const double q = std::clamp(sine * sine, 0.0, 1.0);
	const double width_m =
			earth_radius_m * 2.0 * std::atan2(std::sqrt(q), std::sqrt(1.0 - q));
	return width_m > 0.0 && grid_width > 0 ? grid_width / width_m : 1.0;
}
}

std::string esa_tile_url(int lat, int lng)
{
	char ns = 'N', ew = 'E';
	int la = lat, lo = lng;
	if (la < 0) {
		ns = 'S';
		la = -la;
	}
	if (lo < 0) {
		ew = 'W';
		lo = -lo;
	}
	std::ostringstream name;
	name << "https://esa-worldcover.s3.eu-central-1.amazonaws.com/v200/2021/map/ESA_WorldCover_10m_2021_v200_"
		 << ns << std::setw(2) << std::setfill('0') << la << ew << std::setw(3)
		 << std::setfill('0') << lo << "_Map.tif";
	return name.str();
}
std::vector<std::tuple<int, int, std::string>> esa_tiles_for_bbox(
		double a, double b, double c, double d)
{
	std::vector<std::tuple<int, int, std::string>> o;
	if (c < -60 || a > 84)
		return o;
	a = std::max(a, -60.0);
	// The last valid south-west tile is 81N/177E.  At the exclusive dataset
	// boundary, floor(84 / 3) would otherwise manufacture an invalid N84 tile.
	c = std::min(c, 84.0 - 0.001);
	b = std::max(b, -180.0);
	d = std::min(d, 180.0 - 0.001);
	int y0 = int(std::floor(a / 3.0)) * 3, y1 = int(std::floor(c / 3.0)) * 3;
	int x0 = int(std::floor(b / 3.0)) * 3, x1 = int(std::floor(d / 3.0)) * 3;
	for (int y = y0; y <= y1; y += 3)
		for (int x = x0; x <= x1; x += 3)
			o.emplace_back(y, x, esa_tile_url(y, x));
	return o;
}

LandCoverData fetch_land_cover_data(const GeographicBounds &bbox, std::size_t width,
		std::size_t height, bool smooth_boundaries)
{
	if (!bbox.valid() || width == 0 || height == 0)
		return {};
	const auto fetch = [](const std::string &url, std::uint64_t offset,
							   std::uint64_t length) {
		const auto bytes = http_get_range(url, offset, length);
		return std::vector<std::uint8_t>(bytes.begin(), bytes.end());
	};
	std::vector<EsaRasterTile> tiles;
	for (const auto &[lat, lng, url] :
			esa_tiles_for_bbox(bbox.min_lat, bbox.min_lng, bbox.max_lat, bbox.max_lng)) {
		EsaRasterTile tile;
		if (read_esa_cog_into_raster(url, lat, lng, bbox.min_lat, bbox.min_lng,
					bbox.max_lat, bbox.max_lng, tile, fetch))
			tiles.push_back(std::move(tile));
	}
	return assemble_land_cover_data(bbox, width, height, tiles, smooth_boundaries);
}
bool fetch_esa_tile(const std::string &url, const std::filesystem::path &file)
{
	if (std::filesystem::exists(file) && std::filesystem::file_size(file) > 0)
		return true;
	return http_to_file(url, file.string()) > 0;
}

uint8_t EsaRasterTile::sample(double lat, double lng) const
{
	if (!valid() || !contains(lat, lng))
		return 0;
	// ESA COG scanlines run north to south. Clamp the north/east edge to the
	// final pixel: neighbouring 3-degree tiles share that geometric boundary.
	const double south = min_lat != max_lat ? min_lat : south_lat;
	const double north = min_lat != max_lat ? max_lat : south_lat + 3.0;
	const double west = min_lng != max_lng ? min_lng : west_lng;
	const double east = min_lng != max_lng ? max_lng : west_lng + 3.0;
	const double u =
			east != west ? std::clamp((lng - west) / (east - west), 0.0, 1.0) : 0.0;
	const double v =
			north != south ? std::clamp((north - lat) / (north - south), 0.0, 1.0) : 0.0;
	const auto x =
			std::min(width - 1, static_cast<std::size_t>(std::floor(u * double(width))));
	const auto z = std::min(
			height - 1, static_cast<std::size_t>(std::floor(v * double(height))));
	return pixels[z * width + x];
}

void fill_land_cover_gaps(
		std::vector<std::vector<uint8_t>> &grid, std::size_t width, std::size_t height)
{
	width = std::min(width, grid.empty() ? std::size_t{0} : grid.front().size());
	height = std::min(height, grid.size());
	for (int pass = 0; pass < 10; ++pass) {
		const auto snapshot = grid;
		bool changed = false;
		for (std::size_t z = 0; z < height; ++z)
			for (std::size_t x = 0; x < width; ++x) {
				if (snapshot[z][x] != 0)
					continue;
				for (const auto &[dx, dz] : std::array<std::pair<int, int>, 4>{
							 {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}}) {
					const int nx = int(x) + dx, nz = int(z) + dz;
					if (nx >= 0 && nz >= 0 && nx < int(width) && nz < int(height) &&
							snapshot[std::size_t(nz)][std::size_t(nx)] != 0) {
						grid[z][x] = snapshot[std::size_t(nz)][std::size_t(nx)];
						changed = true;
						break;
					}
				}
			}
		if (!changed)
			break;
	}
}

void smooth_land_cover_boundaries(std::vector<std::vector<uint8_t>> &grid,
		std::size_t width, std::size_t height, double cells_per_meter_value)
{
	width = std::min(width, grid.empty() ? std::size_t{0} : grid.front().size());
	height = std::min(height, grid.size());
	if (width == 0 || height == 0)
		return;
	const double sigma = std::max(2.0, 2.0 * cells_per_meter_value);
	const int radius = static_cast<int>(std::ceil(sigma * 3.0));
	const int side = radius * 2 + 1;
	std::vector<double> kernel(static_cast<std::size_t>(side * side));
	for (int z = -radius; z <= radius; ++z)
		for (int x = -radius; x <= radius; ++x)
			kernel[std::size_t(z + radius) * side + std::size_t(x + radius)] =
					std::exp(-(x * x + z * z) / (2.0 * sigma * sigma));
	const auto snapshot = grid;
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t x = 0; x < width; ++x) {
			const auto center = snapshot[z][x];
			if (center == 0)
				continue;
			bool boundary = false;
			for (const auto &[dx, dz] : std::array<std::pair<int, int>, 4>{
						 {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}}) {
				const int nx = int(x) + dx, nz = int(z) + dz;
				if (nx >= 0 && nz >= 0 && nx < int(width) && nz < int(height) &&
						snapshot[std::size_t(nz)][std::size_t(nx)] != 0 &&
						snapshot[std::size_t(nz)][std::size_t(nx)] != center) {
					boundary = true;
					break;
				}
			}
			if (!boundary)
				continue;
			std::array<double, 256> votes{};
			for (int dz = -radius; dz <= radius; ++dz)
				for (int dx = -radius; dx <= radius; ++dx) {
					const int nx = int(x) + dx, nz = int(z) + dz;
					if (nx >= 0 && nz >= 0 && nx < int(width) && nz < int(height)) {
						const auto cls = snapshot[std::size_t(nz)][std::size_t(nx)];
						if (cls)
							votes[cls] += kernel[std::size_t(dz + radius) * side +
												 std::size_t(dx + radius)];
					}
				}
			uint8_t best = 0;
			for (unsigned cls = 1; cls < votes.size(); ++cls)
				if (votes[cls] > votes[best])
					best = static_cast<uint8_t>(cls);
			if (best)
				grid[z][x] = best;
		}
}

LandCoverData assemble_land_cover_data(const GeographicBounds &bbox, std::size_t width,
		std::size_t height, const std::vector<EsaRasterTile> &tiles,
		bool smooth_boundaries)
{
	LandCoverData out;
	if (!bbox.valid() || width == 0 || height == 0)
		return out;
	out.source_bounds = bbox;
	out.source_tiles = tiles;
	out.width = width;
	out.height = height;
	out.cells_per_meter = cells_per_meter(bbox, width);
	out.grid.assign(height, std::vector<uint8_t>(width));
	bool has_data = false;
	const double lat_span = bbox.max_lat - bbox.min_lat;
	const double lng_span = bbox.max_lng - bbox.min_lng;
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t x = 0; x < width; ++x) {
			const double lon =
					bbox.min_lng +
					(width == 1 ? 0.0 : (double(x) / double(width - 1)) * lng_span);
			const double lat =
					bbox.max_lat -
					(height == 1 ? 0.0 : (double(z) / double(height - 1)) * lat_span);
			for (const auto &tile : tiles) {
				const auto cls = tile.sample(lat, lon);
				if (cls != 0) {
					out.grid[z][x] = cls;
					has_data = true;
					break;
				}
			}
		}
	if (!has_data)
		return {};
	reconstruct_water_shoreline(out);
	fill_land_cover_gaps(out.grid, width, height);
	if (smooth_boundaries)
		smooth_land_cover_boundaries(out.grid, width, height, out.cells_per_meter);
	out.water_distance = compute_water_distance(out.grid, width, height);
	out.refresh_water_blend_grid();
	return out;
}

void apply_bridge_land_cover_repair(LandCoverData &data,
		std::vector<std::vector<float>> &heights, std::size_t world_width,
		std::size_t world_height, const std::vector<ProcessedElement> &elements,
		const XZBBox &bbox, double scale)
{
	if (data.width < 2 || data.height < 2 || world_width < 2 || world_height < 2 ||
			heights.size() < data.height ||
			std::any_of(heights.begin(), heights.begin() + data.height,
					[&](const auto &row) { return row.size() < data.width; }))
		return;
	if (std::none_of(elements.begin(), elements.end(), [](const auto &element) {
			return element.is_way() && bridges::is_bridge_way(element.as_way());
		}))
		return;
	// Rust uses a compact bit-mask and stamps only ESA built-up cells.  Keeping
	// real water/vegetation out of the mask is what lets bridge footprints be
	// reclassified from their surrounding land-cover rather than overwritten.
	std::vector<std::uint64_t> mask((data.width * data.height + 63) / 64);
	std::vector<std::uint32_t> bridge_cells;
	const double sx = double(data.width - 1) / double(world_width - 1);
	const double sz = double(data.height - 1) / double(world_height - 1);
	for (const auto &e : elements) {
		if (!e.is_way() || !bridges::is_bridge_way(e.as_way()) ||
				e.as_way().nodes.size() < 2)
			continue;
		const auto &way = e.as_way();
		const int block_range_world =
				bridge_block_range_world(way, scale) + BRIDGE_STAMP_MARGIN_CELLS;
		const int range_x = std::max(1, int(std::ceil(block_range_world * sx)));
		const int range_z = std::max(1, int(std::ceil(block_range_world * sz)));
		for (std::size_t i = 1; i < way.nodes.size(); ++i) {
			const int x0 = world_to_grid(way.nodes[i - 1].x - bbox.min_x(), sx);
			const int z0 = world_to_grid(way.nodes[i - 1].z - bbox.min_z(), sz);
			const int x1 = world_to_grid(way.nodes[i].x - bbox.min_x(), sx);
			const int z1 = world_to_grid(way.nodes[i].z - bbox.min_z(), sz);
			for (auto [x, y, z] : bresenham::bresenham_line(x0, 0, z0, x1, 0, z1))
				for (int dz = -range_z; dz <= range_z; ++dz)
					for (int dx = -range_x; dx <= range_x; ++dx) {
						int gx = x + dx, gz = z + dz;
						if (gx < 0 || gz < 0 || gx >= int(data.width) ||
								gz >= int(data.height) ||
								data.grid[gz][gx] != LC_BUILT_UP)
							continue;
						const auto index = std::size_t(gz) * data.width + gx;
						const auto bit = std::uint64_t(1) << (index & 63);
						if (!(mask[index >> 6] & bit)) {
							mask[index >> 6] |= bit;
							bridge_cells.push_back(index);
						}
					}
		}
	}
	if (bridge_cells.empty())
		return;
	auto marked = [&](std::size_t i) { return (mask[i >> 6] >> (i & 63)) & 1U; };
	constexpr std::array<std::pair<int, int>, 4> neighbours{
			{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};
	struct Assignment
	{
		std::uint32_t cell;
		std::uint8_t cls;
		float water_level;
	};
	std::unordered_map<std::uint32_t, std::uint8_t> assigned;
	std::unordered_map<std::uint32_t, float> water_levels;
	std::deque<Assignment> current, next;
	auto water_reachable = [&](std::size_t cell, float level) {
		const float h = heights[cell / data.width][cell % data.width];
		return !std::isfinite(h) || !std::isfinite(level) || h <= level + 6.f;
	};
	// Water-adjacent cells seed first, deliberately giving river crossings water
	// precedence over neighbouring built-up terrain.
	for (const auto &cell : bridge_cells) {
		const int x = cell % data.width, z = cell / data.width;
		for (const auto &[dx, dz] : neighbours) {
			const int nx = x + dx, nz = z + dz;
			if (nx < 0 || nz < 0 || nx >= int(data.width) || nz >= int(data.height))
				continue;
			const auto ni = std::size_t(nz) * data.width + nx;
			if (!marked(ni) && data.grid[nz][nx] == LC_WATER) {
				const float level = heights[nz][nx];
				if (!water_reachable(cell, level))
					continue;
				assigned[cell] = LC_WATER;
				water_levels[cell] = level;
				current.push_back({cell, LC_WATER, level});
				break;
			}
		}
	}
	for (const auto &cell : bridge_cells)
		if (!assigned.contains(cell)) {
			const int x = cell % data.width, z = cell / data.width;
			for (const auto &[dx, dz] : neighbours) {
				const int nx = x + dx, nz = z + dz;
				if (nx < 0 || nz < 0 || nx >= int(data.width) || nz >= int(data.height))
					continue;
				const auto ni = std::size_t(nz) * data.width + nx;
				const auto cls = data.grid[nz][nx];
				if (!marked(ni) && cls && cls != LC_WATER) {
					assigned[cell] = cls;
					current.push_back(
							{cell, cls, std::numeric_limits<float>::quiet_NaN()});
					break;
				}
			}
		}
	for (int ring = 1; !current.empty() && ring < 64; ++ring) {
		while (!current.empty()) {
			auto [cell, cls, level] = current.front();
			current.pop_front();
			const int x = cell % data.width, z = cell / data.width;
			for (const auto &[dx, dz] : neighbours) {
				const int nx = x + dx, nz = z + dz;
				if (nx < 0 || nz < 0 || nx >= int(data.width) || nz >= int(data.height))
					continue;
				const auto ni = std::size_t(nz) * data.width + nx;
				if (marked(ni) && !assigned.contains(ni) &&
						(cls != LC_WATER || water_reachable(ni, level))) {
					assigned[ni] = cls;
					if (cls == LC_WATER)
						water_levels[ni] = level;
					next.push_back({std::uint32_t(ni), cls, level});
				}
			}
		}
		current.swap(next);
	}
	bool water_changed = false;
	for (const auto &cell : bridge_cells)
		if (const auto it = assigned.find(cell); it != assigned.end()) {
			const int x = cell % data.width, z = cell / data.width;
			const auto old = data.grid[z][x];
			if (old != it->second) {
				data.grid[z][x] = it->second;
				if (it->second == LC_WATER)
					if (const auto level = water_levels.find(cell);
							level != water_levels.end() && std::isfinite(level->second) &&
							std::isfinite(heights[z][x]) && heights[z][x] > level->second)
						heights[z][x] = level->second;
				water_changed |= (old == LC_WATER) != (it->second == LC_WATER);
			}
		}
	if (water_changed) {
		data.water_distance = compute_water_distance(data.grid, data.width, data.height);
		data.water_blend_grid.clear();
	}
}

void apply_osm_water_override(LandCoverData &data,
		std::vector<std::vector<float>> &heights, std::size_t world_width,
		std::size_t world_height, const std::vector<ProcessedElement> &elements,
		const XZBBox &bbox)
{
	if (data.width < 2 || data.height < 2 || world_width < 2 || world_height < 2 ||
			heights.size() < data.height ||
			std::any_of(heights.begin(), heights.begin() + data.height,
					[&](const auto &row) { return row.size() < data.width; }))
		return;
	const double sx = double(data.width - 1) / double(world_width - 1);
	const double sz = double(data.height - 1) / double(world_height - 1);
	const auto guard = build_water_override_guard(data.grid, heights, sx * sz);
	const auto tag = [](const tags_t &tags, const char *name) -> const std::string * {
		auto it = tags.find(name);
		return it == tags.end() ? nullptr : &it->second;
	};
	const auto has_explicit_water = [&](const tags_t &tags) {
		const auto *water = tag(tags, "water");
		return water && *water != "no" && *water != "0" && *water != "false";
	};
	const auto is_water_polygon_way = [&](const ProcessedWay &w) {
		if (w.nodes.size() < 3)
			return false;
		const auto *natural = tag(w.tags, "natural");
		const auto *landuse = tag(w.tags, "landuse");
		const auto *waterway = tag(w.tags, "waterway");
		return (natural && *natural == "water") || has_explicit_water(w.tags) ||
			   (landuse && *landuse == "reservoir") ||
			   (waterway && (*waterway == "dock" || *waterway == "riverbank"));
	};
	const auto is_water_relation = [&](const tags_t &tags) {
		const auto *natural = tag(tags, "natural");
		return (natural && (*natural == "water" || *natural == "bay")) ||
			   has_explicit_water(tags);
	};
	const auto closed_ring = [](const std::vector<ProcessedNode> &nodes) {
		if (nodes.size() < 3)
			return false;
		const auto &first = nodes.front();
		const auto &last = nodes.back();
		return first.id == last.id ||
			   (std::abs(first.x - last.x) <= 1 && std::abs(first.z - last.z) <= 1);
	};
	const auto to_world_ring = [](const std::vector<ProcessedNode> &nodes) {
		std::vector<std::pair<int, int>> ring;
		ring.reserve(nodes.size());
		for (const auto &node : nodes)
			ring.emplace_back(node.x, node.z);
		return ring;
	};
	std::size_t changed = 0;
	const auto write_water = [&](std::size_t x, std::size_t z) {
		if (data.grid[z][x] == LC_WATER || !guard.allows(heights, x, z))
			return std::size_t{0};
		data.grid[z][x] = LC_WATER;
		return std::size_t{1};
	};
	const auto edge_crossings = [](const std::vector<std::pair<int, int>> &ring, double z,
										std::vector<double> &out) {
		if (ring.size() < 3)
			return;
		std::size_t j = ring.size() - 1;
		for (std::size_t i = 0; i < ring.size(); ++i) {
			const double zi = ring[i].second, zj = ring[j].second;
			if ((zi > z) != (zj > z)) {
				const double t = (z - zi) / (zj - zi);
				out.push_back(ring[i].first + (ring[j].first - ring[i].first) * t);
			}
			j = i;
		}
	};
	const auto inside_ranges = [](double x, const std::vector<double> &ranges) {
		for (std::size_t i = 0; i + 1 < ranges.size(); i += 2)
			if (x > ranges[i] && x < ranges[i + 1])
				return true;
		return false;
	};
	const auto fill_rings =
			[&](const std::vector<std::vector<std::pair<int, int>>> &outers,
					const std::vector<std::vector<std::pair<int, int>>> &inners) {
				if (outers.empty())
					return std::size_t{0};
				int p_min_z = INT32_MAX, p_max_z = INT32_MIN;
				for (const auto &ring : outers)
					for (const auto &[x, z] : ring) {
						(void)x;
						p_min_z = std::min(p_min_z, z);
						p_max_z = std::max(p_max_z, z);
					}
				if (p_min_z == INT32_MAX || p_max_z < bbox.min_z() ||
						p_min_z > bbox.max_z())
					return std::size_t{0};
				const double inv_x = 1.0 / sx, inv_z = 1.0 / sz;
				const int gz_start = std::max(
						0, int(std::floor(std::max(0, p_min_z - bbox.min_z()) * sz)));
				const int gz_end = std::clamp(
						int(std::ceil(std::max(0, p_max_z - bbox.min_z()) * sz)), 0,
						int(data.height) - 1);
				std::vector<double> outer_x, inner_x;
				std::size_t count = 0;
				for (int gz = gz_start; gz <= gz_end; ++gz) {
					const double world_z = gz * inv_z + bbox.min_z();
					outer_x.clear();
					for (const auto &ring : outers)
						edge_crossings(ring, world_z, outer_x);
					if (outer_x.size() < 2)
						continue;
					std::sort(outer_x.begin(), outer_x.end());
					inner_x.clear();
					for (const auto &ring : inners)
						edge_crossings(ring, world_z, inner_x);
					std::sort(inner_x.begin(), inner_x.end());
					for (std::size_t i = 0; i + 1 < outer_x.size(); i += 2) {
						const double left = std::max(outer_x[i], double(bbox.min_x()));
						const double right =
								std::min(outer_x[i + 1], double(bbox.max_x()));
						if (left > right)
							continue;
						const int gx0 =
								std::max(0, int(std::ceil((left - bbox.min_x()) * sx)));
						const int gx1 =
								std::clamp(int(std::floor((right - bbox.min_x()) * sx)),
										0, int(data.width) - 1);
						for (int gx = gx0; gx <= gx1; ++gx) {
							const double world_x = gx * inv_x + bbox.min_x();
							if (!inside_ranges(world_x, inner_x))
								count += write_water(gx, gz);
						}
					}
				}
				return count;
			};
	const auto rasterize_line = [&](const std::vector<ProcessedNode> &nodes,
										int half_width) {
		if (nodes.size() < 2 || half_width < 0)
			return std::size_t{0};
		std::size_t count = 0;
		for (std::size_t i = 1; i < nodes.size(); ++i) {
			const auto &a = nodes[i - 1];
			const auto &b = nodes[i];
			const int margin = std::max(0, half_width);
			if (std::max(a.x, b.x) + margin < bbox.min_x() ||
					std::min(a.x, b.x) - margin > bbox.max_x() ||
					std::max(a.z, b.z) + margin < bbox.min_z() ||
					std::min(a.z, b.z) - margin > bbox.max_z())
				continue;
			for (const auto &[bx, y, bz] :
					bresenham::bresenham_line(a.x, 0, a.z, b.x, 0, b.z)) {
				(void)y;
				for (int dx = -half_width; dx <= half_width; ++dx)
					for (int dz = -half_width; dz <= half_width; ++dz) {
						const int wx = bx + dx, wz = bz + dz;
						if (wx < bbox.min_x() || wz < bbox.min_z() || wx > bbox.max_x() ||
								wz > bbox.max_z())
							continue;
						const auto gx = static_cast<std::size_t>(
								std::llround((wx - bbox.min_x()) * sx));
						const auto gz = static_cast<std::size_t>(
								std::llround((wz - bbox.min_z()) * sz));
						if (gx < data.width && gz < data.height)
							count += write_water(gx, gz);
					}
			}
		}
		return count;
	};
	for (const auto &e : elements) {
		if (e.is_relation()) {
			const auto &rel = e.as_relation();
			if (!is_water_relation(rel.tags))
				continue;
			std::vector<std::vector<ProcessedNode>> outer_nodes, inner_nodes;
			for (const auto &m : rel.members) {
				if (m.way.nodes.size() < 2)
					continue;
				if (m.role == ProcessedMemberRole::Outer)
					outer_nodes.push_back(m.way.nodes);
				else if (m.role == ProcessedMemberRole::Inner)
					inner_nodes.push_back(m.way.nodes);
			}
			merge_way_segments(outer_nodes);
			merge_way_segments(inner_nodes);
			std::vector<std::vector<std::pair<int, int>>> outer, inner;
			for (const auto &ring : outer_nodes)
				if (closed_ring(ring))
					if (auto clipped = clipping::clip_water_ring_to_bbox(ring, bbox))
						outer.push_back(to_world_ring(*clipped));
			for (const auto &ring : inner_nodes)
				if (closed_ring(ring))
					if (auto clipped = clipping::clip_water_ring_to_bbox(ring, bbox))
						inner.push_back(to_world_ring(*clipped));
			changed += fill_rings(outer, inner);
			continue;
		}
		if (!e.is_way() || e.as_way().nodes.size() < 2)
			continue;
		const auto &w = e.as_way();
		const auto *waterway = tag(w.tags, "waterway");
		if (is_water_polygon_way(w)) {
			if (!closed_ring(w.nodes))
				continue;
			auto clipped = clipping::clip_water_ring_to_bbox(w.nodes, bbox);
			if (!clipped)
				continue;
			changed += fill_rings({to_world_ring(*clipped)}, {});
		} else if (waterway && waterways::is_channel_waterway(*waterway) &&
				   !waterways::is_underground_waterway(w.tags)) {
			const int half_width = waterways::waterway_width(*waterway, w.tags) / 2;
			changed += rasterize_line(w.nodes, half_width);
		}
	}
	if (changed) {
		data.water_distance = compute_water_distance(data.grid, data.width, data.height);
		data.water_blend_grid.clear();
		// OSM water may extend ESA water slightly, but should not float on a raised
		// bank. Sink only newly reachable cells that are within Rust's 1.5-block
		// elevation guard; the ESA water seeds remain at their sampled heights.
		if (guard.has_elevation)
			for (std::size_t z = 0; z < data.height; ++z)
				for (std::size_t x = 0; x < data.width; ++x) {
					if (data.grid[z][x] != LC_WATER || !std::isfinite(heights[z][x]) ||
							z >= guard.nearest_water_y.size() ||
							x >= guard.nearest_water_y[z].size())
						continue;
					const float water_y = guard.nearest_water_y[z][x];
					if (std::isfinite(water_y) && heights[z][x] > water_y &&
							heights[z][x] <= water_y + 1.5f)
						heights[z][x] = water_y;
				}
	}
}

uint64_t coord_hash(int32_t x, int32_t z)
{
	// Keep the Rust arithmetic and wrapping points verbatim: this hash drives
	// deterministic surface dithering, so a merely "good" different hash would
	// visibly diverge between implementations.
	uint64_t h = uint64_t(uint32_t(x)) * 0x9E3779B97F4A7C15ULL;
	h ^= uint64_t(uint32_t(z)) * 0x517CC1B727220A95ULL;
	h *= 0x6C62272E07BB0142ULL;
	return h ^ (h >> 32);
}

static std::vector<double> gaussian_kernel(double sigma)
{
	const int radius = std::max(1, static_cast<int>(std::ceil(sigma * 3.0)));
	std::vector<double> kernel(static_cast<std::size_t>(radius * 2 + 1));
	double sum = 0.0;
	for (int i = -radius; i <= radius; ++i) {
		const double v = std::exp(-(static_cast<double>(i * i)) / (2.0 * sigma * sigma));
		kernel[static_cast<std::size_t>(i + radius)] = v;
		sum += v;
	}
	for (double &v : kernel)
		v /= sum;
	return kernel;
}

std::vector<std::vector<float>> compute_water_blend_smooth(
		const std::vector<std::vector<uint8_t>> &grid, std::size_t width,
		std::size_t height, double cells_per_meter_value)
{
	if (width == 0 || height == 0)
		return {};
	height = std::min(height, grid.size());
	if (height == 0 || grid.front().empty())
		return {};
	width = std::min(width, grid.front().size());
	if (width == 0)
		return {};

	const double sigma = std::max(3.0, 3.0 * cells_per_meter_value);
	const auto kernel = gaussian_kernel(sigma);
	const int radius = static_cast<int>(kernel.size() / 2);

	std::vector<std::vector<double>> tmp(height, std::vector<double>(width, 0.0));
	for (std::size_t z = 0; z < height; ++z) {
		std::vector<std::uint32_t> water_prefix(width + 1, 0);
		for (std::size_t x = 0; x < width; ++x)
			water_prefix[x + 1] = water_prefix[x] + (grid[z][x] == LC_WATER ? 1u : 0u);
		for (std::size_t x = 0; x < width; ++x) {
			const auto lo = x > static_cast<std::size_t>(radius)
									? x - static_cast<std::size_t>(radius)
									: 0;
			const auto hi = std::min(width, x + static_cast<std::size_t>(radius) + 1);
			const auto hits = water_prefix[hi] - water_prefix[lo];
			if (hits == 0) {
				tmp[z][x] = 0.0;
				continue;
			}
			if (hits == hi - lo) {
				tmp[z][x] = 1.0;
				continue;
			}
			double sum = 0.0;
			double weight = 0.0;
			for (int k = -radius; k <= radius; ++k) {
				const int sx = static_cast<int>(x) + k;
				if (sx < 0 || sx >= static_cast<int>(width))
					continue;
				const double w = kernel[static_cast<std::size_t>(k + radius)];
				sum += (grid[z][static_cast<std::size_t>(sx)] == LC_WATER ? 1.0 : 0.0) *
					   w;
				weight += w;
			}
			tmp[z][x] = weight > 0.0 ? sum / weight : 0.0;
		}
	}

	std::vector<std::vector<float>> out(height, std::vector<float>(width, 0.0f));
	for (std::size_t x = 0; x < width; ++x) {
		std::vector<std::uint32_t> ones_prefix(height + 1, 0),
				zeros_prefix(height + 1, 0);
		for (std::size_t y = 0; y < height; ++y) {
			ones_prefix[y + 1] = ones_prefix[y] + (tmp[y][x] == 1.0 ? 1u : 0u);
			zeros_prefix[y + 1] = zeros_prefix[y] + (tmp[y][x] == 0.0 ? 1u : 0u);
		}
		for (std::size_t z = 0; z < height; ++z) {
			const auto lo = z > static_cast<std::size_t>(radius)
									? z - static_cast<std::size_t>(radius)
									: 0;
			const auto hi = std::min(height, z + static_cast<std::size_t>(radius) + 1);
			const auto window = static_cast<std::uint32_t>(hi - lo);
			if (ones_prefix[hi] - ones_prefix[lo] == window) {
				out[z][x] = 1.0f;
				continue;
			}
			if (zeros_prefix[hi] - zeros_prefix[lo] == window) {
				out[z][x] = 0.0f;
				continue;
			}
			double sum = 0.0;
			double weight = 0.0;
			for (int k = -radius; k <= radius; ++k) {
				const int sz = static_cast<int>(z) + k;
				if (sz < 0 || sz >= static_cast<int>(height))
					continue;
				const double w = kernel[static_cast<std::size_t>(k + radius)];
				sum += tmp[static_cast<std::size_t>(sz)][x] * w;
				weight += w;
			}
			out[z][x] = static_cast<float>(weight > 0.0 ? sum / weight : 0.0);
		}
	}
	return out;
}

WaterOverrideGuard build_water_override_guard(
		const std::vector<std::vector<uint8_t>> &grid,
		const std::vector<std::vector<float>> &heights, double cells_per_m2)
{
	WaterOverrideGuard out;
	const std::size_t height = grid.size(), width = height ? grid.front().size() : 0;
	if (width < 2 || height < 2)
		return out;
	// Rust returns no WaterContext when the source grid has no ESA-water seed.
	// In that case OSM water must not be constrained by a land-component mask:
	// with no reference water, every component would otherwise be classified as
	// protected land and polygon/river overrides would silently do nothing.
	const bool has_water_seed =
			std::any_of(grid.begin(), grid.end(), [&](const auto &row) {
				return row.size() >= width && std::find(row.begin(), row.begin() + width,
													  LC_WATER) != row.begin() + width;
			});
	if (!has_water_seed)
		return out;
	out.width = width;
	out.protected_mask.assign((width * height + 63) / 64, 0);
	auto bit = [&](std::size_t i) {
		return (out.protected_mask[i >> 6] >> (i & 63)) & 1U;
	};
	auto set = [&](std::size_t i) {
		out.protected_mask[i >> 6] |= std::uint64_t(1) << (i & 63);
	};
	const std::size_t threshold = std::max<std::size_t>(
			100, std::size_t(std::llround(4000.0 * std::max(0.0, cells_per_m2))));
	std::vector<std::uint32_t> seeds, stack;
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t x = 0; x < width; ++x) {
			const auto start = z * width + x;
			if (grid[z][x] == LC_WATER || bit(start))
				continue;
			stack = {std::uint32_t(start)};
			set(start);
			std::size_t count = 0, minx = x, maxx = x, minz = z, maxz = z;
			while (!stack.empty()) {
				auto cur = stack.back();
				stack.pop_back();
				auto cx = cur % width, cz = cur / width;
				++count;
				minx = std::min(minx, std::size_t(cx));
				maxx = std::max(maxx, std::size_t(cx));
				minz = std::min(minz, std::size_t(cz));
				maxz = std::max(maxz, std::size_t(cz));
				for (auto [dx, dz] : std::array<std::pair<int, int>, 4>{
							 {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}}) {
					int nx = int(cx) + dx, nz = int(cz) + dz;
					if (nx < 0 || nz < 0 || nx >= int(width) || nz >= int(height))
						continue;
					auto ni = std::size_t(nz) * width + nx;
					if (grid[nz][nx] != LC_WATER && !bit(ni)) {
						set(ni);
						stack.push_back(ni);
					}
				}
			}
			const auto long_axis = std::max(maxx - minx + 1, maxz - minz + 1),
					   short_axis = std::max<std::size_t>(
							   1, std::min(maxx - minx + 1, maxz - minz + 1));
			if (count >= threshold && double(long_axis) / short_axis <= 3.0)
				seeds.push_back(start);
		}
	std::fill(out.protected_mask.begin(), out.protected_mask.end(), 0);
	for (auto seed : seeds) {
		if (bit(seed))
			continue;
		stack = {seed};
		set(seed);
		while (!stack.empty()) {
			auto cur = stack.back();
			stack.pop_back();
			auto cx = cur % width, cz = cur / width;
			for (auto [dx, dz] : std::array<std::pair<int, int>, 4>{
						 {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}}) {
				int nx = int(cx) + dx, nz = int(cz) + dz;
				if (nx < 0 || nz < 0 || nx >= int(width) || nz >= int(height))
					continue;
				auto ni = std::size_t(nz) * width + nx;
				if (grid[nz][nx] != LC_WATER && !bit(ni)) {
					set(ni);
					stack.push_back(ni);
				}
			}
		}
	}
	if (heights.size() < height || heights.front().size() < width ||
			width * height > 100U * 1024U * 1024U)
		return out;
	out.nearest_water_y.assign(
			height, std::vector<float>(width, std::numeric_limits<float>::quiet_NaN()));
	std::deque<std::pair<int, int>> q;
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t x = 0; x < width; ++x)
			if (grid[z][x] == LC_WATER && std::isfinite(heights[z][x])) {
				out.nearest_water_y[z][x] = heights[z][x];
				q.push_back({int(x), int(z)});
			}
	while (!q.empty()) {
		auto [x, z] = q.front();
		q.pop_front();
		for (auto [dx, dz] :
				std::array<std::pair<int, int>, 4>{{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}}) {
			int nx = x + dx, nz = z + dz;
			if (nx < 0 || nz < 0 || nx >= int(width) || nz >= int(height) ||
					std::isfinite(out.nearest_water_y[nz][nx]))
				continue;
			out.nearest_water_y[nz][nx] = out.nearest_water_y[z][x];
			q.push_back({nx, nz});
		}
	}
	out.has_elevation = true;
	return out;
}

bool WaterOverrideGuard::allows(const std::vector<std::vector<float>> &heights,
		std::size_t x, std::size_t z, float tolerance) const
{
	if (!width || x >= width || z >= heights.size() || x >= heights[z].size())
		return true;
	if ((protected_mask[(z * width + x) >> 6] >> ((z * width + x) & 63)) & 1U)
		return false;
	if (!has_elevation || z >= nearest_water_y.size() || x >= nearest_water_y[z].size())
		return true;
	const float h = heights[z][x], water = nearest_water_y[z][x];
	return !std::isfinite(h) || !std::isfinite(water) || h <= water + tolerance;
}

std::vector<std::vector<uint8_t>> compute_water_distance(
		const std::vector<std::vector<uint8_t>> &grid, std::size_t width,
		std::size_t height)
{
	std::vector<std::vector<uint8_t>> dist(height, std::vector<uint8_t>(width, 0));
	if (width == 0 || height == 0)
		return dist;

	std::deque<std::pair<int, int>> q;
	const int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

	for (std::size_t z = 0; z < height; ++z) {
		for (std::size_t x = 0; x < width; ++x) {
			if (grid[z][x] != LC_WATER)
				continue;
			bool shore = false;
			for (const auto &d : dirs) {
				const int nx = static_cast<int>(x) + d[0];
				const int nz = static_cast<int>(z) + d[1];
				if (nx < 0 || nz < 0 || nx >= static_cast<int>(width) ||
						nz >= static_cast<int>(height) ||
						grid[static_cast<std::size_t>(nz)]
							[static_cast<std::size_t>(nx)] != LC_WATER) {
					shore = true;
					break;
				}
			}
			if (shore) {
				dist[z][x] = 1;
				q.emplace_back(static_cast<int>(x), static_cast<int>(z));
			}
		}
	}

	while (!q.empty()) {
		auto [x, z] = q.front();
		q.pop_front();
		if (dist[static_cast<std::size_t>(z)][static_cast<std::size_t>(x)] >= 15)
			continue;
		const uint8_t next =
				dist[static_cast<std::size_t>(z)][static_cast<std::size_t>(x)] ==
								std::numeric_limits<uint8_t>::max()
						? std::numeric_limits<uint8_t>::max()
						: static_cast<uint8_t>(dist[static_cast<std::size_t>(z)]
												   [static_cast<std::size_t>(x)] +
											   1);
		for (const auto &d : dirs) {
			const int nx = x + d[0];
			const int nz = z + d[1];
			if (nx < 0 || nz < 0 || nx >= static_cast<int>(width) ||
					nz >= static_cast<int>(height))
				continue;
			auto &cell = dist[static_cast<std::size_t>(nz)][static_cast<std::size_t>(nx)];
			if (grid[static_cast<std::size_t>(nz)][static_cast<std::size_t>(nx)] !=
							LC_WATER ||
					cell != 0)
				continue;
			cell = next;
			q.emplace_back(nx, nz);
		}
	}
	return dist;
}

void LandCoverData::refresh_water_blend_grid()
{
	water_blend_grid = compute_water_blend_smooth(grid, width, height, cells_per_meter);
}

void LandCoverData::remap_rows_to_mercator(double lat_top, double lat_bottom)
{
	const auto rows = height;
	const auto source = [=](std::size_t z) {
		return grid_ops::mercator_source_row(lat_top, lat_bottom, rows, z);
	};
	const auto nearest = [](std::uint8_t a, std::uint8_t b, double t) {
		return t < 0.5 ? a : b;
	};
	grid_ops::remap_rows_in_place(grid, source, nearest);
	grid_ops::remap_rows_in_place(water_distance, source, nearest);
	refresh_water_blend_grid();
}

void LandCoverData::crop(
		std::size_t x0, std::size_t z0, std::size_t new_width, std::size_t new_height)
{
	grid_ops::crop_rows(grid, x0, z0, new_width, new_height);
	grid_ops::crop_rows(water_distance, x0, z0, new_width, new_height);
	width = new_width;
	height = new_height;
	refresh_water_blend_grid();
}

void mark_beaches(LandCoverData &data)
{
	if (data.width == 0 || data.height == 0)
		return;
	const auto reach = std::clamp<int>(
			static_cast<int>(std::lround(50.0 * data.cells_per_meter)), 1, 255);
	std::vector<std::uint8_t> distance(data.width * data.height, 255);
	std::queue<std::pair<int, int>> frontier;
	for (std::size_t z = 0; z < data.height; ++z)
		for (std::size_t x = 0; x < data.width; ++x) {
			if (data.grid[z][x] != LC_WATER)
				continue;
			for (const auto &[dx, dz] : std::array<std::pair<int, int>, 4>{
						 {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}}) {
				const int nx = static_cast<int>(x) + dx, nz = static_cast<int>(z) + dz;
				if (nx < 0 || nz < 0 || nx >= static_cast<int>(data.width) ||
						nz >= static_cast<int>(data.height) ||
						data.grid[nz][nx] != LC_BARE)
					continue;
				const auto i = static_cast<std::size_t>(nz) * data.width + nx;
				if (distance[i] == 255) {
					distance[i] = 1;
					frontier.emplace(nx, nz);
				}
			}
		}
	while (!frontier.empty()) {
		const auto [x, z] = frontier.front();
		frontier.pop();
		const auto i = static_cast<std::size_t>(z) * data.width + x;
		data.grid[z][x] = LC_BEACH;
		if (distance[i] >= reach)
			continue;
		for (const auto &[dx, dz] :
				std::array<std::pair<int, int>, 4>{{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}}) {
			const int nx = x + dx, nz = z + dz;
			if (nx < 0 || nz < 0 || nx >= static_cast<int>(data.width) ||
					nz >= static_cast<int>(data.height) || data.grid[nz][nx] != LC_BARE)
				continue;
			const auto ni = static_cast<std::size_t>(nz) * data.width + nx;
			if (distance[ni] == 255) {
				distance[ni] = static_cast<std::uint8_t>(distance[i] + 1);
				frontier.emplace(nx, nz);
			}
		}
	}
}

std::size_t clear_land_cover_cache(const std::filesystem::path &cache_dir)
{
	std::size_t removed = 0;
	if (!std::filesystem::exists(cache_dir))
		return 0;
	for (const auto &entry : std::filesystem::directory_iterator(cache_dir)) {
		std::error_code ec;
		if (entry.is_regular_file(ec) && std::filesystem::remove(entry.path(), ec) && !ec)
			++removed;
	}
	return removed;
}

}
