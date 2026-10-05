#include "facades.h"
#include "../bresenham.h"
#include "../block_palette.h"
#include "../element_processing/buildings.h"
#include "displays.h"
#include "../../arnis_adapter.h"
#include "../../arnis_world_editor.h"
#include "../floodfill_cache.h"
#include "stb_image.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <set>
#include <shared_mutex>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace arnis::mapillary::facades
{
namespace
{
std::optional<std::uint64_t> u64(const nlohmann::json &value)
{
	if (value.is_number_unsigned())
		return value.get<std::uint64_t>();
	if (value.is_number_integer() && value.get<std::int64_t>() >= 0)
		return static_cast<std::uint64_t>(value.get<std::int64_t>());
	if (value.is_string()) {
		try {
			return std::stoull(value.get<std::string>());
		} catch (...) {
		}
	}
	return std::nullopt;
}

std::shared_mutex store_mutex;
std::optional<Export> store;
double world_scale = 1.0;

struct CellKey
{
	int x = 0, z = 0;
	bool operator==(const CellKey &) const = default;
};

struct CellHash
{
	std::size_t operator()(const CellKey &p) const noexcept
	{
		const auto x = std::hash<std::uint32_t>{}(static_cast<std::uint32_t>(p.x));
		const auto z = std::hash<std::uint32_t>{}(static_cast<std::uint32_t>(p.z));
		return x ^ (z + std::size_t{0x9e3779b9} + (x << 6) + (x >> 2));
	}
};

struct CellRef
{
	std::size_t building = 0, wall = 0;
	std::uint32_t col = 0;
	std::uint64_t way_id = 0;
	std::int8_t nx = 0, nz = 0;
	std::int32_t direction_x = 0, direction_z = 0;
};

std::unordered_map<CellKey, CellRef, CellHash> projected_cells;
std::unordered_map<CellKey, std::uint64_t, CellHash> projected_photo_cells;
std::unordered_map<std::uint64_t, imgops::Rgb> projected_building_colors;
std::unordered_set<std::uint64_t> projected_buildings;
bool displays_enabled = false;

std::optional<Owner> owner_for(const nlohmann::json &record)
{
	const bool relation = record.value("kind", "") == "relation";
	const char *primary = relation ? "relation_id" : "way_id";
	const char *fallback = "osm_id";
	const auto id = record.contains(primary) ? u64(record[primary]) : std::nullopt;
	const auto fallback_id =
			record.contains(fallback) ? u64(record[fallback]) : std::nullopt;
	if (!id && !fallback_id)
		return std::nullopt;
	return Owner{
			relation ? OwnerKind::Relation : OwnerKind::Way, id ? *id : *fallback_id};
}

std::optional<std::filesystem::path> safe_image_path(
		const std::filesystem::path &directory, const std::string &name)
{
	if (name.empty() || name.size() > 128 || name.find("..") != std::string::npos ||
			!std::all_of(name.begin(), name.end(), [](unsigned char c) {
				return std::isalnum(c) || c == '_' || c == '-' || c == '.';
			}))
		return std::nullopt;
	for (const auto &candidate :
			{directory / name, directory / "texture" / "blocks" / name,
					directory / "texture" / "tex" / name})
		if (std::filesystem::is_regular_file(candidate))
			return candidate;
	return std::nullopt;
}

bool load_wall_pixels(const std::filesystem::path &path, ExportWall &wall)
{
	int width = 0, height = 0, channels = 0;
	stbi_uc *pixels = stbi_load(path.string().c_str(), &width, &height, &channels, 4);
	if (!pixels || width <= 0 || height <= 0 ||
			static_cast<std::uint32_t>(width) != wall.columns ||
			static_cast<std::uint32_t>(height) != wall.rows) {
		if (pixels)
			stbi_image_free(pixels);
		return false;
	}
	const auto count = static_cast<std::size_t>(width) * height;
	if (count > 16 * 1024 * 1024) {
		stbi_image_free(pixels);
		return false;
	}
	wall.cells.resize(count);
	for (std::size_t i = 0; i < count; ++i)
		wall.cells[i] = {
				pixels[i * 4], pixels[i * 4 + 1], pixels[i * 4 + 2], pixels[i * 4 + 3]};
	stbi_image_free(pixels);

	std::vector<std::array<std::vector<std::uint8_t>, 3>> row_colors(wall.rows);
	for (std::uint32_t row = 0; row < wall.rows; ++row)
		for (std::uint32_t col = 0; col < wall.columns; ++col) {
			const auto &cell = wall.cells[std::size_t(row) * wall.columns + col];
			if (cell[3] != 255)
				continue;
			for (std::size_t channel = 0; channel < 3; ++channel)
				row_colors[row][channel].push_back(cell[channel]);
		}
	std::vector<std::optional<imgops::Rgb>> own(wall.rows);
	for (std::uint32_t row = 0; row < wall.rows; ++row) {
		if (row_colors[row][0].empty())
			continue;
		imgops::Rgb rgb{};
		for (std::size_t channel = 0; channel < 3; ++channel) {
			auto &values = row_colors[row][channel];
			std::sort(values.begin(), values.end());
			rgb[channel] = values[values.size() / 2];
		}
		own[row] = rgb;
	}
	if (std::none_of(own.begin(), own.end(), [](const auto &v) { return v.has_value(); }))
		return true;
	wall.row_bands.resize(wall.rows);
	for (std::uint32_t row = 0; row < wall.rows; ++row) {
		std::size_t best_distance = std::numeric_limits<std::size_t>::max();
		for (std::uint32_t candidate = 0; candidate < wall.rows; ++candidate)
			if (own[candidate]) {
				const auto distance = candidate > row ? candidate - row : row - candidate;
				if (distance < best_distance ||
						(distance == best_distance && candidate > row)) {
					best_distance = distance;
					wall.row_bands[row] = *own[candidate];
				}
			}
	}
	return true;
}

bool load_texture_pixels(const std::filesystem::path &path, ExportWall &wall)
{
	int width = 0, height = 0, channels = 0;
	stbi_uc *pixels = stbi_load(path.string().c_str(), &width, &height, &channels, 4);
	if (!pixels || width <= 0 || height <= 0 ||
			static_cast<std::size_t>(width) * height > 16 * 1024 * 1024) {
		if (pixels)
			stbi_image_free(pixels);
		return false;
	}
	wall.texture_width = static_cast<std::uint32_t>(width);
	wall.texture_height = static_cast<std::uint32_t>(height);
	const auto count = static_cast<std::size_t>(width) * height;
	wall.texture_pixels.resize(count);
	for (std::size_t i = 0; i < count; ++i)
		wall.texture_pixels[i] = {
				pixels[i * 4], pixels[i * 4 + 1], pixels[i * 4 + 2], pixels[i * 4 + 3]};
	stbi_image_free(pixels);
	return true;
}
} // namespace

std::optional<Export> load_export(
		const std::filesystem::path &directory, std::string *error)
{
	std::error_code ec;
	if (!std::filesystem::is_directory(directory, ec)) {
		if (error)
			*error = "not a facade export directory: " + directory.string();
		return std::nullopt;
	}
	Export result;
	std::set<std::string> images;
	for (const auto &entry : std::filesystem::directory_iterator(directory, ec)) {
		if (ec)
			break;
		if (!entry.is_regular_file(ec) || entry.path().extension() != ".json" ||
				entry.path().filename() == "manifest.json")
			continue;
		std::ifstream input(entry.path());
		if (!input)
			continue;
		try {
			nlohmann::json record;
			input >> record;
			const auto owner = owner_for(record);
			if (!owner)
				continue;
			ExportBuilding building;
			building.owner = *owner;
			if (const auto color = record.value("building_colour", nlohmann::json{});
					color.is_object() && color.contains("rgb") &&
					color["rgb"].is_array() && color["rgb"].size() == 3) {
				imgops::Rgb rgb{};
				for (std::size_t i = 0; i < 3; ++i)
					rgb[i] = static_cast<std::uint8_t>(std::min<std::uint64_t>(
							255, color["rgb"][i].is_number()
										 ? color["rgb"][i].get<std::uint64_t>()
										 : 0));
				building.building_color = rgb;
			}
			for (const auto &wall : record.value("walls", nlohmann::json::array())) {
				const auto tier = wall.value("tier", "D");
				if (tier != "A" && tier != "B")
					continue;
				ExportWall out;
				out.owner = *owner;
				out.tier = tier;
				out.columns = wall.value("cols", wall.value("columns", 0u));
				out.rows = wall.value("rows", 0u);
				if (out.columns == 0 || out.rows == 0 || out.columns > 16384 ||
						out.rows > 16384)
					continue;
				const auto color_name = wall.value("png", "");
				const auto color_path = safe_image_path(directory, color_name);
				if (!color_path || !load_wall_pixels(*color_path, out))
					continue;
				out.color_png = *color_path;
				if (const auto name = wall.value("tex", ""); !name.empty())
					if (const auto path = safe_image_path(directory, name))
						if (load_texture_pixels(*path, out))
							out.texture_png = *path;
				if (const auto extent = wall.value("extent", nlohmann::json{});
						extent.is_object() &&
						extent.value("s_l", nlohmann::json{}).is_number())
					out.col0_m = extent["s_l"].get<double>();
				for (const auto &edge : wall.value("edges", nlohmann::json::array())) {
					const auto node_a = edge.is_object() && edge.contains("node_a")
												? u64(edge["node_a"])
												: std::nullopt;
					const auto node_b = edge.is_object() && edge.contains("node_b")
												? u64(edge["node_b"])
												: std::nullopt;
					if (!node_a || !node_b)
						continue;
					ExportWall::Edge parsed;
					parsed.node_a = *node_a;
					parsed.node_b = *node_b;
					parsed.col0 = edge.value("col0", 0);
					parsed.col1 = edge.value("col1", static_cast<int>(out.columns) - 1);
					if (edge.contains("s0") && edge.contains("s1") &&
							edge["s0"].is_number() && edge["s1"].is_number()) {
						const double s0 = edge["s0"].get<double>();
						const double s1 = edge["s1"].get<double>();
						if (s1 > s0)
							parsed.span_m = std::pair{s0, s1};
					}
					out.edges.push_back(std::move(parsed));
				}
				if (out.edges.empty()) {
					const auto node_a =
							wall.contains("node_a") ? u64(wall["node_a"]) : std::nullopt;
					const auto node_b =
							wall.contains("node_b") ? u64(wall["node_b"]) : std::nullopt;
					if (!node_a || !node_b)
						continue;
					out.edges.push_back({*node_a, *node_b, 0,
							static_cast<std::int32_t>(out.columns) - 1, std::nullopt});
				}
				for (const auto &view : wall.value("views", nlohmann::json::array())) {
					const auto id = view.is_string() ? view.get<std::string>()
													 : view.value("pano", "");
					if (!id.empty()) {
						out.views.push_back(id);
						images.insert(id);
					}
				}
				building.walls.push_back(std::move(out));
			}
			if (!building.walls.empty())
				result.buildings.push_back(std::move(building));
		} catch (const std::exception &e) {
			if (error)
				*error = entry.path().string() + ": " + e.what();
			return std::nullopt;
		}
	}
	result.image_ids.assign(images.begin(), images.end());
	return result;
}

void install_export(Export value)
{
	std::unique_lock lock(store_mutex);
	store = std::move(value);
	displays_enabled = false;
	projected_cells.clear();
	projected_photo_cells.clear();
	projected_building_colors.clear();
	projected_buildings.clear();
}

void set_displays_enabled(bool enabled)
{
	std::unique_lock lock(store_mutex);
	displays_enabled = enabled && store.has_value();
}

void clear()
{
	std::unique_lock lock(store_mutex);
	store.reset();
	displays_enabled = false;
	projected_cells.clear();
	projected_photo_cells.clear();
	projected_building_colors.clear();
	projected_buildings.clear();
	world_scale = 1.0;
}

void project_export(const std::vector<arnis::ProcessedElement> &elements,
		const XZBBox &bbox, double scale)
{
	std::unique_lock lock(store_mutex);
	projected_cells.clear();
	projected_photo_cells.clear();
	projected_building_colors.clear();
	projected_buildings.clear();
	world_scale = std::max(scale, 1e-9);
	if (!store)
		return;
	std::unordered_map<std::uint64_t, const arnis::ProcessedWay *> ways;
	std::unordered_map<std::uint64_t, const arnis::ProcessedRelation *> relations;
	ways.reserve(elements.size());
	for (const auto &element : elements)
		if (element.is_way()) {
			const auto &way = element.as_way();
			ways.insert_or_assign(way.id, &way);
		} else if (element.is_relation()) {
			const auto &relation = element.as_relation();
			relations.insert_or_assign(relation.id, &relation);
		}
	std::unordered_map<std::uint64_t, std::vector<arnis::ProcessedWay>> relation_rings;
	for (const auto &[relation_id, relation] : relations) {
		const bool has_export = std::any_of(store->buildings.begin(),
				store->buildings.end(), [&](const ExportBuilding &building) {
					return building.owner.kind == OwnerKind::Relation &&
						   building.owner.id == relation_id;
				});
		if (!has_export)
			continue;
		auto rings = buildings::facade_outer_rings(*relation, bbox);
		for (const auto &building : store->buildings)
			if (building.owner.kind == OwnerKind::Relation &&
					building.owner.id == relation_id && building.building_color)
				for (const auto &ring : rings)
					projected_building_colors.insert_or_assign(
							ring.id, *building.building_color);
		relation_rings.emplace(relation_id, std::move(rings));
	}
	auto project_wall = [&](std::size_t building_index, std::size_t wall_index,
								const ExportWall &wall, const arnis::ProcessedWay &way) {
		if (way.nodes.size() < 2)
			return;
		double twice_area = 0.0;
		for (std::size_t i = 0; i < way.nodes.size(); ++i) {
			const auto &a = way.nodes[i];
			const auto &b = way.nodes[(i + 1) % way.nodes.size()];
			twice_area += static_cast<double>(a.x) * b.z - static_cast<double>(b.x) * a.z;
		}
		const bool clockwise = twice_area > 0.0;
		for (const auto &edge : wall.edges) {
			for (std::size_t segment = 1; segment < way.nodes.size(); ++segment) {
				const auto &a = way.nodes[segment - 1];
				const auto &b = way.nodes[segment];
				const bool forward = a.id == edge.node_a && b.id == edge.node_b;
				const bool backward = a.id == edge.node_b && b.id == edge.node_a;
				if (!forward && !backward)
					continue;
				projected_buildings.insert(way.id);
				const double dx = static_cast<double>(b.x - a.x);
				const double dz = static_cast<double>(b.z - a.z);
				const double length = std::max(1e-9, std::hypot(dx, dz));
				const double tx = dx / length, tz = dz / length;
				const double nx = clockwise ? tz : -tz;
				const double nz = clockwise ? -tx : tx;
				const std::int8_t snx =
						std::abs(nx) >= std::abs(nz)
								? static_cast<std::int8_t>((nx > 0.0) - (nx < 0.0))
								: 0;
				const std::int8_t snz =
						std::abs(nx) >= std::abs(nz)
								? 0
								: static_cast<std::int8_t>((nz > 0.0) - (nz < 0.0));
				std::optional<std::pair<double, double>> span = edge.span_m;
				if (span && !forward)
					std::swap(span->first, span->second);
				const auto points = bresenham::bresenham_line(a.x, 0, a.z, b.x, 0, b.z);
				const auto count = points.size();
				std::vector<bool> covered(count, false);
				for (std::size_t i = 0; i < count; ++i) {
					const int x = std::get<0>(points[i]);
					const int z = std::get<2>(points[i]);
					if (x < bbox.min_x() || x > bbox.max_x() || z < bbox.min_z() ||
							z > bbox.max_z())
						continue;
					const double fraction =
							count > 1 ? static_cast<double>(i) / (count - 1) : 0.0;
					int column;
					if (span) {
						const double c = span->first +
										 fraction * (span->second - span->first) -
										 wall.col0_m;
						if (c < 0.0 || c >= wall.columns)
							continue;
						column = static_cast<int>(std::floor(c));
					} else {
						const int span_columns = std::max(0, edge.col1 - edge.col0);
						const double along = forward ? fraction : 1.0 - fraction;
						column = std::clamp(edge.col0 + static_cast<int>(std::lround(
																along * span_columns)),
								0, static_cast<int>(wall.columns) - 1);
					}
					projected_cells[{x, z}] = {building_index, wall_index,
							static_cast<std::uint32_t>(column), way.id, snx, snz,
							b.x - a.x, b.z - a.z};
					covered[i] = true;
				}
				for (std::size_t i = 0; i < count; ++i) {
					if (!covered[i])
						continue;
					for (std::size_t nearr = i > 0 ? i - 1 : i;
							nearr <= std::min(count - 1, i + 1); ++nearr) {
						projected_photo_cells.insert_or_assign(
								CellKey{std::get<0>(points[nearr]),
										std::get<2>(points[nearr])},
								way.id);
					}
				}
				break;
			}
		}
	};
	for (std::size_t building_index = 0; building_index < store->buildings.size();
			++building_index) {
		auto &building = store->buildings[building_index];
		for (std::size_t wall_index = 0; wall_index < building.walls.size();
				++wall_index) {
			const auto &wall = building.walls[wall_index];
			if (building.owner.kind == OwnerKind::Way) {
				projected_buildings.insert(building.owner.id);
				const auto way_it = ways.find(building.owner.id);
				if (way_it != ways.end())
					project_wall(building_index, wall_index, wall, *way_it->second);
				continue;
			}
			const auto relation_it = relations.find(building.owner.id);
			const auto rings_it = relation_rings.find(building.owner.id);
			if (relation_it == relations.end() || rings_it == relation_rings.end())
				continue;
			for (const auto &ring : rings_it->second)
				project_wall(building_index, wall_index, wall, ring);
		}
	}
}

bool has_building(std::uint64_t way_id)
{
	std::shared_lock lock(store_mutex);
	return projected_buildings.contains(way_id);
}

std::optional<imgops::Rgb> building_color(std::uint64_t way_id)
{
	std::shared_lock lock(store_mutex);
	if (!store)
		return std::nullopt;
	if (const auto projected = projected_building_colors.find(way_id);
			projected != projected_building_colors.end())
		return projected->second;
	for (const auto &building : store->buildings)
		if (building.owner.kind == OwnerKind::Way && building.owner.id == way_id)
			return building.building_color;
	return std::nullopt;
}

bool photo_column(int x, int z, std::uint64_t way_id)
{
	std::shared_lock lock(store_mutex);
	if (!store || world_scale < 2.0)
		return false;
	const auto found = projected_photo_cells.find({x, z});
	return found != projected_photo_cells.end() && found->second == way_id;
}

std::size_t collect_displays(arnis::world_editor::WorldEditor &editor,
		std::uint64_t way_id, int start_y_offset, int abs_terrain_offset,
		int building_height, std::uint32_t pixels_per_block)
{
	std::shared_lock lock(store_mutex);
	if (!store || !displays_enabled || !editor.map_decals_enabled() ||
			building_height <= 0 || pixels_per_block == 0)
		return 0;
	struct WallCells
	{
		std::vector<std::tuple<int, int, std::uint32_t>> cells;
		std::int64_t snapped_x = 0, snapped_z = 0;
		std::int64_t direction_x = 0, direction_z = 0;
	};
	std::map<std::pair<std::size_t, std::size_t>, WallCells> by_wall;
	for (const auto &[position, cell] : projected_cells) {
		if (cell.way_id != way_id || cell.building >= store->buildings.size() ||
				cell.wall >= store->buildings[cell.building].walls.size())
			continue;
		const auto &wall = store->buildings[cell.building].walls[cell.wall];
		if (wall.texture_pixels.empty())
			continue;
		auto &group = by_wall[{cell.building, cell.wall}];
		group.cells.emplace_back(position.x, position.z, cell.col);
		group.snapped_x += cell.nx;
		group.snapped_z += cell.nz;
		group.direction_x += cell.direction_x;
		group.direction_z += cell.direction_z;
	}
	if (by_wall.empty())
		return 0;

	std::size_t placed = 0;
	const int base_y = start_y_offset + 1 + abs_terrain_offset;
	for (auto &[owner, group] : by_wall) {
		const auto &texture = store->buildings[owner.first].walls[owner.second];
		const auto direction =
				std::pair{static_cast<int>(std::clamp<std::int64_t>(group.direction_x,
								  std::numeric_limits<int>::min(),
								  std::numeric_limits<int>::max())),
						static_cast<int>(std::clamp<std::int64_t>(group.direction_z,
								std::numeric_limits<int>::min(),
								std::numeric_limits<int>::max()))};
		const auto normal =
				displays::outward_normal(direction, group.snapped_x, group.snapped_z);
		if (!normal || texture.texture_width == 0 || texture.texture_height == 0)
			continue;
		const int total_height = std::min(
				static_cast<int>(
						std::llround(static_cast<double>(texture.rows) * world_scale)),
				building_height);
		if (total_height <= 0)
			continue;
		const int top = base_y + total_height;
		if (std::none_of(group.cells.begin(), group.cells.end(), [&](const auto &cell) {
				const int x = std::get<0>(cell), z = std::get<1>(cell);
				return top > editor.get_absolute_y(x, 0, z) + 1;
			}))
			continue;
		const auto right = displays::right_of(*normal);
		std::sort(group.cells.begin(), group.cells.end(),
				[&](const auto &a, const auto &b) {
					const double pa =
							std::get<0>(a) * right.first + std::get<1>(a) * right.second;
					const double pb =
							std::get<0>(b) * right.first + std::get<1>(b) * right.second;
					return pa < pb;
				});
		const double step = displays::cell_step(direction);
		if (step <= 0.0)
			continue;
		std::vector<std::pair<int, int>> footprints;
		footprints.reserve(group.cells.size());
		for (const auto &cell : group.cells)
			footprints.emplace_back(std::get<0>(cell), std::get<1>(cell));
		const auto wall_pieces =
				displays::pieces(footprints, *normal, step, base_y, total_height);
		for (const auto &piece : wall_pieces) {
			if (piece.p0 < 0 || piece.p1 <= piece.p0 ||
					static_cast<std::size_t>(piece.p1) > group.cells.size())
				continue;
			std::uint32_t first_col = std::numeric_limits<std::uint32_t>::max();
			std::uint32_t last_col = 0;
			for (int index = piece.p0; index < piece.p1; ++index) {
				const auto col =
						std::get<2>(group.cells[static_cast<std::size_t>(index)]);
				first_col = std::min(first_col, col);
				last_col = std::max(last_col, col);
			}
			if (first_col >= texture.columns || last_col >= texture.columns)
				continue;
			const double u0 = static_cast<double>(first_col) * 8.0;
			const double u1 = static_cast<double>(last_col + 1) * 8.0;
			const double v0 = (static_cast<double>(texture.rows) -
									  static_cast<double>(piece.b1) / world_scale) *
							  8.0;
			const double v1 = (static_cast<double>(texture.rows) -
									  static_cast<double>(piece.b0) / world_scale) *
							  8.0;
			const auto x0 = static_cast<std::uint32_t>(std::clamp(
					std::floor(u0), 0.0, static_cast<double>(texture.texture_width - 1)));
			const auto x1 = static_cast<std::uint32_t>(
					std::clamp(std::ceil(u1), static_cast<double>(x0 + 1),
							static_cast<double>(texture.texture_width)));
			const auto y0 = static_cast<std::uint32_t>(std::clamp(std::floor(v0), 0.0,
					static_cast<double>(texture.texture_height - 1)));
			const auto y1 = static_cast<std::uint32_t>(
					std::clamp(std::ceil(v1), static_cast<double>(y0 + 1),
							static_cast<double>(texture.texture_height)));
			const auto crop_width = x1 - x0, crop_height = y1 - y0;
			if (crop_width == 0 || crop_height == 0 ||
					static_cast<std::size_t>(crop_width) * crop_height >
							16 * 1024 * 1024 ||
					texture.texture_pixels.size() <
							static_cast<std::size_t>(texture.texture_width) *
									texture.texture_height)
				continue;
			const auto count = static_cast<std::size_t>(crop_width) * crop_height;
			std::vector<std::array<std::uint8_t, 3>> rgb(count);
			std::vector<bool> valid(count);
			std::vector<std::array<std::uint8_t, 3>> valid_rgb;
			for (std::uint32_t y = 0; y < crop_height; ++y)
				for (std::uint32_t x = 0; x < crop_width; ++x) {
					const auto &px =
							texture.texture_pixels[static_cast<std::size_t>(y + y0) *
														   texture.texture_width +
												   x + x0];
					const auto i = static_cast<std::size_t>(y) * crop_width + x;
					rgb[i] = {px[0], px[1], px[2]};
					valid[i] = px[3] > 0;
					if (valid[i])
						valid_rgb.push_back(rgb[i]);
				}
			if (valid_rgb.empty() || valid_rgb.size() < count / 4)
				continue;
			auto median = [](std::vector<std::array<std::uint8_t, 3>> values) {
				std::array<std::uint8_t, 3> result{};
				for (std::size_t channel = 0; channel < 3; ++channel) {
					std::vector<std::uint8_t> values_channel;
					values_channel.reserve(values.size());
					for (const auto &value : values)
						values_channel.push_back(value[channel]);
					std::sort(values_channel.begin(), values_channel.end());
					result[channel] = values_channel[values_channel.size() / 2];
				}
				return result;
			};
			const auto crop_median = median(valid_rgb);
			std::vector<std::array<std::uint8_t, 3>> filled = rgb;
			for (std::uint32_t y = 0; y < crop_height; ++y) {
				std::vector<std::array<std::uint8_t, 3>> row_pixels;
				for (std::uint32_t x = 0; x < crop_width; ++x) {
					const auto i = static_cast<std::size_t>(y) * crop_width + x;
					if (valid[i])
						row_pixels.push_back(rgb[i]);
				}
				const auto fill = row_pixels.empty() ? crop_median : median(row_pixels);
				for (std::uint32_t x = 0; x < crop_width; ++x) {
					const auto i = static_cast<std::size_t>(y) * crop_width + x;
					if (!valid[i])
						filled[i] = fill;
				}
			}
			std::vector<std::array<std::uint8_t, 3>> smooth = filled;
			for (std::uint32_t y = 0; y < crop_height; ++y)
				for (std::uint32_t x = 0; x < crop_width; ++x) {
					const auto i = static_cast<std::size_t>(y) * crop_width + x;
					if (valid[i])
						continue;
					std::array<std::uint32_t, 3> sum{};
					std::uint32_t samples = 0;
					for (std::uint32_t sy = y > 0 ? y - 1 : y;
							sy <= std::min(crop_height - 1, y + 1); ++sy)
						for (std::uint32_t sx = x > 0 ? x - 1 : x;
								sx <= std::min(crop_width - 1, x + 1); ++sx) {
							const auto &pixel =
									filled[static_cast<std::size_t>(sy) * crop_width +
											sx];
							for (std::size_t c = 0; c < 3; ++c)
								sum[c] += pixel[c];
							++samples;
						}
					for (std::size_t c = 0; c < 3; ++c)
						smooth[i][c] = static_cast<std::uint8_t>(sum[c] / samples);
				}
			if (displays::flip_crop(direction, *normal))
				for (std::uint32_t y = 0; y < crop_height; ++y)
					std::reverse(
							smooth.begin() + static_cast<std::size_t>(y) * crop_width,
							smooth.begin() +
									static_cast<std::size_t>(y + 1) * crop_width);
			std::vector<std::uint8_t> crop_rgb;
			crop_rgb.reserve(count * 3);
			for (const auto &pixel : smooth)
				crop_rgb.insert(crop_rgb.end(), pixel.begin(), pixel.end());
			const auto sprite =
					displays::Sprite::of(piece.quad.w, piece.quad.h, pixels_per_block);
			const auto panel_rgb =
					displays::lay_out_rgb(crop_rgb, crop_width, crop_height, sprite);
			if (!panel_rgb.empty() &&
					editor.place_facade_panel(
							static_cast<int>(std::llround(piece.quad.cx)),
							static_cast<int>(std::llround(piece.quad.cy)),
							static_cast<int>(std::llround(piece.quad.cz)),
							displays::facing_index(*normal), panel_rgb, sprite.side_w,
							sprite.side_h))
				++placed;
		}
	}
	return placed;
}

std::optional<Block> block_at(
		int x, int y, int z, int start_y_offset, std::uint64_t way_id, Block window_block)
{
	std::shared_lock lock(store_mutex);
	if (!store || world_scale < 2.0)
		return std::nullopt;
	const auto found = projected_cells.find({x, z});
	if (found == projected_cells.end())
		return std::nullopt;
	const auto &ref = found->second;
	const auto &building = store->buildings[ref.building];
	const auto &wall = building.walls[ref.wall];
	if (ref.way_id != way_id)
		return std::nullopt;
	const int height_index = y - start_y_offset - 1;
	if (height_index < 0)
		return std::nullopt;
	const auto metres_up =
			static_cast<std::int64_t>(std::floor(height_index / world_scale));
	const auto row_signed = static_cast<std::int64_t>(wall.rows) - 1 - metres_up;
	if (row_signed < 0 || ref.col >= wall.columns ||
			static_cast<std::uint64_t>(row_signed) >= wall.rows)
		return std::nullopt;
	const auto &cell =
			wall.cells[static_cast<std::size_t>(row_signed) * wall.columns + ref.col];
	if (cell[3] == 255)
		return block_palette::facade_block_for_color({cell[0], cell[1], cell[2]});
	if (cell[3] == 192)
		return window_block;
	if (cell[3] == 128)
		return DARK_OAK_PLANKS;
	return std::nullopt;
}

std::optional<Block> band_block_at(
		int x, int y, int z, int start_y_offset, std::uint64_t way_id)
{
	std::shared_lock lock(store_mutex);
	if (!store || world_scale >= 2.0)
		return std::nullopt;
	const auto found = projected_cells.find({x, z});
	if (found == projected_cells.end())
		return std::nullopt;
	const auto &ref = found->second;
	const auto &building = store->buildings[ref.building];
	const auto &wall = building.walls[ref.wall];
	if (ref.way_id != way_id)
		return std::nullopt;
	const int height_index = y - start_y_offset - 1;
	if (height_index < 0)
		return std::nullopt;
	const auto metres_up =
			static_cast<std::int64_t>(std::floor(height_index / world_scale));
	const auto row_signed = static_cast<std::int64_t>(wall.rows) - 1 - metres_up;
	if (row_signed < 0 || static_cast<std::uint64_t>(row_signed) >= wall.row_bands.size())
		return std::nullopt;
	const auto &rgb = wall.row_bands[static_cast<std::size_t>(row_signed)];
	return block_palette::facade_block_for_color({rgb[0], rgb[1], rgb[2]});
}
} // namespace arnis::mapillary::facades
