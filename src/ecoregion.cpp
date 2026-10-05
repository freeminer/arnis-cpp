#include "ecoregion.h"
#include "assets_root.h"
#include "fm_ecoregion_cache.h"
#include "coordinate_system/cartesian/xzpoint.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <unordered_map>
#include <vector>
#include "land_cover/land_cover.h"

namespace arnis::ecoregion
{
namespace
{
struct Row
{
	Ecoregion eco;
	std::string pack, mix, name;
};
std::vector<std::optional<Row>> &rows()
{
	static std::vector<std::optional<Row>> value;
	static std::once_flag once;
	std::call_once(once, [] {
		const auto path = assets::path("climate/ecoregions.tsv");
		std::ifstream in;
		in.open(path);
		if (!in)
			return;
		std::string line;
		while (std::getline(in, line)) {
			std::istringstream s(line);
			std::string f;
			std::vector<std::string> fields;
			while (std::getline(s, f, '\t'))
				fields.push_back(f);
			if (fields.size() < 3)
				continue;
			char *end = nullptr;
			const auto id = std::strtoul(fields[0].c_str(), &end, 10);
			if (!end || *end || id > UINT16_MAX)
				continue;
			int biome = 0;
			try {
				biome = fields[1].empty() ? 0 : std::stoi(fields[1]);
			} catch (...) {
				continue;
			}
			if (biome < 1 || biome > 14)
				continue;
			static const std::array<const char *, 8> realms = {
					"AT", "AN", "AA", "IM", "NA", "NT", "OC", "PA"};
			const auto it = std::find(realms.begin(), realms.end(), fields[2]);
			if (it == realms.end())
				continue;
			if (value.size() <= id)
				value.resize(id + 1);
			value[id] =
					Row{{static_cast<std::uint16_t>(id), static_cast<EcoBiome>(biome - 1),
								static_cast<Realm>(it - realms.begin())},
							fields.size() > 3 ? fields[3] : "",
							fields.size() > 4 ? fields[4] : "",
							fields.size() > 5 ? fields[5] : ""};
		}
	});
	return value;
}
const Row *row(std::uint16_t id)
{
	auto &r = rows();
	return id < r.size() && r[id] ? &*r[id] : nullptr;
}

double warped_noise(int x, int z, int scale, std::uint32_t salt)
{
	constexpr double c = 0.891006524188368, s = 0.453990499739547;
	const double u = (x * c - z * s) / std::max(1, scale);
	const double v = (x * s + z * c) / std::max(1, scale);
	const int u0 = static_cast<int>(std::floor(u)), v0 = static_cast<int>(std::floor(v));
	const double tu = u - u0, tv = v - v0;
	const double su = tu * tu * (3.0 - 2.0 * tu), sv = tv * tv * (3.0 - 2.0 * tv);
	const int sx = static_cast<int>(salt),
			  sz = static_cast<int>((salt << 16) | (salt >> 16));
	const auto sample = [&](int a, int b) {
		return double(land_cover::coord_hash(a ^ sx, b ^ sz) % 1000) / 1000.0;
	};
	const double a = sample(u0, v0) * (1.0 - su) + sample(u0 + 1, v0) * su;
	const double b = sample(u0, v0 + 1) * (1.0 - su) + sample(u0 + 1, v0 + 1) * su;
	return a * (1.0 - sv) + b * sv;
}

std::pair<double, double> warp(double col, double row)
{
	const int x = static_cast<int>(std::floor(col * 64.0));
	const int z = static_cast<int>(std::floor(row * 64.0));
	const auto octave = [&](std::uint32_t salt, int period, double amp) {
		return (warped_noise(x, z, period, salt) - .5) * 2.0 * amp;
	};
	return {octave(0xEC00001, 384, .9) + octave(0xEC00002, 96, .35) +
					octave(0xEC00005, 16, .12),
			octave(0xEC00003, 384, .9) + octave(0xEC00004, 96, .35) +
					octave(0xEC00006, 16, .12)};
}

std::pair<std::size_t, std::size_t> warped_cell(
		const geo_grid::TiledGrid &grid, double lat, double lon)
{
	const auto [col, row] = grid.position(lat, lon);
	const auto offset = warp(col, row);
	return grid.cell(col + offset.first, row + offset.second);
}

std::int32_t euclidean_remainder(std::int32_t value, std::int32_t divisor)
{
	const auto r = value % divisor;
	return r < 0 ? r + divisor : r;
}
}
std::optional<Ecoregion> lookup(std::uint16_t id)
{
	if (auto *r = row(id))
		return r->eco;
	return {};
}
std::optional<std::string_view> name(std::uint16_t id)
{
	if (auto *r = row(id))
		return r->name;
	return {};
}
std::optional<std::pair<std::string_view, std::string_view>> tree_mix(std::uint16_t id)
{
	if (auto *r = row(id); r && !r->pack.empty())
		return {{r->pack, r->mix}};
	return {};
}
bool realm_is_americas(Realm r)
{
	return r == Realm::Nearctic || r == Realm::Neotropic;
}
bool palms_belong(Ecoregion e, double lat)
{
	const auto a = std::abs(lat);
	switch (e.biome) {
	case EcoBiome::MoistTropical:
	case EcoBiome::DryTropical:
	case EcoBiome::TropicalConifer:
	case EcoBiome::TropicalGrassland:
	case EcoBiome::Mangroves:
		return true;
	case EcoBiome::Boreal:
	case EcoBiome::MontaneGrassland:
	case EcoBiome::Tundra:
		return false;
	case EcoBiome::Mediterranean:
		return a <= 45.0;
	case EcoBiome::TemperateBroadleaf:
		return e.realm == Realm::Australasia ? a <= 42.0 : a <= 35.0;
	default:
		return a <= 35.0;
	}
}

std::optional<EcoMap> EcoMap::load(const std::filesystem::path &path)
{
	std::ifstream in(path, std::ios::binary);
	if (!in)
		return {};
	EcoMap out;
	out.bytes_ = std::make_shared<std::vector<std::uint8_t>>(
			std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
	out.grid_ = geo_grid::TiledGrid::parse(*out.bytes_);
	if (!out.grid_)
		return {};
	return out;
}

std::optional<EcoMap> EcoMap::build(std::size_t world_width, std::size_t world_height,
		std::pair<std::int32_t, std::int32_t> align,
		const std::function<std::pair<double, double>(double, double)> &geo)
{
	const auto &source = generation_map().map;
	if (!source || !source->grid_ || !world_width || !world_height || !geo ||
			world_width > std::size_t(std::numeric_limits<std::int32_t>::max()) ||
			world_height > std::size_t(std::numeric_limits<std::int32_t>::max()))
		return std::nullopt;
	const auto &grid = *source->grid_;
	const double lat0 = geo(0.0, 0.0).first;
	const double lat1 = geo(0.0, double(world_height)).first;
	if (!std::isfinite(lat0) || !std::isfinite(lat1))
		return std::nullopt;
	const double source_cells =
			std::max(std::abs(lat0 - lat1) * grid.cells_per_degree(), 1e-6);
	const double per_source = double(world_height) / source_cells;
	const double quarter_estimate = per_source / 4.0;
	const int quarter = quarter_estimate <= 4.0	   ? 4
						: quarter_estimate >= 32.0 ? 32
												   : static_cast<int>(quarter_estimate);
	const auto base_cell =
			std::int32_t{1} << (31 -
								std::countl_zero(static_cast<std::uint32_t>(quarter)));
	const auto dimension = std::max(world_width, world_height);
	const auto cap_cell = static_cast<std::int32_t>((dimension + 511) / 512);
	const std::int32_t cell = std::max(base_cell, cap_cell);
	const std::int32_t ox = -euclidean_remainder(align.first, cell);
	const std::int32_t oz = -euclidean_remainder(align.second, cell);
	const auto span = [cell](std::size_t length, std::int32_t origin) {
		const auto numerator = static_cast<std::int64_t>(length) - origin + cell - 1;
		return static_cast<std::size_t>(std::max<std::int64_t>(1, numerator / cell));
	};
	const auto width = span(world_width, ox);
	const auto height = span(world_height, oz);
	if (width > std::numeric_limits<std::size_t>::max() / height)
		return std::nullopt;
	const auto count = width * height;
	std::vector<std::pair<std::size_t, std::size_t>> cells;
	cells.reserve(count);
	const double half = double(cell) / 2.0;
	std::set<std::size_t> tiles;
	for (std::size_t i = 0; i < count; ++i) {
		const double x = double(ox + static_cast<std::int32_t>(i % width) * cell) + half;
		const double z = double(oz + static_cast<std::int32_t>(i / width) * cell) + half;
		const auto [lat, lon] = geo(x, z);
		cells.push_back(warped_cell(grid, lat, lon));
		tiles.insert(grid.tile_of(cells.back()));
	}
	std::unordered_map<std::size_t, std::vector<std::uint8_t>> decoded;
	decoded.reserve(tiles.size());
	for (const auto tile : tiles)
		if (auto data = grid.decode(tile))
			decoded.emplace(tile, std::move(*data));

	EcoMap out;
	out.bytes_ = source->bytes_;
	out.grid_ = source->grid_;
	out.cell_ = cell;
	out.origin_x_ = ox;
	out.origin_z_ = oz;
	out.align_x_ = align.first;
	out.align_z_ = align.second;
	out.width_ = width;
	out.height_ = height;
	out.ids_.reserve(count);
	for (const auto &sample : cells) {
		const auto tile_id = grid.tile_of(sample);
		const auto decoded_tile = decoded.find(tile_id);
		out.ids_.push_back(
				decoded_tile == decoded.end()
						? 0
						: grid.value(decoded_tile->second, grid.index_in_tile(sample)));
	}
	return out;
}

std::uint16_t EcoMap::id_at_local(std::int32_t x, std::int32_t z) const
{
	const auto index = [](std::int32_t value, std::int32_t origin, std::int32_t cell_size,
							   std::size_t count) {
		const auto delta = std::int64_t(value) - origin;
		const auto quotient =
				delta >= 0 ? delta / cell_size : -((-delta + cell_size - 1) / cell_size);
		return static_cast<std::size_t>(std::clamp<std::int64_t>(
				quotient, 0, static_cast<std::int64_t>(count) - 1));
	};
	const auto cx = index(x, origin_x_, cell_, width_);
	const auto cz = index(z, origin_z_, cell_, height_);
	return ids_[cz * width_ + cx];
}

std::optional<Ecoregion> EcoMap::at(const cartesian::XZPoint &coord) const
{
	if (!is_local())
		return std::nullopt;
	return lookup(id_at_local(coord.x, coord.z));
}

std::optional<EcoMap> EcoMap::resample(std::size_t world_width, std::size_t world_height,
		const std::function<std::pair<std::int32_t, std::int32_t>(
				std::int32_t, std::int32_t)> &source) const
{
	if (!is_local() || !world_width || !world_height || !source)
		return std::nullopt;
	const auto width = std::max<std::size_t>(
			1, (world_width + std::size_t(cell_) - 1) / std::size_t(cell_));
	const auto height = std::max<std::size_t>(
			1, (world_height + std::size_t(cell_) - 1) / std::size_t(cell_));
	if (width > std::numeric_limits<std::size_t>::max() / height)
		return std::nullopt;
	EcoMap out;
	out.bytes_ = bytes_;
	out.grid_ = grid_;
	out.cell_ = cell_;
	out.origin_x_ = 0;
	out.origin_z_ = 0;
	out.width_ = width;
	out.height_ = height;
	out.ids_.reserve(width * height);
	for (std::size_t i = 0; i < width * height; ++i) {
		const auto x = static_cast<std::int32_t>(
				i % width * std::size_t(cell_) + std::size_t(cell_ / 2));
		const auto z = static_cast<std::int32_t>(
				i / width * std::size_t(cell_) + std::size_t(cell_ / 2));
		const auto [sx, sz] = source(x, z);
		out.ids_.push_back(id_at_local(sx + align_x_, sz + align_z_));
	}
	return out;
}

std::optional<std::uint16_t> EcoMap::id_at(double latitude, double longitude) const
{
	if (!grid_)
		return {};
	const auto position = grid_->position(latitude, longitude);
	const auto offset = warp(position.first, position.second);
	const auto cell =
			grid_->cell(position.first + offset.first, position.second + offset.second);
	auto tile = grid_->decode(grid_->tile_of(cell));
	if (!tile)
		return {};
	return grid_->value(*tile, grid_->index_in_tile(cell));
}
std::vector<std::pair<std::uint16_t, std::size_t>> EcoMap::by_area() const
{
	std::map<std::uint16_t, std::size_t> counts;
	if (is_local()) {
		for (const auto id : ids_)
			++counts[id];
		std::vector<std::pair<std::uint16_t, std::size_t>> out(
				counts.begin(), counts.end());
		std::sort(out.begin(), out.end(), [](const auto &a, const auto &b) {
			return a.second != b.second ? a.second > b.second : a.first < b.first;
		});
		return out;
	}
	if (!grid_)
		return {};
	for (std::size_t tile = 0; tile < grid_->tile_count(); ++tile) {
		auto decoded = grid_->decode(tile);
		if (!decoded)
			continue;
		for (std::size_t i = 0; i < grid_->cells_per_tile(); ++i) {
			++counts[grid_->value(*decoded, i)];
		}
	}
	std::vector<std::pair<std::uint16_t, std::size_t>> out(counts.begin(), counts.end());
	std::sort(out.begin(), out.end(), [](const auto &a, const auto &b) {
		return a.second != b.second ? a.second > b.second : a.first < b.first;
	});
	return out;
}
bool EcoMap::has_gaps() const
{
	if (is_local())
		return std::any_of(ids_.begin(), ids_.end(),
				[](std::uint16_t id) { return !lookup(id).has_value(); });
	if (!grid_)
		return true;
	for (std::size_t tile = 0; tile < grid_->tile_count(); ++tile) {
		const auto decoded = grid_->decode(tile);
		if (!decoded)
			return true;
		for (std::size_t i = 0; i < grid_->cells_per_tile(); ++i)
			if (!lookup(grid_->value(*decoded, i)))
				return true;
	}
	return false;
}
std::optional<std::string> EcoMap::dominant_tree_pack() const
{
	for (const auto &[id, count] : by_area()) {
		(void)count;
		if (const auto mix = tree_mix(id))
			return std::string(mix->first);
	}
	return std::nullopt;
}
}
