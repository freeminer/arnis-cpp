#include "ecoregion.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <mutex>
#include <sstream>
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
		std::array<std::filesystem::path, 3> paths = {
				std::filesystem::path("assets/climate/ecoregions.tsv"),
				std::filesystem::path(__FILE__).parent_path().parent_path() /
						"assets/climate/ecoregions.tsv",
				std::filesystem::current_path() / "assets/climate/ecoregions.tsv"};
		std::ifstream in;
		for (const auto &p : paths) {
			in.clear();
			in.open(p);
			if (in)
				break;
		}
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
