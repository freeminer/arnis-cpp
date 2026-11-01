#include "field_texture.h"

#include "../clipping.h"
#include "../ground_generation.h"
#include "../land_cover/land_cover.h"
#include "../../../arnis_world_editor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <limits>
#include <string>

namespace arnis::field_texture
{
namespace
{
constexpr std::array<Crop, 7> CROPS{Crop::Wheat, Crop::Potatoes, Crop::Carrots,
		Crop::Beetroots, Crop::Sunflowers, Crop::Pumpkins, Crop::Fallow};
constexpr std::array<Crop, 4> TILLED{
		Crop::Wheat, Crop::Potatoes, Crop::Carrots, Crop::Beetroots};
constexpr double MIN_STRIP_BLOCKS = 4.0;
constexpr double MIN_LENGTH_BLOCKS = 12.0;
constexpr int TEXTURE_NOISE = 6;

struct Parameters
{
	double strip_min_m{};
	double strip_max_m{};
	double length_m{};
	std::array<std::uint32_t, 7> weights{};

	double strip_blocks(double scale, std::uint64_t roll) const
	{
		const double t = static_cast<double>(roll % 1000) / 1000.0;
		return std::max((strip_min_m + t * (strip_max_m - strip_min_m)) * scale,
				MIN_STRIP_BLOCKS);
	}

	Plot plot(std::uint64_t key, bool rows_along_x) const;
};

std::optional<Parameters> parameters(FieldPreset preset)
{
	switch (preset) {
	case FieldPreset::Classic:
		return std::nullopt;
	case FieldPreset::Smallholding:
		return Parameters{20.0, 45.0, 200.0, {22, 18, 18, 14, 10, 10, 8}};
	case FieldPreset::Patchwork:
		return Parameters{45.0, 100.0, 400.0, {40, 15, 15, 8, 10, 5, 7}};
	case FieldPreset::Prairie:
		return Parameters{120.0, 250.0, 1000.0, {60, 8, 6, 4, 12, 2, 8}};
	}
	return std::nullopt;
}

std::uint64_t mix(std::uint64_t value)
{
	value ^= value >> 30;
	value *= 0xBF58476D1CE4E5B9ULL;
	value ^= value >> 27;
	value *= 0x94D049BB133111EBULL;
	return value ^ (value >> 31);
}

std::uint64_t plot_key(std::uint64_t id, std::int64_t i, std::int64_t j)
{
	return mix(id ^ mix(static_cast<std::uint64_t>(i) ^
							mix(static_cast<std::uint64_t>(j) ^ 0x51EDULL)));
}

double sane_scale(double scale)
{
	return std::isfinite(scale) && scale > 0.0 ? scale : 1.0;
}

std::int64_t euclidean_div(std::int64_t value, std::int64_t divisor)
{
	const auto quotient = value / divisor;
	const auto remainder = value % divisor;
	return remainder < 0 ? quotient - 1 : quotient;
}
} // namespace

Plot Parameters::plot(std::uint64_t key, bool rows_along_x) const
{
	std::uint32_t total = 0;
	for (const auto weight : weights)
		total += weight;
	std::uint32_t roll = static_cast<std::uint32_t>(mix(key) % total);
	Crop crop = Crop::Wheat;
	for (std::size_t i = 0; i < weights.size(); ++i) {
		if (roll < weights[i]) {
			crop = CROPS[i];
			break;
		}
		roll -= weights[i];
	}
	return {crop, mix(key ^ 0xA9E3ULL) % 4 == 0, mix(key ^ 0xF10EULL), rows_along_x};
}

FieldPreset field_preset_from_string(std::string_view value)
{
	std::string normalized(value);
	std::transform(normalized.begin(), normalized.end(), normalized.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	const auto begin = normalized.find_first_not_of(" \t\r\n");
	const auto end = normalized.find_last_not_of(" \t\r\n");
	const auto trimmed = begin == std::string::npos
								 ? std::string{}
								 : normalized.substr(begin, end - begin + 1);
	if (trimmed == "classic")
		return FieldPreset::Classic;
	if (trimmed == "smallholding")
		return FieldPreset::Smallholding;
	if (trimmed == "prairie")
		return FieldPreset::Prairie;
	return FieldPreset::Patchwork;
}

std::string_view field_preset_name(FieldPreset preset)
{
	switch (preset) {
	case FieldPreset::Classic:
		return "classic";
	case FieldPreset::Smallholding:
		return "smallholding";
	case FieldPreset::Patchwork:
		return "patchwork";
	case FieldPreset::Prairie:
		return "prairie";
	}
	return "patchwork";
}

const std::array<Block, 14> PLOT_PLANTS{GRASS, FERN, DEAD_BUSH, HAY_BALE, PUMPKIN,
		SUNFLOWER_LOWER, WHEAT, WHEAT_YOUNG, CARROTS, CARROTS_YOUNG, POTATOES,
		POTATOES_YOUNG, BEETROOTS, BEETROOTS_YOUNG};

void place_cover(WorldEditor &editor, const Cover &cover, int x, int ground_y, int z)
{
	if (!cover.upper) {
		editor.set_block_absolute(cover.lower, x, ground_y + 1, z);
		return;
	}
	if (!editor.block_exists_absolute(x, ground_y + 1, z) &&
			!editor.block_exists_absolute(x, ground_y + 2, z)) {
		editor.set_block_absolute(cover.lower, x, ground_y + 1, z);
		editor.set_block_absolute(*cover.upper, x, ground_y + 2, z);
	}
}

bool Plot::on_row(int x, int z) const
{
	const int coordinate = rows_along_x ? z : x;
	return ((coordinate % 2) + 2) % 2 == 0;
}

Block Plot::surface(int x, int z) const
{
	const double n = ground_generation::value_noise_01(x, z, TEXTURE_NOISE);
	switch (crop) {
	case Crop::Wheat:
	case Crop::Potatoes:
	case Crop::Carrots:
	case Crop::Beetroots:
		return n < 0.04 ? COARSE_DIRT : FARMLAND;
	case Crop::Sunflowers:
		return n < 0.16 ? GRASS_BLOCK : COARSE_DIRT;
	case Crop::Pumpkins:
		return n < 0.42 ? COARSE_DIRT : GRASS_BLOCK;
	case Crop::Fallow:
		if (n >= 0.33 && n < 0.45)
			return MUD;
		if (n >= 0.45 && n < 0.70)
			return GRASS_BLOCK;
		return COARSE_DIRT;
	}
	return FARMLAND;
}

std::optional<Cover> Plot::cover(Block surface_block, int x, int z) const
{
	const auto roll = land_cover::coord_hash(x ^ 0x0C209A11, z) % 100;
	const auto single = [](Block block) -> std::optional<Cover> {
		return Cover{block, {}};
	};
	switch (crop) {
	case Crop::Wheat:
	case Crop::Potatoes:
	case Crop::Carrots:
	case Crop::Beetroots:
		if (surface_block == FARMLAND)
			return single(crop_block(x, z, roll));
		if (crop == Crop::Wheat && roll <= 2)
			return single(HAY_BALE);
		if (roll >= 3 && roll <= 7)
			return single(DEAD_BUSH);
		if (roll >= 8 && roll <= 20)
			return single(GRASS);
		return std::nullopt;
	case Crop::Sunflowers:
		if (surface_block == COARSE_DIRT && roll < 85 && on_row(x, z))
			return Cover{SUNFLOWER_LOWER, SUNFLOWER_UPPER};
		if (surface_block == GRASS_BLOCK && roll < 25)
			return single(GRASS);
		return std::nullopt;
	case Crop::Pumpkins:
		if (surface_block == GRASS_BLOCK) {
			if (roll <= 5)
				return single(PUMPKIN);
			if (roll <= 25)
				return single(GRASS);
			if (roll == 26)
				return single(DEAD_BUSH);
		} else if (surface_block == COARSE_DIRT && roll <= 7) {
			return single(GRASS);
		}
		return std::nullopt;
	case Crop::Fallow:
		if (surface_block == GRASS_BLOCK)
			return roll <= 19 ? single(GRASS) : std::nullopt;
		if (surface_block == MUD)
			return std::nullopt;
		if (roll <= 5)
			return single(DEAD_BUSH);
		if (roll <= 14)
			return single(GRASS);
		if (roll == 15 || roll == 16)
			return single(FERN);
		return std::nullopt;
	}
	return std::nullopt;
}

Block Plot::crop_block(int x, int z, std::uint64_t roll) const
{
	const Crop selected =
			ground_generation::value_noise_01(x + 321, z - 777, 4) < 0.022
					? TILLED[static_cast<std::size_t>((seed >> 9) % TILLED.size())]
					: crop;
	const bool is_young = young || roll < 15;
	switch (selected) {
	case Crop::Wheat:
		return is_young ? WHEAT_YOUNG : WHEAT;
	case Crop::Potatoes:
		return is_young ? POTATOES_YOUNG : POTATOES;
	case Crop::Carrots:
		return is_young ? CARROTS_YOUNG : CARROTS;
	default:
		return is_young ? BEETROOTS_YOUNG : BEETROOTS;
	}
}

std::optional<FieldLayout> FieldLayout::make(FieldPreset preset, double scale,
		std::uint64_t id, const std::vector<ProcessedNode> &nodes)
{
	const auto params = parameters(preset);
	if (!params)
		return std::nullopt;
	scale = sane_scale(scale);
	double c = 0.0, s = 0.0;
	for (std::size_t i = 1; i < nodes.size(); ++i) {
		if (clipping::is_invented_node_id(nodes[i - 1].id) &&
				clipping::is_invented_node_id(nodes[i].id))
			continue;
		const double dx = static_cast<double>(nodes[i].x) - nodes[i - 1].x;
		const double dz = static_cast<double>(nodes[i].z) - nodes[i - 1].z;
		const double segment = std::hypot(dx, dz);
		if (segment >= 1.0) {
			const double angle = 4.0 * std::atan2(dz, dx);
			c += segment * std::cos(angle);
			s += segment * std::sin(angle);
		}
	}
	const double angle = std::atan2(s, c) / 4.0;
	const double along_x = std::cos(angle), along_z = std::sin(angle);
	const double across_x = -along_z, across_z = along_x;
	auto extent = [&nodes](double dx, double dz) {
		std::pair<double, double> range{
				std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()};
		for (const auto &node : nodes) {
			const double p = dx * node.x + dz * node.z;
			range.first = std::min(range.first, p);
			range.second = std::max(range.second, p);
		}
		return range;
	};
	const auto eu = extent(along_x, along_z);
	const auto ev = extent(across_x, across_z);
	if (nodes.empty())
		return std::nullopt;
	const bool use_u = eu.second - eu.first >= ev.second - ev.first;
	const double ax = use_u ? along_x : across_x;
	const double az = use_u ? along_z : across_z;
	const double cx = use_u ? across_x : along_x;
	const double cz = use_u ? across_z : along_z;
	const auto ea = use_u ? eu : ev;
	const auto ec = use_u ? ev : eu;
	const double width = params->strip_blocks(scale, mix(id ^ 0x57171D7AULL));
	const double length = std::max(params->length_m * scale, MIN_LENGTH_BLOCKS);
	const auto n_across = std::max<std::int64_t>(
			1, static_cast<std::int64_t>(std::round((ec.second - ec.first) / width)));
	const auto n_along = std::max<std::int64_t>(
			1, static_cast<std::int64_t>(std::round((ea.second - ea.first) / length)));
	FieldLayout out;
	out.weights_ = params->weights;
	out.strip_min_m_ = params->strip_min_m;
	out.strip_max_m_ = params->strip_max_m;
	out.length_m_ = params->length_m;
	out.id_ = id;
	out.across_x_ = cx;
	out.across_z_ = cz;
	out.along_x_ = ax;
	out.along_z_ = az;
	out.across_min_ = ec.first;
	out.along_min_ = ea.first;
	out.strip_ = std::max((ec.second - ec.first) / static_cast<double>(n_across), 1.0);
	out.length_ = std::max((ea.second - ea.first) / static_cast<double>(n_along), 1.0);
	out.n_across_ = n_across;
	out.n_along_ = n_along;
	out.rows_along_x_ = std::abs(ax) >= std::abs(az);
	return out;
}

Plot FieldLayout::plot_at(int x, int z) const
{
	const double across = across_x_ * x + across_z_ * z - across_min_;
	const double along = along_x_ * x + along_z_ * z - along_min_;
	const auto i = std::clamp<std::int64_t>(
			static_cast<std::int64_t>(std::floor(across / strip_)), 0, n_across_ - 1);
	std::int64_t j = 0;
	if (n_along_ > 1) {
		double shift;
		if (last_strip_ && last_strip_->first == i) {
			shift = last_strip_->second;
		} else {
			const double t = static_cast<double>(
					mix(id_ ^ (static_cast<std::uint64_t>(i) * 0x9E37ULL)) % 1000);
			shift = (0.3 + 0.4 * t / 1000.0) * length_;
			last_strip_ = std::pair{i, shift};
		}
		j = std::clamp<std::int64_t>(
				static_cast<std::int64_t>(std::floor((along + shift) / length_)), 0,
				n_along_);
	}
	if (last_plot_ && last_plot_->first == std::pair{i, j})
		return last_plot_->second;
	Parameters params{strip_min_m_, strip_max_m_, length_m_, weights_};
	const auto plot = params.plot(plot_key(id_, i, j), rows_along_x_);
	last_plot_ = std::pair{std::pair{i, j}, plot};
	return plot;
}

std::optional<CroplandGrid> CroplandGrid::make(FieldPreset preset, double scale)
{
	const auto params = parameters(preset);
	if (!params)
		return std::nullopt;
	scale = sane_scale(scale);
	CroplandGrid out;
	out.weights_ = params->weights;
	out.strip_min_m_ = params->strip_min_m;
	out.strip_max_m_ = params->strip_max_m;
	out.length_m_ = params->length_m;
	out.scale_ = scale;
	out.block_ = static_cast<int>(
			std::round(std::max(params->length_m * scale, MIN_LENGTH_BLOCKS)));
	return out;
}

Plot CroplandGrid::plot_at(int x, int z) const
{
	const auto key = std::pair{static_cast<int>(euclidean_div(x, block_)),
			static_cast<int>(euclidean_div(z, block_))};
	StripBlock block;
	if (last_block_ && last_block_->key == key) {
		block = *last_block_;
	} else {
		const auto hash = plot_key(0xC0FFEE11ULL, key.first, key.second);
		Parameters params{strip_min_m_, strip_max_m_, length_m_, weights_};
		const int width =
				static_cast<int>(std::round(params.strip_blocks(scale_, hash >> 8)));
		block = {key, hash, (hash & 1) == 0, width,
				static_cast<int>((hash >> 32) % static_cast<std::uint64_t>(width))};
		last_block_ = block;
	}
	const int across = block.rows_along_x ? z : x;
	const int strip = static_cast<int>(
			euclidean_div(static_cast<std::int64_t>(across) + block.phase, block.width));
	const auto plot_key_pair = std::pair{block.hash, strip};
	if (last_plot_ && last_plot_->first == plot_key_pair)
		return last_plot_->second;
	Parameters params{strip_min_m_, strip_max_m_, length_m_, weights_};
	const auto plot = params.plot(plot_key(block.hash, strip, 0), block.rows_along_x);
	last_plot_ = std::pair{plot_key_pair, plot};
	return plot;
}

} // namespace arnis::field_texture
