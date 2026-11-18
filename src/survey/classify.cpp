#include "classify.h"

#include "grid.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <vector>

namespace arnis::survey::classify
{
namespace
{
constexpr float TURN = 65535.0f;
constexpr std::size_t PAVING_REACH = 30;
constexpr double MIN_GROUND_AROUND = .4;

std::uint8_t scaled_count(float count, float density_scale)
{
	return static_cast<std::uint8_t>(std::max(1.0f, std::round(count * density_scale)));
}

bool earthy(const std::array<std::uint8_t, 3> &rgb)
{
	const int r = rgb[0], g = rgb[1], b = rgb[2];
	return r - b >= 12 && r - g <= 25;
}

std::optional<std::pair<float, float>> bands(
		const std::vector<float> &grass, const std::vector<float> &pave, float scale)
{
	const auto at = [](const std::vector<float> &values, float q) {
		return values[static_cast<std::size_t>((values.size() - 1) * q)];
	};
	const float g = at(grass, .5f), p = at(pave, .5f), gap = g - p;
	if (gap < .045f * scale)
		return std::nullopt;
	return std::pair{std::min(p + .2f * gap, at(grass, .1f)),
			std::max(p + .6f * gap, at(pave, .9f))};
}

std::vector<double> ring_planted_share(const std::vector<std::uint32_t> &labels,
		std::uint32_t count, std::size_t w, std::size_t h,
		const std::vector<std::uint8_t> &planted)
{
	std::vector<double> hit(static_cast<std::size_t>(count) + 1, 0.0);
	std::vector<double> all(hit.size(), 0.0);
	std::vector<std::uint32_t> seen(w * h, 0);
	for (std::size_t z = 0; z < h; ++z)
		for (std::size_t x = 0; x < w; ++x) {
			const auto label = labels[z * w + x];
			if (!label)
				continue;
			for (int dz = -2; dz <= 2; ++dz)
				for (int dx = -2; dx <= 2; ++dx) {
					const auto nx = static_cast<std::int64_t>(x) + dx;
					const auto nz = static_cast<std::int64_t>(z) + dz;
					if (nx < 0 || nz < 0 || nx >= static_cast<std::int64_t>(w) ||
							nz >= static_cast<std::int64_t>(h))
						continue;
					const auto i = static_cast<std::size_t>(nz) * w +
								   static_cast<std::size_t>(nx);
					if (!labels[i] && seen[i] != label) {
						seen[i] = label;
						all[label] += 1.0;
						hit[label] += planted[i] != 0;
					}
				}
		}
	for (std::size_t i = 0; i < hit.size(); ++i)
		if (all[i] > 0)
			hit[i] /= all[i];
	return hit;
}

void close_hedge_gaps(std::vector<std::uint8_t> &classes, std::vector<float> &heights,
		std::size_t w, std::size_t h)
{
	const auto snapshot = classes;
	auto at = [=](std::int64_t x, std::int64_t z) -> std::optional<std::size_t> {
		if (x < 0 || z < 0 || x >= static_cast<std::int64_t>(w) ||
				z >= static_cast<std::int64_t>(h))
			return std::nullopt;
		return static_cast<std::size_t>(z) * w + static_cast<std::size_t>(x);
	};
	for (std::int64_t z = 0; z < static_cast<std::int64_t>(h); ++z)
		for (std::int64_t x = 0; x < static_cast<std::int64_t>(w); ++x) {
			const auto c = *at(x, z);
			if (snapshot[c] != LAWN && snapshot[c] != BED && snapshot[c] != BUSH)
				continue;
			bool filled = false;
			for (const auto [dx, dz] : {std::pair<std::int64_t, std::int64_t>{1, 0},
						 {0, 1}, {1, 1}, {1, -1}}) {
				for (int gap = 1; gap <= 2 && !filled; ++gap)
					for (int back = 1; back <= 3 - gap && !filled; ++back) {
						const auto a = at(x - dx * back, z - dz * back);
						const auto b = at(x + dx * gap, z + dz * gap);
						if (a && b && snapshot[*a] == HEDGE && snapshot[*b] == HEDGE) {
							classes[c] = HEDGE;
							heights[c] = (heights[*a] + heights[*b]) * .5f;
							filled = true;
						}
					}
				if (filled)
					break;
			}
		}
}

void smooth_heights(const std::vector<std::uint8_t> &classes, std::vector<float> &heights,
		std::size_t w, std::size_t h)
{
	for (const auto kind : {HEDGE, SHED}) {
		std::vector<std::uint8_t> mask(classes.size());
		for (std::size_t i = 0; i < classes.size(); ++i)
			mask[i] = classes[i] == kind;
		auto [labels, count] = grid::label(mask, w, h, true);
		if (!count)
			continue;
		const auto medians = grid::component_median(heights, labels, count);
		for (std::size_t i = 0; i < labels.size(); ++i)
			if (labels[i])
				heights[i] = medians[labels[i]];
	}
	std::vector<std::uint8_t> bushes(classes.size());
	for (std::size_t i = 0; i < classes.size(); ++i)
		bushes[i] = classes[i] == BUSH;
	const auto smoothed = grid::median3_within(heights, bushes, w, h);
	std::size_t j = 0;
	for (std::size_t i = 0; i < classes.size(); ++i)
		if (bushes[i])
			heights[i] = std::max(2.0f, smoothed[j++]);
}

void paving_colors(std::vector<Out> &out, const Inputs &input)
{
	std::vector<std::uint8_t> mask(out.size());
	for (std::size_t i = 0; i < out.size(); ++i)
		mask[i] = out[i].cls == PAVED || out[i].cls == SHED;
	auto [labels, count] = grid::label(mask, input.w, input.h, false);
	std::vector<std::vector<std::array<std::uint8_t, 3>>> lit(count + 1), any(count + 1);
	for (std::size_t c = 0; c < labels.size(); ++c) {
		const auto label = labels[c];
		if (!label || !input.has_photo(c))
			continue;
		any[label].push_back(input.rgb(c));
		if (input.lum(c) > 70.0f)
			lit[label].push_back(input.rgb(c));
	}
	std::vector<std::array<std::uint8_t, 3>> colors(count + 1, {150, 150, 150});
	for (std::size_t label = 1; label <= count; ++label) {
		const auto &src = lit[label].size() >= 3 ? lit[label] : any[label];
		if (src.empty())
			continue;
		std::array<float, 3> median{};
		for (std::size_t channel = 0; channel < 3; ++channel) {
			std::vector<std::uint8_t> values;
			values.reserve(src.size());
			for (const auto &rgb : src)
				values.push_back(rgb[channel]);
			std::sort(values.begin(), values.end());
			median[channel] = values[values.size() / 2];
		}
		const auto [r, g, b] = median;
		if (r - b > 35.0f && r > g + 12.0f)
			colors[label] = {static_cast<std::uint8_t>(r), static_cast<std::uint8_t>(g),
					static_cast<std::uint8_t>(b)};
		else {
			const auto grey = static_cast<std::uint8_t>(std::lround((r + g + b) / 3.0f));
			colors[label] = {grey, grey, grey};
		}
	}
	for (std::size_t i = 0; i < out.size(); ++i)
		if (labels[i])
			out[i].color = colors[labels[i]];
}
} // namespace

Inputs::Inputs(std::size_t width, std::size_t height) : w(width), h(height)
{
	if (width && height <= std::numeric_limits<std::size_t>::max() / width) {
		const auto n = width * height;
		n_all.resize(n);
		n_gnd.resize(n);
		n_obj.resize(n);
		n_bld.resize(n);
		n_veg.resize(n);
		h_obj_q.resize(n);
		h_bld_q.resize(n);
		i_gnd_q.resize(n);
		photo.resize(n);
	} else {
		w = h = 0;
	}
}

float Inputs::h_obj(std::size_t c) const
{
	return h_obj_q[c] / 5.0f;
}
float Inputs::h_bld(std::size_t c) const
{
	return h_bld_q[c] / 5.0f;
}
float Inputs::i_gnd(std::size_t c) const
{
	return paving_brighter ? TURN - i_gnd_q[c] : i_gnd_q[c];
}
bool Inputs::has_photo(std::size_t c) const
{
	return photo[c].present;
}
float Inputs::exg(std::size_t c) const
{
	return photo[c].exg / 200.0f;
}
float Inputs::green_f(std::size_t c) const
{
	return photo[c].green / 25.0f;
}
float Inputs::lum(std::size_t c) const
{
	return photo[c].lum;
}
float Inputs::blue(std::size_t c) const
{
	return photo[c].blue / 200.0f;
}
const std::array<std::uint8_t, 3> &Inputs::rgb(std::size_t c) const
{
	return photo[c].rgb;
}

Counts Counts::at_density(float median_all)
{
	const float scale = std::clamp(median_all / TUNED_DENSITY, 0.0f, 1.0f);
	return {scaled_count(4.0f, scale), scaled_count(3.0f, scale),
			scaled_count(2.0f, scale), scaled_count(4.0f, scale)};
}

float median_density(const Inputs &input)
{
	std::array<std::size_t, 256> hist{};
	for (const auto value : input.n_all)
		++hist[value];
	const auto reached = std::accumulate(hist.begin() + 1, hist.end(), std::size_t{0});
	if (!reached)
		return 0.0f;
	std::size_t below = 0;
	for (std::size_t value = 1; value < hist.size(); ++value) {
		below += hist[value];
		if (below > reached / 2)
			return static_cast<float>(value);
	}
	return 255.0f;
}

std::optional<float> split_intensity(std::vector<float> &values)
{
	if (values.size() < 500)
		return std::nullopt;
	std::sort(values.begin(), values.end());
	const auto n = values.size();
	const float lo = values[n / 100], hi = values[n - 1 - n / 100];
	if (hi <= lo)
		return std::nullopt;
	constexpr std::size_t BINS = 128;
	const auto bin = [=](float value) {
		return std::min(static_cast<std::size_t>(
								(value - lo) / (hi - lo) * static_cast<float>(BINS)),
				BINS - 1);
	};
	std::array<double, BINS> hist{}, sums{};
	for (const auto value : values) {
		const float clipped = std::clamp(value, lo, hi);
		++hist[bin(clipped)];
		sums[bin(clipped)] += clipped;
	}
	const double total = std::accumulate(hist.begin(), hist.end(), 0.0);
	const double total_sum = std::accumulate(sums.begin(), sums.end(), 0.0);
	const double mean = total_sum / total;
	double variance = 0;
	for (const auto value : values) {
		const double delta = std::clamp(value, lo, hi) - mean;
		variance += delta * delta;
	}
	variance /= total;
	double weight0 = 0, sum0 = 0, best = 0, share = 0, mean0 = 0, mean1 = 0;
	std::size_t threshold = 0;
	for (std::size_t t = 0; t + 1 < BINS; ++t) {
		weight0 += hist[t];
		sum0 += sums[t];
		const double weight1 = total - weight0;
		if (!weight0 || !weight1)
			continue;
		const double m0 = sum0 / weight0, m1 = (total_sum - sum0) / weight1;
		const double between = weight0 * weight1 * (m1 - m0) * (m1 - m0) / total / total;
		if (between > best) {
			best = between;
			threshold = t;
			share = weight0 / total;
			mean0 = m0;
			mean1 = m1;
		}
	}
	const double median = values[n / 2];
	if (variance <= 0 || best / variance < .55 || share < .1 || share > .9 ||
			mean1 - mean0 < .25 * median)
		return std::nullopt;
	return lo + static_cast<float>(threshold + 1) * (hi - lo) / BINS;
}

std::pair<float, float> calibrate(
		const Inputs &input, std::pair<float, float> defaults, Calibration *method)
{
	const auto need = Counts::at_density(median_density(input));
	std::vector<float> grass, pave, all;
	for (std::size_t c = 0; c < input.w * input.h; ++c) {
		if (input.n_obj[c] || input.n_bld[c] || input.n_gnd[c] < need.flat)
			continue;
		all.push_back(input.i_gnd_q[c]);
		if (!input.has_photo(c))
			continue;
		if (input.exg(c) > .08f && input.lum(c) > 70)
			grass.push_back(input.i_gnd(c));
		else if (input.exg(c) < -.02f && input.lum(c) > 90 && input.blue(c) < .02f)
			pave.push_back(input.i_gnd(c));
	}
	if (grass.size() >= 200 && pave.size() >= 200) {
		std::sort(grass.begin(), grass.end());
		std::sort(pave.begin(), pave.end());
		const float scale = input.paving_brighter ? TURN - pave[pave.size() / 2]
												  : grass[(grass.size() - 1) / 2];
		if (auto threshold = bands(grass, pave, scale)) {
			if (method)
				*method = Calibration::Photo;
			return *threshold;
		}
	}
	auto threshold = split_intensity(all);
	const auto turn = [&](float value) {
		return input.paving_brighter ? TURN - value : value;
	};
	if (threshold) {
		if (method)
			*method = Calibration::Laser;
		return {turn(*threshold), turn(*threshold)};
	}
	if (method)
		*method = Calibration::Defaults;
	return input.paving_brighter ? std::pair{turn(defaults.second), turn(defaults.first)}
								 : defaults;
}

void crowns_not_roofs(Inputs &input)
{
	for (std::size_t c = 0; c < input.w * input.h; ++c) {
		const bool green =
				input.has_photo(c) && (input.green_f(c) >= .35f || input.exg(c) > .04f);
		if (green && input.n_bld[c]) {
			input.n_obj[c] = static_cast<std::uint8_t>(
					std::min(255, int(input.n_obj[c]) + input.n_bld[c]));
			input.h_obj_q[c] = std::max(input.h_obj_q[c], input.h_bld_q[c]);
			input.n_bld[c] = input.h_bld_q[c] = 0;
		}
	}
}

std::vector<Out> classify(const Inputs &input, std::pair<float, float> intensity_bands)
{
	const auto w = input.w, h = input.h;
	if (!w || h > std::numeric_limits<std::size_t>::max() / w ||
			input.n_all.size() != w * h || input.n_gnd.size() != w * h ||
			input.n_obj.size() != w * h || input.n_bld.size() != w * h ||
			input.n_veg.size() != w * h || input.h_obj_q.size() != w * h ||
			input.h_bld_q.size() != w * h || input.i_gnd_q.size() != w * h ||
			input.photo.size() != w * h)
		return {};
	const std::size_t n = w * h;
	std::vector<std::uint8_t> classes(n, NONE);
	std::vector<float> heights(n, 0.0f);
	const auto need = Counts::at_density(median_density(input));
	auto vegetation = [&](std::size_t c) {
		return input.n_obj[c] && 2u * input.n_veg[c] >= input.n_obj[c];
	};
	auto green_at = [&](std::size_t c) {
		if (input.has_photo(c))
			return input.green_f(c) >= .35f || input.exg(c) > .04f ||
				   (input.laser_plants && vegetation(c));
		return vegetation(c);
	};
	std::vector<std::uint8_t> green(n), building(n), object(n);
	for (std::size_t c = 0; c < n; ++c) {
		green[c] = green_at(c);
		building[c] = input.n_bld[c] >= need.bld;
		object[c] = input.n_obj[c] >= need.obj && input.h_obj(c) >= .4f && !building[c];
	}
	auto green_near = grid::dilate(green, w, h, 1);
	std::vector<std::uint8_t> planted(n), unsure(n);
	for (std::size_t c = 0; c < n; ++c) {
		const float intensity = input.i_gnd(c);
		const bool has_intensity = input.n_gnd[c] >= need.few;
		planted[c] =
				has_intensity && intensity >= intensity_bands.second
						? 1
						: (has_intensity && intensity < intensity_bands.first ? 0
																			  : green[c]);
		unsure[c] = has_intensity && input.has_photo(c) &&
					intensity >= intensity_bands.first &&
					intensity < intensity_bands.second && !green[c] &&
					earthy(input.rgb(c));
	}
	// The Rust implementation drops the per-cell green mask as soon as the
	// planted/unsure masks are formed. These full-grid temporaries are large for
	// regional survey mosaics, so release their capacity at the same lifetime
	// boundary rather than retaining them through component classification.
	std::vector<std::uint8_t>().swap(green);
	std::vector<std::uint8_t> planted_share(n, 0);
	grid::box_share(planted, w, h, 2, [&](std::size_t c, float share) {
		planted_share[c] = static_cast<std::uint8_t>(std::lround(share * 40.0f));
		if (input.n_all[c])
			classes[c] = share >= .5f ? LAWN : PAVED;
	});
	grid::box_share(unsure, w, h, 2, [&](std::size_t c, float share) {
		const float sealed = 1.0f - planted_share[c] / 40.0f - share;
		if (classes[c] == PAVED && share > sealed)
			classes[c] = BARE;
	});
	std::vector<std::uint8_t>().swap(unsure);
	std::vector<std::uint8_t>().swap(planted_share);
	if (input.paving_brighter)
		grid::box_share(building, w, h, PAVING_REACH, [&](std::size_t c, float share) {
			if (share <= 0 && classes[c] == PAVED)
				classes[c] = LAWN;
		});
	std::vector<std::uint8_t> ground_hit(n);
	for (std::size_t c = 0; c < n; ++c)
		ground_hit[c] = input.n_gnd[c] >= need.few;

	// Blue components count as pools only when laser returns are scarce and their
	// exterior has enough measured garden ground.
	{
		std::vector<std::uint8_t> blue(n);
		for (std::size_t c = 0; c < n; ++c)
			blue[c] = input.has_photo(c) && input.blue(c) > .03f && input.lum(c) > 80 &&
					  !object[c] && !building[c];
		auto [labels, count] = grid::label(blue, w, h, true);
		if (count) {
			const auto shapes = grid::shapes(labels, count, w);
			const auto grounded = ring_planted_share(labels, count, w, h, ground_hit);
			std::vector<double> wet(static_cast<std::size_t>(count) + 1, 0.0);
			const float dry_from =
					input.strict_pools ? 0.0f : .6f * median_density(input);
			for (std::size_t c = 0; c < n; ++c)
				if (labels[c] && input.n_all[c] <= dry_from)
					wet[labels[c]] += 1.0;
			for (std::size_t c = 0; c < n; ++c) {
				const auto label = labels[c];
				if (label && shapes[label].area >= 6 && shapes[label].area <= 120 &&
						wet[label] >= .4 * shapes[label].area &&
						grounded[label] >= MIN_GROUND_AROUND)
					classes[c] = POOL;
			}
		}
	}
	std::vector<std::uint8_t> tree_raw(n);
	for (std::size_t c = 0; c < n; ++c)
		tree_raw[c] = input.n_obj[c] >= need.few && input.h_obj(c) >= 3.0f &&
					  green_near[c] && !building[c];
	auto not_grown = grid::dilate(tree_raw, w, h, 1);
	std::vector<std::uint8_t>().swap(tree_raw);
	for (auto &cell : not_grown)
		cell = !cell;
	auto outside = grid::dilate(not_grown, w, h, 1);
	std::vector<std::uint8_t>().swap(not_grown);
	std::vector<std::uint8_t> tree(n);
	for (std::size_t c = 0; c < n; ++c)
		tree[c] = !outside[c] && !building[c];
	std::vector<std::uint8_t>().swap(outside);

	std::vector<std::uint8_t> tall_building(n);
	for (std::size_t c = 0; c < n; ++c)
		tall_building[c] = building[c] && input.h_bld(c) > 5.0f;
	auto near_house = grid::dilate(tall_building, w, h, 2);
	std::vector<std::uint8_t>().swap(tall_building);
	std::vector<std::uint8_t> shed_candidate(n);
	for (std::size_t c = 0; c < n; ++c) {
		const float hb = input.h_bld(c), ho = input.h_obj(c);
		shed_candidate[c] =
				((building[c] && hb >= 1.8f && hb <= 5.0f) ||
						(object[c] && !green_near[c] && ho >= 1.9f && ho <= 4.5f)) &&
				!near_house[c];
	}
	std::vector<std::uint8_t>().swap(near_house);
	auto [shed_labels, shed_count] = grid::label(shed_candidate, w, h, false);
	std::vector<std::uint8_t>().swap(shed_candidate);
	std::vector<std::uint8_t> shed(n);
	if (shed_count) {
		const auto shapes = grid::shapes(shed_labels, shed_count, w);
		const auto grounded =
				ring_planted_share(shed_labels, shed_count, w, h, ground_hit);
		for (std::size_t c = 0; c < n; ++c) {
			const auto label = shed_labels[c];
			if (label && shapes[label].area >= 5 && shapes[label].area <= 70 &&
					shapes[label].width >= 1.8 && grounded[label] >= MIN_GROUND_AROUND) {
				shed[c] = 1;
				classes[c] = SHED;
				heights[c] = (building[c] ? input.h_bld(c) : input.h_obj(c)) * 5.0f;
			}
		}
	}
	std::vector<std::uint32_t>().swap(shed_labels);
	std::vector<std::uint8_t>().swap(ground_hit);

	std::vector<std::uint8_t> lowveg(n);
	for (std::size_t c = 0; c < n; ++c)
		lowveg[c] = object[c] && green_near[c] && input.h_obj(c) < 3.5f && !shed[c] &&
					!tree[c];
	auto [low_labels, low_count] = grid::label(lowveg, w, h, true);
	if (low_count) {
		const auto shapes = grid::shapes(low_labels, low_count, w);
		const auto ring = ring_planted_share(low_labels, low_count, w, h, planted);
		std::vector<std::uint32_t> cells(low_count + 1), greens(low_count + 1),
				neutral(low_count + 1), tall(low_count + 1);
		for (std::size_t c = 0; c < n; ++c) {
			const auto label = low_labels[c];
			if (!label)
				continue;
			++cells[label];
			greens[label] += green_at(c);
			const auto &rgb = input.rgb(c);
			const auto [lo, hi] = std::minmax_element(rgb.begin(), rgb.end());
			neutral[label] += input.has_photo(c) && *hi - *lo <= 24;
			tall[label] += input.h_obj(c) >= 1.0f;
		}
		for (std::size_t c = 0; c < n; ++c) {
			const auto label = low_labels[c];
			if (!label || shapes[label].area < 2)
				continue;
			const bool parked = 4 * greens[label] < cells[label] &&
								5 * neutral[label] >= 3 * cells[label] &&
								2 * tall[label] >= cells[label] && ring[label] < .5;
			if (parked)
				continue;
			const float height = input.h_obj(c);
			classes[c] = shapes[label].length >= 5 && shapes[label].width <= 2.2
								 ? HEDGE
								 : (height < .8f ? BED : BUSH);
			heights[c] = height * 5.0f;
		}
	}
	std::vector<std::uint8_t>().swap(lowveg);
	std::vector<std::uint32_t>().swap(low_labels);
	std::vector<std::uint8_t>().swap(object);

	std::vector<std::uint8_t> fence_candidate(n);
	for (std::size_t c = 0; c < n; ++c) {
		const auto kind = classes[c];
		const auto height = input.h_obj(c);
		fence_candidate[c] = input.n_obj[c] >= need.few && height >= .7f &&
							 height <= 2.3f && !green_near[c] && !building[c] &&
							 kind != SHED && kind != HEDGE && kind != BUSH && kind != BED;
	}
	auto [fence_labels, fence_count] = grid::label(fence_candidate, w, h, true);
	if (fence_count) {
		const auto shapes = grid::shapes(fence_labels, fence_count, w);
		const auto ring = ring_planted_share(fence_labels, fence_count, w, h, planted);
		for (std::size_t c = 0; c < n; ++c) {
			const auto label = fence_labels[c];
			if (label && shapes[label].length >= 5 && shapes[label].width <= 1.4 &&
					ring[label] >= .35) {
				classes[c] = FENCE;
				heights[c] = input.h_obj(c) * 5.0f;
			}
		}
	}
	std::vector<std::uint8_t>().swap(fence_candidate);
	std::vector<std::uint32_t>().swap(fence_labels);
	for (std::size_t c = 0; c < n; ++c) {
		if (tree[c]) {
			classes[c] = TREE;
			heights[c] = std::max(input.h_obj(c), 3.0f) * 5.0f;
		}
		if (building[c] && !shed[c]) {
			classes[c] = BUILDING;
			heights[c] = input.h_bld(c) * 5.0f;
		}
	}
	std::vector<std::uint8_t>().swap(planted);
	std::vector<std::uint8_t>().swap(green_near);
	std::vector<std::uint8_t>().swap(tree);
	std::vector<std::uint8_t>().swap(shed);
	std::vector<std::uint8_t>().swap(building);
	close_hedge_gaps(classes, heights, w, h);
	smooth_heights(classes, heights, w, h);
	std::vector<Out> out(n);
	for (std::size_t c = 0; c < n; ++c) {
		out[c].cls = classes[c];
		out[c].height_q =
				static_cast<std::uint8_t>(std::clamp(std::lround(heights[c]), 0l, 255l));
	}
	paving_colors(out, input);
	return out;
}

std::vector<std::array<std::uint8_t, 5>> encode_cells(const std::vector<Out> &cells)
{
	std::vector<std::array<std::uint8_t, 5>> encoded;
	encoded.reserve(cells.size());
	for (const auto &cell : cells)
		encoded.push_back(
				{cell.cls, cell.height_q, cell.color[0], cell.color[1], cell.color[2]});
	return encoded;
}

} // namespace arnis::survey::classify
