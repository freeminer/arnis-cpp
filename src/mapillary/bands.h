#pragma once
#include "../block_palette.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <cstdint>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>
namespace arnis::mapillary::bands
{
inline arnis::Block block_for(const std::array<std::uint8_t, 3> &color)
{
	return arnis::block_palette::closest_block(RGBTuple{color[0], color[1], color[2]});
}
inline constexpr double band_tau = .05;
inline constexpr std::size_t band_max = 5;
inline constexpr std::uint8_t cls_window = 192;
inline constexpr std::uint8_t cls_door = 128;
inline constexpr double floor_share = .2;
inline constexpr double col_share = .25;
struct Band
{
	std::size_t first{}, last{};
	std::optional<std::array<double, 3>> lab;
	std::optional<std::array<std::uint8_t, 3>> rgb;
	std::optional<std::array<std::uint8_t, 3>> window_rgb;
	std::size_t rows_with_wall{};
	std::size_t height() const { return last >= first ? last - first + 1 : 0; }
};
struct Lattice
{
	std::optional<std::size_t> period;
	std::size_t width{}, phase{};
	double score{}, on_mean{}, coverage{};
	bool accepted{};
	std::vector<bool> model_columns, window_columns;
	std::vector<bool> floor_rows;
	std::optional<std::size_t> row_period;
	double row_score{};
	std::vector<double> p, q;
	std::vector<std::size_t> model_column_indices() const
	{
		std::vector<std::size_t> result;
		for (std::size_t i = 0; i < model_columns.size(); ++i)
			if (model_columns[i])
				result.push_back(i);
		return result;
	}
	std::vector<std::size_t> window_column_indices() const
	{
		std::vector<std::size_t> result;
		for (std::size_t i = 0; i < window_columns.size(); ++i)
			if (window_columns[i])
				result.push_back(i);
		return result;
	}
	std::vector<std::size_t> floor_row_indices() const
	{
		std::vector<std::size_t> result;
		for (std::size_t i = 0; i < floor_rows.size(); ++i)
			if (floor_rows[i])
				result.push_back(i);
		return result;
	}
};
inline Lattice column_lattice(const std::vector<double> &share)
{
	Lattice result;
	result.model_columns.assign(share.size(), false);
	if (share.empty())
		return result;
	double total = 0;
	for (double value : share)
		total += value;
	for (std::size_t period = 2; period < 10; ++period)
		for (std::size_t width = 1; width <= 3 && width < period; ++width)
			for (std::size_t phase = 0; phase < period; ++phase) {
				std::vector<bool> on(share.size());
				double in = 0, out = 0;
				std::size_t ni = 0, no = 0;
				for (std::size_t i = 0; i < share.size(); ++i) {
					on[i] = ((i + period - phase) % period) < width;
					if (on[i]) {
						in += share[i];
						++ni;
					} else {
						out += share[i];
						++no;
					}
				}
				double mean = ni ? in / ni : 0, score = mean - (no ? out / no : 0);
				const double rounded_score = std::round(score * 1e6) / 1e6;
				const double best_rounded = std::round(result.score * 1e6) / 1e6;
				const bool better =
						rounded_score > best_rounded ||
						(rounded_score == best_rounded &&
								(width < result.width ||
										(width == result.width &&
												period < result.period.value_or(
																 period + 1))));
				if (better)
					result = {period, width, phase, rounded_score, mean, 0, false,
							std::move(on), {}};
			}
	double explained = 0;
	for (std::size_t i = 0; i < share.size(); ++i)
		if (result.model_columns[i])
			explained += share[i];
	result.coverage = total > 0 ? explained / total : 0;
	result.accepted =
			result.score >= .15 && result.on_mean >= .25 && result.coverage >= .7;
	result.window_columns = result.model_columns;
	return result;
}
inline void find_floor_rows(Lattice &lattice, const std::vector<double> &share)
{
	if (share.empty())
		return;
	double best = -std::numeric_limits<double>::infinity();
	for (std::size_t period = 2; period < 6; ++period)
		for (std::size_t phase = 0; phase < period; ++phase) {
			double on = 0, off = 0;
			std::size_t ni = 0, no = 0;
			for (std::size_t row = 0; row < share.size(); ++row)
				if ((row + period - phase) % period == 0) {
					on += share[row];
					++ni;
				} else {
					off += share[row];
					++no;
				}
			double score = (ni ? on / ni : 0) - (no ? off / no : 0);
			if (score > best) {
				best = score;
				lattice.row_period = period;
				lattice.row_score = score;
				lattice.floor_rows.assign(share.size(), false);
				for (std::size_t row = phase; row < share.size(); row += period)
					lattice.floor_rows[row] = true;
			}
		}
}

inline Lattice find_lattice(const std::vector<std::uint8_t> &classes,
		const std::vector<bool> &observed, std::size_t rows, std::size_t cols)
{
	Lattice result;
	result.model_columns.assign(cols, false);
	result.window_columns.assign(cols, false);
	result.floor_rows.assign(rows, false);
	result.p.assign(cols, 0.0);
	result.q.assign(rows, 0.0);
	if (rows == 0 || cols == 0 || classes.size() < rows * cols ||
			observed.size() < rows * cols)
		return result;
	for (std::size_t c = 0; c < cols; ++c) {
		std::size_t n = 0, windows = 0;
		for (std::size_t r = 0; r < rows; ++r) {
			const auto i = r * cols + c;
			if (observed[i]) {
				++n;
				windows += classes[i] == cls_window;
			}
		}
		if (n)
			result.p[c] = static_cast<double>(windows) / n;
	}
	for (std::size_t r = 0; r < rows; ++r) {
		std::size_t n = 0, windows = 0;
		for (std::size_t c = 0; c < cols; ++c) {
			const auto i = r * cols + c;
			if (observed[i]) {
				++n;
				windows += classes[i] == cls_window;
			}
		}
		if (n)
			result.q[r] = static_cast<double>(windows) / n;
		result.floor_rows[r] = result.q[r] >= floor_share;
	}
	result = [&] {
		Lattice lattice = column_lattice(result.p);
		lattice.p = result.p;
		lattice.q = result.q;
		lattice.floor_rows = result.floor_rows;
		return lattice;
	}();
	for (std::size_t c = 0; c < cols; ++c)
		result.window_columns[c] = result.p[c] >= col_share;
	if (result.accepted)
		for (std::size_t c = 0; c < cols; ++c)
			result.window_columns[c] =
					result.window_columns[c] || result.model_columns[c];
	const auto initial_floor_rows = result.floor_rows;
	find_floor_rows(result, result.q);
	if (result.row_score < .15 || result.row_period == std::nullopt) {
		result.floor_rows = initial_floor_rows;
	} else {
		const auto model_rows = result.floor_rows;
		result.floor_rows = initial_floor_rows;
		double on_sum = 0.0;
		std::size_t on_count = 0;
		for (std::size_t r = 0; r < result.q.size(); ++r)
			if (model_rows[r] && result.q[r] >= .08) {
				result.floor_rows[r] = true;
				on_sum += result.q[r];
				++on_count;
			}
		if (!on_count || on_sum / on_count < .2)
			result.floor_rows = initial_floor_rows;
	}
	return result;
}

inline std::vector<std::uint8_t> doors_and_shopfronts(
		const std::vector<std::uint8_t> &classes, std::size_t rows, std::size_t cols)
{
	std::vector<std::uint8_t> result = classes;
	if (result.size() < rows * cols)
		return result;
	for (std::size_t row = 0; row < rows; ++row) {
		std::size_t column = 0;
		while (column < cols) {
			if (result[row * cols + column] != cls_door) {
				++column;
				continue;
			}
			std::size_t end = column;
			while (end < cols && result[row * cols + end] == cls_door)
				++end;
			if (end - column >= 3)
				for (std::size_t c = column; c < end; ++c)
					result[row * cols + c] = cls_window;
			column = end;
		}
	}
	return result;
}

inline std::tuple<std::vector<std::uint8_t>, std::vector<bool>, std::vector<bool>>
complete_windows(const std::vector<std::uint8_t> &classes,
		const std::vector<bool> &observed, const Lattice &lattice,
		const std::optional<std::vector<double>> &evidence, std::size_t rows,
		std::size_t cols)
{
	constexpr double shop_share = .6, support = .5, min_evidence = .12;
	std::vector<std::uint8_t> out = classes;
	std::vector<bool> added(rows * cols, false), removed(rows * cols, false);
	if (rows == 0 || cols == 0 || out.size() < rows * cols ||
			observed.size() < rows * cols)
		return {std::move(out), std::move(added), std::move(removed)};
	std::vector<bool> windows(rows * cols), doors(rows * cols);
	for (std::size_t i = 0; i < rows * cols; ++i) {
		windows[i] = classes[i] == cls_window;
		doors[i] = classes[i] == cls_door;
	}
	// A one-cell gap in a glassy ground-floor/shop row is glass.
	for (std::size_t r = rows > 4 ? rows - 4 : 0; r < rows; ++r)
		if (r < lattice.q.size() && lattice.q[r] >= shop_share)
			for (std::size_t c = 1; c + 1 < cols; ++c) {
				const auto i = r * cols + c;
				if (!windows[i] && windows[i - 1] && windows[i + 1] &&
						(out[i] == 255 || out[i] == 64)) {
					out[i] = cls_window;
					added[i] = true;
				}
			}
	std::vector<bool> floor_rows(rows, false);
	for (std::size_t r = 0; r < rows; ++r)
		floor_rows[r] = r < lattice.rows_on.size() && lattice.rows_on[r] &&
						r < lattice.q.size() && lattice.q[r] < shop_share;
	std::vector<std::pair<std::size_t, std::size_t>> groups;
	for (std::size_t r = 0; r < rows;) {
		if (!floor_rows[r]) {
			++r;
			continue;
		}
		std::size_t end = r;
		while (end + 1 < rows && floor_rows[end + 1])
			++end;
		groups.emplace_back(r, end);
		r = end + 1;
	}
	if (groups.size() < 2)
		return {std::move(out), std::move(added), std::move(removed)};
	std::vector<std::vector<bool>> has(groups.size(), std::vector<bool>(cols));
	std::vector<std::vector<bool>> observed_floor(groups.size(), std::vector<bool>(cols));
	for (std::size_t f = 0; f < groups.size(); ++f)
		for (std::size_t c = 0; c < cols; ++c)
			for (std::size_t r = groups[f].first; r <= groups[f].second; ++r) {
				const auto i = r * cols + c;
				has[f][c] = has[f][c] || windows[i];
				observed_floor[f][c] = observed_floor[f][c] || observed[i];
			}
	std::vector<bool> on_column(cols, false);
	for (std::size_t c = 0; c < cols; ++c) {
		std::size_t floors = 0;
		for (const auto &floor : has)
			floors += floor[c];
		on_column[c] = floors >= 2;
		if (lattice.accepted && c < lattice.model_columns.size())
			on_column[c] = on_column[c] || lattice.model_columns[c];
	}
	if (std::count(on_column.begin(), on_column.end(), true) < 2)
		return {std::move(out), std::move(added), std::move(removed)};
	std::vector<double> floor_support(groups.size()), column_support(cols);
	for (std::size_t f = 0; f < groups.size(); ++f) {
		std::size_t selected = 0, present = 0;
		for (std::size_t c = 0; c < cols; ++c)
			if (on_column[c] && observed_floor[f][c]) {
				++selected;
				present += has[f][c];
			}
		floor_support[f] = selected ? static_cast<double>(present) / selected : 0.0;
	}
	for (std::size_t c = 0; c < cols; ++c) {
		std::size_t selected = 0, present = 0;
		for (std::size_t f = 0; f < groups.size(); ++f)
			if (observed_floor[f][c]) {
				++selected;
				present += has[f][c];
			}
		column_support[c] = selected ? static_cast<double>(present) / selected : 0.0;
	}
	for (std::size_t f = 0; f < groups.size(); ++f) {
		std::size_t active = 0;
		for (std::size_t c = 0; c < cols; ++c)
			active += on_column[c] && has[f][c];
		if (active < 3)
			continue;
		std::vector<std::pair<std::pair<std::size_t, std::size_t>, std::size_t>> extents;
		for (std::size_t c = 0; c < cols; ++c) {
			if (!has[f][c])
				continue;
			std::size_t top = groups[f].first;
			while (top <= groups[f].second && !windows[top * cols + c])
				++top;
			if (top > groups[f].second)
				continue;
			std::size_t bottom = top;
			while (top > 0 && windows[(top - 1) * cols + c])
				--top;
			while (bottom + 1 < rows && windows[(bottom + 1) * cols + c])
				++bottom;
			const std::pair<std::size_t, std::size_t> extent{top, bottom};
			auto it = std::find_if(extents.begin(), extents.end(),
					[&](const auto &v) { return v.first == extent; });
			if (it == extents.end())
				extents.push_back({extent, 1});
			else
				++it->second;
		}
		std::pair<std::size_t, std::size_t> use{groups[f].first, groups[f].second};
		std::size_t best_count = 0;
		for (const auto &candidate : extents)
			if (candidate.second > best_count) {
				best_count = candidate.second;
				use = candidate.first;
			}
		for (std::size_t c = 0; c < cols; ++c) {
			if (!on_column[c] || has[f][c])
				continue;
			bool has_door = false, left = false, right = false, seen = false;
			for (std::size_t r = groups[f].first; r <= groups[f].second; ++r) {
				const auto i = r * cols + c;
				has_door = has_door || doors[i];
				seen = seen || observed[i];
			}
			for (std::size_t r = use.first; r <= use.second; ++r) {
				left = left || (c > 0 && windows[r * cols + c - 1]);
				right = right || (c + 1 < cols && windows[r * cols + c + 1]);
			}
			if (has_door || left || right || floor_support[f] < support ||
					column_support[c] < support)
				continue;
			if (seen && evidence && evidence->size() >= rows * cols) {
				double mean = 0.0;
				for (std::size_t r = use.first; r <= use.second; ++r)
					mean += (*evidence)[r * cols + c];
				mean /= use.second - use.first + 1;
				if (mean < min_evidence)
					continue;
			} else if (!seen) {
				std::size_t floor_seen = 0, column_seen = 0;
				for (std::size_t cc = 0; cc < cols; ++cc)
					floor_seen += has[f][cc] && observed_floor[f][cc];
				for (std::size_t ff = 0; ff < groups.size(); ++ff)
					column_seen += has[ff][c] && observed_floor[ff][c];
				if (floor_seen < 2 || column_seen < 2)
					continue;
			}
			for (std::size_t r = use.first; r <= use.second; ++r) {
				const auto i = r * cols + c;
				if (out[i] == 255 || out[i] == 64) {
					out[i] = cls_window;
					added[i] = true;
				}
			}
		}
	}
	return {std::move(out), std::move(added), std::move(removed)};
}

inline std::array<std::uint8_t, 3> median_rgb(
		const std::vector<std::array<std::uint8_t, 3>> &rgb,
		const std::vector<std::size_t> &indices, std::array<std::uint8_t, 3> fallback)
{
	if (indices.empty())
		return fallback;
	std::array<std::uint8_t, 3> result{};
	for (unsigned channel = 0; channel < 3; ++channel) {
		std::vector<std::uint8_t> values;
		for (const auto index : indices)
			if (index < rgb.size())
				values.push_back(rgb[index][channel]);
		if (values.empty())
			result[channel] = fallback[channel];
		else {
			std::nth_element(
					values.begin(), values.begin() + values.size() / 2, values.end());
			result[channel] = values[values.size() / 2];
		}
	}
	return result;
}

inline std::pair<std::vector<std::array<std::uint8_t, 3>>, std::array<std::uint8_t, 3>>
apply_bands(const std::vector<std::array<std::uint8_t, 3>> &rgb,
		const std::vector<std::uint8_t> &classes, std::vector<Band> &bands,
		std::size_t cols)
{
	constexpr std::uint8_t wall = 255, unknown = 64, window = 192, door = 128;
	constexpr std::array<std::uint8_t, 3> window_fallback{60, 65, 75};
	constexpr std::array<std::uint8_t, 3> door_fallback{70, 50, 35};
	std::vector<std::array<std::uint8_t, 3>> output = rgb;
	std::vector<std::size_t> windows, doors;
	for (std::size_t i = 0; i < classes.size(); ++i) {
		if (classes[i] == window)
			windows.push_back(i);
		else if (classes[i] == door)
			doors.push_back(i);
	}
	const auto all_windows = median_rgb(rgb, windows, window_fallback);
	const auto door_color = median_rgb(rgb, doors, door_fallback);
	if (!cols)
		return {std::move(output), door_color};
	for (auto &band : bands) {
		const auto begin = band.first * cols;
		const auto end = std::min((band.last + 1) * cols, classes.size());
		std::vector<std::size_t> band_windows;
		for (std::size_t i = begin; i < end; ++i) {
			if ((classes[i] == wall || classes[i] == unknown) && band.rgb)
				output[i] = *band.rgb;
			if (classes[i] == window)
				band_windows.push_back(i);
		}
		band.window_rgb = median_rgb(rgb, band_windows, all_windows);
		for (const auto i : band_windows)
			output[i] = *band.window_rgb;
	}
	for (const auto i : doors)
		if (i < output.size())
			output[i] = door_color;
	return {std::move(output), door_color};
}

inline std::tuple<std::vector<std::uint8_t>, std::size_t, std::size_t> drop_sky(
		const std::vector<std::array<std::uint8_t, 3>> &rgb,
		const std::vector<std::uint8_t> &classes, std::size_t rows, std::size_t cols);
inline std::vector<Band> segment(
		const std::vector<std::optional<std::array<double, 3>>> &rows,
		const std::vector<double> &weights, double tau);

struct Structure
{
	std::size_t rows{}, cols{};
	std::vector<Band> bands;
	Lattice lattice;
	std::vector<std::uint8_t> classes;
	std::vector<std::array<std::uint8_t, 3>> rgb;
	std::vector<bool> added, removed;
	std::array<std::uint8_t, 3> door_rgb{70, 50, 35};
	std::size_t sky_rows{}, sky_cells{};
};

// Cell-grid counterpart of Rust's bands::analyse.  Texture/Openings callers
// can feed their already classified cells here; all later decisions are then
// shared with the no-texture path rather than being reimplemented by exporters.
inline Structure analyse_cells(std::vector<std::array<std::uint8_t, 3>> rgb,
		std::vector<std::uint8_t> classes, std::size_t rows, std::size_t cols,
		const std::vector<bool> *observed_input = nullptr, double tau = band_tau)
{
	Structure result;
	result.rows = rows;
	result.cols = cols;
	if (rows == 0 || cols == 0 || rgb.size() < rows * cols ||
			classes.size() < rows * cols)
		return result;
	std::vector<bool> observed(rows * cols, true);
	for (std::size_t i = 0; i < rows * cols; ++i)
		observed[i] = (!observed_input || i >= observed_input->size() ||
							  (*observed_input)[i]) &&
					  classes[i] != 0;
	const auto sky = drop_sky(rgb, classes, rows, cols);
	classes = std::get<0>(sky);
	result.sky_rows = std::get<1>(sky);
	result.sky_cells = std::get<2>(sky);
	classes = doors_and_shopfronts(classes, rows, cols);
	for (std::size_t i = 0; i < rows * cols; ++i)
		observed[i] = observed[i] && classes[i] != 0;
	std::vector<std::optional<std::array<double, 3>>> row_lab(rows);
	std::vector<double> row_weight(rows, 0.0);
	// A normalized RGB row median is sufficient for the C++ cell path; texture
	// callers that need full OkLab segmentation can still provide precomputed
	// rows through segment().
	for (std::size_t row = 0; row < rows; ++row) {
		std::array<double, 3> sum{};
		std::size_t count = 0;
		for (std::size_t col = 0; col < cols; ++col) {
			const auto i = row * cols + col;
			if (observed[i] && (classes[i] == 255 || classes[i] == 64)) {
				for (unsigned channel = 0; channel < 3; ++channel)
					sum[channel] += rgb[i][channel] / 255.0;
				++count;
			}
		}
		if (count) {
			for (double &value : sum)
				value /= count;
			row_lab[row] = sum;
			row_weight[row] = static_cast<double>(count) / cols;
		}
	}
	result.bands = segment(row_lab, row_weight, tau);
	result.lattice = find_lattice(classes, observed, rows, cols);
	const auto completed =
			complete_windows(classes, observed, result.lattice, std::nullopt, rows, cols);
	result.classes = std::get<0>(completed);
	result.added = std::get<1>(completed);
	result.removed = std::get<2>(completed);
	const auto colored = apply_bands(rgb, result.classes, result.bands, cols);
	result.rgb = colored.first;
	result.door_rgb = colored.second;
	return result;
}

// Remove sky-colored cells from the over-tall top of a facade texture.  The
// Rust implementation uses OkLab; this compact RGB form preserves its two
// important invariants (blue-dominant bright cells and eave-shadow repair)
// without making the bands header depend on the image-processing module.
inline std::tuple<std::vector<std::uint8_t>, std::size_t, std::size_t> drop_sky(
		const std::vector<std::array<std::uint8_t, 3>> &rgb,
		const std::vector<std::uint8_t> &classes, std::size_t rows, std::size_t cols)
{
	constexpr std::uint8_t nodata = 0, wall = 255, unknown = 64, window = 192;
	std::vector<std::uint8_t> out = classes;
	if (rows < 4 || cols == 0 || rgb.size() < rows * cols || out.size() < rows * cols)
		return {std::move(out), 0, 0};
	const std::size_t lower = rows / 2;
	std::array<double, 3> body{};
	std::size_t body_count = 0;
	for (std::size_t i = lower * cols; i < rows * cols; ++i)
		if (classes[i] == wall || classes[i] == unknown) {
			for (unsigned channel = 0; channel < 3; ++channel)
				body[channel] += rgb[i][channel] / 255.0;
			++body_count;
		}
	if (!body_count)
		return {std::move(out), 0, 0};
	for (double &channel : body)
		channel /= body_count;
	auto sky_like = [&](std::size_t i) {
		const auto &p = rgb[i];
		const double red = p[0] / 255.0, green = p[1] / 255.0, blue = p[2] / 255.0;
		const double brightness = (red + green + blue) / 3.0;
		return blue > red + .10 && blue > green + .06 && brightness > .58 &&
			   blue > body[2] + .06;
	};
	std::size_t dropped_rows = 0, dropped_cells = 0;
	for (std::size_t row = 0; row + 3 < rows; ++row) {
		bool has_wall = false, blue_row = true;
		for (std::size_t c = 0; c < cols; ++c) {
			const auto i = row * cols + c;
			has_wall = has_wall || classes[i] == wall || classes[i] == unknown;
			blue_row = blue_row && sky_like(i);
		}
		if (!has_wall) {
			if (dropped_rows == row)
				++dropped_rows;
			else
				break;
		} else if (blue_row && dropped_rows == row) {
			for (std::size_t c = 0; c < cols; ++c) {
				if (classes[row * cols + c] != nodata)
					++dropped_cells;
				out[row * cols + c] = nodata;
			}
			++dropped_rows;
		} else {
			break;
		}
	}
	for (std::size_t row = 0; row < rows; ++row)
		for (std::size_t c = 0; c < cols; ++c) {
			const auto i = row * cols + c;
			const auto &p = rgb[i];
			const bool strong = p[2] > p[0] + 18 && p[2] > p[1] + 10 && p[0] > 178 &&
								classes[i] != nodata;
			const bool zone = row < std::max<std::size_t>(1, rows / 3) ||
							  (row < lower && (c < 2 || c + 2 >= cols));
			if (strong && zone) {
				out[i] = nodata;
				++dropped_cells;
			}
		}
	// A nearly all-window top row over a non-window row is an eave shadow.
	for (std::size_t row = dropped_rows;
			row + 1 < rows && row < std::max<std::size_t>(1, rows / 3); ++row) {
		std::size_t observed = 0, top_windows = 0, below = 0, below_windows = 0;
		for (std::size_t c = 0; c < cols; ++c) {
			const auto top = out[row * cols + c], next = out[(row + 1) * cols + c];
			observed += top != nodata;
			top_windows += top == window;
			below += next != nodata;
			below_windows += next == window;
		}
		if (observed && below && static_cast<double>(top_windows) / observed > .8 &&
				static_cast<double>(below_windows) / below < .3)
			for (std::size_t c = 0; c < cols; ++c)
				if (out[row * cols + c] == window)
					out[row * cols + c] = wall;
	}
	return {std::move(out), dropped_rows, dropped_cells};
}
inline std::vector<Band> segment(
		const std::vector<std::optional<std::array<double, 3>>> &rows,
		const std::vector<double> &weights, double tau = band_tau)
{
	std::size_t n = rows.size();
	if (weights.size() != n)
		throw std::invalid_argument("band row weights must match row colours");
	if (!n)
		return {};
	std::vector<double> s0(n + 1);
	std::vector<std::array<double, 3>> s1(n + 1), s2(n + 1);
	for (std::size_t i = 0; i < n; ++i) {
		double w = rows[i] ? weights[i] : 0;
		s0[i + 1] = s0[i] + w;
		for (unsigned c = 0; c < 3; ++c) {
			double scale = c ? 1 : .5, x = (rows[i] ? (*rows[i])[c] : 0) * scale;
			s1[i + 1][c] = s1[i][c] + w * x;
			s2[i + 1][c] = s2[i][c] + w * x * x;
		}
	}
	auto cost = [&](std::size_t a, std::size_t b) {
		double w = s0[b + 1] - s0[a], v = 0;
		if (w <= 1e-9)
			return v;
		for (unsigned c = 0; c < 3; ++c) {
			double x = s1[b + 1][c] - s1[a][c];
			v += s2[b + 1][c] - s2[a][c] - x * x / w;
		}
		return std::max(0., v);
	};
	std::size_t max = std::min(band_max, n);
	std::vector<std::vector<std::size_t>> back(max + 1, std::vector<std::size_t>(n + 1));
	std::vector<double> previous(n + 1, std::numeric_limits<double>::infinity());
	previous[0] = 0;
	std::vector<double> totals(max + 1);
	for (std::size_t k = 1; k <= max; ++k) {
		std::vector<double> current(n + 1, std::numeric_limits<double>::infinity());
		for (std::size_t j = 1; j <= n; ++j)
			for (std::size_t i = 0; i < j; ++i)
				if (std::isfinite(previous[i]) &&
						previous[i] + cost(i, j - 1) < current[j]) {
					current[j] = previous[i] + cost(i, j - 1);
					back[k][j] = i;
				}
		totals[k] = current[n];
		previous = std::move(current);
	}
	std::size_t best = 1;
	for (std::size_t k = 2; k <= max; ++k)
		if (totals[k] + 1.5 * tau * tau * k < totals[best] + 1.5 * tau * tau * best)
			best = k;
	std::vector<Band> out;
	for (std::size_t j = n, k = best; k; --k) {
		std::size_t i = back[k][j];
		double w = s0[j] - s0[i];
		std::optional<std::array<double, 3>> lab;
		if (w > 1e-9) {
			std::array<double, 3> x{};
			for (unsigned c = 0; c < 3; ++c)
				x[c] = (s1[j][c] - s1[i][c]) / w / (c ? 1 : .5);
			lab = x;
		}
		std::size_t observed = 0;
		for (std::size_t r = i; r < j; ++r)
			observed += bool(rows[r]);
		out.push_back({i, j - 1, lab, std::nullopt, std::nullopt, observed});
		j = i;
	}
	std::reverse(out.begin(), out.end());
	// bands.rs::segment_bands: missing rows inherit their nearest band colour.
	const auto original = out;
	for (auto &band : out) {
		if (band.lab)
			continue;
		std::size_t nearest = std::numeric_limits<std::size_t>::max();
		for (const auto &candidate : original) {
			if (!candidate.lab)
				continue;
			auto gap = [](std::size_t a, std::size_t b) { return a > b ? a - b : b - a; };
			const auto distance = std::min(
					gap(candidate.first, band.last), gap(band.first, candidate.last));
			if (distance < nearest) {
				nearest = distance;
				band.lab = candidate.lab;
			}
		}
	}
	auto distance = [](const auto &a, const auto &b) {
		return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) +
						 (a[2] - b[2]) * (a[2] - b[2]));
	};
	auto merge = [&](const Band &a, const Band &b) {
		Band joined{a.first, b.last, a.lab ? a.lab : b.lab, std::nullopt, std::nullopt,
				a.rows_with_wall + b.rows_with_wall};
		const double weight = s0[b.last + 1] - s0[a.first];
		if (weight > 1e-9) {
			std::array<double, 3> colour{};
			for (unsigned c = 0; c < 3; ++c)
				colour[c] = (s1[b.last + 1][c] - s1[a.first][c]) / weight / (c ? 1 : .5);
			joined.lab = colour;
		}
		return joined;
	};
	// Merge shading changes, retaining sharp material boundaries.
	for (std::size_t k = 0; k + 1 < out.size();) {
		const auto a = out[k], b = out[k + 1];
		const double step = b.first && rows[b.first - 1] && rows[b.first]
									? distance(*rows[b.first - 1], *rows[b.first])
									: 0;
		if (a.lab && b.lab &&
				std::hypot((*a.lab)[1] - (*b.lab)[1], (*a.lab)[2] - (*b.lab)[2]) < .02 &&
				std::abs((*a.lab)[0] - (*b.lab)[0]) < .12 && step < tau) {
			out[k] = merge(a, b);
			out.erase(out.begin() + k + 1);
			k = 0;
		} else {
			++k;
		}
	}
	for (std::size_t k = 1; k + 1 < out.size();) {
		const auto band = out[k];
		if (band.first != band.last || !band.lab) {
			++k;
			continue;
		}
		const double before = out[k - 1].lab ? distance(*out[k - 1].lab, *band.lab) : 9;
		const double after = out[k + 1].lab ? distance(*out[k + 1].lab, *band.lab) : 9;
		const bool use_previous = before <= after;
		if (!(use_previous ? out[k - 1].lab : out[k + 1].lab) ||
				std::min(before, after) > 2 * tau) {
			++k;
			continue;
		}
		if (use_previous)
			out[k - 1] = merge(out[k - 1], band);
		else
			out[k + 1] = merge(band, out[k + 1]);
		out.erase(out.begin() + k);
		k = 1;
	}
	if (out.size() >= 2) {
		auto &last = out.back();
		const auto &previous = out[out.size() - 2];
		if (last.last - last.first < 2 && last.lab && previous.lab &&
				(*last.lab)[0] < .35 && (*previous.lab)[0] - (*last.lab)[0] > .25)
			last.lab = previous.lab;
		auto &first = out.front();
		const auto &second = out[1];
		if (first.last - first.first < 2 && first.lab && second.lab &&
				(*first.lab)[0] < .4 && (*second.lab)[0] - (*first.lab)[0] > .25)
			first.lab = second.lab;
	}
	return out;
}
} // namespace arnis::mapillary::bands
