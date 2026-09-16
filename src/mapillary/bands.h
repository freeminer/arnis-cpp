#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>
namespace arnis::mapillary::bands
{
inline constexpr double band_tau = .05;
inline constexpr std::size_t band_max = 5;
struct Band
{
	std::size_t first{}, last{};
	std::optional<std::array<double, 3>> lab;
	std::size_t rows_with_wall{};
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
				if (score > result.score)
					result = {period, width, phase, score, mean, 0, false, std::move(on),
							{}};
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
		out.push_back({i, j - 1, lab, observed});
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
		Band joined{a.first, b.last, a.lab ? a.lab : b.lab,
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
