#include "detail.h"

#include "../building_facades/choose.h"
#include "../coordinate_system/geographic/llbbox.h"
#include "../coordinate_system/transformation.h"
#include "../land_cover/land_cover.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <numeric>
#include <span>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace arnis::elevation::detail
{
namespace
{
constexpr double RES_FROM_BLOCKS = 10.0;
constexpr double RES_FULL_BLOCKS = 25.0;
constexpr double SLOPE_FROM_DEG = 32.0;
constexpr double SLOPE_FULL_DEG = 48.0;
constexpr double AMPLITUDE_M = 9.0;
constexpr std::array<std::pair<double, double>, 3> OCTAVES{
		{{40.0, 1.0}, {16.0, 0.5}, {6.0, 0.3}}};
constexpr double CONTOUR_STRETCH = 1.8;
constexpr double PATCH_M = 250.0;
constexpr double BLOCK_M = 8.0;
constexpr double GRADED_CLEAR_M = 12.0;
constexpr std::size_t SOURCE_FADE_BLOCKS = 4;
constexpr std::size_t CLEAR_FADE_BLOCKS = 1;
constexpr std::size_t BAND_ROWS = 256;
constexpr double EARTH_RADIUS_M = 6'371'000.0;
constexpr std::uint64_t SALT_PATCH = 0x5EED0001ULL;
constexpr std::uint64_t SALT_OCTAVE = 0x5EED0100ULL;

std::size_t detail_worker_count()
{
	if (const char *value = std::getenv("RAYON_NUM_THREADS")) {
		const std::string_view text(value);
		std::size_t requested = 0;
		const auto [end, error] =
				std::from_chars(text.data(), text.data() + text.size(), requested);
		if (error == std::errc{} && end == text.data() + text.size() && requested > 0)
			return requested;
	}
	const auto hardware = std::max(1u, std::thread::hardware_concurrency());
	return std::max<std::size_t>(1, static_cast<std::size_t>(hardware * 0.9));
}

template <typename Function>
void parallel_for(std::size_t count, Function &&function)
{
	if (count == 0)
		return;
	const auto workers_count = std::min(count, detail_worker_count());
	std::atomic_size_t next{0};
	auto work = [&] {
		for (;;) {
			const auto index = next.fetch_add(1, std::memory_order_relaxed);
			if (index >= count)
				return;
			function(index);
		}
	};
	if (workers_count <= 1) {
		work();
		return;
	}
	std::vector<std::thread> workers;
	workers.reserve(workers_count - 1);
	for (std::size_t i = 1; i < workers_count; ++i) {
		try {
			workers.emplace_back(work);
		} catch (...) {
			break;
		}
	}
	work();
	for (auto &worker : workers)
		worker.join();
}

double smoothstep(double edge0, double edge1, double value)
{
	const double t = std::clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0);
	return t * t * (3.0 - 2.0 * t);
}

std::vector<float> feather(const std::vector<float> &values, std::size_t width,
		std::size_t height, std::size_t blocks);

struct Taps
{
	std::size_t lo{};
	std::size_t hi{};
	float t{};
};

struct Lattice
{
	std::pair<std::int64_t, std::int64_t> origin{};
	std::size_t step{1};
	std::pair<std::int64_t, std::int64_t> first{};
	std::size_t width{};
	std::size_t height{};

	static std::int64_t block(std::int64_t cell, std::size_t step)
	{
		const auto divisor = static_cast<std::int64_t>(step);
		return cell / divisor - (cell % divisor < 0 ? 1 : 0);
	}

	Lattice over(std::size_t cols, std::size_t rows) const
	{
		Lattice out = *this;
		out.first = {block(origin.first, step), block(origin.second, step)};
		out.width = static_cast<std::size_t>(
				block(origin.first + static_cast<std::int64_t>(cols) - 1, step) -
				out.first.first + 1);
		out.height = static_cast<std::size_t>(
				block(origin.second + static_cast<std::int64_t>(rows) - 1, step) -
				out.first.second + 1);
		return out;
	}

	std::size_t block_x(std::int64_t gx) const
	{
		const auto index = block(gx + origin.first, step) - first.first;
		return static_cast<std::size_t>(
				std::clamp<std::int64_t>(index, 0, static_cast<std::int64_t>(width) - 1));
	}

	std::size_t block_z(double pos) const
	{
		const auto cell = static_cast<std::int64_t>(std::llround(pos));
		const auto index = block(cell + origin.second, step) - first.second;
		return static_cast<std::size_t>(std::clamp<std::int64_t>(
				index, 0, static_cast<std::int64_t>(height) - 1));
	}

	static Taps taps(double cell, std::int64_t origin, std::size_t step,
			std::int64_t first, std::size_t blocks)
	{
		const double centred = cell + static_cast<double>(origin) -
							   (static_cast<double>(step) - 1.0) / 2.0;
		const double f = std::clamp(
				centred / static_cast<double>(step) - static_cast<double>(first), 0.0,
				static_cast<double>(blocks - 1));
		const auto lo = static_cast<std::size_t>(std::floor(f));
		return {lo, std::min(lo + 1, blocks - 1), static_cast<float>(f - lo)};
	}

	Taps taps_x(std::size_t gx) const
	{
		return taps(static_cast<double>(gx), origin.first, step, first.first, width);
	}

	Taps taps_z(double pos) const
	{
		return taps(pos, origin.second, step, first.second, height);
	}
};

struct RowPlacement
{
	double lat_top{};
	double lat_bottom{};
	std::optional<std::pair<double, double>> mercator;
	double last{};

	RowPlacement(const geographic::LLBBox &bbox, std::size_t rows, bool use_mercator) :
			lat_top(bbox.max().lat()), lat_bottom(bbox.min().lat()),
			last(static_cast<double>(rows - 1))
	{
		if (use_mercator)
			mercator = std::pair{coordinate_system::lat_to_mercator_y(lat_top),
					coordinate_system::lat_to_mercator_y(lat_bottom)};
	}

	double at_lat(double lat) const
	{
		if (mercator && mercator->first != mercator->second)
			return (coordinate_system::lat_to_mercator_y(lat) - mercator->first) /
				   (mercator->second - mercator->first) * last;
		if (lat_top != lat_bottom)
			return (lat_top - lat) / (lat_top - lat_bottom) * last;
		return 0.0;
	}

	double at_row(double row) const
	{
		if (last <= 0.0)
			return row;
		return at_lat(lat_top - row / last * (lat_top - lat_bottom));
	}
};

struct Gate
{
	Lattice lattice;
	std::vector<float> strength;
	std::vector<bool> live_rows;

	static std::optional<Gate> make(const SourcePixels &source, Lattice lattice,
			const RowPlacement &placement, std::size_t cols, std::size_t rows,
			double scale, const geographic::LLBBox &bbox, double cell_m,
			const land_cover::LandCoverData *land_cover,
			const std::vector<std::vector<std::pair<double, double>>> &graded)
	{
		const std::size_t width = lattice.width;
		const std::size_t height = lattice.height;
		std::vector<std::pair<std::size_t, std::size_t>> block_rows(height, {0, 0});
		for (std::size_t gz = 0; gz < rows; ++gz) {
			auto &span = block_rows[lattice.block_z(placement.at_row(gz))];
			span = span.first == span.second ? std::pair{gz, gz + 1}
											 : std::pair{span.first, gz + 1};
		}

		std::vector<float> coarse(width * height, 0.0f);
		parallel_for(height, [&](std::size_t bz) {
			std::vector<float> finest(width, std::numeric_limits<float>::infinity());
			for (std::size_t gz = block_rows[bz].first; gz < block_rows[bz].second;
					++gz) {
				if (gz >= source.rows.size())
					continue;
				const auto &runs = source.row(gz).runs;
				for (std::size_t i = 0; i < runs.size(); ++i) {
					const auto start = static_cast<std::int64_t>(runs[i].first);
					const auto end =
							i + 1 < runs.size()
									? static_cast<std::int64_t>(runs[i + 1].first)
									: static_cast<std::int64_t>(cols);
					if (end <= start)
						continue;
					const auto bx0 = lattice.block_x(start);
					const auto bx1 = lattice.block_x(end - 1);
					for (auto bx = bx0; bx <= bx1; ++bx)
						finest[bx] = std::min(finest[bx], runs[i].second);
				}
			}
			for (std::size_t bx = 0; bx < width; ++bx)
				coarse[bz * width + bx] =
						std::isfinite(finest[bx])
								? static_cast<float>(smoothstep(RES_FROM_BLOCKS,
										  RES_FULL_BLOCKS,
										  static_cast<double>(finest[bx]) * scale))
								: 0.0f;
		});
		if (std::none_of(coarse.begin(), coarse.end(), [](float v) { return v > 0.0f; }))
			return std::nullopt;

		std::vector<float> keep(width * height, 1.0f);
		if (land_cover && land_cover->width == cols && land_cover->height == rows) {
			parallel_for(height, [&](std::size_t bz) {
				for (std::size_t gz = block_rows[bz].first; gz < block_rows[bz].second;
						++gz) {
					if (gz >= land_cover->grid.size())
						continue;
					const auto &row = land_cover->grid[gz];
					for (std::size_t gx = 0; gx < std::min(cols, row.size()); ++gx) {
						const auto cover = row[gx];
						if (cover == land_cover::LC_WATER ||
								cover == land_cover::LC_BUILT_UP)
							keep[bz * width + lattice.block_x(static_cast<std::int64_t>(
													  gx))] = 0.0f;
					}
				}
			});
		}

		const double reach = GRADED_CLEAR_M / cell_m;
		const double spacing = std::max(static_cast<double>(lattice.step) / 2.0, 0.5);
		const auto &min = bbox.min();
		const auto &max = bbox.max();
		for (const auto &way : graded) {
			if (way.empty())
				continue;
			for (std::size_t segment = 0;
					segment < std::max<std::size_t>(1, way.size() - 1); ++segment) {
				const auto a_geo = way[std::min(segment, way.size() - 1)];
				const auto b_geo = way[std::min(segment + 1, way.size() - 1)];
				const auto point = [&](const std::pair<double, double> &p) {
					return std::pair{
							(p.second - min.lng()) / (max.lng() - min.lng()) * (cols - 1),
							(max.lat() - p.first) / (max.lat() - min.lat()) * (rows - 1)};
				};
				const auto a = point(a_geo);
				const auto b = point(b_geo);
				const double dx = b.first - a.first;
				const double dz = b.second - a.second;
				const double lo = -0.5 - reach;
				const double hi_x = static_cast<double>(cols) - 0.5 + reach;
				const double hi_z = static_cast<double>(rows) - 0.5 + reach;
				double t0 = 0.0, t1 = 1.0;
				const std::array<std::pair<double, double>, 4> clipping{
						std::pair<double, double>{-dx, a.first - lo},
						std::pair<double, double>{dx, hi_x - a.first},
						std::pair<double, double>{-dz, a.second - lo},
						std::pair<double, double>{dz, hi_z - a.second}};
				for (const auto &[p, q] : clipping) {
					if (p == 0.0) {
						if (std::isnan(q) || q < 0.0)
							t1 = -1.0;
					} else if (p < 0.0) {
						t0 = std::max(t0, q / p);
					} else {
						t1 = std::min(t1, q / p);
					}
				}
				if (t0 > t1)
					continue;
				const auto steps = static_cast<std::size_t>(
						std::ceil((t1 - t0) * std::hypot(dx, dz) / spacing));
				for (std::size_t k = 0; k <= steps; ++k) {
					const double t =
							t0 +
							(t1 - t0) * static_cast<double>(k) /
									static_cast<double>(std::max<std::size_t>(1, steps));
					const double x = a.first + dx * t;
					const double z = placement.at_row(a.second + dz * t);
					const auto bx0 = lattice.block_x(
							static_cast<std::int64_t>(std::floor(x - reach)));
					const auto bx1 = lattice.block_x(
							static_cast<std::int64_t>(std::ceil(x + reach)));
					const auto bz0 = lattice.block_z(z - reach);
					const auto bz1 = lattice.block_z(z + reach);
					for (auto bz = bz0; bz <= bz1; ++bz)
						std::fill(keep.begin() +
										  static_cast<std::ptrdiff_t>(bz * width + bx0),
								keep.begin() +
										static_cast<std::ptrdiff_t>(bz * width + bx1 + 1),
								0.0f);
				}
			}
		}

		const auto source_faded = feather(coarse, width, height, SOURCE_FADE_BLOCKS);
		const auto clear_faded = feather(keep, width, height, CLEAR_FADE_BLOCKS);
		Gate gate;
		gate.lattice = lattice;
		gate.strength.resize(width * height);
		gate.live_rows.resize(height, false);
		for (std::size_t i = 0; i < gate.strength.size(); ++i) {
			gate.strength[i] = source_faded[i] * clear_faded[i];
			if (gate.strength[i] > 0.0f)
				gate.live_rows[i / width] = true;
		}
		return gate;
	}

	bool any_in(const Taps &z) const { return live_rows[z.lo] || live_rows[z.hi]; }
	double at(const Taps &x, const Taps &z) const
	{
		const auto s = [&](std::size_t bx, std::size_t bz) {
			return strength[bz * lattice.width + bx];
		};
		const auto lerp = [](float a, float b, float t) { return a + (b - a) * t; };
		return static_cast<double>(lerp(lerp(s(x.lo, z.lo), s(x.hi, z.lo), x.t),
				lerp(s(x.lo, z.hi), s(x.hi, z.hi), x.t), z.t));
	}
};

std::vector<float> separable(const std::vector<float> &values, std::size_t width,
		std::size_t height, const auto &line)
{
	std::vector<float> rows;
	rows.reserve(values.size());
	for (std::size_t z = 0; z < height; ++z) {
		const auto result =
				line(std::span<const float>(values.data() + z * width, width));
		rows.insert(rows.end(), result.begin(), result.end());
	}
	std::vector<float> out(values.size());
	std::vector<float> column(height);
	for (std::size_t x = 0; x < width; ++x) {
		for (std::size_t z = 0; z < height; ++z)
			column[z] = rows[z * width + x];
		const auto result = line(std::span<const float>(column));
		for (std::size_t z = 0; z < height; ++z)
			out[z * width + x] = result[z];
	}
	return out;
}

std::vector<float> feather(const std::vector<float> &values, std::size_t width,
		std::size_t height, std::size_t blocks)
{
	const auto hard = separable(values, width, height, [](std::span<const float> line) {
		std::vector<float> out(line.size());
		for (std::size_t i = 0; i < line.size(); ++i) {
			out[i] = std::numeric_limits<float>::infinity();
			for (std::size_t j = i ? i - 1 : 0; j <= std::min(i + 1, line.size() - 1);
					++j)
				out[i] = std::min(out[i], line[j]);
		}
		return out;
	});
	const auto soft =
			separable(hard, width, height, [blocks](std::span<const float> line) {
				std::vector<float> sums(line.size() + 1, 0.0f);
				for (std::size_t i = 0; i < line.size(); ++i)
					sums[i + 1] = sums[i] + line[i];
				std::vector<float> out(line.size());
				for (std::size_t i = 0; i < line.size(); ++i) {
					const auto lo = i > blocks ? i - blocks : 0;
					const auto hi = std::min(line.size(), i + blocks + 1);
					out[i] = (sums[hi] - sums[lo]) / static_cast<float>(hi - lo);
				}
				return out;
			});
	std::vector<float> result(soft.size());
	for (std::size_t i = 0; i < result.size(); ++i)
		result[i] = std::min(soft[i], hard[i]);
	return result;
}

double lattice_value(std::int64_t x, std::int64_t y, std::int64_t z, std::uint64_t salt)
{
	const auto h = building_facades::hash64(
			building_facades::hash64(
					building_facades::hash64(salt ^ static_cast<std::uint64_t>(x)) ^
					static_cast<std::uint64_t>(y)) ^
			static_cast<std::uint64_t>(z));
	return static_cast<double>(h >> 11) * (2.0 / 9007199254740992.0) - 1.0;
}

double value_noise(const std::array<double, 3> &p, std::uint64_t salt)
{
	const std::array<double, 3> base{
			std::floor(p[0]), std::floor(p[1]), std::floor(p[2])};
	std::array<double, 3> t{};
	std::array<std::int64_t, 3> b{};
	for (std::size_t i = 0; i < 3; ++i) {
		const double f = p[i] - base[i];
		t[i] = f * f * (3.0 - 2.0 * f);
		b[i] = static_cast<std::int64_t>(base[i]);
	}
	const auto corner = [&](int dx, int dy, int dz) {
		return lattice_value(b[0] + dx, b[1] + dy, b[2] + dz, salt);
	};
	const auto lerp = [](double a, double c, double weight) {
		return a + (c - a) * weight;
	};
	const double near = lerp(lerp(corner(0, 0, 0), corner(1, 0, 0), t[0]),
			lerp(corner(0, 1, 0), corner(1, 1, 0), t[0]), t[1]);
	const double far = lerp(lerp(corner(0, 0, 1), corner(1, 0, 1), t[0]),
			lerp(corner(0, 1, 1), corner(1, 1, 1), t[0]), t[1]);
	return lerp(near, far, t[2]);
}

struct Field
{
	const Gate &gate;
	const std::vector<Taps> &col_taps;
	const std::vector<Taps> &row_taps;
	const std::vector<std::pair<double, double>> &lats;
	const std::vector<std::pair<double, double>> &lngs;
	std::array<std::pair<double, double>, 3> octaves;
	double cell_x{};
	double cell_z{};
	std::optional<std::pair<double, double>> range;

	std::size_t row(std::size_t gz, std::vector<double> &out,
			const std::vector<double> &orig, const std::vector<double> *above,
			const std::vector<double> *below) const
	{
		const auto &taps_z = row_taps[gz];
		if (!gate.any_in(taps_z))
			return 0;
		const auto &up = above ? *above : orig;
		const auto &down = below ? *below : orig;
		const double span_z = ((above ? 1.0 : 0.0) + (below ? 1.0 : 0.0)) * cell_z;
		const auto [sin_lat, cos_lat] = lats[gz];
		std::size_t changed = 0;
		for (std::size_t gx = 0; gx < orig.size(); ++gx) {
			const double g = gate.at(col_taps[gx], taps_z);
			if (g <= 0.0)
				continue;
			const auto x0 = gx ? gx - 1 : 0;
			const auto x1 = std::min(gx + 1, orig.size() - 1);
			const double dx =
					(orig[x1] - orig[x0]) / (static_cast<double>(x1 - x0) * cell_x);
			const double dz = (down[gx] - up[gx]) / span_z;
			const double slope = std::atan(std::hypot(dx, dz)) * 180.0 / std::numbers::pi;
			const double slope_weight = smoothstep(SLOPE_FROM_DEG, SLOPE_FULL_DEG, slope);
			if (!std::isfinite(slope_weight) || slope_weight <= 0.0)
				continue;
			const double h = orig[gx];
			const auto [sin_lng, cos_lng] = lngs[gx];
			const std::array<double, 3> up_vec{
					cos_lat * cos_lng, cos_lat * sin_lng, sin_lat};
			const auto at = [&](double radius) {
				return std::array{
						up_vec[0] * radius, up_vec[1] * radius, up_vec[2] * radius};
			};
			const double patch =
					0.35 +
					0.65 * smoothstep(0.2, 0.7,
								   0.5 + 0.5 * value_noise(at(EARTH_RADIUS_M / PATCH_M),
													   SALT_PATCH));
			const double radius = EARTH_RADIUS_M + CONTOUR_STRETCH * h;
			double noise = 0.0;
			for (std::size_t i = 0; i < octaves.size(); ++i)
				if (octaves[i].second > 0.0)
					noise += octaves[i].second *
							 value_noise(at(radius / octaves[i].first), SALT_OCTAVE + i);
			double detailed = h + AMPLITUDE_M * g * slope_weight * patch * noise;
			if (range)
				detailed = std::clamp(detailed, range->first, range->second);
			if (std::isfinite(detailed) && detailed != h) {
				out[gx] = detailed;
				++changed;
			}
		}
		return changed;
	}
};
} // namespace

std::size_t add_steep_detail(std::vector<std::vector<double>> &heights,
		const geographic::LLBBox &bbox, const SourcePixels &source, double scale,
		std::optional<std::pair<std::int64_t, std::int64_t>> one_world_origin,
		const land_cover::LandCoverData *land_cover,
		const std::vector<std::vector<std::pair<double, double>>> &graded)
{
	const std::size_t rows = heights.size();
	const std::size_t cols = rows ? heights.front().size() : 0;
	if (rows < 3 || cols < 3 || source.height() != rows)
		return 0;
	for (const auto &row : heights)
		if (row.size() != cols)
			return 0;
	const auto [bbox_h_m, bbox_w_m] =
			coordinate_system::geo_distance(bbox.min(), bbox.max());
	const double cell_x = bbox_w_m / static_cast<double>(cols - 1);
	const double cell_z = bbox_h_m / static_cast<double>(rows - 1);
	const double cell_m = std::min(cell_x, cell_z);
	auto octaves = OCTAVES;
	for (auto &[size, weight] : octaves)
		weight *= smoothstep(2.0, 4.0, size / std::max(cell_x, cell_z));
	if (std::none_of(octaves.begin(), octaves.end(),
				[](const auto &octave) { return octave.second > 0.0; }))
		return 0;

	const RowPlacement placement(bbox, rows, one_world_origin.has_value());
	const auto step =
			one_world_origin
					? std::max<std::size_t>(
							  1, static_cast<std::size_t>(std::llround(BLOCK_M * scale)))
					: std::max<std::size_t>(1,
							  static_cast<std::size_t>(std::llround(BLOCK_M / cell_m)));
	Lattice lattice{
			one_world_origin.value_or(std::pair<std::int64_t, std::int64_t>{0, 0}), step};
	lattice = lattice.over(cols, rows);
	std::vector<Taps> row_taps(rows), col_taps(cols);
	for (std::size_t z = 0; z < rows; ++z)
		row_taps[z] = lattice.taps_z(placement.at_row(static_cast<double>(z)));
	for (std::size_t x = 0; x < cols; ++x)
		col_taps[x] = lattice.taps_x(x);
	const auto *matching_cover =
			land_cover && land_cover->width == cols && land_cover->height == rows
					? land_cover
					: nullptr;
	auto gate = Gate::make(source, lattice, placement, cols, rows, scale, bbox, cell_m,
			matching_cover, graded);
	if (!gate)
		return 0;

	std::optional<std::pair<double, double>> range;
	if (!one_world_origin) {
		double lo = std::numeric_limits<double>::infinity();
		double hi = -std::numeric_limits<double>::infinity();
		for (const auto &row : heights)
			for (const double h : row)
				if (std::isfinite(h)) {
					lo = std::min(lo, h);
					hi = std::max(hi, h);
				}
		if (lo > hi)
			return 0;
		range = std::pair{lo, hi};
	}
	std::vector<std::pair<double, double>> lats(rows), lngs(cols);
	for (std::size_t z = 0; z < rows; ++z) {
		const double lat =
				bbox.max().lat() - static_cast<double>(z) / (rows - 1) *
										   (bbox.max().lat() - bbox.min().lat());
		lats[z] = {std::sin(lat * std::numbers::pi / 180.0),
				std::cos(lat * std::numbers::pi / 180.0)};
	}
	for (std::size_t x = 0; x < cols; ++x) {
		const double lon =
				bbox.min().lng() + static_cast<double>(x) / (cols - 1) *
										   (bbox.max().lng() - bbox.min().lng());
		lngs[x] = {std::sin(lon * std::numbers::pi / 180.0),
				std::cos(lon * std::numbers::pi / 180.0)};
	}
	const Field field{
			*gate, col_taps, row_taps, lats, lngs, octaves, cell_x, cell_z, range};
	const std::size_t bands = (rows + BAND_ROWS - 1) / BAND_ROWS;
	std::vector<std::pair<std::vector<double>, std::vector<double>>> edges;
	edges.reserve(bands);
	for (std::size_t band = 0; band < bands; ++band) {
		const auto start = band * BAND_ROWS;
		const auto end = std::min(start + BAND_ROWS, rows);
		edges.emplace_back(start ? heights[start - 1] : std::vector<double>{},
				end < rows ? heights[end] : std::vector<double>{});
	}
	std::vector<std::size_t> changed_by_band(bands, 0);
	parallel_for(bands, [&](std::size_t band) {
		const auto start = band * BAND_ROWS;
		const auto end = std::min(start + BAND_ROWS, rows);
		std::vector<double> prev = std::move(edges[band].first);
		auto &changed = changed_by_band[band];
		for (std::size_t z = start; z < end; ++z) {
			auto orig = heights[z];
			const std::vector<double> *above = prev.empty() ? nullptr : &prev;
			const std::vector<double> *below =
					z + 1 < end ? &heights[z + 1]
								: (edges[band].second.empty() ? nullptr
															  : &edges[band].second);
			changed += field.row(z, heights[z], orig, above, below);
			prev.swap(orig);
		}
	});
	const std::size_t changed = std::accumulate(
			changed_by_band.begin(), changed_by_band.end(), std::size_t{0});
	return changed;
}

} // namespace arnis::elevation::detail
