#include "fuse.h"

#include <cstdint>
#include <functional>
#include <queue>
#include <tuple>

namespace arnis::mapillary::fuse
{
namespace
{
using QueueEntry = std::tuple<float, std::uint64_t, std::size_t, std::size_t>;
using Queue = std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>>;

void push(Queue &queue, std::uint64_t &sequence, std::size_t y, std::size_t x, float t)
{
	queue.emplace(t, sequence++, y, x);
}

float solve(std::size_t y1, std::size_t x1, std::size_t y2, std::size_t x2,
		const std::vector<std::uint8_t> &flags, const std::vector<float> &time,
		std::size_t width)
{
	const float a = time[y1 * width + x1];
	const float b = time[y2 * width + x2];
	const float minimum = std::min(a, b);
	const bool known_a = flags[y1 * width + x1] != telea_inside;
	const bool known_b = flags[y2 * width + x2] != telea_inside;
	if (known_a && known_b) {
		if (std::abs(a - b) >= 1.0f)
			return 1.0f + minimum;
		const double da = a, db = b;
		return static_cast<float>(
				0.5 * (da + db + std::sqrt(std::max(0.0, 2.0 - (da - db) * (da - db)))));
	}
	if (known_a)
		return 1.0f + a;
	if (known_b)
		return 1.0f + b;
	return 1.0f + minimum;
}

void march(std::vector<std::uint8_t> &flags, std::vector<float> &time, Queue &queue,
		std::uint64_t &sequence, std::size_t width, std::size_t height, bool negate)
{
	while (!queue.empty()) {
		const auto [arrival, order, y, x] = queue.top();
		(void)order;
		queue.pop();
		flags[y * width + x] = negate ? telea_change : telea_known;
		const std::array<std::pair<std::ptrdiff_t, std::ptrdiff_t>, 4> offsets{
				{{-1, 0}, {0, -1}, {1, 0}, {0, 1}}};
		for (const auto &[dy, dx] : offsets) {
			const auto ny = static_cast<std::ptrdiff_t>(y) + dy;
			const auto nx = static_cast<std::ptrdiff_t>(x) + dx;
			if (ny <= 0 || nx <= 0 || ny >= static_cast<std::ptrdiff_t>(height) - 1 ||
					nx >= static_cast<std::ptrdiff_t>(width) - 1)
				continue;
			const auto iy = static_cast<std::size_t>(ny),
					   ix = static_cast<std::size_t>(nx);
			if (flags[iy * width + ix] != telea_inside)
				continue;
			const float d = std::min({solve(iy - 1, ix, iy, ix - 1, flags, time, width),
					solve(iy + 1, ix, iy, ix - 1, flags, time, width),
					solve(iy - 1, ix, iy, ix + 1, flags, time, width),
					solve(iy + 1, ix, iy, ix + 1, flags, time, width)});
			time[iy * width + ix] = d;
			flags[iy * width + ix] = telea_band;
			push(queue, sequence, iy, ix, d);
		}
		(void)arrival;
	}
	if (negate)
		for (std::size_t i = 0; i < width * height; ++i)
			if (flags[i] == telea_change) {
				flags[i] = telea_known;
				time[i] = -time[i];
			}
}

std::array<double, 2> time_gradient(const std::vector<std::uint8_t> &mask,
		const std::vector<float> &time, std::size_t width, std::size_t y, std::size_t x)
{
	auto known = [&](std::size_t py, std::size_t px) {
		return mask[py * width + px] != telea_inside;
	};
	auto t = [&](std::size_t py, std::size_t px) {
		return static_cast<double>(time[py * width + px]);
	};
	const double gx = known(y, x + 1)
							  ? (known(y, x - 1) ? 0.5 * (t(y, x + 1) - t(y, x - 1))
												 : t(y, x + 1) - t(y, x))
							  : (known(y, x - 1) ? t(y, x) - t(y, x - 1) : 0.0);
	const double gy = known(y + 1, x)
							  ? (known(y - 1, x) ? 0.5 * (t(y + 1, x) - t(y - 1, x))
												 : t(y + 1, x) - t(y, x))
							  : (known(y - 1, x) ? t(y, x) - t(y - 1, x) : 0.0);
	return {gx, gy};
}

void inpaint_telea(projection::Image &image, const std::vector<bool> &hole, int radius)
{
	const std::size_t image_width = image.width, image_height = image.height;
	if (!image_width || !image_height || hole.size() != image_width * image_height ||
			image.pixels.size() < hole.size())
		return;
	const std::size_t width = image_width + 2, height = image_height + 2;
	std::vector<std::uint8_t> mask(width * height, telea_known);
	for (std::size_t y = 0; y < image_height; ++y)
		for (std::size_t x = 0; x < image_width; ++x)
			if (hole[y * image_width + x])
				mask[(y + 1) * width + x + 1] = telea_inside;
	std::vector<float> time(width * height, 1.0e6f);
	std::vector<bool> band(width * height, false);
	for (std::size_t y = 1; y + 1 < height; ++y)
		for (std::size_t x = 1; x + 1 < width; ++x) {
			if (mask[y * width + x] == telea_inside)
				continue;
			if (mask[(y - 1) * width + x] == telea_inside ||
					mask[(y + 1) * width + x] == telea_inside ||
					mask[y * width + x - 1] == telea_inside ||
					mask[y * width + x + 1] == telea_inside)
				band[y * width + x] = true;
		}
	Queue inward;
	std::uint64_t inward_sequence = 0;
	for (std::size_t y = 0; y < height; ++y)
		for (std::size_t x = 0; x < width; ++x)
			if (band[y * width + x]) {
				time[y * width + x] = 0.0f;
				push(inward, inward_sequence, y, x, 0.0f);
			}
	const auto range = static_cast<std::size_t>(std::max(radius, 0));
	std::vector<std::uint8_t> outward_flags(width * height, telea_known);
	for (std::size_t y = 1; y + 1 < height; ++y)
		for (std::size_t x = 1; x + 1 < width; ++x) {
			if (mask[y * width + x] == telea_inside || band[y * width + x])
				continue;
			const auto y0 = y > range ? y - range : 0;
			const auto y1 = std::min(y + range, height - 1);
			const auto x0 = x > range ? x - range : 0;
			const auto x1 = std::min(x + range, width - 1);
			bool near_hole = false;
			for (std::size_t sy = y0; sy <= y1 && !near_hole; ++sy)
				for (std::size_t sx = x0; sx <= x1; ++sx)
					if (mask[sy * width + sx] == telea_inside) {
						near_hole = true;
						break;
					}
			if (near_hole)
				outward_flags[y * width + x] = telea_inside;
		}
	Queue outward;
	std::uint64_t outward_sequence = 0;
	for (std::size_t y = 0; y < height; ++y)
		for (std::size_t x = 0; x < width; ++x)
			if (band[y * width + x])
				push(outward, outward_sequence, y, x, 0.0f);
	march(outward_flags, time, outward, outward_sequence, width, height, true);
	auto pixels = image.pixels;
	auto get = [&](std::size_t y, std::size_t x, std::size_t c) {
		return static_cast<double>(pixels[y * image_width + x][c]);
	};
	while (!inward.empty()) {
		const auto [arrival, order, y, x] = inward.top();
		(void)arrival;
		(void)order;
		inward.pop();
		mask[y * width + x] = telea_known;
		const std::array<std::pair<std::ptrdiff_t, std::ptrdiff_t>, 4> offsets{
				{{-1, 0}, {0, -1}, {1, 0}, {0, 1}}};
		for (const auto &[dy, dx] : offsets) {
			const auto ny = static_cast<std::ptrdiff_t>(y) + dy;
			const auto nx = static_cast<std::ptrdiff_t>(x) + dx;
			if (ny <= 0 || nx <= 0 || ny >= static_cast<std::ptrdiff_t>(height) - 1 ||
					nx >= static_cast<std::ptrdiff_t>(width) - 1)
				continue;
			const auto iy = static_cast<std::size_t>(ny),
					   ix = static_cast<std::size_t>(nx);
			if (mask[iy * width + ix] != telea_inside)
				continue;
			const float dist = std::min({solve(iy - 1, ix, iy, ix - 1, mask, time, width),
					solve(iy + 1, ix, iy, ix - 1, mask, time, width),
					solve(iy - 1, ix, iy, ix + 1, mask, time, width),
					solve(iy + 1, ix, iy, ix + 1, mask, time, width)});
			time[iy * width + ix] = dist;
			const auto grad = time_gradient(mask, time, width, iy, ix);
			for (std::size_t c = 0; c < 3; ++c) {
				double intensity = 0.0, jx = 0.0, jy = 0.0, sum = 1.0e-20;
				const auto k0 = iy > range ? iy - range : 0;
				const auto k1 = std::min(iy + range, height - 1);
				const auto l0 = ix > range ? ix - range : 0;
				const auto l1 = std::min(ix + range, width - 1);
				for (std::size_t k = k0; k <= k1; ++k) {
					if (k == 0 || k >= height - 1)
						continue;
					const auto km = k - 1 + (k == 1 ? 1 : 0);
					for (std::size_t l = l0; l <= l1; ++l) {
						if (l == 0 || l >= width - 1 ||
								mask[k * width + l] == telea_inside)
							continue;
						const double dy = static_cast<double>(iy) - k;
						const double dx = static_cast<double>(ix) - l;
						if (dx * dx + dy * dy > static_cast<double>(radius * radius))
							continue;
						const auto lm = l - 1 + (l == 1 ? 1 : 0);
						const auto lp = l - 1 - (l == width - 2 ? 1 : 0);
						const auto kp = k - 1 - (k == height - 2 ? 1 : 0);
						const double len2 = dx * dx + dy * dy;
						const double dst = 1.0 / (len2 * std::sqrt(len2));
						const double lev = 1.0 / (1.0 + std::abs(time[k * width + l] -
																 time[iy * width + ix]));
						double dir = dx * grad[0] + dy * grad[1];
						if (std::abs(dir) <= 0.01)
							dir = 0.000001;
						const double weight = std::abs(dst * lev * dir);
						const double gx =
								mask[k * width + l + 1] != telea_inside
										? (mask[k * width + l - 1] != telea_inside
														  ? (get(km, lp + 1, c) -
																	get(km, lm - 1, c)) *
																	2.0
														  : get(km, lp + 1, c) -
																	get(km, lm, c))
										: (mask[k * width + l - 1] != telea_inside
														  ? get(km, lm, c) -
																	get(km, lm - 1, c)
														  : 0.0);
						const double gy =
								mask[(k + 1) * width + l] != telea_inside
										? (mask[(k - 1) * width + l] != telea_inside
														  ? (get(kp + 1, lm, c) -
																	get(km - 1, lm, c)) *
																	2.0
														  : get(kp + 1, lm, c) -
																	get(km, lm, c))
										: (mask[(k - 1) * width + l] != telea_inside
														  ? get(km, lm, c) -
																	get(km - 1, lm, c)
														  : 0.0);
						intensity += weight * get(km, lm, c);
						jx -= weight * gx * dx;
						jy -= weight * gy * dy;
						sum += weight;
					}
				}
				const double value = intensity / sum +
									 (jx + jy) / (std::hypot(jx, jy) + 1.0e-20) + 0.5;
				pixels[(iy - 1) * image_width + (ix - 1)][c] = static_cast<std::uint8_t>(
						std::clamp(imgops::round_half_even(value), 0.0, 255.0));
			}
			mask[iy * width + ix] = telea_band;
			push(inward, inward_sequence, iy, ix, dist);
		}
	}
	image.pixels = std::move(pixels);
}
} // namespace

HoleFill fill_holes(const projection::Image &rgb, const std::vector<bool> &valid,
		unsigned pixels_per_block, double maximum_hole_m)
{
	const std::size_t width = rgb.width, height = rgb.height, count = width * height;
	HoleFill result{rgb, valid, 0.0, std::vector<bool>(count, false)};
	if (!width || !height || valid.size() != count || rgb.pixels.size() < count ||
			std::all_of(valid.begin(), valid.end(), [](bool v) { return v; }) ||
			std::none_of(valid.begin(), valid.end(), [](bool v) { return v; }))
		return result;
	std::vector<bool> holes(count);
	for (std::size_t i = 0; i < count; ++i)
		holes[i] = !valid[i];
	const auto labels =
			imgops::connected_components(imgops::Mask::from_bits(width, height, holes));
	const auto distance = distance_to_false(holes, width, height);
	std::vector<double> maximum_radius(labels.count(), 0.0);
	for (std::size_t i = 0; i < labels.labels.size() && i < distance.size(); ++i) {
		const auto id = static_cast<std::size_t>(labels.labels[i]);
		if (id && id < maximum_radius.size())
			maximum_radius[id] = std::max(maximum_radius[id], distance[i]);
	}
	const double limit = maximum_hole_m * pixels_per_block;
	for (std::size_t i = 0; i < count && i < labels.labels.size(); ++i) {
		const auto id = static_cast<std::size_t>(labels.labels[i]);
		result.filled[i] =
				id && id < maximum_radius.size() && 2.0 * maximum_radius[id] <= limit;
	}
	const auto filled_count = static_cast<std::size_t>(
			std::count(result.filled.begin(), result.filled.end(), true));
	if (!filled_count)
		return result;
	inpaint_telea(result.rgb, result.filled, inpaint_radius_px);
	for (std::size_t i = 0; i < count; ++i)
		result.valid[i] = valid[i] || result.filled[i];
	result.fraction = static_cast<double>(filled_count) / count;
	return result;
}
} // namespace arnis::mapillary::fuse
