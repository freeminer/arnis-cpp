#include "caves_noise.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace arnis::caves
{
namespace
{
constexpr std::array<std::array<int, 3>, 16> GRADIENT{{
		{1, 1, 0}, {-1, 1, 0}, {1, -1, 0}, {-1, -1, 0},
		{1, 0, 1}, {-1, 0, 1}, {1, 0, -1}, {-1, 0, -1},
		{0, 1, 1}, {0, -1, 1}, {0, 1, -1}, {0, -1, -1},
		{1, 1, 0}, {0, -1, 1}, {-1, 1, 0}, {0, -1, -1}}};
constexpr double ROUND_OFF = 3.3554432e7;
constexpr double INPUT_FACTOR = 1.0181268882175227;

double grad_dot(unsigned gradient, double x, double y, double z)
{
	const auto &v = GRADIENT[gradient & 15];
	return v[0] * x + v[1] * y + v[2] * z;
}
double lerp(double delta, double start, double end)
{
	return start + delta * (end - start);
}
double smoothstep(double t)
{
	return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}
double lerp2(double dx, double dy, double a, double b, double c, double d)
{
	return lerp(dy, lerp(dx, a, b), lerp(dx, c, d));
}
double lerp3(double dx, double dy, double dz, const std::array<double, 8> &v)
{
	return lerp(dz, lerp2(dx, dy, v[0], v[1], v[2], v[3]),
			lerp2(dx, dy, v[4], v[5], v[6], v[7]));
}
double wrap(double value)
{
	return value - std::floor(value / ROUND_OFF + .5) * ROUND_OFF;
}
double expected_deviation(int octaves)
{
	return .1 * (1.0 + 1.0 / (octaves + 1.0));
}
}

ImprovedNoise::ImprovedNoise(XoroRandom &random)
{
	x_offset_ = random.next_double() * 256.0;
	y_offset_ = random.next_double() * 256.0;
	z_offset_ = random.next_double() * 256.0;
	for (unsigned i = 0; i < permutation_.size(); ++i)
		permutation_[i] = static_cast<std::uint8_t>(i);
	for (int i = 0; i < 256; ++i) {
		const int swap_index = i + random.next_int(256 - i);
		std::swap(permutation_[i], permutation_[swap_index]);
	}
}

int ImprovedNoise::p(int index) const
{
	return permutation_[static_cast<unsigned>(index) & 0xff];
}

double ImprovedNoise::sample_and_lerp(int gx, int gy, int gz, double dx,
		double dy, double dz) const
{
	const int i = p(gx), i1 = p(gx + 1);
	const int i2 = p(i + gy), i3 = p(i + gy + 1);
	const int i4 = p(i1 + gy), i5 = p(i1 + gy + 1);
	const std::array<double, 8> corners{
			grad_dot(p(i2 + gz), dx, dy, dz),
			grad_dot(p(i4 + gz), dx - 1.0, dy, dz),
			grad_dot(p(i3 + gz), dx, dy - 1.0, dz),
			grad_dot(p(i5 + gz), dx - 1.0, dy - 1.0, dz),
			grad_dot(p(i2 + gz + 1), dx, dy, dz - 1.0),
			grad_dot(p(i4 + gz + 1), dx - 1.0, dy, dz - 1.0),
			grad_dot(p(i3 + gz + 1), dx, dy - 1.0, dz - 1.0),
			grad_dot(p(i5 + gz + 1), dx - 1.0, dy - 1.0, dz - 1.0)};
	return lerp3(smoothstep(dx), smoothstep(dy), smoothstep(dz), corners);
}

double ImprovedNoise::noise(double x, double y, double z) const
{
	const double dx = x + x_offset_, dy = y + y_offset_, dz = z + z_offset_;
	const int fx = static_cast<int>(std::floor(dx));
	const int fy = static_cast<int>(std::floor(dy));
	const int fz = static_cast<int>(std::floor(dz));
	return sample_and_lerp(fx, fy, fz, dx - fx, dy - fy, dz - fz);
}

PerlinNoise PerlinNoise::create(XoroRandom &random, int first_octave,
		const std::vector<double> &amplitudes)
{
	PerlinNoise out;
	out.levels_.resize(amplitudes.size());
	out.amplitudes_ = amplitudes;
	const auto factory = random.fork_positional();
	for (std::size_t i = 0; i < amplitudes.size(); ++i) {
		if (amplitudes[i] == 0.0)
			continue;
		const int octave = first_octave + static_cast<int>(i);
		XoroRandom octave_random = XoroRandom::from_hash_of(
				factory.first, factory.second, "octave_" + std::to_string(octave));
		out.levels_[i].emplace(octave_random);
	}
	out.lowest_frequency_input_factor_ = std::pow(2.0, first_octave);
	out.lowest_frequency_value_factor_ = std::pow(2.0,
			static_cast<int>(amplitudes.size()) - 1) /
			(std::pow(2.0, static_cast<int>(amplitudes.size())) - 1.0);
	return out;
}

double PerlinNoise::get_value(double x, double y, double z) const
{
	double total = 0.0;
	double input_factor = lowest_frequency_input_factor_;
	double value_factor = lowest_frequency_value_factor_;
	for (std::size_t i = 0; i < levels_.size(); ++i) {
		if (levels_[i])
			total += amplitudes_[i] * levels_[i]->noise(wrap(x * input_factor),
					wrap(y * input_factor), wrap(z * input_factor)) * value_factor;
		input_factor *= 2.0;
		value_factor /= 2.0;
	}
	return total;
}

NormalNoise::NormalNoise(PerlinNoise first, PerlinNoise second, double value_factor)
		: first_(std::move(first)), second_(std::move(second)), value_factor_(value_factor)
{
}

NormalNoise NormalNoise::create(XoroRandom &random, int first_octave,
		const std::vector<double> &amplitudes)
{
	auto first = PerlinNoise::create(random, first_octave, amplitudes);
	auto second = PerlinNoise::create(random, first_octave, amplitudes);
	int min_index = static_cast<int>(amplitudes.size()), max_index = -1;
	for (std::size_t i = 0; i < amplitudes.size(); ++i) {
		if (amplitudes[i] != 0.0) {
			min_index = std::min(min_index, static_cast<int>(i));
			max_index = std::max(max_index, static_cast<int>(i));
		}
	}
	const double factor = (1.0 / 6.0) / expected_deviation(max_index - min_index);
	return NormalNoise(std::move(first), std::move(second), factor);
}

double NormalNoise::get_value(double x, double y, double z) const
{
	return (first_.get_value(x, y, z) +
				second_.get_value(x * INPUT_FACTOR, y * INPUT_FACTOR,
						y == y ? z * INPUT_FACTOR : z)) *
			value_factor_;
}
}
