#pragma once

#include "caves_rng.h"
#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace arnis::caves
{
class ImprovedNoise
{
	std::array<std::uint8_t, 256> permutation_{};
	double x_offset_ = 0.0, y_offset_ = 0.0, z_offset_ = 0.0;
	int p(int index) const;
	double sample_and_lerp(int gx, int gy, int gz, double dx, double dy,
			double dz) const;

public:
	explicit ImprovedNoise(XoroRandom &random);
	double noise(double x, double y, double z) const;
};

class PerlinNoise
{
	std::vector<std::optional<ImprovedNoise>> levels_;
	std::vector<double> amplitudes_;
	double lowest_frequency_input_factor_ = 1.0;
	double lowest_frequency_value_factor_ = 1.0;

public:
	static PerlinNoise create(XoroRandom &random, int first_octave,
			const std::vector<double> &amplitudes);
	double get_value(double x, double y, double z) const;
};

class NormalNoise
{
	PerlinNoise first_;
	PerlinNoise second_;
	double value_factor_ = 1.0;

	NormalNoise(PerlinNoise first, PerlinNoise second, double value_factor);

public:
	static NormalNoise create(XoroRandom &random, int first_octave,
			const std::vector<double> &amplitudes);
	double get_value(double x, double y, double z) const;
};
}
