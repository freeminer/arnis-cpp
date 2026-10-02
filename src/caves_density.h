#pragma once

#include "caves_noise.h"
#include <cstdint>
#include <vector>

namespace arnis::world_editor
{
struct WorldEditor;
}

namespace arnis::caves
{
class CaveGen
{
	std::vector<NormalNoise> noises_;
	int y_shift_;
	int world_max_y_;

	double s2d_thickness_modulator(double x, double y, double z) const;
	double spaghetti_roughness(double x, double y, double z) const;
	double spaghetti_2d(double x, double y, double z) const;
	double entrances(double x, double y, double z) const;
	double pillars(double x, double y, double z) const;
	double noodle(double x, double y, double z) const;

public:
	CaveGen(std::int64_t seed, int floor_y, int world_max_y);
	double combined_density(int x, int y, int z) const;
	double noodle_density(int x, int y, int z) const;
};

// Rust's 4x8x4 global density-cell interpolation, including the full-resolution noodle term.
void carve_density_region(const CaveGen &gen, world_editor::WorldEditor &editor,
		int min_x, int max_x, int min_z, int max_z, int floor_y);
}
