#pragma once

#include "../block_definitions.h"
#include "../../../arnis_adapter.h"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace arnis::structure_kit
{

struct Frame
{
	double ox{}, oz{}, ux{1.0}, uz{}, side{1.0};
	static Frame from_heading(double ox, double oz, double dx, double dz);
	Frame flipped() const;
	std::pair<double, double> v() const;
	std::pair<double, double> world(double a, double b) const;
	std::pair<double, double> local(double x, double z) const;
	std::pair<int, int> cell(double a, double b) const;
	std::vector<std::array<double, 4>> cells(
			double a0, double a1, double b0, double b1) const;
};

std::pair<int, int> axis_of(double dx, double dz);
const char *facing_name(std::pair<int, int> step);
BlockWithProperties with_prop(
		Block block, const std::string &key, const std::string &value);
BlockWithProperties with_props(
		Block block, const std::vector<std::pair<std::string, std::string>> &properties);
BlockWithProperties stair_backed(Block block, std::pair<int, int> back);
BlockWithProperties top_slab(Block block);
void place(WorldEditor &editor, const BlockWithProperties &block, int x, int y, int z);
std::optional<Block> concrete_for_colour(const std::string &text);
std::uint64_t mix(std::uint64_t a, std::uint64_t b);

using Point3 = std::array<double, 3>;
std::vector<std::pair<std::array<int, 3>, std::uint8_t>> line_cells(Point3 p, Point3 q);
void beam(WorldEditor &editor, Point3 p, Point3 q, double thickness,
		const std::string &display_block, std::int64_t seed, Block block, Block vertical);
void display_beam(WorldEditor &editor, Point3 p, Point3 q, double thickness,
		const std::string &display_block, std::int64_t seed);

} // namespace arnis::structure_kit
