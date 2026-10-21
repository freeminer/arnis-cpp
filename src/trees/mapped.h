#pragma once

#include "../element_processing/tree.h"
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace arnis::trees::mapped
{
struct MappedTree
{
	TreeType kind{TreeType::Oak};
	std::optional<std::string> genus;
	std::optional<bool> conifer;
	std::optional<double> height_m;

	static MappedTree from_tags(
			const std::unordered_map<std::string, std::string> &tags, std::uint64_t id);
};
bool is_conifer_genus(const std::string &genus);
bool is_tree_row(const std::unordered_map<std::string, std::string> &tags);
std::vector<std::pair<int, int>> tree_row_positions(
		const std::vector<std::pair<int, int>> &nodes, double scale);

class MappedTrunks
{
public:
	static MappedTrunks collect(
			const std::vector<ProcessedElement> &elements, double scale);
	void measure_typical_height(
			const std::function<std::optional<std::uint8_t>(int, int)> &canopy_height);
	std::optional<double> typical_height_m() const { return typical_height_m_; }
	bool under_crown(int x, int z) const;

private:
	struct Trunk
	{
		int cell_x, cell_z, x, z;
		bool operator<(const Trunk &other) const
		{
			return std::tie(cell_x, cell_z, x, z) <
				   std::tie(other.cell_x, other.cell_z, other.x, other.z);
		}
	};
	std::vector<Trunk> trunks_;
	int radius_{1};
	std::optional<double> typical_height_m_;
};
}
