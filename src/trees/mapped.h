#pragma once

#include "../element_processing/tree.h"
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
	std::string genus;
	std::optional<bool> conifer;
	double height_m{0};
};
bool is_conifer_genus(const std::string &genus);
MappedTree from_tags(
		const std::unordered_map<std::string, std::string> &tags, std::uint64_t id);
bool is_tree_row(const std::unordered_map<std::string, std::string> &tags);
std::vector<std::pair<int, int>> tree_row_positions(
		const std::vector<std::pair<int, int>> &nodes, double scale);

class MappedTrunks
{
public:
	static MappedTrunks collect(
			const std::vector<ProcessedElement> &elements, double scale);
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
};
}
