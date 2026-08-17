#pragma once
#include <cstdint>
#include <string>
namespace arnis::trees
{
enum class TreeSize
{
	Small,
	Medium,
	Big,
	Tall,
	Giant
};
TreeSize size_for_height(int height);
TreeSize size_for_canopy_m(std::uint8_t metres);
struct SizeFilter
{
	bool small = true, medium = true, big = true, tall = true, giant = true;
	static SizeFilter up_to(TreeSize max);
	bool allows(TreeSize size) const;
};
TreeSize smaller_enabled(TreeSize requested, const SizeFilter &filter);
TreeSize tree_size_from_string(const std::string &name);
SizeFilter size_filter_from_string(const std::string &name);
}
