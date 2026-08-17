#include "tree_library.h"
#include <algorithm>
#include <cctype>
namespace arnis::trees
{
TreeSize size_for_height(int h)
{
	return h <= 6	 ? TreeSize::Small
		   : h <= 12 ? TreeSize::Medium
		   : h <= 20 ? TreeSize::Big
		   : h <= 28 ? TreeSize::Tall
					 : TreeSize::Giant;
}
TreeSize size_for_canopy_m(std::uint8_t m)
{
	return size_for_height(m);
}
SizeFilter SizeFilter::up_to(TreeSize m)
{
	SizeFilter f;
	f.small = m >= TreeSize::Small;
	f.medium = m >= TreeSize::Medium;
	f.big = m >= TreeSize::Big;
	f.tall = m >= TreeSize::Tall;
	f.giant = m >= TreeSize::Giant;
	return f;
}
bool SizeFilter::allows(TreeSize s) const
{
	return s == TreeSize::Small	   ? small
		   : s == TreeSize::Medium ? medium
		   : s == TreeSize::Big	   ? big
		   : s == TreeSize::Tall   ? tall
								   : giant;
}
TreeSize smaller_enabled(TreeSize requested, const SizeFilter &f)
{
	for (int i = int(requested); i >= 0; --i) {
		auto s = TreeSize(i);
		if (f.allows(s))
			return s;
	}
	return TreeSize::Small;
}
TreeSize tree_size_from_string(const std::string &s)
{
	std::string n = s;
	for (char &c : n)
		c = char(std::tolower((unsigned char)c));
	return n == "small"	   ? TreeSize::Small
		   : n == "medium" ? TreeSize::Medium
		   : n == "big"	   ? TreeSize::Big
		   : n == "tall"   ? TreeSize::Tall
						   : TreeSize::Giant;
}
SizeFilter size_filter_from_string(const std::string &s)
{
	return SizeFilter::up_to(tree_size_from_string(s));
}
}
