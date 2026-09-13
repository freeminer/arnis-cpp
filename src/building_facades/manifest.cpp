#include "manifest.h"
#include <algorithm>
#include <cmath>
namespace arnis::building_facades
{
std::optional<FacadeSet> FacadeSet::validate(std::vector<Entry> entries)
{
	entries.erase(std::remove_if(entries.begin(), entries.end(),
						  [](const Entry &e) {
							  return e.file.empty() || e.categories.empty() ||
									 !std::isfinite(e.metres_wide) ||
									 !std::isfinite(e.metres_tall) ||
									 e.metres_wide <= 0 || e.metres_tall <= 0 ||
									 e.storeys == 0;
						  }),
			entries.end());
	if (entries.empty())
		return {};
	std::sort(entries.begin(), entries.end(),
			[](const Entry &a, const Entry &b) { return a.file < b.file; });
	entries.erase(
			std::unique(entries.begin(), entries.end(),
					[](const Entry &a, const Entry &b) { return a.file == b.file; }),
			entries.end());
	FacadeSet set;
	set.entries_ = std::move(entries);
	return set;
}
}
