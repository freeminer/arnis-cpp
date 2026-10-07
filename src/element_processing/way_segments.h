#pragma once

#include "../../../arnis_adapter.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>

namespace arnis
{
inline void merge_way_segments(std::vector<std::vector<ProcessedNode>> &rings)
{
	auto same_point = [](const ProcessedNode &a, const ProcessedNode &b) {
		return a.id == b.id || (std::abs(a.x - b.x) <= 1 && std::abs(a.z - b.z) <= 1);
	};
	std::vector<bool> removed(rings.size(), false);
	std::vector<std::vector<ProcessedNode>> merged;
	for (std::size_t i = 0; i < rings.size(); ++i)
		for (std::size_t j = 0; j < rings.size(); ++j) {
			if (i == j || removed[i] || removed[j] || rings[i].empty() ||
					rings[j].empty())
				continue;
			const auto &x = rings[i];
			const auto &y = rings[j];
			if (same_point(x.front(), x.back()) || same_point(y.front(), y.back()))
				continue;
			if (same_point(x.front(), y.front())) {
				removed[i] = removed[j] = true;
				auto result = x;
				std::reverse(result.begin(), result.end());
				result.insert(result.end(), std::next(y.begin()), y.end());
				merged.push_back(std::move(result));
			} else if (same_point(x.back(), y.back())) {
				removed[i] = removed[j] = true;
				auto result = x;
				result.insert(result.end(), std::next(y.rbegin()), y.rend());
				merged.push_back(std::move(result));
			} else if (same_point(x.front(), y.back())) {
				removed[i] = removed[j] = true;
				auto result = y;
				result.insert(result.end(), std::next(x.begin()), x.end());
				merged.push_back(std::move(result));
			} else if (same_point(x.back(), y.front())) {
				removed[i] = removed[j] = true;
				auto result = x;
				result.insert(result.end(), std::next(y.begin()), y.end());
				merged.push_back(std::move(result));
			}
		}
	for (std::size_t i = removed.size(); i > 0; --i)
		if (removed[i - 1])
			rings.erase(rings.begin() + static_cast<std::ptrdiff_t>(i - 1));
	const auto merged_count = merged.size();
	for (auto &ring : merged)
		rings.push_back(std::move(ring));
	if (merged_count > 0)
		merge_way_segments(rings);
}
} // namespace arnis
