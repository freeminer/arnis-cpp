#include "deferred_placement.h"

#include <algorithm>
#include <cstdint>
#include <utility>

namespace arnis
{

DeferredPlacements::DeferredPlacements(std::vector<std::vector<Region>> placements,
		const std::vector<Region> &world_regions) : regions_(std::move(placements))
{
	std::vector<Region> sorted_world = world_regions;
	std::sort(sorted_world.begin(), sorted_world.end());
	sorted_world.erase(
			std::unique(sorted_world.begin(), sorted_world.end()), sorted_world.end());
	waiting_.resize(regions_.size());
	placed_.resize(regions_.size(), false);
	for (std::size_t unit = 0; unit < regions_.size(); ++unit) {
		auto &regions = regions_[unit];
		std::sort(regions.begin(), regions.end());
		regions.erase(std::unique(regions.begin(), regions.end()), regions.end());
		regions.erase(std::remove_if(regions.begin(), regions.end(),
							  [&](Region r) {
								  return !std::binary_search(
										  sorted_world.begin(), sorted_world.end(), r);
							  }),
				regions.end());
		waiting_[unit] = regions.size();
		for (const auto &region : regions)
			queues_[region].push_back(unit);
	}
}

bool DeferredPlacements::holds(Region region) const
{
	const auto it = queues_.find(region);
	return it != queues_.end() && !it->second.empty();
}

std::vector<DeferredPlacements::Region> DeferredPlacements::regions() const
{
	std::vector<Region> result;
	result.reserve(queues_.size());
	for (const auto &[region, queue] : queues_) {
		(void)queue;
		result.push_back(region);
	}
	return result;
}

const std::vector<DeferredPlacements::Region> &DeferredPlacements::regions_of(
		std::size_t unit) const
{
	return regions_.at(unit);
}

bool DeferredPlacements::runnable(std::size_t unit) const
{
	if (waiting_[unit] != 0)
		return false;
	for (const auto &region : regions_[unit]) {
		const auto it = queues_.find(region);
		if (it == queues_.end() || it->second.empty() || it->second.front() != unit)
			return false;
	}
	return true;
}

std::vector<std::vector<std::size_t>> DeferredPlacements::complete(Region region)
{
	auto it = queues_.find(region);
	if (it == queues_.end())
		return {};
	for (const auto unit : it->second)
		if (waiting_[unit] > 0)
			--waiting_[unit];
	std::vector<std::size_t> candidates;
	if (!it->second.empty())
		candidates.push_back(it->second.front());
	return release(std::move(candidates));
}

std::vector<std::vector<std::size_t>> DeferredPlacements::release(
		std::vector<std::size_t> candidates)
{
	std::vector<std::vector<std::size_t>> waves;
	for (;;) {
		std::sort(candidates.begin(), candidates.end());
		candidates.erase(
				std::unique(candidates.begin(), candidates.end()), candidates.end());
		std::vector<std::size_t> wave;
		for (const auto unit : candidates)
			if (!placed_[unit] && runnable(unit))
				wave.push_back(unit);
		if (wave.empty())
			return waves;
		candidates.clear();
		for (const auto unit : wave) {
			placed_[unit] = true;
			for (const auto &region : regions_[unit]) {
				auto &queue = queues_.at(region);
				if (!queue.empty() && queue.front() == unit)
					queue.pop_front();
				if (!queue.empty())
					candidates.push_back(queue.front());
			}
		}
		waves.push_back(std::move(wave));
	}
}

std::vector<std::size_t> DeferredPlacements::drain()
{
	queues_.clear();
	std::vector<std::size_t> remaining;
	for (std::size_t unit = 0; unit < placed_.size(); ++unit)
		if (!placed_[unit]) {
			placed_[unit] = true;
			remaining.push_back(unit);
		}
	return remaining;
}

} // namespace arnis
