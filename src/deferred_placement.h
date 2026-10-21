#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <utility>
#include <vector>

namespace arnis
{

// Orders late placements by the regions they can touch. A placement is released
// only after every affected region has completed tile merging, and overlapping
// placements retain their original order.
class DeferredPlacements
{
	using Region = std::pair<int, int>;
	struct RegionHash
	{
		std::size_t operator()(const Region &r) const noexcept
		{
			const auto x =
					static_cast<std::uint64_t>(static_cast<std::uint32_t>(r.first));
			const auto z =
					static_cast<std::uint64_t>(static_cast<std::uint32_t>(r.second));
			return static_cast<std::size_t>((x << 32) ^ z);
		}
	};

	std::vector<std::vector<Region>> regions_;
	std::vector<std::size_t> waiting_;
	std::unordered_map<Region, std::deque<std::size_t>, RegionHash> queues_;
	std::vector<bool> placed_;

	bool runnable(std::size_t unit) const;
	std::vector<std::vector<std::size_t>> release(std::vector<std::size_t> candidates);

public:
	DeferredPlacements() = default;
	DeferredPlacements(std::vector<std::vector<Region>> placements,
			const std::vector<Region> &world_regions);
	bool holds(Region region) const;
	std::vector<Region> regions() const;
	const std::vector<Region> &regions_of(std::size_t unit) const;
	std::vector<std::vector<std::size_t>> complete(Region region);
	std::vector<std::size_t> drain();
};

} // namespace arnis
