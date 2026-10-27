#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>
namespace arnis
{
class ProcessedElement;
namespace highways
{
struct PairHash
{
	std::size_t operator()(const std::pair<int, int> &p) const noexcept
	{
		return std::hash<std::uint64_t>{}(
				(std::uint64_t(std::uint32_t(p.first)) << 32) | std::uint32_t(p.second));
	}
};
struct HighwayConnectivity
{
	using Ends = std::unordered_map<std::pair<int, int>, std::vector<int>, PairHash>;
	Ends ends;
	// Packed/sorted coordinates which occur in more than one distinct way.
	std::vector<std::uint64_t> shared;
	// One World bounds; profile pins here keep adjacent generation areas aligned.
	std::optional<std::array<int, 4>> area_bounds;

	bool empty() const { return ends.empty(); }
	bool is_shared(int x, int z) const noexcept
	{
		const auto packed = (std::uint64_t(std::uint32_t(x)) << 32) | std::uint32_t(z);
		return std::binary_search(shared.begin(), shared.end(), packed);
	}
};
HighwayConnectivity build_highway_connectivity_map(
		const std::vector<ProcessedElement> &elements,
		std::optional<std::array<int, 4>> area_bounds = std::nullopt);
}
}
