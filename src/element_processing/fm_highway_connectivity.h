#pragma once
#include <cstdint>
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
using HighwayConnectivity =
		std::unordered_map<std::pair<int, int>, std::vector<int>, PairHash>;
HighwayConnectivity build_highway_connectivity_map(
		const std::vector<ProcessedElement> &elements);
}
}
