#pragma once

#include "uses.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

struct XZBBox;
namespace arnis
{
namespace interior_uses
{
struct Claim
{
	std::vector<std::pair<int, int>> ring;
	double top_m{4.0};
};

class InteriorUseIndex
{
	std::unordered_map<std::uint64_t, std::vector<Tenant>> tenants_;
	std::unordered_map<std::uint64_t, Use> areas_;
	std::unordered_map<std::uint64_t, std::vector<Claim>> claims_;

public:
	static InteriorUseIndex build(
			const std::vector<ProcessedElement> &elements, const ::XZBBox &bbox);
	const std::vector<Tenant> &tenants(std::uint64_t id) const;
	std::optional<Use> area(std::uint64_t id) const;
	const std::vector<Claim> &claims(std::uint64_t id) const;
};

bool covers(const std::vector<std::pair<int, int>> &ring, int x, int z);
} // namespace interior_uses
} // namespace arnis
