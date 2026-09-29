#include "caves_rng.h"
#include <cassert>
#include <bit>
#include <limits>

namespace arnis::caves
{
namespace
{
constexpr std::int64_t GOLDEN = -7046029254386353131LL;
constexpr std::int64_t SILVER = 7640891576956012809LL;
}
std::int64_t XoroRandom::mix(std::int64_t s)
{
	s = (s ^ std::int64_t(std::uint64_t(s) >> 30)) * -4658895280553007687LL;
	s = (s ^ std::int64_t(std::uint64_t(s) >> 27)) * -7723592293110705685LL;
	return s ^ std::int64_t(std::uint64_t(s) >> 31);
}
Seed128 XoroRandom::upgrade(std::int64_t seed)
{
	const auto lo = seed ^ SILVER;
	return {mix(lo), mix(lo + GOLDEN)};
}
XoroRandom::XoroRandom(std::int64_t lo, std::int64_t hi) : lo_(lo), hi_(hi)
{
	if ((lo_ | hi_) == 0) {
		lo_ = GOLDEN;
		hi_ = SILVER;
	}
}
XoroRandom XoroRandom::from_seed(std::int64_t seed)
{
	const auto s = upgrade(seed);
	return {s.lo, s.hi};
}
std::int64_t XoroRandom::next_long()
{
	// Keep every rotate/add/xor in unsigned arithmetic.  Rust uses wrapping
	// i64 operations here; doing the same through signed C++ overflow is UB and
	// can produce different cave noise under aggressive optimisation.
	const auto l = static_cast<std::uint64_t>(lo_);
	auto l1 = static_cast<std::uint64_t>(hi_);
	const auto sum = l + l1;
	const auto out = static_cast<std::int64_t>(std::rotl(sum, 17) + l);
	l1 ^= l;
	lo_ = static_cast<std::int64_t>(std::rotl(l, 49) ^ l1 ^ (l1 << 21));
	hi_ = static_cast<std::int64_t>(std::rotl(l1, 28));
	return out;
}
double XoroRandom::next_double()
{
	const auto bits = std::uint64_t(next_long()) >> 11;
	return static_cast<double>(static_cast<float>(bits * 1.1102230246251565e-16f));
}
float XoroRandom::next_float()
{
	return float(std::uint64_t(next_long()) >> 40) * 5.9604645e-8f;
}
std::int32_t XoroRandom::next_int(std::int32_t bound)
{
	assert(bound > 0);
	const std::uint64_t b = bound;
	std::uint64_t l = std::uint32_t(next_long()), p = l * b, low = p & 0xffffffffu;
	if (low < b) {
		const auto threshold = ((std::uint64_t(1) << 32) - b) % b;
		while (low < threshold) {
			l = std::uint32_t(next_long());
			p = l * b;
			low = p & 0xffffffffu;
		}
	}
	return std::int32_t(p >> 32);
}
std::pair<std::int64_t, std::int64_t> XoroRandom::fork_positional()
{
	return {next_long(), next_long()};
}
}
