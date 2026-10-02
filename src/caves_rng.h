#pragma once
#include <cstdint>
#include <string_view>
#include <utility>

namespace arnis::caves
{
struct Seed128
{
	std::int64_t lo, hi;
};
class XoroRandom
{
	std::int64_t lo_, hi_;
	static std::int64_t mix(std::int64_t);
	static Seed128 upgrade(std::int64_t);
	XoroRandom(std::int64_t lo, std::int64_t hi);

public:
	static XoroRandom from_seed(std::int64_t seed);
	static XoroRandom from_hash_of(
			std::int64_t factory_lo, std::int64_t factory_hi, std::string_view name);
	std::int64_t next_long();
	double next_double();
	float next_float();
	std::int32_t next_int(std::int32_t bound);
	std::pair<std::int64_t, std::int64_t> fork_positional();
};
}
