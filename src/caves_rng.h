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

// Match caves/{carver,water,schems}.rs chunk_rng: coordinate multiplications
// wrap as signed i64 in Rust. Do the arithmetic as u64 to avoid C++ signed
// overflow while preserving the exact two's-complement bit pattern.
std::int64_t chunk_seed(
		std::int64_t seed, std::int32_t cx, std::int32_t cz, std::int64_t salt);

class XoroRandom;

class PositionalFactory
{
	std::int64_t lo_, hi_;
	explicit PositionalFactory(std::int64_t lo, std::int64_t hi) : lo_(lo), hi_(hi) {}
	friend class XoroRandom;

public:
	XoroRandom from_hash_of(std::string_view name) const;
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
	PositionalFactory fork_positional();
};
}
