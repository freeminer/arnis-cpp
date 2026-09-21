#pragma once

#include <array>
#include <cstdint>
#include <limits>

namespace arnis
{

class ChaCha8Rng
{
	std::array<std::uint32_t, 16> state_{};
	std::array<std::uint32_t, 16> block_{};
	std::size_t next_ = 16;
	static void quarter_round(
			std::uint32_t &a, std::uint32_t &b, std::uint32_t &c, std::uint32_t &d)
	{
		a += b;
		d ^= a;
		d = (d << 16) | (d >> 16);
		c += d;
		b ^= c;
		b = (b << 12) | (b >> 20);
		a += b;
		d ^= a;
		d = (d << 8) | (d >> 24);
		c += d;
		b ^= c;
		b = (b << 7) | (b >> 25);
	}
	void refill()
	{
		block_ = state_;
		auto work = state_;
		for (int i = 0; i < 4; ++i) {
			quarter_round(work[0], work[4], work[8], work[12]);
			quarter_round(work[1], work[5], work[9], work[13]);
			quarter_round(work[2], work[6], work[10], work[14]);
			quarter_round(work[3], work[7], work[11], work[15]);
			quarter_round(work[0], work[5], work[10], work[15]);
			quarter_round(work[1], work[6], work[11], work[12]);
			quarter_round(work[2], work[7], work[8], work[13]);
			quarter_round(work[3], work[4], work[9], work[14]);
		}
		for (std::size_t i = 0; i < 16; ++i)
			block_[i] += work[i];
		if (++state_[12] == 0)
			++state_[13];
		next_ = 0;
	}

public:
	using result_type = std::uint32_t;
	ChaCha8Rng() = default;
	explicit ChaCha8Rng(std::uint64_t seed)
	{
		state_ = {0x61707865, 0x3320646e, 0x79622d32, 0x6b206574};
		// rand_core::SeedableRng::seed_from_u64 (used by Rust's
		// rand_chacha::ChaCha8Rng) expands a u64 with SplitMix64, then writes
		// the resulting 32-byte seed little-endian into the ChaCha key.  The
		// previous LCG expansion produced deterministic but incompatible worlds.
		for (std::size_t i = 0; i < 4; ++i) {
			seed += 0x9e3779b97f4a7c15ULL;
			std::uint64_t z = seed;
			z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
			z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
			z ^= z >> 31;
			state_[4 + i * 2] = static_cast<std::uint32_t>(z);
			state_[5 + i * 2] = static_cast<std::uint32_t>(z >> 32);
		}
	}
	static constexpr result_type min() { return std::numeric_limits<result_type>::min(); }
	static constexpr result_type max() { return std::numeric_limits<result_type>::max(); }
	result_type operator()()
	{
		if (next_ == 16)
			refill();
		return block_[next_++];
	}
	bool random_bool(double probability = 0.5)
	{
		// rand::Rng::random_bool delegates to random::<f64>(); the standard
		// f64 sampler consumes a u64 and keeps its top 53 bits.  Consuming two
		// ChaCha words here preserves both the Rust sequence and probability.
		const std::uint64_t raw = (std::uint64_t((*this)()) << 32) | (*this)();
		const std::uint64_t mantissa = raw >> 11;
		return static_cast<long double>(mantissa) / 9007199254740992.0L < probability;
	}
	std::uint32_t uniform(std::uint32_t upper_exclusive)
	{
		// rand's UniformInt sampler uses Lemire's multiply-and-reject method,
		// rather than `%`, for random_range(0..upper_exclusive).
		if (upper_exclusive <= 1)
			return 0;
		const std::uint64_t range = upper_exclusive;
		const std::uint32_t threshold =
				static_cast<std::uint32_t>(-upper_exclusive) % upper_exclusive;
		for (;;) {
			const std::uint64_t product = std::uint64_t((*this)()) * range;
			const auto low = static_cast<std::uint32_t>(product);
			if (low >= threshold)
				return static_cast<std::uint32_t>(product >> 32);
		}
	}
};

inline ChaCha8Rng element_rng(std::uint64_t id)
{
	return ChaCha8Rng(id);
}
inline ChaCha8Rng element_rng_salted(std::uint64_t id, std::uint64_t salt)
{
	return ChaCha8Rng(id ^ ((salt << 32) | (salt >> 32)));
}
inline ChaCha8Rng coord_rng(std::int32_t x, std::int32_t z, std::uint64_t id)
{
	return ChaCha8Rng((std::uint64_t(std::uint32_t(x)) << 32 | std::uint32_t(z)) ^ id);
}

}
