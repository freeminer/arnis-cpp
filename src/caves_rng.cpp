#include "caves_rng.h"
#include <cassert>
#include <array>
#include <bit>
#include <limits>
#include <string_view>
#include <vector>

namespace arnis::caves
{
namespace
{
constexpr std::int64_t GOLDEN = -7046029254386353131LL;
constexpr std::int64_t SILVER = 7640891576956012809LL;

std::array<std::uint8_t, 16> md5(std::string_view input)
{
	constexpr std::array<std::uint32_t, 64> shifts{7, 12, 17, 22, 7, 12, 17, 22, 7, 12,
			17, 22, 7, 12, 17, 22, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20,
			4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 6, 10, 15, 21, 6,
			10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};
	constexpr std::array<std::uint32_t, 64> constants{0xd76aa478, 0xe8c7b756, 0x242070db,
			0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501, 0x698098d8,
			0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e,
			0x49b40821, 0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d,
			0x02441453, 0xd8a1e681, 0xe7d3fbc8, 0x21e1cde6, 0xc33707d6, 0xf4d50d87,
			0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a, 0xfffa3942,
			0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60,
			0xbebfbc70, 0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039,
			0xe6db99e5, 0x1fa27cf8, 0xc4ac5665, 0xf4292244, 0x432aff97, 0xab9423a7,
			0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1, 0x6fa87e4f,
			0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb,
			0xeb86d391};
	std::vector<std::uint8_t> data(input.begin(), input.end());
	const std::uint64_t bit_length = std::uint64_t(data.size()) * 8;
	data.push_back(0x80);
	while (data.size() % 64 != 56)
		data.push_back(0);
	for (unsigned i = 0; i < 8; ++i)
		data.push_back(std::uint8_t(bit_length >> (8 * i)));

	std::uint32_t a0 = 0x67452301, b0 = 0xefcdab89, c0 = 0x98badcfe, d0 = 0x10325476;
	for (std::size_t offset = 0; offset < data.size(); offset += 64) {
		std::array<std::uint32_t, 16> words{};
		for (unsigned i = 0; i < words.size(); ++i) {
			const auto p = offset + i * 4;
			words[i] = std::uint32_t(data[p]) | (std::uint32_t(data[p + 1]) << 8) |
					   (std::uint32_t(data[p + 2]) << 16) |
					   (std::uint32_t(data[p + 3]) << 24);
		}
		auto [a, b, c, d] = std::array{a0, b0, c0, d0};
		for (unsigned i = 0; i < 64; ++i) {
			std::uint32_t f;
			unsigned g;
			if (i < 16) {
				f = (b & c) | (~b & d);
				g = i;
			} else if (i < 32) {
				f = (d & b) | (~d & c);
				g = (5 * i + 1) % 16;
			} else if (i < 48) {
				f = b ^ c ^ d;
				g = (3 * i + 5) % 16;
			} else {
				f = c ^ (b | ~d);
				g = (7 * i) % 16;
			}
			f += a + constants[i] + words[g];
			a = d;
			d = c;
			c = b;
			b += std::rotl(f, int(shifts[i]));
		}
		a0 += a;
		b0 += b;
		c0 += c;
		d0 += d;
	}
	std::array<std::uint8_t, 16> digest{};
	const std::array state{a0, b0, c0, d0};
	for (unsigned i = 0; i < state.size(); ++i)
		for (unsigned j = 0; j < 4; ++j)
			digest[i * 4 + j] = std::uint8_t(state[i] >> (8 * j));
	return digest;
}

std::int64_t big_endian_i64(const std::uint8_t *bytes)
{
	std::uint64_t value = 0;
	for (unsigned i = 0; i < 8; ++i)
		value = (value << 8) | bytes[i];
	return std::bit_cast<std::int64_t>(value);
}
}
std::int64_t XoroRandom::mix(std::int64_t s)
{
	auto value = std::bit_cast<std::uint64_t>(s);
	value = (value ^ (value >> 30)) * static_cast<std::uint64_t>(-4658895280553007687LL);
	value = (value ^ (value >> 27)) * static_cast<std::uint64_t>(-7723592293110705685LL);
	return std::bit_cast<std::int64_t>(value ^ (value >> 31));
}
Seed128 XoroRandom::upgrade(std::int64_t seed)
{
	const auto lo = seed ^ SILVER;
	const auto hi = std::bit_cast<std::int64_t>(
			std::bit_cast<std::uint64_t>(lo) + static_cast<std::uint64_t>(GOLDEN));
	return {mix(lo), mix(hi)};
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
XoroRandom XoroRandom::from_hash_of(
		std::int64_t factory_lo, std::int64_t factory_hi, std::string_view name)
{
	const auto digest = md5(name);
	return {big_endian_i64(digest.data()) ^ factory_lo,
			big_endian_i64(digest.data() + 8) ^ factory_hi};
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
