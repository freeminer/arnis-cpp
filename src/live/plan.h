#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace arnis::live::plan
{
struct Square
{
	std::int32_t min_cx = 0;
	std::int32_t min_cz = 0;
	std::int32_t side = 0;

	bool contains(std::int32_t cx, std::int32_t cz) const;
	// Centre X/Z and width in blocks, as Minecraft's worldborder command takes them.
	std::tuple<double, double, double> border() const;
	std::int32_t inset(std::int32_t cx, std::int32_t cz) const;
};

using ChunkReady = std::function<bool(std::int32_t, std::int32_t)>;
bool chunk_is_safe(
		const ChunkReady &ready, std::int32_t cx, std::int32_t cz, std::int32_t margin);
std::optional<Square> safe_square(const ChunkReady &ready,
		const std::vector<std::pair<std::int32_t, std::int32_t>> &players,
		std::int32_t margin, std::int32_t max_half);

struct Mover
{
	double x = 0.0;
	double z = 0.0;
	double vx = 0.0;
	double vz = 0.0;
};

// Region coordinates in urgency order, matching Rust live::plan::wanted_regions.
std::vector<std::pair<std::int32_t, std::int32_t>> wanted_regions(
		const std::vector<Mover> &movers, double reach, double lead);
} // namespace arnis::live::plan
