#include "plan.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace arnis::live::plan
{
bool Square::contains(std::int32_t cx, std::int32_t cz) const
{
	return cx >= min_cx && cz >= min_cz &&
		   static_cast<std::int64_t>(cx) < static_cast<std::int64_t>(min_cx) + side &&
		   static_cast<std::int64_t>(cz) < static_cast<std::int64_t>(min_cz) + side;
}

std::tuple<double, double, double> Square::border() const
{
	const double size = static_cast<double>(side) * 16.0;
	return {static_cast<double>(min_cx) * 16.0 + size / 2.0,
			static_cast<double>(min_cz) * 16.0 + size / 2.0, size};
}

std::int32_t Square::inset(std::int32_t cx, std::int32_t cz) const
{
	const auto last_x = static_cast<std::int64_t>(min_cx) + side - 1;
	const auto last_z = static_cast<std::int64_t>(min_cz) + side - 1;
	return static_cast<std::int32_t>(std::min({static_cast<std::int64_t>(cx) - min_cx,
			last_x - cx, static_cast<std::int64_t>(cz) - min_cz, last_z - cz}));
}

bool chunk_is_safe(
		const ChunkReady &ready, std::int32_t cx, std::int32_t cz, std::int32_t margin)
{
	if (!ready || margin < 0)
		return false;
	for (std::int64_t x = static_cast<std::int64_t>(cx) - margin;
			x <= static_cast<std::int64_t>(cx) + margin; ++x) {
		for (std::int64_t z = static_cast<std::int64_t>(cz) - margin;
				z <= static_cast<std::int64_t>(cz) + margin; ++z) {
			if (x < std::numeric_limits<std::int32_t>::min() ||
					x > std::numeric_limits<std::int32_t>::max() ||
					z < std::numeric_limits<std::int32_t>::min() ||
					z > std::numeric_limits<std::int32_t>::max() ||
					!ready(static_cast<std::int32_t>(x), static_cast<std::int32_t>(z)))
				return false;
		}
	}
	return true;
}

namespace
{
class Prefix
{
public:
	Prefix(std::size_t width, std::size_t height,
			const std::function<std::uint32_t(std::size_t, std::size_t)> &value) :
			width_(width), height_(height), stride_(width + 1),
			data_(stride_ * (height + 1))
	{
		for (std::size_t z = 0; z < height_; ++z) {
			std::uint32_t row = 0;
			for (std::size_t x = 0; x < width_; ++x) {
				row += value(x, z);
				data_[(z + 1) * stride_ + x + 1] = data_[z * stride_ + x + 1] + row;
			}
		}
	}

	std::uint32_t sum(std::size_t x, std::size_t z, std::size_t dx, std::size_t dz) const
	{
		const auto a = data_[(z + dz) * stride_ + x + dx];
		const auto b = data_[z * stride_ + x];
		const auto c = data_[z * stride_ + x + dx];
		const auto d = data_[(z + dz) * stride_ + x];
		return a + b - c - d;
	}

private:
	std::size_t width_, height_, stride_;
	std::vector<std::uint32_t> data_;
};
} // namespace

std::optional<Square> safe_square(const ChunkReady &ready,
		const std::vector<std::pair<std::int32_t, std::int32_t>> &players,
		std::int32_t margin, std::int32_t max_half)
{
	if (!ready || players.empty() || margin < 0 || max_half < 0)
		return std::nullopt;
	std::int64_t min_x = players.front().first, max_x = min_x;
	std::int64_t min_z = players.front().second, max_z = min_z;
	long double centre_x = 0.0L, centre_z = 0.0L;
	for (const auto &[x, z] : players) {
		min_x = std::min<std::int64_t>(min_x, x);
		max_x = std::max<std::int64_t>(max_x, x);
		min_z = std::min<std::int64_t>(min_z, z);
		max_z = std::max<std::int64_t>(max_z, z);
		centre_x += x;
		centre_z += z;
	}
	centre_x /= players.size();
	centre_z /= players.size();
	const std::int64_t spread = std::max(max_x - min_x, max_z - min_z) + 1;
	const std::int64_t max_side = static_cast<std::int64_t>(max_half) * 2 + 1;
	if (spread > max_side || spread > std::numeric_limits<std::int32_t>::max())
		return std::nullopt;

	const std::int64_t ix0 = max_x - max_side + 1;
	const std::int64_t iz0 = max_z - max_side + 1;
	const std::int64_t ix1 = min_x + max_side - 1;
	const std::int64_t iz1 = min_z + max_side - 1;
	const std::int64_t wx0 = ix0 - margin, wz0 = iz0 - margin;
	const auto iw = static_cast<std::size_t>(ix1 - ix0 + 1);
	const auto ih = static_cast<std::size_t>(iz1 - iz0 + 1);
	const auto m = static_cast<std::size_t>(margin);
	const auto w = iw + 2 * m, h = ih + 2 * m;
	if (w == 0 || h == 0 || w > std::numeric_limits<std::size_t>::max() / (h + 1) - 1)
		return std::nullopt;
	const Prefix not_ready(w, h, [&](std::size_t x, std::size_t z) {
		const std::int64_t cx = wx0 + static_cast<std::int64_t>(x);
		const std::int64_t cz = wz0 + static_cast<std::int64_t>(z);
		return (cx < std::numeric_limits<std::int32_t>::min() ||
					   cx > std::numeric_limits<std::int32_t>::max() ||
					   cz < std::numeric_limits<std::int32_t>::min() ||
					   cz > std::numeric_limits<std::int32_t>::max() ||
					   !ready(static_cast<std::int32_t>(cx),
							   static_cast<std::int32_t>(cz)))
					   ? 1u
					   : 0u;
	});
	const Prefix unsafe(iw, ih, [&](std::size_t x, std::size_t z) {
		return not_ready.sum(x, z, 2 * m + 1, 2 * m + 1) ? 1u : 0u;
	});

	auto best_at = [&](std::int64_t side) -> std::optional<Square> {
		std::optional<Square> best;
		long double best_distance = std::numeric_limits<long double>::infinity();
		for (std::int64_t a = max_x - side + 1; a <= min_x; ++a) {
			for (std::int64_t b = max_z - side + 1; b <= min_z; ++b) {
				const auto x = static_cast<std::size_t>(a - ix0);
				const auto z = static_cast<std::size_t>(b - iz0);
				if (unsafe.sum(x, z, static_cast<std::size_t>(side),
							static_cast<std::size_t>(side)) != 0)
					continue;
				const long double half = static_cast<long double>(side - 1) / 2.0L;
				const long double dx = static_cast<long double>(a) + half - centre_x;
				const long double dz = static_cast<long double>(b) + half - centre_z;
				const long double distance = dx * dx + dz * dz;
				if (distance < best_distance) {
					best_distance = distance;
					best = Square{static_cast<std::int32_t>(a),
							static_cast<std::int32_t>(b),
							static_cast<std::int32_t>(side)};
				}
			}
		}
		return best;
	};

	std::int64_t lo = spread, hi = max_side;
	auto found = best_at(lo);
	if (!found)
		return std::nullopt;
	while (lo < hi) {
		const auto mid = lo + (hi - lo + 1) / 2;
		if (auto square = best_at(mid)) {
			found = square;
			lo = mid;
		} else {
			hi = mid - 1;
		}
	}
	return found;
}

std::vector<std::pair<std::int32_t, std::int32_t>> wanted_regions(
		const std::vector<Mover> &movers, double reach, double lead)
{
	struct RegionHash
	{
		std::size_t operator()(
				const std::pair<std::int32_t, std::int32_t> &p) const noexcept
		{
			const auto a =
					static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.first));
			const auto b =
					static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.second));
			return static_cast<std::size_t>((a << 32) ^ b);
		}
	};
	if (!(reach > 0.0) || !std::isfinite(reach) || !std::isfinite(lead))
		return {};
	std::unordered_map<std::pair<std::int32_t, std::int32_t>, double, RegionHash> scores;
	for (const auto &m : movers) {
		const double speed = std::hypot(m.vx, m.vz);
		const double dx = speed > 0.5 ? m.vx / speed : 0.0;
		const double dz = speed > 0.5 ? m.vz / speed : 0.0;
		const double ahead_len = std::min(speed * lead, 2048.0);
		const auto steps = static_cast<std::int32_t>(
				std::max(0.0, std::ceil(ahead_len / (reach / 2.0))));
		for (std::int32_t step = 0; step <= steps; ++step) {
			const double t = steps == 0 ? 0.0 : static_cast<double>(step) / steps;
			const double px = m.x + dx * ahead_len * t;
			const double pz = m.z + dz * ahead_len * t;
			const auto floor_region = [](double value) -> std::int32_t {
				const double r = std::floor(value / 512.0);
				return static_cast<std::int32_t>(std::clamp(r,
						static_cast<double>(std::numeric_limits<std::int32_t>::min()),
						static_cast<double>(std::numeric_limits<std::int32_t>::max())));
			};
			const auto rx0 = floor_region(px - reach), rx1 = floor_region(px + reach);
			const auto rz0 = floor_region(pz - reach), rz1 = floor_region(pz + reach);
			for (std::int64_t rx = rx0; rx <= rx1; ++rx) {
				for (std::int64_t rz = rz0; rz <= rz1; ++rz) {
					const double cx = static_cast<double>(rx) * 512.0 + 256.0;
					const double cz = static_cast<double>(rz) * 512.0 + 256.0;
					const double ox = cx - m.x, oz = cz - m.z;
					const double ahead = std::max(ox * dx + oz * dz, 0.0);
					const double urgency = std::min(speed / 10.0, 1.0);
					const double score = std::hypot(ox, oz) - 0.6 * ahead * urgency;
					auto [it, inserted] =
							scores.try_emplace({static_cast<std::int32_t>(rx),
													   static_cast<std::int32_t>(rz)},
									score);
					if (!inserted)
						it->second = std::min(it->second, score);
				}
			}
		}
	}
	std::vector<std::pair<std::pair<std::int32_t, std::int32_t>, double>> ordered(
			scores.begin(), scores.end());
	std::sort(ordered.begin(), ordered.end(), [](const auto &a, const auto &b) {
		if (a.second != b.second)
			return a.second < b.second;
		return a.first < b.first;
	});
	std::vector<std::pair<std::int32_t, std::int32_t>> result;
	result.reserve(ordered.size());
	for (const auto &entry : ordered)
		result.push_back(entry.first);
	return result;
}
} // namespace arnis::live::plan
