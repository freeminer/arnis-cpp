#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace arnis::water_probe
{
enum class Spot : std::uint8_t
{
	Open,
	Water,
	Solid,
	Half,
	Tall,
	Leaves
};

struct Sample
{
	std::optional<std::int16_t> water_y, overhead_y;
	bool operator==(const Sample &) const = default;
	static Sample make(std::optional<int> water, std::optional<int> overhead);
	std::optional<int> water() const;
	std::optional<int> clearance() const;
};

struct Footing
{
	std::int16_t base{};
	std::uint8_t levels{}, half{};
	bool is_thin{};
	bool operator==(const Footing &) const = default;
	std::optional<double> nearest(double y) const;
	bool thin() const { return is_thin; }
	bool post() const;
};

using Column = std::tuple<int, int, int>;
template <typename T>
class Probe;
using WaterProbe = Probe<Sample>;
using WalkProbe = Probe<Footing>;

template <typename T>
class Probe
{
	using Region = std::pair<int, int>;
	std::unordered_map<Region, std::vector<Column>,
			std::function<std::size_t(const Region &)>>
			wanted_{0, [](const Region &r) {
						const auto a = std::hash<int>{}(r.first);
						return a ^ (std::hash<int>{}(r.second) + 0x9e3779b9U + (a << 6) +
										   (a >> 2));
					}};
	mutable std::mutex mutex_;
	std::unordered_map<Region, std::vector<T>, typename decltype(wanted_)::hasher> read_{
			0, wanted_.hash_function()};
	std::unordered_map<std::pair<int, int>, T,
			std::function<std::size_t(const std::pair<int, int> &)>>
			round_{0, [](const std::pair<int, int> &p) {
					   const auto a = std::hash<int>{}(p.first);
					   return a ^ (std::hash<int>{}(p.second) + 0x9e3779b9U + (a << 6) +
										  (a >> 2));
				   }};

public:
	template <typename It>
	explicit Probe(It first, It last)
	{
		for (; first != last; ++first) {
			const auto &[x, z, y] = *first;
			wanted_[{x >> 9, z >> 9}].emplace_back(x, z, y);
		}
		for (auto &[region, cols] : wanted_) {
			std::sort(cols.begin(), cols.end());
			cols.erase(std::unique(cols.begin(), cols.end()), cols.end());
			cols.shrink_to_fit();
		}
	}
	explicit Probe(const std::vector<Column> &cols) : Probe(cols.begin(), cols.end()) {}
	std::vector<std::pair<int, int>> regions() const
	{
		std::vector<Region> out;
		out.reserve(wanted_.size());
		for (const auto &[r, _] : wanted_)
			out.push_back(r);
		return out;
	}
	template <typename Fn>
	void read_region(int rx, int rz, Fn column)
	{
		read_region_round(rx, rz, std::move(column), [](const T &) { return false; });
	}
	template <typename Fn, typename Round>
	void read_region_round(int rx, int rz, Fn column, Round round)
	{
		const Region region{rx, rz};
		const auto found = wanted_.find(region);
		if (found == wanted_.end())
			return;
		{
			std::lock_guard lock(mutex_);
			if (read_.contains(region))
				return;
		}
		const auto &cols = found->second;
		std::vector<T> samples;
		samples.reserve(cols.size());
		for (const auto &[x, z, y] : cols)
			samples.push_back(column(x, z, y));
		std::vector<std::pair<std::pair<int, int>, T>> extra;
		for (std::size_t i = 0; i < cols.size(); ++i) {
			if (!round(samples[i]))
				continue;
			const auto &[x, z, y] = cols[i];
			for (int dx = -2; dx <= 2; ++dx)
				for (int dz = -2; dz <= 2; ++dz) {
					const int nx = x + dx, nz = z + dz;
					if ((dx || dz) && (nx >> 9) == rx && (nz >> 9) == rz)
						extra.emplace_back(std::pair{nx, nz}, column(nx, nz, y));
				}
		}
		std::lock_guard lock(mutex_);
		for (auto &[p, sample] : extra)
			round_.insert_or_assign(p, std::move(sample));
		read_.try_emplace(region, std::move(samples));
	}
	std::optional<T> get(int x, int z, int y) const
	{
		const Region region{x >> 9, z >> 9};
		std::lock_guard lock(mutex_);
		if (const auto it = wanted_.find(region); it != wanted_.end()) {
			const Column key{x, z, y};
			const auto index =
					std::lower_bound(it->second.begin(), it->second.end(), key);
			if (index != it->second.end() && *index == key) {
				const auto read = read_.find(region);
				if (read != read_.end())
					return read->second[static_cast<std::size_t>(
							index - it->second.begin())];
			}
		}
		if (const auto it = round_.find({x, z}); it != round_.end())
			return it->second;
		return std::nullopt;
	}
};

std::pair<int, int> water_span(int expect);
Sample water_column(const std::function<Spot(int)> &block, int expect, int top);
Footing footing(const std::function<Spot(int)> &block, int expect);
Spot walk_spot(const std::string &name, bool bottom_slab);
} // namespace arnis::water_probe
