#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <optional>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

struct XZBBox;

namespace arnis::tunnel_grade
{

inline constexpr int FLOOR_GUARD = 3;
inline constexpr std::size_t CUTTING_GAP = 5;

int lowest_floor();

struct Bore
{
	double grade{};
	double depth{};
	double lowest{};
	static Bore make(std::size_t cells, int depth, double least, double most, int lowest);
};

std::vector<double> grade_line(const std::vector<double> &ceiling,
		const std::vector<double> &top, const std::vector<std::optional<double>> &pins,
		std::array<bool, 2> portal, const Bore &bore);

std::vector<bool> portal_cuttings(
		const std::vector<bool> &bare, std::array<bool, 2> portal, std::size_t gap);

void pin_area_edges(std::vector<std::optional<int>> &pins,
		const std::vector<std::pair<int, int>> &cells, const ::XZBBox *area,
		const std::function<int(std::size_t)> &level);

void walkable(
		std::vector<int> &levels, const std::vector<std::optional<int>> &pins, int step);

class Beside
{
public:
	void add(const std::vector<std::pair<int, int>> &path,
			const std::vector<double> &levels);
	std::vector<std::optional<double>> pins(const std::vector<std::pair<int, int>> &path,
			const std::vector<double> &line, int reach, std::array<bool, 2> portal) const;

private:
	struct CellHash
	{
		std::size_t operator()(const std::pair<int, int> &cell) const noexcept;
	};
	std::unordered_map<std::pair<int, int>, std::pair<double, std::pair<int, int>>,
			CellHash>
			cells_;
	std::unordered_set<std::pair<int, int>, CellHash> blocks_;
	bool near(int x, int z, int reach) const;
};

std::vector<std::vector<std::pair<std::size_t, bool>>> chains(std::size_t count,
		const std::function<std::optional<std::pair<std::size_t, std::size_t>>(
				std::size_t, std::size_t)> &partner);

std::vector<std::size_t> planning_order(
		const std::vector<std::pair<std::size_t, std::uint64_t>> &size,
		const std::vector<std::vector<std::uint64_t>> &nodes);

} // namespace arnis::tunnel_grade
