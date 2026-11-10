#pragma once

#include "../../../arnis_types.h"
#include "../coordinate_system/cartesian/xzbbox/xzbbox_enum.h"
#include "../water_probe.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <map>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <tuple>
#include <utility>
#include <vector>

namespace arnis::element_processing::pedestrians
{
/// Rust's density rule: people per metre per direction for a tagged way.
std::optional<double> walk_density(const tags_t &tags);
bool off_limits(const tags_t &tags);
bool draws_people(const tags_t &tags);
std::vector<std::pair<ProcessedWay, double>> across_squares(
		const std::vector<ProcessedElement> &elements, double scale);

struct Point
{
	double x{}, y{}, z{};
	std::optional<std::uint8_t> light;
	bool kerb{};
	bool operator==(const Point &) const = default;
};

class WalkNetwork
{
public:
	using SurfaceY = std::function<int(const ProcessedWay &, int, int)>;
	static WalkNetwork build(const std::vector<ProcessedElement> &elements, double scale,
			bool keep_left, bool signals, const cartesian::XZBBox &world,
			const SurfaceY &surface_y);
	bool is_empty() const;
	std::size_t lane_count() const;
	std::shared_ptr<water_probe::WalkProbe> probe() const;
	void finish(const water_probe::WalkProbe &probe);
	std::vector<std::pair<std::string, std::string>> pack_functions() const;

private:
	struct Lane
	{
		std::vector<Point> points;
		std::vector<Point> alternate;
		std::vector<Point> plain;
		std::vector<std::size_t> next;
		std::optional<std::size_t> back;
		std::pair<bool, bool> cut{};
		std::vector<std::optional<std::size_t>> beside;
		double density{};
		bool across{};
		bool out{};
	};
	using Meta = std::tuple<std::size_t, bool, std::uint64_t, std::uint64_t, bool>;
	void link();
	void untrap();
	void build_spots();
	std::vector<Lane> lanes_;
	std::vector<Meta> meta_;
	std::map<std::pair<int, int>, std::vector<std::pair<std::size_t, std::size_t>>>
			spots_;
	double size_{1.0};
	double scale_{1.0};
	bool keep_left_{};
};

class Footfall
{
public:
	Footfall(
			const std::vector<std::pair<double, double>> &destinations, double cell_size);
	double at(std::pair<double, double> point) const;

private:
	struct CellHash
	{
		std::size_t operator()(const std::pair<int, int> &p) const noexcept;
	};
	double cell_size_{};
	std::unordered_map<std::pair<int, int>, std::uint32_t, CellHash> counts_;
};

/// Footprint of mapped street furniture that a walking lane should avoid.
struct Item
{
	double x{}, z{}, hx{}, hz{}, along{};
};
std::optional<Item> furniture(const ProcessedNode &node);

/// Mapped pedestrian squares/plazas, retaining holes and their outline node ids.
struct Square
{
	std::vector<std::vector<std::pair<double, double>>> rings;
	std::unordered_set<std::uint64_t> outline;
	std::pair<double, double> min{}, max{};
	double people{};

	static std::optional<Square> of(const ProcessedElement &element, double scale);
	bool contains(std::pair<double, double> point) const;
	bool near(std::pair<double, double> point, double reach) const;
	bool by_edge(std::pair<double, double> point, double reach) const;
};

/// Spatially indexed mapped furniture and the Rust-equivalent lane detour operation.
class Furniture
{
public:
	explicit Furniture(std::vector<Item> items = {});
	std::vector<Point> go_round(std::vector<Point> points) const;

private:
	struct CellHash
	{
		std::size_t operator()(const std::pair<int, int> &p) const noexcept;
	};
	std::vector<Item> items_;
	std::unordered_map<std::pair<int, int>, std::vector<std::uint32_t>, CellHash> cells_;
};
} // namespace arnis::element_processing::pedestrians
