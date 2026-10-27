#pragma once

#include <nlohmann/json_fwd.hpp>

#include <string>
#include <utility>
#include <vector>
#include <cstdint>
#include <optional>
#include <string_view>

#include "celestial.h"

namespace arnis::datapack::disasters
{
enum class SurfaceClass : std::uint8_t
{
	Pass,
	Solid,
	Fluid
};

enum class ColumnKind : std::uint8_t
{
	Land,
	Water,
	Outside
};

struct Column
{
	ColumnKind kind = ColumnKind::Outside;
	int y = 0;
};

// Sparse-to-dense snapshot of the generated block surface. Sampling spacing
// grows exactly as in Rust so disaster plans stay bounded for large regions.
class Surface
{
public:
	Surface(int min_x, int min_z, int max_x, int max_z);
	static SurfaceClass classify(std::string_view block_name);
	bool is_sample(int x, int z) const;
	void observe(int x, int y, int z, SurfaceClass block_class);
	Column column(int x, int z) const;
	std::optional<int> highest() const;
	int x_origin() const { return x0_; }
	int z_origin() const { return z0_; }
	int step() const { return step_; }
	std::size_t width() const { return width_; }
	std::size_t height() const { return height_; }
	int min_x() const { return min_x_; }
	int min_z() const { return min_z_; }
	int max_x() const { return max_x_; }
	int max_z() const { return max_z_; }

private:
	struct Cell
	{
		std::int16_t y = 0;
		ColumnKind kind = ColumnKind::Outside;
	};
	std::optional<std::size_t> index(int x, int z) const;
	int x0_ = 0, z0_ = 0, step_ = 1;
	int min_x_ = 0, min_z_ = 0, max_x_ = 0, max_z_ = 0;
	std::size_t width_ = 0, height_ = 0;
	std::vector<Cell> cells_;
};

struct WaterRect
{
	int x0 = 0, z0 = 0, x1 = 0, z1 = 0;
	int lo = 0, src = 0;
	std::uint8_t need = 0;
};

struct WaterCell
{
	int x = 0, z = 0;
	std::vector<WaterRect> rects;
};

struct WaterMap
{
	std::uint8_t levels = 0;
	std::vector<WaterCell> cells;
	std::size_t fill_count() const;
	std::string tops(const std::string &name) const;
	std::vector<std::pair<std::string, std::string>> cell_functions(
			const std::string &name) const;
};

// Rust Plan::without_maps counterpart. The generated functions deliberately
// omits water disasters when no recorded surface map exists.
class Plan
{
public:
	Plan(CelestialBody body, int floor, int world_max_y, bool wooded = false,
			const std::vector<std::pair<std::string, std::uint64_t>> &biomes = {},
			std::optional<int> highest_surface = std::nullopt,
			const Surface *surface = nullptr, std::optional<int> sea_y = std::nullopt);

	std::string coming() const;
	std::string summary() const;
	std::vector<std::pair<std::string, std::string>> files(const std::string &area) const;
	nlohmann::json dialog(const std::string &area, bool has_pause_menu) const;

private:
	CelestialBody body_;
	int floor_;
	int snow_top_;
	std::vector<int> kinds_;
	std::vector<std::pair<std::string, std::string>> snow_;
	std::optional<WaterMap> flood_, tsunami_;
};
} // namespace arnis::datapack::disasters
