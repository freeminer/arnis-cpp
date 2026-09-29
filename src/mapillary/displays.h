#pragma once
#include <array>
#include <cstdint>
#include <utility>
#include <vector>
#include <optional>
#include <string>
#include <functional>
#include <unordered_set>
namespace arnis::world_editor
{
struct WorldEditor;
}
namespace arnis::mapillary::displays
{
inline constexpr double PUSH_OUT = 0.06;
inline constexpr int MAX_PANEL = 32;
std::optional<std::pair<double, double>> outward_normal(
		std::pair<int, int> dir, int snx, int snz);
std::pair<double, double> right_of(std::pair<double, double> n);
double yaw_deg(std::pair<double, double> n);
std::array<float, 4> left_rotation(std::pair<double, double> n);
std::pair<double, double> facing_of(std::array<float, 4> q);
bool flip_crop(std::pair<int, int> dir, std::pair<double, double> n);
double cell_step(std::pair<int, int> dir);
// Backend-facing cardinal direction used by WorldEditor::place_facade_panel:
// 0=east, 1=north, 2=west, 3=south in the mapgen frame.
std::int8_t facing_index(std::pair<double, double> normal);
struct Quad
{
	double cx{}, cy{}, cz{}, w{}, h{};
	std::array<float, 4> rot{};
};
Quad quad_for(const std::vector<std::pair<int, int>> &cells, std::pair<double, double> n,
		double step, int base_y, int h);
std::vector<std::pair<int, int>> cut(int length, int maximum);
struct Piece
{
	int p0{}, p1{}, b0{}, b1{};
	Quad quad;
};
struct Panel
{
	std::string name;
	double w{}, h{};
	std::vector<std::uint8_t> pixels;
	std::size_t pixel_width{}, pixel_height{};
};
struct PlacementStats
{
	std::size_t candidates{}, displays{}, placed{}, dropped{};
};
struct Sprite
{
	std::uint32_t side_w{}, side_h{}, used_w{}, used_h{};
	static Sprite of(
			double width_blocks, double height_blocks, std::uint32_t pixels_per_block);
	std::pair<double, double> uv_extent() const;
	std::uint64_t area() const { return std::uint64_t(side_w) * side_h; }
};
std::vector<std::uint8_t> lay_out_rgb(const std::vector<std::uint8_t> &rgb,
		std::uint32_t input_width, std::uint32_t input_height, Sprite sprite);
std::string item_definition_json(const std::string &name);
std::string model_json_for(const std::string &texture_name, Sprite sprite);
inline std::string model_json(const std::string &name, Sprite sprite)
{
	return model_json_for(name, sprite);
}
std::vector<Piece> pieces(const std::vector<std::pair<int, int>> &cells,
		std::pair<double, double> normal, double step, int base_y, int total_height);
class Registry
{
	bool enabled_ = false;
	std::unordered_set<std::string> names_;
	PlacementStats stats_;

public:
	void reset(bool enabled)
	{
		enabled_ = enabled;
		names_.clear();
		stats_ = {};
	}
	bool enabled() const { return enabled_; }
	const PlacementStats &stats() const { return stats_; }
	bool collect(const std::string &name, const std::vector<std::pair<int, int>> &cells,
			std::pair<double, double> normal, double step, int base_y, int total_height,
			const std::vector<std::uint8_t> &rgb, std::uint32_t rgb_width,
			std::uint32_t rgb_height, std::uint32_t pixels_per_block,
			const std::function<bool(const Quad &, const Panel &)> &sink);
	bool collect_to_editor(arnis::world_editor::WorldEditor &, const std::string &name,
			const std::vector<std::pair<int, int>> &cells,
			std::pair<double, double> normal, double step, int base_y, int total_height,
			const std::vector<std::uint8_t> &rgb, std::uint32_t rgb_width,
			std::uint32_t rgb_height, std::uint32_t pixels_per_block);
};
}
