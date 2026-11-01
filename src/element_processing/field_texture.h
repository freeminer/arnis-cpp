#pragma once

#include "../block_definitions.h"
#include "../../../arnis_block.h"

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

namespace arnis
{
struct ProcessedNode;
namespace world_editor
{
struct WorldEditor;
}
}

namespace arnis::field_texture
{

enum class FieldPreset : std::uint8_t
{
	Classic,
	Smallholding,
	Patchwork,
	Prairie
};

FieldPreset field_preset_from_string(std::string_view value);
std::string_view field_preset_name(FieldPreset preset);

enum class Crop : std::uint8_t
{
	Wheat,
	Potatoes,
	Carrots,
	Beetroots,
	Sunflowers,
	Pumpkins,
	Fallow
};

struct Cover
{
	Block lower;
	std::optional<Block> upper;
};

struct Plot
{
	Crop crop{Crop::Wheat};
	bool young{};
	std::uint64_t seed{};
	bool rows_along_x{};

	Block surface(int x, int z) const;
	std::optional<Cover> cover(Block surface_block, int x, int z) const;
	bool on_row(int x, int z) const;
	Block crop_block(int x, int z, std::uint64_t roll) const;
};

extern const std::array<Block, 14> PLOT_PLANTS;
void place_cover(world_editor::WorldEditor &editor, const Cover &cover, int x,
		int ground_y, int z);

class FieldLayout
{
public:
	static std::optional<FieldLayout> make(FieldPreset preset, double scale,
			std::uint64_t id, const std::vector<ProcessedNode> &nodes);
	Plot plot_at(int x, int z) const;

private:
	std::array<std::uint32_t, 7> weights_{};
	double strip_min_m_{};
	double strip_max_m_{};
	double length_m_{};
	std::uint64_t id_{};
	double across_x_{}, across_z_{}, along_x_{}, along_z_{};
	double across_min_{}, along_min_{}, strip_{}, length_{};
	std::int64_t n_across_{}, n_along_{};
	bool rows_along_x_{};
	mutable std::optional<std::pair<std::int64_t, double>> last_strip_;
	mutable std::optional<std::pair<std::pair<std::int64_t, std::int64_t>, Plot>>
			last_plot_;
};

class CroplandGrid
{
public:
	static std::optional<CroplandGrid> make(FieldPreset preset, double scale);
	Plot plot_at(int x, int z) const;

private:
	std::array<std::uint32_t, 7> weights_{};
	double strip_min_m_{}, strip_max_m_{}, length_m_{}, scale_{};
	int block_{};
	struct StripBlock
	{
		std::pair<int, int> key;
		std::uint64_t hash{};
		bool rows_along_x{};
		int width{};
		int phase{};
	};
	mutable std::optional<StripBlock> last_block_;
	mutable std::optional<std::pair<std::pair<std::uint64_t, int>, Plot>> last_plot_;
};

} // namespace arnis::field_texture
