#pragma once

#include "../../../arnis_adapter.h"

#include <optional>
#include <functional>
#include <utility>
#include <vector>

namespace arnis::bridge_styles
{
struct BridgePathSample;
}

namespace arnis::bridge_towers
{
inline constexpr int SADDLE_BELOW_TOP = 3;

struct MappedTower
{
	int x = 0;
	int z = 0;
	int top_y = 0;
	std::optional<int> deck_y;
	Block paint{};
};

struct MappedSupports
{
	std::vector<MappedTower> towers;
	std::vector<std::pair<int, int>> minor;
};

struct CableSpan
{
	float origin_x = 0.0f;
	float origin_z = 0.0f;
	float dir_x = 0.0f;
	float dir_z = 0.0f;
	std::vector<std::pair<float, float>> points;
	std::vector<float> sags;
	float hanger_spacing = 2.0f;
	Block paint{};
	int girder_depth = 2;

	float u(int x, int z) const;
	std::optional<float> cable_y(float u) const;
	long long hanger_bay(float u) const;
	bool suspends(int x, int z) const;
};

std::optional<CableSpan> make_cable_span(float origin_x, float origin_z, float dir_x,
		float dir_z, const std::vector<std::pair<float, int>> &towers,
		std::pair<float, float> anchors, const std::function<int(float)> &deck_at,
		Block paint, double scale);
void decorate(world_editor::WorldEditor &editor,
		const std::vector<bridge_styles::BridgePathSample> &path,
		const std::function<std::vector<std::pair<std::size_t, std::pair<int, int>>>(
				std::size_t)> &sides,
		const CableSpan &span, Block hanger);

bool is_support(const tags_t &tags);
Block support_block(const tags_t &tags);
void place_support(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int, int>> &cells, int base_y, int height,
		bool grounded);
MappedSupports collect(const std::vector<ProcessedElement> &elements,
		const world_editor::WorldEditor &editor, double scale);
}
