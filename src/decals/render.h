#pragma once
#include "draw.h"
#include "registry.h"
#include <cstdint>
#include <optional>
namespace arnis::decals
{
struct PreviewRaster
{
	const RgbaImage *image{};
	int min_x{};
	int min_z{};
	std::uint32_t step{1};
};

inline constexpr int LOCAL_MAP_HALF_SPAN = 96;

Canvas render(
		const DecalKey &key, const std::optional<PreviewRaster> &preview = std::nullopt);
}
