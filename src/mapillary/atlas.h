#pragma once

#include <cstdint>

namespace arnis::mapillary::atlas
{
// Minecraft's practical block-atlas limits.  These match mapillary/atlas.rs.
inline constexpr std::uint32_t ATLAS_SIDE_STANDARD = 8192;
inline constexpr std::uint32_t ATLAS_SIDE_HIGH = 16384;
inline constexpr std::uint32_t MIN_PX_PER_BLOCK = 4;

// Select the atlas size used by this generation.  The setting is process-wide
// because facade panels and preset building facades share one atlas.
void set_atlas_side(std::uint32_t side);

// Number of pixels available to generated panels after conservative packing
// allowance.  This is intentionally public for panel sizing code.
std::uint64_t atlas_budget();

// Kept public for tests and callers that need to preview a settings value
// without changing the active generation.
std::uint64_t atlas_budget_for(std::uint32_t side);
}
