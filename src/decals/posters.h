#pragma once
#include <cstdint>
#include <filesystem>
#include <utility>
namespace arnis::decals::posters
{
inline constexpr std::pair<std::uint32_t, std::uint32_t> BILLBOARD_TILES{3, 2},
		COLUMN_TILES{1, 2};
inline constexpr std::uint8_t BILLBOARD_COUNT = 6, COLUMN_COUNT = 5;
std::filesystem::path billboard(std::uint8_t variant,
		const std::filesystem::path &root = "assets/decorations/posters");
std::filesystem::path column(std::uint8_t variant,
		const std::filesystem::path &root = "assets/decorations/posters");
}
