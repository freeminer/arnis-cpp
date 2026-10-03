#include "posters.h"
#include "../assets_root.h"
namespace arnis::decals::posters
{
std::filesystem::path billboard(std::uint8_t v, const std::filesystem::path &root)
{
	const auto &directory = root.empty() ? assets::path("decorations/posters") : root;
	return directory / ("billboard_" + std::to_string(v % BILLBOARD_COUNT) + ".png");
}
std::filesystem::path column(std::uint8_t v, const std::filesystem::path &root)
{
	const auto &directory = root.empty() ? assets::path("decorations/posters") : root;
	return directory / ("column_" + std::to_string(v % COLUMN_COUNT) + ".png");
}
}
