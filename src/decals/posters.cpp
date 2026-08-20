#include "posters.h"
namespace arnis::decals::posters
{
std::filesystem::path billboard(std::uint8_t v, const std::filesystem::path &root)
{
	return root / ("billboard_" + std::to_string(v % BILLBOARD_COUNT) + ".png");
}
std::filesystem::path column(std::uint8_t v, const std::filesystem::path &root)
{
	return root / ("column_" + std::to_string(v % COLUMN_COUNT) + ".png");
}
}
