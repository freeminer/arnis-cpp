#include "bridge_modules.h"
namespace arnis::bridge_modules
{
std::optional<std::size_t> pick_module_index(int r, std::size_t n)
{
	if (n < MIN_MODULE_BRIDGE_LEN)
		return {};
	if (r >= 6)
		return 0;
	if (r == 5)
		return n >= 45 ? 1 : 2;
	return 3;
}
int module_half_width(std::size_t i)
{
	return i < 2 ? 18 : 8;
}
bool module_has_pillars(std::size_t i)
{
	return i < 2;
}
}
