#include "luanti_block_map.h"
#include "../../arnis_block.h"
#include <string>
namespace arnis {
LuantiNode to_luanti_node(const Block &block, LuantiGame, const char *facing, bool open, bool top)
{
	static const char *basic[] = {"air","mcl_core:stone","mcl_core:dirt","mcl_core:grass_block"};
	const auto id = block.id();
	if (id < sizeof(basic)/sizeof(*basic)) return {basic[id],0};
	if (facing && id == 17) {
		unsigned d = 0; if (!std::string(facing).compare("east")) d=1; else if (!std::string(facing).compare("south")) d=2; else if (!std::string(facing).compare("west")) d=3;
		return {"mcl_stairs:stair_cobble", static_cast<std::uint8_t>(d + (top ? 20 : 0))};
	}
	(void)open; (void)top; return {"mcl_core:stone",0};
}
}
