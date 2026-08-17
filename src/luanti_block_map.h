#pragma once
#include "block_definitions.h"
#include <cstdint>
namespace arnis {
enum class LuantiGame { Mineclonia };
struct LuantiNode { const char *name; std::uint8_t param2 = 0; };
LuantiNode to_luanti_node(const Block &block, LuantiGame game = LuantiGame::Mineclonia,
		const char *facing = nullptr, bool open = false, bool top = false);
}
