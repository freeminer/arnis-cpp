#pragma once
#include "block_definitions.h"
#include <cstdint>
namespace arnis
{
class BlockWithProperties;
enum class LuantiGame
{
	Mineclonia
};
constexpr const char *game_id(LuantiGame)
{
	return "mineclonia";
}
struct LuantiNode
{
	const char *name;
	std::uint8_t param2 = 0;
};
LuantiNode to_luanti_node(const Block &block, LuantiGame game = LuantiGame::Mineclonia,
		const char *facing = nullptr, bool open = false, bool top = false);
// Rust's block mapper receives palette properties as well as the base block.
// Preserve those properties for Mineclonia nodes whose name/param2 depends on
// block state, especially bed halves and facing.
LuantiNode to_luanti_node(const BlockWithProperties &block,
		LuantiGame game = LuantiGame::Mineclonia, const char *facing = nullptr,
		bool open = false, bool top = false);
}
