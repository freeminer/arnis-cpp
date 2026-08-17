#include <array>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <string>
#include <optional>
#include <utility>

#include "../../../../arnis_adapter.h"
#include "../../floodfill_cache.h"
namespace arnis
{

namespace
{
constexpr int BUILDING_PASSAGE_HEIGHT = 4;
}

// INTERIOR1_LAYER1
static constexpr std::array<std::array<char, 23>, 23> INTERIOR1_LAYER1 = {{
		{'1', 'U', ' ', 'W', 'C', ' ', ' ', ' ', 'S', 'S', 'W', 'B', 'T', 'T', 'B', 'W',
				'7', '8', ' ', ' ', ' ', ' ', 'W'},
		{'2', ' ', ' ', 'W', 'F', ' ', ' ', ' ', 'U', 'U', 'W', 'B', 'T', 'T', 'B', 'W',
				'7', '8', ' ', ' ', ' ', 'B', 'W'},
		{' ', ' ', ' ', 'W', 'F', ' ', ' ', ' ', ' ', ' ', 'W', 'B', 'T', 'T', 'B', 'W',
				'W', 'W', 'D', 'W', 'W', 'W', 'W'},
		{'W', 'W', 'D', 'W', 'L', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'A', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', 'B', 'B', ' ', ' ', 'J', 'W',
				' ', ' ', ' ', 'B', 'W', 'W', 'W'},
		{'W', 'W', 'W', 'W', 'D', 'W', ' ', ' ', 'W', 'T', 'S', 'S', 'T', ' ', ' ', 'W',
				'S', 'S', ' ', 'B', 'W', 'W', 'W'},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', 'T', 'T', 'T', 'T', ' ', ' ', 'W',
				'U', 'U', ' ', 'B', 'W', ' ', ' '},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'D', 'T', 'T', 'T', 'T', ' ', 'B', 'W',
				' ', ' ', ' ', 'B', 'W', ' ', ' '},
		{'L', ' ', 'A', 'L', 'W', 'W', ' ', ' ', 'W', 'J', 'U', 'U', ' ', ' ', 'B', 'W',
				'W', 'D', 'W', 'W', 'W', ' ', ' '},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				' ', ' ', 'W', 'C', 'C', 'W', 'W'},
		{'B', 'B', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', 'W', ' ', ' ', 'W', 'W'},
		{' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'D'},
		{' ', '6', ' ', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'D', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'U', '5', ' ', 'W', ' ', ' ', 'W', 'C', 'F', 'F', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', 'W', 'L', ' ', 'W',
				'A', ' ', 'B', 'W', ' ', ' ', 'W'},
		{'B', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				' ', ' ', 'B', 'W', 'J', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W',
				'U', ' ', ' ', 'W', 'B', ' ', 'D'},
		{'J', ' ', ' ', 'C', 'B', 'B', 'W', 'L', 'F', ' ', 'W', 'F', ' ', 'W', 'L', 'W',
				'7', '8', ' ', 'W', 'B', ' ', 'W'},
		{'B', ' ', ' ', 'B', 'W', 'W', 'W', 'W', 'W', ' ', 'W', 'A', ' ', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', 'C', ' ', 'W'},
		{'B', ' ', ' ', 'B', 'W', ' ', ' ', ' ', 'D', ' ', 'W', 'C', ' ', ' ', 'W', 'W',
				'B', 'B', 'B', 'B', 'W', 'D', 'W'},
		{'W', 'W', 'D', 'W', 'C', ' ', ' ', ' ', 'W', 'W', 'W', 'B', 'T', 'T', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
}};

// INTERIOR1_LAYER2
static constexpr std::array<std::array<char, 23>, 23> INTERIOR1_LAYER2 = {{
		{' ', 'P', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'P', 'P', 'W', 'B', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', 'B', 'W'},
		{' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', 'B', 'W',
				'W', 'W', 'D', 'W', 'W', 'W', 'W'},
		{'W', 'W', 'D', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', 'B', 'B', ' ', ' ', ' ', 'W',
				' ', ' ', ' ', 'B', 'W', 'W', 'W'},
		{'W', 'W', 'W', 'W', 'D', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				' ', ' ', ' ', 'B', 'W', 'W', 'W'},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				'P', 'P', ' ', 'B', 'W', ' ', ' '},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', 'B', 'W', ' ', ' '},
		{' ', ' ', ' ', ' ', 'W', 'W', ' ', ' ', 'W', ' ', 'P', 'P', ' ', ' ', 'B', 'W',
				'W', 'D', 'W', 'W', 'W', ' ', ' '},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				' ', ' ', 'W', 'C', 'C', 'W', 'W'},
		{'B', 'B', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', 'W', ' ', ' ', 'W', 'W'},
		{' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'D'},
		{' ', ' ', ' ', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'D', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'P', ' ', ' ', 'W', ' ', ' ', 'W', 'N', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				' ', ' ', 'B', 'W', ' ', ' ', 'W'},
		{'B', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				' ', ' ', 'C', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W',
				'P', ' ', ' ', 'W', 'B', ' ', 'D'},
		{' ', ' ', ' ', ' ', 'B', 'B', 'W', ' ', ' ', ' ', 'W', ' ', ' ', 'W', 'P', 'W',
				' ', ' ', ' ', 'W', 'B', ' ', 'W'},
		{'B', ' ', ' ', 'B', 'W', 'W', 'W', 'W', 'W', ' ', 'W', ' ', ' ', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', ' ', ' ', 'W'},
		{'B', ' ', ' ', 'B', 'W', ' ', ' ', ' ', 'D', ' ', 'W', 'N', ' ', ' ', 'W', 'W',
				'B', 'B', 'B', 'B', 'W', 'D', 'W'},
		{'W', 'W', 'D', 'W', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'B', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
}};

// INTERIOR2_LAYER1
static constexpr std::array<std::array<char, 23>, 23> INTERIOR2_LAYER1 = {{
		{'W', 'W', 'W', 'D', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'D', 'W', 'W', 'W'},
		{'U', ' ', ' ', ' ', ' ', ' ', 'C', 'W', 'L', ' ', ' ', 'L', 'W', 'A', 'A', 'W',
				' ', ' ', ' ', ' ', ' ', 'L', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', 'S', 'S', 'S', ' ', 'W'},
		{' ', ' ', 'W', 'F', ' ', ' ', ' ', 'W', 'C', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				'J', ' ', 'U', 'U', 'U', ' ', 'D'},
		{'U', ' ', 'W', 'F', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'W', 'W', 'W', 'W', 'W', 'W'},
		{'U', ' ', 'W', 'F', ' ', ' ', ' ', 'D', ' ', ' ', 'T', 'T', 'W', ' ', ' ', ' ',
				' ', ' ', 'U', 'W', ' ', 'L', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', 'T', 'J', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				'W', ' ', ' ', 'W', 'L', ' ', 'W'},
		{'J', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'C', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'L', ' ', ' ', ' ', ' ', 'W', 'C', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', 'A', 'B', 'B', 'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'B', 'W', 'L', ' ', ' ', ' ', ' ', 'W', 'L', ' ', ' ', 'B', 'W',
				'W', 'B', 'B', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'B', 'W', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', 'D', ' ', ' ', 'U', ' ', ' ', ' ', 'D', ' ', ' ', 'F', 'F',
				'W', 'A', 'A', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', 'W', ' ', ' ', 'U', ' ', ' ', 'W', 'W', ' ', ' ', ' ', ' ',
				'C', ' ', ' ', 'W', ' ', ' ', 'W'},
		{'C', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', ' ', ' ',
				'L', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'L', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'L', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'U', 'U', ' ', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', 'W', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'U', 'U', ' ', 'W', 'B', ' ', 'U', 'U',
				'B', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'S', 'S', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', 'B', ' ', 'W'},
		{'U', 'U', ' ', ' ', ' ', 'L', 'B', 'B', 'B', ' ', ' ', 'W', 'B', 'B', 'B', 'B',
				'B', 'B', 'B', ' ', 'B', 'D', 'W'},
}};

// INTERIOR2_LAYER2
static constexpr std::array<std::array<char, 23>, 23> INTERIOR2_LAYER2 = {{
		{'W', 'W', 'W', 'D', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'D', 'W', 'W', 'W'},
		{'P', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'E', ' ', ' ', 'E', 'W', ' ', ' ', 'W',
				' ', ' ', ' ', ' ', ' ', 'E', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', 'W', 'F', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				' ', ' ', 'P', 'P', 'P', ' ', 'D'},
		{'P', ' ', 'W', 'F', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'W', 'W', 'W', 'W', 'W', 'W'},
		{'P', ' ', 'W', 'F', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', 'P', 'W', ' ', 'P', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'P', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'E', ' ', ' ', ' ', ' ', 'W', 'P', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', ' ', 'B', 'B', 'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'B', 'W', 'E', ' ', ' ', ' ', ' ', 'W', 'E', ' ', ' ', 'B', 'W',
				'W', 'B', 'B', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'B', 'W', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', 'D', ' ', ' ', 'P', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', 'W', ' ', ' ', 'P', ' ', ' ', 'W', 'W', ' ', ' ', ' ', ' ',
				' ', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', ' ', ' ',
				'E', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'E', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'E', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'P', 'P', ' ', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', 'W', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'P', 'P', ' ', 'W', 'B', ' ', 'P', 'P',
				'B', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', 'B', ' ', 'W'},
		{'P', 'P', ' ', ' ', ' ', 'E', 'B', 'B', 'B', ' ', ' ', 'W', 'B', 'B', 'B', 'B',
				'B', 'B', 'B', ' ', 'B', ' ', 'D'},
}};

// Generic Abandoned Building Interiors
// ABANDONED_INTERIOR1_LAYER1
static constexpr std::array<std::array<char, 23>, 23> ABANDONED_INTERIOR1_LAYER1 = {{
		{'1', 'U', ' ', 'W', 'C', ' ', ' ', ' ', 'S', 'S', 'W', 'b', 'T', 'T', 'd', 'W',
				'7', '8', ' ', ' ', ' ', ' ', 'W'},
		{'2', ' ', ' ', 'W', 'F', ' ', ' ', ' ', 'U', 'U', 'W', 'b', 'T', 'T', 'd', 'W',
				'7', '8', ' ', ' ', ' ', 'B', 'W'},
		{' ', ' ', ' ', 'W', 'F', ' ', ' ', ' ', ' ', ' ', 'W', 'b', 'T', 'T', 'd', 'W',
				'W', 'W', 'D', 'W', 'W', 'W', 'W'},
		{'W', 'W', 'D', 'W', 'L', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'M', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'c', 'c', 'c', ' ', ' ', 'J', 'W',
				' ', ' ', ' ', 'd', 'W', 'W', 'W'},
		{'W', 'W', 'W', 'W', 'D', 'W', ' ', ' ', 'W', 'T', 'S', 'S', 'T', ' ', ' ', 'W',
				'S', 'S', ' ', 'd', 'W', 'W', 'W'},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', 'T', 'T', 'T', 'T', ' ', ' ', 'W',
				'U', 'U', ' ', 'd', 'W', ' ', ' '},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'D', 'T', 'T', 'T', 'T', ' ', 'B', 'W',
				' ', ' ', ' ', 'd', 'W', ' ', ' '},
		{'L', ' ', 'M', 'L', 'W', 'W', ' ', ' ', 'W', 'J', 'U', 'U', ' ', ' ', 'B', 'W',
				'W', 'D', 'W', 'W', 'W', ' ', ' '},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				' ', ' ', 'W', 'C', 'C', 'W', 'W'},
		{'c', 'c', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', 'W', ' ', ' ', 'W', 'W'},
		{' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'D'},
		{' ', '6', ' ', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'D', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'U', '5', ' ', 'W', ' ', ' ', 'W', 'C', 'F', 'F', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', 'W', 'L', ' ', 'W',
				'M', ' ', 'b', 'W', ' ', ' ', 'W'},
		{'B', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				' ', ' ', 'b', 'W', 'J', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W',
				'U', ' ', ' ', 'W', 'B', ' ', 'D'},
		{'J', ' ', ' ', 'C', 'a', 'a', 'W', 'L', 'F', ' ', 'W', 'F', ' ', 'W', 'L', 'W',
				'7', '8', ' ', 'W', 'B', ' ', 'W'},
		{'B', ' ', ' ', 'd', 'W', 'W', 'W', 'W', 'W', ' ', 'W', 'M', ' ', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', 'C', ' ', 'W'},
		{'B', ' ', ' ', 'd', 'W', ' ', ' ', ' ', 'D', ' ', 'W', 'C', ' ', ' ', 'W', 'W',
				'c', 'c', 'c', 'c', 'W', 'D', 'W'},
		{'W', 'W', 'D', 'W', 'C', ' ', ' ', ' ', 'W', 'W', 'W', 'b', 'T', 'T', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
}};

// ABANDONED_INTERIOR1_LAYER2
static constexpr std::array<std::array<char, 23>, 23> ABANDONED_INTERIOR1_LAYER2 = {{
		{' ', 'P', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'P', 'P', 'W', 'B', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', 'B', 'W'},
		{' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', ' ', ' ', 'B', 'W',
				'W', 'W', 'D', 'W', 'W', 'W', 'W'},
		{'W', 'W', 'D', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'B', 'B', 'B', ' ', ' ', ' ', 'W',
				' ', ' ', ' ', 'B', 'W', 'W', 'W'},
		{'W', 'W', 'W', 'W', 'D', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				' ', ' ', ' ', 'B', 'W', 'W', 'W'},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				'P', 'P', ' ', 'B', 'W', ' ', ' '},
		{' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', 'B', 'W', ' ', ' '},
		{' ', ' ', ' ', ' ', 'W', 'W', ' ', ' ', 'W', ' ', 'P', 'P', ' ', ' ', 'B', 'W',
				'W', 'D', 'W', 'W', 'W', ' ', ' '},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				' ', ' ', 'W', 'C', 'C', 'W', 'W'},
		{'B', 'B', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', 'W', ' ', ' ', 'W', 'W'},
		{' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D',
				' ', ' ', ' ', ' ', ' ', ' ', 'D'},
		{' ', ' ', ' ', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W', 'D', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'P', ' ', ' ', 'W', ' ', ' ', 'W', 'N', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'D', 'W', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				' ', ' ', 'B', 'W', ' ', ' ', 'W'},
		{'B', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				' ', ' ', 'C', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W',
				'P', ' ', ' ', 'W', 'B', ' ', 'D'},
		{' ', ' ', ' ', ' ', 'B', 'B', 'W', ' ', ' ', ' ', 'W', ' ', ' ', 'W', 'P', 'W',
				' ', ' ', ' ', 'W', 'B', ' ', 'W'},
		{'B', ' ', ' ', 'B', 'W', 'W', 'W', 'W', 'W', ' ', 'W', ' ', ' ', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', ' ', ' ', 'W'},
		{'B', ' ', ' ', 'B', 'W', ' ', ' ', ' ', 'D', ' ', 'W', 'N', ' ', ' ', 'W', 'W',
				'B', 'B', 'B', 'B', 'W', 'D', 'W'},
		{'W', 'W', 'D', 'W', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'B', ' ', ' ', 'B', 'W',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
}};

// ABANDONED_INTERIOR2_LAYER1
static constexpr std::array<std::array<char, 23>, 23> ABANDONED_INTERIOR2_LAYER1 = {{
		{'W', 'W', 'W', 'D', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'D', 'W', 'W', 'W'},
		{'U', ' ', ' ', ' ', ' ', ' ', 'C', 'W', 'L', ' ', ' ', 'L', 'W', 'M', 'M', 'W',
				' ', ' ', ' ', ' ', ' ', 'L', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'Q', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', 'S', 'S', 'S', ' ', 'W'},
		{' ', ' ', 'W', 'F', ' ', ' ', ' ', 'Q', 'C', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				'J', ' ', 'U', 'U', 'U', ' ', 'D'},
		{'U', ' ', 'W', 'F', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'W', 'W', 'W', 'W', 'W', 'W'},
		{'U', ' ', 'W', 'F', ' ', ' ', ' ', 'D', ' ', ' ', 'T', 'T', 'W', ' ', ' ', ' ',
				' ', ' ', 'U', 'W', ' ', 'L', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', 'T', 'J', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				'W', ' ', ' ', 'W', 'L', ' ', 'W'},
		{'J', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'C', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'L', ' ', ' ', ' ', ' ', 'W', 'C', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', 'M', 'c', 'B', 'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'd', 'W', 'L', ' ', ' ', ' ', ' ', 'W', 'L', ' ', ' ', 'B', 'W',
				'W', 'B', 'B', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'd', 'W', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', 'D', ' ', ' ', 'U', ' ', ' ', ' ', 'D', ' ', ' ', 'F', 'F',
				'W', 'M', 'M', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', 'W', ' ', ' ', 'U', ' ', ' ', 'W', 'W', ' ', ' ', ' ', ' ',
				'C', ' ', ' ', 'W', ' ', ' ', 'W'},
		{'C', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', ' ', ' ',
				'L', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'L', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'L', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'U', 'U', ' ', 'Q', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', 'W', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'U', 'U', ' ', 'Q', 'b', ' ', 'U', 'U',
				'B', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'S', 'S', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'Q', 'b', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', 'd', ' ', 'W'},
		{'U', 'U', ' ', ' ', ' ', 'L', 'a', 'a', 'a', ' ', ' ', 'Q', 'B', 'a', 'a', 'a',
				'a', 'a', 'a', ' ', 'd', 'D', 'W'},
}};

// ABANDONED_INTERIOR2_LAYER2
static constexpr std::array<std::array<char, 23>, 23> ABANDONED_INTERIOR2_LAYER2 = {{
		{'W', 'W', 'W', 'D', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'D', 'W', 'W', 'W'},
		{'P', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'O', ' ', ' ', 'O', 'W', ' ', ' ', 'W',
				' ', ' ', ' ', ' ', ' ', 'O', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'Q', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', 'W', 'F', ' ', ' ', ' ', 'Q', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W',
				' ', ' ', 'P', 'P', 'P', ' ', 'D'},
		{'P', ' ', 'W', 'F', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W',
				'W', 'W', 'W', 'W', 'W', 'W', 'W'},
		{'P', ' ', 'W', 'F', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', 'P', 'W', ' ', 'P', 'W'},
		{' ', ' ', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ',
				' ', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'D', 'W', 'W',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'P', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'O', ' ', ' ', ' ', ' ', 'W', 'P', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', ' ', 'c', 'B', 'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'B',
				'W', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'd', 'W', 'O', ' ', ' ', ' ', ' ', 'W', 'O', ' ', ' ', 'B', 'W',
				'W', 'B', 'B', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', 'd', 'W', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', ' ', ' ', 'D'},
		{' ', ' ', ' ', ' ', 'D', ' ', ' ', 'P', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ',
				'W', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', 'W', ' ', ' ', 'P', ' ', ' ', 'W', 'W', ' ', ' ', ' ', ' ',
				' ', ' ', ' ', 'W', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', ' ', ' ',
				'O', ' ', ' ', 'W', 'W', 'D', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'D', ' ', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'O', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'O', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', ' ', ' ', 'W'},
		{'W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', 'P', 'P', ' ', 'Q', 'W', 'W', 'W', 'W',
				'W', 'W', 'W', 'W', 'W', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'P', 'P', ' ', 'Q', 'b', ' ', 'P', 'P',
				'c', ' ', ' ', ' ', ' ', ' ', 'W'},
		{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'Q', 'b', ' ', ' ', ' ',
				' ', ' ', ' ', ' ', 'd', ' ', 'W'},
		{'P', 'P', ' ', ' ', ' ', 'O', 'a', 'a', 'a', ' ', ' ', 'Q', 'b', 'a', 'a', 'a',
				'a', 'a', 'a', ' ', 'd', ' ', 'D'},
}};

/// Maps interior layout characters to actual block types for different floor layers
inline std::optional<Block> get_interior_block(char c, bool is_layer2, Block wall_block)
{
	switch (c) {
	case ' ':
		return {}; // Nothing
	case 'W':
		return wall_block; // Use the building's wall block for interior walls
	case 'U':
		return OAK_FENCE; // Oak Fence
	case 'S':
		return OAK_STAIRS; // Oak Stairs
	case 'B':
		return BOOKSHELF; // Bookshelf
	case 'C':
		return CRAFTING_TABLE; // Crafting Table
	case 'F':
		return FURNACE; // Furnace
	case '1':
		return RED_BED_NORTH_HEAD; // Bed North Head
	case '2':
		return RED_BED_NORTH_FOOT; // Bed North Foot
	case '3':
		return RED_BED_EAST_HEAD; // Bed East Head
	case '4':
		return RED_BED_EAST_FOOT; // Bed East Foot
	case '5':
		return RED_BED_SOUTH_HEAD; // Bed South Head
	case '6':
		return RED_BED_SOUTH_FOOT; // Bed South Foot
	case '7':
		return RED_BED_WEST_HEAD; // Bed West Head
	case '8':
		return RED_BED_WEST_FOOT; // Bed West Foot
	case 'L':
		return CAULDRON; // Cauldron
	case 'A':
		return ANVIL; // Anvil
	case 'P':
		return OAK_PRESSURE_PLATE; // Pressure Plate
	case 'D': {
		// Use different door types for different layers
		if (is_layer2) {
			return DARK_OAK_DOOR_UPPER;
		} else {
			return DARK_OAK_DOOR_LOWER;
		}
	}
	case 'J':
		return NOTE_BLOCK; // Note block
	case 'G':
		return GLOWSTONE; // Glowstone
	case 'N':
		return BREWING_STAND; // Brewing Stand
	case 'T':
		return WHITE_CARPET; // White Carpet
	case 'E':
		return OAK_LEAVES; // Oak Leaves
	case 'O':
		return COBWEB; // Cobweb
	case 'a':
		return CHISELLED_BOOKSHELF_NORTH; // Chiseled Bookshelf
	case 'b':
		return CHISELLED_BOOKSHELF_EAST; // Chiseled Bookshelf East
	case 'c':
		return CHISELLED_BOOKSHELF_SOUTH; // Chiseled Bookshelf South
	case 'd':
		return CHISELLED_BOOKSHELF_WEST; // Chiseled Bookshelf West
	case 'M':
		return DAMAGED_ANVIL; // Damaged Anvil
	case 'Q':
		return SCAFFOLDING; // Scaffolding
	default:
		return {}; // Default case for unknown characters
	}
}

/// Generates interior layouts inside buildings at each floor level
void generate_building_interior(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &floor_area, int min_x, int min_z,
		int max_x, int max_z, int start_y_offset, int building_height, Block wall_block,
		const std::vector<int> &floor_levels, const Args &args,
		const ProcessedWay &element, int abs_terrain_offset, bool is_abandoned_building,
		const CoordinateBitmap &building_passages, bool has_sloped_roof)
{
	(void)args;
	(void)element;
	// Skip interior generation for very small buildings
	int width = max_x - min_x + 1;
	int depth = max_z - min_z + 1;

	if (width < 8 || depth < 8) {
		return; // Building too small for interior
	}

	// For efficiency, create a unordered_set of floor area coordinates
	std::unordered_set<long long> floor_area_set;
	floor_area_set.reserve(floor_area.size());
	for (const auto &p : floor_area) {
		long long key = (static_cast<long long>(p.first) << 32) |
						(static_cast<unsigned int>(p.second));
		floor_area_set.insert(key);
	}

	// Add buffer around edges to avoid placing furniture too close to walls
	int buffer = 2;
	int interior_min_x = min_x + buffer;
	int interior_min_z = min_z + buffer;
	int interior_max_x = max_x - buffer;
	int interior_max_z = max_z - buffer;

	// Generate interiors for each floor
	for (size_t floor_index = 0; floor_index < floor_levels.size(); ++floor_index) {
		int floor_y = floor_levels[floor_index];

		// Store wall and door positions for this floor to extend them to the ceiling
		std::vector<std::pair<int, int>> wall_positions;
		std::vector<std::pair<int, int>> door_positions;

		// Determine the floor extension height (ceiling) - either next floor or roof
		int current_floor_ceiling;
		if (floor_index < floor_levels.size() - 1) {
			// For intermediate floors, extend walls up to just below the next floor
			current_floor_ceiling = floor_levels[floor_index + 1] - 1;
		} else {
			if (has_sloped_roof) {
				current_floor_ceiling = start_y_offset + building_height;
			} else {
				current_floor_ceiling = start_y_offset + building_height + 1;
			}
		}

		// Choose the appropriate interior pattern based on floor number
		const std::array<std::array<char, 23>, 23> *layer1;
		const std::array<std::array<char, 23>, 23> *layer2;
		if (is_abandoned_building) {
			if (floor_index == 0) {
				layer1 = &ABANDONED_INTERIOR1_LAYER1;
				layer2 = &ABANDONED_INTERIOR1_LAYER2;
			} else {
				layer1 = &ABANDONED_INTERIOR2_LAYER1;
				layer2 = &ABANDONED_INTERIOR2_LAYER2;
			}
		} else if (floor_index == 0) {
			// Ground floor uses INTERIOR1 patterns
			layer1 = &INTERIOR1_LAYER1;
			layer2 = &INTERIOR1_LAYER2;
		} else {
			// Upper floors use INTERIOR2 patterns
			layer1 = &INTERIOR2_LAYER1;
			layer2 = &INTERIOR2_LAYER2;
		}

		// Get dimensions for the selected pattern
		int pattern_height = static_cast<int>(layer1->size());
		int pattern_width = static_cast<int>((*layer1)[0].size());

		// Calculate Y offset - place interior 1 block above floor level consistently
		int y_offset = 1;

		// Create a seamless repeating pattern across the interior of this floor
		for (int z = interior_min_z; z <= interior_max_z; ++z) {
			for (int x = interior_min_x; x <= interior_max_x; ++x) {
				// Skip if outside the building's floor area
				long long key = (static_cast<long long>(x) << 32) |
								(static_cast<unsigned int>(z));
				if (floor_area_set.find(key) == floor_area_set.end()) {
					continue;
				}
				if (building_passages.contains(x, z) &&
						floor_y < start_y_offset + std::min(BUILDING_PASSAGE_HEIGHT,
														   building_height)) {
					continue;
				}

				// Map the world coordinates to pattern coordinates using modulo
				// This creates a seamless tiling effect across the entire building
				// Add floor_index offset to create variation between floors
				int pattern_x = ((x - interior_min_x + static_cast<int>(floor_index)) %
												pattern_width +
										pattern_width) %
								pattern_width;
				int pattern_z = ((z - interior_min_z + static_cast<int>(floor_index)) %
												pattern_height +
										pattern_height) %
								pattern_height;

				// Access the pattern arrays safely
				char cell1 = (*layer1)[pattern_z][pattern_x];
				char cell2 = (*layer2)[pattern_z][pattern_x];

				// Place first layer blocks
				auto opt_block1 = get_interior_block(cell1, false, wall_block);
				if (opt_block1.has_value()) {
					editor.set_block_absolute(opt_block1.value(), x,
							floor_y + y_offset + abs_terrain_offset, z, std::nullopt,
							std::nullopt);

					// If this is a wall in layer 1, add to wall positions to extend later
					if (cell1 == 'W') {
						wall_positions.emplace_back(x, z);
					}
					// If this is a door in layer 1, add to door positions to add wall above later
					else if (cell1 == 'D') {
						door_positions.emplace_back(x, z);
					}
				}

				// Place second layer blocks
				auto opt_block2 = get_interior_block(cell2, true, wall_block);
				if (opt_block2.has_value()) {
					editor.set_block_absolute(opt_block2.value(), x,
							floor_y + y_offset + abs_terrain_offset + 1, z, std::nullopt,
							std::nullopt);
				}
			}
		}

		// Extend walls all the way to the next floor ceiling or roof
		for (const auto &p : wall_positions) {
			int x = p.first;
			int z = p.second;
			for (int y = floor_y + y_offset + 2; y <= current_floor_ceiling; ++y) {
				editor.set_block_absolute(wall_block, x, y + abs_terrain_offset, z,
						std::nullopt, std::nullopt);
			}
		}

		// Add wall blocks above doors all the way to the ceiling/next floor
		for (const auto &p : door_positions) {
			int x = p.first;
			int z = p.second;
			for (int y = floor_y + y_offset + 2; y <= current_floor_ceiling; ++y) {
				editor.set_block_absolute(wall_block, x, y + abs_terrain_offset, z,
						std::nullopt, std::nullopt);
			}
		}
	}
}

}
