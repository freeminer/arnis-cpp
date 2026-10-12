#include "emergency.h"
#include "signage.h"
#include "../strict_parse.h"
#include <algorithm>

namespace arnis
{
namespace emergency
{

void generate_emergency(WorldEditor &editor, const ProcessedNode &node)
{
	// Match Rust parse::<i32>().unwrap_or(0): malformed tags default to zero.
	for (const char *key : {"layer", "level"}) {
		if (const auto it = node.tags.find(key);
				it != node.tags.end() && strict_parse::i32_or_zero(it->second) < 0)
			return;
	}

	auto it_emergency = node.tags.find("emergency");
	if (it_emergency != node.tags.end()) {
		if (it_emergency->second == "fire_hydrant") {
			generate_fire_hydrant(editor, node);
		}
	}
}

void generate_fire_hydrant(WorldEditor &editor, const ProcessedNode &node)
{
	int x = node.x;
	int z = node.z;

	// Get hydrant type - skip underground, wall, and pond types
	std::string hydrant_type = "pillar";
	auto it_type = node.tags.find("fire_hydrant:type");
	if (it_type != node.tags.end()) {
		hydrant_type = it_type->second;
	}

	// Skip non-visible hydrant types
	if (hydrant_type == "underground" || hydrant_type == "wall" ||
			hydrant_type == "pond") {
		return;
	}

	editor.set_block(REDSTONE_BLOCK, x, 1, z, std::nullopt, std::nullopt);
	if (const auto key = signage::furniture_pictogram(node.tags);
			key && editor.signage_context && editor.signage_context->has(*key)) {
		const int y = editor.get_ground_level(x, z) + 1;
		for (const std::int8_t facing :
				{std::int8_t(2), std::int8_t(3), std::int8_t(4), std::int8_t(5)})
			editor.place_decal(x, y, z, facing, *key);
	}
}

}
}
