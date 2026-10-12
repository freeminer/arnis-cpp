#include <string>
#include <optional>
#include <stdexcept>
#include "../strict_parse.h"

#include "../../../arnis_adapter.h"
namespace arnis
{

namespace doors
{

void generate_doors(crate::world_editor::WorldEditor &editor,
		crate::osm_parser::ProcessedNode const &element)
{
	if (element.tags.find("door") != element.tags.end() ||
			element.tags.find("entrance") != element.tags.end()) {
		auto it = element.tags.find("level");
		if (it != element.tags.end()) {
			const auto level = strict_parse::i32(it->second);
			if (level && *level != 0)
				return;
		}

		int x = element.x;
		int z = element.z;

		editor.set_block(crate::block_definitions::GRAY_CONCRETE, x, 0, z, std::nullopt,
				std::nullopt);
		editor.set_block(crate::block_definitions::DARK_OAK_DOOR_LOWER, x, 1, z,
				std::nullopt, std::nullopt);
		editor.set_block(crate::block_definitions::DARK_OAK_DOOR_UPPER, x, 2, z,
				std::nullopt, std::nullopt);
	}
}

}
}
