#include <string>
#include <unordered_map>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "../../../arnis_adapter.h"
#include "signage.h"

namespace arnis
{

namespace tourisms
{


std::optional<int> parse_int(const std::string &s)
{
	try {
		size_t pos = 0;
		int v = std::stoi(s, &pos);
		if (pos != s.length()) {
			return std::nullopt;
		}
		return v;
	} catch (const std::exception &) {
		return std::nullopt;
	}
}

void generate_tourisms(WorldEditor &editor, const ProcessedNode &element,
		const RoadMaskBitmap &road_mask)
{
	auto it_layer = element.tags.find("layer");
	if (it_layer != element.tags.end()) {
		auto opt_layer = parse_int(it_layer->second);
		if (opt_layer.has_value() && opt_layer.value() < 0) {
			return;
		}
	}

	auto it_level = element.tags.find("level");
	if (it_level != element.tags.end()) {
		auto opt_level = parse_int(it_level->second);
		if (opt_level.has_value() && opt_level.value() < 0) {
			return;
		}
	}

	auto it_tourism = element.tags.find("tourism");
	if (it_tourism != element.tags.end()) {
		const std::string &tourism_type = it_tourism->second;
		int x = element.x;
		int z = element.z;

		if (tourism_type == "information") {
			auto it_info = element.tags.find("information");
			if (it_info != element.tags.end()) {
				const std::string &info_type = it_info->second;
				if (info_type != "office" && info_type != "visitor_centre") {
					// Try to generate information board with decal first
					if (signage::generate_information_board(editor, element, road_mask)) {
						return;
					}
					// Fallback: simple banner board if decal unavailable
					editor.set_block(COBBLESTONE_WALL, x, 1, z, std::nullopt, std::nullopt);
					editor.set_block(OAK_PLANKS, x, 2, z, std::nullopt, std::nullopt);

					const int abs_y = editor.get_ground_level(x, 0, z) + 2;
					const std::vector<std::pair<std::string, std::string>> info_patterns =
							{
									{"blue", "minecraft:stripe_left"},
									{"blue", "minecraft:stripe_right"},
									{"blue", "minecraft:stripe_top"},
									{"blue", "minecraft:stripe_middle"},
									{"blue", "minecraft:border"},
							};
					const std::vector<std::tuple<int, int, std::string>> banner_faces = {
							{0, 1, "south"},
							{0, -1, "north"},
							{1, 0, "east"},
							{-1, 0, "west"},
					};
					for (const auto &[dx, dz, facing] : banner_faces) {
						editor.place_wall_banner(
								x + dx, abs_y, z + dz, facing, info_patterns);
					}
				}
			}
		}
	}
}

} // namespace tourisms
} // namespace arnis
