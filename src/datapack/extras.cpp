#include "extras.h"

#include "mod.h"

#include "../args.h"
#include "../assets_root.h"
#include "../element_processing/cars.h"
#include "../element_processing/trains.h"
#include "../world_utils.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace arnis::datapack
{
namespace
{
using json = nlohmann::json;
constexpr char PACK_NAME[] = "arnis_extras";
constexpr std::uint32_t BASE_FORMAT = 61;
constexpr std::uint32_t MAX_FORMAT = 0x7fffffff;

struct Feature
{
	const char *name;
	bool enabled;
};

bool selected_asset(
		const std::filesystem::path &relative, const std::vector<Feature> &features)
{
	const auto path = relative.generic_string();
	if (path == "data/arnis/function/load.mcfunction" ||
			path.starts_with("data/arnis/function/util/") ||
			path == "ov101/data/arnis/function/util/daytime.mcfunction")
		return true;
	if (path.starts_with("ov80/data/arnis/loot_table/outbreak/") ||
			path.starts_with("ov121/data/arnis/loot_table/outbreak/"))
		return std::any_of(features.begin(), features.end(), [](const Feature &feature) {
			return feature.enabled && std::string(feature.name) == "outbreak";
		});
	for (const auto &feature : features) {
		if (!feature.enabled)
			continue;
		const auto base = std::string("data/arnis/function/") + feature.name + "/";
		if (path.starts_with(base))
			return true;
		const auto overlay =
				std::string("ov80/data/arnis/function/") + feature.name + "/";
		if (path.starts_with(overlay))
			return true;
		const auto modern =
				std::string("ov101/data/arnis/function/") + feature.name + "/";
		if (path.starts_with(modern))
			return true;
		const auto latest =
				std::string("ov121/data/arnis/function/") + feature.name + "/";
		if (path.starts_with(latest))
			return true;
	}
	return false;
}

bool copy_asset(const std::filesystem::path &source,
		const std::filesystem::path &destination, std::string *error)
{
	std::error_code ec;
	std::filesystem::create_directories(destination.parent_path(), ec);
	if (ec) {
		if (error)
			*error = "create " + destination.parent_path().string() + ": " + ec.message();
		return false;
	}
	ec.clear();
	std::filesystem::copy_file(
			source, destination, std::filesystem::copy_options::overwrite_existing, ec);
	if (ec) {
		if (error)
			*error = "copy " + source.string() + ": " + ec.message();
		return false;
	}
	return true;
}

bool write_text(const std::filesystem::path &path, const std::string &contents,
		std::string *error)
{
	std::error_code ec;
	std::filesystem::create_directories(path.parent_path(), ec);
	if (ec) {
		if (error)
			*error = "create " + path.parent_path().string() + ": " + ec.message();
		return false;
	}
	std::ofstream file(path, std::ios::binary | std::ios::trunc);
	if (!file || !(file << contents)) {
		if (error)
			*error = "write " + path.string();
		return false;
	}
	return true;
}

json pack_metadata(const std::vector<Feature> &features)
{
	std::vector<std::string> active;
	for (const auto &feature : features) {
		if (!feature.enabled)
			continue;
		if (feature.name == std::string("traffic"))
			active.emplace_back("traffic lights");
		else if (feature.name == std::string("gravity"))
			active.emplace_back("body gravity");
		else if (feature.name == std::string("cars"))
			active.emplace_back("moving cars");
		else if (feature.name == std::string("trains"))
			active.emplace_back("trains");
		else if (feature.name == std::string("discoveries"))
			active.emplace_back("discoveries");
		else if (feature.name == std::string("menu"))
			active.emplace_back("pause menu");
		else if (feature.name == std::string("outbreak"))
			active.emplace_back("zombie outbreak");
	}
	json overlays = json::array();
	for (const auto [format, directory] :
			std::array<std::pair<std::uint32_t, const char *>, 5>{{{71, "ov71"},
					{80, "ov80"}, {101, "ov101"}, {107, "ov107"}, {121, "ov121"}}}) {
		overlays.push_back(
				{{"formats", {{"min_inclusive", format}, {"max_inclusive", MAX_FORMAT}}},
						{"min_format", {format, 0}},
						{"max_format", {MAX_FORMAT, MAX_FORMAT}},
						{"directory", directory}});
	}
	std::string description = "Arnis extras: ";
	for (const auto &name : active) {
		if (description.back() != ' ')
			description += ", ";
		description += name;
	}
	json metadata;
	metadata["pack"] = {{"description", description}, {"pack_format", BASE_FORMAT},
			{"supported_formats",
					{{"min_inclusive", BASE_FORMAT}, {"max_inclusive", MAX_FORMAT}}},
			{"min_format", {BASE_FORMAT, 0}}, {"max_format", {MAX_FORMAT, MAX_FORMAT}}};
	metadata["overlays"]["entries"] = std::move(overlays);
	return metadata;
}

json plain_text(const std::string &text, const char *color)
{
	return {{"text", text}, {"color", color}};
}

json dialog_message(json contents)
{
	return {{"type", "minecraft:plain_message"}, {"contents", std::move(contents)},
			{"width", 300}};
}

std::vector<std::string> credit_lines(const Args &args)
{
	std::vector<std::string> lines{
			"Generated with Arnis by Louis Erbkamm and contributors."};
	if (is_earth(args.body)) {
		lines.emplace_back(
				"Map data © OpenStreetMap contributors, available under the Open Database License.");
		lines.emplace_back(
				"Elevation © Mapterhorn (Copernicus GLO-30 © DLR/ESA, national mapping agencies), AWS Terrain Tiles and USGS 3DEP.");
		lines.emplace_back(
				"Land cover © ESA WorldCover 2021 (CC BY 4.0). Climate: Köppen-Geiger, Beck et al. 2023. Ecoregions: RESOLVE 2017.");
	} else {
		lines.emplace_back(
				"Terrain from the NASA Planetary Data System: LRO LOLA (Moon) and MGS MOLA (Mars).");
	}
	if (args.overture && !args.skip_objects())
		lines.emplace_back("Extra buildings © Overture Maps Foundation (ODbL).");
	if (args.canopy_height)
		lines.emplace_back(
				"Tree canopy © Meta and World Resources Institute (CC BY 4.0).");
	if (args.mapillary_facades_on())
		lines.emplace_back(
				"Street-level imagery © Mapillary contributors (CC BY-SA 4.0).");
	lines.emplace_back(
			"Not an official Minecraft product. Not approved by or associated with Mojang or Microsoft.");
	return lines;
}

std::string welcome_function(
		const Args &args, const std::string &area, bool has_discoveries, bool dialogs)
{
	const auto tellraw = [](const json &value) {
		return "tellraw @s " + value.dump() + "\n";
	};
	std::string out = "scoreboard players set @s arnis.seen 1\n";
	out += tellraw(
			json::array({"", plain_text("Welcome to ", "gray"), plain_text(area, "gold"),
					plain_text(". This world was generated by Arnis from real map data.",
							"gray")}));
	if (dialogs) {
		out += tellraw(plain_text(
				"Open the pause menu and choose Arnis for travel, discoveries and credits.",
				"gray"));
		out += tellraw(
				plain_text("Map data © OpenStreetMap contributors (ODbL).", "dark_gray"));
	} else {
		if (has_discoveries)
			out += tellraw(plain_text(
					"Your discoveries are in the Advancements screen (L), under " + area +
							".",
					"gray"));
		for (const auto &line : credit_lines(args))
			out += tellraw(plain_text(line, "dark_gray"));
	}
	return out;
}

json back_to_menu(bool menu)
{
	if (menu)
		return {{"label", "Back"},
				{"action", {{"type", "show_dialog"}, {"dialog", "arnis:menu"}}}};
	return {{"label", "Close"}};
}
} // namespace

std::vector<Destination> destinations_from(const std::vector<discoveries::Place> &places,
		const std::optional<SpawnDestination> &spawn)
{
	std::vector<Destination> out;
	out.reserve((spawn ? 1 : 0) + std::min(places.size(), MAX_DESTINATIONS));
	if (spawn)
		out.push_back(
				{"Spawn", "Where you started", spawn->x, spawn->z, spawn->ground_y});
	for (std::size_t i = 0; i < places.size() && i < MAX_DESTINATIONS; ++i) {
		const auto &place = places[i];
		out.push_back({place.name, place.label, place.anchor.first, place.anchor.second,
				place.ground_y});
	}
	return out;
}

bool install_runtime_extras(const std::filesystem::path &world, const Args &args,
		const std::string &area, bool red_amber, const highways::cars::CarNetwork *cars,
		const trains::TrainRouteNetwork *trains,
		const std::function<std::optional<int>(std::uint64_t, int, int)> &rail_y,
		const std::vector<discoveries::Place> &places,
		const std::optional<SpawnDestination> &requested_spawn, std::string *error)
{
	if (args.bedrock || args.luanti || args.one_world_run || world.empty())
		return true;
	const bool gravity = args.body_gravity && !is_earth(args.body);
	const bool traffic = args.traffic_lights;
	const bool moving_cars = args.moving_vehicles_enabled() && cars;
	const bool moving_trains =
			args.moving_vehicles_enabled() && trains && !trains->is_empty();
	const bool has_discoveries = args.discoveries && !places.empty();
	const bool menu = args.pause_menu;
	const bool outbreak = args.zombie_outbreak;
	if (!gravity && !traffic && !moving_cars && !moving_trains && !args.discoveries &&
			!menu && !outbreak)
		return true;
	const std::vector<Feature> features{{"gravity", gravity}, {"traffic", traffic},
			{"cars", moving_cars}, {"trains", moving_trains},
			{"discoveries", args.discoveries}, {"menu", menu}, {"outbreak", outbreak}};
	const auto source_root = assets::path("minecraft/datapack_extras");
	const auto target_root = world / "datapacks" / PACK_NAME;
	std::error_code ec;
	if (!std::filesystem::is_directory(source_root, ec)) {
		if (error)
			*error = "bundled gameplay extras are missing: " + source_root.string();
		return false;
	}
	for (std::filesystem::recursive_directory_iterator it(source_root, ec), end;
			!ec && it != end; it.increment(ec)) {
		if (!it->is_regular_file(ec))
			continue;
		const auto relative = std::filesystem::relative(it->path(), source_root, ec);
		if (ec)
			break;
		if (!selected_asset(relative, features))
			continue;
		if (!copy_asset(it->path(), target_root / relative, error))
			return false;
	}
	if (ec) {
		if (error)
			*error = "enumerate gameplay extras: " + ec.message();
		return false;
	}
	const auto icon = assets::path("minecraft/icon.png");
	if (std::filesystem::is_regular_file(icon, ec) &&
			!copy_asset(icon, target_root / "pack.png", error))
		return false;
	if (!write_text(target_root / "pack.mcmeta", pack_metadata(features).dump(2) + "\n",
				error))
		return false;

	json load = json::array({"arnis:load"});
	json tick = json::array();
	for (const auto &feature : features)
		if (feature.enabled && feature.name != std::string("discoveries")) {
			load.push_back(std::string("arnis:") + feature.name + "/load");
			tick.push_back(std::string("arnis:") + feature.name + "/tick");
		}
	if (!write_text(target_root / "data/minecraft/tags/function/load.json",
				json{{"values", load}}.dump(2) + "\n", error) ||
			!write_text(target_root / "data/minecraft/tags/function/tick.json",
					json{{"values", tick}}.dump(2) + "\n", error))
		return false;
	const std::string config =
			"data modify storage arnis:world area set value " + json(area).dump() +
			"\nscoreboard players set #red_amber arnis " + (red_amber ? "1\n" : "0\n");
	if (!write_text(target_root / "data/arnis/function/config.mcfunction", config, error))
		return false;
	if (has_discoveries)
		for (const auto &[path, contents] : discoveries::advancement_files(area, places))
			if (!write_text(target_root / path, contents, error))
				return false;
	if (menu || outbreak) {
		json actions = json::array();
		auto spawn = requested_spawn;
		if (!spawn)
			if (const auto existing = world_utils::level_spawn_xz(world))
				spawn = SpawnDestination{existing->first, existing->second, std::nullopt};
		const auto destinations = destinations_from(places, spawn);
		for (std::size_t i = 0; i < destinations.size(); ++i) {
			const auto &destination = destinations[i];
			actions.push_back({{"label", destination.name},
					{"tooltip", destination.label},
					{"action", {{"type", "run_command"},
									   {"command", "trigger arnis.tp set " +
														   std::to_string(i + 1)}}}});
		}
		json dialogs = json::array();
		if (!actions.empty())
			dialogs.push_back({{"label", "Travel"},
					{"tooltip", "Jump to a landmark or back to spawn"},
					{"action", {{"type", "show_dialog"}, {"dialog", "arnis:travel"}}}});
		if (has_discoveries)
			dialogs.push_back({{"label", "Discoveries"},
					{"tooltip", "What there is to find here"},
					{"action",
							{{"type", "show_dialog"}, {"dialog", "arnis:discoveries"}}}});
		if (outbreak)
			dialogs.push_back({{"label", "Zombie Outbreak"},
					{"tooltip", "Restart, skip to night or stop"},
					{"action", {{"type", "show_dialog"}, {"dialog", "arnis:outbreak"}}}});
		dialogs.push_back({{"label", "Credits"},
				{"tooltip", "Where this world's data comes from"},
				{"action", {{"type", "show_dialog"}, {"dialog", "arnis:credits"}}}});
		const auto write_json = [&](const std::filesystem::path &relative,
										const json &value) {
			return write_text(target_root / relative, value.dump(2) + "\n", error);
		};
		const std::string dialog_root = "ov80/data/arnis/dialog/";
		const char *pause_dialog = menu ? "arnis:menu" : "arnis:outbreak";
		if (!write_json("ov80/data/minecraft/tags/dialog/pause_screen_additions.json",
					json{{"values", {pause_dialog}}}))
			return false;
		if (menu) {
			const auto menu_dialog = json{{"type", "minecraft:multi_action"},
					{"title", plain_text(area, "gold")}, {"external_title", "Arnis"},
					{"body",
							{dialog_message(json{{"translate", "arnis.menu.banner"},
									 {"fallback", ""}, {"font", "arnis:banner"},
									 {"color", "white"}, {"shadow_color", 0}}),
									dialog_message(json::array({plain_text("Welcome to ",
																		"gray"),
											plain_text(area, "white"),
											plain_text(
													". This world was generated by Arnis from real map data.",
													"gray")}))}},
					{"columns", 2}, {"actions", dialogs},
					{"exit_action", {{"label", "Back to Game"}}}, {"pause", false}};
			if (!write_json(dialog_root + "menu.json", menu_dialog))
				return false;

			json credits_body = json::array();
			for (const auto &line : credit_lines(args))
				credits_body.push_back(dialog_message(plain_text(line, "gray")));
			json credits_actions = json::array();
			credits_actions.push_back({{"label", "arnismc.com"},
					{"tooltip", "Arnis, free and open source"},
					{"action", {{"type", "open_url"}, {"url", "https://arnismc.com"}}}});
			credits_actions.push_back({{"label", "OpenStreetMap"},
					{"tooltip", "Map data © OpenStreetMap contributors"},
					{"action",
							{{"type", "open_url"},
									{"url", "https://www.openstreetmap.org/copyright"}}}});
			const json credits_dialog{{"type", "minecraft:multi_action"},
					{"title", plain_text("Credits", "gold")}, {"body", credits_body},
					{"columns", 2}, {"actions", credits_actions},
					{"exit_action", back_to_menu(true)}};
			if (!write_json(dialog_root + "credits.json", credits_dialog))
				return false;
		}
		if (menu && !actions.empty() &&
				!write_json(dialog_root + "travel.json",
						json{{"type", "minecraft:multi_action"},
								{"title", plain_text("Travel", "gold")},
								{"body", {dialog_message(plain_text(
												 "You land in the street next to it.",
												 "gray"))}},
								{"columns", 2}, {"actions", actions},
								{"exit_action",
										{{"label", "Back"},
												{"action",
														{{"type", "show_dialog"},
																{"dialog",
																		"arnis:menu"}}}}},
								{"pause", false}}))
			return false;
		if (menu && has_discoveries &&
				!write_json(dialog_root + "discoveries.json",
						json{{"type", "minecraft:notice"},
								{"title", plain_text("Discoveries", "gold")},
								{"body",
										{dialog_message(plain_text(
												 std::to_string(places.size()) +
														 " places to find: landmarks, museums, parks, stations and more. Walk up to one to discover it.",
												 "gray")),
												dialog_message(plain_text(
														"Your progress is in the Advancements screen (L), under " +
																area + ".",
														"gray"))}},
								{"action", back_to_menu(true)}}))
			return false;
		if (outbreak &&
				!write_json(dialog_root + "outbreak.json",
						json{{"type", "minecraft:multi_action"},
								{"title", plain_text("Zombie Outbreak", "dark_red")},
								{"body",
										{dialog_message(json::array(
												 {plain_text("Survive five nights in ",
														  "gray"),
														 plain_text(area, "white"),
														 plain_text(
																 ". Zombies come every night, more each time, and on the fifth night a boss: Patient Zero.",
																 "gray")})),
												dialog_message(plain_text(
														"Loot buildings for gear and follow the smoke to a supply drop each morning. If you die, you come back as a zombie.",
														"gray"))}},
								{"columns", 3},
								{"actions",
										json::array({{{"label", "Restart"},
															 {"tooltip",
																	 "Start again from day one"},
															 {"action",
																	 {{"type",
																			  "run_command"},
																			 {"command",
																					 "trigger arnis.ob.ctl set 1"}}}},
												{{"label", "Skip to Night"},
														{"tooltip",
																"Bring on the next wave now"},
														{"action",
																{{"type", "run_command"},
																		{"command",
																				"trigger arnis.ob.ctl set 2"}}}},
												{{"label", "Stop"},
														{"tooltip",
																"End the outbreak and remove the zombies"},
														{"action",
																{{"type", "run_command"},
																		{"command",
																				"trigger arnis.ob.ctl set 3"}}}}})},
								{"exit_action", back_to_menu(menu)}, {"pause", false}}))
			return false;
		std::string destinations_text;
		auto append_destination = [&](std::size_t id, const std::string &name, int x,
										  int z, std::optional<int> ground_y) {
			const int y = ground_y ? *ground_y + 4 : 4096;
			destinations_text +=
					"execute if score @s arnis.tp matches " + std::to_string(id) +
					" run function arnis:menu/go {x:" + std::to_string(x) +
					",y:" + std::to_string(y) + ",z:" + std::to_string(z) + "}\n";
			destinations_text += "execute if score @s arnis.tp matches " +
								 std::to_string(id) + " run title @s actionbar " +
								 json{{"text", name}, {"color", "gold"}}.dump() + "\n";
		};
		for (std::size_t i = 0; i < destinations.size(); ++i) {
			const auto &destination = destinations[i];
			append_destination(i + 1, destination.name, destination.x, destination.z,
					destination.ground_y);
		}
		if (menu) {
			const auto welcome = welcome_function(args, area, has_discoveries, false);
			const auto modern_welcome =
					welcome_function(args, area, has_discoveries, true);
			if (!write_text(
						target_root / "data/arnis/function/menu/destinations.mcfunction",
						destinations_text, error) ||
					!write_text(
							target_root / "data/arnis/function/menu/welcome.mcfunction",
							welcome, error) ||
					!write_text(
							target_root /
									"ov80/data/arnis/function/menu/welcome.mcfunction",
							modern_welcome, error))
				return false;
		}
		if (outbreak) {
			const auto subtitle = [](const std::string &target, const std::string &text,
										  const char *color) {
				return "title " + target + " subtitle " +
					   json{{"text", text}, {"color", color}}.dump() + "\n";
			};
			if (!write_text(
						target_root /
								"ov80/data/arnis/function/outbreak/text/intro.mcfunction",
						subtitle("@s", "Survive five nights in " + area, "gray"),
						error) ||
					!write_text(
							target_root /
									"ov80/data/arnis/function/outbreak/text/victory.mcfunction",
							subtitle("@a", area + " survived", "yellow"), error))
				return false;
		}
	}
	if (gravity) {
		const double ratio =
				args.body == CelestialBody::Moon ? 1.62 / 9.81 : 3.721 / 9.81;
		const double lower = ratio - 1.0;
		const double safe = 1.0 / ratio - 1.0;
		char command[512];
		std::snprintf(command, sizeof(command),
				"tag @s add arnis.gravity\n"
				"attribute @s minecraft:gravity modifier add arnis:body_gravity %.4f add_multiplied_base\n"
				"attribute @s minecraft:safe_fall_distance modifier add arnis:body_gravity %.4f add_multiplied_base\n"
				"attribute @s minecraft:fall_damage_multiplier modifier add arnis:body_gravity %.4f add_multiplied_base\n",
				lower, safe, lower);
		if (!write_text(target_root / "data/arnis/function/gravity/apply.mcfunction",
					command, error))
			return false;
	}
	if (moving_cars) {
		for (const auto &[path, contents] : cars->datapack_functions())
			if (!write_text(target_root / path, contents, error))
				return false;
	}
	if (moving_trains) {
		for (const auto &[path, contents] : trains->datapack_functions(rail_y))
			if (!write_text(target_root / path, contents, error))
				return false;
	}
	if (!world_utils::enable_datapack_in_level_dat(world, PACK_NAME, error))
		return false;
	if (outbreak && !world_utils::start_in_survival(world, error))
		return false;
	return true;
}
} // namespace arnis::datapack
