#include "bedrock_addon.h"

#include "assets_root.h"
#include "bedrock_addon/models.h"
#include "element_processing/cars.h"
#include "element_processing/trains.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace arnis::bedrock_addon
{
namespace
{
constexpr std::string_view SERVER_MODULE = "2.3.0";
constexpr std::array<int, 3> MIN_ENGINE{1, 21, 120};
constexpr std::string_view ENTITY_FORMAT = "1.21.90";
constexpr std::string_view TEXTURE = "textures/entity/arnis_vehicles";
constexpr std::array<std::string_view, 4> SCRIPTS{
		"main.js", "cars.js", "trains.js", "util.js"};

std::string new_uuid()
{
	std::array<std::uint8_t, 16> bytes{};
	std::random_device source;
	for (auto &byte : bytes)
		byte = static_cast<std::uint8_t>(source());
	bytes[6] = (bytes[6] & 0x0f) | 0x40;
	bytes[8] = (bytes[8] & 0x3f) | 0x80;
	std::ostringstream out;
	for (std::size_t i = 0; i < bytes.size(); ++i) {
		if (i == 4 || i == 6 || i == 8 || i == 10)
			out << '-';
		out << std::hex << std::setfill('0') << std::setw(2)
			<< static_cast<unsigned>(bytes[i]);
	}
	return out.str();
}

std::vector<std::uint8_t> read_file(const std::filesystem::path &path)
{
	std::ifstream input(path, std::ios::binary);
	if (!input)
		throw std::runtime_error("could not read Bedrock add-on asset: " + path.string());
	return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

void add_json(Pack &pack, std::string path, const nlohmann::json &value, int indent = 2)
{
	// serde_json::to_vec[_pretty] emits no trailing newline.
	const auto text = value.dump(indent);
	pack.files.emplace_back(
			std::move(path), std::vector<std::uint8_t>(text.begin(), text.end()));
}

void add_text(Pack &pack, std::string path, std::string text)
{
	pack.files.emplace_back(
			std::move(path), std::vector<std::uint8_t>(text.begin(), text.end()));
}

std::string data_module(const nlohmann::json &data)
{
	const auto text = data.dump();
	std::string quoted;
	quoted.reserve(text.size() + 2);
	for (std::size_t i = 0; i < text.size(); ++i) {
		const auto c = static_cast<unsigned char>(text[i]);
		if (c == '\\')
			quoted += "\\\\";
		else if (c == '\'')
			quoted += "\\'";
		else if (c == '\n')
			quoted += "\\n";
		else if (c == '\r')
			quoted += "\\r";
		else if (i + 2 < text.size() && c == 0xe2 &&
				 static_cast<unsigned char>(text[i + 1]) == 0x80 &&
				 (static_cast<unsigned char>(text[i + 2]) == 0xa8 ||
						 static_cast<unsigned char>(text[i + 2]) == 0xa9)) {
			quoted += static_cast<unsigned char>(text[i + 2]) == 0xa8 ? "\\u2028"
																	  : "\\u2029";
			i += 2;
		} else {
			quoted.push_back(static_cast<char>(c));
		}
	}
	return "export const DATA = JSON.parse('" + quoted + "');\n";
}

nlohmann::json behavior_entity(const Body &body)
{
	nlohmann::json description{{"identifier", "arnis:" + body.name},
			{"is_spawnable", false}, {"is_summonable", true}};
	if (body.fleet == Fleet::Car)
		description["properties"] = {{"arnis:brake",
				{{"type", "bool"}, {"default", false}, {"client_sync", true}}}};
	nlohmann::json components = nlohmann::json::object();
	components["minecraft:type_family"] = {
			{"family", {std::string(family(body.fleet)), "inanimate"}}};
	components["minecraft:collision_box"] = {{"width", 0.0001}, {"height", 0.0001}};
	nlohmann::json hitbox{{"width", 0.0}, {"height", 0.0},
			{"pivot", nlohmann::json::array({0.0, 100.0, 0.0})}};
	components["minecraft:custom_hit_test"] = {
			{"hitboxes", nlohmann::json::array({std::move(hitbox)})}};
	components["minecraft:physics"] = {{"has_gravity", false}, {"has_collision", false}};
	components["minecraft:pushable"] = {
			{"is_pushable", false}, {"is_pushable_by_piston", false}};
	components["minecraft:damage_sensor"] = {
			{"triggers", {{"cause", "all"}, {"deals_damage", "no"}}}};
	components["minecraft:fire_immune"] = nlohmann::json::object();
	components["minecraft:persistent"] = nlohmann::json::object();
	components["minecraft:body_rotation_always_follows_head"] = nlohmann::json::object();
	return {{"format_version", std::string(ENTITY_FORMAT)},
			{"minecraft:entity", {{"description", std::move(description)},
										 {"components", std::move(components)}}}};
}

nlohmann::json client_entity(const Body &body)
{
	const std::string controller = body.fleet == Fleet::Car
										   ? "controller.render.arnis.car"
										   : "controller.render.arnis.vehicle";
	nlohmann::json description{{"identifier", "arnis:" + body.name},
			{"materials", {{"default", "entity_alphatest"}, {"lit", "entity_emissive"}}},
			{"textures", {{"default", std::string(TEXTURE)}}},
			{"geometry", {{"default", "geometry.arnis." + body.name}}},
			{"render_controllers", {controller}}};
	return {{"format_version", "1.10.0"},
			{"minecraft:client_entity", {{"description", std::move(description)}}}};
}

nlohmann::json render_controllers()
{
	nlohmann::json materials = nlohmann::json::array();
	materials.push_back({{"*", "Material.default"}});
	materials.push_back({{"lit*", "Material.lit"}});
	nlohmann::json controllers = nlohmann::json::object();
	controllers["controller.render.arnis.vehicle"] = {{"geometry", "Geometry.default"},
			{"materials", materials}, {"textures", {"Texture.default"}}};
	nlohmann::json brake_off = nlohmann::json::object();
	brake_off["brake_off"] = "!q.property('arnis:brake')";
	nlohmann::json lit_brake = nlohmann::json::object();
	lit_brake["lit_brake"] = "q.property('arnis:brake')";
	nlohmann::json part_visibility =
			nlohmann::json::array({std::move(brake_off), std::move(lit_brake)});
	controllers["controller.render.arnis.car"] = {{"geometry", "Geometry.default"},
			{"materials", materials}, {"textures", {"Texture.default"}},
			{"part_visibility", std::move(part_visibility)}};
	return {{"format_version", "1.8.0"}, {"render_controllers", std::move(controllers)}};
}

nlohmann::json pack_manifest(const std::string &header_id,
		const std::vector<std::string> &module_ids, bool behavior,
		std::optional<std::string> resource_id)
{
	nlohmann::json modules = nlohmann::json::array();
	if (behavior) {
		modules.push_back(
				{{"type", "data"}, {"uuid", module_ids.at(0)}, {"version", {1, 0, 0}}});
		modules.push_back({{"type", "script"}, {"language", "javascript"},
				{"uuid", module_ids.at(1)}, {"version", {1, 0, 0}},
				{"entry", "scripts/main.js"}});
	} else {
		modules.push_back({{"type", "resources"}, {"uuid", module_ids.at(0)},
				{"version", {1, 0, 0}}});
	}
	nlohmann::json manifest{{"format_version", 2},
			{"header",
					{{"name", "Arnis Vehicles"},
							{"description",
									behavior
											? "Cars and trains that drive this world's streets and tracks."
											: "What this world's cars and trains look like."},
							{"uuid", header_id}, {"version", {1, 0, 0}},
							{"min_engine_version", MIN_ENGINE}}},
			{"modules", std::move(modules)}};
	if (behavior) {
		manifest["dependencies"] =
				nlohmann::json::array({{{"module_name", "@minecraft/server"},
											   {"version", std::string(SERVER_MODULE)}},
						{{"uuid", *resource_id}, {"version", {1, 0, 0}}}});
	}
	return manifest;
}
Addon build(const nlohmann::json &data, const std::vector<Body> &bodies)
{
	const std::string behavior_id = new_uuid();
	const std::string resources_id = new_uuid();
	const std::vector<std::string> behavior_module_ids{new_uuid(), new_uuid()};
	const std::vector<std::string> resources_module_ids{new_uuid()};
	const auto icon = read_file(assets::path("minecraft/icon.png"));

	Addon addon;
	addon.behavior.id = behavior_id;
	addon.behavior.folder = "arnis_vehicles";
	addon.resources.id = resources_id;
	addon.resources.folder = "arnis_vehicles";
	add_json(addon.behavior, "manifest.json",
			pack_manifest(behavior_id, behavior_module_ids, true, resources_id));
	addon.behavior.files.emplace_back("pack_icon.png", icon);
	add_text(addon.behavior, "scripts/data.js", data_module(data));
	for (const auto script : SCRIPTS) {
		const auto path = std::string("scripts/") + std::string(script);
		addon.behavior.files.emplace_back(
				path, read_file(assets::path(
							  std::filesystem::path("minecraft/bedrock_addon") / path)));
	}

	add_json(addon.resources, "manifest.json",
			pack_manifest(resources_id, resources_module_ids, false, std::nullopt));
	addon.resources.files.emplace_back("pack_icon.png", icon);
	auto texture = models::texture();
	if (texture.empty())
		throw std::runtime_error("could not encode the Bedrock vehicle palette texture");
	addon.resources.files.emplace_back(std::string(TEXTURE) + ".png", std::move(texture));
	add_json(addon.resources, "render_controllers/arnis_vehicles.render_controllers.json",
			render_controllers());
	add_json(addon.resources, "texts/languages.json", nlohmann::json::array({"en_US"}));
	std::string language;
	for (const auto &body : bodies) {
		add_json(
				addon.behavior, "entities/" + body.name + ".json", behavior_entity(body));
		add_json(addon.resources, "entity/" + body.name + ".entity.json",
				client_entity(body));
		add_json(addon.resources, "models/entity/" + body.name + ".geo.json",
				models::geometry(body.name, body.shapes), -1);
		language += "entity.arnis:" + body.name +
					".name=" + (body.fleet == Fleet::Car ? "Car\n" : "Train\n");
	}
	add_text(addon.resources, "texts/en_US.lang", std::move(language));
	return addon;
}
} // namespace

std::optional<Addon> vehicles(const highways::cars::CarNetwork *cars,
		const trains::TrainRouteNetwork *trains,
		const std::function<std::optional<int>(std::uint64_t, int, int)> &rail_y)
{
	const bool has_cars = cars && cars->car_count() > 0;
	nlohmann::json data{{"cars", nullptr}, {"trains", nullptr}};
	std::vector<Body> bodies;
	bool has_trains = false;
	if (has_cars) {
		data["cars"] = cars->bedrock_data();
		bodies = cars->bedrock_bodies();
	}
	if (trains && !trains->is_empty()) {
		auto [train_data, train_bodies] = trains->bedrock_data(rail_y);
		const auto &services = train_data["services"];
		if (services.is_array() && !services.empty()) {
			has_trains = true;
			data["trains"] = std::move(train_data);
			bodies.insert(bodies.end(), std::make_move_iterator(train_bodies.begin()),
					std::make_move_iterator(train_bodies.end()));
		}
	}
	if (!has_cars && !has_trains)
		return std::nullopt;
	return build(data, bodies);
}

} // namespace arnis::bedrock_addon
