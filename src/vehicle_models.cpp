#include "vehicle_models.h"

#include "assets_root.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <iterator>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <png.h>

namespace arnis::vehicle_models
{
namespace
{
constexpr std::string_view NAMESPACE = "arnis";
constexpr std::string_view CARRIER = "minecraft:stone";
constexpr std::uint8_t LIT = 1;

struct Decoded
{
	std::uint32_t width{};
	std::uint32_t height{};
	std::vector<std::uint8_t> image;
	std::vector<std::uint8_t> mask;
};

std::mutex model_mutex;
std::unordered_map<std::string, std::shared_ptr<const Model>> models;
std::mutex decoded_mutex;
std::unordered_map<std::string, std::shared_ptr<const Decoded>> decoded_models;
std::mutex paints_mutex;
std::optional<std::unordered_map<std::string, std::array<std::uint8_t, 3>>> paints;

std::filesystem::path model_dir()
{
	return assets::path("vehicles/out");
}

std::string read_text(const std::filesystem::path &path)
{
	std::ifstream input(path, std::ios::binary);
	if (!input)
		throw std::runtime_error("cannot open " + path.string());
	return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

nlohmann::json read_json(const std::filesystem::path &path)
{
	return nlohmann::json::parse(read_text(path));
}

bool read_rgba(const std::filesystem::path &path, std::uint32_t &width,
		std::uint32_t &height, std::vector<std::uint8_t> &pixels)
{
	png_image image{};
	image.version = PNG_IMAGE_VERSION;
	if (!png_image_begin_read_from_file(&image, path.string().c_str()))
		return false;
	image.format = PNG_FORMAT_RGBA;
	width = image.width;
	height = image.height;
	pixels.resize(PNG_IMAGE_SIZE(image));
	if (!png_image_finish_read(&image, nullptr, pixels.data(), 0, nullptr)) {
		pixels.clear();
		png_image_free(&image);
		return false;
	}
	png_image_free(&image);
	return true;
}

std::vector<std::uint8_t> encode_rgba(std::uint32_t width, std::uint32_t height,
		const std::vector<std::uint8_t> &pixels)
{
	if (!width || !height || pixels.size() != std::uint64_t(width) * height * 4)
		return {};
	png_image image{};
	image.version = PNG_IMAGE_VERSION;
	image.width = width;
	image.height = height;
	image.format = PNG_FORMAT_RGBA;
	png_alloc_size_t size = 0;
	if (!png_image_write_to_memory(&image, nullptr, &size, 0, pixels.data(), 0, nullptr))
		return {};
	std::vector<std::uint8_t> result(size);
	if (!png_image_write_to_memory(
				&image, result.data(), &size, 0, pixels.data(), 0, nullptr))
		return {};
	result.resize(size);
	return result;
}

std::shared_ptr<const Decoded> decode(std::string_view id)
{
	const std::string key(id);
	{
		std::lock_guard lock(decoded_mutex);
		if (const auto found = decoded_models.find(key); found != decoded_models.end())
			return found->second;
	}
	const auto root = model_dir();
	auto value = std::make_shared<Decoded>();
	std::uint32_t mask_width = 0, mask_height = 0;
	if (!read_rgba(root / (key + ".png"), value->width, value->height, value->image) ||
			!read_rgba(
					root / (key + ".mask.png"), mask_width, mask_height, value->mask) ||
			value->width != mask_width || value->height != mask_height)
		throw std::runtime_error("invalid vehicle texture or mask for " + key);
	value->mask.shrink_to_fit();
	std::lock_guard lock(decoded_mutex);
	const auto [entry, inserted] = decoded_models.emplace(key, value);
	return inserted ? value : entry->second;
}

const auto &paint_table()
{
	std::lock_guard lock(paints_mutex);
	if (!paints) {
		std::unordered_map<std::string, std::array<std::uint8_t, 3>> parsed;
		const auto json = read_json(assets::path("vehicles/paints.json"));
		for (auto it = json.begin(); it != json.end(); ++it) {
			if (!it.value().is_array() || it.value().size() < 3)
				continue;
			std::array<std::uint8_t, 3> rgb{};
			for (std::size_t i = 0; i < rgb.size(); ++i) {
				const auto channel = it.value()[i].get<int>();
				rgb[i] = static_cast<std::uint8_t>(std::clamp(channel, 0, 255));
			}
			parsed.emplace(it.key(), rgb);
		}
		paints = std::move(parsed);
	}
	return *paints;
}

std::array<std::uint8_t, 3> shade(const std::array<std::uint8_t, 3> &base, double amount)
{
	std::array<std::uint8_t, 3> result{};
	for (std::size_t i = 0; i < result.size(); ++i) {
		const double value = base[i];
		const double out =
				amount >= 0.0 ? value + (255.0 - value) * amount : value * (1.0 + amount);
		result[i] = static_cast<std::uint8_t>(std::clamp(std::lround(out), 0l, 255l));
	}
	return result;
}

std::string colour_texture_name(const Look &look)
{
	return std::string(NAMESPACE) + ":block/vehicle/" + look.name();
}

std::string item_definition_model(std::string_view model_name)
{
	return std::string(NAMESPACE) + ":item/vehicle/" + std::string(model_name);
}

nlohmann::json item_definition(std::string_view model_name)
{
	return {{"model", {{"type", "minecraft:model"}, {"model", std::string(model_name)}}}};
}

std::string json_bytes(const nlohmann::json &value)
{
	return value.dump();
}

void add_text(std::vector<PackFile> &out, std::string path, const std::string &text)
{
	out.push_back({std::move(path), std::vector<std::uint8_t>(text.begin(), text.end())});
}

std::array<double, 3> json_vec3(const nlohmann::json &value)
{
	if (!value.is_array() || value.size() != 3)
		throw std::runtime_error("vehicle model vector must have three components");
	return {value[0].get<double>(), value[1].get<double>(), value[2].get<double>()};
}

void lower_json(nlohmann::json &value, double amount)
{
	if (value.is_array() && value.size() >= 2 && value[1].is_number())
		value[1] = std::round((value[1].get<double>() - amount) * 1000.0) / 1000.0;
}

void scale_json(nlohmann::json &value, double factor)
{
	if (!value.is_array())
		return;
	for (auto &entry : value)
		if (entry.is_number())
			entry = std::round(entry.get<double>() * factor * 1000.0) / 1000.0;
}
} // namespace

const Model &model(std::string_view id)
{
	const std::string key(id);
	{
		std::lock_guard lock(model_mutex);
		if (const auto found = models.find(key); found != models.end())
			return *found->second;
	}
	const auto root = model_dir();
	auto value = std::make_shared<Model>();
	value->id = key;
	const auto meta = read_json(root / (key + ".meta.json"));
	const auto &java = meta.at("java");
	value->meta.centre = json_vec3(java.at("centre"));
	value->meta.scale = java.at("scale").get<double>();
	value->meta.bounds = {
			json_vec3(meta.at("bounds").at(0)), json_vec3(meta.at("bounds").at(1))};
	const auto &texture_size = meta.at("texture");
	value->meta.texture = {texture_size.at(0).get<std::uint32_t>(),
			texture_size.at(1).get<std::uint32_t>()};
	for (auto it = meta.at("bones").begin(); it != meta.at("bones").end(); ++it)
		value->meta.bones.emplace(it.key(), json_vec3(it.value()));
	value->brake = std::filesystem::is_regular_file(root / (key + "_brake.java.json"));
	value->parked = std::filesystem::is_regular_file(root / (key + "_parked.java.json"));
	value->gear_up = std::filesystem::is_regular_file(root / (key + "_up.java.json"));
	value->geometry = read_json(root / (key + ".geo.json"));
	value->java = read_json(root / (key + ".java.json"));
	if (value->brake)
		value->brake_java = read_json(root / (key + "_brake.java.json"));
	if (value->parked)
		value->parked_java = read_json(root / (key + "_parked.java.json"));
	if (value->gear_up)
		value->up_java = read_json(root / (key + "_up.java.json"));
	std::lock_guard lock(model_mutex);
	const auto [entry, inserted] = models.emplace(key, value);
	return inserted ? *value : *entry->second;
}

std::vector<std::string> model_ids()
{
	std::vector<std::string> ids;
	std::error_code error;
	for (std::filesystem::directory_iterator it(model_dir(), error), end;
			!error && it != end; it.increment(error)) {
		if (!it->is_regular_file(error))
			continue;
		const auto filename = it->path().filename().string();
		constexpr std::string_view suffix = ".meta.json";
		if (filename.ends_with(suffix))
			ids.push_back(filename.substr(0, filename.size() - suffix.size()));
	}
	std::sort(ids.begin(), ids.end());
	return ids;
}

double Model::top_m() const
{
	return meta.bounds[1][1] / 16.0;
}

std::vector<std::pair<std::string, double>> Model::wheels() const
{
	std::vector<std::pair<std::string, double>> result;
	for (const auto &[name, pivot] : meta.bones)
		if (name.starts_with("wheel"))
			result.emplace_back(name, pivot[1]);
	return result;
}

Look Look::plain(std::string model_name, std::string livery_name, std::string colour)
{
	return {std::move(model_name), std::move(livery_name),
			{colour, colour, colour, colour, colour}, false};
}

std::string Look::name() const
{
	return model + "_" + livery;
}

std::string item_model(const Look &look)
{
	return std::string(NAMESPACE) + ":vehicle/" + look.name();
}

std::string item_model_named(std::string_view model_name, std::string_view livery)
{
	return std::string(NAMESPACE) + ":vehicle/" + std::string(model_name) + "_" +
		   std::string(livery);
}

std::string brake_item_model(std::string_view model_name)
{
	return std::string(NAMESPACE) + ":vehicle/" + std::string(model_name) + "_brake";
}

std::string parked_item_model(const Look &look)
{
	return std::string(NAMESPACE) + ":vehicle/" + look.name() + "_parked";
}

bool flies(std::string_view model_name)
{
	return model_name.starts_with("plane_");
}

std::array<double, 3> display_translation(
		std::string_view model_name, double size, bool backward, double lift)
{
	const auto &frame = model(model_name).meta;
	const std::array<double, 3> centre{frame.centre[0] * size / 16.0,
			frame.centre[1] * size / 16.0, frame.centre[2] * size / 16.0};
	return backward ? std::array<double, 3>{centre[0], centre[1] - lift, centre[2]}
					: std::array<double, 3>{-centre[0], centre[1] - lift, -centre[2]};
}

std::array<double, 4> display_rotation(std::string_view, bool backward)
{
	return backward ? std::array<double, 4>{0.0, 1.0, 0.0, 0.0}
					: std::array<double, 4>{0.0, 0.0, 0.0, 1.0};
}

double display_scale(std::string_view model_name, double size)
{
	return size / model(model_name).meta.scale;
}

std::pair<double, double> culling_box(std::string_view model_name, double size)
{
	const auto &bounds = model(model_name).meta.bounds;
	const auto &lo = bounds[0];
	const auto &hi = bounds[1];
	double reach = 0.0;
	for (const auto x : {std::abs(lo[0]), std::abs(hi[0])})
		for (const auto z : {std::abs(lo[2]), std::abs(hi[2])})
			reach = std::max(reach, std::hypot(x, z));
	return {(2.0 * reach / 16.0 + 0.5) * size, (hi[1] / 16.0 + 0.5) * size};
}

nlohmann::json display_nbt(std::string_view model_name, std::string_view item,
		double size, bool backward, double lift)
{
	const auto translation = display_translation(model_name, size, backward, lift);
	const auto rotation = display_rotation(model_name, backward);
	const double scale = display_scale(model_name, size);
	const auto [width, height] = culling_box(model_name, size);
	return {{"item", {{"id", std::string(CARRIER)}, {"count", 1},
							 {"components",
									 {{"minecraft:item_model", std::string(item)}}}}},
			{"width", static_cast<float>(width)}, {"height", static_cast<float>(height)},
			{"transformation", {{"left_rotation", rotation},
									   {"right_rotation", {0.0, 0.0, 0.0, 1.0}},
									   {"translation", translation},
									   {"scale", {scale, scale, scale}}}}};
}

std::string display_snbt(std::string_view model_name, std::string_view item, double size,
		bool backward, double lift, std::string_view extra)
{
	const auto translation = display_translation(model_name, size, backward, lift);
	const auto rotation = display_rotation(model_name, backward);
	const double scale = display_scale(model_name, size);
	const auto [width, height] = culling_box(model_name, size);
	std::ostringstream out;
	out << std::fixed << std::setprecision(4) << "item:{id:\"" << CARRIER
		<< "\",count:1,components:{\"minecraft:item_model\":\"" << item
		<< "\"}},transformation:{left_rotation:[" << rotation[0] << "f," << rotation[1]
		<< "f," << rotation[2] << "f," << rotation[3]
		<< "f],right_rotation:[0f,0f,0f,1f],translation:[" << translation[0] << "f,"
		<< translation[1] << "f," << translation[2] << "f],scale:[" << scale << "f,"
		<< scale << "f," << scale << "f]},width:" << std::setprecision(2) << width
		<< "f,height:" << height << "f" << extra;
	return out.str();
}

nlohmann::json bedrock_geometry(std::string_view model_name, std::string_view identifier,
		double size, bool backward)
{
	auto geometry = model(model_name).geometry;
	auto &entry = geometry["minecraft:geometry"][0];
	auto &description = entry["description"];
	description["identifier"] = std::string(identifier);
	for (const auto key : {"visible_bounds_width", "visible_bounds_height"})
		description[key] = std::ceil(description.value(key, 1.0) * size);
	scale_json(description["visible_bounds_offset"], size);
	lower_json(description["visible_bounds_offset"], LIFT);
	if (auto &bones = entry["bones"]; bones.is_array()) {
		for (auto &bone : bones) {
			scale_json(bone["pivot"], size);
			lower_json(bone["pivot"], LIFT * 16.0);
			if (auto &cubes = bone["cubes"]; cubes.is_array())
				for (auto &cube : cubes) {
					for (const auto key : {"origin", "size", "pivot"})
						scale_json(cube[key], size);
					lower_json(cube["origin"], LIFT * 16.0);
					lower_json(cube["pivot"], LIFT * 16.0);
				}
			if (backward)
				bone["parent"] = "turn";
		}
		if (backward) {
			nlohmann::json turn{
					{"name", "turn"}, {"pivot", {0, 0, 0}}, {"rotation", {0, 180, 0}}};
			bones.insert(bones.begin(), std::move(turn));
		}
	}
	return geometry;
}

std::vector<std::uint8_t> texture(const Look &look, bool bedrock)
{
	const auto image = decode(look.model);
	if (image->image.size() != image->mask.size())
		return {};
	auto pixels = image->image;
	std::array<std::array<std::array<std::uint8_t, 3>, 256>, 5> shades{};
	const auto &palette = paint_table();
	for (std::size_t slot = 0; slot < look.colours.size(); ++slot) {
		const auto paint = palette.find(look.colours[slot]);
		if (paint == palette.end())
			throw std::runtime_error("no paint for " + look.colours[slot]);
		for (unsigned k = 0; k < 256; ++k)
			shades[slot][k] = shade(paint->second, double(k) / 127.5 - 1.0);
	}
	for (std::size_t i = 0; i < pixels.size(); i += 4) {
		const auto slot = image->mask[i];
		const auto amount = image->mask[i + 1];
		const auto flags = image->mask[i + 2];
		const auto used = image->mask[i + 3];
		if (used == 0) {
			pixels[i] = pixels[i + 1] = pixels[i + 2] = pixels[i + 3] = 0;
			continue;
		}
		if (slot >= 1 && slot <= 5) {
			const auto &rgb = shades[slot - 1][amount];
			pixels[i] = rgb[0];
			pixels[i + 1] = rgb[1];
			pixels[i + 2] = rgb[2];
			pixels[i + 3] = 255;
		} else {
			pixels[i + 3] = 255;
		}
		if (bedrock && (flags & LIT))
			pixels[i + 3] = 0;
	}
	return encode_rgba(image->width, image->height, pixels);
}

std::uint64_t atlas_px(const std::vector<Look> &looks)
{
	std::set<std::pair<std::string, std::string>> seen;
	std::uint64_t pixels = 0;
	for (const auto &look : looks)
		if (seen.emplace(look.model, look.livery).second) {
			const auto size = model(look.model).meta.texture;
			pixels += std::uint64_t(size[0]) * size[1];
		}
	return pixels;
}

std::vector<PackFile> java_pack(
		const std::vector<Look> &looks, const std::vector<Look> &parked)
{
	std::map<std::string, std::vector<const Look *>> by_model;
	auto add_look = [&](const Look &look) {
		auto &entries = by_model[look.model];
		if (std::none_of(entries.begin(), entries.end(),
					[&](const Look *known) { return known->livery == look.livery; }))
			entries.push_back(&look);
	};
	for (const auto &look : looks)
		add_look(look);
	for (const auto &look : parked)
		add_look(look);
	const auto parks = [&](const Look *look) {
		return std::any_of(parked.begin(), parked.end(), [&](const Look &candidate) {
			return candidate.model == look->model && candidate.livery == look->livery;
		});
	};
	std::vector<PackFile> out;
	for (const auto &[id, entries] : by_model) {
		const auto &base = model(id);
		const std::string first_texture = colour_texture_name(*entries.front());
		auto add_model = [&](std::string suffix, nlohmann::json json) {
			json["textures"] = {{"t", first_texture}, {"particle", first_texture}};
			add_text(out, "assets/arnis/models/item/vehicle/" + id + suffix + ".json",
					json_bytes(json));
		};
		add_model("", base.java);
		if (base.brake)
			add_model("_brake", base.brake_java);
		if (base.gear_up)
			add_model("_up", base.up_java);
		if (base.brake)
			add_text(out, "assets/arnis/items/vehicle/" + id + "_brake.json",
					json_bytes(item_definition(item_definition_model(id + "_brake"))));
		if (std::any_of(entries.begin(), entries.end(), parks)) {
			if (!base.parked)
				throw std::runtime_error(
						"vehicle model " + id + " has no parked variant");
			add_model("_parked", base.parked_java);
		}
		for (const auto *look : entries) {
			const auto name = look->name();
			const auto texture_id = colour_texture_name(*look);
			out.push_back({"assets/arnis/textures/block/vehicle/" + name + ".png",
					texture(*look)});
			auto child = [&](std::string suffix, std::string parent) {
				const nlohmann::json json{{"parent", std::move(parent)},
						{"textures", {{"t", texture_id}, {"particle", texture_id}}}};
				add_text(out,
						"assets/arnis/models/item/vehicle/" + name + suffix + ".json",
						json_bytes(json));
			};
			add_text(out, "assets/arnis/items/vehicle/" + name + ".json",
					json_bytes(item_definition(item_definition_model(name))));
			child("", item_definition_model(id));
			if (flies(id)) {
				if (base.gear_up)
					child("_up", item_definition_model(id + "_up"));
				const auto up_parent = base.gear_up ? name + "_up" : name;
				add_text(out, "assets/arnis/items/vehicle/" + name + "_up.json",
						json_bytes(item_definition(item_definition_model(up_parent))));
			}
			if (parks(look)) {
				child("_parked", item_definition_model(id + "_parked"));
				add_text(out, "assets/arnis/items/vehicle/" + name + "_parked.json",
						json_bytes(item_definition(
								item_definition_model(name + "_parked"))));
			}
		}
	}
	return out;
}
} // namespace arnis::vehicle_models
