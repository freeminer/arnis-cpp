#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace arnis::vehicle_models
{
inline constexpr double LIFT = 1.0;

struct Meta
{
	std::array<double, 3> centre{};
	double scale = 1.0;
	std::map<std::string, std::array<double, 3>> bones;
	std::array<std::array<double, 3>, 2> bounds{};
	std::array<std::uint32_t, 2> texture{};
};

struct Model
{
	std::string id;
	Meta meta;
	bool brake = false;
	bool parked = false;
	bool gear_up = false;
	nlohmann::json geometry;
	nlohmann::json java;
	nlohmann::json brake_java;
	nlohmann::json parked_java;
	nlohmann::json up_java;

	double top_m() const;
	std::vector<std::pair<std::string, double>> wheels() const;
};

struct Look
{
	std::string model;
	std::string livery;
	std::array<std::string, 5> colours{};
	bool backward = false;

	static Look plain(std::string model, std::string livery, std::string colour);
	std::string name() const;
};

struct PackFile
{
	std::string path;
	std::vector<std::uint8_t> bytes;
};

const Model &model(std::string_view id);
std::vector<std::string> model_ids();
std::string item_model(const Look &look);
std::string item_model_named(std::string_view model, std::string_view livery);
std::string brake_item_model(std::string_view model);
std::string parked_item_model(const Look &look);
bool flies(std::string_view model);

std::array<double, 3> display_translation(
		std::string_view model, double size, bool backward, double lift = 0.0);
std::array<double, 4> display_rotation(std::string_view model, bool backward);
double display_scale(std::string_view model, double size);
std::pair<double, double> culling_box(std::string_view model, double size);
nlohmann::json display_nbt(std::string_view model, std::string_view item_model,
		double size, bool backward, double lift = 0.0);
std::string display_snbt(std::string_view model, std::string_view item_model, double size,
		bool backward, double lift = 0.0, std::string_view extra = {});

nlohmann::json bedrock_geometry(
		std::string_view model, std::string_view identifier, double size, bool backward);
std::vector<std::uint8_t> texture(const Look &look, bool bedrock = false);
std::uint64_t atlas_px(const std::vector<Look> &looks);
std::vector<PackFile> java_pack(
		const std::vector<Look> &looks, const std::vector<Look> &parked = {});
} // namespace arnis::vehicle_models
