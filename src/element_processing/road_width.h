#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

namespace arnis::road_width
{

inline constexpr int MAX_HALF = 8;

struct RoadHalves
{
	int lanes = 1;
	int kerb = 1;
	bool operator==(const RoadHalves &) const = default;
};

std::optional<double> metres(std::string_view raw);
std::optional<int> count(std::string_view raw);
RoadHalves road_halves(const std::string &highway_type,
		const std::unordered_map<std::string, std::string> &tags, double scale);
std::pair<int, int> kerb_sides(const std::string &highway_type,
		const std::unordered_map<std::string, std::string> &tags, double scale);
int wall_half(const std::string &highway_type,
		const std::unordered_map<std::string, std::string> &tags, double scale);
bool parks_along(const std::unordered_map<std::string, std::string> &tags);
std::optional<std::string_view> parking_orientation(
		const std::unordered_map<std::string, std::string> &tags, std::string_view side);

} // namespace arnis::road_width
