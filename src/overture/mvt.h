#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace arnis::overture::mvt
{
inline constexpr std::uint32_t GEOM_POLYGON = 3;
using Value = std::variant<std::string, double, std::int64_t, std::uint64_t, bool>;
struct Ring
{
	std::vector<std::pair<int, int>> points;
	std::int64_t area2 = 0;
	bool exterior() const { return area2 > 0; }
};
struct Feature
{
	std::uint32_t geom_type = 0;
	std::vector<std::uint32_t> tags;
	std::vector<Ring> rings;
};
struct Layer
{
	std::string name;
	std::uint32_t extent = 4096;
	std::vector<std::string> keys;
	std::vector<Value> values;
	std::vector<Feature> features;
	std::optional<Value> attribute(const Feature &, const std::string &) const;
};
std::optional<std::vector<Layer>> decode(const std::vector<std::uint8_t> &);
} // namespace arnis::overture::mvt
