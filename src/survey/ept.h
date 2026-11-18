#pragma once

#include "copc.h"
#include "fetch.h"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <variant>
#include <vector>

namespace arnis::survey::ept
{
struct Info
{
	source::Rect cube{};
};

struct Node
{
	std::int32_t level = 0;
	std::int32_t x = 0;
	std::int32_t y = 0;
	std::int32_t z = 0;
	std::uint64_t count = 0;

	std::string key() const;
	bool operator==(const Node &) const = default;
};

using Key = std::array<std::int32_t, 4>;

struct Page
{
	std::vector<Node> nodes;
	std::vector<Key> pages;
};

struct Hierarchy
{
	Info info;
	std::vector<Node> nodes;
	std::size_t requests = 0;
	double ground_factor = 1.0;
};

struct Decoded
{
	copc::PointLayout layout;
	std::vector<std::uint8_t> records;
};

std::optional<std::string> parse_info(const nlohmann::json &json, Info &out);
source::Rect node_rect(
		const Info &info, std::int32_t level, std::int32_t x, std::int32_t y);
bool meets(const source::Rect &a, const source::Rect &b);
Page parse_page(const Info &info, const nlohmann::json &page, const source::Rect &window);
std::int32_t choose_level(const Info &info, const std::vector<Node> &nodes,
		double ground_area_factor, double target_density);
std::vector<Node> select(const Info &info, const std::vector<Node> &nodes,
		std::int32_t level, double ground_area_factor);
std::string hierarchy_url(const std::string &base, const Key &key);
std::string data_url(const std::string &base, const Node &node);
std::variant<Decoded, std::string> decode(std::span<const std::uint8_t> data);

using HierarchyResult = std::variant<Hierarchy, fetch::Failure>;
using Visit =
		std::function<void(const copc::PointLayout &, std::span<const std::uint8_t>)>;
using ReadResult = std::variant<copc::ReadStats, fetch::Failure>;
HierarchyResult read_hierarchy(
		const std::string &ept_json_url, const source::Rect &window);
ReadResult read(const std::string &ept_json_url, const source::Rect &window,
		double target_density, std::size_t threads, const Visit &visit);
} // namespace arnis::survey::ept
