#pragma once

#include "source.h"
#include "crs.h"
#include "fetch.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <tuple>
#include <vector>

namespace arnis::survey::copc
{
struct PointLayout
{
	std::uint8_t format = 0;
	std::size_t record_len = 0;
	std::array<double, 3> scale{};
	std::array<double, 3> offset{};

	std::optional<std::string> check() const;
};

struct Info
{
	std::array<double, 3> center{};
	double halfsize = 0.0;
	std::uint64_t root_offset = 0;
	std::uint64_t root_size = 0;
};

struct Header
{
	PointLayout layout;
	source::Rect bounds{};
	std::optional<std::string> wkt;
	Info info;
	std::vector<std::uint8_t> laz_vlr;
};

struct HeaderParse
{
	std::optional<Header> header;
	std::uint64_t consumed = 0;
	std::uint64_t need_bytes = 0;
	std::string error;
};

struct Node
{
	std::int32_t level = 0;
	std::int32_t x = 0;
	std::int32_t y = 0;
	std::uint64_t offset = 0;
	std::uint64_t size = 0;
	std::uint64_t count = 0;

	bool operator==(const Node &) const = default;
};

struct PageRef
{
	std::int32_t level = 0;
	std::int32_t x = 0;
	std::int32_t y = 0;
	std::uint64_t offset = 0;
	std::uint64_t size = 0;

	bool operator==(const PageRef &) const = default;
};

struct Page
{
	std::vector<Node> nodes;
	std::vector<PageRef> pages;
};

struct ReadStats
{
	std::int32_t level = 0;
	std::size_t nodes = 0;
	std::size_t requests = 0;
	std::uint64_t bytes = 0;
	std::uint64_t points = 0;
};

struct Opened
{
	Header header;
	std::vector<Node> nodes;
	std::size_t requests = 0;
	source::Rect window{};
};

using Place = std::function<std::variant<std::optional<crs::Affine>, std::string>(
		const std::optional<std::string> &)>;
using OpenResult = std::variant<Opened, fetch::Failure>;

inline constexpr std::int32_t MAX_LEVEL = 24;
inline constexpr std::uint64_t HEADER_BYTES = 65'536;
inline constexpr std::uint64_t MAX_PAGE_BYTES = 16ULL << 20;
inline constexpr std::uint64_t MAX_NODE_BYTES = 256ULL << 20;
inline constexpr std::uint64_t MAX_NODE_POINTS = 10'000'000;

HeaderParse parse_header(std::span<const std::uint8_t> bytes);
std::optional<Page> parse_page(
		std::span<const std::uint8_t> bytes, std::string *error = nullptr);

source::Rect node_rect(
		const Info &info, std::int32_t level, std::int32_t x, std::int32_t y);
double overlap(const source::Rect &a, const source::Rect &b);
std::int32_t choose_level(const Info &info, const std::vector<Node> &nodes,
		const source::Rect &bounds, double target_density);
std::vector<Node> select(const Info &info, const std::vector<Node> &nodes,
		const source::Rect &window, std::int32_t level);
using Range = std::tuple<std::uint64_t, std::uint64_t, std::vector<Node>>;
std::vector<Range> merge_ranges(
		const std::vector<Node> &nodes, std::uint64_t max_gap, std::uint64_t max_len);
OpenResult open(
		const std::string &url, const source::Rect &window, const Place &place = {});
} // namespace arnis::survey::copc
