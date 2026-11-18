#include "copc.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <bit>
#include <limits>
#include <numeric>

namespace arnis::survey::copc
{
namespace
{
constexpr std::array<std::size_t, 11> MIN_RECORD{
		20, 28, 26, 34, 57, 63, 30, 36, 38, 59, 67};
constexpr std::size_t LAS_MIN_HEADER = 375;
constexpr std::size_t VLR_HEADER_SIZE = 54;
constexpr std::size_t COPC_INFO_SIZE = 72;
constexpr std::size_t PAGE_ENTRY_SIZE = 32;
constexpr std::size_t MAX_HIERARCHY_REQUESTS = 200;

std::uint16_t read_u16(std::span<const std::uint8_t> data, std::size_t offset)
{
	return static_cast<std::uint16_t>(data[offset]) |
		   (static_cast<std::uint16_t>(data[offset + 1]) << 8);
}

std::uint32_t read_u32(std::span<const std::uint8_t> data, std::size_t offset)
{
	std::uint32_t value = 0;
	for (unsigned i = 0; i < 4; ++i)
		value |= static_cast<std::uint32_t>(data[offset + i]) << (i * 8);
	return value;
}

std::uint64_t read_u64(std::span<const std::uint8_t> data, std::size_t offset)
{
	std::uint64_t value = 0;
	for (unsigned i = 0; i < 8; ++i)
		value |= static_cast<std::uint64_t>(data[offset + i]) << (i * 8);
	return value;
}

std::int32_t read_i32(std::span<const std::uint8_t> data, std::size_t offset)
{
	return std::bit_cast<std::int32_t>(read_u32(data, offset));
}

double read_f64(std::span<const std::uint8_t> data, std::size_t offset)
{
	const auto bits = read_u64(data, offset);
	double value;
	std::memcpy(&value, &bits, sizeof(value));
	return value;
}

bool add_overflows(std::uint64_t a, std::uint64_t b)
{
	return b > std::numeric_limits<std::uint64_t>::max() - a;
}

std::string fixed_string(std::span<const std::uint8_t> data)
{
	std::string value(data.begin(), data.end());
	while (!value.empty() && value.back() == '\0')
		value.pop_back();
	return value;
}

} // namespace

std::optional<std::string> PointLayout::check() const
{
	if (format >= MIN_RECORD.size() || record_len < MIN_RECORD[format])
		return "point format " + std::to_string(format) + " in records of " +
			   std::to_string(record_len) + " bytes";
	return std::nullopt;
}

HeaderParse parse_header(std::span<const std::uint8_t> bytes)
{
	HeaderParse result;
	if (bytes.size() < LAS_MIN_HEADER || std::memcmp(bytes.data(), "LASF", 4) != 0) {
		result.error = "not a LAS/LAZ file";
		return result;
	}

	const auto header_size = static_cast<std::size_t>(read_u16(bytes, 94));
	const auto vlr_count = read_u32(bytes, 100);
	Header header;
	header.layout.format = bytes[104] & 0x3f;
	header.layout.record_len = read_u16(bytes, 105);
	for (std::size_t i = 0; i < 3; ++i) {
		header.layout.scale[i] = read_f64(bytes, 131 + i * 8);
		header.layout.offset[i] = read_f64(bytes, 155 + i * 8);
	}
	// LAS stores max/min pairs in the order X, Y, Z. Rects are west,
	// south, east, north.
	header.bounds = {read_f64(bytes, 187), read_f64(bytes, 203), read_f64(bytes, 179),
			read_f64(bytes, 195)};
	if (header.layout.format < 6) {
		result.error = "COPC point format " + std::to_string(header.layout.format);
		return result;
	}
	if (const auto error = header.layout.check()) {
		result.error = *error;
		return result;
	}
	if (header_size > bytes.size()) {
		result.need_bytes = header_size;
		result.error = "need " + std::to_string(header_size);
		return result;
	}

	std::optional<Info> info;
	std::size_t pos = header_size;
	for (std::uint32_t i = 0; i < vlr_count; ++i) {
		if (pos > bytes.size() || VLR_HEADER_SIZE > bytes.size() - pos) {
			const auto need =
					static_cast<std::uint64_t>(pos) + VLR_HEADER_SIZE + HEADER_BYTES;
			result.need_bytes = need;
			result.error = "need " + std::to_string(need);
			return result;
		}
		const auto user = fixed_string(bytes.subspan(pos + 2, 16));
		const auto record_id = read_u16(bytes, pos + 18);
		const auto record_size = read_u16(bytes, pos + 20);
		const auto data = pos + VLR_HEADER_SIZE;
		if (record_size > bytes.size() - data) {
			const auto need = static_cast<std::uint64_t>(data) + record_size;
			result.need_bytes = need;
			result.error = "need " + std::to_string(need);
			return result;
		}
		const auto payload = bytes.subspan(data, record_size);
		if (user == "copc" && record_id == 1 && payload.size() >= COPC_INFO_SIZE) {
			Info parsed;
			for (std::size_t axis = 0; axis < 3; ++axis)
				parsed.center[axis] = read_f64(payload, axis * 8);
			parsed.halfsize = read_f64(payload, 24);
			parsed.root_offset = read_u64(payload, 40);
			parsed.root_size = read_u64(payload, 48);
			info = parsed;
		} else if (user == "laszip encoded" && record_id == 22204) {
			header.laz_vlr.assign(payload.begin(), payload.end());
		} else if (user == "LASF_Projection" && record_id == 2112) {
			header.wkt = fixed_string(payload);
		}
		pos = data + record_size;
	}
	if (!info) {
		result.error = "no COPC info record";
		return result;
	}
	if (header.laz_vlr.empty()) {
		result.error = "no LAZ record";
		return result;
	}
	if (info->root_size > MAX_PAGE_BYTES) {
		result.error =
				"a hierarchy page of " + std::to_string(info->root_size) + " bytes";
		return result;
	}
	header.info = *info;
	result.consumed = pos;
	result.header = std::move(header);
	return result;
}

std::optional<Page> parse_page(std::span<const std::uint8_t> bytes, std::string *error)
{
	Page page;
	const auto count = bytes.size() / PAGE_ENTRY_SIZE;
	page.nodes.reserve(count);
	page.pages.reserve(count / 2);
	for (std::size_t index = 0; index < count; ++index) {
		const auto entry = bytes.subspan(index * PAGE_ENTRY_SIZE, PAGE_ENTRY_SIZE);
		const auto level = read_i32(entry, 0);
		const auto x = read_i32(entry, 4);
		const auto y = read_i32(entry, 8);
		const auto offset = read_u64(entry, 16);
		const auto signed_size = read_i32(entry, 24);
		const auto signed_count = read_i32(entry, 28);
		if (signed_size <= 0 || !(signed_count == -1 || signed_count > 0))
			continue;
		const auto size = static_cast<std::uint64_t>(signed_size);
		if (level < 0 || level > MAX_LEVEL || add_overflows(offset, size)) {
			if (error)
				*error = "a hierarchy entry at level " + std::to_string(level) + ", " +
						 std::to_string(size) + " bytes at " + std::to_string(offset);
			return std::nullopt;
		}
		if (signed_count == -1) {
			if (size > MAX_PAGE_BYTES) {
				if (error)
					*error = "a hierarchy page of " + std::to_string(size) + " bytes";
				return std::nullopt;
			}
			page.pages.push_back({level, x, y, offset, size});
		} else {
			const auto points = static_cast<std::uint64_t>(signed_count);
			if (points > MAX_NODE_POINTS || size > MAX_NODE_BYTES) {
				if (error)
					*error = "a node of " + std::to_string(points) + " points in " +
							 std::to_string(size) + " bytes";
				return std::nullopt;
			}
			page.nodes.push_back({level, x, y, offset, size, points});
		}
	}
	return page;
}

source::Rect node_rect(
		const Info &info, std::int32_t level, std::int32_t x, std::int32_t y)
{
	const auto bounded_level = std::clamp(level, 0, 30);
	const double size = 2.0 * info.halfsize / static_cast<double>(1U << bounded_level);
	const double west = info.center[0] - info.halfsize;
	const double south = info.center[1] - info.halfsize;
	return {west + static_cast<double>(x) * size, south + static_cast<double>(y) * size,
			west + (static_cast<double>(x) + 1.0) * size,
			south + (static_cast<double>(y) + 1.0) * size};
}

double overlap(const source::Rect &a, const source::Rect &b)
{
	const double width = std::min(a[2], b[2]) - std::max(a[0], b[0]);
	const double height = std::min(a[3], b[3]) - std::max(a[1], b[1]);
	return width > 0.0 && height > 0.0 ? width * height : 0.0;
}

std::int32_t choose_level(const Info &info, const std::vector<Node> &nodes,
		const source::Rect &bounds, double target_density)
{
	std::int32_t deepest = 0;
	for (const auto &node : nodes)
		deepest = std::max(deepest, node.level);
	deepest = std::max(deepest, 0);
	std::vector<double> points(static_cast<std::size_t>(deepest) + 1, 0.0);
	std::vector<double> areas(points.size(), 0.0);
	for (const auto &node : nodes) {
		if (node.level < 0)
			continue;
		const auto index = static_cast<std::size_t>(node.level);
		points[index] += static_cast<double>(node.count);
		areas[index] += overlap(node_rect(info, node.level, node.x, node.y), bounds);
	}
	double density = 0.0;
	for (std::size_t level = 0; level < points.size(); ++level) {
		if (areas[level] > 0.0)
			density += points[level] / areas[level];
		if (density >= target_density)
			return static_cast<std::int32_t>(level);
	}
	return deepest;
}

std::vector<Node> select(const Info &info, const std::vector<Node> &nodes,
		const source::Rect &window, std::int32_t level)
{
	std::vector<Node> chosen;
	for (const auto &node : nodes)
		if (node.level <= level &&
				overlap(node_rect(info, node.level, node.x, node.y), window) > 0.0)
			chosen.push_back(node);
	return chosen;
}

std::vector<Range> merge_ranges(
		const std::vector<Node> &nodes, std::uint64_t max_gap, std::uint64_t max_len)
{
	auto sorted = nodes;
	std::stable_sort(sorted.begin(), sorted.end(),
			[](const Node &a, const Node &b) { return a.offset < b.offset; });
	std::vector<Range> ranges;
	for (const auto &node : sorted) {
		if (!ranges.empty()) {
			auto &[start, length, group] = ranges.back();
			if (!add_overflows(start, length) && !add_overflows(node.offset, node.size)) {
				const auto end = start + length;
				if (node.offset >= end && node.offset - end <= max_gap &&
						node.offset + node.size >= start &&
						node.offset + node.size - start <= max_len) {
					length = node.offset + node.size - start;
					group.push_back(node);
					continue;
				}
			}
		}
		ranges.emplace_back(node.offset, node.size, std::vector<Node>{node});
	}
	return ranges;
}

OpenResult open(
		const std::string &url, const source::Rect &area_window, const Place &place)
{
	auto head_result = fetch::get_range(url, 0, HEADER_BYTES);
	if (auto *failure = std::get_if<fetch::Failure>(&head_result))
		return *failure;
	auto head = std::get<std::vector<std::uint8_t>>(std::move(head_result));
	std::size_t requests = 1;
	fetch::count_laser_bytes(head.size());
	Header header;
	for (;;) {
		auto parsed = parse_header(head);
		if (parsed.header) {
			header = std::move(*parsed.header);
			break;
		}
		if (parsed.need_bytes > head.size() && parsed.need_bytes < 16 * HEADER_BYTES) {
			auto expanded = fetch::get_range(url, 0, parsed.need_bytes);
			if (auto *failure = std::get_if<fetch::Failure>(&expanded))
				return failure->of_part();
			head = std::get<std::vector<std::uint8_t>>(std::move(expanded));
			++requests;
			fetch::count_laser_bytes(head.size());
			continue;
		}
		return fetch::Failure{fetch::Failure::Kind::Unusable, parsed.error};
	}

	auto window = area_window;
	if (place) {
		auto mapping = place(header.wkt);
		if (auto *error = std::get_if<std::string>(&mapping))
			return fetch::Failure{fetch::Failure::Kind::Unusable, *error};
		if (const auto *affine = std::get_if<std::optional<crs::Affine>>(&mapping);
				affine && *affine)
			window = (*affine)->inverse().rect(
					area_window[0], area_window[1], area_window[2], area_window[3]);
	}

	std::vector<Node> nodes;
	std::vector<PageRef> pages{{0, 0, 0, header.info.root_offset, header.info.root_size}};
	while (!pages.empty()) {
		const auto page_ref = pages.back();
		pages.pop_back();
		if (page_ref.level > 0 &&
				overlap(node_rect(header.info, page_ref.level, page_ref.x, page_ref.y),
						window) <= 0.0)
			continue;
		auto page_body = fetch::get_range(url, page_ref.offset, page_ref.size);
		if (auto *failure = std::get_if<fetch::Failure>(&page_body))
			return failure->of_part();
		auto &bytes = std::get<std::vector<std::uint8_t>>(page_body);
		++requests;
		fetch::count_laser_bytes(bytes.size());
		std::string error;
		auto page = parse_page(bytes, &error);
		if (!page)
			return fetch::Failure{fetch::Failure::Kind::Unusable, std::move(error)};
		nodes.insert(nodes.end(), page->nodes.begin(), page->nodes.end());
		pages.insert(pages.end(), page->pages.begin(), page->pages.end());
		if (requests > MAX_HIERARCHY_REQUESTS)
			return fetch::Failure{fetch::Failure::Kind::Other, "hierarchy too large"};
	}
	for (const auto &node : nodes) {
		if (node.count > MAX_NODE_BYTES / header.layout.record_len)
			return fetch::Failure{fetch::Failure::Kind::Unusable,
					"a node of " + std::to_string(node.count) + " points of " +
							std::to_string(header.layout.record_len) + " bytes"};
	}
	return Opened{std::move(header), std::move(nodes), requests, window};
}
} // namespace arnis::survey::copc
