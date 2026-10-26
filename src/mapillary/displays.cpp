#include "displays.h"
#include "../../../arnis_world_editor.h"
#include "../assets_root.h"
#include <algorithm>
#include <array>
#include <bit>
#include <climits>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <png.h>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <zlib.h>
namespace arnis::mapillary::displays
{
namespace
{
constexpr std::string_view PACK_MARKER = ".arnis_facade_pack";
constexpr std::uint32_t PACK_FORMAT = 46;

struct ZipEntry
{
	std::string name;
	std::uint32_t crc{};
	std::uint32_t size{};
	std::uint64_t offset{};
	std::vector<std::uint8_t> central_record;
};

struct ExistingZipEntry
{
	std::string name;
	std::vector<std::uint8_t> central_record;
	std::uint64_t local_offset{};
	std::uint64_t range_end{};
};

struct ExistingZipArchive
{
	std::vector<ExistingZipEntry> entries;
	std::uint64_t central_offset{};
};

std::uint16_t read_le16(const std::uint8_t *bytes)
{
	return static_cast<std::uint16_t>(bytes[0]) |
		   (static_cast<std::uint16_t>(bytes[1]) << 8);
}

std::uint32_t read_le32(const std::uint8_t *bytes)
{
	return static_cast<std::uint32_t>(bytes[0]) |
		   (static_cast<std::uint32_t>(bytes[1]) << 8) |
		   (static_cast<std::uint32_t>(bytes[2]) << 16) |
		   (static_cast<std::uint32_t>(bytes[3]) << 24);
}

std::uint64_t read_le64(const std::uint8_t *bytes)
{
	return static_cast<std::uint64_t>(read_le32(bytes)) |
		   (static_cast<std::uint64_t>(read_le32(bytes + 4)) << 32);
}

void patch_le32(std::vector<std::uint8_t> &bytes, std::size_t offset, std::uint32_t value)
{
	for (unsigned int i = 0; i < 4; ++i)
		bytes[offset + i] = static_cast<std::uint8_t>(value >> (8 * i));
}

void patch_le64(std::vector<std::uint8_t> &bytes, std::size_t offset, std::uint64_t value)
{
	for (unsigned int i = 0; i < 8; ++i)
		bytes[offset + i] = static_cast<std::uint8_t>(value >> (8 * i));
}

std::optional<std::uint64_t> zip64_local_offset(const std::vector<std::uint8_t> &record)
{
	if (record.size() < 46)
		return std::nullopt;
	const auto name_size = read_le16(record.data() + 28);
	const auto extra_size = read_le16(record.data() + 30);
	const auto extra_start = std::size_t(46) + name_size;
	if (extra_start + extra_size > record.size())
		return std::nullopt;
	const bool need_uncompressed = read_le32(record.data() + 24) == UINT32_MAX;
	const bool need_compressed = read_le32(record.data() + 20) == UINT32_MAX;
	const bool need_offset = read_le32(record.data() + 42) == UINT32_MAX;
	const bool need_disk = read_le16(record.data() + 34) == UINT16_MAX;
	for (std::size_t pos = extra_start; pos + 4 <= extra_start + extra_size;) {
		const auto tag = read_le16(record.data() + pos);
		const auto size = read_le16(record.data() + pos + 2);
		pos += 4;
		if (pos + size > extra_start + extra_size)
			return std::nullopt;
		if (tag == 0x0001) {
			std::size_t value = pos;
			const auto end = pos + size;
			if (need_uncompressed) {
				if (value + 8 > end)
					return std::nullopt;
				value += 8;
			}
			if (need_compressed) {
				if (value + 8 > end)
					return std::nullopt;
				value += 8;
			}
			if (need_offset) {
				if (value + 8 > end)
					return std::nullopt;
				return read_le64(record.data() + value);
			}
			if (need_disk && value + 4 > end)
				return std::nullopt;
		}
		pos += size;
	}
	return std::nullopt;
}

bool set_central_local_offset(std::vector<std::uint8_t> &record, std::uint64_t offset)
{
	if (record.size() < 46 || record.size() != std::size_t(46) +
													   read_le16(record.data() + 28) +
													   read_le16(record.data() + 30) +
													   read_le16(record.data() + 32))
		return false;
	const auto name_size = read_le16(record.data() + 28);
	const auto extra_size = read_le16(record.data() + 30);
	const auto extra_start = std::size_t(46) + name_size;
	const bool need_uncompressed = read_le32(record.data() + 24) == UINT32_MAX;
	const bool need_compressed = read_le32(record.data() + 20) == UINT32_MAX;
	const bool had_zip64_offset = read_le32(record.data() + 42) == UINT32_MAX;
	if (offset < UINT32_MAX && !had_zip64_offset) {
		patch_le32(record, 42, static_cast<std::uint32_t>(offset));
		return true;
	}
	for (std::size_t pos = extra_start; pos + 4 <= extra_start + extra_size;) {
		const auto tag = read_le16(record.data() + pos);
		const auto size = read_le16(record.data() + pos + 2);
		const auto field = pos;
		pos += 4;
		if (pos + size > extra_start + extra_size)
			return false;
		if (tag == 0x0001) {
			std::size_t value = pos;
			if (need_uncompressed)
				value += 8;
			if (need_compressed)
				value += 8;
			if (had_zip64_offset) {
				if (value + 8 > pos + size)
					return false;
				patch_le64(record, value, offset);
			} else {
				if (value > pos + size || size > UINT16_MAX - 8 ||
						extra_size > UINT16_MAX - 8)
					return false;
				record.insert(record.begin() + pos + size,
						{static_cast<std::uint8_t>(offset),
								static_cast<std::uint8_t>(offset >> 8),
								static_cast<std::uint8_t>(offset >> 16),
								static_cast<std::uint8_t>(offset >> 24),
								static_cast<std::uint8_t>(offset >> 32),
								static_cast<std::uint8_t>(offset >> 40),
								static_cast<std::uint8_t>(offset >> 48),
								static_cast<std::uint8_t>(offset >> 56)});
				const auto new_size = static_cast<std::uint16_t>(size + 8);
				record[field + 2] = static_cast<std::uint8_t>(new_size);
				record[field + 3] = static_cast<std::uint8_t>(new_size >> 8);
				const auto new_extra_size = static_cast<std::uint16_t>(extra_size + 8);
				record[30] = static_cast<std::uint8_t>(new_extra_size);
				record[31] = static_cast<std::uint8_t>(new_extra_size >> 8);
			}
			patch_le32(record, 42, UINT32_MAX);
			return true;
		}
		pos += size;
	}
	if (extra_size > UINT16_MAX - 12)
		return false;
	const std::array<std::uint8_t, 12> field{1, 0, 8, 0,
			static_cast<std::uint8_t>(offset), static_cast<std::uint8_t>(offset >> 8),
			static_cast<std::uint8_t>(offset >> 16),
			static_cast<std::uint8_t>(offset >> 24),
			static_cast<std::uint8_t>(offset >> 32),
			static_cast<std::uint8_t>(offset >> 40),
			static_cast<std::uint8_t>(offset >> 48),
			static_cast<std::uint8_t>(offset >> 56)};
	record.insert(record.begin() + extra_start + extra_size, field.begin(), field.end());
	const auto new_extra_size = static_cast<std::uint16_t>(extra_size + field.size());
	record[30] = static_cast<std::uint8_t>(new_extra_size);
	record[31] = static_cast<std::uint8_t>(new_extra_size >> 8);
	patch_le32(record, 42, UINT32_MAX);
	return true;
}

bool read_zip_archive(const std::filesystem::path &path, ExistingZipArchive *archive)
{
	std::ifstream input(path, std::ios::binary);
	if (!input)
		return false;
	input.seekg(0, std::ios::end);
	const auto end = input.tellg();
	if (end < 22)
		return false;
	const auto file_size = static_cast<std::uint64_t>(end);
	const auto tail_size = static_cast<std::size_t>(
			std::min<std::uint64_t>(file_size, 22u + UINT16_MAX));
	std::vector<std::uint8_t> tail(tail_size);
	input.seekg(static_cast<std::streamoff>(file_size - tail_size));
	input.read(reinterpret_cast<char *>(tail.data()),
			static_cast<std::streamsize>(tail.size()));
	if (!input)
		return false;
	std::optional<std::size_t> eocd;
	for (std::size_t pos = tail.size() - 22 + 1; pos-- > 0;)
		if (read_le32(tail.data() + pos) == 0x06054b50) {
			const auto comment = read_le16(tail.data() + pos + 20);
			if (pos + 22 + comment == tail.size()) {
				eocd = pos;
				break;
			}
		}
	if (!eocd)
		return false;
	const auto *footer = tail.data() + *eocd;
	const auto disk = read_le16(footer + 4);
	const auto central_disk = read_le16(footer + 6);
	const auto disk_entries = read_le16(footer + 8);
	const auto entry_count = read_le16(footer + 10);
	std::uint64_t total_entries = entry_count;
	std::uint64_t central_size = read_le32(footer + 12);
	std::uint64_t central_offset = read_le32(footer + 16);
	const bool zip64_sentinel = entry_count == UINT16_MAX ||
								read_le32(footer + 12) == UINT32_MAX ||
								read_le32(footer + 16) == UINT32_MAX;
	if (disk || central_disk || (!zip64_sentinel && disk_entries != entry_count))
		return false; // Multi-volume archives are not supported.
	const auto footer_absolute = file_size - tail_size + *eocd;
	std::array<std::uint8_t, 20> locator{};
	bool has_zip64_locator = false;
	if (footer_absolute >= locator.size()) {
		input.clear();
		input.seekg(static_cast<std::streamoff>(footer_absolute - locator.size()));
		input.read(reinterpret_cast<char *>(locator.data()), locator.size());
		has_zip64_locator = input && read_le32(locator.data()) == 0x07064b50;
	}
	if (zip64_sentinel || has_zip64_locator) {
		if (!has_zip64_locator)
			return false;
		if (read_le32(locator.data() + 4) != 0 || read_le32(locator.data() + 16) != 1)
			return false;
		const auto zip64_offset = read_le64(locator.data() + 8);
		if (zip64_offset > file_size || file_size - zip64_offset < 56)
			return false;
		std::array<std::uint8_t, 56> zip64{};
		input.clear();
		input.seekg(static_cast<std::streamoff>(zip64_offset));
		input.read(reinterpret_cast<char *>(zip64.data()), zip64.size());
		if (!input || read_le32(zip64.data()) != 0x06064b50 ||
				read_le64(zip64.data() + 4) < 44 || read_le32(zip64.data() + 16) != 0 ||
				read_le32(zip64.data() + 20) != 0 ||
				read_le64(zip64.data() + 24) != read_le64(zip64.data() + 32))
			return false;
		total_entries = read_le64(zip64.data() + 32);
		central_size = read_le64(zip64.data() + 40);
		central_offset = read_le64(zip64.data() + 48);
	}
	if (total_entries > std::numeric_limits<std::size_t>::max() ||
			central_offset > file_size || central_size > file_size - central_offset)
		return false;
	input.clear();
	input.seekg(static_cast<std::streamoff>(central_offset));
	archive->entries.clear();
	archive->entries.reserve(static_cast<std::size_t>(total_entries));
	archive->central_offset = central_offset;
	for (std::uint64_t i = 0; i < total_entries; ++i) {
		std::array<std::uint8_t, 46> header{};
		input.read(reinterpret_cast<char *>(header.data()), header.size());
		if (!input || read_le32(header.data()) != 0x02014b50)
			return false;
		const auto name_len = read_le16(header.data() + 28);
		const auto extra_len = read_le16(header.data() + 30);
		const auto comment_len = read_le16(header.data() + 32);
		std::vector<std::uint8_t> record(header.begin(), header.end());
		const auto variable_size = std::size_t(name_len) + extra_len + comment_len;
		const auto old_size = record.size();
		record.resize(old_size + variable_size);
		input.read(reinterpret_cast<char *>(record.data() + old_size),
				static_cast<std::streamsize>(variable_size));
		if (!input)
			return false;
		std::uint64_t local_offset = read_le32(record.data() + 42);
		if (local_offset == UINT32_MAX) {
			const auto zip64_offset = zip64_local_offset(record);
			if (!zip64_offset)
				return false;
			local_offset = *zip64_offset;
		}
		if (local_offset >= central_offset)
			return false;
		archive->entries.push_back(
				{std::string(
						 reinterpret_cast<const char *>(record.data() + 46), name_len),
						std::move(record), local_offset, 0});
	}
	const auto directory_end = central_offset + central_size;
	if (input.tellg() < 0 || static_cast<std::uint64_t>(input.tellg()) != directory_end)
		return false;
	std::vector<std::size_t> physical_order(archive->entries.size());
	for (std::size_t i = 0; i < physical_order.size(); ++i)
		physical_order[i] = i;
	std::sort(physical_order.begin(), physical_order.end(),
			[&](std::size_t a, std::size_t b) {
				return archive->entries[a].local_offset <
					   archive->entries[b].local_offset;
			});
	for (std::size_t i = 0; i < physical_order.size(); ++i) {
		auto &entry = archive->entries[physical_order[i]];
		const auto next_offset =
				i + 1 < physical_order.size()
						? archive->entries[physical_order[i + 1]].local_offset
						: central_offset;
		if (next_offset <= entry.local_offset)
			return false;
		entry.range_end = next_offset;
	}
	return true;
}

void write_le16(std::ostream &out, std::uint16_t value)
{
	const std::array<char, 2> bytes{
			static_cast<char>(value), static_cast<char>(value >> 8)};
	out.write(bytes.data(), bytes.size());
}

void write_le32(std::ostream &out, std::uint32_t value)
{
	const std::array<char, 4> bytes{static_cast<char>(value),
			static_cast<char>(value >> 8), static_cast<char>(value >> 16),
			static_cast<char>(value >> 24)};
	out.write(bytes.data(), bytes.size());
}

void write_le64(std::ostream &out, std::uint64_t value)
{
	write_le32(out, static_cast<std::uint32_t>(value));
	write_le32(out, static_cast<std::uint32_t>(value >> 32));
}

std::uint32_t zip_crc(const std::uint8_t *data, std::size_t size)
{
	uLong value = crc32(0L, Z_NULL, 0);
	while (size) {
		const auto chunk = static_cast<uInt>(std::min<std::size_t>(size, UINT_MAX));
		value = crc32(value, data, chunk);
		data += chunk;
		size -= chunk;
	}
	return static_cast<std::uint32_t>(value);
}

std::vector<std::uint8_t> encode_rgb_png(
		const arnis::world_editor::WorldEditor::FacadePanel &panel)
{
	png_image image{};
	image.version = PNG_IMAGE_VERSION;
	image.width = panel.width;
	image.height = panel.height;
	image.format = PNG_FORMAT_RGB;
	png_alloc_size_t size = 0;
	if (!png_image_write_to_memory(
				&image, nullptr, &size, 0, panel.pixels.data(), 0, nullptr))
		return {};
	std::vector<std::uint8_t> png(size);
	if (!png_image_write_to_memory(
				&image, png.data(), &size, 0, panel.pixels.data(), 0, nullptr))
		return {};
	png.resize(size);
	return png;
}

bool is_arnis_resource_pack(const std::filesystem::path &path)
{
	ExistingZipArchive archive;
	return read_zip_archive(path, &archive) &&
		   std::any_of(archive.entries.begin(), archive.entries.end(),
				   [](const ExistingZipEntry &entry) {
					   return entry.name == PACK_MARKER;
				   });
}

} // namespace

std::optional<std::pair<double, double>> outward_normal(
		std::pair<int, int> d, int sx, int sz)
{
	double x = d.first, z = d.second, l = std::hypot(x, z), side = z * sx - x * sz;
	if (l > 1e-9 && side > 0)
		return {{z / l, -x / l}};
	if (l > 1e-9 && side < 0)
		return {{-z / l, x / l}};
	double sl = std::hypot(sx, sz);
	return sl < 1e-9 ? std::nullopt
					 : std::optional<std::pair<double, double>>{{sx / sl, sz / sl}};
}
std::pair<double, double> right_of(std::pair<double, double> n)
{
	return {n.second, -n.first};
}
double yaw_deg(std::pair<double, double> n)
{
	return std::atan2(-n.first, n.second) * 180.0 / M_PI;
}
std::array<float, 4> left_rotation(std::pair<double, double> n)
{
	double h = std::atan2(n.first, n.second) / 2;
	return {0, float(std::sin(h)), 0, float(std::cos(h))};
}
std::pair<double, double> facing_of(std::array<float, 4> q)
{
	double y = q[1], w = q[3];
	return {2 * y * w, w * w - y * y};
}
bool flip_crop(std::pair<int, int> d, std::pair<double, double> n)
{
	auto r = right_of(n);
	return d.first * r.first + d.second * r.second < 0;
}
double cell_step(std::pair<int, int> d)
{
	int m = std::max(std::abs(d.first), std::abs(d.second));
	return m ? std::hypot(d.first, d.second) / m : 1.;
}
std::int8_t facing_index(std::pair<double, double> n)
{
	if (std::abs(n.first) >= std::abs(n.second))
		return n.first >= 0 ? 0 : 2;
	return n.second >= 0 ? 3 : 1;
}
Quad quad_for(const std::vector<std::pair<int, int>> &cells, std::pair<double, double> n,
		double step, int base, int h)
{
	auto r = right_of(n);
	double lo = 1e300, hi = -1e300, tm = -1e300;
	for (auto [x, z] : cells) {
		double px = x + .5, pz = z + .5, s = px * r.first + pz * r.second;
		lo = std::min(lo, s);
		hi = std::max(hi, s);
		tm = std::max(tm, px * n.first + pz * n.second);
	}
	double sc = (lo + hi) / 2,
		   plane = tm + .5 * (std::abs(n.first) + std::abs(n.second)) + PUSH_OUT;
	return {sc * r.first + plane * n.first, double(base) + h / 2.,
			sc * r.second + plane * n.second, (hi - lo) + step, double(h),
			left_rotation(n)};
}

std::vector<std::pair<int, int>> cut(int length, int maximum)
{
	std::vector<std::pair<int, int>> out;
	if (length <= 0 || maximum <= 0)
		return out;
	for (int start = 0; start < length; start += maximum)
		out.emplace_back(start, std::min(length, start + maximum));
	return out;
}

std::vector<Piece> pieces(const std::vector<std::pair<int, int>> &cells,
		std::pair<double, double> normal, double step, int base_y, int total_height)
{
	std::vector<Piece> out;
	if (cells.empty() || total_height <= 0 || step <= 0)
		return out;
	const int max_cells = std::max(1, static_cast<int>(MAX_PANEL / step));
	for (const auto &[p0, p1] : cut(static_cast<int>(cells.size()), max_cells))
		for (const auto &[b0, b1] : cut(total_height, MAX_PANEL)) {
			std::vector<std::pair<int, int>> footprint(
					cells.begin() + p0, cells.begin() + p1);
			out.push_back({p0, p1, b0, b1,
					quad_for(footprint, normal, step, base_y + b0, b1 - b0)});
		}
	return out;
}

namespace
{
std::int64_t name_seed(const std::string &name)
{
	std::uint64_t hash = 0x0102'0304'0506'0708ULL;
	for (const unsigned char byte : name) {
		hash ^= byte;
		hash *= 0x0000'0100'0000'01b3ULL;
	}
	return std::bit_cast<std::int64_t>(hash);
}

nlohmann::json display_nbt(const std::string &name, const Quad &quad)
{
	const std::string model = "arnis:" + name;
	return {{"item", {{"id", "minecraft:stone"}, {"count", 1},
							 {"components", {{"minecraft:item_model", model}}}}},
			{"item_display", "fixed"},
			{"transformation",
					{{"left_rotation",
							 {quad.rot[0], quad.rot[1], quad.rot[2], quad.rot[3]}},
							{"right_rotation", {0.0f, 0.0f, 0.0f, 1.0f}},
							{"translation", {0.0f, 0.0f, 0.0f}},
							{"scale", {static_cast<float>(quad.w),
											  static_cast<float>(quad.h), 1.0f}}}},
			{"billboard", "fixed"}, {"view_range", 16.0f}, {"width", 0.0f},
			{"height", 0.0f}};
}

std::uint32_t tex_side(double blocks, std::uint32_t px)
{
	return std::max<std::uint32_t>(
				   1, static_cast<std::uint32_t>(std::llround(blocks * px / 16.0))) *
		   16;
}
}
Sprite Sprite::of(double width, double height, std::uint32_t px)
{
	Sprite out{tex_side(width, px), tex_side(height, px), 0, 0};
	const double exact_w = std::max(1.0, std::round(width * px));
	const double exact_h = std::max(1.0, std::round(height * px));
	const double fit =
			std::min({double(out.side_w) / exact_w, double(out.side_h) / exact_h, 1.0});
	out.used_w = std::clamp<std::uint32_t>(std::llround(exact_w * fit), 1, out.side_w);
	out.used_h = std::clamp<std::uint32_t>(std::llround(exact_h * fit), 1, out.side_h);
	return out;
}
std::pair<double, double> Sprite::uv_extent() const
{
	return {16.0 * used_w / side_w, 16.0 * used_h / side_h};
}

std::vector<std::uint8_t> lay_out_rgb(const std::vector<std::uint8_t> &rgb,
		std::uint32_t input_width, std::uint32_t input_height, Sprite sprite)
{
	if (input_width == 0 || input_height == 0 ||
			rgb.size() != std::size_t(input_width) * input_height * 3 ||
			sprite.side_w == 0 || sprite.side_h == 0 || sprite.used_w == 0 ||
			sprite.used_h == 0)
		return {};
	std::vector<std::uint8_t> out(std::size_t(sprite.side_w) * sprite.side_h * 3);
	for (std::uint32_t y = 0; y < sprite.used_h; ++y) {
		const auto sy = std::min(input_height - 1,
				static_cast<std::uint32_t>((std::uint64_t(y) * input_height) /
										   std::max<std::uint32_t>(1, sprite.used_h)));
		for (std::uint32_t x = 0; x < sprite.used_w; ++x) {
			const auto sx = std::min(
					input_width - 1, static_cast<std::uint32_t>(
											 (std::uint64_t(x) * input_width) /
											 std::max<std::uint32_t>(1, sprite.used_w)));
			std::copy_n(rgb.begin() + (std::size_t(sy) * input_width + sx) * 3, 3,
					out.begin() + (std::size_t(y) * sprite.side_w + x) * 3);
		}
		for (std::uint32_t x = sprite.used_w; x < sprite.side_w; ++x)
			std::copy_n(out.begin() +
								(std::size_t(y) * sprite.side_w + sprite.used_w - 1) * 3,
					3, out.begin() + (std::size_t(y) * sprite.side_w + x) * 3);
	}
	for (std::uint32_t y = sprite.used_h; y < sprite.side_h; ++y)
		std::copy_n(out.begin() + (std::size_t(sprite.used_h - 1) * sprite.side_w) * 3,
				sprite.side_w * 3, out.begin() + (std::size_t(y) * sprite.side_w) * 3);
	return out;
}

std::string item_definition_json(const std::string &name)
{
	nlohmann::json model = nlohmann::json::object();
	model["type"] = "minecraft:model";
	model["model"] = "arnis:item/" + name;
	nlohmann::json item = nlohmann::json::object();
	item["model"] = std::move(model);
	return item.dump();
}
std::string model_json_for(const std::string &texture_name, Sprite sprite)
{
	const auto [u, v] = sprite.uv_extent();
	const auto texture = "arnis:block/" + texture_name;
	const nlohmann::json face{{"uv", {0.0, 0.0, u, v}}, {"texture", "#0"}};
	nlohmann::json model = nlohmann::json::object();
	model["textures"] = {{"0", texture}, {"particle", texture}};
	nlohmann::json element = nlohmann::json::object();
	element["from"] = {0.0, 0.0, 7.9};
	element["to"] = {16.0, 16.0, 8.1};
	element["faces"] = {{"north", face}, {"south", face}};
	model["elements"] = nlohmann::json::array({std::move(element)});
	model["display"]["fixed"] = {
			{"rotation", {0, 0, 0}}, {"translation", {0, 0, 0}}, {"scale", {1, 1, 1}}};
	return model.dump();
}

bool place_item_display(arnis::world_editor::WorldEditor &editor, const std::string &name,
		const Quad &quad, std::int8_t facing, std::vector<std::uint8_t> &&pixels,
		std::uint32_t pixel_width, std::uint32_t pixel_height)
{
	if (name.empty() || pixel_width == 0 || pixel_height == 0 ||
			pixels.size() != std::size_t(pixel_width) * pixel_height * 3)
		return false;
	const auto x = static_cast<int>(std::llround(quad.cx));
	const auto y = static_cast<int>(std::llround(quad.cy));
	const auto z = static_cast<int>(std::llround(quad.cz));
	if (!editor.add_item_display(
				quad.cx, quad.cy, quad.cz, name_seed(name), display_nbt(name, quad)))
		return false;
	editor.record_facade_panel(
			x, y, z, facing, std::move(pixels), pixel_width, pixel_height, name);
	return true;
}

bool Registry::collect(const std::string &name,
		const std::vector<std::pair<int, int>> &cells, std::pair<double, double> normal,
		double step, int base_y, int total_height, const std::vector<std::uint8_t> &rgb,
		std::uint32_t rgb_width, std::uint32_t rgb_height, std::uint32_t pixels_per_block,
		const std::function<bool(const Quad &, const Panel &)> &sink)
{
	if (!enabled_ || cells.empty() || name.empty())
		return false;
	++stats_.candidates;
	if (!names_.insert(name).second) {
		++stats_.dropped;
		return false;
	}
	const auto all = pieces(cells, normal, step, base_y, total_height);
	if (all.empty()) {
		++stats_.dropped;
		return false;
	}
	bool placed = false;
	for (const auto &piece : all) {
		const auto sprite = Sprite::of(piece.quad.w, piece.quad.h, pixels_per_block);
		Panel panel{name, piece.quad.w, piece.quad.h,
				lay_out_rgb(rgb, rgb_width, rgb_height, sprite), sprite.side_w,
				sprite.side_h};
		if (panel.pixels.empty() || !sink || !sink(piece.quad, panel))
			continue;
		placed = true;
		++stats_.displays;
	}
	if (placed)
		++stats_.placed;
	else
		++stats_.dropped;
	return placed;
}

bool Registry::collect_to_editor(arnis::world_editor::WorldEditor &editor,
		const std::string &name, const std::vector<std::pair<int, int>> &cells,
		std::pair<double, double> normal, double step, int base_y, int total_height,
		const std::vector<std::uint8_t> &rgb, std::uint32_t rgb_width,
		std::uint32_t rgb_height, std::uint32_t pixels_per_block)
{
	return collect(name, cells, normal, step, base_y, total_height, rgb, rgb_width,
			rgb_height, pixels_per_block, [&](const Quad &quad, const Panel &panel) {
				const auto x = static_cast<int>(std::llround(quad.cx));
				const auto y = static_cast<int>(std::llround(quad.cy));
				const auto z = static_cast<int>(std::llround(quad.cz));
				const auto facing = facing_index(normal);
				if (editor.has_entity_sink() &&
						editor.add_item_display(quad.cx, quad.cy, quad.cz,
								name_seed(name), display_nbt(name, quad))) {
					editor.record_facade_panel(x, y, z, facing, std::move(panel.pixels),
							panel.pixel_width, panel.pixel_height, name);
					return true;
				}
				return editor.place_facade_panel(x, y, z, facing, std::move(panel.pixels),
						panel.pixel_width, panel.pixel_height);
			});
}

bool write_world_resource_pack(arnis::world_editor::WorldEditor &editor,
		const std::filesystem::path &world, bool include_banner, std::string *error)
{
	if (error)
		error->clear();
	auto panels = editor.take_facade_panels();
	panels.erase(std::remove_if(panels.begin(), panels.end(),
						 [](const auto &panel) {
							 return panel.item_name.empty() || !panel.width ||
									!panel.height ||
									panel.pixels.size() !=
											std::size_t(panel.width) * panel.height * 3;
						 }),
			panels.end());
	const auto panel_count = panels.size();
	const auto pack_path = world / "resources.zip";
	if (panels.empty() && !include_banner)
		return true;
	std::error_code existing_error;
	const bool previous_exists =
			std::filesystem::is_regular_file(pack_path, existing_error);
	ExistingZipArchive previous;
	const bool previous_is_arnis =
			previous_exists && read_zip_archive(pack_path, &previous) &&
			std::any_of(previous.entries.begin(), previous.entries.end(),
					[](const ExistingZipEntry &entry) {
						return entry.name == PACK_MARKER;
					});
	if (panels.empty() && include_banner && previous_exists && !previous_is_arnis)
		return true; // Like Rust, leave a user's resource pack untouched for banner-only runs.
	const auto staged_path = world / "resources.zip.tmp";
	auto fail = [&](const std::string &message) {
		if (error)
			*error = message;
		std::error_code ignored;
		std::filesystem::remove(staged_path, ignored);
		return false;
	};
	std::ofstream output(staged_path, std::ios::binary | std::ios::trunc);
	if (!output)
		return fail("create " + staged_path.string());
	std::vector<ZipEntry> entries;
	auto put = [&](std::string name, const std::uint8_t *bytes, std::size_t size) {
		const auto position = output.tellp();
		if (position < 0 || size > UINT32_MAX || name.size() > UINT16_MAX)
			return false;
		const auto offset = static_cast<std::uint64_t>(position);
		const auto crc = zip_crc(bytes, size);
		write_le32(output, 0x04034b50);
		write_le16(output, 20);
		write_le16(output, 0);
		write_le16(output, 0); // Stored; PNG data is already compressed.
		write_le16(output, 0);
		write_le16(output, 33); // DOS date 1980-01-01.
		write_le32(output, crc);
		write_le32(output, static_cast<std::uint32_t>(size));
		write_le32(output, static_cast<std::uint32_t>(size));
		write_le16(output, static_cast<std::uint16_t>(name.size()));
		write_le16(output, 0);
		output.write(name.data(), static_cast<std::streamsize>(name.size()));
		if (size)
			output.write(reinterpret_cast<const char *>(bytes),
					static_cast<std::streamsize>(size));
		if (!output)
			return false;
		entries.push_back(
				{std::move(name), crc, static_cast<std::uint32_t>(size), offset, {}});
		return true;
	};
	auto put_text = [&](const std::string &name, const std::string &contents) {
		return put(name, reinterpret_cast<const std::uint8_t *>(contents.data()),
				contents.size());
	};
	const std::string metadata =
			nlohmann::json{{"pack", {{"pack_format", PACK_FORMAT},
											{"supported_formats", {PACK_FORMAT, 999}},
											{"min_format", PACK_FORMAT},
											{"max_format", {999, 0}},
											{"description", "Arnis facade panels"}}}}
					.dump();
	if (!put_text("pack.mcmeta", metadata) ||
			!put(".arnis_facade_pack",
					reinterpret_cast<const std::uint8_t *>("arnis facade panels\n"), 20))
		return fail("write resource pack metadata");
	std::unordered_map<std::uint64_t, std::string> textures;
	for (auto &panel : panels) {
		if (!put_text("assets/arnis/items/" + panel.item_name + ".json",
					item_definition_json(panel.item_name)))
			return fail("write resource pack item definition for " + panel.item_name);
		std::uint64_t hash = 14695981039346656037ull;
		auto hash_byte = [&](std::uint8_t byte) {
			hash ^= byte;
			hash *= 1099511628211ull;
		};
		for (int i = 0; i < 4; ++i) {
			hash_byte(static_cast<std::uint8_t>(panel.width >> (8 * i)));
			hash_byte(static_cast<std::uint8_t>(panel.height >> (8 * i)));
		}
		for (const auto byte : panel.pixels)
			hash_byte(byte);
		const auto found = textures.find(hash);
		const auto texture_name =
				found == textures.end() ? panel.item_name : found->second;
		Sprite sprite;
		sprite.side_w = panel.width;
		sprite.side_h = panel.height;
		sprite.used_w = panel.width;
		sprite.used_h = panel.height;
		if (!put_text("assets/arnis/models/item/" + panel.item_name + ".json",
					model_json_for(texture_name, sprite)))
			return fail("write resource pack model for " + panel.item_name);
		if (found == textures.end()) {
			auto png = encode_rgb_png(panel);
			if (png.empty())
				return fail("encode facade texture " + panel.item_name);
			if (!put("assets/arnis/textures/block/" + panel.item_name + ".png",
						png.data(), png.size()))
				return fail("write facade texture " + panel.item_name);
			textures.emplace(hash, panel.item_name);
		}
		std::vector<std::uint8_t>().swap(panel.pixels);
	}
	if (include_banner) {
		const auto banner_path = arnis::assets::path("minecraft/menu_banner.png");
		std::ifstream banner_file(banner_path, std::ios::binary);
		if (!banner_file)
			return fail("read bundled menu banner " + banner_path.string());
		const std::vector<std::uint8_t> banner(
				(std::istreambuf_iterator<char>(banner_file)), {});
		const std::string font =
				nlohmann::json{{"providers", {{{"type", "bitmap"},
													 {"file", "arnis:font/banner.png"},
													 {"height", 40}, {"ascent", 40},
													 {"chars", {"\xee\x80\x80"}}}}}}
						.dump();
		const std::string language =
				nlohmann::json{{"arnis.menu.banner", "\n\n\n\n\xee\x80\x80"}}.dump();
		if (banner.empty() || !put_text("assets/arnis/font/banner.json", font) ||
				!put("assets/arnis/textures/font/banner.png", banner.data(),
						banner.size()) ||
				!put_text("assets/arnis/lang/en_us.json", language))
			return fail("write pause-menu banner resources");
	}
	if (previous_is_arnis) {
		std::unordered_set<std::string> replaced;
		for (const auto &entry : entries)
			replaced.insert(entry.name);
		std::ifstream old_pack(pack_path, std::ios::binary);
		if (!old_pack)
			return fail("open existing Arnis resource pack " + pack_path.string());
		std::vector<std::size_t> physical_order(previous.entries.size());
		for (std::size_t i = 0; i < physical_order.size(); ++i)
			physical_order[i] = i;
		std::sort(physical_order.begin(), physical_order.end(),
				[&](std::size_t a, std::size_t b) {
					return previous.entries[a].local_offset <
						   previous.entries[b].local_offset;
				});
		std::array<char, 64 * 1024> buffer{};
		for (const auto index : physical_order) {
			const auto &old = previous.entries[index];
			if (replaced.contains(old.name))
				continue;
			const auto output_position = output.tellp();
			if (output_position < 0)
				return fail("query merged resource pack position");
			auto central_record = old.central_record;
			const auto new_offset = static_cast<std::uint64_t>(output_position);
			if (!set_central_local_offset(central_record, new_offset))
				return fail("update ZIP central-directory offset for " + old.name);
			old_pack.clear();
			old_pack.seekg(old.local_offset);
			std::uint64_t remaining = old.range_end - old.local_offset;
			while (remaining) {
				const auto chunk = static_cast<std::streamsize>(
						std::min<std::uint64_t>(remaining, buffer.size()));
				old_pack.read(buffer.data(), chunk);
				if (old_pack.gcount() != chunk)
					return fail("read compressed entry from existing resource pack");
				output.write(buffer.data(), chunk);
				if (!output)
					return fail("copy compressed entry into merged resource pack");
				remaining -= static_cast<std::uint64_t>(chunk);
			}
			entries.push_back({old.name, 0, 0, new_offset, std::move(central_record)});
		}
	}
	const auto central_offset = output.tellp();
	if (central_offset < 0)
		return fail("query resource pack central-directory position");
	for (const auto &entry : entries) {
		if (!entry.central_record.empty()) {
			output.write(reinterpret_cast<const char *>(entry.central_record.data()),
					static_cast<std::streamsize>(entry.central_record.size()));
			continue;
		}
		write_le32(output, 0x02014b50);
		const bool zip64_offset = entry.offset >= UINT32_MAX;
		write_le16(output, 20);
		write_le16(output, 20);
		write_le16(output, 0);
		write_le16(output, 0);
		write_le16(output, 0);
		write_le16(output, 33);
		write_le32(output, entry.crc);
		write_le32(output, entry.size);
		write_le32(output, entry.size);
		write_le16(output, static_cast<std::uint16_t>(entry.name.size()));
		write_le16(output, zip64_offset ? 12 : 0);
		write_le16(output, 0);
		write_le16(output, 0);
		write_le16(output, 0);
		write_le32(output, 0);
		write_le32(output,
				zip64_offset ? UINT32_MAX : static_cast<std::uint32_t>(entry.offset));
		output.write(entry.name.data(), static_cast<std::streamsize>(entry.name.size()));
		if (zip64_offset) {
			write_le16(output, 0x0001);
			write_le16(output, 8);
			write_le64(output, entry.offset);
		}
	}
	const auto end_offset = output.tellp();
	if (end_offset < central_offset)
		return fail("query resource pack central-directory size");
	const auto central_offset_u64 = static_cast<std::uint64_t>(central_offset);
	const auto central_size = static_cast<std::uint64_t>(end_offset - central_offset);
	const bool needs_zip64 = entries.size() >= UINT16_MAX || central_size >= UINT32_MAX ||
							 central_offset_u64 >= UINT32_MAX;
	if (needs_zip64) {
		const auto zip64_offset = output.tellp();
		if (zip64_offset < 0)
			return fail("query resource pack ZIP64 directory position");
		write_le32(output, 0x06064b50);
		write_le64(output, 44);
		write_le16(output, 45);
		write_le16(output, 45);
		write_le32(output, 0);
		write_le32(output, 0);
		write_le64(output, entries.size());
		write_le64(output, entries.size());
		write_le64(output, central_size);
		write_le64(output, central_offset_u64);
		write_le32(output, 0x07064b50);
		write_le32(output, 0);
		write_le64(output, static_cast<std::uint64_t>(zip64_offset));
		write_le32(output, 1);
	}
	write_le32(output, 0x06054b50);
	write_le16(output, 0);
	write_le16(output, 0);
	write_le16(output,
			needs_zip64 ? UINT16_MAX : static_cast<std::uint16_t>(entries.size()));
	write_le16(output,
			needs_zip64 ? UINT16_MAX : static_cast<std::uint16_t>(entries.size()));
	write_le32(
			output, needs_zip64 ? UINT32_MAX : static_cast<std::uint32_t>(central_size));
	write_le32(output,
			needs_zip64 ? UINT32_MAX : static_cast<std::uint32_t>(central_offset_u64));
	write_le16(output, 0);
	output.close();
	if (!output)
		return fail("write " + staged_path.string());

	std::error_code ec;
	if (std::filesystem::exists(pack_path, ec)) {
		const bool ours = is_arnis_resource_pack(pack_path);
		auto old_path = pack_path;
		if (ours) {
			old_path += ".replace";
			std::filesystem::remove(old_path, ec);
			ec.clear();
		} else {
			old_path = world / "resources.zip.bak";
			for (unsigned int n = 1; std::filesystem::exists(old_path, ec); ++n)
				old_path = world / ("resources.zip.bak" + std::to_string(n));
		}
		std::filesystem::rename(pack_path, old_path, ec);
		if (ec)
			return fail("move existing resource pack aside: " + ec.message());
		std::filesystem::rename(staged_path, pack_path, ec);
		if (ec) {
			std::error_code restore_error;
			std::filesystem::rename(old_path, pack_path, restore_error);
			return fail("install resource pack: " + ec.message());
		}
		if (ours)
			std::filesystem::remove(old_path, ec);
	} else {
		std::filesystem::rename(staged_path, pack_path, ec);
		if (ec)
			return fail("install resource pack: " + ec.message());
	}
	if (panel_count)
		std::cout << "Facade panels: wrote " << panel_count
				  << " item-display textures and models to resources.zip\n";
	return true;
}
}
