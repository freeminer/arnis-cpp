#include "world_utils.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <string_view>
#include <charconv>
#include <system_error>
#include <zlib.h>

namespace arnis::world_utils
{
namespace
{
constexpr std::size_t MAX_LEVEL_DAT_SIZE = 64 * 1024 * 1024;
constexpr std::size_t REGION_HEADER_SIZE = 8192;
constexpr std::size_t REGION_SECTOR_SIZE = 4096;
constexpr std::size_t MAX_CHUNK_NBT_SIZE = 64 * 1024 * 1024;

enum class NbtContext
{
	Unknown,
	Root,
	Data,
	Spawn,
	Player
};

bool read_be_u16(
		const std::vector<std::uint8_t> &bytes, std::size_t &pos, std::uint16_t &value)
{
	if (pos > bytes.size() || bytes.size() - pos < 2)
		return false;
	value = (std::uint16_t(bytes[pos]) << 8) | bytes[pos + 1];
	pos += 2;
	return true;
}

bool read_be_i32(
		const std::vector<std::uint8_t> &bytes, std::size_t &pos, std::int32_t &value)
{
	if (pos > bytes.size() || bytes.size() - pos < 4)
		return false;
	const auto raw = (std::uint32_t(bytes[pos]) << 24) |
					 (std::uint32_t(bytes[pos + 1]) << 16) |
					 (std::uint32_t(bytes[pos + 2]) << 8) | bytes[pos + 3];
	pos += 4;
	value = static_cast<std::int32_t>(raw);
	return true;
}

bool skip_nbt_payload(const std::vector<std::uint8_t> &bytes, std::size_t &pos,
		std::uint8_t type, unsigned depth)
{
	if (depth > 64 || pos > bytes.size())
		return false;
	std::uint16_t short_len = 0;
	std::int32_t len = 0;
	std::uint8_t item_type = 0;
	switch (type) {
	case 1:
		if (pos >= bytes.size())
			return false;
		++pos;
		return true;
	case 2:
		if (bytes.size() - pos < 2)
			return false;
		pos += 2;
		return true;
	case 3:
	case 5:
		if (bytes.size() - pos < 4)
			return false;
		pos += 4;
		return true;
	case 4:
	case 6:
		if (bytes.size() - pos < 8)
			return false;
		pos += 8;
		return true;
	case 7:
		if (!read_be_i32(bytes, pos, len) || len < 0 ||
				std::size_t(len) > bytes.size() - pos)
			return false;
		pos += std::size_t(len);
		return true;
	case 8:
		if (!read_be_u16(bytes, pos, short_len) || short_len > bytes.size() - pos)
			return false;
		pos += short_len;
		return true;
	case 9:
		if (pos >= bytes.size())
			return false;
		item_type = bytes[pos++];
		if (!read_be_i32(bytes, pos, len) || len < 0 || (item_type == 0 && len != 0))
			return false;
		for (std::int32_t i = 0; i < len; ++i)
			if (!skip_nbt_payload(bytes, pos, item_type, depth + 1))
				return false;
		return true;
	case 10:
		for (;;) {
			if (pos >= bytes.size())
				return false;
			const auto child_type = bytes[pos++];
			if (child_type == 0)
				return true;
			if (!read_be_u16(bytes, pos, short_len) || short_len > bytes.size() - pos)
				return false;
			pos += short_len;
			if (!skip_nbt_payload(bytes, pos, child_type, depth + 1))
				return false;
		}
	case 11:
		if (!read_be_i32(bytes, pos, len) || len < 0 ||
				std::size_t(len) > (bytes.size() - pos) / 4)
			return false;
		pos += std::size_t(len) * 4;
		return true;
	case 12:
		if (!read_be_i32(bytes, pos, len) || len < 0 ||
				std::size_t(len) > (bytes.size() - pos) / 8)
			return false;
		pos += std::size_t(len) * 8;
		return true;
	default:
		return false;
	}
}

std::optional<std::pair<std::int32_t, std::int32_t>> chunk_position(
		const std::vector<std::uint8_t> &bytes)
{
	std::size_t pos = 0;
	std::uint16_t root_name_len = 0;
	if (bytes.empty() || bytes[pos++] != 10 || !read_be_u16(bytes, pos, root_name_len) ||
			root_name_len > bytes.size() - pos)
		return std::nullopt;
	pos += root_name_len;
	std::optional<std::int32_t> x, z;
	for (;;) {
		if (pos >= bytes.size())
			return std::nullopt;
		const auto type = bytes[pos++];
		if (type == 0)
			break;
		std::uint16_t name_len = 0;
		if (!read_be_u16(bytes, pos, name_len) || name_len > bytes.size() - pos)
			return std::nullopt;
		const std::string_view name(
				reinterpret_cast<const char *>(bytes.data() + pos), name_len);
		pos += name_len;
		if (type == 3 && (name == "xPos" || name == "zPos")) {
			std::int32_t value = 0;
			if (!read_be_i32(bytes, pos, value))
				return std::nullopt;
			(name == "xPos" ? x : z) = value;
		} else if (!skip_nbt_payload(bytes, pos, type, 0)) {
			return std::nullopt;
		}
	}
	if (!x || !z)
		return std::nullopt;
	return std::pair{*x, *z};
}

std::optional<std::vector<std::uint8_t>> decompress_chunk(
		std::uint8_t compression, const std::uint8_t *data, std::size_t size)
{
	if (size == 0)
		return std::nullopt;
	if (compression == 3) {
		if (size > MAX_CHUNK_NBT_SIZE)
			return std::nullopt;
		return std::vector<std::uint8_t>(data, data + size);
	}
	if ((compression != 1 && compression != 2) || size > std::numeric_limits<uInt>::max())
		return std::nullopt;
	z_stream stream{};
	const int window_bits = compression == 1 ? 16 + MAX_WBITS : MAX_WBITS;
	if (inflateInit2(&stream, window_bits) != Z_OK)
		return std::nullopt;
	stream.next_in = const_cast<Bytef *>(data);
	stream.avail_in = static_cast<uInt>(size);
	std::vector<std::uint8_t> output;
	std::array<std::uint8_t, 32768> chunk{};
	int status = Z_OK;
	while (status == Z_OK && output.size() <= MAX_CHUNK_NBT_SIZE) {
		stream.next_out = chunk.data();
		stream.avail_out = static_cast<uInt>(chunk.size());
		status = inflate(&stream, Z_NO_FLUSH);
		const auto produced = chunk.size() - stream.avail_out;
		output.insert(output.end(), chunk.begin(), chunk.begin() + produced);
	}
	inflateEnd(&stream);
	if (status != Z_STREAM_END || output.size() > MAX_CHUNK_NBT_SIZE)
		return std::nullopt;
	return output;
}

std::optional<std::vector<std::uint8_t>> read_gzip_nbt(const std::filesystem::path &path)
{
	std::ifstream input(path, std::ios::binary);
	if (!input)
		return std::nullopt;
	std::vector<std::uint8_t> compressed((std::istreambuf_iterator<char>(input)), {});
	if (compressed.empty() || compressed.size() > std::numeric_limits<uInt>::max())
		return std::nullopt;
	z_stream stream{};
	if (inflateInit2(&stream, 16 + MAX_WBITS) != Z_OK)
		return std::nullopt;
	stream.next_in = compressed.data();
	stream.avail_in = static_cast<uInt>(compressed.size());
	std::vector<std::uint8_t> output;
	std::array<std::uint8_t, 32768> chunk{};
	int status = Z_OK;
	while (status == Z_OK && output.size() <= MAX_LEVEL_DAT_SIZE) {
		stream.next_out = chunk.data();
		stream.avail_out = static_cast<uInt>(chunk.size());
		status = inflate(&stream, Z_NO_FLUSH);
		const auto produced = chunk.size() - stream.avail_out;
		output.insert(output.end(), chunk.begin(), chunk.begin() + produced);
	}
	inflateEnd(&stream);
	if (status != Z_STREAM_END || output.size() > MAX_LEVEL_DAT_SIZE)
		return std::nullopt;
	return output;
}

std::optional<std::vector<std::uint8_t>> gzip_nbt(const std::vector<std::uint8_t> &input)
{
	if (input.size() > std::numeric_limits<uInt>::max())
		return std::nullopt;
	z_stream stream{};
	if (deflateInit2(&stream, Z_DEFAULT_COMPRESSION, Z_DEFLATED, 16 + MAX_WBITS, 8,
				Z_DEFAULT_STRATEGY) != Z_OK)
		return std::nullopt;
	stream.next_in = const_cast<Bytef *>(input.data());
	stream.avail_in = static_cast<uInt>(input.size());
	std::vector<std::uint8_t> output;
	std::array<std::uint8_t, 32768> chunk{};
	int status = Z_OK;
	while (status == Z_OK) {
		stream.next_out = chunk.data();
		stream.avail_out = static_cast<uInt>(chunk.size());
		status = deflate(&stream, Z_FINISH);
		const auto produced = chunk.size() - stream.avail_out;
		output.insert(output.end(), chunk.begin(), chunk.begin() + produced);
		if (output.size() > MAX_LEVEL_DAT_SIZE) {
			deflateEnd(&stream);
			return std::nullopt;
		}
	}
	deflateEnd(&stream);
	if (status != Z_STREAM_END)
		return std::nullopt;
	return output;
}

class NbtPatcher
{
	std::vector<std::uint8_t> &bytes_;
	std::size_t pos_{0};
	std::int32_t x_, y_, z_;
	bool player_file_;
	bool changed_{false};
	bool has_player_{false};
	std::uint8_t spawn_mask_{0};
	std::uint8_t legacy_spawn_mask_{0};
	bool has_spawn_compound_{false};
	bool spawn_pos_seen_{false};
	bool spawn_dimension_seen_{false};
	bool player_dimension_key_seen_{false};
	bool player_dimension_string_seen_{false};
	std::optional<std::array<std::uint32_t, 4>> player_uuid_;

	bool read_u8(std::uint8_t &value)
	{
		if (pos_ >= bytes_.size())
			return false;
		value = bytes_[pos_++];
		return true;
	}

	bool read_u16(std::uint16_t &value)
	{
		if (bytes_.size() - pos_ < 2)
			return false;
		value = (std::uint16_t(bytes_[pos_]) << 8) | bytes_[pos_ + 1];
		pos_ += 2;
		return true;
	}

	bool read_i32(std::int32_t &value)
	{
		if (bytes_.size() - pos_ < 4)
			return false;
		const auto raw = (std::uint32_t(bytes_[pos_]) << 24) |
						 (std::uint32_t(bytes_[pos_ + 1]) << 16) |
						 (std::uint32_t(bytes_[pos_ + 2]) << 8) | bytes_[pos_ + 3];
		pos_ += 4;
		value = static_cast<std::int32_t>(raw);
		return true;
	}

	void write_i32(std::size_t offset, std::int32_t value)
	{
		const auto raw = static_cast<std::uint32_t>(value);
		bytes_[offset] = std::uint8_t(raw >> 24);
		bytes_[offset + 1] = std::uint8_t(raw >> 16);
		bytes_[offset + 2] = std::uint8_t(raw >> 8);
		bytes_[offset + 3] = std::uint8_t(raw);
	}

	bool read_name(std::string &name)
	{
		std::uint16_t length = 0;
		if (!read_u16(length) || length > bytes_.size() - pos_)
			return false;
		name.assign(reinterpret_cast<const char *>(bytes_.data() + pos_), length);
		pos_ += length;
		return true;
	}

	void write_u16(std::size_t offset, std::uint16_t value)
	{
		bytes_[offset] = std::uint8_t(value >> 8);
		bytes_[offset + 1] = std::uint8_t(value);
	}

	static void append_u16(std::vector<std::uint8_t> &out, std::uint16_t value)
	{
		out.push_back(std::uint8_t(value >> 8));
		out.push_back(std::uint8_t(value));
	}

	static void append_i32(std::vector<std::uint8_t> &out, std::int32_t value)
	{
		const auto raw = static_cast<std::uint32_t>(value);
		out.push_back(std::uint8_t(raw >> 24));
		out.push_back(std::uint8_t(raw >> 16));
		out.push_back(std::uint8_t(raw >> 8));
		out.push_back(std::uint8_t(raw));
	}

	static bool append_name(
			std::vector<std::uint8_t> &out, std::uint8_t type, std::string_view name)
	{
		if (name.size() > std::numeric_limits<std::uint16_t>::max())
			return false;
		out.push_back(type);
		append_u16(out, static_cast<std::uint16_t>(name.size()));
		out.insert(out.end(), name.begin(), name.end());
		return true;
	}

	void insert_before_end(
			std::size_t end_offset, const std::vector<std::uint8_t> &encoded_tags)
	{
		bytes_.insert(
				bytes_.begin() + end_offset, encoded_tags.begin(), encoded_tags.end());
		pos_ += encoded_tags.size();
		changed_ = changed_ || !encoded_tags.empty();
	}

	bool append_missing_spawn_tags(NbtContext context, std::size_t end_offset)
	{
		std::vector<std::uint8_t> additions;
		if (context == NbtContext::Data && !has_spawn_compound_) {
			const std::array<std::pair<std::string_view, std::int32_t>, 3> coords{
					{{"SpawnX", x_}, {"SpawnY", y_}, {"SpawnZ", z_}}};
			for (std::size_t i = 0; i < coords.size(); ++i) {
				const auto bit = std::uint8_t(1u << i);
				if (legacy_spawn_mask_ & bit)
					continue;
				if (!append_name(additions, 3, coords[i].first))
					return false;
				append_i32(additions, coords[i].second);
				legacy_spawn_mask_ |= bit;
			}
			spawn_mask_ = 0x7;
		} else if (context == NbtContext::Spawn) {
			if (!spawn_pos_seen_) {
				if (!append_name(additions, 11, "pos"))
					return false;
				append_i32(additions, 3);
				append_i32(additions, x_);
				append_i32(additions, y_);
				append_i32(additions, z_);
				spawn_mask_ = 0x7;
			}
			if (!spawn_dimension_seen_) {
				constexpr std::string_view dimension = "minecraft:overworld";
				if (!append_name(additions, 8, "dimension"))
					return false;
				append_u16(additions, static_cast<std::uint16_t>(dimension.size()));
				additions.insert(additions.end(), dimension.begin(), dimension.end());
			}
		} else if (context == NbtContext::Player && player_dimension_key_seen_ &&
				   !player_dimension_string_seen_) {
			constexpr std::string_view dimension = "minecraft:overworld";
			if (!append_name(additions, 8, "Dimension"))
				return false;
			append_u16(additions, static_cast<std::uint16_t>(dimension.size()));
			additions.insert(additions.end(), dimension.begin(), dimension.end());
		}
		insert_before_end(end_offset, additions);
		return true;
	}

	bool patch_string(std::size_t length_offset, std::size_t value_offset,
			std::uint16_t length, std::string_view value)
	{
		if (value.size() > std::numeric_limits<std::uint16_t>::max())
			return false;
		const bool equal =
				length == value.size() &&
				std::equal(value.begin(), value.end(),
						reinterpret_cast<const char *>(bytes_.data() + value_offset));
		if (!equal) {
			bytes_.erase(bytes_.begin() + value_offset,
					bytes_.begin() + value_offset + length);
			bytes_.insert(bytes_.begin() + value_offset, value.begin(), value.end());
			write_u16(length_offset, static_cast<std::uint16_t>(value.size()));
			changed_ = true;
		}
		pos_ = value_offset + value.size();
		return true;
	}

	bool skip_payload(std::uint8_t type, unsigned depth)
	{
		if (depth > 64)
			return false;
		std::uint16_t short_length = 0;
		std::int32_t length = 0;
		std::uint8_t element_type = 0;
		switch (type) {
		case 1:
			return pos_ < bytes_.size() ? (++pos_, true) : false;
		case 2:
			return bytes_.size() - pos_ >= 2 ? (pos_ += 2, true) : false;
		case 3:
		case 5:
			return bytes_.size() - pos_ >= 4 ? (pos_ += 4, true) : false;
		case 4:
		case 6:
			return bytes_.size() - pos_ >= 8 ? (pos_ += 8, true) : false;
		case 7:
			return read_i32(length) && length >= 0 &&
								   std::size_t(length) <= bytes_.size() - pos_
						   ? (pos_ += std::size_t(length), true)
						   : false;
		case 8:
			return read_u16(short_length) && short_length <= bytes_.size() - pos_
						   ? (pos_ += short_length, true)
						   : false;
		case 9:
			if (!read_u8(element_type) || !read_i32(length) || length < 0 ||
					(element_type == 0 && length != 0))
				return false;
			for (std::int32_t i = 0; i < length; ++i)
				if (!skip_payload(element_type, depth + 1))
					return false;
			return true;
		case 10:
			return compound(NbtContext::Unknown, depth + 1);
		case 11:
			return read_i32(length) && length >= 0 &&
								   std::size_t(length) <= (bytes_.size() - pos_) / 4
						   ? (pos_ += std::size_t(length) * 4, true)
						   : false;
		case 12:
			return read_i32(length) && length >= 0 &&
								   std::size_t(length) <= (bytes_.size() - pos_) / 8
						   ? (pos_ += std::size_t(length) * 8, true)
						   : false;
		default:
			return false;
		}
	}

	bool patch_double_list()
	{
		std::uint8_t element_type = 0;
		std::int32_t count = 0;
		if (!read_u8(element_type) || !read_i32(count) || count < 0 ||
				(element_type == 0 && count != 0))
			return false;
		if (element_type != 6) {
			for (std::int32_t i = 0; i < count; ++i)
				if (!skip_payload(element_type, 1))
					return false;
			return true;
		}
		if (std::size_t(count) > (bytes_.size() - pos_) / 8)
			return false;
		const std::array<double, 3> values{double(x_), double(y_), double(z_)};
		for (std::int32_t i = 0; i < count; ++i) {
			if (i < 3) {
				std::uint64_t bits = 0;
				std::memcpy(&bits, &values[std::size_t(i)], sizeof(bits));
				for (int b = 0; b < 8; ++b)
					bytes_[pos_ + std::size_t(b)] = std::uint8_t(bits >> (56 - b * 8));
				changed_ = true;
			}
			pos_ += 8;
		}
		return true;
	}

	bool patch_int_array(bool spawn)
	{
		std::int32_t count = 0;
		if (!read_i32(count) || count < 0 ||
				std::size_t(count) > (bytes_.size() - pos_) / 4)
			return false;
		const auto data_offset = pos_;
		if (!spawn && count == 4) {
			std::array<std::uint32_t, 4> words{};
			for (std::size_t i = 0; i < words.size(); ++i) {
				const auto p = pos_ + i * 4;
				words[i] = (std::uint32_t(bytes_[p]) << 24) |
						   (std::uint32_t(bytes_[p + 1]) << 16) |
						   (std::uint32_t(bytes_[p + 2]) << 8) | bytes_[p + 3];
			}
			player_uuid_ = words;
		}
		if (spawn && count >= 3) {
			write_i32(data_offset, x_);
			write_i32(data_offset + 4, y_);
			write_i32(data_offset + 8, z_);
			spawn_mask_ = 0x7;
			spawn_pos_seen_ = true;
			changed_ = true;
		}
		pos_ += std::size_t(count) * 4;
		return true;
	}

	bool payload(
			std::uint8_t type, std::string_view name, NbtContext parent, unsigned depth)
	{
		if (depth > 64)
			return false;
		if (parent == NbtContext::Data && name == "Player")
			has_player_ = true;
		if (parent == NbtContext::Data && type == 3 &&
				(name == "SpawnX" || name == "SpawnY" || name == "SpawnZ")) {
			const auto offset = pos_;
			std::int32_t ignored = 0;
			if (!read_i32(ignored))
				return false;
			write_i32(offset, name == "SpawnX" ? x_ : name == "SpawnY" ? y_ : z_);
			const auto bit = name == "SpawnX" ? 0x1 : name == "SpawnY" ? 0x2 : 0x4;
			legacy_spawn_mask_ |= bit;
			spawn_mask_ |= bit;
			changed_ = true;
			return true;
		}
		if (parent == NbtContext::Data && name == "singleplayer_uuid" && type == 11)
			return patch_int_array(false);
		if (parent == NbtContext::Spawn && name == "pos" && type == 11)
			return patch_int_array(true);
		if ((parent == NbtContext::Spawn || parent == NbtContext::Player) &&
				name == "dimension" && type == 8) {
			std::uint16_t length = 0;
			if (!read_u16(length) || length > bytes_.size() - pos_)
				return false;
			const bool ok = patch_string(pos_ - 2, pos_, length, "minecraft:overworld");
			if (parent == NbtContext::Spawn && ok)
				spawn_dimension_seen_ = true;
			return ok;
		}
		if (parent == NbtContext::Player && name == "Pos" && type == 9)
			return patch_double_list();
		if (parent == NbtContext::Player && name == "Dimension") {
			player_dimension_key_seen_ = true;
			if (type == 8) {
				std::uint16_t length = 0;
				if (!read_u16(length) || length > bytes_.size() - pos_)
					return false;
				const auto length_offset = pos_ - 2;
				const auto value_offset = pos_;
				const bool ok = patch_string(
						length_offset, value_offset, length, "minecraft:overworld");
				player_dimension_string_seen_ = ok;
				return ok;
			}
			return skip_payload(type, depth + 1);
		}
		if (type == 10) {
			NbtContext child = NbtContext::Unknown;
			if (parent == NbtContext::Root && name == "Data")
				child = NbtContext::Data;
			else if (parent == NbtContext::Data && name == "spawn") {
				child = NbtContext::Spawn;
				has_spawn_compound_ = true;
			} else if ((parent == NbtContext::Data && name == "Player") ||
					   (player_file_ && parent == NbtContext::Root)) {
				child = NbtContext::Player;
			}
			return compound(child, depth + 1);
		}
		return skip_payload(type, depth + 1);
	}

	bool compound(NbtContext context, unsigned depth)
	{
		if (depth > 64)
			return false;
		for (;;) {
			const auto type_offset = pos_;
			std::uint8_t type = 0;
			if (!read_u8(type))
				return false;
			if (type == 0)
				return append_missing_spawn_tags(context, type_offset);
			std::string name;
			if (!read_name(name) || !payload(type, name, context, depth + 1))
				return false;
		}
	}

public:
	NbtPatcher(std::vector<std::uint8_t> &bytes, std::int32_t x, std::int32_t y,
			std::int32_t z, bool player_file) :
			bytes_(bytes), x_(x), y_(y), z_(z), player_file_(player_file)
	{
	}

	bool patch()
	{
		std::uint8_t root_type = 0;
		std::string root_name;
		if (!read_u8(root_type) || root_type != 10 || !read_name(root_name))
			return false;
		if (player_file_)
			has_player_ = true;
		return compound(player_file_ ? NbtContext::Player : NbtContext::Root, 0);
	}

	bool changed() const { return changed_; }
	bool has_player() const { return has_player_; }
	bool has_spawn() const { return spawn_mask_ == 0x7; }
	const auto &player_uuid() const { return player_uuid_; }
};

std::optional<std::filesystem::path> player_path(
		const std::filesystem::path &world, const std::array<std::uint32_t, 4> &uuid)
{
	char hex[33]{};
	std::snprintf(
			hex, sizeof(hex), "%08x%08x%08x%08x", uuid[0], uuid[1], uuid[2], uuid[3]);
	const std::string id(hex);
	const std::string filename = id.substr(0, 8) + "-" + id.substr(8, 4) + "-" +
								 id.substr(12, 4) + "-" + id.substr(16, 4) + "-" +
								 id.substr(20, 12) + ".dat";
	return world / "players" / "data" / filename;
}

bool patch_gzip_file(const std::filesystem::path &path, std::int32_t x, std::int32_t y,
		std::int32_t z, bool player_file,
		std::optional<std::array<std::uint32_t, 4>> *uuid = nullptr,
		bool *has_player = nullptr)
{
	auto nbt = read_gzip_nbt(path);
	if (!nbt)
		return false;
	NbtPatcher patcher(*nbt, x, y, z, player_file);
	if (!patcher.patch())
		return false;
	if (!player_file && !patcher.has_spawn())
		return false;
	if (uuid)
		*uuid = patcher.player_uuid();
	if (has_player)
		*has_player = patcher.has_player();
	if (!patcher.changed())
		return true;
	auto compressed = gzip_nbt(*nbt);
	return compressed && replace_file_atomically(path, *compressed);
}
} // namespace

std::optional<std::size_t> drop_misplaced_chunks(
		const std::filesystem::path &world, std::string *error)
{
	auto fail = [&](std::string message) -> std::optional<std::size_t> {
		if (error)
			*error = std::move(message);
		return std::nullopt;
	};
	if (error)
		error->clear();
	const auto region_dir = WorldLayout::of(world).overworld_dir(world) / "region";
	std::error_code ec;
	if (!std::filesystem::is_directory(region_dir, ec)) {
		if (ec)
			return fail("failed to inspect " + region_dir.string() + ": " + ec.message());
		return std::size_t{0};
	}
	std::size_t removed = 0;
	for (std::filesystem::directory_iterator it(region_dir, ec), end; !ec && it != end;
			it.increment(ec)) {
		const auto path = it->path();
		const auto filename = path.filename().string();
		if (!filename.starts_with("r.") || path.extension() != ".mca")
			continue;
		const auto stem = filename.substr(2, filename.size() - 6);
		const auto dot = stem.find('.');
		if (dot == std::string::npos || stem.find('.', dot + 1) != std::string::npos)
			continue;
		std::int32_t region_x = 0, region_z = 0;
		const auto [x_end, x_error] =
				std::from_chars(stem.data(), stem.data() + dot, region_x);
		const auto [z_end, z_error] = std::from_chars(
				stem.data() + dot + 1, stem.data() + stem.size(), region_z);
		if (x_error != std::errc{} || x_end != stem.data() + dot ||
				z_error != std::errc{} || z_end != stem.data() + stem.size())
			continue;
		const auto base_x = std::int64_t(region_x) * 32;
		const auto base_z = std::int64_t(region_z) * 32;
		if (base_x < std::numeric_limits<std::int32_t>::min() ||
				base_x + 31 > std::numeric_limits<std::int32_t>::max() ||
				base_z < std::numeric_limits<std::int32_t>::min() ||
				base_z + 31 > std::numeric_limits<std::int32_t>::max())
			continue;
		std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
		if (!file)
			return fail("failed to open " + path.string());
		file.seekg(0, std::ios::end);
		const auto end_position = file.tellg();
		if (end_position < 0)
			continue;
		const auto file_size =
				static_cast<std::uint64_t>(static_cast<std::streamoff>(end_position));
		if (file_size < REGION_HEADER_SIZE)
			continue;
		std::array<std::uint8_t, REGION_HEADER_SIZE> header{};
		file.seekg(0, std::ios::beg);
		if (!file.read(reinterpret_cast<char *>(header.data()), header.size()))
			continue;
		for (std::size_t cz = 0; cz < 32; ++cz)
			for (std::size_t cx = 0; cx < 32; ++cx) {
				const auto index = cx + cz * 32;
				const auto slot = index * 4;
				const std::uint32_t sector = (std::uint32_t(header[slot]) << 16) |
											 (std::uint32_t(header[slot + 1]) << 8) |
											 header[slot + 2];
				const auto sector_count = header[slot + 3];
				if (sector == 0 || sector_count == 0)
					continue;
				const auto chunk_offset = std::uint64_t(sector) * REGION_SECTOR_SIZE;
				if (chunk_offset + 5 > file_size)
					continue;
				file.seekg(static_cast<std::streamoff>(chunk_offset), std::ios::beg);
				std::array<std::uint8_t, 5> prefix{};
				if (!file.read(reinterpret_cast<char *>(prefix.data()), prefix.size())) {
					file.clear();
					continue;
				}
				const std::uint32_t length = (std::uint32_t(prefix[0]) << 24) |
											 (std::uint32_t(prefix[1]) << 16) |
											 (std::uint32_t(prefix[2]) << 8) | prefix[3];
				const auto allocation = std::uint64_t(sector_count) * REGION_SECTOR_SIZE;
				if (length < 1 || std::uint64_t(length) + 4 > allocation ||
						chunk_offset + std::uint64_t(length) + 4 > file_size ||
						(prefix[4] & 0x80))
					continue;
				std::vector<std::uint8_t> compressed(length - 1);
				if (!compressed.empty() &&
						!file.read(reinterpret_cast<char *>(compressed.data()),
								compressed.size())) {
					file.clear();
					continue;
				}
				auto nbt =
						decompress_chunk(prefix[4], compressed.data(), compressed.size());
				if (!nbt)
					continue;
				const auto position = chunk_position(*nbt);
				if (!position ||
						position->first == static_cast<std::int32_t>(base_x + cx) &&
								position->second ==
										static_cast<std::int32_t>(base_z + cz))
					continue;
				const std::array<char, 4> empty_slot{};
				file.seekp(static_cast<std::streamoff>(slot), std::ios::beg);
				if (!file.write(empty_slot.data(), empty_slot.size()) || !file.flush())
					return fail(
							"failed to clear misplaced chunk slot in " + path.string());
				++removed;
			}
		if (file.bad())
			return fail("I/O error while repairing " + path.string());
	}
	if (ec)
		return fail("failed to list " + region_dir.string() + ": " + ec.message());
	return removed;
}

bool set_spawn_in_level_dat(const std::filesystem::path &world, std::int32_t x,
		std::int32_t y, std::int32_t z)
{
	std::optional<std::array<std::uint32_t, 4>> uuid;
	bool has_player = false;
	if (!patch_gzip_file(world / "level.dat", x, y, z, false, &uuid, &has_player))
		return false;
	if (!has_player && uuid) {
		const auto path = player_path(world, *uuid);
		std::error_code ec;
		if (std::filesystem::is_regular_file(*path, ec) &&
				!patch_gzip_file(*path, x, y, z, true))
			return false;
	}
	return true;
}
} // namespace arnis::world_utils
