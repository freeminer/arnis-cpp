#include "servers_dat.h"

#include "../../arnis_block.h"
#include "../world_utils.h"

#include <algorithm>
#include <bit>
#include <climits>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

namespace arnis::live::servers_dat
{
namespace
{
constexpr std::size_t MAX_FILE_BYTES = 64 * 1024 * 1024;
constexpr std::size_t MAX_ITEMS = 1'000'000;
constexpr unsigned MAX_DEPTH = 64;

void set_error(std::string *out, std::string message)
{
	if (out)
		*out = std::move(message);
}

class Reader
{
public:
	explicit Reader(const std::vector<std::uint8_t> &bytes) : bytes_(bytes) {}

	bool root(NbtValue &value)
	{
		std::uint8_t type = 0;
		std::string ignored_name;
		return read_u8(type) && type == 10 && read_string(ignored_name) &&
			   payload(type, value, 0) && position_ == bytes_.size();
	}

private:
	bool take(std::size_t size, const std::uint8_t *&data)
	{
		if (size > bytes_.size() - position_)
			return false;
		data = bytes_.data() + position_;
		position_ += size;
		return true;
	}

	bool read_u8(std::uint8_t &value)
	{
		const std::uint8_t *data = nullptr;
		if (!take(1, data))
			return false;
		value = *data;
		return true;
	}

	bool read_u16(std::uint16_t &value)
	{
		const std::uint8_t *data = nullptr;
		if (!take(2, data))
			return false;
		value = (std::uint16_t(data[0]) << 8) | data[1];
		return true;
	}

	bool read_u32(std::uint32_t &value)
	{
		const std::uint8_t *data = nullptr;
		if (!take(4, data))
			return false;
		value = (std::uint32_t(data[0]) << 24) | (std::uint32_t(data[1]) << 16) |
				(std::uint32_t(data[2]) << 8) | data[3];
		return true;
	}

	bool read_u64(std::uint64_t &value)
	{
		const std::uint8_t *data = nullptr;
		if (!take(8, data))
			return false;
		value = 0;
		for (unsigned i = 0; i < 8; ++i)
			value = (value << 8) | data[i];
		return true;
	}

	bool read_string(std::string &value)
	{
		std::uint16_t size = 0;
		const std::uint8_t *data = nullptr;
		if (!read_u16(size) || !take(size, data))
			return false;
		if (size == 0) {
			value.clear();
			return true;
		}
		value.assign(reinterpret_cast<const char *>(data), size);
		return true;
	}

	bool payload(std::uint8_t type, NbtValue &out, unsigned depth)
	{
		if (depth > MAX_DEPTH)
			return false;
		std::uint8_t byte = 0;
		std::uint16_t short_value = 0;
		std::uint32_t int_value = 0;
		std::uint64_t long_value = 0;
		std::string text;
		switch (type) {
		case 1:
			if (!read_u8(byte))
				return false;
			out = NbtValue::byte(static_cast<std::int8_t>(byte));
			return true;
		case 2:
			if (!read_u16(short_value))
				return false;
			out = NbtValue::short_(static_cast<std::int16_t>(short_value));
			return true;
		case 3:
			if (!read_u32(int_value))
				return false;
			out = NbtValue::int_(static_cast<std::int32_t>(int_value));
			return true;
		case 4:
			if (!read_u64(long_value))
				return false;
			out = NbtValue::long_(static_cast<std::int64_t>(long_value));
			return true;
		case 5:
			if (!read_u32(int_value))
				return false;
			out = NbtValue::float_(std::bit_cast<float>(int_value));
			return true;
		case 6:
			if (!read_u64(long_value))
				return false;
			out = NbtValue::double_(std::bit_cast<double>(long_value));
			return true;
		case 7: {
			if (!read_u32(int_value) || int_value > INT32_MAX ||
					int_value > MAX_FILE_BYTES)
				return false;
			const std::uint8_t *data = nullptr;
			if (!take(int_value, data))
				return false;
			std::vector<std::int8_t> values(int_value);
			for (std::size_t i = 0; i < values.size(); ++i)
				values[i] = static_cast<std::int8_t>(data[i]);
			out = NbtValue::byte_array(std::move(values));
			return true;
		}
		case 8:
			if (!read_string(text))
				return false;
			out = NbtValue::string(std::move(text));
			return true;
		case 9: {
			std::uint8_t element_type = 0;
			if (!read_u8(element_type) || !read_u32(int_value) || int_value > INT32_MAX ||
					int_value > MAX_ITEMS || (element_type == 0 && int_value != 0) ||
					element_type > 12)
				return false;
			std::vector<NbtValue> values;
			values.reserve(int_value);
			for (std::uint32_t i = 0; i < int_value; ++i) {
				NbtValue item;
				if (!payload(element_type, item, depth + 1))
					return false;
				values.push_back(std::move(item));
			}
			out = NbtValue::list_value(std::move(values));
			return true;
		}
		case 10: {
			std::map<std::string, NbtValue> entries;
			for (std::size_t count = 0; count < MAX_ITEMS; ++count) {
				if (!read_u8(byte))
					return false;
				if (byte == 0) {
					out = NbtValue::compound_value(std::move(entries));
					return true;
				}
				if (byte > 12 || !read_string(text))
					return false;
				NbtValue item;
				if (!payload(byte, item, depth + 1))
					return false;
				entries.insert_or_assign(text, std::move(item));
			}
			return false;
		}
		case 11: {
			if (!read_u32(int_value) || int_value > INT32_MAX || int_value > MAX_ITEMS ||
					int_value > (bytes_.size() - position_) / 4)
				return false;
			std::vector<std::int32_t> values;
			values.reserve(int_value);
			for (std::uint32_t i = 0; i < int_value; ++i) {
				if (!read_u32(int_value))
					return false;
				values.push_back(static_cast<std::int32_t>(int_value));
			}
			out = NbtValue::int_array(std::move(values));
			return true;
		}
		case 12: {
			if (!read_u32(int_value) || int_value > INT32_MAX || int_value > MAX_ITEMS ||
					int_value > (bytes_.size() - position_) / 8)
				return false;
			std::vector<std::int64_t> values;
			values.reserve(int_value);
			for (std::uint32_t i = 0; i < int_value; ++i) {
				if (!read_u64(long_value))
					return false;
				values.push_back(static_cast<std::int64_t>(long_value));
			}
			out = NbtValue::long_array(std::move(values));
			return true;
		}
		default:
			return false;
		}
	}

	const std::vector<std::uint8_t> &bytes_;
	std::size_t position_ = 0;
};

class Writer
{
public:
	bool root(const NbtValue &value)
	{
		return value.tag == NbtValue::Tag::Compound && write_u8(10) && write_string("") &&
			   payload(value, 0) && bytes_.size() <= MAX_FILE_BYTES;
	}
	const std::vector<std::uint8_t> &bytes() const { return bytes_; }

private:
	bool write_u8(std::uint8_t value)
	{
		bytes_.push_back(value);
		return true;
	}
	bool write_u16(std::uint16_t value)
	{
		bytes_.push_back(static_cast<std::uint8_t>(value >> 8));
		bytes_.push_back(static_cast<std::uint8_t>(value));
		return true;
	}
	bool write_u32(std::uint32_t value)
	{
		for (int shift = 24; shift >= 0; shift -= 8)
			bytes_.push_back(static_cast<std::uint8_t>(value >> shift));
		return true;
	}
	bool write_u64(std::uint64_t value)
	{
		for (int shift = 56; shift >= 0; shift -= 8)
			bytes_.push_back(static_cast<std::uint8_t>(value >> shift));
		return true;
	}
	bool write_string(std::string_view value)
	{
		if (value.size() > UINT16_MAX)
			return false;
		write_u16(static_cast<std::uint16_t>(value.size()));
		bytes_.insert(bytes_.end(), value.begin(), value.end());
		return bytes_.size() <= MAX_FILE_BYTES;
	}
	static std::uint8_t type(const NbtValue &value)
	{
		switch (value.tag) {
		case NbtValue::Tag::Byte:
			return 1;
		case NbtValue::Tag::Short:
			return 2;
		case NbtValue::Tag::Int:
			return 3;
		case NbtValue::Tag::Long:
			return 4;
		case NbtValue::Tag::Float:
			return 5;
		case NbtValue::Tag::Double:
			return 6;
		case NbtValue::Tag::ByteArray:
			return 7;
		case NbtValue::Tag::String:
			return 8;
		case NbtValue::Tag::List:
			return 9;
		case NbtValue::Tag::Compound:
			return 10;
		case NbtValue::Tag::IntArray:
			return 11;
		case NbtValue::Tag::LongArray:
			return 12;
		case NbtValue::Tag::Null:
			return 0;
		}
		return 0;
	}
	bool write_tag(std::string_view name, const NbtValue &value, unsigned depth)
	{
		return write_u8(type(value)) && write_string(name) && payload(value, depth + 1);
	}
	bool payload(const NbtValue &value, unsigned depth)
	{
		if (depth > MAX_DEPTH)
			return false;
		switch (value.tag) {
		case NbtValue::Tag::Byte:
			return write_u8(static_cast<std::uint8_t>(value.integer));
		case NbtValue::Tag::Short:
			return write_u16(static_cast<std::uint16_t>(value.integer));
		case NbtValue::Tag::Int:
			return write_u32(static_cast<std::uint32_t>(value.integer));
		case NbtValue::Tag::Long:
			return write_u64(static_cast<std::uint64_t>(value.integer));
		case NbtValue::Tag::Float:
			return write_u32(
					std::bit_cast<std::uint32_t>(static_cast<float>(value.decimal)));
		case NbtValue::Tag::Double:
			return write_u64(std::bit_cast<std::uint64_t>(value.decimal));
		case NbtValue::Tag::ByteArray:
			if (value.bytes.size() > INT32_MAX)
				return false;
			write_u32(static_cast<std::uint32_t>(value.bytes.size()));
			for (const auto item : value.bytes)
				write_u8(static_cast<std::uint8_t>(item));
			return bytes_.size() <= MAX_FILE_BYTES;
		case NbtValue::Tag::String:
			return write_string(value.text);
		case NbtValue::Tag::List: {
			if (value.list.size() > INT32_MAX)
				return false;
			const auto element_type = value.list.empty() ? 0 : type(value.list.front());
			if (element_type == 0 || std::any_of(value.list.begin(), value.list.end(),
											 [&](const NbtValue &item) {
												 return type(item) != element_type;
											 }))
				return value.list.empty() && write_u8(0) && write_u32(0);
			write_u8(element_type);
			write_u32(static_cast<std::uint32_t>(value.list.size()));
			for (const auto &item : value.list)
				if (!payload(item, depth + 1))
					return false;
			return bytes_.size() <= MAX_FILE_BYTES;
		}
		case NbtValue::Tag::Compound:
			for (const auto &[name, item] : value.compound)
				if (type(item) == 0 || !write_tag(name, item, depth))
					return false;
			write_u8(0);
			return bytes_.size() <= MAX_FILE_BYTES;
		case NbtValue::Tag::IntArray:
			if (value.ints.size() > INT32_MAX)
				return false;
			write_u32(static_cast<std::uint32_t>(value.ints.size()));
			for (const auto item : value.ints)
				write_u32(static_cast<std::uint32_t>(item));
			return bytes_.size() <= MAX_FILE_BYTES;
		case NbtValue::Tag::LongArray:
			if (value.longs.size() > INT32_MAX)
				return false;
			write_u32(static_cast<std::uint32_t>(value.longs.size()));
			for (const auto item : value.longs)
				write_u64(static_cast<std::uint64_t>(item));
			return bytes_.size() <= MAX_FILE_BYTES;
		case NbtValue::Tag::Null:
			return false;
		}
		return false;
	}

	std::vector<std::uint8_t> bytes_;
};

std::optional<std::vector<std::uint8_t>> read_file(
		const std::filesystem::path &path, std::string *error)
{
	std::ifstream input(path, std::ios::binary | std::ios::ate);
	if (!input) {
		set_error(error, "could not read servers.dat");
		return std::nullopt;
	}
	const auto end = input.tellg();
	if (end < 0 || static_cast<std::uint64_t>(end) > MAX_FILE_BYTES) {
		set_error(error, "servers.dat exceeds the supported size");
		return std::nullopt;
	}
	std::vector<std::uint8_t> bytes(static_cast<std::size_t>(end));
	input.seekg(0);
	if (!bytes.empty() && !input.read(reinterpret_cast<char *>(bytes.data()),
								  static_cast<std::streamsize>(bytes.size()))) {
		set_error(error, "could not read servers.dat");
		return std::nullopt;
	}
	return bytes;
}

} // namespace

bool ensure_entry(const std::filesystem::path &game_dir, const std::string &address,
		std::string *error)
{
	const auto path = game_dir / "servers.dat";
	NbtValue root;
	std::error_code file_error;
	const bool exists = std::filesystem::exists(path, file_error);
	if (file_error) {
		set_error(error, file_error.message());
		return false;
	}
	if (exists) {
		auto bytes = read_file(path, error);
		if (!bytes)
			return false;
		Reader reader(*bytes);
		if (!reader.root(root)) {
			set_error(error, "servers.dat is unreadable");
			return false;
		}
	} else {
		root = NbtValue::compound_value({});
	}
	if (root.tag != NbtValue::Tag::Compound) {
		set_error(error, "servers.dat has an unexpected layout");
		return false;
	}
	auto &top = root.compound;
	auto servers = top.find("servers");
	if (servers == top.end()) {
		servers = top.emplace("servers", NbtValue::list_value({})).first;
	}
	if (servers->second.tag != NbtValue::Tag::List) {
		set_error(error, "servers.dat has an unexpected layout");
		return false;
	}
	auto &list = servers->second.list;
	for (auto &server : list) {
		if (server.tag != NbtValue::Tag::Compound)
			continue;
		auto &entry = server.compound;
		const auto name = entry.find("name");
		if (name == entry.end() || name->second.tag != NbtValue::Tag::String ||
				name->second.text != ENTRY_NAME)
			continue;
		const auto ip = entry.find("ip");
		if (ip != entry.end() && ip->second.tag == NbtValue::Tag::String &&
				ip->second.text == address)
			return true;
		entry["ip"] = NbtValue::string(address);
		goto save;
	}
	{
		std::map<std::string, NbtValue> entry;
		entry.emplace("name", NbtValue::string(ENTRY_NAME));
		entry.emplace("ip", NbtValue::string(address));
		entry.emplace("hidden", NbtValue::byte(0));
		list.insert(list.begin(), NbtValue::compound_value(std::move(entry)));
	}

save:
	Writer writer;
	if (!writer.root(root)) {
		set_error(error, "servers.dat could not be serialized");
		return false;
	}
	if (!world_utils::replace_file_atomically(path, writer.bytes())) {
		set_error(error, "could not write servers.dat");
		return false;
	}
	return true;
}
} // namespace arnis::live::servers_dat
