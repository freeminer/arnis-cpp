#include "region.h"

#include <zlib.h>

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <cerrno>
#include <chrono>
#include <climits>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>

#if defined(_WIN32)
#include <windows.h>
#else
#include <cerrno>
#include <unistd.h>
#endif

namespace arnis::live::region
{
namespace
{
constexpr std::size_t SECTOR = 4096;
constexpr std::uint8_t EXTERNAL = 0x80;
constexpr std::size_t MAX_INLINE_SECTORS = 255;
constexpr std::size_t MAX_NBT_BYTES = 64 * 1024 * 1024;

std::uint32_t be32(const std::uint8_t *p)
{
	return (static_cast<std::uint32_t>(p[0]) << 24) |
		   (static_cast<std::uint32_t>(p[1]) << 16) |
		   (static_cast<std::uint32_t>(p[2]) << 8) | p[3];
}

void put_be32(std::uint8_t *p, std::uint32_t v)
{
	p[0] = static_cast<std::uint8_t>(v >> 24);
	p[1] = static_cast<std::uint8_t>(v >> 16);
	p[2] = static_cast<std::uint8_t>(v >> 8);
	p[3] = static_cast<std::uint8_t>(v);
}

std::vector<std::uint8_t> read_file(
		const std::filesystem::path &path, std::error_code &ec)
{
	std::ifstream in(path, std::ios::binary | std::ios::ate);
	if (!in) {
		ec = std::error_code(errno ? errno : ENOENT, std::generic_category());
		return {};
	}
	const auto end = in.tellg();
	if (end < 0) {
		ec = std::make_error_code(std::errc::io_error);
		return {};
	}
	std::vector<std::uint8_t> data(static_cast<std::size_t>(end));
	in.seekg(0);
	if (!data.empty() && !in.read(reinterpret_cast<char *>(data.data()), data.size())) {
		ec = std::make_error_code(std::errc::io_error);
		return {};
	}
	ec.clear();
	return data;
}

bool write_file(const std::filesystem::path &path, const std::vector<std::uint8_t> &bytes,
		std::error_code &ec)
{
	std::ofstream out(path, std::ios::binary | std::ios::trunc);
	if (!out) {
		ec = std::error_code(errno ? errno : EIO, std::generic_category());
		return false;
	}
	if (!bytes.empty())
		out.write(reinterpret_cast<const char *>(bytes.data()),
				static_cast<std::streamsize>(bytes.size()));
	out.flush();
	if (!out) {
		ec = std::make_error_code(std::errc::io_error);
		return false;
	}
	out.close();
	if (!out) {
		ec = std::make_error_code(std::errc::io_error);
		return false;
	}
	ec.clear();
	return true;
}

std::size_t sectors_for(std::size_t payload_size)
{
	return (payload_size + 5 + SECTOR - 1) / SECTOR;
}

std::optional<std::vector<std::uint8_t>> decompress_chunk(
		std::uint8_t version, const std::vector<std::uint8_t> &input)
{
	if (version == 3)
		return input;
	if (version != 1 && version != 2)
		return std::nullopt;
	z_stream stream{};
	if (inflateInit2(&stream, version == 1 ? 15 + 16 : 15) != Z_OK)
		return std::nullopt;
	stream.next_in = const_cast<Bytef *>(input.data());
	stream.avail_in = static_cast<uInt>(std::min<std::size_t>(input.size(), UINT_MAX));
	std::vector<std::uint8_t> output;
	std::array<std::uint8_t, 64 * 1024> buffer{};
	int status = Z_OK;
	while (status == Z_OK && output.size() <= MAX_NBT_BYTES) {
		stream.next_out = buffer.data();
		stream.avail_out = static_cast<uInt>(buffer.size());
		status = inflate(&stream, Z_NO_FLUSH);
		const auto produced = buffer.size() - stream.avail_out;
		if (produced > MAX_NBT_BYTES - std::min(output.size(), MAX_NBT_BYTES)) {
			status = Z_MEM_ERROR;
			break;
		}
		output.insert(output.end(), buffer.begin(), buffer.begin() + produced);
	}
	inflateEnd(&stream);
	if (status != Z_STREAM_END || output.size() > MAX_NBT_BYTES)
		return std::nullopt;
	return output;
}

// Small selective NBT reader. It retains only strings, lists and compounds needed to
// distinguish generated terrain from a server's temporary all-air chunk.
struct Nbt
{
	std::uint8_t type = 0;
	std::string string;
	std::vector<Nbt> list;
	std::vector<std::pair<std::string, std::unique_ptr<Nbt>>> compound;
	const Nbt *find(std::string_view key) const
	{
		for (const auto &entry : compound)
			if (entry.first == key)
				return entry.second.get();
		return nullptr;
	}
};

class NbtReader
{
public:
	explicit NbtReader(const std::vector<std::uint8_t> &bytes) : bytes_(bytes) {}
	bool root_compound(Nbt &root)
	{
		if (read_u8() != 10)
			return false;
		(void)read_string(); // root name
		return compound(root, 0) && position_ == bytes_.size();
	}

private:
	std::uint8_t read_u8()
	{
		if (position_ >= bytes_.size()) {
			ok_ = false;
			return 0;
		}
		return bytes_[position_++];
	}
	std::uint16_t read_u16()
	{
		const auto a = read_u8(), b = read_u8();
		return static_cast<std::uint16_t>((a << 8) | b);
	}
	std::string read_string()
	{
		const auto size = read_u16();
		if (!ok_ || size > bytes_.size() - position_) {
			ok_ = false;
			return {};
		}
		std::string s(reinterpret_cast<const char *>(bytes_.data() + position_), size);
		position_ += size;
		return s;
	}
	bool skip(std::size_t size)
	{
		if (size > bytes_.size() - position_) {
			ok_ = false;
			return false;
		}
		position_ += size;
		return true;
	}
	bool payload(std::uint8_t type, Nbt &out, unsigned depth)
	{
		if (!ok_ || depth > 64)
			return ok_ = false;
		out.type = type;
		switch (type) {
		case 1:
			return skip(1);
		case 2:
			return skip(2);
		case 3:
		case 5:
			return skip(4);
		case 4:
		case 6:
			return skip(8);
		case 7:
		case 11:
		case 12: {
			const auto raw = read_u32();
			if (!ok_ || raw > 64u * 1024u * 1024u)
				return ok_ = false;
			const std::uint64_t bytes =
					type == 7 ? raw
							  : static_cast<std::uint64_t>(raw) * (type == 11 ? 4 : 8);
			if (bytes > std::numeric_limits<std::size_t>::max())
				return ok_ = false;
			return skip(static_cast<std::size_t>(bytes));
		}
		case 8:
			out.string = read_string();
			return ok_;
		case 9: {
			const auto child_type = read_u8();
			const auto count = read_u32();
			if (!ok_ || count > 1'000'000 || (count && child_type == 0))
				return ok_ = false;
			// Preserve list children because target block-state palettes are nested.
			if (child_type == 10 || child_type == 8 || child_type == 9) {
				out.list.reserve(count);
				for (std::uint32_t i = 0; i < count; ++i) {
					Nbt child;
					if (!payload(child_type, child, depth + 1))
						return false;
					out.list.push_back(std::move(child));
				}
			} else {
				for (std::uint32_t i = 0; i < count; ++i) {
					Nbt ignored;
					if (!payload(child_type, ignored, depth + 1))
						return false;
				}
			}
			return ok_;
		}
		case 10:
			return compound(out, depth + 1);
		default:
			return ok_ = false;
		}
	}
	std::uint32_t read_u32()
	{
		const auto a = read_u8(), b = read_u8(), c = read_u8(), d = read_u8();
		return (static_cast<std::uint32_t>(a) << 24) |
			   (static_cast<std::uint32_t>(b) << 16) |
			   (static_cast<std::uint32_t>(c) << 8) | d;
	}
	bool compound(Nbt &out, unsigned depth)
	{
		for (std::size_t count = 0; count <= 1'000'000; ++count) {
			const auto type = read_u8();
			if (!ok_)
				return false;
			if (type == 0)
				return true;
			const auto name = read_string();
			Nbt child;
			if (!payload(type, child, depth + 1))
				return false;
			// Duplicate names are legal in the wire format; the last value wins as in a map.
			auto existing = std::find_if(out.compound.begin(), out.compound.end(),
					[&](const auto &v) { return v.first == name; });
			if (existing == out.compound.end())
				out.compound.emplace_back(name, std::make_unique<Nbt>(std::move(child)));
			else
				existing->second = std::make_unique<Nbt>(std::move(child));
		}
		return ok_ = false;
	}
	const std::vector<std::uint8_t> &bytes_;
	std::size_t position_ = 0;
	bool ok_ = true;
};

bool all_air_palette(const Nbt &palette)
{
	for (const auto &entry : palette.list) {
		std::string_view name;
		if (entry.type == 8) {
			name = entry.string;
		} else if (entry.type == 10) {
			const auto *tag = entry.find("Name");
			if (!tag || tag->type != 8)
				continue;
			name = tag->string;
		} else {
			continue;
		}
		if (name != "minecraft:air" && name != "minecraft:cave_air" &&
				name != "minecraft:void_air")
			return false;
	}
	return true;
}
} // namespace

Bits Bits::all()
{
	Bits b;
	b.words.fill(UINT64_MAX);
	return b;
}

bool Bits::get(std::size_t index) const
{
	return (words.at(index / 64) >> (index % 64)) & 1u;
}

void Bits::set(std::size_t index)
{
	words.at(index / 64) |= std::uint64_t{1} << (index % 64);
}

std::uint32_t Bits::count() const
{
	std::uint32_t result = 0;
	for (const auto word : words)
		result += static_cast<std::uint32_t>(std::popcount(word));
	return result;
}

bool Bits::is_full() const
{
	return std::all_of(
			words.begin(), words.end(), [](auto word) { return word == UINT64_MAX; });
}

Bits Bits::united(const Bits &other) const
{
	Bits result = *this;
	for (std::size_t i = 0; i < words.size(); ++i)
		result.words[i] |= other.words[i];
	return result;
}

Slot Slot::inline_chunk(std::uint8_t compression, std::vector<std::uint8_t> payload)
{
	return {Kind::Inline, compression, std::move(payload)};
}

Slot Slot::external_chunk(std::uint8_t compression)
{
	return {Kind::External, compression, {}};
}

std::size_t slot_index(std::int32_t cx, std::int32_t cz)
{
	return static_cast<std::size_t>((cx & 31) + (cz & 31) * 32);
}

std::filesystem::path region_file(
		const std::filesystem::path &dir, std::int32_t rx, std::int32_t rz)
{
	return dir / ("r." + std::to_string(rx) + "." + std::to_string(rz) + ".mca");
}

std::filesystem::path mcc_file(
		const std::filesystem::path &dir, std::int32_t cx, std::int32_t cz)
{
	return dir / ("c." + std::to_string(cx) + "." + std::to_string(cz) + ".mcc");
}

std::optional<Bits> occupied(const std::filesystem::path &path, std::error_code &ec)
{
	const auto bytes = read_file(path, ec);
	if (ec) {
		if (ec == std::errc::no_such_file_or_directory) {
			ec.clear();
			return std::nullopt;
		}
		return std::nullopt;
	}
	Bits bits;
	for (std::size_t i = 0; i < SLOT_COUNT; ++i)
		if (bytes.size() >= i * 4 + 4 && be32(bytes.data() + i * 4) != 0)
			bits.set(i);
	return bits;
}

Bits external_slots(const std::filesystem::path &path, std::error_code &ec)
{
	const auto bytes = read_file(path, ec);
	Bits bits;
	if (ec)
		return bits;
	if (bytes.size() < SECTOR) {
		ec = std::make_error_code(std::errc::invalid_argument);
		return bits;
	}
	for (std::size_t i = 0; i < SLOT_COUNT; ++i) {
		const auto entry = be32(bytes.data() + i * 4);
		const auto sector = entry >> 8;
		if (sector < 2 || static_cast<std::uint64_t>(sector) * SECTOR + 5 > bytes.size())
			continue;
		if (bytes[static_cast<std::size_t>(sector) * SECTOR + 4] & EXTERNAL)
			bits.set(i);
	}
	return bits;
}

std::vector<Slot> read_slots(const std::filesystem::path &path, std::error_code &ec)
{
	const auto bytes = read_file(path, ec);
	if (ec)
		return {};
	std::vector<Slot> slots(SLOT_COUNT);
	if (bytes.size() < 2 * SECTOR)
		return slots;
	for (std::size_t i = 0; i < SLOT_COUNT; ++i) {
		const auto entry = be32(bytes.data() + i * 4);
		if (!entry)
			continue;
		const std::uint64_t start64 = static_cast<std::uint64_t>(entry >> 8) * SECTOR;
		if (start64 < 2 * SECTOR || start64 + 5 > bytes.size())
			continue;
		const auto start = static_cast<std::size_t>(start64);
		const auto length = be32(bytes.data() + start);
		const auto version = bytes[start + 4];
		if (version & EXTERNAL) {
			slots[i] = Slot::external_chunk(version & ~EXTERNAL);
			continue;
		}
		if (length < 1 || static_cast<std::uint64_t>(start) + 4 + length > bytes.size())
			continue;
		slots[i] = Slot::inline_chunk(
				version, std::vector<std::uint8_t>(bytes.begin() + start + 5,
								 bytes.begin() + start + 4 + length));
	}
	ec.clear();
	return slots;
}

bool write_slots(const std::filesystem::path &path, const std::vector<Slot> &slots,
		std::error_code &ec)
{
	if (slots.size() != SLOT_COUNT) {
		ec = std::make_error_code(std::errc::invalid_argument);
		return false;
	}
	std::array<std::uint32_t, SLOT_COUNT> offsets{};
	std::vector<std::uint8_t> body;
	std::size_t sector = 2;
	for (std::size_t i = 0; i < slots.size(); ++i) {
		const auto &slot = slots[i];
		if (slot.kind == Slot::Kind::Empty)
			continue;
		if (sector > 0x00ffffff) {
			ec = std::make_error_code(std::errc::file_too_large);
			return false;
		}
		const bool external = slot.kind == Slot::Kind::External;
		const auto sectors = sectors_for(external ? 0 : slot.data.size());
		if (sectors > MAX_INLINE_SECTORS || sectors == 0) {
			ec = std::make_error_code(std::errc::file_too_large);
			return false;
		}
		offsets[i] = (static_cast<std::uint32_t>(sector) << 8) |
					 (static_cast<std::uint32_t>(sectors) & 0xff);
		const auto old = body.size();
		const auto chunk_size = external ? 1 : slot.data.size() + 1;
		body.resize(old + 4 + chunk_size);
		put_be32(body.data() + old, static_cast<std::uint32_t>(chunk_size));
		body[old + 4] = slot.version | (external ? EXTERNAL : 0);
		if (!external && !slot.data.empty())
			std::copy(slot.data.begin(), slot.data.end(), body.begin() + old + 5);
		body.resize(old + sectors * SECTOR, 0);
		sector += sectors;
	}
	std::vector<std::uint8_t> output(2 * SECTOR, 0);
	for (std::size_t i = 0; i < SLOT_COUNT; ++i)
		put_be32(output.data() + i * 4, offsets[i]);
	const auto now =
			static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(
					std::chrono::system_clock::now().time_since_epoch())
											   .count());
	for (std::size_t i = 0; i < SLOT_COUNT; ++i)
		put_be32(output.data() + SECTOR + i * 4, offsets[i] ? now : 0);
	output.insert(output.end(), body.begin(), body.end());
	return replace_atomically(path, output, ec);
}

bool replace_atomically(const std::filesystem::path &path,
		const std::vector<std::uint8_t> &bytes, std::error_code &ec)
{
	const auto name =
			path.filename().empty() ? std::string("region") : path.filename().string();
	const auto tmp = path.parent_path() / ("." + name + ".arnis-tmp");
	if (!write_file(tmp, bytes, ec)) {
		std::filesystem::remove(tmp, ec);
		return false;
	}
	std::filesystem::rename(tmp, path, ec);
	if (ec) {
		std::error_code ignored;
		std::filesystem::remove(tmp, ignored);
		return false;
	}
	return true;
}

Bits add_stubs(const std::filesystem::path &path, std::error_code &ec)
{
	auto slots = read_slots(path, ec);
	if (ec)
		return {};
	Bits stubbed;
	for (std::size_t i = 0; i < slots.size(); ++i) {
		if (slots[i].kind == Slot::Kind::Empty) {
			slots[i] = Slot::external_chunk(2);
			stubbed.set(i);
		}
	}
	if (stubbed.count() && !write_slots(path, slots, ec))
		return {};
	return stubbed;
}

bool write_mcc(const std::filesystem::path &dir, std::int32_t cx, std::int32_t cz,
		const std::vector<std::uint8_t> &compressed, std::error_code &ec)
{
	return replace_atomically(mcc_file(dir, cx, cz), compressed, ec);
}

bool link_no_replace(const std::filesystem::path &src, const std::filesystem::path &dst,
		std::error_code &ec)
{
	std::filesystem::create_hard_link(src, dst, ec);
	if (!ec) {
		std::error_code ignored;
		std::filesystem::remove(src, ignored);
		return true;
	}
	if (ec == std::errc::file_exists)
		return false;
	ec.clear();
	if (std::filesystem::exists(dst, ec) || ec)
		return false;
	std::filesystem::rename(src, dst, ec);
	return !ec;
}

std::size_t normalize(const std::filesystem::path &path,
		const std::filesystem::path &region_dir, std::int32_t rx, std::int32_t rz,
		std::error_code &ec)
{
	auto slots = read_slots(path, ec);
	if (ec)
		return 0;
	std::size_t changed = 0;
	for (std::size_t i = 0; i < slots.size(); ++i) {
		auto &slot = slots[i];
		if (slot.kind != Slot::Kind::External)
			continue;
		const auto cx = rx * 32 + static_cast<std::int32_t>(i % 32);
		const auto cz = rz * 32 + static_cast<std::int32_t>(i / 32);
		ec.clear();
		auto payload = read_file(mcc_file(region_dir, cx, cz), ec);
		if (ec == std::errc::no_such_file_or_directory) {
			ec.clear();
			slot = Slot{};
			++changed;
		} else if (ec) {
			return changed;
		} else if (sectors_for(payload.size()) <= MAX_INLINE_SECTORS) {
			slot = Slot::inline_chunk(slot.version, std::move(payload));
			++changed;
		}
	}
	if (changed && !write_slots(path, slots, ec))
		return changed;
	if (changed) {
		for (std::size_t i = 0; i < slots.size(); ++i) {
			if (slots[i].kind == Slot::Kind::Inline) {
				const auto cx = rx * 32 + static_cast<std::int32_t>(i % 32);
				const auto cz = rz * 32 + static_cast<std::int32_t>(i / 32);
				std::error_code ignored;
				std::filesystem::remove(mcc_file(region_dir, cx, cz), ignored);
			}
		}
	}
	return changed;
}

std::size_t fill_from(const std::filesystem::path &live,
		const std::filesystem::path &staged, const std::filesystem::path &region_dir,
		std::int32_t rx, std::int32_t rz, std::error_code &ec)
{
	auto generated = read_slots(staged, ec);
	if (ec)
		return 0;
	if (!std::filesystem::exists(live, ec)) {
		if (ec)
			return 0;
		const auto count = std::count_if(generated.begin(), generated.end(),
				[](const Slot &s) { return s.kind != Slot::Kind::Empty; });
		std::filesystem::copy_file(
				staged, live, std::filesystem::copy_options::overwrite_existing, ec);
		return ec ? 0 : static_cast<std::size_t>(count);
	}
	auto slots = read_slots(live, ec);
	if (ec)
		return 0;
	std::size_t filled = 0;
	for (std::size_t i = 0; i < slots.size(); ++i) {
		if (generated[i].kind == Slot::Kind::Empty)
			continue;
		auto &current = slots[i];
		bool replace = current.kind == Slot::Kind::Empty;
		if (current.kind == Slot::Kind::Inline)
			replace = is_void_chunk(current.version, current.data);
		if (current.kind == Slot::Kind::External) {
			const auto cx = rx * 32 + static_cast<std::int32_t>(i % 32);
			const auto cz = rz * 32 + static_cast<std::int32_t>(i / 32);
			ec.clear();
			replace = !std::filesystem::exists(mcc_file(region_dir, cx, cz), ec);
			if (ec)
				return filled;
		}
		if (replace) {
			current = std::move(generated[i]);
			++filled;
		}
	}
	if (filled && !write_slots(live, slots, ec))
		return 0;
	return filled;
}

std::size_t clear_void(const std::filesystem::path &path, std::error_code &ec)
{
	auto slots = read_slots(path, ec);
	if (ec)
		return 0;
	std::size_t cleared = 0;
	for (auto &slot : slots) {
		if (slot.kind == Slot::Kind::Inline && is_void_chunk(slot.version, slot.data)) {
			slot = Slot{};
			++cleared;
		}
	}
	if (cleared && !write_slots(path, slots, ec))
		return 0;
	return cleared;
}

bool is_void_chunk(std::uint8_t compression, const std::vector<std::uint8_t> &payload)
{
	auto raw = decompress_chunk(compression, payload);
	if (!raw)
		return false;
	Nbt root;
	if (!NbtReader(*raw).root_compound(root))
		return false;
	if (const auto *status = root.find("Status"); status && status->type == 8) {
		std::string_view value = status->string;
		constexpr std::string_view prefix = "minecraft:";
		if (value.substr(0, prefix.size()) == prefix)
			value.remove_prefix(prefix.size());
		if (value != "full")
			return true;
	}
	for (const auto key : {"block_entities", "entities"}) {
		const auto *value = root.find(key);
		if (value && value->type == 9 && !value->list.empty())
			return false;
	}
	const auto *sections = root.find("sections");
	if (!sections || sections->type != 9)
		return true;
	for (const auto &section : sections->list) {
		if (section.type != 10)
			continue;
		const auto *states = section.find("block_states");
		if (!states || states->type != 10)
			continue;
		const auto *palette = states->find("palette");
		if (palette && palette->type == 9 && !all_air_palette(*palette))
			return false;
	}
	return true;
}

std::optional<std::pair<std::int32_t, std::int32_t>> parse_region_name(
		const std::string &name)
{
	if (name.size() < 8 || name.compare(0, 2, "r.") != 0 ||
			name.compare(name.size() - 4, 4, ".mca") != 0)
		return std::nullopt;
	const auto middle = name.substr(2, name.size() - 6);
	const auto dot = middle.find('.');
	if (dot == std::string::npos || middle.find('.', dot + 1) != std::string::npos)
		return std::nullopt;
	std::int32_t rx = 0, rz = 0;
	const auto [x_end, x_ec] = std::from_chars(middle.data(), middle.data() + dot, rx);
	const auto [z_end, z_ec] =
			std::from_chars(middle.data() + dot + 1, middle.data() + middle.size(), rz);
	if (x_ec != std::errc{} || z_ec != std::errc{} || x_end != middle.data() + dot ||
			z_end != middle.data() + middle.size())
		return std::nullopt;
	return std::pair{rx, rz};
}
} // namespace arnis::live::region
