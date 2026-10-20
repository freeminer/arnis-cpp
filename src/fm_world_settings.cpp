#include "world_utils.h"

#include "args.h"
#include "assets_root.h"
#include "world_editor/floor_state.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <string_view>
#include <utility>
#include <zlib.h>

namespace arnis::world_utils
{
namespace
{
constexpr std::size_t MAX_LEVEL_DAT_SIZE = 64 * 1024 * 1024;
constexpr std::size_t MAX_NBT_TAG_COUNT = 500'000;

struct Tag
{
	std::uint8_t type{};
	std::string name;
	std::uint64_t number{};
	std::string text;
	std::uint8_t list_type{};
	std::vector<Tag> list;
	std::vector<Tag> compound;
	std::vector<std::uint8_t> raw;
};

class NbtReader
{
	const std::vector<std::uint8_t> &bytes_;
	std::size_t pos_{};
	std::size_t tags_read_{};

	bool take(std::size_t count, const std::uint8_t **data)
	{
		if (pos_ > bytes_.size() || count > bytes_.size() - pos_)
			return false;
		*data = bytes_.data() + pos_;
		pos_ += count;
		return true;
	}

	bool u8(std::uint8_t &value)
	{
		const std::uint8_t *p;
		if (!take(1, &p))
			return false;
		value = *p;
		return true;
	}

	bool u16(std::uint16_t &value)
	{
		const std::uint8_t *p;
		if (!take(2, &p))
			return false;
		value = (std::uint16_t(p[0]) << 8) | p[1];
		return true;
	}

	bool i32(std::int32_t &value)
	{
		const std::uint8_t *p;
		if (!take(4, &p))
			return false;
		const auto raw = (std::uint32_t(p[0]) << 24) | (std::uint32_t(p[1]) << 16) |
						 (std::uint32_t(p[2]) << 8) | p[3];
		value = static_cast<std::int32_t>(raw);
		return true;
	}

	bool name(std::string &value)
	{
		std::uint16_t length{};
		const std::uint8_t *p;
		if (!u16(length) || !take(length, &p))
			return false;
		value.assign(reinterpret_cast<const char *>(p), length);
		return true;
	}

	bool payload(Tag &tag, unsigned depth)
	{
		if (depth > 64)
			return false;
		const std::uint8_t *p = nullptr;
		std::int32_t count{};
		std::uint16_t length{};
		switch (tag.type) {
		case 1:
			if (!take(1, &p))
				return false;
			tag.number = p[0];
			return true;
		case 2:
			if (!take(2, &p))
				return false;
			tag.number = (std::uint16_t(p[0]) << 8) | p[1];
			return true;
		case 3:
		case 5:
			if (!take(4, &p))
				return false;
			tag.number = (std::uint32_t(p[0]) << 24) | (std::uint32_t(p[1]) << 16) |
						 (std::uint32_t(p[2]) << 8) | p[3];
			return true;
		case 4:
		case 6:
			if (!take(8, &p))
				return false;
			for (int i = 0; i < 8; ++i)
				tag.number = (tag.number << 8) | p[i];
			return true;
		case 7:
		case 11:
		case 12: {
			const auto start = pos_;
			if (!i32(count) || count < 0)
				return false;
			const std::size_t width = tag.type == 7 ? 1 : (tag.type == 11 ? 4 : 8);
			if (std::size_t(count) > (bytes_.size() - pos_) / width)
				return false;
			if (!take(std::size_t(count) * width, &p))
				return false;
			tag.raw.assign(bytes_.begin() + start, bytes_.begin() + pos_);
			return true;
		}
		case 8:
			if (!u16(length) || !take(length, &p))
				return false;
			tag.text.assign(reinterpret_cast<const char *>(p), length);
			return true;
		case 9:
			if (!u8(tag.list_type) || !i32(count) || count < 0 ||
					(tag.list_type == 0 && count != 0))
				return false;
			if (std::size_t(count) > MAX_NBT_TAG_COUNT - tags_read_)
				return false;
			tag.list.reserve(static_cast<std::size_t>(count));
			for (std::int32_t i = 0; i < count; ++i) {
				if (++tags_read_ > MAX_NBT_TAG_COUNT)
					return false;
				Tag item;
				item.type = tag.list_type;
				if (!payload(item, depth + 1))
					return false;
				tag.list.push_back(std::move(item));
			}
			return true;
		case 10:
			for (;;) {
				Tag child;
				if (!u8(child.type))
					return false;
				if (child.type == 0)
					return true;
				if (child.type > 12 || ++tags_read_ > MAX_NBT_TAG_COUNT ||
						!name(child.name) || !payload(child, depth + 1))
					return false;
				tag.compound.push_back(std::move(child));
			}
		default:
			return false;
		}
	}

public:
	explicit NbtReader(const std::vector<std::uint8_t> &bytes) : bytes_(bytes) {}
	bool root(Tag &tag)
	{
		tags_read_ = 1;
		if (!u8(tag.type) || tag.type != 10 || !name(tag.name))
			return false;
		return payload(tag, 0) && pos_ == bytes_.size();
	}
};

void append_u16(std::vector<std::uint8_t> &out, std::uint16_t v)
{
	out.push_back(std::uint8_t(v >> 8));
	out.push_back(std::uint8_t(v));
}
void append_i32(std::vector<std::uint8_t> &out, std::int32_t v)
{
	const auto n = static_cast<std::uint32_t>(v);
	out.push_back(std::uint8_t(n >> 24));
	out.push_back(std::uint8_t(n >> 16));
	out.push_back(std::uint8_t(n >> 8));
	out.push_back(std::uint8_t(n));
}
void append_u32(std::vector<std::uint8_t> &out, std::uint32_t n)
{
	out.push_back(std::uint8_t(n >> 24));
	out.push_back(std::uint8_t(n >> 16));
	out.push_back(std::uint8_t(n >> 8));
	out.push_back(std::uint8_t(n));
}
bool write_name(std::vector<std::uint8_t> &out, const std::string &name)
{
	if (name.size() > std::numeric_limits<std::uint16_t>::max())
		return false;
	append_u16(out, static_cast<std::uint16_t>(name.size()));
	out.insert(out.end(), name.begin(), name.end());
	return true;
}

bool write_payload(std::vector<std::uint8_t> &out, const Tag &tag, unsigned depth)
{
	if (depth > 64)
		return false;
	switch (tag.type) {
	case 1:
		out.push_back(std::uint8_t(tag.number));
		return true;
	case 2:
		append_u16(out, std::uint16_t(tag.number));
		return true;
	case 3:
	case 5:
		append_u32(out, static_cast<std::uint32_t>(tag.number));
		return true;
	case 4:
	case 6:
		for (int i = 7; i >= 0; --i)
			out.push_back(std::uint8_t(tag.number >> (i * 8)));
		return true;
	case 7:
	case 11:
	case 12:
		out.insert(out.end(), tag.raw.begin(), tag.raw.end());
		return true;
	case 8:
		if (tag.text.size() > std::numeric_limits<std::uint16_t>::max())
			return false;
		append_u16(out, static_cast<std::uint16_t>(tag.text.size()));
		out.insert(out.end(), tag.text.begin(), tag.text.end());
		return true;
	case 9:
		if (tag.list.size() > std::size_t(std::numeric_limits<std::int32_t>::max()))
			return false;
		out.push_back(tag.list_type);
		append_i32(out, static_cast<std::int32_t>(tag.list.size()));
		for (const auto &item : tag.list)
			if (item.type != tag.list_type || !write_payload(out, item, depth + 1))
				return false;
		return true;
	case 10:
		for (const auto &child : tag.compound) {
			out.push_back(child.type);
			if (!write_name(out, child.name) || !write_payload(out, child, depth + 1))
				return false;
		}
		out.push_back(0);
		return true;
	default:
		return false;
	}
}

bool serialize(const Tag &root, std::vector<std::uint8_t> &out)
{
	if (root.type != 10 || !write_name(out, root.name))
		return false;
	out.insert(out.begin(), 10);
	return write_payload(out, root, 0) && out.size() <= MAX_LEVEL_DAT_SIZE;
}

Tag *field(Tag &tag, std::string_view name)
{
	if (tag.type != 10)
		return nullptr;
	for (auto &value : tag.compound)
		if (value.name == name)
			return &value;
	return nullptr;
}

void set_field(Tag &tag, Tag value)
{
	if (tag.type != 10)
		return;
	for (auto &current : tag.compound)
		if (current.name == value.name) {
			current = std::move(value);
			return;
		}
	tag.compound.push_back(std::move(value));
}

Tag *nested(Tag &root, std::initializer_list<std::string_view> path)
{
	Tag *current = &root;
	for (const auto key : path) {
		current = field(*current, key);
		if (!current)
			return nullptr;
	}
	return current;
}

Tag string_tag(std::string name, std::string value)
{
	Tag tag;
	tag.type = 8;
	tag.name = std::move(name);
	tag.text = std::move(value);
	return tag;
}
Tag int_tag(std::string name, std::int32_t value)
{
	Tag tag;
	tag.type = 3;
	tag.name = std::move(name);
	tag.number = std::uint32_t(value);
	return tag;
}
Tag byte_tag(std::string name, std::uint8_t value)
{
	Tag tag;
	tag.type = 1;
	tag.name = std::move(name);
	tag.number = value;
	return tag;
}
Tag compound_tag(std::string name)
{
	Tag tag;
	tag.type = 10;
	tag.name = std::move(name);
	return tag;
}
Tag air_layer(std::int32_t height)
{
	Tag layer = compound_tag({});
	set_field(layer, string_tag("block", "minecraft:air"));
	set_field(layer, int_tag("height", height));
	return layer;
}

void set_void_generator(Tag &root)
{
	Tag *generator = nested(root, {"Data", "WorldGenSettings", "dimensions",
										  "minecraft:overworld", "generator"});
	if (!generator || !field(*generator, "type") ||
			field(*generator, "type")->type != 8 ||
			field(*generator, "type")->text != "minecraft:flat")
		return;
	Tag *settings = field(*generator, "settings");
	if (!settings || settings->type != 10)
		return;
	Tag layers;
	layers.type = 9;
	layers.name = "layers";
	layers.list_type = 10;
	layers.list.push_back(air_layer(1));
	set_field(*settings, std::move(layers));
	set_field(*settings, string_tag("biome", "minecraft:the_void"));
	set_field(*settings, byte_tag("features", 0));
	set_field(*settings, byte_tag("lakes", 0));
	Tag overrides;
	overrides.type = 9;
	overrides.name = "structure_overrides";
	overrides.list_type = 0;
	set_field(*settings, std::move(overrides));
}

void raise_flat_generator(Tag &root, std::int32_t base_y, std::int32_t min_y)
{
	const auto air_height = base_y - min_y - 2;
	if (air_height <= 0)
		return;
	Tag *generator = nested(root, {"Data", "WorldGenSettings", "dimensions",
										  "minecraft:overworld", "generator"});
	if (!generator || !field(*generator, "type") ||
			field(*generator, "type")->type != 8 ||
			field(*generator, "type")->text != "minecraft:flat")
		return;
	Tag *settings = field(*generator, "settings");
	Tag *layers = settings ? field(*settings, "layers") : nullptr;
	if (!layers || layers->type != 9 || layers->list_type != 10)
		return;
	if (!layers->list.empty()) {
		Tag &first = layers->list.front();
		Tag *block = field(first, "block");
		if (block && block->type == 8 && block->text == "minecraft:air") {
			set_field(first, int_tag("height", air_height));
			return;
		}
	}
	layers->list.insert(layers->list.begin(), air_layer(air_height));
}

bool read_gzip(const std::filesystem::path &path, std::vector<std::uint8_t> &out,
		std::string *error)
{
	std::ifstream file(path, std::ios::binary);
	if (!file) {
		if (error)
			*error = "failed to read " + path.string();
		return false;
	}
	std::vector<std::uint8_t> input((std::istreambuf_iterator<char>(file)), {});
	if (input.empty() || input.size() > std::numeric_limits<uInt>::max()) {
		if (error)
			*error = "invalid or oversized gzip file " + path.string();
		return false;
	}
	z_stream stream{};
	if (inflateInit2(&stream, 16 + MAX_WBITS) != Z_OK) {
		if (error)
			*error = "cannot initialize gzip decompressor";
		return false;
	}
	stream.next_in = input.data();
	stream.avail_in = static_cast<uInt>(input.size());
	std::array<std::uint8_t, 32768> chunk{};
	int status = Z_OK;
	while (status == Z_OK && out.size() <= MAX_LEVEL_DAT_SIZE) {
		stream.next_out = chunk.data();
		stream.avail_out = static_cast<uInt>(chunk.size());
		status = inflate(&stream, Z_NO_FLUSH);
		out.insert(out.end(), chunk.begin(),
				chunk.begin() + (chunk.size() - stream.avail_out));
	}
	inflateEnd(&stream);
	if (status != Z_STREAM_END || out.size() > MAX_LEVEL_DAT_SIZE) {
		if (error)
			*error = "failed to decompress or oversized level.dat";
		return false;
	}
	return true;
}

bool write_gzip(const std::filesystem::path &path, const std::vector<std::uint8_t> &input,
		std::string *error)
{
	if (input.size() > std::numeric_limits<uInt>::max()) {
		if (error)
			*error = "oversized level.dat payload";
		return false;
	}
	z_stream stream{};
	if (deflateInit2(&stream, Z_DEFAULT_COMPRESSION, Z_DEFLATED, 16 + MAX_WBITS, 8,
				Z_DEFAULT_STRATEGY) != Z_OK) {
		if (error)
			*error = "cannot initialize gzip compressor";
		return false;
	}
	stream.next_in = const_cast<Bytef *>(input.data());
	stream.avail_in = static_cast<uInt>(input.size());
	std::vector<std::uint8_t> compressed;
	std::array<std::uint8_t, 32768> chunk{};
	int status = Z_OK;
	while (status == Z_OK) {
		stream.next_out = chunk.data();
		stream.avail_out = static_cast<uInt>(chunk.size());
		status = deflate(&stream, Z_FINISH);
		compressed.insert(compressed.end(), chunk.begin(),
				chunk.begin() + (chunk.size() - stream.avail_out));
	}
	deflateEnd(&stream);
	if (status != Z_STREAM_END || !replace_file_atomically(path, compressed)) {
		if (error)
			*error = "failed to atomically write " + path.string();
		return false;
	}
	return true;
}

bool edit_level_dat(const std::filesystem::path &world,
		const std::function<bool(Tag &, std::string *)> &edit, std::string *error)
{
	const auto path = world / "level.dat";
	std::vector<std::uint8_t> bytes;
	if (!read_gzip(path, bytes, error))
		return false;
	Tag root;
	if (!NbtReader(bytes).root(root)) {
		if (error)
			*error = "invalid level.dat NBT";
		return false;
	}
	if (!edit(root, error))
		return false;
	std::vector<std::uint8_t> encoded;
	if (!serialize(root, encoded)) {
		if (error)
			*error = "failed to serialize level.dat NBT";
		return false;
	}
	return write_gzip(path, encoded, error);
}
} // namespace

void remove_untouched_template_region(const std::filesystem::path &world)
{
	const auto template_path = assets::path("minecraft/region.template");
	const auto region_path = world / "region" / "r.0.0.mca";
	std::error_code ec;
	const auto size = std::filesystem::file_size(region_path, ec);
	if (ec)
		return;
	std::ifstream file(template_path, std::ios::binary);
	if (!file)
		return;
	std::vector<std::uint8_t> expected((std::istreambuf_iterator<char>(file)), {});
	if (expected.size() != size)
		return;
	std::ifstream current(region_path, std::ios::binary);
	if (!current)
		return;
	std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(current)), {});
	if (bytes == expected)
		std::filesystem::remove(region_path, ec);
}

bool touch_last_played(const std::filesystem::path &world, std::string *error)
{
	if (error)
		error->clear();
	const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch())
							 .count();
	return edit_level_dat(
			world,
			[now](Tag &root, std::string *why) {
				Tag *data = field(root, "Data");
				if (!data || data->type != 10) {
					if (why)
						*why = "level.dat missing Data compound";
					return false;
				}
				Tag last;
				last.type = 4;
				last.name = "LastPlayed";
				last.number = static_cast<std::uint64_t>(now);
				set_field(*data, std::move(last));
				return true;
			},
			error);
}

bool apply_java_world_settings(const std::filesystem::path &world, int game_type,
		std::int64_t world_time, bool void_world, std::int32_t base_y, std::int32_t min_y,
		std::string *error)
{
	if (error)
		error->clear();
	return edit_level_dat(
			world,
			[=](Tag &root, std::string *why) {
				Tag *data = field(root, "Data");
				if (!data || data->type != 10) {
					if (why)
						*why = "level.dat missing Data compound";
					return false;
				}
				set_field(*data, int_tag("GameType", game_type));
				Tag time;
				time.type = 4;
				time.name = "DayTime";
				time.number = static_cast<std::uint64_t>(world_time);
				set_field(*data, std::move(time));
				if (Tag *player = field(*data, "Player"); player && player->type == 10)
					set_field(*player, int_tag("playerGameType", game_type));
				if (void_world)
					set_void_generator(root);
				else
					raise_flat_generator(root, base_y, min_y);
				return true;
			},
			error);
}
} // namespace arnis::world_utils
