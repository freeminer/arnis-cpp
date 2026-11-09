#include "map_item.h"
#include "world_utils.h"
#include <algorithm>
#include <array>
#include <limits>
#include <zlib.h>

namespace
{
constexpr std::int32_t MAP_FORMAT_DATA_VERSION = 4189;
constexpr std::int32_t JAVA_DATA_VERSION = 3955;

void append_u16(std::vector<std::uint8_t> &out, std::uint16_t value)
{
	out.push_back(static_cast<std::uint8_t>(value >> 8));
	out.push_back(static_cast<std::uint8_t>(value));
}

void append_i32(std::vector<std::uint8_t> &out, std::int32_t value)
{
	const auto bits = static_cast<std::uint32_t>(value);
	out.push_back(static_cast<std::uint8_t>(bits >> 24));
	out.push_back(static_cast<std::uint8_t>(bits >> 16));
	out.push_back(static_cast<std::uint8_t>(bits >> 8));
	out.push_back(static_cast<std::uint8_t>(bits));
}

void append_named_int(
		std::vector<std::uint8_t> &out, const char *name, std::int32_t value)
{
	const auto length = std::char_traits<char>::length(name);
	out.push_back(3); // TAG_Int
	append_u16(out, static_cast<std::uint16_t>(length));
	out.insert(out.end(), name, name + length);
	append_i32(out, value);
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
	std::array<std::uint8_t, 4096> buffer{};
	int status = Z_OK;
	while (status == Z_OK) {
		stream.next_out = buffer.data();
		stream.avail_out = static_cast<uInt>(buffer.size());
		status = deflate(&stream, Z_FINISH);
		const auto produced = buffer.size() - stream.avail_out;
		output.insert(output.end(), buffer.begin(), buffer.begin() + produced);
	}
	deflateEnd(&stream);
	return status == Z_STREAM_END ? std::optional(std::move(output)) : std::nullopt;
}

std::vector<std::uint8_t> id_counter_nbt(std::int32_t last_id, std::int32_t data_version)
{
	std::vector<std::uint8_t> nbt{10, 0, 0}; // Root compound with an empty name.
	append_named_int(nbt, "DataVersion", data_version);
	nbt.push_back(10); // TAG_Compound "data".
	append_u16(nbt, 4);
	nbt.insert(nbt.end(), {'d', 'a', 't', 'a'});
	append_named_int(nbt, "map", last_id);
	nbt.push_back(0); // End data compound.
	nbt.push_back(0); // End root compound.
	return nbt;
}
} // namespace

namespace arnis::map_item
{
std::tuple<int, std::int8_t, bool> map_geometry(int d)
{
	for (int s = 0; s <= 4; ++s) {
		const int blocks = 1 << s;
		if (d <= MAP_SIZE * blocks)
			return {blocks, static_cast<std::int8_t>(s), true};
	}
	// An oversized preview still uses scale 4, but fit the full world into its
	// 128 pixels instead of clipping it to Minecraft's largest standard map.
	const int blocks = d / MAP_SIZE + (d % MAP_SIZE != 0);
	return {blocks, 4, false};
}

std::optional<std::pair<int, int>> read_spawn_xz(const std::filesystem::path &world_path)
{
	return world_utils::level_spawn_xz(world_path);
}
}

namespace arnis::map_item
{
int next_map_id(const std::filesystem::path &maps_dir)
{
	std::optional<int> highest;
	for (const char *name : {"idcounts.dat", "last_id.dat"}) {
		if (const auto value = world_utils::map_id_counter(maps_dir / name))
			highest = highest ? std::max(*highest, *value) : *value;
	}
	return highest ? (*highest == std::numeric_limits<int>::max()
									 ? *highest
									 : std::max(*highest + 1, FIRST_MAP_ID))
				   : FIRST_MAP_ID;
}

bool set_map_counter(
		const std::filesystem::path &world_path, int last_id, std::string *error)
{
	const auto layout = world_utils::WorldLayout::of(world_path);
	const auto maps_dir = layout.maps_dir(world_path);
	std::error_code ec;
	std::filesystem::create_directories(maps_dir, ec);
	if (ec) {
		if (error)
			*error = "create " + maps_dir.string() + ": " + ec.message();
		return false;
	}
	const auto format_version = std::min(
			world_utils::level_data_version(world_path).value_or(JAVA_DATA_VERSION),
			MAP_FORMAT_DATA_VERSION);
	const auto compressed = gzip_nbt(id_counter_nbt(last_id, format_version));
	if (!compressed) {
		if (error)
			*error = "could not encode map id counter for " + world_path.string();
		return false;
	}
	const auto name = layout.kind == world_utils::WorldLayout::Kind::Legacy
							  ? "idcounts.dat"
							  : "last_id.dat";
	if (!world_utils::replace_file_atomically(maps_dir / name, *compressed)) {
		if (error)
			*error = "could not write map id counter " + (maps_dir / name).string();
		return false;
	}
	return true;
}
} // namespace arnis::map_item
