#include "schem_decoder.h"
#include "../../../arnis_adapter.h"

#include <cstring>
#include <stdexcept>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <zlib.h>

namespace arnis::structures
{
StructureSchematic structure_schematic(const SchemDocument &document)
{
	StructureSchematic out{document.width, document.length, document.voxels};
	out.entities = document.entities;
	for (const auto &v : out.voxels)
		out.max_extent = std::max(out.max_extent,
				std::max(std::abs(v.x - out.anchor_x), std::abs(v.z - out.anchor_z)));
	return out;
}
namespace
{
SchemDocument document_from_structure(const StructureSchematic &structure)
{
	SchemDocument document;
	document.width = structure.width;
	document.length = structure.length;
	document.height = 1;
	document.offset_x = -structure.anchor_x;
	document.offset_z = -structure.anchor_z;
	document.voxels = structure.voxels;
	document.entities = structure.entities;
	for (const auto &voxel : document.voxels)
		document.height = std::max(document.height, voxel.y + 1);
	return document;
}
}
bool place_structure_yaw(world_editor::WorldEditor &editor,
		const StructureSchematic &structure, int base_x, int base_y, int base_z,
		double yaw_degrees, double pitch_degrees)
{
	return place_schem_document_yaw(editor, document_from_structure(structure), base_x,
			base_y, base_z, yaw_degrees, pitch_degrees);
}
bool place_structure(world_editor::WorldEditor &editor,
		const StructureSchematic &structure, int base_x, int base_y, int base_z,
		unsigned rotation)
{
	return place_structure_yaw(editor, structure, base_x, base_y, base_z,
			90.0 * static_cast<double>(rotation & 3));
}
StructureSchematic StructureSchematic::centered() const
{
	auto out = *this;
	out.anchor_x = out.width / 2;
	out.anchor_z = out.length / 2;
	out.max_extent = 0;
	for (const auto &v : out.voxels)
		out.max_extent = std::max(out.max_extent,
				std::max(std::abs(v.x - out.anchor_x), std::abs(v.z - out.anchor_z)));
	return out;
}
StructureSchematic StructureSchematic::base_anchored() const
{
	auto out = *this;
	long long sx = 0, sz = 0, count = 0;
	for (const auto &v : out.voxels)
		if (v.y == 0) {
			sx += v.x;
			sz += v.z;
			++count;
		}
	if (count) {
		out.anchor_x = int(sx / count);
		out.anchor_z = int(sz / count);
	}
	out.max_extent = 0;
	for (const auto &v : out.voxels)
		out.max_extent = std::max(out.max_extent,
				std::max(std::abs(v.x - out.anchor_x), std::abs(v.z - out.anchor_z)));
	return out;
}
namespace
{
struct Reader
{
	const std::vector<std::uint8_t> &b;
	std::size_t p = 0;
	std::uint8_t u8()
	{
		if (p >= b.size())
			throw std::runtime_error("truncated NBT");
		return b[p++];
	}
	std::uint16_t u16() { return (u16be(u8()) << 8) | u8(); }
	std::int32_t i32()
	{
		const auto v = u32();
		return static_cast<std::int32_t>(v);
	}
	std::uint32_t u32()
	{
		u32_last = (static_cast<std::uint32_t>(u8()) << 24) |
				   (static_cast<std::uint32_t>(u8()) << 16) |
				   (static_cast<std::uint32_t>(u8()) << 8) | u8();
		return u32_last;
	}
	std::int64_t i64()
	{
		std::uint64_t v = 0;
		for (int i = 0; i < 8; i++)
			v = (v << 8) | u8();
		return static_cast<std::int64_t>(v);
	}
	std::string str()
	{
		const auto n = u16();
		if (p + n > b.size())
			throw std::runtime_error("truncated NBT string");
		std::string s(reinterpret_cast<const char *>(b.data() + p), n);
		p += n;
		return s;
	}
	std::uint32_t u32_last = 0;

private:
	static std::uint16_t u16be(std::uint8_t v) { return v; }
};

void skip_payload(Reader &r, std::uint8_t type)
{
	switch (type) {
	case 1:
		r.u8();
		break;
	case 2:
		r.u16();
		break;
	case 3:
		r.u32();
		break;
	case 4:
		r.i64();
		break;
	case 5:
		r.u32();
		r.u32();
		break;
	case 6:
		r.i64();
		r.i64();
		break;
	case 7: {
		auto n = r.u32();
		for (std::uint32_t i = 0; i < n; i++)
			r.u8();
		break;
	}
	case 8:
		r.str();
		break;
	case 9: {
		auto t = r.u8();
		auto n = r.u32();
		for (std::uint32_t i = 0; i < n; i++)
			skip_payload(r, t);
		break;
	}
	case 10:
		for (;;) {
			auto t = r.u8();
			if (!t)
				break;
			r.str();
			skip_payload(r, t);
		}
		break;
	case 11: {
		auto n = r.u32();
		for (std::uint32_t i = 0; i < n; i++)
			r.u32();
		break;
	}
	case 12: {
		auto n = r.u32();
		for (std::uint32_t i = 0; i < n; i++)
			r.i64();
		break;
	}
	default:
		throw std::runtime_error("unsupported NBT type");
	}
}

std::vector<std::uint8_t> ungzip(const std::vector<std::uint8_t> &in)
{
	std::vector<std::uint8_t> out(in.size() * 8 + 1024);
	z_stream z{};
	if (inflateInit2(&z, 15 + 16) != Z_OK)
		throw std::runtime_error("gzip init failed");
	z.next_in = const_cast<Bytef *>(in.data());
	z.avail_in = static_cast<uInt>(in.size());
	for (;;) {
		if (z.total_out == out.size())
			out.resize(out.size() * 2);
		z.next_out = out.data() + z.total_out;
		z.avail_out = static_cast<uInt>(out.size() - z.total_out);
		int rc = inflate(&z, Z_NO_FLUSH);
		if (rc == Z_STREAM_END) {
			out.resize(z.total_out);
			inflateEnd(&z);
			return out;
		}
		if (rc != Z_OK) {
			inflateEnd(&z);
			throw std::runtime_error("invalid gzip schem");
		}
	}
}

void decode_block_data(Reader &r, std::vector<std::int32_t> &data)
{
	const auto byte_count = r.u32();
	if (byte_count > r.b.size() - r.p)
		throw std::runtime_error("truncated schem BlockData");
	const std::size_t end = r.p + byte_count;
	while (r.p < end) {
		std::uint32_t value = 0;
		unsigned shift = 0;
		for (;;) {
			if (r.p >= end)
				throw std::runtime_error("truncated schem BlockData varint");
			const std::uint8_t byte = r.u8();
			if (shift >= 32 && (byte & 0x7f))
				throw std::runtime_error("schem BlockData varint overflow");
			if (shift < 32)
				value |= std::uint32_t(byte & 0x7f) << shift;
			if (!(byte & 0x80))
				break;
			shift += 7;
		}
		data.push_back(static_cast<std::int32_t>(value));
	}
}

void decode_palette(Reader &r, std::unordered_map<int, std::string> &palette)
{
	for (;;) {
		const auto type = r.u8();
		if (!type)
			return;
		const auto name = r.str();
		if (type == 3)
			palette[static_cast<int>(r.u32())] = name;
		else
			skip_payload(r, type);
	}
}

void decode_block_entities(Reader &r, SchemDocument &doc)
{
	const auto element_type = r.u8();
	const auto count = r.u32();
	if (element_type != 10)
		throw std::runtime_error("schem block entities are not compounds");
	for (std::uint32_t i = 0; i < count; ++i) {
		const auto begin = r.p;
		SchemEntity entity;
		for (;;) {
			const auto type = r.u8();
			if (!type)
				break;
			const auto name = r.str();
			if (type == 3 && (name == "x" || name == "y" || name == "z")) {
				const auto value = r.i32();
				if (name == "x")
					entity.x = value;
				else if (name == "y")
					entity.y = value;
				else
					entity.z = value;
			} else {
				skip_payload(r, type);
			}
		}
		entity.nbt.assign(r.b.begin() + begin, r.b.begin() + r.p);
		doc.entities.push_back(std::move(entity));
	}
}

std::vector<std::tuple<std::string, int, int>> entity_items(
		const std::vector<std::uint8_t> &bytes)
{
	std::vector<std::tuple<std::string, int, int>> out;
	try {
		Reader r{bytes};
		std::string id;
		for (;;) {
			const auto type = r.u8();
			if (!type)
				break;
			const auto name = r.str();
			if (type == 8) {
				const auto value = r.str();
				if (name == "id")
					id = value;
			} else if (type == 9 && name == "Items") {
				const auto item_type = r.u8();
				const auto count = r.u32();
				if (item_type != 10) {
					skip_payload(r, item_type);
					continue;
				}
				for (std::uint32_t i = 0; i < count; ++i) {
					std::string item_id;
					int slot = static_cast<int>(i), amount = 1;
					for (;;) {
						const auto t = r.u8();
						if (!t)
							break;
						const auto n = r.str();
						if (t == 8 && n == "id")
							item_id = r.str();
						else if (t == 1 && n == "Count")
							amount = static_cast<int>(r.u8());
						else if (t == 1 && n == "Slot")
							slot = static_cast<int>(r.u8());
						else
							skip_payload(r, t);
					}
					if (!item_id.empty())
						out.emplace_back(item_id, slot, amount);
				}
			} else {
				skip_payload(r, type);
			}
		}
	} catch (...) {
	}
	return out;
}

std::string entity_string(
		const std::vector<std::uint8_t> &bytes, const std::string &wanted)
{
	try {
		Reader r{bytes};
		for (;;) {
			const auto type = r.u8();
			if (!type)
				break;
			const auto name = r.str();
			if (type == 8) {
				auto value = r.str();
				if (name == wanted)
					return value;
			} else
				skip_payload(r, type);
		}
	} catch (...) {
	}
	return {};
}

std::vector<std::pair<std::string, std::string>> entity_banner_patterns(
		const std::vector<std::uint8_t> &bytes)
{
	std::vector<std::pair<std::string, std::string>> out;
	try {
		Reader r{bytes};
		for (;;) {
			const auto type = r.u8();
			if (!type)
				break;
			const auto name = r.str();
			if (type != 9 || name != "Patterns") {
				skip_payload(r, type);
				continue;
			}
			const auto element_type = r.u8();
			const auto count = r.u32();
			if (element_type != 10)
				continue;
			for (std::uint32_t i = 0; i < count; ++i) {
				std::string pattern, color;
				for (;;) {
					const auto t = r.u8();
					if (!t)
						break;
					const auto n = r.str();
					if (t == 8 && n == "Pattern")
						pattern = r.str();
					else if (t == 3 && n == "Color")
						color = std::to_string(r.u32());
					else
						skip_payload(r, t);
				}
				if (!pattern.empty())
					out.emplace_back(pattern, color);
			}
		}
	} catch (...) {
	}
	return out;
}

std::string entity_item_id(const std::vector<std::uint8_t> &bytes)
{
	try {
		Reader r{bytes};
		for (;;) {
			const auto type = r.u8();
			if (!type)
				break;
			const auto name = r.str();
			if (type == 10 && name == "Item") {
				for (;;) {
					const auto t = r.u8();
					if (!t)
						break;
					const auto n = r.str();
					if (t == 8 && n == "id")
						return r.str();
					skip_payload(r, t);
				}
			} else
				skip_payload(r, type);
		}
	} catch (...) {
	}
	return {};
}

void place_native_entity(
		world_editor::WorldEditor &editor, int x, int y, int z, const SchemEntity &entity)
{
	const auto items = entity_items(entity.nbt);
	const auto id = entity_string(entity.nbt, "id");
	if (id.find("chest") != std::string::npos) {
		editor.set_chest_with_items_absolute(x, y, z, items);
		return;
	}
	if (id.find("barrel") != std::string::npos) {
		editor.set_barrel_with_items_absolute(x, y, z, items);
		return;
	}
	if (id.find("sign") != std::string::npos ||
			id.find("hanging_sign") != std::string::npos) {
		std::string text;
		for (int line = 1; line <= 4; ++line) {
			const auto value = entity_string(entity.nbt, "Text" + std::to_string(line));
			if (!value.empty()) {
				if (!text.empty())
					text += '\n';
				text += value;
			}
		}
		if (editor.place_sign_node(block_definitions::SIGN, x, y, z, 0, text))
			return;
	}
	if (id.find("item_frame") != std::string::npos) {
		if (editor.item_frame_sink) {
			editor.item_frame_sink(x, y, z, entity_item_id(entity.nbt));
			return;
		}
	}
	if (id.find("banner") != std::string::npos) {
		if (editor.banner_sink) {
			editor.banner_sink(x, y, z, entity_string(entity.nbt, "Facing"),
					entity_string(entity.nbt, "Base"),
					entity_banner_patterns(entity.nbt));
			return;
		}
	}
	if (id.find("bed") != std::string::npos) {
		editor.set_bed_block_entity_absolute(x, y, z);
		return;
	}
	if (editor.schem_entity_sink)
		editor.schem_entity_sink(x, y, z, entity.nbt);
}

// Sponge v3 nests its palette/data beneath `Blocks`, and some writers wrap
// every field in a `Schematic` compound.  Accept both arrangements, sharing
// the same strict byte-stream decoder as the v2 top-level form.
void decode_schematic_fields(Reader &r, SchemDocument &doc,
		std::unordered_map<int, std::string> &palette, std::vector<std::int32_t> &data)
{
	for (;;) {
		const auto type = r.u8();
		if (!type)
			return;
		const auto name = r.str();
		if ((name == "Width" || name == "Height" || name == "Length") && type == 2) {
			const int value = r.u16();
			if (name == "Width")
				doc.width = value;
			else if (name == "Height")
				doc.height = value;
			else
				doc.length = value;
		} else if (name == "Offset" && type == 11) {
			const auto count = r.u32();
			for (std::uint32_t i = 0; i < count; ++i) {
				const int value = r.i32();
				if (i == 0)
					doc.offset_x = value;
				else if (i == 1)
					doc.offset_y = value;
				else if (i == 2)
					doc.offset_z = value;
			}
		} else if (name == "Palette" && type == 10) {
			decode_palette(r, palette);
		} else if ((name == "BlockData" || name == "Data") && type == 7) {
			decode_block_data(r, data);
		} else if ((name == "BlockEntities" || name == "TileEntities") && type == 9) {
			decode_block_entities(r, doc);
		} else if ((name == "Schematic" || name == "Blocks") && type == 10) {
			decode_schematic_fields(r, doc, palette, data);
		} else {
			skip_payload(r, type);
		}
	}
}
}

SchemDocument decode_sponge_schem(const std::vector<std::uint8_t> &gzip_data)
{
	Reader r{ungzip(gzip_data)};
	SchemDocument doc;
	std::unordered_map<int, std::string> palette;
	std::vector<std::int32_t> data;
	if (r.u8() != 10)
		throw std::runtime_error("schem root is not compound");
	r.str();
	// V3 and wrapped v2 schematics are handled by the recursive reader.  Keep
	// the legacy loop below for compatibility with old direct-root files.
	for (;;) {
		auto type = r.u8();
		if (!type)
			break;
		auto name = r.str();
		if ((name == "Schematic" || name == "Blocks") && type == 10) {
			decode_schematic_fields(r, doc, palette, data);
			continue;
		}
		if (name == "Width" || name == "Height" || name == "Length") {
			auto v = r.u16();
			if (name == "Width")
				doc.width = v;
			else if (name == "Height")
				doc.height = v;
			else
				doc.length = v;
		} else if (name == "Offset" && type == 11) {
			// Sponge stores three big-endian ints describing the schematic's
			// world-space origin.  Keep the values even when placement chooses
			// a different anchor.
			auto n = r.u32();
			for (std::uint32_t i = 0; i < n; ++i) {
				const auto v = r.i32();
				if (i == 0)
					doc.offset_x = v;
				else if (i == 1)
					doc.offset_y = v;
				else if (i == 2)
					doc.offset_z = v;
			}
		} else if (name == "Palette" && type == 10) {
			for (;;) {
				auto t = r.u8();
				if (!t)
					break;
				auto n = r.str();
				if (t != 3) {
					skip_payload(r, t);
					continue;
				}
				int id = static_cast<int>(r.u32());
				palette[id] = n;
			}
		} else if (name == "BlockData" && type == 7) {
			decode_block_data(r, data);
		} else if ((name == "BlockEntities" || name == "TileEntities") && type == 9) {
			decode_block_entities(r, doc);
		} else
			skip_payload(r, type);
	}
	if (!doc.width || !doc.height || !doc.length)
		throw std::runtime_error("schem dimensions missing");
	// Rust rejects volumes beyond the signed 32-bit index range before
	// allocating or iterating BlockData.  Besides matching that contract, the
	// checked multiplication prevents hostile dimensions from wrapping size_t.
	constexpr std::uint64_t max_cells = std::numeric_limits<std::int32_t>::max();
	const auto volume = static_cast<std::uint64_t>(doc.width) *
						static_cast<std::uint64_t>(doc.height) *
						static_cast<std::uint64_t>(doc.length);
	if (volume > max_cells)
		throw std::runtime_error("schem volume exceeds the supported range");
	const std::size_t cells = static_cast<std::size_t>(volume);
	if (data.size() != cells)
		throw std::runtime_error("schem BlockData size mismatch");
	for (std::size_t i = 0; i < cells; i++) {
		auto it = palette.find(data[i]);
		if (it == palette.end())
			continue;
		const int x = i % doc.width, z = (i / doc.width) % doc.length,
				  y = i / (doc.width * doc.length);
		std::unordered_map<std::string, std::string> properties;
		const auto open = it->second.find('['), close = it->second.rfind(']');
		if (open != std::string::npos && close != std::string::npos && close > open + 1) {
			std::size_t start = open + 1;
			while (start < close) {
				std::size_t end = start;
				int depth = 0;
				bool quoted = false;
				for (; end < close; ++end) {
					const char ch = it->second[end];
					if (ch == '"' && (end == start || it->second[end - 1] != '\\'))
						quoted = !quoted;
					if (!quoted && (ch == '[' || ch == '{'))
						++depth;
					else if (!quoted && (ch == ']' || ch == '}') && depth)
						--depth;
					else if (!quoted && ch == ',' && depth == 0)
						break;
				}
				const auto equals = it->second.find('=', start);
				if (equals != std::string::npos && equals < end) {
					auto key = it->second.substr(start, equals - start);
					auto value = it->second.substr(equals + 1, end - equals - 1);
					const auto trim = [](std::string &s) {
						const auto a = s.find_first_not_of(" \t");
						const auto b = s.find_last_not_of(" \t");
						s = a == std::string::npos ? "" : s.substr(a, b - a + 1);
					};
					trim(key);
					trim(value);
					properties.emplace(std::move(key), std::move(value));
				}
				start = end < close ? end + 1 : close;
			}
		}
		doc.voxels.push_back({x, y, z, it->second, std::move(properties)});
	}
	return doc;
}

SchemDocument load_palettized(const std::vector<std::uint8_t> &gzip_bytes)
{
	return decode_sponge_schem(gzip_bytes);
}

StructureSchematic load_structure(const std::vector<std::uint8_t> &gzip_bytes)
{
	return structure_schematic(decode_sponge_schem(gzip_bytes));
}

Block resolve_schem_block(const std::string &name)
{
	using namespace block_definitions;
	const auto colon = name.find(':');
	std::string n = colon == std::string::npos ? name : name.substr(colon + 1);
	// Palette entries may carry Java block-state properties, e.g.
	// `oak_log[axis=y]`; the block identity is the part before '['.
	if (const auto state = n.find('['); state != std::string::npos)
		n.resize(state);
	if (n == "air" || n == "cave_air")
		return AIR;
	if (n == "stone")
		return STONE;
	if (n == "stone_bricks")
		return STONE_BRICKS;
	if (n == "smooth_stone")
		return SMOOTH_STONE;
	if (n == "cobblestone")
		return COBBLESTONE;
	if (n == "andesite")
		return ANDESITE;
	if (n == "diorite")
		return DIORITE;
	if (n == "dirt")
		return DIRT;
	if (n == "grass_block")
		return GRASS_BLOCK;
	if (n == "sandstone")
		return SANDSTONE;
	if (n == "smooth_sandstone")
		return SMOOTH_SANDSTONE;
	if (n == "glass")
		return GLASS;
	if (n == "iron_block")
		return IRON_BLOCK;
	if (n == "iron_bars")
		return IRON_BARS;
	if (n == "chain" || n == "iron_chain")
		return CHAIN;
	if (n == "oak_planks")
		return OAK_PLANKS;
	if (n == "spruce_planks")
		return SPRUCE_PLANKS;
	if (n == "dark_oak_planks")
		return DARK_OAK_PLANKS;
	if (n == "oak_log" || n == "oak_wood" || n == "stripped_oak_log" ||
			n == "stripped_oak_wood")
		return OAK_LOG;
	if (n == "birch_log" || n == "birch_wood" || n == "stripped_birch_log" ||
			n == "stripped_birch_wood" || n == "pale_oak_log" || n == "pale_oak_wood" ||
			n == "stripped_pale_oak_log" || n == "stripped_pale_oak_wood")
		return BIRCH_LOG;
	if (n == "spruce_log" || n == "spruce_wood" || n == "stripped_spruce_log" ||
			n == "stripped_spruce_wood" || n == "warped_stem" || n == "warped_hyphae" ||
			n == "stripped_warped_stem" || n == "stripped_warped_hyphae")
		return SPRUCE_LOG;
	if (n == "dark_oak_log" || n == "dark_oak_wood" || n == "stripped_dark_oak_log" ||
			n == "stripped_dark_oak_wood")
		return DARK_OAK_LOG;
	if (n == "jungle_log" || n == "jungle_wood" || n == "stripped_jungle_log" ||
			n == "stripped_jungle_wood" || n == "bamboo_block" ||
			n == "stripped_bamboo_block" || n == "mangrove_log" || n == "mangrove_wood" ||
			n == "stripped_mangrove_log" || n == "stripped_mangrove_wood" ||
			n == "mangrove_roots" || n == "muddy_mangrove_roots")
		return JUNGLE_LOG;
	if (n == "acacia_log" || n == "acacia_wood" || n == "stripped_acacia_log" ||
			n == "stripped_acacia_wood")
		return ACACIA_LOG;
	if (n == "cherry_log" || n == "cherry_wood" || n == "stripped_cherry_log" ||
			n == "stripped_cherry_wood")
		return CHERRY_LOG;
	if (n == "oak_leaves" || n == "vine" || n == "moss_block" || n == "moss_carpet")
		return OAK_LEAVES;
	if (n == "birch_leaves" || n == "pale_oak_leaves")
		return BIRCH_LEAVES;
	if (n == "spruce_leaves")
		return SPRUCE_LEAVES;
	if (n == "dark_oak_leaves")
		return DARK_OAK_LEAVES;
	if (n == "jungle_leaves" || n == "mangrove_leaves" || n == "mangrove_propagule")
		return JUNGLE_LEAVES;
	if (n == "acacia_leaves")
		return ACACIA_LEAVES;
	if (n == "cherry_leaves")
		return CHERRY_LEAVES;
	if (n == "azalea_leaves" || n == "flowering_azalea_leaves")
		return OAK_LEAVES;
	if (n == "water")
		return WATER;
	if (n == "redstone_lamp")
		return REDSTONE_BLOCK;
	if (n == "glowstone")
		return GLOWSTONE;
	if (n == "sea_lantern")
		return SEA_LANTERN;
	if (n == "polished_andesite")
		return POLISHED_ANDESITE;
	if (n == "polished_blackstone")
		return POLISHED_BLACKSTONE;
	if (n == "deepslate_bricks")
		return DEEPSLATE_BRICKS;
	if (n == "quartz_block")
		return QUARTZ_BLOCK;
	if (n == "white_concrete")
		return WHITE_CONCRETE;
	if (n == "light_gray_concrete")
		return LIGHT_GRAY_CONCRETE;
	if (n == "gray_concrete")
		return GRAY_CONCRETE;
	if (n == "black_concrete")
		return BLACK_CONCRETE;
	if (n == "red_concrete")
		return RED_CONCRETE;
	if (n == "green_wool")
		return GREEN_WOOL;
	if (n == "green_terracotta")
		return GREEN_STAINED_HARDENED_CLAY;
	if (n == "red_terracotta")
		return RED_TERRACOTTA;
	if (n == "brown_glazed_terracotta")
		return BROWN_TERRACOTTA;
	if (n == "black_terracotta")
		return BLACK_TERRACOTTA;
	if (n == "mud")
		return MUD;
	if (n == "nether_bricks")
		return NETHER_BRICK;
	// Aeroplane liveries and the jetbridge use these newer Java materials.  The
	// Luanti host does not expose every slab/stair variant, so retain each
	// palette family with its closest available base node instead of dropping it
	// through the old generic-stone fallback.
	if (n == "purpur_block" || n == "purpur_slab" || n == "purpur_stairs")
		return PURPUR_BLOCK;
	if (n == "crimson_planks" || n == "crimson_slab" || n == "crimson_stairs")
		return CRIMSON_PLANKS;
	if (n == "cherry_planks" || n == "cherry_slab" || n == "cherry_stairs")
		return OAK_PLANKS;
	if (n == "dark_prismarine" || n == "dark_prismarine_slab" ||
			n == "dark_prismarine_stairs")
		return PRISMARINE;
	if (n == "waxed_exposed_cut_copper" || n == "waxed_exposed_cut_copper_slab" ||
			n == "waxed_exposed_cut_copper_stairs")
		return WAXED_EXPOSED_CUT_COPPER;
	if (n == "pale_oak_trapdoor")
		return BIRCH_TRAPDOOR;
	if (n == "coal_block")
		return BLACKSTONE;
	if (n == "blackstone_slab")
		return BLACKSTONE;
	if (n == "iron_door")
		return IRON_BARS;
	if (n == "cut_red_sandstone")
		return RED_TERRACOTTA;
	if (n == "white_stained_glass" || n == "tinted_glass" || n == "brown_stained_glass" ||
			n == "black_stained_glass")
		return n == "white_stained_glass" ? WHITE_STAINED_GLASS : GLASS;
	if (n == "cracked_polished_blackstone_bricks")
		return POLISHED_BLACKSTONE_BRICKS;
	if (n == "stone_brick_slab")
		return STONE_BRICK_SLAB;
	if (n == "smooth_stone_slab")
		return SMOOTH_STONE_SLAB;
	if (n == "stone_brick_stairs")
		return STONE_BRICK_STAIRS;
	if (n == "andesite_stairs")
		return ANDESITE_STAIRS;
	if (n == "oak_stairs")
		return OAK_STAIRS;
	if (n == "rail")
		return RAIL;
	if (n == "smooth_sandstone_stairs")
		return SMOOTH_SANDSTONE_STAIRS;
	if (n == "andesite_wall")
		return ANDESITE_WALL;
	if (n == "stone_slab")
		return STONE_BLOCK_SLAB;
	if (n == "white_carpet")
		return WHITE_CARPET;
	if (n == "anvil" || n == "chipped_anvil")
		return ANVIL;
	if (n == "birch_trapdoor")
		return BIRCH_TRAPDOOR;
	if (n == "oak_fence")
		return OAK_FENCE;
	if (n == "spruce_log")
		return SPRUCE_LOG;
	if (n == "dark_oak_log")
		return DARK_OAK_LOG;
	if (n == "cobblestone_stairs")
		return COBBLESTONE_STAIRS;
	if (n == "cobblestone_wall")
		return COBBLESTONE_WALL;
	if (n == "gray_concrete_powder")
		return GRAY_CONCRETE_POWDER;
	// Rust's map_structure_block returns None for unmodelled palette entries;
	// do not invent stone geometry for unsupported blocks.
	return AIR;
}

namespace
{
std::string rotate_direction(const std::string &direction, unsigned rotation)
{
	static constexpr const char *order[] = {"north", "east", "south", "west"};
	for (unsigned i = 0; i < 4; ++i)
		if (direction == order[i])
			return order[(i + rotation) & 3];
	return direction;
}
std::string rotate_rail_shape(const std::string &shape, unsigned rotation)
{
	if ((shape == "north_south" || shape == "east_west") && (rotation & 1))
		return shape == "north_south" ? "east_west" : "north_south";
	if (shape.starts_with("ascending_"))
		return "ascending_" + rotate_direction(shape.substr(10), rotation);
	if (shape == "north_east" || shape == "north_west" || shape == "south_east" ||
			shape == "south_west") {
		const auto split = shape.find('_');
		auto a = rotate_direction(shape.substr(0, split), rotation);
		auto b = rotate_direction(shape.substr(split + 1), rotation);
		return (a == "north" || a == "south") ? a + "_" + b : b + "_" + a;
	}
	return shape;
}
std::unordered_map<std::string, std::string> rotated_properties_impl(
		const std::unordered_map<std::string, std::string> &input, unsigned rotation)
{
	std::unordered_map<std::string, std::string> output;
	for (const auto &[key, value] : input) {
		if (key == "north" || key == "east" || key == "south" || key == "west")
			output.emplace(rotate_direction(key, rotation), value);
		else if (key == "facing")
			output.emplace(key, rotate_direction(value, rotation));
		else if (key == "shape")
			output.emplace(key, rotate_rail_shape(value, rotation));
		else if (key == "axis" && (rotation & 1))
			output.emplace(key, value == "x" ? "z" : value == "z" ? "x" : value);
		else
			output.emplace(key, value);
	}
	return output;
}
}

std::unordered_map<std::string, std::string> rotate_schem_properties(
		const std::unordered_map<std::string, std::string> &input, unsigned rotation)
{
	return rotated_properties_impl(input, rotation);
}

BlockWithProperties resolve_schem_block_with_properties(const std::string &name)
{
	std::unordered_map<std::string, std::string> properties;
	const auto open = name.find('['), close = name.rfind(']');
	if (open != std::string::npos && close != std::string::npos && close > open + 1)
		for (std::size_t start = open + 1, end; start < close; start = end + 1) {
			end = name.find(',', start);
			if (end == std::string::npos || end > close)
				end = close;
			const auto equals = name.find('=', start);
			if (equals != std::string::npos && equals < end)
				properties.emplace(name.substr(start, equals - start),
						name.substr(equals + 1, end - equals - 1));
			if (end == close)
				break;
		}
	return BlockWithProperties{resolve_schem_block(name), std::move(properties)};
}

bool place_schem_document_yaw(world_editor::WorldEditor &editor,
		const SchemDocument &document, int base_x, int base_y, int base_z,
		double yaw_degrees, double pitch_degrees)
{
	if (document.width <= 0 || document.length <= 0 || document.voxels.empty())
		return false;
	const double quarter = yaw_degrees / 90.0;
	const auto quarter_turn = std::llround(quarter);
	const unsigned state_rotation = static_cast<unsigned>((quarter_turn % 4 + 4) % 4);
	double sine, cosine;
	if (std::abs(quarter - double(quarter_turn)) < 1e-9) {
		switch ((quarter_turn % 4 + 4) % 4) {
		case 0:
			sine = 0;
			cosine = 1;
			break;
		case 1:
			sine = 1;
			cosine = 0;
			break;
		case 2:
			sine = 0;
			cosine = -1;
			break;
		default:
			sine = -1;
			cosine = 0;
			break;
		}
	} else {
		sine = std::sin(yaw_degrees * M_PI / 180.0);
		cosine = std::cos(yaw_degrees * M_PI / 180.0);
	}
	const double cx = double(document.width) * .5, cz = double(document.length) * .5;
	const auto pitch_lift = [&](int z) {
		if (pitch_degrees == 0.0)
			return 0;
		return int(std::lround((double(document.length - 1) * .5 - z) *
							   std::tan(pitch_degrees * M_PI / 180.0)));
	};
	int lift_floor = std::numeric_limits<int>::max();
	for (const auto &voxel : document.voxels)
		lift_floor = std::min(lift_floor, voxel.y + pitch_lift(voxel.z));
	std::map<std::pair<int, int>, std::vector<const SchemVoxel *>> columns;
	for (const auto &voxel : document.voxels)
		columns[{voxel.x, voxel.z}].push_back(&voxel);
	int min_x = std::numeric_limits<int>::max(), max_x = std::numeric_limits<int>::min();
	int min_z = std::numeric_limits<int>::max(), max_z = std::numeric_limits<int>::min();
	for (const auto &[mx, mz] : {std::pair{0.0, 0.0}, {double(document.width), 0.0},
				 {0.0, double(document.length)},
				 {double(document.width), double(document.length)}}) {
		const double dx = mx - cx, dz = mz - cz;
		const double wx = base_x + dx * cosine - dz * sine;
		const double wz = base_z + dx * sine + dz * cosine;
		min_x = std::min(min_x, int(std::floor(wx)));
		max_x = std::max(max_x, int(std::ceil(wx)));
		min_z = std::min(min_z, int(std::floor(wz)));
		max_z = std::max(max_z, int(std::ceil(wz)));
	}
	bool placed = false;
	for (int wz = min_z; wz <= max_z; ++wz)
		for (int wx = min_x; wx <= max_x; ++wx) {
			const double dx = wx - base_x, dz = wz - base_z;
			const int mx = int(std::floor(cx + dx * cosine + dz * sine));
			const int mz = int(std::floor(cz - dx * sine + dz * cosine));
			auto column = columns.find({mx, mz});
			if (column == columns.end())
				continue;
			const int lift = pitch_lift(mz) - lift_floor;
			for (const auto *voxel : column->second) {
				// State has no arbitrary-angle representation. Snap it to the
				// nearest cardinal while preserving the hull's true free yaw.
				BlockWithProperties block{resolve_schem_block(voxel->block),
						state_rotation ? rotate_schem_properties(
												 voxel->properties, state_rotation)
									   : voxel->properties};
				if (block.block == block_definitions::AIR)
					continue;
				editor.set_block_with_properties_absolute(std::move(block), wx,
						base_y + voxel->y + lift, wz, nullptr, nullptr);
				placed = true;
			}
		}
	return placed;
}

bool place_schem_file(world_editor::WorldEditor &editor,
		const std::filesystem::path &file, int ox, int oy, int oz)
{
	std::ifstream in(file, std::ios::binary);
	if (!in)
		return false;
	std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)), {});
	SchemDocument doc;
	try {
		doc = decode_sponge_schem(bytes);
	} catch (...) {
		return false;
	}
	for (const auto &v : doc.voxels) {
		BlockWithProperties block{resolve_schem_block(v.block), v.properties};
		if (block.block == block_definitions::AIR)
			continue;
		editor.set_block_with_properties_absolute(std::move(block),
				ox + doc.offset_x + v.x, oy + doc.offset_y + v.y, oz + doc.offset_z + v.z,
				nullptr, nullptr);
	}
	if (editor.schem_entity_sink)
		for (const auto &e : doc.entities)
			place_native_entity(editor, ox + doc.offset_x + e.x, oy + doc.offset_y + e.y,
					oz + doc.offset_z + e.z, e);
	return true;
}

bool place_schem_file_rotated(world_editor::WorldEditor &editor,
		const std::filesystem::path &file, int ox, int oy, int oz, unsigned rotation,
		const Block *ground)
{
	std::ifstream in(file, std::ios::binary);
	if (!in)
		return false;
	std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)), {});
	SchemDocument doc;
	try {
		doc = decode_sponge_schem(bytes);
	} catch (...) {
		return false;
	}
	for (const auto &v : doc.voxels) {
		BlockWithProperties b{resolve_schem_block(v.block),
				rotate_schem_properties(v.properties, rotation & 3)};
		if (b.block == block_definitions::AIR)
			continue;
		int x = v.x + doc.offset_x, z = v.z + doc.offset_z;
		switch (rotation & 3) {
		case 1:
			std::swap(x, z);
			x = -x;
			break;
		case 2:
			x = -x;
			z = -z;
			break;
		case 3:
			std::swap(x, z);
			z = -z;
			break;
		}
		const int wx = ox + x, wy = oy + doc.offset_y + v.y, wz = oz + z;
		editor.set_block_with_properties_absolute(
				std::move(b), wx, wy, wz, nullptr, nullptr);
		// Rust's place_structure optionally force-fills the terrain immediately
		// below each schematic column (used by bundled playgrounds).
		if (ground)
			editor.set_block_absolute(*ground, wx, wy - 1, wz, nullptr, nullptr);
	}
	if (editor.schem_entity_sink)
		for (const auto &e : doc.entities) {
			int x = e.x + doc.offset_x, z = e.z + doc.offset_z;
			switch (rotation & 3) {
			case 1:
				std::swap(x, z);
				x = -x;
				break;
			case 2:
				x = -x;
				z = -z;
				break;
			case 3:
				std::swap(x, z);
				z = -z;
				break;
			}
			place_native_entity(editor, ox + x, oy + doc.offset_y + e.y, oz + z, e);
		}
	return true;
}

bool place_schem_file_anchored(world_editor::WorldEditor &editor,
		const std::filesystem::path &file, int ox, int oy, int oz, unsigned rotation,
		SchemAnchor anchor)
{
	std::ifstream in(file, std::ios::binary);
	if (!in)
		return false;
	std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)), {});
	SchemDocument doc;
	try {
		doc = decode_sponge_schem(bytes);
	} catch (...) {
		return false;
	}
	if (anchor == SchemAnchor::Centered) {
		doc.offset_x -= doc.width / 2;
		doc.offset_z -= doc.length / 2;
	} else if (anchor == SchemAnchor::BaseCentroid) {
		long long sx = 0, sz = 0;
		std::size_t n = 0;
		for (const auto &v : doc.voxels)
			if (v.y == 0) {
				sx += v.x;
				sz += v.z;
				++n;
			}
		if (n) {
			doc.offset_x -= int(sx / static_cast<long long>(n));
			doc.offset_z -= int(sz / static_cast<long long>(n));
		}
	}
	for (const auto &v : doc.voxels) {
		BlockWithProperties b{resolve_schem_block(v.block),
				rotate_schem_properties(v.properties, rotation & 3)};
		if (b.block == block_definitions::AIR)
			continue;
		int x = v.x + doc.offset_x, z = v.z + doc.offset_z;
		switch (rotation & 3) {
		case 1:
			std::swap(x, z);
			x = -x;
			break;
		case 2:
			x = -x;
			z = -z;
			break;
		case 3:
			std::swap(x, z);
			z = -z;
			break;
		}
		editor.set_block_with_properties_absolute(
				std::move(b), ox + x, oy + doc.offset_y + v.y, oz + z, nullptr, nullptr);
	}
	if (editor.schem_entity_sink)
		for (const auto &e : doc.entities) {
			int x = e.x + doc.offset_x, z = e.z + doc.offset_z;
			switch (rotation & 3) {
			case 1:
				std::swap(x, z);
				x = -x;
				break;
			case 2:
				x = -x;
				z = -z;
				break;
			case 3:
				std::swap(x, z);
				z = -z;
				break;
			}
			place_native_entity(editor, ox + x, oy + doc.offset_y + e.y, oz + z, e);
		}
	return true;
}

bool place_named_schem(world_editor::WorldEditor &editor, const std::string &name, int ox,
		int oy, int oz, unsigned rotation, const Block *ground)
{
	const auto file = std::filesystem::path(__FILE__).parent_path().parent_path() /
					  ("assets/structures/" + name + ".schem");
	return place_schem_file_rotated(editor, file, ox, oy, oz, rotation, ground);
}
}
