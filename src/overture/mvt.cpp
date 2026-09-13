#include "mvt.h"

#include <cstring>
#include <limits>

namespace arnis::overture::mvt
{
namespace
{
struct Reader
{
	const std::vector<std::uint8_t> &data;
	std::size_t pos = 0;
	bool varint(std::uint64_t &out)
	{
		out = 0;
		for (unsigned shift = 0; shift < 64; shift += 7) {
			if (pos == data.size())
				return false;
			const auto byte = data[pos++];
			out |= std::uint64_t(byte & 127) << shift;
			if (!(byte & 128))
				return true;
		}
		return false;
	}
	bool bytes(std::vector<std::uint8_t> &out)
	{
		std::uint64_t n;
		if (!varint(n) || n > data.size() - pos)
			return false;
		out.assign(data.begin() + pos, data.begin() + pos + std::size_t(n));
		pos += std::size_t(n);
		return true;
	}
	bool skip(unsigned wire)
	{
		std::uint64_t n;
		if (wire == 0)
			return varint(n);
		if (wire == 1) {
			if (data.size() - pos < 8)
				return false;
			pos += 8;
			return true;
		}
		if (wire == 2) {
			std::vector<std::uint8_t> ignored;
			return bytes(ignored);
		}
		if (wire == 5) {
			if (data.size() - pos < 4)
				return false;
			pos += 4;
			return true;
		}
		return false;
	}
};
std::int64_t zigzag64(std::uint64_t value)
{
	return static_cast<std::int64_t>(
			(value >> 1) ^ -static_cast<std::int64_t>(value & 1));
}
std::int32_t zigzag(std::uint64_t value)
{
	return static_cast<std::int32_t>(zigzag64(value));
}
std::uint32_t little_u32(const std::uint8_t *data)
{
	return std::uint32_t(data[0]) | std::uint32_t(data[1]) << 8 |
		   std::uint32_t(data[2]) << 16 | std::uint32_t(data[3]) << 24;
}
std::uint64_t little_u64(const std::uint8_t *data)
{
	std::uint64_t value = 0;
	for (unsigned byte = 0; byte < 8; ++byte)
		value |= std::uint64_t(data[byte]) << (byte * 8);
	return value;
}
bool packed(const std::vector<std::uint8_t> &bytes, std::vector<std::uint32_t> &out)
{
	Reader r{bytes};
	std::uint64_t value;
	while (r.pos < bytes.size()) {
		if (!r.varint(value) || value > UINT32_MAX)
			return false;
		out.push_back(std::uint32_t(value));
	}
	return true;
}
void decode_geometry(const std::vector<std::uint32_t> &commands, Feature &feature)
{
	int x = 0, y = 0;
	std::size_t i = 0;
	Ring ring;
	auto finish = [&] {
		if (ring.points.size() >= 3) {
			std::int64_t area = 0;
			for (std::size_t p = 0; p < ring.points.size(); ++p) {
				auto a = ring.points[p], b = ring.points[(p + 1) % ring.points.size()];
				area += std::int64_t(a.first) * b.second -
						std::int64_t(b.first) * a.second;
			}
			ring.area2 = area;
			feature.rings.push_back(std::move(ring));
			ring = {};
		}
	};
	while (i < commands.size()) {
		const auto command = commands[i++];
		const auto id = command & 7, count = command >> 3;
		if (!count)
			return;
		if (id == 1 || id == 2)
			for (std::uint32_t n = 0; n < count && i + 1 < commands.size(); ++n) {
				x += zigzag(commands[i++]);
				y += zigzag(commands[i++]);
				if (id == 1)
					finish();
				ring.points.emplace_back(x, y);
			}
		else if (id == 7)
			finish();
		else
			return;
	}
	finish();
}
bool decode_value(const std::vector<std::uint8_t> &bytes, Value &value)
{
	Reader r{bytes};
	while (r.pos < bytes.size()) {
		std::uint64_t key;
		if (!r.varint(key))
			return false;
		const auto field = key >> 3, wire = key & 7;
		if (field == 1 && wire == 2) {
			std::vector<std::uint8_t> s;
			if (!r.bytes(s))
				return false;
			value = std::string(s.begin(), s.end());
			return true;
		}
		if (field == 2 && wire == 5) {
			if (r.pos + 4 > bytes.size())
				return false;
			const auto bits = little_u32(bytes.data() + r.pos);
			float decoded;
			std::memcpy(&decoded, &bits, sizeof(decoded));
			r.pos += sizeof(decoded);
			value = double(decoded);
			return true;
		}
		if (field == 3 && wire == 1) {
			if (r.pos + 8 > bytes.size())
				return false;
			const auto bits = little_u64(bytes.data() + r.pos);
			double decoded;
			std::memcpy(&decoded, &bits, sizeof(decoded));
			r.pos += sizeof(decoded);
			value = decoded;
			return true;
		}
		if ((field == 4 || field == 5 || field == 6) && wire == 0) {
			std::uint64_t v;
			if (!r.varint(v))
				return false;
			value = field == 4	 ? Value(std::int64_t(v))
					: field == 6 ? Value(zigzag64(v))
								 : Value(v);
			return true;
		}
		if (field == 7 && wire == 0) {
			std::uint64_t v;
			if (!r.varint(v))
				return false;
			value = bool(v);
			return true;
		}
		if (!r.skip(wire))
			return false;
	}
	return false;
}
bool decode_feature(const std::vector<std::uint8_t> &bytes, Feature &out)
{
	Reader r{bytes};
	std::vector<std::uint32_t> geometry;
	while (r.pos < bytes.size()) {
		std::uint64_t key;
		if (!r.varint(key))
			return false;
		auto field = key >> 3, wire = key & 7;
		if (field == 2 && wire == 2) {
			std::vector<std::uint8_t> b;
			if (!r.bytes(b) || !packed(b, out.tags))
				return false;
		} else if (field == 3 && wire == 0) {
			std::uint64_t v;
			if (!r.varint(v))
				return false;
			out.geom_type = v;
		} else if (field == 4 && wire == 2) {
			std::vector<std::uint8_t> b;
			if (!r.bytes(b) || !packed(b, geometry))
				return false;
		} else if (!r.skip(wire))
			return false;
	}
	decode_geometry(geometry, out);
	return true;
}
bool decode_layer(const std::vector<std::uint8_t> &bytes, Layer &out)
{
	Reader r{bytes};
	while (r.pos < bytes.size()) {
		std::uint64_t key;
		if (!r.varint(key))
			return false;
		auto field = key >> 3, wire = key & 7;
		std::vector<std::uint8_t> b;
		if ((field == 1 || field == 2 || field == 4) && wire == 2) {
			if (!r.bytes(b))
				return false;
			if (field == 1)
				out.name.assign(b.begin(), b.end());
			else if (field == 2) {
				Feature f;
				if (!decode_feature(b, f))
					return false;
				out.features.push_back(std::move(f));
			} else if (field == 4) {
				Value v;
				if (decode_value(b, v))
					out.values.push_back(std::move(v));
			}
		} else if (field == 3 && wire == 2) {
			if (!r.bytes(b))
				return false;
			out.keys.emplace_back(b.begin(), b.end());
		} else if (field == 5 && wire == 0) {
			std::uint64_t v;
			if (!r.varint(v))
				return false;
			out.extent = std::uint32_t(std::min<std::uint64_t>(v, UINT32_MAX));
		} else if (!r.skip(wire))
			return false;
	}
	return !out.name.empty();
}
}
std::optional<Value> Layer::attribute(
		const Feature &feature, const std::string &key) const
{
	for (std::size_t i = 0; i + 1 < feature.tags.size(); i += 2)
		if (feature.tags[i] < keys.size() && keys[feature.tags[i]] == key &&
				feature.tags[i + 1] < values.size())
			return values[feature.tags[i + 1]];
	return std::nullopt;
}
std::optional<std::vector<Layer>> decode(const std::vector<std::uint8_t> &data)
{
	if (data.size() > 64 * 1024 * 1024)
		return std::nullopt;
	Reader r{data};
	std::vector<Layer> out;
	while (r.pos < data.size()) {
		std::uint64_t key;
		if (!r.varint(key))
			return std::nullopt;
		if ((key >> 3) == 3 && (key & 7) == 2) {
			std::vector<std::uint8_t> b;
			if (!r.bytes(b))
				return std::nullopt;
			Layer l;
			if (!decode_layer(b, l))
				return std::nullopt;
			out.push_back(std::move(l));
		} else if (!r.skip(key & 7))
			return std::nullopt;
	}
	return out;
}
} // namespace arnis::overture::mvt
