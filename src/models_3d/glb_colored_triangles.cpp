#include "voxelize.h"
#include <tiny_gltf.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace arnis::models_3d
{
namespace
{
using Matrix = std::array<std::array<float, 4>, 4>; // glTF column-major.
constexpr Matrix identity{{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};

Matrix multiply(const Matrix &a, const Matrix &b)
{
	Matrix out{};
	for (int col = 0; col < 4; ++col)
		for (int row = 0; row < 4; ++row)
			for (int k = 0; k < 4; ++k)
				out[col][row] += a[k][row] * b[col][k];
	return out;
}
std::array<float, 3> transform_point(const Matrix &m, std::array<float, 3> p)
{
	return {m[0][0] * p[0] + m[1][0] * p[1] + m[2][0] * p[2] + m[3][0],
			m[0][1] * p[0] + m[1][1] * p[1] + m[2][1] * p[2] + m[3][1],
			m[0][2] * p[0] + m[1][2] * p[1] + m[2][2] * p[2] + m[3][2]};
}
Matrix local_matrix(const tinygltf::Node &n)
{
	if (n.matrix.size() == 16) {
		Matrix m{};
		for (int col = 0; col < 4; ++col)
			for (int row = 0; row < 4; ++row)
				m[col][row] = float(n.matrix[col * 4 + row]);
		return m;
	}
	const float x = n.rotation.size() == 4 ? float(n.rotation[0]) : 0,
				y = n.rotation.size() == 4 ? float(n.rotation[1]) : 0,
				z = n.rotation.size() == 4 ? float(n.rotation[2]) : 0,
				w = n.rotation.size() == 4 ? float(n.rotation[3]) : 1;
	const float sx = n.scale.size() == 3 ? float(n.scale[0]) : 1,
				sy = n.scale.size() == 3 ? float(n.scale[1]) : 1,
				sz = n.scale.size() == 3 ? float(n.scale[2]) : 1;
	const float tx = n.translation.size() == 3 ? float(n.translation[0]) : 0,
				ty = n.translation.size() == 3 ? float(n.translation[1]) : 0,
				tz = n.translation.size() == 3 ? float(n.translation[2]) : 0;
	return {{{(1 - 2 * y * y - 2 * z * z) * sx, (2 * x * y + 2 * w * z) * sx,
					 (2 * x * z - 2 * w * y) * sx, 0},
			{(2 * x * y - 2 * w * z) * sy, (1 - 2 * x * x - 2 * z * z) * sy,
					(2 * y * z + 2 * w * x) * sy, 0},
			{(2 * x * z + 2 * w * y) * sz, (2 * y * z - 2 * w * x) * sz,
					(1 - 2 * x * x - 2 * y * y) * sz, 0},
			{tx, ty, tz, 1}}};
}
std::optional<std::array<float, 3>> image_average(const tinygltf::Model &m, int texture)
{
	if (texture < 0 || std::size_t(texture) >= m.textures.size())
		return std::nullopt;
	const int source = m.textures[texture].source;
	if (source < 0 || std::size_t(source) >= m.images.size())
		return std::nullopt;
	const auto &img = m.images[source];
	// Rust's glTF image helper accepts only RGB8/RGBA8. Reject other channel
	// layouts instead of interpreting their first three bytes as RGB.
	if (img.bits != 8 || (img.component != 3 && img.component != 4) || img.image.empty())
		return std::nullopt;
	std::uint64_t r = 0, g = 0, b = 0, count = 0;
	for (std::size_t p = 0; p + std::size_t(img.component) <= img.image.size();
			p += std::size_t(img.component)) {
		if (img.component >= 4 && img.image[p + 3] < 16)
			continue;
		r += img.image[p];
		g += img.image[p + 1];
		b += img.image[p + 2];
		++count;
	}
	if (!count)
		return std::nullopt;
	return std::array<float, 3>{
			float(r) / count / 255.f, float(g) / count / 255.f, float(b) / count / 255.f};
}
bool is_default_white(const std::array<float, 3> &color)
{
	return std::abs(color[0] - 1.0f) < 1e-3f && std::abs(color[1] - 1.0f) < 1e-3f &&
		   std::abs(color[2] - 1.0f) < 1e-3f;
}
std::size_t component_size(int type)
{
	switch (type) {
	case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
	case TINYGLTF_COMPONENT_TYPE_BYTE:
		return 1;
	case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
	case TINYGLTF_COMPONENT_TYPE_SHORT:
		return 2;
	case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
	case TINYGLTF_COMPONENT_TYPE_FLOAT:
		return 4;
	default:
		return 0;
	}
}
float component_value(const unsigned char *p, int type, bool normalized)
{
	switch (type) {
	case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
		return normalized ? *p / 255.f : *p;
	case TINYGLTF_COMPONENT_TYPE_BYTE: {
		std::int8_t v;
		std::memcpy(&v, p, 1);
		return normalized ? std::max(-1.f, v / 127.f) : v;
	}
	case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: {
		std::uint16_t v;
		std::memcpy(&v, p, 2);
		return normalized ? v / 65535.f : v;
	}
	case TINYGLTF_COMPONENT_TYPE_SHORT: {
		std::int16_t v;
		std::memcpy(&v, p, 2);
		return normalized ? std::max(-1.f, v / 32767.f) : v;
	}
	case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT: {
		std::uint32_t v;
		std::memcpy(&v, p, 4);
		return normalized ? float(double(v) / 4294967295.) : float(v);
	}
	case TINYGLTF_COMPONENT_TYPE_FLOAT: {
		float v;
		std::memcpy(&v, p, 4);
		return v;
	}
	default:
		return 1;
	}
}
bool index_value(const unsigned char *p, int type, std::uint32_t &out)
{
	switch (type) {
	case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
		out = *p;
		return true;
	case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: {
		std::uint16_t value;
		std::memcpy(&value, p, sizeof(value));
		out = value;
		return true;
	}
	case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
		std::memcpy(&out, p, sizeof(out));
		return true;
	default:
		return false;
	}
}

bool element_offset(std::size_t view_offset, std::size_t accessor_offset,
		std::size_t ordinal, std::size_t stride, std::size_t &out)
{
	const auto max = std::numeric_limits<std::size_t>::max();
	if (stride == 0 || accessor_offset > max - view_offset)
		return false;
	const auto base = view_offset + accessor_offset;
	if (ordinal > (max - base) / stride)
		return false;
	out = base + ordinal * stride;
	return true;
}

bool sparse_index(const tinygltf::Model &m, int view_index, std::size_t byte_offset,
		int component_type, std::size_t ordinal, std::uint32_t &out)
{
	if (view_index < 0 || std::size_t(view_index) >= m.bufferViews.size())
		return false;
	const auto &view = m.bufferViews[view_index];
	if (view.buffer < 0 || std::size_t(view.buffer) >= m.buffers.size())
		return false;
	const auto size = component_size(component_type);
	std::size_t offset = 0;
	if (!size || !element_offset(view.byteOffset, byte_offset, ordinal, size, offset))
		return false;
	const auto &data = m.buffers[view.buffer].data;
	return offset <= data.size() && size <= data.size() - offset &&
		   index_value(data.data() + offset, component_type, out);
}

bool sparse_value_offset(const tinygltf::Model &m, int view_index,
		std::size_t byte_offset, std::size_t ordinal, std::size_t element_size,
		const unsigned char *&out)
{
	if (view_index < 0 || std::size_t(view_index) >= m.bufferViews.size() ||
			element_size == 0)
		return false;
	const auto &view = m.bufferViews[view_index];
	std::size_t offset = 0;
	if (view.buffer < 0 || std::size_t(view.buffer) >= m.buffers.size() ||
			!element_offset(view.byteOffset, byte_offset, ordinal, element_size, offset))
		return false;
	const auto &data = m.buffers[view.buffer].data;
	if (offset > data.size() || element_size > data.size() - offset)
		return false;
	out = data.data() + offset;
	return true;
}

std::optional<std::vector<std::array<float, 3>>> read_positions(
		const tinygltf::Model &m, const tinygltf::Accessor &a)
{
	if (a.type != TINYGLTF_TYPE_VEC3 || a.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT)
		return std::nullopt;
	std::vector<std::array<float, 3>> positions(a.count, {0.f, 0.f, 0.f});
	if (a.bufferView >= 0) {
		if (std::size_t(a.bufferView) >= m.bufferViews.size())
			return std::nullopt;
		const auto &view = m.bufferViews[a.bufferView];
		if (view.buffer < 0 || std::size_t(view.buffer) >= m.buffers.size())
			return std::nullopt;
		const auto &data = m.buffers[view.buffer].data;
		const int stride = a.ByteStride(view);
		if (stride < int(3 * sizeof(float)) || a.sparse.count < 0)
			return std::nullopt;
		for (std::size_t i = 0; i < a.count; ++i) {
			std::size_t offset = 0;
			if (!element_offset(
						view.byteOffset, a.byteOffset, i, std::size_t(stride), offset))
				return std::nullopt;
			if (offset > data.size() || 3 * sizeof(float) > data.size() - offset)
				return std::nullopt;
			std::memcpy(positions[i].data(), data.data() + offset, 3 * sizeof(float));
		}
	} else if (!a.sparse.isSparse) {
		return std::nullopt;
	}
	if (a.sparse.count < 0)
		return std::nullopt;
	if (a.sparse.isSparse) {
		const auto value_size = 3 * sizeof(float);
		for (int i = 0; i < a.sparse.count; ++i) {
			std::uint32_t index = 0;
			const unsigned char *value = nullptr;
			if (!sparse_index(m, a.sparse.indices.bufferView, a.sparse.indices.byteOffset,
						a.sparse.indices.componentType, std::size_t(i), index) ||
					index >= positions.size() ||
					!sparse_value_offset(m, a.sparse.values.bufferView,
							a.sparse.values.byteOffset, std::size_t(i), value_size,
							value))
				return std::nullopt;
			std::memcpy(positions[index].data(), value, value_size);
		}
	}
	return positions;
}

std::optional<std::vector<std::uint32_t>> read_indices(
		const tinygltf::Model &m, const tinygltf::Accessor &a)
{
	if (a.type != TINYGLTF_TYPE_SCALAR ||
			(a.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE &&
					a.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT &&
					a.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT))
		return std::nullopt;
	std::vector<std::uint32_t> indices(a.count, 0);
	const auto size = component_size(a.componentType);
	if (a.bufferView >= 0) {
		if (std::size_t(a.bufferView) >= m.bufferViews.size())
			return std::nullopt;
		const auto &view = m.bufferViews[a.bufferView];
		if (view.buffer < 0 || std::size_t(view.buffer) >= m.buffers.size())
			return std::nullopt;
		const auto &data = m.buffers[view.buffer].data;
		const int stride = a.ByteStride(view);
		if (stride < int(size) || a.sparse.count < 0)
			return std::nullopt;
		for (std::size_t i = 0; i < a.count; ++i) {
			std::size_t offset = 0;
			if (!element_offset(
						view.byteOffset, a.byteOffset, i, std::size_t(stride), offset))
				return std::nullopt;
			if (offset > data.size() || size > data.size() - offset ||
					!index_value(data.data() + offset, a.componentType, indices[i]))
				return std::nullopt;
		}
	} else if (!a.sparse.isSparse) {
		return std::nullopt;
	}
	if (a.sparse.count < 0)
		return std::nullopt;
	if (a.sparse.isSparse)
		for (int i = 0; i < a.sparse.count; ++i) {
			std::uint32_t index = 0, value = 0;
			const unsigned char *value_ptr = nullptr;
			if (!sparse_index(m, a.sparse.indices.bufferView, a.sparse.indices.byteOffset,
						a.sparse.indices.componentType, std::size_t(i), index) ||
					index >= indices.size() ||
					!sparse_value_offset(m, a.sparse.values.bufferView,
							a.sparse.values.byteOffset, std::size_t(i), size,
							value_ptr) ||
					!index_value(value_ptr, a.componentType, value))
				return std::nullopt;
			indices[index] = value;
		}
	return indices;
}

std::optional<std::vector<std::array<float, 4>>> read_colors(
		const tinygltf::Model &m, const tinygltf::Accessor &a)
{
	if (a.type != TINYGLTF_TYPE_VEC3 && a.type != TINYGLTF_TYPE_VEC4)
		return std::nullopt;
	const auto size = component_size(a.componentType);
	const auto components = a.type == TINYGLTF_TYPE_VEC4 ? 4u : 3u;
	const auto element_size = size * components;
	if (!size || !element_size)
		return std::nullopt;
	std::vector<std::array<float, 4>> colors(a.count, {1.f, 1.f, 1.f, 1.f});
	if (a.bufferView >= 0) {
		if (std::size_t(a.bufferView) >= m.bufferViews.size())
			return std::nullopt;
		const auto &view = m.bufferViews[a.bufferView];
		if (view.buffer < 0 || std::size_t(view.buffer) >= m.buffers.size())
			return std::nullopt;
		const auto &data = m.buffers[view.buffer].data;
		const int stride = a.ByteStride(view);
		if (stride < int(element_size) || a.sparse.count < 0)
			return std::nullopt;
		for (std::size_t i = 0; i < a.count; ++i) {
			std::size_t offset = 0;
			if (!element_offset(
						view.byteOffset, a.byteOffset, i, std::size_t(stride), offset))
				return std::nullopt;
			if (offset > data.size() || element_size > data.size() - offset)
				return std::nullopt;
			for (std::size_t c = 0; c < components; ++c)
				colors[i][c] = component_value(
						data.data() + offset + c * size, a.componentType, a.normalized);
			if (components == 3)
				colors[i][3] = 1.0f;
		}
	} else if (!a.sparse.isSparse) {
		return std::nullopt;
	}
	if (a.sparse.count < 0)
		return std::nullopt;
	if (a.sparse.isSparse)
		for (int i = 0; i < a.sparse.count; ++i) {
			std::uint32_t index = 0;
			const unsigned char *value = nullptr;
			if (!sparse_index(m, a.sparse.indices.bufferView, a.sparse.indices.byteOffset,
						a.sparse.indices.componentType, std::size_t(i), index) ||
					index >= colors.size() ||
					!sparse_value_offset(m, a.sparse.values.bufferView,
							a.sparse.values.byteOffset, std::size_t(i), element_size,
							value))
				return std::nullopt;
			for (std::size_t c = 0; c < components; ++c)
				colors[index][c] =
						component_value(value + c * size, a.componentType, a.normalized);
			if (components == 3)
				colors[index][3] = 1.0f;
		}
	return colors;
}

std::optional<std::vector<std::array<float, 2>>> read_texcoords(
		const tinygltf::Model &m, const tinygltf::Accessor &a)
{
	if (a.type != TINYGLTF_TYPE_VEC2)
		return std::nullopt;
	const auto size = component_size(a.componentType);
	const auto element_size = size * 2;
	if (!size)
		return std::nullopt;
	std::vector<std::array<float, 2>> uv(a.count, {0, 0});
	if (a.bufferView >= 0) {
		if (std::size_t(a.bufferView) >= m.bufferViews.size())
			return std::nullopt;
		const auto &view = m.bufferViews[a.bufferView];
		if (view.buffer < 0 || std::size_t(view.buffer) >= m.buffers.size())
			return std::nullopt;
		const auto &data = m.buffers[view.buffer].data;
		const int stride = a.ByteStride(view);
		if (stride < int(element_size) || a.sparse.count < 0)
			return std::nullopt;
		for (std::size_t i = 0; i < a.count; ++i) {
			std::size_t offset = 0;
			if (!element_offset(
						view.byteOffset, a.byteOffset, i, std::size_t(stride), offset) ||
					offset > data.size() || element_size > data.size() - offset)
				return std::nullopt;
			for (std::size_t c = 0; c < 2; ++c)
				uv[i][c] = component_value(
						data.data() + offset + c * size, a.componentType, a.normalized);
		}
	} else if (!a.sparse.isSparse) {
		return std::nullopt;
	}
	if (a.sparse.count < 0)
		return std::nullopt;
	if (a.sparse.isSparse)
		for (int i = 0; i < a.sparse.count; ++i) {
			std::uint32_t index = 0;
			const unsigned char *value = nullptr;
			if (!sparse_index(m, a.sparse.indices.bufferView, a.sparse.indices.byteOffset,
						a.sparse.indices.componentType, std::size_t(i), index) ||
					index >= uv.size() ||
					!sparse_value_offset(m, a.sparse.values.bufferView,
							a.sparse.values.byteOffset, std::size_t(i), element_size,
							value))
				return std::nullopt;
			for (std::size_t c = 0; c < 2; ++c)
				uv[index][c] =
						component_value(value + c * size, a.componentType, a.normalized);
		}
	return uv;
}

float srgb_to_linear(float c)
{
	return c <= .04045f ? c / 12.92f : std::pow((c + .055f) / 1.055f, 2.4f);
}

std::shared_ptr<const GlbMipTexture> make_mips(
		const tinygltf::Image &image, int wrap_s, int wrap_t, bool use_alpha)
{
	if (image.bits != 8 || (image.component != 3 && image.component != 4) ||
			image.width <= 0 || image.height <= 0)
		return {};
	const auto w = static_cast<std::size_t>(image.width);
	const auto h = static_cast<std::size_t>(image.height);
	const auto stride = static_cast<std::size_t>(image.component);
	if (w > std::numeric_limits<std::size_t>::max() / h ||
			w * h > image.image.size() / stride)
		return {};
	auto texture = std::make_shared<GlbMipTexture>();
	texture->wrap_s = wrap_s;
	texture->wrap_t = wrap_t;
	std::size_t factor = 1;
	while (std::max(w / factor, h / factor) > 512)
		factor *= 2;
	GlbMipLevel base;
	base.width = std::max<std::size_t>(1, w / factor);
	base.height = std::max<std::size_t>(1, h / factor);
	base.pixels.reserve(base.width * base.height);
	const float norm = 1.0f / static_cast<float>(factor * factor);
	for (std::size_t y = 0; y < base.height; ++y)
		for (std::size_t x = 0; x < base.width; ++x) {
			std::array<float, 4> sum{};
			for (std::size_t sy = y * factor; sy < (y + 1) * factor; ++sy)
				for (std::size_t sx = x * factor; sx < (x + 1) * factor; ++sx) {
					const auto p =
							(std::min(sy, h - 1) * w + std::min(sx, w - 1)) * stride;
					const float a = image.component == 4 && use_alpha
											? image.image[p + 3] / 255.0f
											: 1.0f;
					for (int c = 0; c < 3; ++c)
						sum[c] += srgb_to_linear(image.image[p + c] / 255.0f) * a;
					sum[3] += a;
				}
			for (float &v : sum)
				v *= norm;
			base.pixels.push_back(sum);
		}
	texture->levels.push_back(std::move(base));
	while (texture->levels.back().width > 1 || texture->levels.back().height > 1) {
		const auto &prev = texture->levels.back();
		GlbMipLevel next;
		next.width = std::max<std::size_t>(1, prev.width / 2);
		next.height = std::max<std::size_t>(1, prev.height / 2);
		next.pixels.reserve(next.width * next.height);
		for (std::size_t y = 0; y < next.height; ++y)
			for (std::size_t x = 0; x < next.width; ++x) {
				std::array<float, 4> sum{};
				for (const auto &[dx, dy] : {std::pair{0, 0}, {1, 0}, {0, 1}, {1, 1}}) {
					const auto sx = std::min(2 * x + dx, prev.width - 1);
					const auto sy = std::min(2 * y + dy, prev.height - 1);
					const auto &px = prev.pixels[sy * prev.width + sx];
					for (int c = 0; c < 4; ++c)
						sum[c] += px[c] * .25f;
				}
				next.pixels.push_back(sum);
			}
		texture->levels.push_back(std::move(next));
	}
	return texture;
}
}

std::vector<ColoredTriangle> glb_colored_triangles(const std::vector<std::uint8_t> &bytes)
{
	tinygltf::TinyGLTF loader;
	tinygltf::Model model;
	std::string err, warn;
	if (!loader.LoadBinaryFromMemory(&model, &err, &warn, bytes.data(), bytes.size()))
		throw std::runtime_error("GLB parse: " + err);
	std::vector<ColoredTriangle> out;
	auto append_mesh = [&](int mesh_id, const Matrix &world) {
		if (mesh_id < 0 || std::size_t(mesh_id) >= model.meshes.size())
			return;
		for (const auto &prim : model.meshes[mesh_id].primitives) {
			if (prim.mode != TINYGLTF_MODE_TRIANGLES)
				continue;
			auto pi = prim.attributes.find("POSITION");
			if (pi == prim.attributes.end() || pi->second < 0 ||
					std::size_t(pi->second) >= model.accessors.size())
				continue;
			const auto &a = model.accessors[pi->second];
			auto positions = read_positions(model, a);
			if (!positions)
				continue;
			std::vector<std::uint32_t> indices;
			if (prim.indices >= 0 && std::size_t(prim.indices) < model.accessors.size()) {
				const auto &ia = model.accessors[prim.indices];
				auto decoded = read_indices(model, ia);
				if (!decoded)
					continue;
				indices = std::move(*decoded);
			} else {
				indices.resize(positions->size());
				for (std::size_t i = 0; i < positions->size(); ++i)
					indices[i] = static_cast<std::uint32_t>(i);
			}
			std::array<float, 3> material{1, 1, 1};
			bool material_has_texture = false;
			if (prim.material >= 0 &&
					std::size_t(prim.material) < model.materials.size()) {
				const auto &pbr = model.materials[prim.material].pbrMetallicRoughness;
				material = {float(pbr.baseColorFactor[0]), float(pbr.baseColorFactor[1]),
						float(pbr.baseColorFactor[2])};
				if (auto tex = image_average(model, pbr.baseColorTexture.index)) {
					for (int k = 0; k < 3; ++k)
						material[k] *= (*tex)[k];
					material_has_texture = true;
				}
			}
			const bool uncolored = !material_has_texture && is_default_white(material);
			std::optional<std::vector<std::array<float, 4>>> vertex_colors;
			auto ci = prim.attributes.find("COLOR_0");
			if (ci != prim.attributes.end() && ci->second >= 0 &&
					std::size_t(ci->second) < model.accessors.size())
				vertex_colors = read_colors(model, model.accessors[ci->second]);
			for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
				const auto ia = indices[i], ib = indices[i + 1], ic = indices[i + 2];
				if (ia >= positions->size() || ib >= positions->size() ||
						ic >= positions->size())
					continue;
				ColoredTriangle t;
				for (int vertex = 0; vertex < 3; ++vertex)
					t.vertices[vertex] =
							transform_point(world, (*positions)[vertex == 0	  ? ia
																: vertex == 1 ? ib
																			  : ic]);
				t.color = material;
				t.uncolored = uncolored;
				if (vertex_colors && ia < vertex_colors->size() &&
						ib < vertex_colors->size() && ic < vertex_colors->size()) {
					const auto &c0 = (*vertex_colors)[ia];
					const auto &c1 = (*vertex_colors)[ib];
					const auto &c2 = (*vertex_colors)[ic];
					for (int k = 0; k < 3; ++k)
						t.color[k] = material[k] * (c0[k] + c1[k] + c2[k]) / 3;
					t.uncolored = false;
				}
				out.push_back(t);
			}
		}
	};
	std::vector<int> roots;
	if (model.defaultScene >= 0 && std::size_t(model.defaultScene) < model.scenes.size())
		roots = model.scenes[model.defaultScene].nodes;
	else
		for (const auto &s : model.scenes)
			roots.insert(roots.end(), s.nodes.begin(), s.nodes.end());
	std::vector<std::pair<int, Matrix>> stack;
	for (int n : roots)
		stack.push_back({n, identity});
	while (!stack.empty()) {
		auto [id, parent] = stack.back();
		stack.pop_back();
		if (id < 0 || std::size_t(id) >= model.nodes.size())
			continue;
		const auto &n = model.nodes[id];
		const auto world = multiply(parent, local_matrix(n));
		append_mesh(n.mesh, world);
		for (int child : n.children)
			stack.push_back({child, world});
	}
	return out;
}

std::vector<GlbPaintTriangle> glb_paint_triangles(const std::vector<std::uint8_t> &bytes)
{
	tinygltf::TinyGLTF loader;
	tinygltf::Model model;
	std::string err, warn;
	if (!loader.LoadBinaryFromMemory(&model, &err, &warn, bytes.data(), bytes.size()))
		throw std::runtime_error("GLB parse: " + err);
	std::vector<GlbPaintTriangle> out;
	const auto append_mesh = [&](int mesh_id, const Matrix &world) {
		if (mesh_id < 0 || std::size_t(mesh_id) >= model.meshes.size())
			return;
		for (const auto &primitive : model.meshes[mesh_id].primitives) {
			if (primitive.mode != TINYGLTF_MODE_TRIANGLES)
				continue;
			const auto pi = primitive.attributes.find("POSITION");
			if (pi == primitive.attributes.end() || pi->second < 0 ||
					std::size_t(pi->second) >= model.accessors.size())
				continue;
			auto positions = read_positions(model, model.accessors[pi->second]);
			if (!positions)
				continue;
			std::vector<std::uint32_t> indices;
			if (primitive.indices >= 0 &&
					std::size_t(primitive.indices) < model.accessors.size()) {
				auto decoded = read_indices(model, model.accessors[primitive.indices]);
				if (!decoded)
					continue;
				indices = std::move(*decoded);
			} else {
				indices.resize(positions->size());
				for (std::size_t i = 0; i < positions->size(); ++i)
					indices[i] = static_cast<std::uint32_t>(i);
			}

			GlbPaintTriangle base;
			std::shared_ptr<const GlbMipTexture> texture;
			int uv_set = 0;
			if (primitive.material >= 0 &&
					std::size_t(primitive.material) < model.materials.size()) {
				const auto &mat = model.materials[primitive.material];
				const auto &pbr = mat.pbrMetallicRoughness;
				for (int c = 0; c < 4; ++c)
					base.factor[c] = static_cast<float>(pbr.baseColorFactor[c]);
				if (mat.alphaMode == "MASK")
					base.alpha_cutoff = static_cast<float>(mat.alphaCutoff);
				else if (mat.alphaMode == "BLEND")
					base.alpha_cutoff = .25f;
				const auto &info = pbr.baseColorTexture;
				if (info.index >= 0 && std::size_t(info.index) < model.textures.size()) {
					const auto &tex = model.textures[info.index];
					uv_set = info.texCoord;
					int wrap_s = TINYGLTF_TEXTURE_WRAP_REPEAT;
					int wrap_t = TINYGLTF_TEXTURE_WRAP_REPEAT;
					if (tex.sampler >= 0 &&
							std::size_t(tex.sampler) < model.samplers.size()) {
						wrap_s = model.samplers[tex.sampler].wrapS;
						wrap_t = model.samplers[tex.sampler].wrapT;
					}
					if (tex.source >= 0 && std::size_t(tex.source) < model.images.size())
						texture = make_mips(model.images[tex.source], wrap_s, wrap_t,
								base.alpha_cutoff.has_value());
				}
			}
			base.texture = texture;
			base.uncolored = !texture && std::abs(base.factor[0] - 1.0f) < 1e-3f &&
							 std::abs(base.factor[1] - 1.0f) < 1e-3f &&
							 std::abs(base.factor[2] - 1.0f) < 1e-3f;
			std::optional<std::vector<std::array<float, 4>>> colors;
			if (const auto ci = primitive.attributes.find("COLOR_0");
					ci != primitive.attributes.end() && ci->second >= 0 &&
					std::size_t(ci->second) < model.accessors.size())
				colors = read_colors(model, model.accessors[ci->second]);
			std::optional<std::vector<std::array<float, 2>>> texcoords;
			const auto uv_attribute =
					primitive.attributes.find("TEXCOORD_" + std::to_string(uv_set));
			if (texture && uv_attribute != primitive.attributes.end() &&
					uv_attribute->second >= 0 &&
					std::size_t(uv_attribute->second) < model.accessors.size())
				texcoords = read_texcoords(model, model.accessors[uv_attribute->second]);

			for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
				const std::array<std::uint32_t, 3> ids{
						indices[i], indices[i + 1], indices[i + 2]};
				if (std::any_of(ids.begin(), ids.end(),
							[&](std::uint32_t id) { return id >= positions->size(); }))
					continue;
				GlbPaintTriangle triangle = base;
				bool valid_uv = texcoords.has_value();
				bool valid_color = colors.has_value();
				for (int vertex = 0; vertex < 3; ++vertex) {
					const auto index = ids[vertex];
					triangle.vertices[vertex] =
							transform_point(world, (*positions)[index]);
					if (valid_uv) {
						if (index >= texcoords->size())
							valid_uv = false;
						else
							triangle.uv[vertex] = (*texcoords)[index];
					}
					if (valid_color && index >= colors->size())
						valid_color = false;
				}
				triangle.has_uv = valid_uv;
				triangle.has_vertex_color = valid_color;
				if (valid_color)
					for (int c = 0; c < 4; ++c)
						triangle.vertex_color[c] =
								((*colors)[ids[0]][c] + (*colors)[ids[1]][c] +
										(*colors)[ids[2]][c]) /
								3.0f;
				out.push_back(std::move(triangle));
			}
		}
	};
	std::vector<int> roots;
	if (model.defaultScene >= 0 && std::size_t(model.defaultScene) < model.scenes.size())
		roots = model.scenes[model.defaultScene].nodes;
	else
		for (const auto &scene : model.scenes)
			roots.insert(roots.end(), scene.nodes.begin(), scene.nodes.end());
	std::vector<std::pair<int, Matrix>> stack;
	for (int node : roots)
		stack.push_back({node, identity});
	while (!stack.empty()) {
		auto [id, parent] = stack.back();
		stack.pop_back();
		if (id < 0 || std::size_t(id) >= model.nodes.size())
			continue;
		const auto &node = model.nodes[id];
		const auto world = multiply(parent, local_matrix(node));
		append_mesh(node.mesh, world);
		for (int child : node.children)
			stack.push_back({child, world});
	}
	return out;
}

std::pair<std::array<float, 3>, std::array<float, 3>> glb_vertex_bounds(
		const std::vector<std::uint8_t> &bytes)
{
	tinygltf::TinyGLTF loader;
	tinygltf::Model model;
	std::string err, warn;
	if (!loader.LoadBinaryFromMemory(&model, &err, &warn, bytes.data(), bytes.size()))
		throw std::runtime_error("GLB parse: " + err);
	std::array<float, 3> minimum{INFINITY, INFINITY, INFINITY};
	std::array<float, 3> maximum{-INFINITY, -INFINITY, -INFINITY};
	bool any = false;
	std::vector<int> roots;
	if (model.defaultScene >= 0 && std::size_t(model.defaultScene) < model.scenes.size())
		roots = model.scenes[model.defaultScene].nodes;
	else
		for (const auto &scene : model.scenes)
			roots.insert(roots.end(), scene.nodes.begin(), scene.nodes.end());
	std::vector<std::pair<int, Matrix>> stack;
	for (int node : roots)
		stack.push_back({node, identity});
	while (!stack.empty()) {
		auto [node_id, parent] = stack.back();
		stack.pop_back();
		if (node_id < 0 || std::size_t(node_id) >= model.nodes.size())
			continue;
		const auto &node = model.nodes[node_id];
		const auto world = multiply(parent, local_matrix(node));
		if (node.mesh >= 0 && std::size_t(node.mesh) < model.meshes.size())
			for (const auto &primitive : model.meshes[node.mesh].primitives) {
				if (primitive.mode != TINYGLTF_MODE_TRIANGLES)
					continue;
				const auto position = primitive.attributes.find("POSITION");
				if (position == primitive.attributes.end() || position->second < 0 ||
						std::size_t(position->second) >= model.accessors.size())
					continue;
				auto positions = read_positions(model, model.accessors[position->second]);
				if (!positions)
					continue;
				for (const auto &vertex : *positions) {
					std::array<float, 3> point = vertex;
					point = transform_point(world, point);
					for (std::size_t axis = 0; axis < 3; ++axis)
						if (std::isfinite(point[axis])) {
							minimum[axis] = std::min(minimum[axis], point[axis]);
							maximum[axis] = std::max(maximum[axis], point[axis]);
							any = true;
						}
				}
			}
		for (int child : node.children)
			stack.push_back({child, world});
	}
	if (!any)
		throw std::runtime_error("GLB has no finite vertices");
	return {minimum, maximum};
}
}
