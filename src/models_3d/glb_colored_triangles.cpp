#include "voxelize.h"
#include <tiny_gltf.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
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
	if (img.bits != 8 || img.component < 3 || img.image.empty())
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
std::optional<std::array<float, 3>> color_at(
		const tinygltf::Model &m, int accessor, std::size_t index)
{
	if (accessor < 0 || std::size_t(accessor) >= m.accessors.size())
		return std::nullopt;
	const auto &a = m.accessors[accessor];
	if ((a.type != TINYGLTF_TYPE_VEC3 && a.type != TINYGLTF_TYPE_VEC4) ||
			a.bufferView < 0 || index >= a.count ||
			std::size_t(a.bufferView) >= m.bufferViews.size())
		return std::nullopt;
	const auto &v = m.bufferViews[a.bufferView];
	if (v.buffer < 0 || std::size_t(v.buffer) >= m.buffers.size())
		return std::nullopt;
	const std::size_t cs = component_size(a.componentType);
	const int raw = a.ByteStride(v);
	if (!cs || raw < int(cs * 3))
		return std::nullopt;
	const auto &b = m.buffers[v.buffer];
	const std::size_t off = v.byteOffset + a.byteOffset + index * std::size_t(raw);
	if (off + cs * 3 > b.data.size())
		return std::nullopt;
	return std::array<float, 3>{
			component_value(b.data.data() + off, a.componentType, a.normalized),
			component_value(b.data.data() + off + cs, a.componentType, a.normalized),
			component_value(b.data.data() + off + cs * 2, a.componentType, a.normalized)};
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
			if (a.bufferView < 0 ||
					std::size_t(a.bufferView) >= model.bufferViews.size() ||
					a.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT ||
					a.type != TINYGLTF_TYPE_VEC3)
				continue;
			const auto &v = model.bufferViews[a.bufferView];
			if (v.buffer < 0 || std::size_t(v.buffer) >= model.buffers.size())
				continue;
			const auto &b = model.buffers[v.buffer];
			const std::size_t po = v.byteOffset + a.byteOffset;
			const int raw = a.ByteStride(v);
			if (raw < int(3 * sizeof(float)))
				continue;
			auto position = [&](std::size_t i, std::array<float, 3> &p) {
				const auto off = po + i * std::size_t(raw);
				if (i >= a.count || off + 3 * sizeof(float) > b.data.size())
					return false;
				std::memcpy(p.data(), b.data.data() + off, 3 * sizeof(float));
				p = transform_point(world, p);
				return true;
			};
			std::vector<std::uint32_t> indices;
			if (prim.indices >= 0 && std::size_t(prim.indices) < model.accessors.size()) {
				const auto &ia = model.accessors[prim.indices];
				if (ia.type != TINYGLTF_TYPE_SCALAR || ia.bufferView < 0 ||
						std::size_t(ia.bufferView) >= model.bufferViews.size())
					continue;
				const auto &iv = model.bufferViews[ia.bufferView];
				if (iv.buffer < 0 || std::size_t(iv.buffer) >= model.buffers.size())
					continue;
				const auto &ib = model.buffers[iv.buffer];
				const auto size = component_size(ia.componentType);
				const int iraw = ia.ByteStride(iv);
				if (!size || iraw < int(size))
					continue;
				for (std::size_t i = 0; i < ia.count; ++i) {
					const auto off =
							iv.byteOffset + ia.byteOffset + i * std::size_t(iraw);
					if (off + size > ib.data.size()) {
						indices.clear();
						break;
					}
					float q = component_value(
							ib.data.data() + off, ia.componentType, false);
					indices.push_back(std::uint32_t(q));
				}
				if (indices.empty() && ia.count)
					continue;
			} else {
				indices.resize(a.count);
				for (std::size_t i = 0; i < a.count; ++i)
					indices[i] = std::uint32_t(i);
			}
			std::array<float, 3> material{1, 1, 1};
			bool uncolored = true;
			if (prim.material >= 0 &&
					std::size_t(prim.material) < model.materials.size()) {
				const auto &pbr = model.materials[prim.material].pbrMetallicRoughness;
				material = {float(pbr.baseColorFactor[0]), float(pbr.baseColorFactor[1]),
						float(pbr.baseColorFactor[2])};
				if (auto tex = image_average(model, pbr.baseColorTexture.index)) {
					for (int k = 0; k < 3; ++k)
						material[k] *= (*tex)[k];
					uncolored = false;
				}
				if (std::abs(material[0] - 1) > .001f ||
						std::abs(material[1] - 1) > .001f ||
						std::abs(material[2] - 1) > .001f)
					uncolored = false;
			}
			auto ci = prim.attributes.find("COLOR_0");
			for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
				ColoredTriangle t;
				if (!position(indices[i], t.vertices[0]) ||
						!position(indices[i + 1], t.vertices[1]) ||
						!position(indices[i + 2], t.vertices[2]))
					continue;
				t.color = material;
				t.uncolored = uncolored;
				if (ci != prim.attributes.end()) {
					auto c0 = color_at(model, ci->second, indices[i]),
						 c1 = color_at(model, ci->second, indices[i + 1]),
						 c2 = color_at(model, ci->second, indices[i + 2]);
					if (c0 && c1 && c2) {
						for (int k = 0; k < 3; ++k)
							t.color[k] =
									material[k] * ((*c0)[k] + (*c1)[k] + (*c2)[k]) / 3;
						t.uncolored = false;
					}
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
	if (roots.empty())
		for (std::size_t i = 0; i < model.nodes.size(); ++i)
			roots.push_back(int(i));
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
}
