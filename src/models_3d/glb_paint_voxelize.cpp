#include "voxelize.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace arnis::models_3d
{
namespace
{
using Vec2 = std::array<float, 2>;
using Vec3 = std::array<float, 3>;
using Position = std::array<int, 3>;

struct PositionHash
{
	std::size_t operator()(const Position &p) const noexcept
	{
		std::size_t h = std::hash<int>{}(p[0]);
		for (int v : {p[1], p[2]})
			h ^= std::hash<int>{}(v) + 0x9e3779b9u + (h << 6) + (h >> 2);
		return h;
	}
};

struct Accum
{
	std::array<float, 3> rgb{};
	float weight = 0, coverage = 0;
	std::uint16_t samples = 0;
	float occupancy = 0, glass = 0, uncolored = 0;
	void add(const Accum &s)
	{
		for (int i = 0; i < 3; ++i)
			rgb[i] += s.rgb[i];
		weight += s.weight;
		coverage += s.coverage;
		samples = static_cast<std::uint16_t>(std::min<unsigned>(
				std::numeric_limits<std::uint16_t>::max(), samples + s.samples));
		occupancy = std::max(occupancy, s.occupancy);
		glass += s.glass;
		uncolored += s.uncolored;
	}
};

Vec3 sub(Vec3 a, Vec3 b)
{
	return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
float dot(Vec3 a, Vec3 b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vec3 cross(Vec3 a, Vec3 b)
{
	return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
			a[0] * b[1] - a[1] * b[0]};
}
std::optional<Vec3> unit(Vec3 v)
{
	const float len = std::sqrt(dot(v, v));
	if (!(len > 1e-9f) || !std::isfinite(len))
		return std::nullopt;
	for (auto &x : v)
		x /= len;
	return v;
}
float linear_to_srgb(float c)
{
	c = std::clamp(c, 0.0f, 1.0f);
	return c <= .0031308f ? c * 12.92f : 1.055f * std::pow(c, 1.0f / 2.4f) - .055f;
}

struct Frame
{
	Vec3 p0{}, e1{}, e2{}, n_scaled{}, n{}, u{}, v{};
	static std::optional<Frame> make(const std::array<Vec3, 3> &t)
	{
		Frame f;
		f.p0 = t[0];
		f.e1 = sub(t[1], t[0]);
		f.e2 = sub(t[2], t[0]);
		const auto normal = cross(f.e1, f.e2);
		const float len2 = dot(normal, normal);
		if (!(len2 > 0) || !std::isfinite(len2))
			return std::nullopt;
		auto nn = unit(normal), uu = unit(f.e1);
		if (!nn || !uu)
			return std::nullopt;
		f.n = *nn;
		f.u = *uu;
		f.v = cross(f.n, f.u);
		f.n_scaled = {normal[0] / len2, normal[1] / len2, normal[2] / len2};
		return f;
	}
	std::array<float, 3> barycentric(Vec3 p) const
	{
		const auto d = sub(p, p0);
		const float b1 = dot(cross(d, e2), n_scaled);
		const float b2 = dot(cross(e1, d), n_scaled);
		return {1.0f - b1 - b2, b1, b2};
	}
};

std::size_t wrapped_index(long long i, std::size_t size, int mode)
{
	const auto n = static_cast<long long>(size);
	if (mode == 33071) // CLAMP_TO_EDGE
		return static_cast<std::size_t>(std::clamp(i, 0LL, n - 1));
	if (mode == 33648) { // MIRRORED_REPEAT
		const auto m = ((i % (2 * n)) + 2 * n) % (2 * n);
		return static_cast<std::size_t>(m < n ? m : 2 * n - 1 - m);
	}
	return static_cast<std::size_t>(((i % n) + n) % n); // REPEAT
}

std::array<float, 4> sample_texture(
		const GlbMipTexture &texture, Vec2 uv, float texels_per_voxel)
{
	const auto lod = static_cast<std::size_t>(
			std::max(0L, std::lround(std::log2(std::max(1.0f, texels_per_voxel)))));
	const auto &level = texture.levels[std::min(lod, texture.levels.size() - 1)];
	const float fx = uv[0] * level.width - .5f;
	const float fy = uv[1] * level.height - .5f;
	const float x0 = std::floor(fx), y0 = std::floor(fy);
	const float tx = fx - x0, ty = fy - y0;
	const auto at = [&](float x, float y) -> const std::array<float, 4> & {
		const auto ix =
				wrapped_index(static_cast<long long>(x), level.width, texture.wrap_s);
		const auto iy =
				wrapped_index(static_cast<long long>(y), level.height, texture.wrap_t);
		return level.pixels[iy * level.width + ix];
	};
	const auto &a = at(x0, y0);
	const auto &b = at(x0 + 1, y0);
	const auto &c = at(x0, y0 + 1);
	const auto &d = at(x0 + 1, y0 + 1);
	std::array<float, 4> out{};
	for (int i = 0; i < 4; ++i) {
		const float top = a[i] + (b[i] - a[i]) * tx;
		const float bottom = c[i] + (d[i] - c[i]) * tx;
		out[i] = top + (bottom - top) * ty;
	}
	return out;
}

float texel_density(const GlbPaintTriangle &t)
{
	if (!t.texture || !t.has_uv)
		return 1.0f;
	const auto &base = t.texture->levels.front();
	const float du1 = (t.uv[1][0] - t.uv[0][0]) * base.width;
	const float dv1 = (t.uv[1][1] - t.uv[0][1]) * base.height;
	const float du2 = (t.uv[2][0] - t.uv[0][0]) * base.width;
	const float dv2 = (t.uv[2][1] - t.uv[0][1]) * base.height;
	const float uv_area = std::abs(du1 * dv2 - du2 * dv1);
	const auto e1 = sub(t.vertices[1], t.vertices[0]);
	const auto e2 = sub(t.vertices[2], t.vertices[0]);
	const float world_area = std::sqrt(dot(cross(e1, e2), cross(e1, e2)));
	if (world_area <= 1e-6f || !std::isfinite(uv_area))
		return 1.0f;
	return std::sqrt(uv_area / world_area);
}

Accum shade_sample(
		const GlbPaintTriangle &t, const std::array<float, 3> &w, float texels_per_voxel)
{
	std::array<float, 3> linear{t.factor[0], t.factor[1], t.factor[2]};
	float alpha = t.factor[3];
	if (t.texture && t.has_uv) {
		const Vec2 uv{t.uv[0][0] * w[0] + t.uv[1][0] * w[1] + t.uv[2][0] * w[2],
				t.uv[0][1] * w[0] + t.uv[1][1] * w[1] + t.uv[2][1] * w[2]};
		const auto tex = sample_texture(*t.texture, uv, texels_per_voxel);
		if (tex[3] > 1e-4f)
			for (int i = 0; i < 3; ++i)
				linear[i] *= tex[i] / tex[3];
		alpha *= tex[3];
	}
	std::array<float, 3> rgb{};
	for (int i = 0; i < 3; ++i) {
		rgb[i] = linear_to_srgb(linear[i]);
		if (t.has_vertex_color)
			rgb[i] *= t.vertex_color[i];
	}
	if (t.has_vertex_color)
		alpha *= t.vertex_color[3];
	const float coverage =
			t.alpha_cutoff
					? std::min(1.0f, alpha / (2.0f * std::max(*t.alpha_cutoff, 1e-3f)))
					: 1.0f;
	const bool uncolored = t.uncolored && !t.has_vertex_color;
	const float weight = uncolored ? 0.0f : coverage;
	const bool glass = std::abs(rgb[0] - 1.0f) < 1e-3f && std::abs(rgb[1]) < 1e-3f &&
					   std::abs(rgb[2] - 1.0f) < 1e-3f;
	Accum a;
	for (int i = 0; i < 3; ++i)
		a.rgb[i] = rgb[i] * weight;
	a.weight = weight;
	a.coverage = coverage;
	a.samples = 1;
	a.glass = glass ? coverage : 0.0f;
	a.uncolored = uncolored ? coverage : 0.0f;
	return a;
}

Accum shade_footprint(const GlbPaintTriangle &t, const Frame &frame, Position cell,
		float texels_per_voxel)
{
	constexpr int steps = 6;
	constexpr float span = 1.5f;
	const Vec3 c{static_cast<float>(cell[0]), static_cast<float>(cell[1]),
			static_cast<float>(cell[2])};
	const float off = dot(sub(c, frame.p0), frame.n);
	const Vec3 q{
			c[0] - frame.n[0] * off, c[1] - frame.n[1] * off, c[2] - frame.n[2] * off};
	const float step = span / steps;
	std::optional<Accum> sum;
	for (int i = 0; i < steps; ++i)
		for (int j = 0; j < steps; ++j) {
			const float s = (i + .5f) * step - span / 2.0f;
			const float v = (j + .5f) * step - span / 2.0f;
			const Vec3 p{q[0] + frame.u[0] * s + frame.v[0] * v,
					q[1] + frame.u[1] * s + frame.v[1] * v,
					q[2] + frame.u[2] * s + frame.v[2] * v};
			if (std::abs(p[0] - c[0]) > .5f || std::abs(p[1] - c[1]) > .5f ||
					std::abs(p[2] - c[2]) > .5f)
				continue;
			const auto w = frame.barycentric(p);
			if (std::any_of(w.begin(), w.end(), [](float x) { return x < -1e-4f; }))
				continue;
			const auto sample = shade_sample(t, w, texels_per_voxel * step);
			if (!sum)
				sum = sample;
			else
				sum->add(sample);
		}
	if (sum)
		return *sum;
	auto w = frame.barycentric(q);
	for (auto &x : w)
		x = std::max(0.0f, x);
	const float total = std::max(1e-6f, w[0] + w[1] + w[2]);
	for (auto &x : w)
		x /= total;
	return shade_sample(t, w, texels_per_voxel);
}

} // namespace

std::vector<std::pair<std::array<int, 3>, VoxelPaint>> voxelize_glb_paint(
		const std::vector<std::uint8_t> &bytes, const WorldTransform &transform)
{
	std::unordered_map<Position, Accum, PositionHash> accum;
	for (auto triangle : glb_paint_triangles(bytes)) {
		for (auto &point : triangle.vertices)
			point = transform.apply(point);
		if (!std::all_of(triangle.vertices.begin(), triangle.vertices.end(),
					[](const Vec3 &p) {
						return std::all_of(p.begin(), p.end(),
								[](float x) { return std::isfinite(x); });
					}))
			continue;
		const auto frame =
				triangle.has_uv ? Frame::make(triangle.vertices) : std::nullopt;
		const float density = texel_density(triangle);
		visit_triangle_voxels(triangle.vertices, [&](Position p) {
			Accum sample = frame ? shade_footprint(triangle, *frame, p, density)
								 : shade_sample(triangle, {1.0f / 3, 1.0f / 3, 1.0f / 3},
										   density);
			sample.occupancy = sample.coverage / std::max<unsigned>(1, sample.samples);
			accum[p].add(sample);
		});
	}

	constexpr float rescue_min = .25f;
	const auto coverage = [&](const Position &p) -> std::optional<float> {
		const auto it = accum.find(p);
		return it == accum.end() ? std::nullopt
								 : std::optional<float>(it->second.occupancy);
	};
	std::unordered_set<Position, PositionHash> kept;
	for (const auto &[p, a] : accum) {
		const float c = a.occupancy;
		bool rescue = false;
		if (c >= rescue_min && c < .5f) {
			float best = 0;
			bool dominated = false;
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz) {
					if (!dx && !dz)
						continue;
					const Position q{p[0] + dx, p[1], p[2] + dz};
					const auto qcov = coverage(q);
					if (!qcov || *qcov >= .5f)
						continue;
					if (*qcov > c || (*qcov == c && q < p))
						dominated = true;
					best = std::max(best, *qcov);
				}
			rescue = !dominated && c + best >= .5f;
		}
		if (c >= .5f || rescue)
			kept.insert(p);
	}
	std::vector<Position> specks;
	for (const auto &p : kept) {
		const auto c = coverage(p);
		if (!c || *c >= .999f)
			continue;
		bool neighbour = false;
		for (int dx = -1; dx <= 1 && !neighbour; ++dx)
			for (int dy = -1; dy <= 1 && !neighbour; ++dy)
				for (int dz = -1; dz <= 1; ++dz)
					if ((dx || dy || dz) &&
							kept.contains({p[0] + dx, p[1] + dy, p[2] + dz})) {
						neighbour = true;
						break;
					}
		if (!neighbour)
			specks.push_back(p);
	}
	for (const auto &p : specks)
		kept.erase(p);

	std::vector<std::pair<Position, VoxelPaint>> output;
	output.reserve(kept.size());
	for (const auto &p : kept) {
		const auto &a = accum.at(p);
		const float total = std::max(a.coverage, 1e-6f);
		VoxelPaint paint;
		if (a.glass * 2 >= total) {
			paint.kind = VoxelPaint::Kind::Glass;
		} else if (a.weight <= 0 || a.uncolored * 2 > total) {
			paint.kind = VoxelPaint::Kind::Uncolored;
		} else {
			paint.kind = VoxelPaint::Kind::Rgb;
			for (int c = 0; c < 3; ++c)
				paint.rgb[c] = static_cast<std::uint8_t>(std::lround(
						std::clamp(a.rgb[c] / a.weight, 0.0f, 1.0f) * 255.0f));
		}
		output.emplace_back(p, paint);
	}
	return output;
}
} // namespace arnis::models_3d
