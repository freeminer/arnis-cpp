#include "one_world.h"
#include "args.h"
#include "world_utils.h"
#include <nlohmann/json.hpp>
#include <fstream>
namespace arnis::one_world
{
using json = nlohmann::json;
std::filesystem::path Manifest::path_in(const std::filesystem::path &w)
{
	return w / MANIFEST_FILE;
}
std::optional<Manifest> Manifest::load(const std::filesystem::path &w, std::string *error)
{
	const auto p = path_in(w);
	if (!std::filesystem::is_regular_file(p))
		return std::nullopt;
	try {
		std::ifstream in(p);
		json j;
		in >> j;
		Manifest m;
		m.version = j.value("version", 0u);
		if (m.version > MANIFEST_VERSION)
			throw std::runtime_error("manifest version is newer");
		m.created_with = j.value("created_with", "");
		m.created_at = j.value("created_at", 0ull);
		m.origin_lat = j.value("origin_lat", 0.);
		m.origin_lon = j.value("origin_lon", 0.);
		m.scale = j.value("scale", 1.);
		m.ground_level = j.value("ground_level", 0);
		m.terrain = j.value("terrain", true);
		m.disable_height_limit = j.value("disable_height_limit", false);
		m.aws_only_elevation = j.value("aws_only_elevation", false);
		m.height_multiplier = j.value("height_multiplier", 1.);
		if (j.contains("elevation") && !j["elevation"].is_null()) {
			ElevationAffine e;
			const auto &ej = j.at("elevation");
			e.min_height_m = ej.value("min_height_m", 0.);
			e.blocks_per_meter = ej.value("blocks_per_meter", 0.);
			if (ej.contains("soft_top") && !ej["soft_top"].is_null())
				e.soft_top = SoftTop{ej["soft_top"].value("knee_m", 0.),
						ej["soft_top"].value("width_blocks", 0.)};
			m.elevation = e;
		}
		m.next_area_id = j.value("next_area_id", 1u);
		for (const auto &x : j.value("areas", json::array())) {
			GeneratedArea a;
			a.id = x.value("id", 0u);
			a.generated_at = x.value("generated_at", 0ull);
			a.arnis_version = x.value("arnis_version", "");
			a.min_x = x.at("min_x");
			a.min_z = x.at("min_z");
			a.max_x = x.at("max_x");
			a.max_z = x.at("max_z");
			a.min_lat = x.at("min_lat");
			a.min_lon = x.at("min_lon");
			a.max_lat = x.at("max_lat");
			a.max_lon = x.at("max_lon");
			if (x.contains("preview") && !x["preview"].is_null())
				a.preview = x.at("preview").get<std::string>();
			if (!a.contains(a.min_x, a.min_z, a.max_x, a.max_z))
				throw std::runtime_error("invalid generated area");
			m.areas.push_back(std::move(a));
		}
		std::string validation;
		if (!m.valid(&validation))
			throw std::runtime_error("invalid manifest: " + validation);
		return m;
	} catch (const std::exception &e) {
		if (error)
			*error = e.what();
		return std::nullopt;
	}
}
bool Manifest::save(const std::filesystem::path &w, std::string *error) const
{
	json j = {{"version", version}, {"created_with", created_with},
			{"created_at", created_at}, {"origin_lat", origin_lat},
			{"origin_lon", origin_lon}, {"scale", scale}, {"ground_level", ground_level},
			{"terrain", terrain}, {"disable_height_limit", disable_height_limit},
			{"aws_only_elevation", aws_only_elevation},
			{"height_multiplier", height_multiplier}, {"next_area_id", next_area_id},
			{"areas", json::array()}};
	if (elevation) {
		json e = {{"min_height_m", elevation->min_height_m},
				{"blocks_per_meter", elevation->blocks_per_meter}};
		if (elevation->soft_top)
			e["soft_top"] = {{"knee_m", elevation->soft_top->knee_m},
					{"width_blocks", elevation->soft_top->width_blocks}};
		j["elevation"] = std::move(e);
	}
	for (const auto &a : areas)
		j["areas"].push_back({{"id", a.id}, {"generated_at", a.generated_at},
				{"arnis_version", a.arnis_version}, {"min_x", a.min_x},
				{"min_z", a.min_z}, {"max_x", a.max_x}, {"max_z", a.max_z},
				{"min_lat", a.min_lat}, {"min_lon", a.min_lon}, {"max_lat", a.max_lat},
				{"max_lon", a.max_lon}, {"preview", a.preview}});
	const auto text = j.dump(2);
	const std::vector<std::uint8_t> bytes(text.begin(), text.end());
	if (!world_utils::replace_file_atomically(path_in(w), bytes)) {
		if (error)
			*error = "failed to write manifest";
		return false;
	}
	return true;
}
std::optional<std::array<int, 4>> Manifest::extent() const
{
	if (areas.empty())
		return std::nullopt;
	std::array<int, 4> e{areas.front().min_x, areas.front().min_z, areas.front().max_x,
			areas.front().max_z};
	for (const auto &a : areas) {
		e[0] = std::min(e[0], a.min_x);
		e[1] = std::min(e[1], a.min_z);
		e[2] = std::max(e[2], a.max_x);
		e[3] = std::max(e[3], a.max_z);
	}
	return e;
}
bool Manifest::valid(std::string *error) const
{
	if (!std::isfinite(origin_lat) || std::abs(origin_lat) > 85.0 ||
			!std::isfinite(origin_lon) || std::abs(origin_lon) > 180.0 ||
			!valid_scale(scale) || !std::isfinite(height_multiplier) ||
			height_multiplier <= 0.0 ||
			(elevation &&
					(!std::isfinite(elevation->min_height_m) ||
							elevation->blocks_per_meter < 0.0 ||
							!std::isfinite(elevation->blocks_per_meter) ||
							elevation->soft_top &&
									(!std::isfinite(elevation->soft_top->knee_m) ||
											!std::isfinite(
													elevation->soft_top->width_blocks) ||
											elevation->soft_top->width_blocks <= 0.0)))) {
		if (error)
			*error = "invalid world frame";
		return false;
	}
	for (const auto &a : areas)
		if (a.min_x > a.max_x || a.min_z > a.max_z) {
			if (error)
				*error = "generated area has an empty rectangle";
			return false;
		}
	return true;
}
std::uint64_t existing_chunks(
		const std::filesystem::path &world, int min_x, int min_z, int max_x, int max_z)
{
	auto div16 = [](int v) { return v >= 0 ? v / 16 : -((-v + 15) / 16); };
	const int cx0 = div16(min_x), cz0 = div16(min_z), cx1 = div16(max_x),
			  cz1 = div16(max_z);
	auto div32 = [](int v) { return v >= 0 ? v / 32 : -((-v + 31) / 32); };
	std::uint64_t count = 0;
	for (int rz = div32(cz0); rz <= div32(cz1); ++rz)
		for (int rx = div32(cx0); rx <= div32(cx1); ++rx) {
			std::ifstream f(world / "region" /
									("r." + std::to_string(rx) + "." +
											std::to_string(rz) + ".mca"),
					std::ios::binary);
			std::array<unsigned char, 4096> h{};
			if (!f || !f.read(reinterpret_cast<char *>(h.data()), h.size()))
				continue;
			for (int lz = 0; lz < 32; ++lz)
				for (int lx = 0; lx < 32; ++lx) {
					const int cx = rx * 32 + lx, cz = rz * 32 + lz;
					if (cx < cx0 || cx > cx1 || cz < cz0 || cz > cz1)
						continue;
					const auto i = 4u * unsigned(lx + lz * 32);
					if (h[i] || h[i + 1] || h[i + 2] || h[i + 3])
						++count;
				}
		}
	return count;
}
std::optional<std::filesystem::path> safe_preview_path(
		const std::filesystem::path &world, const std::string &relative)
{
	const std::filesystem::path p(relative), prefix(PREVIEW_DIR);
	if (p.is_absolute() || p.extension() != ".png")
		return std::nullopt;
	const auto normalized = p.lexically_normal();
	if (normalized != prefix && normalized.string().rfind(prefix.string() + "/", 0) != 0)
		return std::nullopt;
	const auto rel = normalized.lexically_relative(prefix);
	if (rel.empty() || rel.has_parent_path() || rel.filename() != rel)
		return std::nullopt;
	return world / prefix / rel;
}
}
