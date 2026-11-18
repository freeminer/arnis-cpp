#include "index.h"

#include "../cache_root.h"
#include "fetch.h"

#include <zstd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <tuple>

namespace arnis::survey::index
{
namespace
{
std::string temporary_suffix()
{
	static std::atomic<std::uint64_t> serial{0};
	const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
	return ".tmp" + std::to_string(ticks) + "_" +
		   std::to_string(serial.fetch_add(1, std::memory_order_relaxed));
}

bool number(const nlohmann::json &v, double &out)
{
	if (!v.is_number())
		return false;
	out = v.get<double>();
	return std::isfinite(out);
}

void append_polygon(const nlohmann::json &polygon,
		std::vector<std::vector<std::pair<double, double>>> &rings)
{
	if (!polygon.is_array())
		return;
	for (const auto &ring : polygon) {
		if (!ring.is_array())
			continue;
		std::vector<std::pair<double, double>> points;
		for (const auto &coordinate : ring) {
			if (!coordinate.is_array() || coordinate.size() < 2)
				continue;
			double lon, lat;
			if (number(coordinate[0], lon) && number(coordinate[1], lat))
				points.emplace_back(lon, lat);
		}
		if (points.size() >= 3)
			rings.push_back(std::move(points));
	}
}

struct Candidate
{
	std::vector<bool> covered;
	std::uint16_t year = 0;
	double density = 0.0;
	std::vector<const Footprint *> files;
};
} // namespace

std::optional<Footprint> Footprint::from_geojson(const std::string &url,
		const std::string &survey, std::uint16_t year, const nlohmann::json &geometry,
		double point_count)
{
	if (!geometry.is_object() || !geometry.contains("type") ||
			!geometry["type"].is_string() || !geometry.contains("coordinates"))
		return std::nullopt;
	const auto type = geometry["type"].get<std::string>();
	const auto &coordinates = geometry["coordinates"];
	Footprint out;
	out.url = url;
	out.survey = survey;
	out.year = year;
	if (type == "Polygon") {
		append_polygon(coordinates, out.rings);
	} else if (type == "MultiPolygon" && coordinates.is_array()) {
		for (const auto &polygon : coordinates)
			append_polygon(polygon, out.rings);
	} else {
		return std::nullopt;
	}
	if (out.rings.empty())
		return std::nullopt;

	const auto inf = std::numeric_limits<double>::infinity();
	out.bbox = {inf, inf, -inf, -inf};
	double signed_area = 0.0;
	for (const auto &ring : out.rings) {
		double mean_lat = 0.0, area2 = 0.0;
		for (const auto &[lon, lat] : ring)
			mean_lat += lat;
		mean_lat = mean_lat / ring.size() * (3.14159265358979323846 / 180.0);
		for (std::size_t i = 0; i < ring.size(); ++i) {
			const auto &[x0, y0] = ring[i];
			const auto &[x1, y1] = ring[(i + 1) % ring.size()];
			out.bbox[0] = std::min(out.bbox[0], x0);
			out.bbox[1] = std::min(out.bbox[1], y0);
			out.bbox[2] = std::max(out.bbox[2], x0);
			out.bbox[3] = std::max(out.bbox[3], y0);
			area2 += x0 * y1 - x1 * y0;
		}
		// Ring winding preserves holes as signed subtraction, as in the Rust
		// index parser. The absolute net area is used for point density.
		signed_area += area2 * std::cos(mean_lat) / 2.0;
	}
	const double area_m2 = std::abs(signed_area) * 111320.0 * 110574.0;
	out.density = area_m2 > 0.0 ? point_count / area_m2 : 0.0;
	return out;
}

bool Footprint::contains(double latitude, double longitude) const
{
	if (longitude < bbox[0] || longitude > bbox[2] || latitude < bbox[1] ||
			latitude > bbox[3])
		return false;
	bool inside = false;
	for (const auto &ring : rings) {
		if (ring.empty())
			continue;
		std::size_t j = ring.size() - 1;
		for (std::size_t i = 0; i < ring.size(); ++i) {
			const auto [xi, yi] = ring[i];
			const auto [xj, yj] = ring[j];
			if ((yi > latitude) != (yj > latitude) &&
					longitude < (xj - xi) * (latitude - yi) / (yj - yi) + xi)
				inside = !inside;
			j = i;
		}
	}
	return inside;
}

std::uint16_t survey_year(const std::string &name)
{
	for (std::size_t i = 0; i < name.size();) {
		if (!std::isdigit(static_cast<unsigned char>(name[i]))) {
			++i;
			continue;
		}
		const auto start = i;
		while (i < name.size() && std::isdigit(static_cast<unsigned char>(name[i])))
			++i;
		if (i - start == 4) {
			try {
				const auto year =
						static_cast<unsigned>(std::stoul(name.substr(start, 4)));
				if (year >= 1990 && year < 2100)
					return static_cast<std::uint16_t>(year);
			} catch (...) {
			}
		}
	}
	std::vector<std::string> tokens;
	std::size_t start = 0;
	for (std::size_t i = 0; i <= name.size(); ++i) {
		if (i != name.size() && name[i] != '_' && name[i] != '-' && name[i] != ' ')
			continue;
		tokens.push_back(name.substr(start, i - start));
		start = i + 1;
	}
	for (auto it = tokens.rbegin(); it != tokens.rend(); ++it) {
		if (it->size() != 3 || !std::isupper(static_cast<unsigned char>((*it)[0])) ||
				!std::isdigit(static_cast<unsigned char>((*it)[1])) ||
				!std::isdigit(static_cast<unsigned char>((*it)[2])))
			continue;
		return static_cast<std::uint16_t>(
				2000 + ((*it)[1] - '0') * 10 + ((*it)[2] - '0'));
	}
	return 0;
}

source::TileUrls choose(const std::vector<Footprint> &footprints,
		const source::Sheet &sheet, const source::Rect &rect)
{
	using Tile = std::pair<std::int32_t, std::int32_t>;
	source::TileUrls out;
	for (const auto &[te, tn] : sheet.tiles(rect)) {
		const double x0 = static_cast<double>(te) * 1000.0;
		const double y0 = static_cast<double>(tn) * 1000.0;
		const double a0 = std::max(x0, rect[0]), b0 = std::max(y0, rect[1]);
		const double a1 = std::min(x0 + 1000.0, rect[2]);
		const double b1 = std::min(y0 + 1000.0, rect[3]);
		std::array<std::pair<double, double>, 25> probes;
		std::size_t p = 0;
		for (int i = 0; i < 5; ++i)
			for (int j = 0; j < 5; ++j)
				probes[p++] = sheet.projection.inverse(a0 + (a1 - a0) * (0.1 + 0.2 * i),
						b0 + (b1 - b0) * (0.1 + 0.2 * j));
		const std::array corners{sheet.projection.inverse(a0, b0),
				sheet.projection.inverse(a0, b1), sheet.projection.inverse(a1, b0),
				sheet.projection.inverse(a1, b1)};
		double south = std::numeric_limits<double>::max();
		double west = std::numeric_limits<double>::max();
		double north = std::numeric_limits<double>::lowest();
		double east = std::numeric_limits<double>::lowest();
		for (const auto &[lat, lon] : corners) {
			south = std::min(south, lat);
			north = std::max(north, lat);
			west = std::min(west, lon);
			east = std::max(east, lon);
		}
		std::map<std::string, Candidate> surveys;
		for (const auto &foot : footprints) {
			if (!(foot.bbox[0] < east && foot.bbox[2] > west && foot.bbox[1] < north &&
						foot.bbox[3] > south))
				continue;
			auto [it, inserted] = surveys.try_emplace(foot.survey);
			auto &candidate = it->second;
			if (inserted) {
				candidate.covered.assign(probes.size(), false);
				candidate.year = foot.year;
				candidate.density = foot.density;
			}
			for (std::size_t k = 0; k < probes.size(); ++k)
				candidate.covered[k] = candidate.covered[k] ||
									   foot.contains(probes[k].first, probes[k].second);
			candidate.files.push_back(&foot);
		}
		auto coverage = [](const Candidate &candidate) {
			return static_cast<std::size_t>(
					std::count(candidate.covered.begin(), candidate.covered.end(), true));
		};
		std::size_t most = 0;
		for (const auto &[_, candidate] : surveys)
			most = std::max(most, coverage(candidate));
		if (!most)
			continue;
		const Candidate *best = nullptr;
		for (const auto &[_, candidate] : surveys) {
			const auto count = coverage(candidate);
			if (4 * count < 3 * most)
				continue;
			// Rust's max_by keeps the later candidate on an equal comparison.
			// Treat incomparable densities (NaN) as equal, matching partial_cmp's
			// Equal fallback in the source implementation.
			if (!best || !(std::tie(candidate.year, candidate.density) <
								 std::tie(best->year, best->density)))
				best = &candidate;
		}
		if (best)
			for (const auto *foot : best->files)
				out.emplace_back(Tile{te, tn}, foot->url);
	}
	return out;
}

std::variant<nlohmann::json, std::string> cached_geojson(
		const std::string &provider, const std::string &url, std::uint64_t max_age_days)
{
	const auto path =
			arnis::cache::base_directory() / "laser" / provider / "index.geojson.zst";
	auto read_cached = [&]() -> std::optional<nlohmann::json> {
		std::ifstream file(path, std::ios::binary);
		if (!file)
			return std::nullopt;
		std::vector<std::uint8_t> packed((std::istreambuf_iterator<char>(file)), {});
		const auto size = ZSTD_getFrameContentSize(packed.data(), packed.size());
		if (size == ZSTD_CONTENTSIZE_ERROR || size == ZSTD_CONTENTSIZE_UNKNOWN ||
				size > 256ULL * 1024 * 1024)
			return std::nullopt;
		std::string raw(static_cast<std::size_t>(size), '\0');
		const auto decoded =
				ZSTD_decompress(raw.data(), raw.size(), packed.data(), packed.size());
		if (ZSTD_isError(decoded) || decoded != raw.size())
			return std::nullopt;
		auto json = nlohmann::json::parse(raw, nullptr, false);
		if (json.is_discarded())
			return std::nullopt;
		return json;
	};

	std::error_code ec;
	const auto modified = std::filesystem::last_write_time(path, ec);
	if (!ec && std::filesystem::exists(path, ec) && !ec &&
			std::filesystem::file_time_type::clock::now() - modified <
					std::chrono::hours(24 * max_age_days)) {
		if (auto cached = read_cached())
			return std::move(*cached);
	}

	auto response = fetch::get_bytes(url);
	if (const auto *body = std::get_if<std::vector<std::uint8_t>>(&response)) {
		auto json = nlohmann::json::parse(body->begin(), body->end(), nullptr, false);
		if (json.is_discarded())
			return url + ": invalid JSON index";
		std::error_code create_error;
		std::filesystem::create_directories(path.parent_path(), create_error);
		if (!create_error) {
			const auto bound = ZSTD_compressBound(body->size());
			std::vector<std::uint8_t> packed(bound);
			const auto packed_size = ZSTD_compress(
					packed.data(), packed.size(), body->data(), body->size(), 9);
			if (!ZSTD_isError(packed_size)) {
				packed.resize(packed_size);
				const auto tmp = path.string() + temporary_suffix();
				std::ofstream file(tmp, std::ios::binary | std::ios::trunc);
				if (file) {
					file.write(
							reinterpret_cast<const char *>(packed.data()), packed.size());
					file.close();
					if (file) {
						std::error_code rename_error;
						std::filesystem::rename(tmp, path, rename_error);
					}
				}
			}
		}
		return json;
	}
	if (auto cached = read_cached())
		return std::move(*cached);
	return url + ": " + std::get<fetch::Failure>(response).to_string();
}
} // namespace arnis::survey::index
