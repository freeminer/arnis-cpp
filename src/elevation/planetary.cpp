#include "planetary.h"

#include "../../../http.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <limits>
#include <thread>

namespace arnis::elevation
{
namespace
{
constexpr std::size_t MAX_SOURCE_DIM = 3072;

struct DemSpec
{
	const char *base_url;
	std::size_t ppd;
	double lat_span, lon_span, lat_min, lat_max, dn_to_meters;
	bool big_endian;

	std::size_t lines() const { return std::size_t(lat_span * ppd); }
	std::size_t samples() const { return std::size_t(lon_span * ppd); }
};

std::optional<DemSpec> spec_for(CelestialBody body)
{
	switch (body) {
	case CelestialBody::Mars:
		return DemSpec{"https://pds-geosciences.wustl.edu/mgs/"
					   "mgs-m-mola-5-megdr-l3-v1/mgsl_300x/meg128/",
				128, 44.0, 90.0, -88.0, 88.0, 1.0, true};
	case CelestialBody::Moon:
		return DemSpec{"https://pds-geosciences.wustl.edu/lro/"
					   "lro-l-lola-3-rdr-v1/lrolol_1xxx/data/lola_gdr/cylindrical/img/",
				128, 180.0, 360.0, -90.0, 90.0, .5, false};
	case CelestialBody::Earth:
		return std::nullopt;
	}
	return std::nullopt;
}

double east_longitude(double longitude)
{
	const auto wrapped = std::fmod(longitude, 360.0);
	return wrapped < 0.0 ? wrapped + 360.0 : wrapped;
}

std::string tile_name(
		CelestialBody body, const DemSpec &spec, double lat_band, double lon_band)
{
	if (body == CelestialBody::Moon)
		return "ldem_128.img";
	const auto max_lat = lat_band + spec.lat_span;
	const auto hemisphere = max_lat >= 0.0 ? 'n' : 's';
	char filename[32];
	std::snprintf(filename, sizeof(filename), "megt%02d%c%03dhb.img",
			static_cast<int>(std::abs(max_lat)), hemisphere, static_cast<int>(lon_band));
	return filename;
}

struct Window
{
	std::size_t line0, sample0, lines, samples, step, ppd;

	static Window plan(const DemSpec &spec, const geographic::LLBBox &bbox,
			std::size_t width, std::size_t height)
	{
		const auto ppd = double(spec.ppd);
		const auto west = east_longitude(bbox.min().lng());
		auto east = east_longitude(bbox.max().lng());
		if (east < west)
			east += 360.0;
		const auto line0 =
				std::size_t(std::max(0.0, std::floor((90.0 - bbox.max().lat()) * ppd)));
		const auto line1 = std::size_t(std::ceil((90.0 - bbox.min().lat()) * ppd));
		const auto sample0 = std::size_t(std::floor(west * ppd));
		const auto sample1 = std::size_t(std::ceil(east * ppd));
		const auto raw_lines = std::max<std::size_t>(1, line1 - line0);
		const auto raw_samples = std::max<std::size_t>(1, sample1 - sample0);
		const auto by_grid =
				std::max(double(raw_lines) / std::max<std::size_t>(1, height),
						double(raw_samples) / std::max<std::size_t>(1, width));
		const auto by_budget = double(std::max(raw_lines, raw_samples)) / MAX_SOURCE_DIM;
		const auto step = std::max<std::size_t>(
				1, std::size_t(std::ceil(std::max(by_grid, by_budget))));
		return {line0, sample0, (raw_lines + step - 1) / step,
				(raw_samples + step - 1) / step, step, spec.ppd};
	}
};

std::filesystem::path cache_path(
		const std::filesystem::path &directory, CelestialBody body, const Window &window)
{
	return directory /
		   (std::string(celestial_body_name(body)) + "_p" + std::to_string(window.ppd) +
				   "_" + std::to_string(window.line0) + "_" +
				   std::to_string(window.sample0) + "_" + std::to_string(window.lines) +
				   "x" + std::to_string(window.samples) + "_s" +
				   std::to_string(window.step) + ".f64");
}

std::optional<std::vector<double>> load_window(
		const std::filesystem::path &path, std::size_t count)
{
	std::ifstream input(path, std::ios::binary);
	if (!input)
		return std::nullopt;
	std::vector<double> values(count);
	input.read(reinterpret_cast<char *>(values.data()), values.size() * sizeof(double));
	return input && input.peek() == std::char_traits<char>::eof()
				   ? std::optional<std::vector<double>>(std::move(values))
				   : std::nullopt;
}

void save_window(const std::filesystem::path &path, const std::vector<double> &values)
{
	std::error_code error;
	std::filesystem::create_directories(path.parent_path(), error);
	if (error)
		return;
	const auto temporary = path.string() + ".tmp";
	std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
	if (!output)
		return;
	output.write(reinterpret_cast<const char *>(values.data()),
			values.size() * sizeof(double));
	output.close();
	if (output)
		std::filesystem::rename(temporary, path, error);
	if (error)
		std::filesystem::remove(temporary, error);
}

double interpolate(double a, double b, double c, double d, double x, double y)
{
	const double values[] = {a, b, c, d};
	const double weights[] = {(1.0 - x) * (1.0 - y), x * (1.0 - y), (1.0 - x) * y, x * y};
	double sum = 0.0, weight = 0.0;
	for (unsigned i = 0; i < 4; ++i)
		if (std::isfinite(values[i])) {
			sum += values[i] * weights[i];
			weight += weights[i];
		}
	return weight > 0.0 ? sum / weight : std::numeric_limits<double>::quiet_NaN();
}

} // namespace

double planetary_native_resolution_m(CelestialBody body)
{
	const auto spec = spec_for(body);
	return spec ? celestial_radius_m(body) * 3.14159265358979323846 / (180.0 * spec->ppd)
				: std::numeric_limits<double>::max();
}

PlanetaryRangeReader http_planetary_range_reader()
{
	return [](const std::string &url, std::uint64_t offset,
				   std::uint64_t length) -> std::optional<std::vector<std::uint8_t>> {
		const auto response = http_get_range(url, offset, length);
		if (response.size() != length)
			return std::nullopt;
		return std::vector<std::uint8_t>(response.begin(), response.end());
	};
}

std::optional<ElevationData> fetch_planetary_elevation(CelestialBody body,
		const geographic::LLBBox &bbox, std::size_t width, std::size_t height,
		const PlanetaryRangeReader &read_range,
		const std::filesystem::path &cache_directory)
{
	const auto spec = spec_for(body);
	if (!spec || !read_range || width == 0 || height == 0 ||
			bbox.min().lat() < spec->lat_min || bbox.max().lat() > spec->lat_max)
		return std::nullopt;
	const auto window = Window::plan(*spec, bbox, width, height);
	const auto sample_count = window.lines * window.samples;
	const auto cached = cache_directory.empty()
								? std::filesystem::path{}
								: cache_path(cache_directory, body, window);
	std::vector<double> source =
			cached.empty()
					? std::vector<double>{}
					: load_window(cached, sample_count).value_or(std::vector<double>{});
	if (source.empty())
		source.assign(sample_count, std::numeric_limits<double>::quiet_NaN());
	const bool fetched = source.size() == sample_count &&
						 (!cached.empty() && std::filesystem::exists(cached));
	std::size_t successful_rows = fetched ? window.lines : 0;
	if (!fetched) {
		for (std::size_t row = 0; row < window.lines; ++row) {
			std::vector<double> values(
					window.samples, std::numeric_limits<double>::quiet_NaN());
			bool complete = true;
			const auto line = window.line0 + row * window.step;
			const auto latitude = 90.0 - double(line) / spec->ppd;
			const auto lat_band = std::clamp(
					std::floor((latitude - std::numeric_limits<double>::epsilon()) /
							   spec->lat_span) *
							spec->lat_span,
					spec->lat_min, spec->lat_max - spec->lat_span);
			const auto tile_line = std::min(spec->lines() - 1,
					std::size_t(std::llround(
							(lat_band + spec->lat_span - latitude) * spec->ppd)));
			for (std::size_t col = 0; col < window.samples;) {
				const auto global_sample =
						(window.sample0 + col * window.step) % (360 * spec->ppd);
				const auto longitude = double(global_sample) / spec->ppd;
				const auto lon_band =
						std::floor(longitude / spec->lon_span) * spec->lon_span;
				const auto tile_sample =
						std::size_t(std::llround((longitude - lon_band) * spec->ppd));
				const auto take = std::min(
						(spec->samples() - tile_sample + window.step - 1) / window.step,
						window.samples - col);
				const auto length = std::uint64_t(((take - 1) * window.step + 1) * 2);
				const auto offset =
						std::uint64_t((tile_line * spec->samples() + tile_sample) * 2);
				std::optional<std::vector<std::uint8_t>> bytes;
				for (unsigned attempt = 0; attempt < 3 && !bytes; ++attempt) {
					if (attempt)
						std::this_thread::sleep_for(
								std::chrono::milliseconds(400U << (attempt - 1)));
					bytes = read_range(std::string(spec->base_url) +
											   tile_name(body, *spec, lat_band, lon_band),
							offset, length);
					if (bytes && bytes->size() != length)
						bytes.reset();
				}
				if (!bytes) {
					complete = false;
				} else {
					for (std::size_t i = 0; i < take; ++i) {
						const auto at = i * window.step * 2;
						const auto raw =
								spec->big_endian
										? std::int16_t(
												  (std::uint16_t((*bytes)[at]) << 8) |
												  (*bytes)[at + 1])
										: std::int16_t(
												  (std::uint16_t((*bytes)[at + 1]) << 8) |
												  (*bytes)[at]);
						if (raw != std::numeric_limits<std::int16_t>::min())
							values[col + i] = raw * spec->dn_to_meters;
					}
				}
				col += take;
			}
			if (complete) {
				std::copy(values.begin(), values.end(),
						source.begin() +
								static_cast<std::ptrdiff_t>(row * window.samples));
				++successful_rows;
			}
		}
	}
	if (successful_rows == 0)
		return std::nullopt;
	if (!fetched && !cached.empty())
		save_window(cached, source);
	ElevationData result;
	result.width = result.world_width = width;
	result.height = result.world_height = height;
	result.heights.assign(height, std::vector<double>(width));
	const auto west = east_longitude(bbox.min().lng());
	auto east = east_longitude(bbox.max().lng());
	if (east < west)
		east += 360.0;
	for (std::size_t z = 0; z < height; ++z) {
		const auto lat = bbox.max().lat() +
						 (bbox.min().lat() - bbox.max().lat()) *
								 (double(z) / std::max<std::size_t>(1, height - 1));
		const auto fy =
				std::clamp(((90.0 - lat) * spec->ppd - window.line0) / window.step, 0.0,
						double(window.lines - 1));
		const auto y = std::min(window.lines - 1, std::size_t(fy));
		for (std::size_t x = 0; x < width; ++x) {
			const auto lon = west + (east - west) * (double(x) / std::max<std::size_t>(
																		 1, width - 1));
			const auto fx = std::clamp((lon * spec->ppd - window.sample0) / window.step,
					0.0, double(window.samples - 1));
			const auto sx = std::min(window.samples - 1, std::size_t(fx));
			auto at = [&](std::size_t yy, std::size_t xx) {
				return source[std::min(yy, window.lines - 1) * window.samples +
							  std::min(xx, window.samples - 1)];
			};
			result.heights[z][x] = interpolate(at(y, sx), at(y, sx + 1), at(y + 1, sx),
										   at(y + 1, sx + 1), fx - sx, fy - y) *
								   celestial_height_gain(body);
		}
	}
	return result;
}

} // namespace arnis::elevation
