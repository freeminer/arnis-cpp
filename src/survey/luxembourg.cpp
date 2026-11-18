#include "source.h"

#include "fetch.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace arnis::survey::source
{
namespace
{
constexpr const char *DATASET =
		"https://data.public.lu/api/1/datasets/lidar-2024-releve-3d-du-territoire-luxembourgeois/";
constexpr int ZIP_E0 = 48500;
constexpr int ZIP_N0 = 56500;
constexpr int ZIP_SIDE = 1500;

struct Entry
{
	std::string name;
	std::uint64_t local_offset = 0;
};

using Zip = std::pair<std::string, std::vector<Entry>>;

int floor_div(int value, int divisor)
{
	const int quotient = value / divisor;
	const int remainder = value % divisor;
	return remainder < 0 ? quotient - 1 : quotient;
}

std::string three_digits(int value)
{
	auto digits = std::to_string(value);
	if (digits.size() < 3)
		digits.insert(0, 3 - digits.size(), '0');
	return digits;
}

std::pair<int, int> zip_of(int east, int north)
{
	return {floor_div(east - ZIP_E0, ZIP_SIDE) + 1,
			floor_div(north - ZIP_N0, ZIP_SIDE) + 1};
}

std::optional<std::uint64_t> little(
		const std::vector<std::uint8_t> &bytes, std::size_t offset, std::size_t size)
{
	if (size > 8 || offset > bytes.size() || size > bytes.size() - offset)
		return std::nullopt;
	std::uint64_t value = 0;
	for (std::size_t i = 0; i < size; ++i)
		value |= std::uint64_t(bytes[offset + i]) << (8 * i);
	return value;
}

std::optional<std::vector<Entry>> parse_directory(const std::vector<std::uint8_t> &tail)
{
	if (tail.size() < 22)
		return std::nullopt;
	std::optional<std::size_t> end;
	for (std::size_t pos = tail.size() - 4;; --pos) {
		if (tail[pos] == 'P' && tail[pos + 1] == 'K' && tail[pos + 2] == 5 &&
				tail[pos + 3] == 6) {
			end = pos;
			break;
		}
		if (pos == 0)
			break;
	}
	if (!end)
		return std::nullopt;
	const auto count = little(tail, *end + 10, 2);
	const auto size = little(tail, *end + 12, 4);
	if (!count || !size || *size > *end)
		return std::nullopt;
	std::size_t pos = *end - static_cast<std::size_t>(*size);
	std::vector<Entry> entries;
	for (std::uint64_t i = 0; i < *count; ++i) {
		if (pos > tail.size() || tail.size() - pos < 46 || tail[pos] != 'P' ||
				tail[pos + 1] != 'K' || tail[pos + 2] != 1 || tail[pos + 3] != 2)
			return std::nullopt;
		const auto method = little(tail, pos + 10, 2);
		const auto name_length = little(tail, pos + 28, 2);
		const auto extra_length = little(tail, pos + 30, 2);
		const auto comment_length = little(tail, pos + 32, 2);
		const auto local = little(tail, pos + 42, 4);
		if (!method || !name_length || !extra_length || !comment_length || !local)
			return std::nullopt;
		const std::size_t name_at = pos + 46;
		const auto name_size = static_cast<std::size_t>(*name_length);
		if (name_at > tail.size() || name_size > tail.size() - name_at)
			return std::nullopt;
		if (*method == 0)
			entries.push_back({std::string(tail.begin() + name_at,
									   tail.begin() + name_at + name_size),
					*local});
		const auto next = name_at + name_size + static_cast<std::size_t>(*extra_length) +
						  static_cast<std::size_t>(*comment_length);
		if (next > tail.size())
			return std::nullopt;
		pos = next;
	}
	return entries;
}

std::variant<std::vector<Entry>, std::string> directory(const std::string &url)
{
	auto tail = fetch::get_tail(url, 65536);
	const auto *bytes = std::get_if<std::vector<std::uint8_t>>(&tail);
	if (!bytes)
		return url + ": " + std::get<fetch::Failure>(tail).to_string();
	auto entries = parse_directory(*bytes);
	if (!entries)
		return url + ": no zip directory";
	return std::move(*entries);
}

std::variant<std::uint64_t, std::string> data_start(
		const std::string &url, std::uint64_t local)
{
	auto header = fetch::get_range(url, local, 30);
	const auto *bytes = std::get_if<std::vector<std::uint8_t>>(&header);
	if (!bytes)
		return url + ": " + std::get<fetch::Failure>(header).to_string();
	if (bytes->size() < 30 || (*bytes)[0] != 'P' || (*bytes)[1] != 'K' ||
			(*bytes)[2] != 3 || (*bytes)[3] != 4)
		return url + ": no local header at " + std::to_string(local);
	const auto name_length =
			std::uint64_t((*bytes)[26]) | (std::uint64_t((*bytes)[27]) << 8);
	const auto extra_length =
			std::uint64_t((*bytes)[28]) | (std::uint64_t((*bytes)[29]) << 8);
	return local + 30 + name_length + extra_length;
}

std::variant<TileUrls, std::string> lookup(const Sheet &, const Rect &rect)
{
	auto record = fetch::get_json(DATASET);
	const auto *json = std::get_if<nlohmann::json>(&record);
	if (!json)
		return DATASET + std::string(": ") + std::get<fetch::Failure>(record).to_string();
	std::map<std::string, std::string> zips;
	if (const auto it = json->find("resources"); it != json->end() && it->is_array()) {
		for (const auto &resource : *it) {
			if (!resource.is_object())
				continue;
			const auto title = resource.value("title", std::string{});
			const auto url = resource.value("url", std::string{});
			if (!title.empty() && !url.empty())
				zips[title] = url;
		}
	}
	std::map<std::pair<int, int>, std::optional<Zip>> directories;
	TileUrls out;
	const int e0 = static_cast<int>(std::floor(rect[0] / 1000.0));
	const int e1 = static_cast<int>(std::floor((rect[2] - 1e-6) / 1000.0));
	const int n0 = static_cast<int>(std::floor(rect[1] / 1000.0));
	const int n1 = static_cast<int>(std::floor((rect[3] - 1e-6) / 1000.0));
	for (int te = e0; te <= e1; ++te) {
		for (int tn = n0; tn <= n1; ++tn) {
			for (const auto [de, dn] : std::array<std::pair<int, int>, 4>{
						 {{0, 0}, {500, 0}, {0, 500}, {500, 500}}}) {
				const int east = te * 1000 + de;
				const int north = tn * 1000 + dn;
				const auto key = zip_of(east, north);
				auto dir = directories.find(key);
				if (dir == directories.end()) {
					const auto title = "lidar2024-c" + three_digits(key.first) + "-r" +
									   three_digits(key.second) + ".zip";
					const auto url = zips.find(title);
					std::optional<Zip> value;
					if (url != zips.end()) {
						auto entries = directory(url->second);
						if (const auto *failure = std::get_if<std::string>(&entries))
							return *failure;
						value = Zip{url->second,
								std::get<std::vector<Entry>>(std::move(entries))};
					}
					dir = directories.emplace(key, std::move(value)).first;
				}
				if (!dir->second)
					continue;
				const auto name =
						std::to_string(east) + "_" + std::to_string(north + 500) + ".laz";
				const auto &zip = *dir->second;
				const auto entry = std::find_if(zip.second.begin(), zip.second.end(),
						[&](const Entry &item) { return item.name == name; });
				if (entry == zip.second.end())
					continue;
				auto offset = data_start(zip.first, entry->local_offset);
				if (const auto *failure = std::get_if<std::string>(&offset))
					return *failure;
				out.emplace_back(Tile{te, tn},
						zip.first +
								"#at=" + std::to_string(std::get<std::uint64_t>(offset)));
			}
		}
	}
	return out;
}
} // namespace

const Source &luxembourg()
{
	static const Source value = [] {
		Source source;
		source.id = "luxembourg";
		source.name = "LiDAR 2024 Luxembourg";
		source.grid.fixed = {crs::Kind::Luref, 0};
		source.rings = {{{49.826, 5.714}, {49.830, 5.710}, {49.845, 5.714},
				{49.854, 5.690}, {49.863, 5.691}, {49.867, 5.716}, {49.877, 5.718},
				{49.874, 5.726}, {49.880, 5.726}, {49.887, 5.700}, {49.916, 5.717},
				{49.924, 5.734}, {49.978, 5.753}, {49.979, 5.776}, {49.990, 5.796},
				{50.024, 5.788}, {50.035, 5.828}, {50.041, 5.824}, {50.047, 5.833},
				{50.056, 5.822}, {50.082, 5.838}, {50.091, 5.862}, {50.122, 5.870},
				{50.138, 5.901}, {50.144, 5.931}, {50.161, 5.929}, {50.163, 5.934},
				{50.189, 5.938}, {50.194, 5.994}, {50.208, 6.044}, {50.184, 6.053},
				{50.190, 6.081}, {50.188, 6.112}, {50.171, 6.153}, {50.162, 6.148},
				{50.148, 6.176}, {50.141, 6.158}, {50.134, 6.174}, {50.123, 6.165},
				{50.120, 6.171}, {50.110, 6.163}, {50.095, 6.181}, {50.092, 6.167},
				{50.076, 6.182}, {50.076, 6.162}, {50.056, 6.151}, {50.043, 6.165},
				{50.037, 6.162}, {50.027, 6.186}, {50.003, 6.176}, {49.998, 6.200},
				{50.002, 6.207}, {49.975, 6.217}, {49.968, 6.225}, {49.968, 6.273},
				{49.962, 6.277}, {49.945, 6.253}, {49.910, 6.264}, {49.896, 6.281},
				{49.890, 6.313}, {49.893, 6.327}, {49.867, 6.343}, {49.870, 6.376},
				{49.837, 6.415}, {49.832, 6.447}, {49.848, 6.471}, {49.830, 6.501},
				{49.835, 6.505}, {49.825, 6.549}, {49.799, 6.562}, {49.790, 6.547},
				{49.787, 6.552}, {49.774, 6.541}, {49.758, 6.553}, {49.744, 6.529},
				{49.721, 6.561}, {49.711, 6.539}, {49.708, 6.540}, {49.681, 6.496},
				{49.676, 6.479}, {49.664, 6.466}, {49.658, 6.471}, {49.612, 6.447},
				{49.586, 6.416}, {49.585, 6.408}, {49.554, 6.418}, {49.555, 6.415},
				{49.527, 6.388}, {49.505, 6.396}, {49.429, 6.397}, {49.427, 6.392},
				{49.449, 6.344}, {49.449, 6.323}, {49.457, 6.310}, {49.464, 6.284},
				{49.461, 6.274}, {49.471, 6.263}, {49.478, 6.263}, {49.479, 6.251},
				{49.486, 6.251}, {49.490, 6.230}, {49.488, 6.227}, {49.488, 6.198},
				{49.474, 6.210}, {49.475, 6.163}, {49.468, 6.150}, {49.458, 6.148},
				{49.456, 6.128}, {49.429, 6.130}, {49.440, 6.067}, {49.429, 6.058},
				{49.431, 5.997}, {49.435, 5.995}, {49.426, 5.968}, {49.462, 5.945},
				{49.459, 5.937}, {49.480, 5.923}, {49.482, 5.913}, {49.478, 5.903},
				{49.481, 5.864}, {49.497, 5.826}, {49.505, 5.823}, {49.519, 5.806},
				{49.543, 5.766}, {49.570, 5.814}, {49.579, 5.810}, {49.584, 5.823},
				{49.604, 5.817}, {49.618, 5.848}, {49.627, 5.848}, {49.632, 5.855},
				{49.641, 5.850}, {49.651, 5.872}, {49.663, 5.844}, {49.679, 5.833},
				{49.703, 5.842}, {49.709, 5.794}, {49.739, 5.802}, {49.744, 5.792},
				{49.756, 5.789}, {49.775, 5.771}, {49.776, 5.727}, {49.808, 5.724},
				{49.821, 5.707}, {49.826, 5.714}}};
		source.laser.kind = Laser::Kind::Copc;
		source.laser.mb_per_km2 = 330;
		source.laser.locate.kind = Locate::Kind::Lookup;
		source.laser.locate.lookup = lookup;
		source.photo.kind = Photo::Kind::Wms;
		source.photo.url = "https://wms.geoportail.lu/opendata/service";
		source.photo.layer = "ortho_latest";
		source.intensity_defaults = {44500.0f, 48600.0f};
		source.classes.ground = {2};
		source.classes.building = {6};
		source.classes.object = {1, 3, 4, 5};
		source.classes.vegetation = {3, 4, 5};
		source.laser_plants = true;
		source.attribution =
				"Administration du cadastre et de la topographie, Luxembourg (CC0)";
		return source;
	}();
	return value;
}
} // namespace arnis::survey::source
