#include "source.h"

#include "fetch.h"

#include <cmath>

namespace arnis::survey::source
{
namespace
{
constexpr const char *PAGE =
		"https://geoportaal.maaruum.ee/index.php?lang_id=1&plugin_act=otsing";

int sheet_number(std::int32_t east_km, std::int32_t north_km)
{
	return (north_km - 6000) * 1000 + east_km;
}

std::optional<std::string> latest_regular(const std::string &page, int sheet)
{
	const auto prefix = "f=" + std::to_string(sheet) + "_";
	std::optional<std::pair<unsigned, std::string>> latest;
	std::size_t position = 0;
	while ((position = page.find(prefix, position)) != std::string::npos) {
		const auto start = position + 2;
		const auto end = page.find(".laz", start);
		if (end == std::string::npos) {
			position = start;
			continue;
		}
		const auto filename = page.substr(start, end + 4 - start);
		const auto first = filename.find('_');
		const auto second =
				first == std::string::npos ? first : filename.find('_', first + 1);
		unsigned year = 0;
		if (second != std::string::npos && filename.substr(second) == "_tava.laz") {
			try {
				year = static_cast<unsigned>(
						std::stoul(filename.substr(first + 1, second - first - 1)));
			} catch (...) {
				year = 0;
			}
			if (year && (!latest || std::pair{year, filename} > *latest))
				latest = std::pair{year, filename};
		}
		position = end + 4;
	}
	if (!latest)
		return std::nullopt;
	return latest->second;
}

std::variant<TileUrls, std::string> lookup(const Sheet &, const Rect &rect)
{
	TileUrls out;
	const int e0 = static_cast<int>(std::floor(rect[0] / 1000.0));
	const int e1 = static_cast<int>(std::floor((rect[2] - 1e-6) / 1000.0));
	const int n0 = static_cast<int>(std::floor(rect[1] / 1000.0));
	const int n1 = static_cast<int>(std::floor((rect[3] - 1e-6) / 1000.0));
	for (int east = e0; east <= e1; ++east) {
		for (int north = n0; north <= n1; ++north) {
			const int number = sheet_number(east, north);
			const auto page_url = std::string(PAGE) +
								  "&kaardiruut=" + std::to_string(number) +
								  "&andmetyyp=lidar_laz_tava&page_id=614";
			auto response = fetch::get_bytes(page_url);
			const auto *body = std::get_if<std::vector<std::uint8_t>>(&response);
			if (!body) {
				const auto &failure = std::get<fetch::Failure>(response);
				if (failure.kind == fetch::Failure::Kind::Missing)
					continue;
				return page_url + ": " + failure.to_string();
			}
			const std::string page(body->begin(), body->end());
			const auto filename = latest_regular(page, number);
			if (filename) {
				out.emplace_back(Tile{east, north},
						std::string(PAGE) + "&kaardiruut=" + std::to_string(number) +
								"&andmetyyp=lidar_laz_tava&dl=1&f=" + *filename +
								"&page_id=614");
			}
		}
	}
	return out;
}
} // namespace

const Source &estonia()
{
	static const Source value = [] {
		Source source;
		source.id = "estonia";
		source.name = "Maa- ja Ruumiamet";
		source.grid.fixed = {crs::Kind::Lest97, 0};
		source.rings = {{{57.855, 24.379}, {57.857, 24.343}, {57.895, 24.259},
				{57.881, 24.220}, {57.881, 23.644}, {57.782, 23.684}, {57.659, 23.612},
				{57.576, 23.419}, {57.568, 23.165}, {57.692, 22.971}, {57.921, 22.691},
				{57.910, 22.597}, {57.762, 22.157}, {57.731, 21.914}, {57.772, 21.692},
				{57.888, 21.581}, {58.300, 21.348}, {58.540, 21.387}, {58.606, 21.421},
				{58.660, 21.488}, {58.699, 21.577}, {58.831, 21.660}, {58.870, 21.632},
				{58.923, 21.615}, {59.069, 21.825}, {59.261, 22.290}, {59.306, 22.487},
				{59.313, 22.602}, {59.310, 22.721}, {59.479, 23.105}, {59.501, 23.183},
				{59.500, 23.515}, {59.721, 24.471}, {59.845, 25.576}, {59.834, 26.006},
				{59.916, 26.134}, {59.959, 26.452}, {59.833, 26.657}, {59.677, 26.840},
				{59.637, 27.056}, {59.671, 27.390}, {59.561, 27.830}, {59.502, 27.977},
				{59.490, 28.064}, {59.477, 28.077}, {59.471, 28.114}, {59.432, 28.164},
				{59.413, 28.218}, {59.386, 28.245}, {59.341, 28.224}, {59.275, 28.154},
				{59.270, 28.035}, {59.257, 27.978}, {59.233, 27.937}, {59.178, 27.923},
				{59.124, 27.844}, {59.102, 27.845}, {59.086, 27.824}, {59.065, 27.828},
				{59.049, 27.806}, {59.021, 27.799}, {59.016, 27.778}, {58.970, 27.772},
				{58.781, 27.395}, {58.364, 27.590}, {58.299, 27.522}, {58.229, 27.526},
				{58.128, 27.630}, {58.105, 27.676}, {58.053, 27.642}, {58.045, 27.644},
				{58.004, 27.741}, {57.973, 27.715}, {57.948, 27.725}, {57.945, 27.710},
				{57.939, 27.716}, {57.935, 27.751}, {57.920, 27.753}, {57.914, 27.772},
				{57.913, 27.851}, {57.859, 27.853}, {57.845, 27.830}, {57.833, 27.832},
				{57.813, 27.783}, {57.816, 27.767}, {57.803, 27.746}, {57.819, 27.705},
				{57.822, 27.628}, {57.798, 27.570}, {57.781, 27.576}, {57.777, 27.548},
				{57.771, 27.546}, {57.748, 27.592}, {57.744, 27.574}, {57.721, 27.597},
				{57.717, 27.561}, {57.682, 27.559}, {57.692, 27.516}, {57.673, 27.482},
				{57.691, 27.457}, {57.681, 27.446}, {57.682, 27.430}, {57.652, 27.412},
				{57.638, 27.441}, {57.626, 27.433}, {57.620, 27.462}, {57.577, 27.391},
				{57.574, 27.357}, {57.492, 27.400}, {57.485, 27.382}, {57.533, 27.247},
				{57.532, 27.189}, {57.550, 27.085}, {57.579, 26.989}, {57.587, 26.996},
				{57.581, 26.905}, {57.607, 26.912}, {57.609, 26.903}, {57.582, 26.895},
				{57.565, 26.823}, {57.539, 26.799}, {57.549, 26.705}, {57.537, 26.700},
				{57.536, 26.666}, {57.524, 26.650}, {57.479, 26.654}, {57.497, 26.587},
				{57.498, 26.514}, {57.513, 26.471}, {57.556, 26.445}, {57.554, 26.396},
				{57.564, 26.347}, {57.556, 26.316}, {57.576, 26.291}, {57.579, 26.257},
				{57.615, 26.213}, {57.650, 26.196}, {57.655, 26.177}, {57.668, 26.178},
				{57.679, 26.132}, {57.708, 26.160}, {57.721, 26.112}, {57.739, 26.105},
				{57.746, 26.079}, {57.739, 26.056}, {57.750, 26.031}, {57.737, 25.991},
				{57.741, 25.978}, {57.827, 26.013}, {57.835, 25.993}, {57.825, 25.965},
				{57.834, 25.928}, {57.825, 25.866}, {57.838, 25.857}, {57.851, 25.794},
				{57.893, 25.760}, {57.900, 25.743}, {57.885, 25.718}, {57.888, 25.647},
				{57.898, 25.645}, {57.902, 25.623}, {57.917, 25.611}, {57.932, 25.549},
				{57.926, 25.537}, {57.952, 25.525}, {57.948, 25.459}, {57.959, 25.467},
				{57.998, 25.389}, {58.007, 25.413}, {58.012, 25.398}, {58.019, 25.338},
				{57.976, 25.337}, {57.976, 25.222}, {57.996, 25.222}, {58.026, 25.167},
				{58.029, 25.185}, {58.039, 25.188}, {58.058, 25.169}, {58.059, 25.139},
				{58.048, 25.138}, {58.048, 25.096}, {58.002, 25.042}, {57.992, 24.962},
				{57.947, 24.822}, {57.970, 24.793}, {57.968, 24.778}, {57.950, 24.779},
				{57.946, 24.734}, {57.916, 24.691}, {57.937, 24.662}, {57.920, 24.647},
				{57.928, 24.598}, {57.937, 24.581}, {57.915, 24.535}, {57.910, 24.494},
				{57.870, 24.495}, {57.835, 24.385}, {57.855, 24.379}}};
		source.laser.kind = Laser::Kind::Whole;
		source.laser.tile_mb = 60;
		source.laser.locate.kind = Locate::Kind::Lookup;
		source.laser.locate.lookup = lookup;
		source.photo.kind = Photo::Kind::Wms;
		source.photo.url = "https://kaart.maaamet.ee/wms/fotokaart";
		source.photo.layer = "EESTIFOTO";
		source.intensity_defaults = {46900.0f, 49500.0f};
		source.classes.ground = {2};
		source.classes.building = {6};
		source.classes.object = {1, 5};
		source.classes.vegetation = {5};
		source.laser_plants = true;
		source.attribution = "Maa- ja Ruumiamet (open data licence)";
		return source;
	}();
	return value;
}
} // namespace arnis::survey::source
