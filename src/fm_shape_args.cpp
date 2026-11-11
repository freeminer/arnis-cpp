#include "args.h"

#include <fstream>
#include <iterator>

namespace arnis
{
bool Args::resolve_shapes(std::string *error)
{
	if (bbox && (!polygon.empty() || polygon_file)) {
		if (error)
			*error = "--polygon/--polygon-file cannot be combined with --bbox";
		return false;
	}
	std::vector<LLShapes::Polygon> polygons;
	for (std::size_t i = 0; i < polygon.size(); ++i) {
		std::string parse_error;
		auto points = LLShapes::parse_points(polygon[i], &parse_error);
		if (!points) {
			if (error)
				*error = "--polygon " + std::to_string(i + 1) + ": " + parse_error;
			return false;
		}
		LLShapes::Ring ring;
		try {
			for (const auto &[lat, lng] : *points)
				ring.emplace_back(lat, lng);
		} catch (const std::exception &e) {
			if (error)
				*error = "--polygon " + std::to_string(i + 1) + ": " + e.what();
			return false;
		}
		polygons.push_back({std::move(ring)});
	}
	if (polygon_file) {
		std::ifstream input(*polygon_file, std::ios::binary);
		if (!input) {
			if (error)
				*error = "--polygon-file " + *polygon_file + ": cannot read file";
			return false;
		}
		const std::string text{
				std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
		std::string parse_error;
		auto from_file = LLShapes::from_geojson(text, &parse_error);
		if (!from_file) {
			if (error)
				*error = "--polygon-file " + *polygon_file + ": " + parse_error;
			return false;
		}
		polygons.insert(polygons.end(), std::make_move_iterator(from_file->begin()),
				std::make_move_iterator(from_file->end()));
	}
	if (polygons.empty())
		return true;
	std::string shape_error;
	auto resolved = LLShapes::create(polygons, &shape_error);
	if (!resolved) {
		if (error)
			*error = "--polygon: " + shape_error;
		return false;
	}
	bbox = resolved->bbox();
	shapes = std::move(*resolved);
	return true;
}
}
