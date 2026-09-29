#include "mapped.h"
#include "../deterministic_rng.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <iterator>
#include <limits>
#include <stdexcept>

namespace arnis::trees::mapped
{
namespace
{
constexpr double ROW_SPACING_M = 8.0;
constexpr double CROWN_RADIUS_M = 5.0;
std::string genus(const std::unordered_map<std::string, std::string> &tags)
{
	for (const char *key : {"genus", "species", "taxon"}) {
		auto it = tags.find(key);
		if (it == tags.end())
			continue;
		auto value = it->second;
		const auto p = value.find_first_of(" /_-");
		if (p != std::string::npos)
			value.resize(p);
		if (!value.empty()) {
			value[0] = char(std::toupper(value[0]));
			return value;
		}
	}
	auto it = tags.find("genus:wikidata");
	if (it != tags.end()) {
		if (it->second == "Q12004")
			return "Betula";
		if (it->second == "Q26782")
			return "Quercus";
		if (it->second == "Q25243")
			return "Picea";
	}
	return {};
}
}
bool is_conifer_genus(const std::string &g)
{
	static const char *names[] = {"Abies", "Agathis", "Araucaria", "Callitris",
			"Calocedrus", "Cedrus", "Cephalotaxus", "Chamaecyparis", "Cryptomeria",
			"Cunninghamia", "Cupressocyparis", "Cupressus", "Glyptostrobus", "Juniperus",
			"Keteleeria", "Larix", "Metasequoia", "Picea", "Pinus", "Platycladus",
			"Podocarpus", "Pseudolarix", "Pseudotsuga", "Sciadopitys", "Sequoia",
			"Sequoiadendron", "Taxodium", "Taxus", "Tetraclinis", "Thuja", "Thujopsis",
			"Torreya", "Tsuga", "Wollemia", "Xanthocyparis"};
	return std::find_if(std::begin(names), std::end(names),
				   [&](const char *n) { return g == n; }) != std::end(names);
}
MappedTree from_tags(
		const std::unordered_map<std::string, std::string> &tags, std::uint64_t id)
{
	MappedTree out;
	out.genus = genus(tags);
	out.has_conifer = !out.genus.empty();
	out.conifer = is_conifer_genus(out.genus);
	const auto leaf = tags.find("leaf_type");
	std::vector<TreeType> pool;
	if (out.genus == "Betula")
		pool = {TreeType::Birch};
	else if (out.genus == "Quercus")
		pool = {TreeType::Oak};
	else if (out.genus == "Picea" || out.genus == "Pinus" || out.genus == "Larix" ||
			 out.genus == "Cedrus")
		pool = {TreeType::Pine};
	else if (out.genus == "Salix")
		pool = {TreeType::Willow};
	else if (out.genus == "Prunus" || out.genus == "Malus" || out.genus == "Pyrus" ||
			 out.genus == "Magnolia" || out.genus == "Cercis" ||
			 out.genus == "Crataegus" || out.genus == "Sorbus" ||
			 out.genus == "Amelanchier" || out.genus == "Jacaranda" ||
			 out.genus == "Lagerstroemia")
		pool = {TreeType::FloweringOak};
	else if (out.genus == "Acacia" || out.genus == "Vachellia" ||
			 out.genus == "Senegalia" || out.genus == "Albizia" ||
			 out.genus == "Prosopis" || out.genus == "Parkinsonia" ||
			 out.genus == "Delonix")
		pool = {TreeType::Acacia};
	else if (out.genus == "Phoenix" || out.genus == "Washingtonia" ||
			 out.genus == "Cocos" || out.genus == "Trachycarpus" ||
			 out.genus == "Sabal" || out.genus == "Roystonea" || out.genus == "Syagrus" ||
			 out.genus == "Butia" || out.genus == "Livistona" ||
			 out.genus == "Chamaerops" || out.genus == "Elaeis" ||
			 out.genus == "Archontophoenix")
		pool = {TreeType::Jungle};
	else if (out.genus == "Rhizophora" || out.genus == "Avicennia" ||
			 out.genus == "Laguncularia" || out.genus == "Bruguiera" ||
			 out.genus == "Sonneratia")
		pool = {TreeType::Mangrove};
	else if (leaf != tags.end() && leaf->second == "needleleaved")
		pool = {TreeType::Spruce, TreeType::Pine};
	else if (leaf != tags.end() && leaf->second == "broadleaved")
		pool = {TreeType::Oak, TreeType::Birch, TreeType::TallOak};
	else if (!out.genus.empty() && out.has_conifer && out.conifer)
		pool = {TreeType::Spruce};
	else if (!out.genus.empty())
		pool = {TreeType::Oak, TreeType::TallOak};
	else
		pool = {TreeType::Oak, TreeType::Spruce, TreeType::Birch, TreeType::TallOak};
	auto rng = element_rng(id);
	out.kind = pool[rng.uniform(static_cast<std::uint32_t>(pool.size()))];
	if (auto it = tags.find("height"); it != tags.end()) {
		try {
			std::string text = it->second;
			const auto first = text.find_first_not_of(" \t");
			const auto last = text.find_last_not_of(" \t");
			if (first == std::string::npos)
				throw std::invalid_argument("empty height");
			text = text.substr(first, last - first + 1);
			const bool feet = text.size() > 2 && text.substr(text.size() - 2) == "ft";
			if (feet)
				text.resize(text.size() - 2);
			else if (!text.empty() && text.back() == 'm')
				text.pop_back();
			const auto h = std::stod(text) * (feet ? 0.3048 : 1.0);
			if (std::isfinite(h) && h >= 1.0 && h <= 120.0)
				out.height_m = h;
		} catch (...) {
			// Ignore malformed OSM heights, matching Rust's filtered parse.
		}
	}
	return out;
}
bool is_tree_row(const std::unordered_map<std::string, std::string> &tags)
{
	auto it = tags.find("natural");
	return it != tags.end() && it->second == "tree_row" && !tags.contains("building") &&
		   !tags.contains("building:part") && !tags.contains("highway") &&
		   !tags.contains("landuse");
}
std::vector<std::pair<int, int>> tree_row_positions(
		const std::vector<std::pair<int, int>> &nodes, double scale)
{
	if (nodes.empty())
		return {};
	if (nodes.size() == 1)
		return nodes;
	auto dist = [](auto a, auto b) {
		return std::hypot(double(b.first - a.first), double(b.second - a.second));
	};
	double total = 0;
	for (size_t i = 1; i < nodes.size(); ++i)
		total += dist(nodes[i - 1], nodes[i]);
	if (total < 1)
		return {nodes.front()};
	const int intervals =
			std::max(1, int(std::round(total / std::max(1.0, ROW_SPACING_M * scale))));
	const bool closed = nodes.size() > 2 && nodes.front() == nodes.back();
	std::vector<std::pair<int, int>> out;
	size_t seg = 0;
	double start = 0, step = total / intervals;
	const int count = closed ? intervals : intervals + 1;
	for (int k = 0; k < count && seg + 1 < nodes.size(); ++k) {
		double target = k * step;
		while (seg + 2 < nodes.size() &&
				start + dist(nodes[seg], nodes[seg + 1]) < target) {
			start += dist(nodes[seg], nodes[seg + 1]);
			++seg;
		}
		double len = dist(nodes[seg], nodes[seg + 1]),
			   t = len ? std::clamp((target - start) / len, 0.0, 1.0) : 0;
		auto p = std::make_pair(
				int(std::lround(nodes[seg].first +
								(nodes[seg + 1].first - nodes[seg].first) * t)),
				int(std::lround(nodes[seg].second +
								(nodes[seg + 1].second - nodes[seg].second) * t)));
		if (out.empty() || out.back() != p)
			out.push_back(p);
	}
	return out;
}

MappedTrunks MappedTrunks::collect(
		const std::vector<ProcessedElement> &elements, double scale)
{
	MappedTrunks result;
	result.radius_ = std::max(1, int(std::lround(CROWN_RADIUS_M * scale)));
	const auto add = [&](int x, int z) {
		const int r = result.radius_;
		result.trunks_.push_back(
				{int(std::floor(double(x) / r)), int(std::floor(double(z) / r)), x, z});
	};
	for (const auto &element : elements) {
		if (element.is_node()) {
			const auto &node = element.as_node();
			if (node.tags.get("natural") == "tree")
				add(node.x, node.z);
		} else if (element.is_way()) {
			const auto &way = element.as_way();
			if (!is_tree_row(way.tags))
				continue;
			std::vector<std::pair<int, int>> points;
			points.reserve(way.nodes.size());
			for (const auto &node : way.nodes)
				points.emplace_back(node.x, node.z);
			for (const auto [x, z] : tree_row_positions(points, scale))
				add(x, z);
		}
	}
	std::sort(result.trunks_.begin(), result.trunks_.end());
	result.trunks_.shrink_to_fit();
	return result;
}

bool MappedTrunks::under_crown(int x, int z) const
{
	if (trunks_.empty())
		return false;
	const int r = radius_;
	const int cx = int(std::floor(double(x) / r));
	const int cz = int(std::floor(double(z) / r));
	for (int gx = cx - 1; gx <= cx + 1; ++gx) {
		const auto begin = std::lower_bound(trunks_.begin(), trunks_.end(),
				MappedTrunks::Trunk{gx, cz - 1, std::numeric_limits<int>::min(),
						std::numeric_limits<int>::min()},
				[](const Trunk &a, const Trunk &b) {
					return std::tie(a.cell_x, a.cell_z) < std::tie(b.cell_x, b.cell_z);
				});
		for (auto it = begin; it != trunks_.end(); ++it) {
			if (it->cell_x > gx || it->cell_z > cz + 1)
				break;
			if (it->cell_x < gx || it->cell_z < cz - 1)
				continue;
			const long long dx = static_cast<long long>(it->x) - x;
			const long long dz = static_cast<long long>(it->z) - z;
			if (dx * dx + dz * dz <= static_cast<long long>(r) * r)
				return true;
		}
	}
	return false;
}
}
