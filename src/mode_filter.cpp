#include "mode_filter.h"

#include <algorithm>
#include <cctype>

namespace arnis::mode_filter
{
namespace
{
bool waterway_structure(const std::string &waterway)
{
	return waterway == "dam" || waterway == "weir" || waterway == "lock_gate" ||
		   waterway == "boatyard" || waterway == "fuel";
}

const std::string *tag(const tags_t &tags, const char *key)
{
	const auto it = tags.find(key);
	return it == tags.end() ? nullptr : &it->second;
}

bool terrain_feature(const ProcessedElement &element)
{
	const auto &tags = element.tags();
	if (element.is_node())
		return false;
	for (const char *key : {"building", "building:part", "highway", "railway", "amenity",
				 "leisure", "man_made", "barrier", "power", "aeroway", "tourism",
				 "historic", "bridge", "place"})
		if (tags.contains(key))
			return false;
	if (const auto natural = tag(tags, "natural"))
		return *natural != "tree" && *natural != "tree_row";
	if (const auto waterway = tag(tags, "waterway"))
		return !waterway_structure(*waterway);
	return tags.contains("water") || tags.get("landuse") == "reservoir" ||
		   tags.get("landuse") == "basin" || tags.get("landuse") == "forest";
}

bool blueprint_feature(const ProcessedElement &element)
{
	const auto &tags = element.tags();
	if (element.is_node())
		return tags.get("natural") == "tree";
	if (const auto part = tag(tags, "building:part")) {
		const bool is_no = part->size() == 2 &&
						   std::tolower(static_cast<unsigned char>((*part)[0])) == 'n' &&
						   std::tolower(static_cast<unsigned char>((*part)[1])) == 'o';
		if (!is_no)
			return false;
	}
	if (tags.contains("building") || tags.get("type") == "building")
		return true;
	if (const auto waterway = tag(tags, "waterway"))
		return !waterway_structure(*waterway);
	if (const auto leisure = tag(tags, "leisure"))
		return *leisure != "playground";
	if (const auto man_made = tag(tags, "man_made"))
		return *man_made == "pier" || *man_made == "breakwater";
	if (const auto amenity = tag(tags, "amenity"))
		return *amenity == "parking";
	for (const char *key :
			{"highway", "railway", "natural", "landuse", "water", "aeroway", "place"})
		if (tags.contains(key))
			return true;
	return false;
}
}

void retain_for_mode(std::vector<ProcessedElement> &elements, GenerationMode mode)
{
	if (mode == GenerationMode::TerrainOnly) {
		elements.erase(
				std::remove_if(elements.begin(), elements.end(),
						[&](const auto &element) { return !terrain_feature(element); }),
				elements.end());
	} else if (mode == GenerationMode::Blueprint) {
		elements.erase(
				std::remove_if(elements.begin(), elements.end(),
						[&](const auto &element) { return !blueprint_feature(element); }),
				elements.end());
	}
}
}
