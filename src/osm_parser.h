#pragma once

#include "../../arnis_adapter.h"
#include <istream>
#include <optional>
#include <string>
#include <vector>

namespace arnis::osm_parser
{

struct RawNode
{
	std::int64_t id = 0;
	double lat = 0.0;
	double lon = 0.0;
	tags_t tags;
};

struct RawWay
{
	std::int64_t id = 0;
	std::vector<std::int64_t> node_refs;
	tags_t tags;
};

struct RawRelationMember
{
	std::string type;
	std::int64_t ref = 0;
	std::string role;
};

struct RawRelation
{
	std::int64_t id = 0;
	std::vector<RawRelationMember> members;
	tags_t tags;
};

struct RawOsmDocument
{
	std::vector<RawNode> nodes;
	std::vector<RawWay> ways;
	std::vector<RawRelation> relations;
	std::optional<std::array<double, 4>> bounds; // minlat,minlon,maxlat,maxlon
	std::optional<std::string> remark;
};

// Parses standard .osm XML, including multiline/self-closing elements and XML entities.
// The returned raw document preserves geographic coordinates; projection/clipping remains
// the responsibility of the existing processed-element pipeline.
RawOsmDocument parse_osm_xml(std::istream &input);
RawOsmDocument parse_overpass_json(std::istream &input);

} // namespace arnis::osm_parser
