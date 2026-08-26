#pragma once

#include "../../arnis_adapter.h"
#include <cstdint>
#include <istream>
#include <optional>
#include <string>
#include <vector>

namespace arnis::osm_parser
{

// The top two seed bits carry the Rust building-facade hint.  Keeping these
// helpers in the parser API ensures relation and way processing consume the
// same packed seed without perturbing the lower-bit variant RNG.
enum class StyleHint : std::uint8_t { None = 0, Masonry = 1, Contemporary = 2, Glass = 3 };
inline constexpr std::uint64_t STYLE_HINT_SHIFT = 61;
inline constexpr std::uint64_t STYLE_HINT_MASK = 0b11ULL << STYLE_HINT_SHIFT;
inline StyleHint style_hint_from_seed(std::uint64_t seed)
{
	switch ((seed & STYLE_HINT_MASK) >> STYLE_HINT_SHIFT) {
	case 1: return StyleHint::Masonry;
	case 2: return StyleHint::Contemporary;
	case 3: return StyleHint::Glass;
	default: return StyleHint::None;
	}
}
inline std::uint64_t seed_without_hint(std::uint64_t seed)
{
	return seed & ~STYLE_HINT_MASK;
}
// Use this name at random-variant call sites: style metadata must not shift
// the lower-bit deterministic choices shared by Rust and C++ generation.
inline std::uint64_t variant_seed(std::uint64_t seed)
{
	return seed_without_hint(seed);
}
inline std::uint64_t seed_with_hint(std::uint64_t seed, StyleHint hint)
{
	return seed_without_hint(seed) |
			(static_cast<std::uint64_t>(hint) << STYLE_HINT_SHIFT);
}
StyleHint building_style_hint(const tags_t &tags);
enum class ArchEra : std::uint8_t { Unknown, HistoricOrnate, TraditionalPreWar, PostWarPanel, Contemporary };
ArchEra arch_era_from_hint(StyleHint hint);

struct RawNode
{
	std::uint64_t id = 0;
	double lat = 0.0;
	double lon = 0.0;
	tags_t tags;
};

struct RawWay
{
	std::uint64_t id = 0;
	std::vector<std::uint64_t> node_refs;
	tags_t tags;
};

struct RawRelationMember
{
	std::string type;
	std::uint64_t ref = 0;
	std::string role;
};

struct RawRelation
{
	std::uint64_t id = 0;
	std::vector<RawRelationMember> members;
	tags_t tags;
};

struct RawOsmDocument
{
	struct Completeness
	{
		std::uint64_t unresolved_node_refs{0};
		std::uint64_t total_node_refs{0};
		std::uint64_t ways_missing_nodes{0};
	};
	std::vector<RawNode> nodes;
	std::vector<RawWay> ways;
	std::vector<RawRelation> relations;
	std::optional<std::array<double, 4>> bounds; // minlat,minlon,maxlat,maxlon
	std::optional<std::string> remark;
	Completeness completeness;
};

RawOsmDocument::Completeness analyze_completeness(const RawOsmDocument &document);

// Parses standard .osm XML, including multiline/self-closing elements and XML entities.
// The returned raw document preserves geographic coordinates; projection/clipping remains
// the responsibility of the existing processed-element pipeline.
RawOsmDocument parse_osm_xml(std::istream &input);
RawOsmDocument parse_overpass_json(std::istream &input);

} // namespace arnis::osm_parser
