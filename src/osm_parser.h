#pragma once

#include "../../arnis_adapter.h"
#include <array>
#include <cstdint>
#include <istream>
#include <optional>
#include <string>
#include <vector>

namespace arnis::osm_parser
{

// Rust packs building façade, construction-year and housing metadata into the
// high seed bits. These helpers keep grouping and deterministic variant RNG
// independent of that metadata.
enum class StyleHint : std::uint8_t
{
	None = 0,
	Masonry = 1,
	Contemporary = 2,
	Glass = 3
};
inline constexpr std::uint64_t STYLE_HINT_SHIFT = 61;
inline constexpr std::uint64_t STYLE_HINT_MASK = 0b11ULL << STYLE_HINT_SHIFT;
inline constexpr std::uint64_t RESIDENTIAL_GROUP_BIT = std::uint64_t{1} << 60;
inline constexpr std::uint64_t YEAR_SHIFT = 52;
inline constexpr std::uint64_t YEAR_MASK = std::uint64_t{0xFF} << YEAR_SHIFT;
inline constexpr std::int32_t YEAR_BASE = 1800;
inline StyleHint style_hint_from_seed(std::uint64_t seed)
{
	switch ((seed & STYLE_HINT_MASK) >> STYLE_HINT_SHIFT) {
	case 1:
		return StyleHint::Masonry;
	case 2:
		return StyleHint::Contemporary;
	case 3:
		return StyleHint::Glass;
	default:
		return StyleHint::None;
	}
}
inline std::uint64_t seed_without_hint(std::uint64_t seed)
{
	return seed & ~(STYLE_HINT_MASK | RESIDENTIAL_GROUP_BIT | YEAR_MASK);
}
// Parts of one building share the seed with style, year and housing metadata packed in
// its upper bits. Grouping must ignore all three without changing the actual OSM id.
inline std::uint64_t group_key(std::uint64_t seed)
{
	return seed & ~(STYLE_HINT_MASK | YEAR_MASK | RESIDENTIAL_GROUP_BIT);
}
// Use this name at random-variant call sites: style metadata must not shift
// the lower-bit deterministic choices shared by Rust and C++ generation.
inline std::uint64_t variant_seed(std::uint64_t seed)
{
	return seed_without_hint(seed);
}
inline std::uint64_t seed_with_hint(std::uint64_t seed, StyleHint hint)
{
	return (seed & ~STYLE_HINT_MASK) |
		   (static_cast<std::uint64_t>(hint) << STYLE_HINT_SHIFT);
}
inline bool residential_group_from_seed(std::uint64_t seed)
{
	return (seed & RESIDENTIAL_GROUP_BIT) != 0;
}
inline std::uint64_t seed_without_residential_mark(std::uint64_t seed)
{
	return seed & ~RESIDENTIAL_GROUP_BIT;
}
inline std::optional<std::int32_t> year_from_seed(std::uint64_t seed)
{
	const auto packed = static_cast<std::int32_t>((seed & YEAR_MASK) >> YEAR_SHIFT);
	return packed > 0 ? std::optional<std::int32_t>(YEAR_BASE + packed) : std::nullopt;
}
inline std::uint64_t seed_without_year(std::uint64_t seed)
{
	return seed & ~YEAR_MASK;
}
std::optional<std::int32_t> building_start_year(const tags_t &tags);
std::optional<std::int32_t> building_year_for_parts(const tags_t &tags);
bool is_housing_type(const tags_t &tags);
bool part_covers_ground(const tags_t &tags);
std::uint64_t seed_with_metadata(std::uint64_t seed, StyleHint hint,
		std::optional<std::int32_t> year, bool residential);
StyleHint building_style_hint(const tags_t &tags);
std::size_t get_priority(const ProcessedElement &element);
void filter_tags(tags_t &tags);
enum class ArchEra : std::uint8_t
{
	Unknown,
	HistoricOrnate,
	TraditionalPreWar,
	PostWarPanel,
	Contemporary
};
ArchEra arch_era_from_hint(StyleHint hint);
ArchEra era_for_year(std::int32_t year);
ArchEra building_arch_era_dated(const tags_t &tags, std::optional<std::int32_t> year);
bool is_art_deco(const tags_t &tags);

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
// Return source bounds when supplied, otherwise derive minlat/minlon/maxlat/maxlon
// from decoded nodes, matching Rust's resolve_bbox fallback.
std::optional<std::array<double, 4>> resolve_bbox(const RawOsmDocument &document);

} // namespace arnis::osm_parser
