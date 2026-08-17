#pragma once
#include "wikidata_index.h"
#include "../../../../arnis_block.h"
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <functional>
namespace arnis
{
class ProcessedElement;
}
namespace arnis::models_3d::wikidata
{
struct Bounds
{
	int min_x = 0, min_z = 0, max_x = 0, max_z = 0;
	bool contains(int x, int z) const
	{
		return x >= min_x && x <= max_x && z >= min_z && z <= max_z;
	}
};
struct Placement
{
	std::int64_t osm_id = 0;
	// Kept with the placement so fetch filtering can rebuild exactly the claims
	// this model owns (Rust only claims OSM geometry after its download works).
	std::string osm_kind;
	bool has_raw_footprint = false;
	std::string qid;
	int anchor_x = 0, anchor_z = 0;
	Bounds footprint{};
	double yaw_degrees = 0;
	std::optional<double> height_m, xz_extent_m;
	// Rust's contextual fallback palette. Used only by untextured/STL voxels.
	std::vector<Block> palette;
	std::vector<WikidataEntry::PaletteLayer> palette_layers;
};
struct SuppressionClaim
{
	std::pair<std::string, std::int64_t> key;
	// OSM id of the successful model placement that owns this claim.  The
	// claimed key may be a different building inside that model's footprint.
	std::int64_t owner_osm_id = 0;
};
struct PrescanResult
{
	std::vector<Placement> placements;
	std::vector<std::pair<std::string, std::int64_t>> suppressed;
	std::vector<SuppressionClaim> suppression_claims;
};
PrescanResult prescan(const std::vector<ProcessedElement> &elements, double rotation,
		double scale,
		const std::vector<std::pair<std::string, std::int64_t>> &already_suppressed = {});
std::vector<std::pair<int, int>> deferred_regions(const PrescanResult &, double scale);
// Rust suppresses OSM geometry only after a model fetch succeeds.  Hosts that
// fetch during discovery can apply this filter before merging suppression IDs.
void retain_fetchable(
		PrescanResult &, const std::function<bool(const std::string &qid)> &);
}
