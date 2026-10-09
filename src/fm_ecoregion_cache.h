#pragma once

#include "assets_root.h"
#include "ecoregion.h"

namespace arnis::ecoregion
{
struct GenerationMap
{
	std::optional<EcoMap> map;
	std::optional<std::string> dominant_tree_realm;
	std::vector<std::pair<std::uint16_t, std::size_t>> by_area;
	bool has_gaps = true;
};

inline const GenerationMap &generation_map()
{
	// Bundled climate assets are immutable during a process. Share their bytes
	// and global summary across chunks; function-local initialization also
	// prevents concurrent emerge workers from repeating the initial scan.
	static const GenerationMap cached = [] {
		GenerationMap result;
		if (auto map = EcoMap::load(assets::path("climate/ecoregions.grid"))) {
			// The bundled raster is process-wide, while Freeminer calls
			// generate_world once per emerging chunk. Decode and summarize this
			// global map exactly once instead of rescanning it in every worker.
			result.by_area = map->by_area(&result.has_gaps);
			for (const auto &[id, count] : result.by_area) {
				(void)count;
				if (!result.dominant_tree_realm)
					if (const auto mix = tree_mix(id))
						result.dominant_tree_realm = std::string(mix->first);
			}
			result.map = std::move(map);
		}
		return result;
	}();
	return cached;
}
}
