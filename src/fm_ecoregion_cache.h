#pragma once

#include "assets_root.h"
#include "ecoregion.h"

namespace arnis::ecoregion
{
struct GenerationMap
{
	std::optional<EcoMap> map;
	std::optional<std::string> dominant_tree_realm;
};

inline const GenerationMap &generation_map()
{
	// Bundled climate assets are immutable during a process. Share their bytes
	// and global summary across chunks; function-local initialization also
	// prevents concurrent emerge workers from repeating the initial scan.
	static const GenerationMap cached = [] {
		GenerationMap result;
		if (auto map = EcoMap::load(assets::path("climate/ecoregions.grid"))) {
			result.dominant_tree_realm = map->dominant_tree_pack();
			result.map = std::move(map);
		}
		return result;
	}();
	return cached;
}
}
