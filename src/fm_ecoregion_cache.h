#pragma once

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
		for (const auto &candidate :
				{std::filesystem::path("assets/climate/ecoregions.grid"),
						std::filesystem::path(__FILE__).parent_path().parent_path() /
								"assets/climate/ecoregions.grid"}) {
			if (auto map = EcoMap::load(candidate)) {
				result.dominant_tree_realm = map->dominant_tree_pack();
				result.map = std::move(map);
				break;
			}
		}
		return result;
	}();
	return cached;
}
}
