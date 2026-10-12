#pragma once

#include "assets_root.h"
#include "ecoregion.h"

namespace arnis::ecoregion
{
struct GenerationMap
{
	std::optional<EcoMap> map;
};

inline const GenerationMap &generation_map()
{
	// Bundled climate assets are immutable during a process. Share the archive
	// bytes and parsed index; summarize only the requested generation area.
	static const GenerationMap cached = [] {
		GenerationMap result;
		if (auto map = EcoMap::load(assets::path("climate/ecoregions.grid"))) {
			// Decoding every global tile here changes area-specific tree-pack
			// selection and adds unnecessary startup work in chunk generation.
			result.map = std::move(map);
		}
		return result;
	}();
	return cached;
}
}
