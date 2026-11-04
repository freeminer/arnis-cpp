#pragma once

#include "../floodfill_cache.h"
#include "../args.h"
#include "../../../arnis_adapter.h"

namespace arnis::golf
{
bool claims(const tags_t &tags);
bool is_managed(const tags_t &tags);
bool is_hole(const tags_t &tags);
void generate_golf(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const FloodFillCache &fills);
void generate_golf_from_relation(WorldEditor &editor, const ProcessedRelation &relation,
		const Args &args, const FloodFillCache &fills);
void generate_hole_flag(WorldEditor &editor, const ProcessedWay &way);
} // namespace arnis::golf
