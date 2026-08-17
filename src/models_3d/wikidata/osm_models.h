#pragma once
#include "../../osm_parser.h"
#include "../../../../arnis_adapter.h"
#include "remote_provider.h"
namespace arnis::models_3d
{
bool place_wikidata_element(
		world_editor::WorldEditor &, const ProcessedElement &, RemoteModelProvider &);
}
