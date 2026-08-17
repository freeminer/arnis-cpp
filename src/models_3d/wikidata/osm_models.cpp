#include "osm_models.h"
#include "../placement_pipeline.h"
namespace arnis::models_3d
{
bool place_wikidata_element(
		world_editor::WorldEditor &e, const ProcessedElement &el, RemoteModelProvider &p)
{
	auto q = el.tag("wikidata");
	if (!q)
		return false;
	auto a = p.fetch(*q);
	if (!a)
		return false;
	auto n = el.first_node();
	if (!n)
		return false;
	return place_model_asset(
			e, *a, n->x, e.get_absolute_y(n->x, 1, n->z), n->z, 1.0f, 0.0f);
}
}
