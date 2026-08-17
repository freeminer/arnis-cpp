#include "placement.h"
#include "../placement_pipeline.h"
#include "wikidata_index.h"
#include <algorithm>
#include <cmath>
namespace arnis::models_3d::wikidata
{
float scale_for_height(const ModelAsset &a, float h)
{
	return h / std::max(0.001f, a.max[1] - a.min[1]);
}
static float normalize_yaw(float yaw)
{
	if (!std::isfinite(yaw))
		return 0.0f;
	yaw = std::fmod(yaw, 360.0f);
	return yaw < 0 ? yaw + 360.0f : yaw;
}
bool place_wikidata_model(ModelProvider &provider, world_editor::WorldEditor &editor,
		const std::string &key, float x, float y, float z, float scale, float yaw)
{
	auto asset = provider.fetch(key);
	return asset && place_model_asset(editor, *asset, x, y, z, scale, normalize_yaw(yaw));
}
bool place_wikidata_model_height(ModelProvider &p, world_editor::WorldEditor &e,
		const std::string &k, float x, float y, float z, float h, float yaw)
{
	if (h <= 0.0f || !std::isfinite(h))
		return false;
	auto a = p.fetch(k);
	return a &&
		   place_model_asset(e, *a, x, y, z, scale_for_height(*a, h), normalize_yaw(yaw));
}
bool place_wikidata_model_with_info(ModelProvider &p, world_editor::WorldEditor &e,
		const std::string &k, const three_dmr::ModelInfo &i, float x, float y, float z,
		float scale, float yaw)
{
	if (!std::isfinite(scale) || scale <= 0)
		return false;
	auto a = p.fetch(k);
	if (!a)
		return false;
	const float combined = scale * float(i.scale);
	const float tx = x + float(i.translation[0] * scale),
				ty = y + float(i.translation[1] * scale),
				tz = z + float(i.translation[2] * scale);
	return place_model_asset(
			e, *a, tx, ty, tz, combined, normalize_yaw(yaw + float(i.rotation)));
}
PlacementAttribution place_wikidata_attributed(ModelProvider &p,
		world_editor::WorldEditor &e, const std::string &q, float x, float y, float z,
		float scale, float yaw)
{
	PlacementAttribution r;
	r.qid = q;
	if (const auto *a = lookup_wikidata(q)) {
		r.label = a->label;
		r.artist = a->artist;
		r.license = a->license;
		r.license_url = a->license_url;
	}
	r.placed = place_wikidata_model(p, e, q, x, y, z, scale, yaw);
	return r;
}
}
