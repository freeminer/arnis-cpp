#include "wikidata_provider.h"
#include "wikidata_index.h"
namespace arnis::models_3d
{
std::optional<ModelAsset> WikidataProvider::fetch(const std::string &qid)
{
	const auto *e = lookup_wikidata(qid);
	if (!e)
		return {};
	auto p = e->url.substr(e->url.find_last_of('/') + 1);
	return files_.fetch(p);
}
}
