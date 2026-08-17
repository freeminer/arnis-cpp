#include "client.h"
#include <cassert>
namespace arnis::models_3d::wikidata_client
{
void self_check()
{
	const auto a = url_hash("https://commons.wikimedia.org/wiki/Special:FilePath/X.stl");
	const auto b = url_hash("https://commons.wikimedia.org/wiki/Special:FilePath/X.stl");
	assert(a == b && a.size() == 16);
	assert(a != url_hash("https://commons.wikimedia.org/wiki/Special:FilePath/Y.stl"));
}
}
