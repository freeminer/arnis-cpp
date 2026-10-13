#include "client.h"
#include <cassert>
namespace arnis::models_3d::wikidata_client
{
void self_check()
{
	const auto a = url_hash("https://commons.wikimedia.org/wiki/Special:FilePath/X.stl");
	const auto b = url_hash("https://commons.wikimedia.org/wiki/Special:FilePath/X.stl");
	assert(a == b && a.size() == 16);
	// Golden value includes Rust Hash-for-str's trailing 0xff marker.
	assert(a == "1fd68ae9f0ade793");
	assert(a != url_hash("https://commons.wikimedia.org/wiki/Special:FilePath/Y.stl"));
}
}
