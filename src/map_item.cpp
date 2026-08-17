#include "map_item.h"
#include <algorithm>
namespace arnis::map_item { std::tuple<int,std::int8_t,bool> map_geometry(int d){for(int s=0;s<=4;++s){int blocks=16<<s;if(d<=MAP_SIZE*blocks)return {blocks,static_cast<std::int8_t>(s),true};}return {16<<4,4,false};} }
