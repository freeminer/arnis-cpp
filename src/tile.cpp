#include "tile.h"
#include <algorithm>
namespace arnis::tiles { std::vector<TileBounds> create_tiles(int minx,int minz,int maxx,int maxz,int s){std::vector<TileBounds>o;if(s<=0)return o;int ax=(minx>>9)<<9,az=(minz>>9)<<9,bx=((maxx+512)>>9)<<9,bz=((maxz+512)>>9)<<9;for(int z=az;z<bz;z+=s)for(int x=ax;x<bx;x+=s){int ex=std::min(x+s,bx),ez=std::min(z+s,bz);if(ex>minx&&x<=maxx&&ez>minz&&z<=maxz)o.push_back({x,z,ex,ez});}return o;} }
