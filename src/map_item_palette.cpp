#include "map_item_palette.h"
#include <array>
#include <limits>
namespace arnis::map_palette {
std::uint8_t nearest_map_color(std::uint8_t r,std::uint8_t g,std::uint8_t b){static constexpr std::array<std::array<int,3>,8> c={{{0,0,0},{127,178,56},{247,233,163},{199,199,199},{160,160,255},{112,112,112},{64,64,255},{143,119,72}}};unsigned best=4;int bd=std::numeric_limits<int>::max();for(unsigned i=1;i<c.size();++i)for(unsigned s=0;s<4;++s){int m[4]={180,220,255,135};int dr=r-c[i][0]*m[s]/255,dg=g-c[i][1]*m[s]/255,db=b-c[i][2]*m[s]/255,d=dr*dr+dg*dg+db*db;if(d<bd){bd=d;best=i*4+s;}}return best;}
}
