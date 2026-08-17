#include "world_utils.h"
#include <cstdlib>
#include <algorithm>
#include <cctype>
namespace arnis::world_utils {
static std::filesystem::path home(){ if(const char *h=std::getenv("HOME")) return h; return "."; }
std::filesystem::path get_bedrock_output_directory(){ auto p=home()/"Desktop"; return std::filesystem::exists(p)?p:home(); }
std::filesystem::path get_luanti_worlds_directory(){ auto p=home()/".minetest"/"worlds"; if(std::filesystem::exists(p.parent_path())) return p; return get_bedrock_output_directory()/"Arnis Luanti Worlds"; }
std::string sanitize_for_filename(const std::string &name){ std::string out; for(unsigned char c:name) out += (c<32 || std::string("<>:\"/\\|?*").find(c)!=std::string::npos)?'_':char(c); while(!out.empty()&&std::isspace((unsigned char)out.back())) out.pop_back(); std::size_t b=0; while(b<out.size()&&std::isspace((unsigned char)out[b])) ++b; out=out.substr(b,64); return out.empty()?"Unknown Location":out; }
}
