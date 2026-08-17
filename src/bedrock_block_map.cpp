#include "bedrock_block_map.h"
#include "../../arnis_block.h"
namespace arnis {
static BedrockBlock simple(const std::string &n){return {"minecraft:"+n,{}};}
static BedrockBlock state(const std::string &n,std::map<std::string,BedrockStateValue> s){return {"minecraft:"+n,std::move(s)};}
BedrockBlock to_bedrock_block(const Block &block){
 const auto n=std::string("block_")+std::to_string(block.id());
 if(n=="short_grass") return state("tallgrass",{{"tall_grass_type",std::string("tall")}});
 if(n=="tall_grass") return state("double_plant",{{"double_plant_type",std::string("grass")}});
 if(n=="sugar_cane") return state("reeds",{{"age",0}});
 if(n=="oak_leaves"||n=="birch_leaves") return state("leaves",{{"old_leaf_type",BedrockStateValue(std::string(n=="oak_leaves"?"oak":"birch"))},{"persistent_bit",BedrockStateValue(true)},{"update_bit",BedrockStateValue(false)}});
 if(n=="grass_block"||n=="stone"||n=="dirt") return simple(n);
 return simple(n);
}
}
