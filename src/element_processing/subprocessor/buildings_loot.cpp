#include "buildings_loot.h"
#include "../../deterministic_rng.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace arnis::buildings_loot {
namespace {
constexpr unsigned COMMON=9, UNCOMMON=3, RARE=1, EMPTY_WEIGHT=3;
struct Item { const char *id; int min,max; unsigned weight; };
struct Theme { unsigned weight; const Item *items; std::size_t count; };
#define I(id,a,b,w) Item{id,a,b,w}
static constexpr Item food[]={I("minecraft:bread",2,6,COMMON),I("minecraft:potato",3,9,COMMON),I("minecraft:carrot",2,7,COMMON),I("minecraft:wheat",3,9,COMMON),I("minecraft:apple",2,6,COMMON),I("minecraft:baked_potato",2,6,COMMON),I("minecraft:cooked_chicken",1,4,COMMON),I("minecraft:sweet_berries",2,7,COMMON),I("minecraft:beetroot",2,6,COMMON),I("minecraft:pumpkin_pie",1,3,UNCOMMON),I("minecraft:mushroom_stew",1,1,UNCOMMON),I("minecraft:golden_carrot",1,3,RARE),I("minecraft:cake",1,1,RARE)};
static constexpr Item junk[]={I("minecraft:paper",2,7,COMMON),I("minecraft:bone",2,7,COMMON),I("minecraft:string",2,7,COMMON),I("minecraft:rotten_flesh",2,6,COMMON),I("minecraft:book",1,4,COMMON),I("minecraft:dead_bush",1,3,COMMON),I("minecraft:gunpowder",1,4,COMMON),I("minecraft:flower_pot",1,1,UNCOMMON),I("minecraft:cobweb",1,3,UNCOMMON),I("minecraft:name_tag",1,1,UNCOMMON),I("minecraft:map",1,1,UNCOMMON)};
static constexpr Item building[]={I("minecraft:oak_planks",4,16,COMMON),I("minecraft:cobblestone",6,20,COMMON),I("minecraft:coal",3,9,COMMON),I("minecraft:clay_ball",2,7,COMMON),I("minecraft:glass_pane",3,9,COMMON),I("minecraft:torch",2,8,COMMON),I("minecraft:iron_ingot",2,6,UNCOMMON),I("minecraft:candle",1,4,UNCOMMON)};
static constexpr Item tools[]={I("minecraft:stick",2,7,COMMON),I("minecraft:bucket",1,1,UNCOMMON),I("minecraft:fishing_rod",1,1,UNCOMMON),I("minecraft:shears",1,1,UNCOMMON),I("minecraft:flint_and_steel",1,1,UNCOMMON),I("minecraft:compass",1,1,UNCOMMON),I("minecraft:iron_pickaxe",1,1,RARE),I("minecraft:iron_axe",1,1,RARE),I("minecraft:clock",1,1,RARE)};
static constexpr Item valuables[]={I("minecraft:iron_nugget",3,8,COMMON),I("minecraft:gold_nugget",2,7,UNCOMMON),I("minecraft:lapis_lazuli",2,6,UNCOMMON),I("minecraft:emerald",1,4,UNCOMMON),I("minecraft:gold_ingot",1,3,RARE),I("minecraft:amethyst_shard",1,4,RARE),I("minecraft:diamond",1,2,RARE)};
static constexpr Item adventure[]={I("minecraft:arrow",3,12,COMMON),I("minecraft:leather_boots",1,1,UNCOMMON),I("minecraft:leather_chestplate",1,1,UNCOMMON),I("minecraft:shield",1,1,RARE),I("minecraft:golden_apple",1,1,RARE),I("minecraft:ender_pearl",1,3,RARE)};
static constexpr Theme themes[]={{25,food,std::size(food)},{20,junk,std::size(junk)},{18,building,std::size(building)},{15,tools,std::size(tools)},{12,valuables,std::size(valuables)},{10,adventure,std::size(adventure)}};
#undef I
template<class R> const Item &pick_item(const Theme &t,R &rng) { unsigned total=0; for(std::size_t i=0;i<t.count;++i) total+=t.items[i].weight; unsigned p=rng()%total; for(std::size_t i=0;i<t.count;++i){if(p<t.items[i].weight)return t.items[i];p-=t.items[i].weight;} return t.items[t.count-1]; }
}
std::vector<LootEntry> chest_loot(int x,int z,unsigned salt) {
 auto rng=coord_rng(x,z,std::uint64_t(salt)^0x1007C0DEULL); constexpr unsigned total=25+20+18+15+12+10+EMPTY_WEIGHT; unsigned rolls=3+rng()%6; std::array<bool,27> used{}; std::vector<LootEntry> out;
 for(unsigned i=0;i<rolls;++i){ unsigned p=rng()%total; if(p<EMPTY_WEIGHT)continue; p-=EMPTY_WEIGHT; const Theme *theme=&themes[0]; for(const Theme &candidate:themes){if(p<candidate.weight){theme=&candidate;break;}p-=candidate.weight;} const Item &item=pick_item(*theme,rng); int slot=-1; for(int a=0;a<4;++a){int c=int(rng()%27);if(!used[c]){slot=c;break;}} if(slot<0)continue; used[slot]=true; int count=item.min+int(rng()%unsigned(item.max-item.min+1)); out.push_back({item.id,slot,count}); }
 return out;
}
}
