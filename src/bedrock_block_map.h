#pragma once
#include "block_definitions.h"
#include <map>
#include <string>
#include <variant>
#include <functional>
#include <type_traits>
namespace arnis {
using BedrockStateValue=std::variant<std::string,bool,int>;
struct BedrockBlock { std::string name; std::map<std::string,BedrockStateValue> states; };
inline bool operator==(const BedrockBlock &a,const BedrockBlock &b){return a.name==b.name&&a.states==b.states;}
struct BedrockBlockHash {
	std::size_t operator()(const BedrockBlock &b) const noexcept {
		std::size_t h=std::hash<std::string>{}(b.name);
		for(const auto &kv:b.states){
			h ^= std::hash<std::string>{}(kv.first)+0x9e3779b9+(h<<6)+(h>>2);
			std::visit([&](const auto &v){ using V=std::decay_t<decltype(v)>; h ^= std::hash<V>{}(v)+0x9e3779b9+(h<<6)+(h>>2); },kv.second);
		}
		return h;
	}
};
BedrockBlock to_bedrock_block(const Block &block);
}
