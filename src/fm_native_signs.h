#pragma once

#include "block_definitions.h"
#include <map>
#include <string>

namespace arnis::block_definitions
{
// Filled from registered nodes by the host before generation starts.
inline std::map<std::string, Block> native_sign_nodes;

inline Block native_sign_node(const std::string &name)
{
	const auto it = native_sign_nodes.find(name);
	return it == native_sign_nodes.end() ? AIR : it->second;
}
} // namespace arnis::block_definitions
