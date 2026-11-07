#pragma once

#include "bridges.h"

namespace arnis::bridges
{
// Complete source-geometry analysis shared by streamed Freeminer chunks.
// Clipped render ways retain their OSM IDs, so they can look up these profiles
// instead of independently inventing incompatible short-bridge decks.
struct BridgeContext
{
	bridge_styles::BridgeOutlineIndex outlines;
	BridgeStructureMap structures;
	BridgeSurfaceMap surface;
};
} // namespace arnis::bridges
