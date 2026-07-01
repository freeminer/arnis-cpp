#pragma once

#include <cstddef>
#include <optional>
#include <unordered_map>
#include <utility>

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"
#include "bridge_styles.h"

namespace arnis::bridges
{

struct BridgeMemberInfo {
    int deck_y = 0;
    bridge_styles::BridgeStyle style = bridge_styles::BridgeStyle::Beam;
    std::optional<int> start_internal_ramp;
    std::optional<int> end_internal_ramp;

    int y_at(std::size_t tds, std::size_t total_bresenham, std::size_t ramp_length) const;
};

struct BridgeRampInfo {
    bool bridge_side_at_start = false;
    int deck_y = 0;
    int ground_y = 0;

    int y_at(std::size_t tds, std::size_t total_bresenham) const;
};

class BridgeStructureMap {
public:
    static BridgeStructureMap build(
            const std::vector<ProcessedElement> &elements, const WorldEditor &editor);
    static BridgeStructureMap build(const std::vector<ProcessedElement> &elements,
            const WorldEditor &editor, const bridge_styles::BridgeOutlineIndex &outlines);

    const BridgeMemberInfo *lookup_member(std::int64_t way_id) const;
    const BridgeRampInfo *lookup_ramp(std::int64_t way_id) const;

private:
    std::unordered_map<std::int64_t, BridgeMemberInfo> members_;
    std::unordered_map<std::int64_t, BridgeRampInfo> ramps_;
};

class BridgeSurfaceMap {
public:
    static BridgeSurfaceMap build(const std::vector<ProcessedElement> &elements,
            const BridgeStructureMap &structures, double scale);

    std::optional<int> deck_y_at(int x, int z) const;
    std::optional<int> nearby_deck_y(int x, int z, int radius) const;
    bool contains(int x, int z) const { return deck_y_at(x, z).has_value(); }

private:
    struct PairHash {
        std::size_t operator()(const std::pair<int, int> &p) const noexcept
        {
            return std::hash<long long>()(
                    (static_cast<long long>(p.first) << 32) ^
                    static_cast<unsigned long long>(p.second));
        }
    };
    std::unordered_map<std::pair<int, int>, int, PairHash> deck_y_;
};

bool is_bridge_way(const ProcessedWay &way);

}
