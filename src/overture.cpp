#include "overture.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_map>

namespace arnis::overture
{

namespace
{

constexpr std::size_t MAX_OVERTURE_BUILDINGS = 100000;

struct OvertureBuilding {
    std::string id;
    std::vector<std::pair<double, double>> exterior_ring;
    std::optional<double> height;
    std::optional<double> min_height;
    std::optional<int> num_floors;
    std::optional<std::string> subtype;
    std::optional<std::string> clazz;
    std::optional<std::string> roof_shape;
    std::optional<std::string> roof_material;
    std::optional<std::string> roof_orientation;
    std::optional<std::string> facade_color;
    std::optional<std::string> roof_color;
};

uint32_t read_u32(const std::vector<uint8_t> &bytes, std::size_t offset, bool little_endian)
{
    if (offset + 4 > bytes.size())
        return 0;
    if (little_endian) {
        return static_cast<uint32_t>(bytes[offset]) |
               (static_cast<uint32_t>(bytes[offset + 1]) << 8) |
               (static_cast<uint32_t>(bytes[offset + 2]) << 16) |
               (static_cast<uint32_t>(bytes[offset + 3]) << 24);
    }
    return (static_cast<uint32_t>(bytes[offset]) << 24) |
           (static_cast<uint32_t>(bytes[offset + 1]) << 16) |
           (static_cast<uint32_t>(bytes[offset + 2]) << 8) |
           static_cast<uint32_t>(bytes[offset + 3]);
}

double read_f64(const std::vector<uint8_t> &bytes, std::size_t offset, bool little_endian)
{
    std::array<uint8_t, 8> raw{};
    if (offset + 8 > bytes.size())
        return 0.0;
    if (little_endian) {
        std::copy_n(bytes.begin() + static_cast<std::ptrdiff_t>(offset), 8, raw.begin());
    } else {
        std::reverse_copy(bytes.begin() + static_cast<std::ptrdiff_t>(offset),
                bytes.begin() + static_cast<std::ptrdiff_t>(offset + 8), raw.begin());
    }
    double out = 0.0;
    std::memcpy(&out, raw.data(), sizeof(out));
    return out;
}

[[maybe_unused]] std::optional<std::vector<std::pair<double, double>>> parse_wkb_polygon(
        const std::vector<uint8_t> &wkb)
{
    if (wkb.size() < 13)
        return std::nullopt;

    const uint8_t byte_order = wkb[0];
    if (byte_order > 1)
        return std::nullopt;
    const bool little_endian = byte_order == 1;

    const uint32_t geom_type = read_u32(wkb, 1, little_endian);
    if ((geom_type % 1000) != 3)
        return std::nullopt;

    const uint32_t num_rings = read_u32(wkb, 5, little_endian);
    if (num_rings == 0)
        return std::nullopt;

    std::size_t offset = 9;
    if (offset + 4 > wkb.size())
        return std::nullopt;
    const uint32_t num_points = read_u32(wkb, offset, little_endian);
    offset += 4;

    const uint32_t dimension = geom_type / 1000;
    const bool has_z = dimension == 1 || dimension == 3;
    const bool has_m = dimension == 2 || dimension == 3;
    const std::size_t point_size = 16 + (has_z ? 8 : 0) + (has_m ? 8 : 0);
    if (offset + static_cast<std::size_t>(num_points) * point_size > wkb.size())
        return std::nullopt;

    std::vector<std::pair<double, double>> coords;
    coords.reserve(num_points);
    for (uint32_t i = 0; i < num_points; ++i) {
        const double lng = read_f64(wkb, offset, little_endian);
        const double lat = read_f64(wkb, offset + 8, little_endian);
        coords.emplace_back(lng, lat);
        offset += point_size;
    }
    return coords;
}

[[maybe_unused]] uint64_t gers_id_to_u64(const std::string &gers_id)
{
    constexpr uint64_t FNV_OFFSET = 0xcbf29ce484222325ULL;
    constexpr uint64_t FNV_PRIME = 0x100000001b3ULL;

    uint64_t hash = FNV_OFFSET;
    for (unsigned char byte : gers_id) {
        hash ^= static_cast<uint64_t>(byte);
        hash *= FNV_PRIME;
    }
    return hash | OVERTURE_ID_HIGH_BIT;
}

[[maybe_unused]] std::string overture_class_to_osm_building(
        const std::optional<std::string> &subtype,
        const std::optional<std::string> &clazz)
{
    if (clazz) {
        const auto &c = *clazz;
        if (c == "house" || c == "detached") return "house";
        if (c == "apartments" || c == "apartment") return "apartments";
        if (c == "residential") return "residential";
        if (c == "commercial") return "commercial";
        if (c == "retail") return "retail";
        if (c == "office") return "office";
        if (c == "industrial") return "industrial";
        if (c == "warehouse") return "warehouse";
        if (c == "garage" || c == "garages") return "garage";
        if (c == "shed") return "shed";
        if (c == "school") return "school";
        if (c == "hospital") return "hospital";
        if (c == "church" || c == "mosque" || c == "temple" || c == "synagogue")
            return "church";
        if (c == "hotel") return "hotel";
        if (c == "farm" || c == "barn") return "farm";
    }

    if (subtype) {
        const auto &s = *subtype;
        if (s == "residential") return "residential";
        if (s == "commercial") return "commercial";
        if (s == "industrial") return "industrial";
        if (s == "agricultural") return "farm";
        if (s == "civic" || s == "government" || s == "education") return "public";
        if (s == "medical") return "hospital";
        if (s == "religious") return "church";
        if (s == "transportation") return "transportation";
        if (s == "outbuilding") return "shed";
    }
    return "yes";
}

}

std::vector<ProcessedElement> fetch_overture_buildings(double, double, double, double,
        double, bool debug)
{
    if (debug) {
        std::cerr << "Overture Maps fetching is not enabled in the C++ mapgen build "
                     "(Parquet/HTTP range reader not linked). Self-contained WKB, ID, "
                     "tag mapping, and OSM dedup helpers are ported, but live STAC "
                     "partition reads still require a C++ Parquet backend. Continuing "
                     "with OSM data only. Limit would be " << MAX_OVERTURE_BUILDINGS
                  << " buildings."
                  << std::endl;
    }
    return {};
}

std::vector<ProcessedElement> deduplicate_against_osm(
        std::vector<ProcessedElement> overture_elements,
        const std::vector<ProcessedElement> &osm_elements)
{
    struct BBox {
        int min_x;
        int min_z;
        int max_x;
        int max_z;
    };

    std::vector<BBox> osm_buildings;
    for (const auto &element : osm_elements) {
        if (!element.is_way())
            continue;
        const auto &way = element.as_way();
        if (!(way.tags.contains("building") || way.tags.contains("building:part")) ||
                way.nodes.size() < 3)
            continue;
        BBox b{way.nodes.front().x, way.nodes.front().z, way.nodes.front().x,
                way.nodes.front().z};
        for (const auto &node : way.nodes) {
            b.min_x = std::min(b.min_x, node.x);
            b.min_z = std::min(b.min_z, node.z);
            b.max_x = std::max(b.max_x, node.x);
            b.max_z = std::max(b.max_z, node.z);
        }
        osm_buildings.push_back(b);
    }

    if (osm_buildings.empty())
        return overture_elements;

    constexpr int CELL_SIZE = 64;
    int grid_min_x = osm_buildings.front().min_x;
    int grid_min_z = osm_buildings.front().min_z;
    for (const auto &b : osm_buildings) {
        grid_min_x = std::min(grid_min_x, b.min_x);
        grid_min_z = std::min(grid_min_z, b.min_z);
    }

    struct PairHash {
        std::size_t operator()(const std::pair<int, int> &p) const noexcept
        {
            return std::hash<long long>()(
                    (static_cast<long long>(p.first) << 32) ^
                    static_cast<unsigned int>(p.second));
        }
    };
    std::unordered_map<std::pair<int, int>, std::vector<std::size_t>, PairHash> grid;
    for (std::size_t i = 0; i < osm_buildings.size(); ++i) {
        const auto &b = osm_buildings[i];
        const int cell_x0 = (b.min_x - grid_min_x) / CELL_SIZE;
        const int cell_z0 = (b.min_z - grid_min_z) / CELL_SIZE;
        const int cell_x1 = (b.max_x - grid_min_x) / CELL_SIZE;
        const int cell_z1 = (b.max_z - grid_min_z) / CELL_SIZE;
        for (int cx = cell_x0; cx <= cell_x1; ++cx)
            for (int cz = cell_z0; cz <= cell_z1; ++cz)
                grid[{cx, cz}].push_back(i);
    }

    std::vector<ProcessedElement> out;
    out.reserve(overture_elements.size());
    for (auto &element : overture_elements) {
        if (!element.is_way()) {
            out.push_back(std::move(element));
            continue;
        }
        const auto &way = element.as_way();
        if (way.nodes.empty())
            continue;
        long long sum_x = 0;
        long long sum_z = 0;
        for (const auto &node : way.nodes) {
            sum_x += node.x;
            sum_z += node.z;
        }
        const int cx = static_cast<int>(sum_x / static_cast<long long>(way.nodes.size()));
        const int cz = static_cast<int>(sum_z / static_cast<long long>(way.nodes.size()));
        const auto key = std::make_pair((cx - grid_min_x) / CELL_SIZE,
                (cz - grid_min_z) / CELL_SIZE);

        bool duplicate = false;
        if (auto it = grid.find(key); it != grid.end()) {
            for (std::size_t idx : it->second) {
                const auto &b = osm_buildings[idx];
                if (cx >= b.min_x && cx <= b.max_x && cz >= b.min_z && cz <= b.max_z) {
                    duplicate = true;
                    break;
                }
            }
        }
        if (!duplicate)
            out.push_back(std::move(element));
    }
    return out;
}

}
