#include "bresenham.h"
namespace arnis {
namespace bresenham {


std::vector<std::tuple<int,int,int>> bresenham_line(
    int x1, int y1, int z1,
    int x2, int y2, int z2)
{
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int dz = std::abs(z2 - z1);

    int capacity = std::max({dx, dy, dz}) + 1;
    std::vector<std::tuple<int,int,int>> points;
    points.reserve(capacity);

    int xs = (x1 < x2) ? 1 : -1;
    int ys = (y1 < y2) ? 1 : -1;
    int zs = (z1 < z2) ? 1 : -1;

    int x = x1, y = y1, z = z1;

    // Choose dominant axis and step along it
    if (dx >= dy && dx >= dz) {
        int p1 = 2 * dy - dx;
        int p2 = 2 * dz - dx;
        while (x != x2) {
            points.emplace_back(x, y, z);
            if (p1 >= 0) { y += ys; p1 -= 2 * dx; }
            if (p2 >= 0) { z += zs; p2 -= 2 * dx; }
            p1 += 2 * dy;
            p2 += 2 * dz;
            x += xs;
        }
    } else if (dy >= dx && dy >= dz) {
        int p1 = 2 * dx - dy;
        int p2 = 2 * dz - dy;
        while (y != y2) {
            points.emplace_back(x, y, z);
            if (p1 >= 0) { x += xs; p1 -= 2 * dy; }
            if (p2 >= 0) { z += zs; p2 -= 2 * dy; }
            p1 += 2 * dx;
            p2 += 2 * dz;
            y += ys;
        }
    } else {
        int p1 = 2 * dy - dz;
        int p2 = 2 * dx - dz;
        while (z != z2) {
            points.emplace_back(x, y, z);
            if (p1 >= 0) { y += ys; p1 -= 2 * dz; }
            if (p2 >= 0) { x += xs; p2 -= 2 * dz; }
            p1 += 2 * dy;
            p2 += 2 * dx;
            z += zs;
        }
    }

    // push the final endpoint
    points.emplace_back(x2, y2, z2);
    return points;
}

}
}