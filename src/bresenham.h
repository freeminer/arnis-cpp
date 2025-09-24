#include <vector>
#include <tuple>
#include <algorithm>

namespace arnis
{
namespace bresenham
{
// Function to generate a line between two 3D points using Bresenham's algorithm

std::vector<std::tuple<int, int, int>> bresenham_line(
		int x1, int y1, int z1, int x2, int y2, int z2);

}

using namespace bresenham;
}