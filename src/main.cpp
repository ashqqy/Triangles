#include <iostream>
#include <set>
#include <vector>

#include "triangles_intersection.hpp"

int main ()
{
    auto triangles = Intersection::InputTriangles<double, 3>();

    std::set<std::size_t> intersecting_triangles = Intersection::FindIntersectingTriangles(triangles);

    for (auto i: intersecting_triangles)
    {
        std::cout << i << std::endl;
    }
}
