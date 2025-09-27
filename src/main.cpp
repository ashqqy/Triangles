#include <iostream>
#include <set>
#include <vector>

#include "triangles.hpp"

int main ()
{
    size_t triangles_count = 0;
    std::cin >> triangles_count;
    if (!(0 < triangles_count && triangles_count < 1000000))
    {
        std::cerr << "Wrong number of triangles (0 < triangles count < 1000000)" << std::endl;
        return 1;
    }

    std::vector<Triangles::Triangle> triangles;
    triangles.reserve (triangles_count);

    for (size_t i = 0; i < triangles_count; ++i)
    {
        double x1, y1, z1, x2, y2, z2, x3, y3, z3;
        std::cin >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3;

        Triangles::Point A (x1, y1, z1);
        Triangles::Point B (x2, y2, z2);
        Triangles::Point C (x3, y3, z3);
        
        triangles.emplace_back (A, B, C);
    }

    std::set<size_t> intersecting_triangles;

    for (size_t i = 0; i < triangles_count; ++i)
    {
        for (size_t j = i + 1; j < triangles_count; ++j)
        {
            if (Triangles::CheckTrianglesIntersection (triangles[i], triangles[j]))
            {
                intersecting_triangles.insert (i);
                intersecting_triangles.insert (j);
            }
        }
    }

    for (auto i: intersecting_triangles)
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;
}
