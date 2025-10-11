#include <iostream>
#include <set>
#include <vector>

#include "triangles.hpp"

int main ()
{
    std::size_t triangles_count = 0;
    std::cin >> triangles_count;
    if (!(0 < triangles_count && triangles_count < 1000000))
    {
        std::cerr << "Wrong number of triangles (0 < triangles count < 1000000)" << std::endl;
        return 1;
    }

    std::vector<Triangles::Triangle<double, 3>> triangles;
    triangles.reserve(triangles_count);

    for (std::size_t i = 0; i < triangles_count; ++i)
    {
        Triangles::Vector<double, 3> A;
        Triangles::Vector<double, 3> B;
        Triangles::Vector<double, 3> C;

        std::cin >> A >> B >> C;
        
        triangles.emplace_back(A, B, C);
    }

    std::set<std::size_t> intersecting_triangles;

    for (std::size_t i = 0; i < triangles_count; ++i)
    {
        for (std::size_t j = i + 1; j < triangles_count; ++j)
        {
            if (triangles[i].CheckIntersection(triangles[j]))
            {
                intersecting_triangles.insert (i);
                intersecting_triangles.insert (j);
            }
        }
    }

    for (auto i: intersecting_triangles)
    {
        std::cout << i << std::endl;
    }
}
