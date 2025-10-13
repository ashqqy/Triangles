#include <iostream>
#include <set>
#include <vector>

#include "geometry.hpp"
#include "uniform_grid.hpp"

int main ()
{
    std::size_t triangles_count = 0;
    std::cin >> triangles_count;
    if (!(0 < triangles_count && triangles_count < 1000000))
    {
        std::cerr << "Wrong number of triangles (0 < triangles count < 1000000)" << std::endl;
        return 1;
    }

    std::vector<Geometry::Triangle<double, 3>> triangles;
    triangles.reserve(triangles_count);

    for (std::size_t i = 0; i < triangles_count; ++i)
    {
        Geometry::Vector<double, 3> A;
        Geometry::Vector<double, 3> B;
        Geometry::Vector<double, 3> C;

        std::cin >> A >> B >> C;
        
        triangles.emplace_back(A, B, C);
    }

    UniformGrid<double, 3> grid (triangles);
    std::vector<std::pair<std::size_t, std::size_t>> potential_intersections = grid.FindPotentialIntersections();
    std::set<std::size_t> intersecting_triangles;

    for (auto& [first_idx, second_idx]: potential_intersections)
    {
        if (triangles[first_idx].CheckIntersection(triangles[second_idx]))
        {
            intersecting_triangles.insert (first_idx);
            intersecting_triangles.insert (second_idx);
        }
    }

    for (auto i: intersecting_triangles)
    {
        std::cout << i << std::endl;
    }
}
