#include <iostream>
#include <set>
#include <vector>

#include "geometry.hpp"
#include "uniform_grid.hpp"

template<FloatingPoint T, std::size_t Dim>
requires TriangleValidDimension<Dim>

std::set<std::size_t> FindIntersectingTriangles(std::vector<Geometry::Triangle<T, Dim>> triangles)
{
    UniformGrid<T, Dim> grid (triangles);
    return grid.FindIntersectingTriangles();
}

template<FloatingPoint T, std::size_t Dim>
requires TriangleValidDimension<Dim>

std::set<std::size_t> FindIntersectingTrianglesNaive(std::vector<Geometry::Triangle<T, Dim>> triangles)
{
    std::set<std::size_t> intersecting_triangles;

    for (std::size_t i = 0; i < triangles.size(); ++i)
    {
        for (std::size_t j = i + 1; j < triangles.size(); ++j)
        {
            if (triangles[i].CheckIntersection(triangles[j]))
            {
                intersecting_triangles.insert(i);
                intersecting_triangles.insert(j);
            }
        }
    }

    return intersecting_triangles;
}

template<FloatingPoint T, std::size_t Dim>
requires TriangleValidDimension<Dim>

std::vector<Geometry::Triangle<T, Dim>> InputTriangles()
{
    std::size_t triangles_count = 0;
    std::cin >> triangles_count;
    if (!(0 <= triangles_count && triangles_count <= 1000000))
    {
        std::cerr << "Wrong number of triangles (0 < triangles count < 1000000)" << std::endl;
        return {};
    }

    std::vector<Geometry::Triangle<T, Dim>> triangles;
    triangles.reserve(triangles_count);

    for (std::size_t triangle_idx = 0; triangle_idx < triangles_count; ++triangle_idx)
    {
        Geometry::Vector<T, Dim> A;
        Geometry::Vector<T, Dim> B;
        Geometry::Vector<T, Dim> C;

        std::cin >> A >> B >> C;
        
        triangles.emplace_back(A, B, C);
    }

    return triangles;
}

int main ()
{
    auto triangles = InputTriangles<double, 3>();

    std::set<std::size_t> intersecting_triangles = FindIntersectingTriangles(triangles);

    for (auto i: intersecting_triangles)
    {
        std::cout << i << std::endl;
    }

    std::cout << intersecting_triangles.size() << std::endl;
}
