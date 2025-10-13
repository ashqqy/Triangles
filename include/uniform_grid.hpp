#pragma once

#include <vector>
#include <set>

#include "common.hpp"
#include "geometry.hpp"

// ---------------------------------------------------------------------------------------------------

template<FloatingPoint T, std::size_t Dim>
requires TriangleValidDimension<Dim>

struct BoundingBox
{
    Geometry::Vector<T, Dim> min;
    Geometry::Vector<T, Dim> max;
    
    BoundingBox() = default;
    
    BoundingBox(const Geometry::Vector<T, Dim>& min_point, const Geometry::Vector<T, Dim>& max_point)
        : min(min_point), max(max_point) {}
};

template<FloatingPoint T, std::size_t Dim>
requires TriangleValidDimension<Dim>

struct UniformGrid
{
  public:
    UniformGrid (const std::vector<Geometry::Triangle<T, Dim>>& triangles) : triangles_(triangles)
    {
        T bounding_space_vertex_coordinate = NextPowerOfTwo(FindTrianglesFarthestCoordinate());

        for (std::size_t i = 0; i < Dim; ++i)
        {
            bounding_space.min[i] = -bounding_space_vertex_coordinate;
            bounding_space.max[i] = bounding_space_vertex_coordinate;
        }

        cells_per_axis_count_ = CalculateOptimalCellsPerAxisCount(triangles.size());

        InitializeGrid();

        for (std::size_t i = 0; i < triangles_.size(); ++i)
        {
            AddTriangleToCells(i);
        }
    }

    std::set<std::size_t> FindIntersectingTriangles() const
    {
        std::set<std::size_t> intersecting_triangles;
        
        for (const auto& cell : cells_)
        {
            if (cell.size() > 1)
            {
                for (std::size_t i = 0; i < cell.size(); ++i)
                {
                    for (std::size_t j = i + 1; j < cell.size(); ++j)
                    {
                        if (triangles_[cell[i]].CheckIntersection(triangles_[cell[j]]))
                        {
                            intersecting_triangles.insert (cell[i]);
                            intersecting_triangles.insert (cell[j]);
                        }
                    }
                }
            }
        }
        
        return intersecting_triangles;
    }

  private:
    const std::vector<Geometry::Triangle<T, Dim>>& triangles_;
    BoundingBox<T, Dim> bounding_space{};
    std::vector<std::vector<std::size_t>> cells_;
    std::size_t cells_per_axis_count_{};

    static BoundingBox<T, Dim> CalculateBoundingBox(const Geometry::Triangle<T, Dim>& triangle)
    {
        Geometry::Vector<T, Dim> min_point = triangle.a;
        Geometry::Vector<T, Dim> max_point = triangle.a;
        
        const Geometry::Vector<T, Dim> vertices[] = {triangle.a, triangle.b, triangle.c};
        for (const auto& vertex : vertices)
        {
            for (std::size_t i = 0; i < Dim; ++i)
            {
                min_point[i] = std::min(min_point[i], vertex[i]);
                max_point[i] = std::max(max_point[i], vertex[i]);
            }
        }
        
        return {min_point, max_point};
    }

    T FindTrianglesFarthestCoordinate() const
    {
        Geometry::Vector<T, Dim> min_coords{triangles_[0].a};
        Geometry::Vector<T, Dim> max_coords = min_coords;

        for (std::size_t i = 0; i < triangles_.size(); ++i)
        {
            BoundingBox bounding_box = CalculateBoundingBox(triangles_[i]);
            
            for (std::size_t dim = 0; dim < Dim; ++dim)
            {
                min_coords[dim] = std::min(min_coords[dim], bounding_box.min[dim]);
                max_coords[dim] = std::max(max_coords[dim], bounding_box.max[dim]);
            }
        }

        T farthest_coordinate{};

        for (std::size_t i = 0; i < Dim; ++i)
        {
            farthest_coordinate = std::max(farthest_coordinate, std::max(std::fabs(min_coords[i]), std::fabs(max_coords[i])));
        }

        return farthest_coordinate;
    }

    // For around 8 triangles per cell
    static std::size_t CalculateOptimalCellsPerAxisCount(std::size_t triangle_count)
    {
        T triangles_per_cell = 8.0;
        T cells_per_axis = std::pow(triangle_count / triangles_per_cell, 1.0 / Dim);
        
        return static_cast<std::size_t>(std::ceil(cells_per_axis));
    }

    void InitializeGrid()
    {
        std::size_t total_cells = 1;
        for (std::size_t i = 0; i < Dim; ++i) {
            total_cells *= cells_per_axis_count_;
        }
        
        cells_.resize(total_cells);
    }

    void AddTriangleToCells(std::size_t triangle_idx)
    {
        const Geometry::Triangle<T, Dim>& triangle = triangles_[triangle_idx];
        BoundingBox bounding_box = CalculateBoundingBox(triangle);

        // Find the range of cells in which a triangle falls
        std::array<int, Dim> min_cell, max_cell;
        for (std::size_t dim = 0; dim < Dim; ++dim)
        {
            T normalized_min = (bounding_box.min[dim] - bounding_space.min[dim]) / (bounding_space.max[dim] - bounding_space.min[dim]);
            T normalized_max = (bounding_box.max[dim] - bounding_space.min[dim]) / (bounding_space.max[dim] - bounding_space.min[dim]);
            
            min_cell[dim] = static_cast<int>(std::floor(normalized_min * cells_per_axis_count_));
            max_cell[dim] = static_cast<int>(std::ceil(normalized_max * cells_per_axis_count_)) - 1;
        }

        // Add triangle to cells
        if constexpr (Dim == 2)
        {
            for (int x = min_cell[0]; x <= max_cell[0]; ++x) 
            {
                for (int y = min_cell[1]; y <= max_cell[1]; ++y)
                {
                    cells_[x + y * cells_per_axis_count_].push_back(triangle_idx);
                }
            }
        } 
        else if constexpr (Dim == 3)
        {
            for (int x = min_cell[0]; x <= max_cell[0]; ++x)
            {
                for (int y = min_cell[1]; y <= max_cell[1]; ++y)
                {
                    for (int z = min_cell[2]; z <= max_cell[2]; ++z)
                    {
                        cells_[x + y * cells_per_axis_count_ + 
                                   z * cells_per_axis_count_ * cells_per_axis_count_].push_back(triangle_idx);
                    }
                }
            }
        }
    }
};
