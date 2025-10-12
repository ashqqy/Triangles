#pragma once

#include <array>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <utility>
#include <iostream>

namespace Triangles
{
// --------------------------------------------- Concepts --------------------------------------------

template<typename T>
concept FloatingPoint = std::floating_point<T>;

template<std::size_t Dim>
concept VectorValidDimension = (1 <= Dim && Dim <= 3);

template<std::size_t Dim>
concept TriangleValidDimension = (Dim == 2 || Dim == 3);

// ---------------------------------------- Double comparsion ----------------------------------------

template<FloatingPoint T>
inline bool FloatingPointLE(T left, T right, T epsilon = std::numeric_limits<T>::epsilon())
{
    T max_val = std::max({std::abs(left), std::abs(right), T(1.0)});
    return left < right + epsilon * max_val;
}

template<FloatingPoint T>
inline bool FloatingPointL(T left, T right, T epsilon = std::numeric_limits<T>::epsilon())
{
    T max_val = std::max({std::abs(left), std::abs(right), T(1.0)});
    return left < right - epsilon * max_val;
}

template<FloatingPoint T>
inline bool FloatingPointGE(T left, T right, T epsilon = std::numeric_limits<T>::epsilon())
{
    T max_val = std::max({std::abs(left), std::abs(right), T(1.0)});
    return left > right - epsilon * max_val;
}

template<FloatingPoint T>
inline bool FloatingPointG(T left, T right, T epsilon = std::numeric_limits<T>::epsilon())
{
    T max_val = std::max({std::abs(left), std::abs(right), T(1.0)});
    return left > right + epsilon * max_val;
}

template<FloatingPoint T>
inline bool FloatingPointE(T a, T b, T epsilon = std::numeric_limits<T>::epsilon())
{
    T max_val = std::max({std::abs(a), std::abs(b), T(1.0)});
    return std::abs(a - b) < epsilon * max_val;
}

// --------------------------------------------- Vector ----------------------------------------------

template<FloatingPoint T, std::size_t Dim>
requires VectorValidDimension<Dim>

struct Vector
{
  public:
    std::array<T, Dim> coords{};

    Vector() = default;
    Vector(std::initializer_list<T> init)
    {
        assert(init.size() == Dim);

        std::size_t i = 0;
        for (const auto& val : init)
        {
            if (i < Dim)
                coords[i++] = val;
        }
    }
    Vector(const Vector<T, Dim>& begin, const Vector<T, Dim>& end) : Vector(end - begin) {}

    Vector operator+(const Vector& other) const
    {
        Vector result;
        for (std::size_t i = 0 ; i < Dim; ++i)
        {
            result.coords[i] = coords[i] + other.coords[i];
        }
        return result;
    }

    Vector operator-(const Vector& other) const
    {
        Vector result;
        for (std::size_t i = 0 ; i < Dim; ++i)
            result.coords[i] = coords[i] - other.coords[i];

        return result;
    }

    Vector operator/(T scalar) const
    {
        assert(!FloatingPointE(scalar, T{0}) && "Division by zero");

        Vector result;
        for (std::size_t i = 0 ; i < Dim; ++i)
            result.coords[i] = coords[i] / scalar;

        return result;
    }

    Vector operator*(T scalar) const
    {
        Vector result;
        for (std::size_t i = 0 ; i < Dim; ++i)
            result.coords[i] = coords[i] * scalar;

        return result;
    }

    bool operator==(const Vector& other) const
    {
        if (this == &other)
            return true;

        for (std::size_t i = 0; i < Dim; ++i)
            if (!FloatingPointE(coords[i], other.coords[i]))
                return false;

        return true;
    }

    T& operator[](std::size_t index)
    {
        return coords[index];
    }

    const T& operator[](std::size_t index) const
    {
        return coords[index];
    }

    friend std::istream& operator>>(std::istream& is, Vector<T, Dim>& vec)
    {
        for (std::size_t i = 0; i < Dim; ++i)
            if (!(is >> vec[i]))
                break;

        return is;
    }

    friend std::ostream& operator<<(std::ostream& os, Vector<T, Dim>& vec)
    {
        std::cout << "(";
        for (std::size_t i = 0; i < Dim - 1; ++i)
        {
            std::cout << vec[i] << ", ";
        }
        std::cout << vec[Dim - 1] << ")";

        return os;
    }

    T DotProduct(const Vector<T, Dim>& other) const
    {
        T result = 0;
        for (std::size_t i = 0; i < Dim; ++i)
            result += (*this)[i] * other[i]; 

        return result;
    }

    auto CrossProduct(const Vector<T, Dim>& other) const
    {
        if constexpr (Dim == 3)
        {
            T c_x = (*this)[1] * other[2] - (*this)[2] * other[1];
            T c_y = (*this)[2] * other[0] - (*this)[0] * other[2];
            T c_z = (*this)[0] * other[1] - (*this)[1] * other[0];
            return Vector<T, 3>{c_x, c_y, c_z};
        }
        else if constexpr (Dim == 2)
        {
            return T{(*this)[0] * other[1] - (*this)[1] * other[0]};
        }
        else // Dim == 1
        {
            return T{0};
        }
    }

    bool IsZero() const 
    {
        for (const auto& coord : coords)
            if (!FloatingPointE(coord, T{0}))
                return false;

        return true;
    }

    bool IsParallel(const Vector<T, Dim>& other) const 
    {
        if constexpr (Dim == 3)
        {
            return CrossProduct(other).IsZero();
        }

        if constexpr (Dim == 2)
        {
            return FloatingPointE(CrossProduct(other), T{0});
        }

        return true;
    }

    T LengthSquared() const
    {
        return DotProduct(*this);
    }
};

// ------------------------------------------ Line segment -------------------------------------------

template<FloatingPoint T, std::size_t Dim>
requires VectorValidDimension<Dim>

struct LineSegment
{
  public:
    Vector<T, Dim> begin{};
    Vector<T, Dim> end{};
    Vector<T, Dim> direction{};

    LineSegment() = default;

    LineSegment(Vector<T, Dim> from, Vector<T, Dim> to) :
        begin(from), end(to), direction(to - from) {} 

    T LengthSquared() const
    {
        return direction.DotProduct(direction);
    }

    bool ContainsPoint(const Vector<T, Dim>& point) const
    {
        Vector<T, Dim> connection {begin, point};

        if (!direction.IsParallel(connection))
        {
            return false;
        }

        T connection_dot_direction = connection.DotProduct(direction);

        return FloatingPointLE(T{0}, connection_dot_direction) && 
               FloatingPointLE(connection_dot_direction, direction.DotProduct(direction));
    }
    
    // Dim == 2 || Dim == 3
    bool CheckIntersection(const LineSegment<T, Dim>& other) const
    {
        if constexpr (Dim == 2)
            return CheckIntersection2D(other);
        else
            return CheckIntersection3D(other);
    }

  private:
    bool CheckIntersection3D(const LineSegment<T, 3>& other) const
    {
        Vector segments_cross = direction.CrossProduct(other.direction);
        Vector vec_between_segments(begin, other.begin);
        T t1{}, t2{};

        // Check if segments are parallel
        if (segments_cross.IsZero())
        {
            return CheckParallelSegmentsIntersection3D(other);
        }

        // Find the volume of the parallelepiped formed by these segments
        // volume = scalar triple product = dot (vec_between_segments, cross (direction, other.direction)) = dot (vec_between_segments, segments_cross)
        T volume = vec_between_segments.DotProduct(segments_cross);

        // if (volume != 0) => segments are skew => segments don't intersect
        if (!FloatingPointE(volume, T{0}))
        {
            return false;
        }

        // Segments are co-planar and not parallel

        // Find t1 and t2: 
        // begin1 + t1 * direction = begin2 + t2 * other.direction

        Vector cross1 = vec_between_segments.CrossProduct(direction);
        Vector cross2 = vec_between_segments.CrossProduct(other.direction);

        t1 = cross2.DotProduct(segments_cross) / 
                    segments_cross.DotProduct(segments_cross);

        t2 = cross1.DotProduct(segments_cross) / 
                    segments_cross.DotProduct(segments_cross);

        return (FloatingPointLE(T{0}, t1) && FloatingPointLE(t1, T{1.0}) && 
                FloatingPointLE(T{0}, t2) && FloatingPointLE(t2, T{1.0}));
    }

    bool CheckIntersection2D(const LineSegment<T, 2>& other) const
    {
        T segments_cross = direction.CrossProduct(other.direction);
        Vector vec_between_segments(begin, other.begin);
        T t1{}, t2{};

        // Check if segments are parallel
        if (FloatingPointE(segments_cross, T{0}))
        {
            return CheckParallelSegmentsIntersection2D(other);
        }

        // Segments aren't parallel

        // Find t1 and t2: 
        // begin1 + t1 * direction = begin2 + t2 * other.direction

        T cross1 = vec_between_segments.CrossProduct(direction);
        T cross2 = vec_between_segments.CrossProduct(other.direction);

        t1 = cross2 / segments_cross;
        t2 = cross1 / segments_cross;

        return (FloatingPointLE(T{0}, t1) && FloatingPointLE(t1, T{1.0}) && 
                FloatingPointLE(T{0}, t2) && FloatingPointLE(t2, T{1.0}));
    }

    bool CheckParallelSegmentsIntersection3D(const LineSegment<T, 3>& other) const
    {
        Vector vec_between_begins(begin, other.begin);

        // Check if segments aren't on the same line
        if (!direction.CrossProduct(vec_between_begins).IsZero())
        {
            return false;
        }

        // Make a projection on each axis and check 1D case
        LineSegment<T, 1> proj_x1{begin[0],  end[0]};
        LineSegment<T, 1> proj_y1(begin[1],  end[1]);
        LineSegment<T, 1> proj_z1(begin[2],  end[2]);

        LineSegment<T, 1> proj_x2(other.begin[0], other.end[0]);
        LineSegment<T, 1> proj_y2(other.begin[1], other.end[1]);
        LineSegment<T, 1> proj_z2(other.begin[2], other.end[2]);
        
        return proj_x1.CheckIntersection(proj_x2) &&
               proj_y1.CheckIntersection(proj_y2) &&
               proj_z1.CheckIntersection(proj_z2);
    }

    bool CheckParallelSegmentsIntersection2D(const LineSegment<T, 2>& other) const
    {
        Vector vec_between_begins(begin, other.begin);

        // Check if segments aren't on the same line
        if (!FloatingPointE(direction.CrossProduct(vec_between_begins), T{0}))
        {
            return false;
        }

        // Make a projection on each axis and check 1D case
        LineSegment<T, 1> proj_x1{begin[0],  end[0]};
        LineSegment<T, 1> proj_y1(begin[1],  end[1]);

        LineSegment<T, 1> proj_x2(other.begin[0], other.end[0]);
        LineSegment<T, 1> proj_y2(other.begin[1], other.end[1]);
        
        return proj_x1.CheckIntersection(proj_x2) &&
               proj_y1.CheckIntersection(proj_y2);
    }
};

template<FloatingPoint T>
struct LineSegment<T, 1>
{
    T begin{};
    T end{};

    LineSegment() = default;

    LineSegment(T from, T to) : begin(std::min(from, to)), end(std::max(from, to)) {}

    T Length() const
    {
        return std::abs(end - begin);
    }
    
    bool ContainsPoint(const Vector<T, 1>& point) const
    {
        return ContainsPoint(point[0]);
    }

    bool ContainsPoint(T point) const
    {
            return FloatingPointLE(begin, point) && FloatingPointLE(point, end);
    }

    bool CheckIntersection(const LineSegment<T, 1>& other) const
    {
        return FloatingPointLE(begin, other.end) &&
               FloatingPointLE(other.begin, end);
    }
};

// ---------------------------------------------- Plane ----------------------------------------------

template<FloatingPoint T>

struct Plane
{
    // plane: (normal_, X) + offset_ = 0, where X belong pi
    Vector<T, 3> normal;
    T offset;

  public:
    Plane(const Vector<T, 3>& a, const Vector<T, 3>& b, const Vector<T, 3>& c) : 
        normal (Vector<T, 3>(a, b).CrossProduct(Vector<T, 3>(a, c)))
    {
        if (!normal.IsZero())
            normal = normal / std::sqrt(normal.LengthSquared()); // normalize
        
        offset = -normal.DotProduct(a);
    }

    T DistanceToPoint(const Vector<T, 3>& point) const
    {
        return normal.DotProduct(point) + offset;
    }

    bool ContainsPoint(const Vector<T, 3>& point) const
    {
        return FloatingPointE(DistanceToPoint(point), T{0});
    }
};

// -------------------------------------------- Triangle ---------------------------------------------

enum class TriangleDegenerationType
{
    POINT,
    SEGMENT,
    TRIANGLE
};

enum class TrianglePlaneIntersection
{
    COPLANAR,
    NO_INTERSECTION,
    INTERSECTING
};

template<FloatingPoint T>
struct TrianglePlaneDistances
{
    T d_a;
    T d_b;
    T d_c;

    TrianglePlaneDistances() = default;

    TrianglePlaneDistances(const T& a, const T& b, const T& c) 
        : d_a(a), d_b(b), d_c(c) {}
};

template<FloatingPoint T, std::size_t Dim>
requires TriangleValidDimension<Dim>
struct Triangle
{
    Vector<T, Dim> a;
    Vector<T, Dim> b;
    Vector<T, Dim> c;
    TriangleDegenerationType degeneration_type;

    Triangle (const Vector<T, Dim>& first, const Vector<T, Dim>& second, const Vector<T, Dim>& third) 
        : a(first), b(second), c(third)
    {
        if (a == b && b == c)
            degeneration_type = TriangleDegenerationType::POINT;

        else if (Vector<T, Dim>{a, b}.IsParallel(Vector<T, Dim>{a, c}))
            degeneration_type = TriangleDegenerationType::SEGMENT;
        
        else
            degeneration_type = TriangleDegenerationType::TRIANGLE;
    }

    bool ContainsPoint(const Vector<T, Dim>& point) const
    {
        if constexpr (Dim == 3)
        {
            Plane plane(a, b, c);

            // Point doesn't lie in triangle plane
            if (!plane.ContainsPoint(point))
            {
                return false;
            }
        }

        Vector<T, Dim> ab(a, b);
        Vector<T, Dim> bc(b, c);
        Vector<T, Dim> ca(c, a);

        Vector<T, Dim> ap(a, point);
        Vector<T, Dim> bp(b, point);
        Vector<T, Dim> cp(c, point);

        // Check from which side of the edge is a Vector 
        T ab_side, bc_side, ca_side;

        if constexpr (Dim == 2)
        {
            ab_side = ab.CrossProduct(ap);
            bc_side = bc.CrossProduct(bp);
            ca_side = ca.CrossProduct(cp);
        }
        else if constexpr (Dim == 3)
        {
            Plane<T> plane(a, b, c);
            ab_side = (ab.CrossProduct(ap)).DotProduct(plane.normal);
            bc_side = (bc.CrossProduct(bp)).DotProduct(plane.normal);
            ca_side = (ca.CrossProduct(cp)).DotProduct(plane.normal);
        }

        if (FloatingPointLE(T{0}, ab_side) && FloatingPointLE(T{0}, bc_side) && FloatingPointLE(T{0}, ca_side))
        {
            return true;
        }

        if (FloatingPointLE(ab_side, T{0}) && FloatingPointLE(bc_side, T{0}) && FloatingPointLE(ca_side, T{0}))
        {
            return true;
        }

        return false;
    }

    // Triangle with segment intersection (3D)
    bool CheckIntersection(const LineSegment<T, 3>& segment) const
    {
        LineSegment ab{a, b};
        LineSegment bc{b, c};
        LineSegment ca{c, a};

        Plane plane(a, b, c);

        T dot = plane.normal.DotProduct(segment.direction);

        // Is segment parallel to plane
        if (FloatingPointE(dot, T{0}))
        {
            return CheckParallelTriangleSegmentIntersection(segment);
        }

        // not parallel
        // T t = -(plane.normal.DotProduct(segment.begin) + plane.offset) / dot;
        T t = -(plane.DistanceToPoint(segment.begin)) / dot;
        

        if (FloatingPointLE(T{0}, t) && FloatingPointLE(t, T{1.0}))
        {
            Vector intersection_point = segment.begin + segment.direction * t;

            return ContainsPoint(intersection_point);
        }

        return false;
    }

    // Triangle with segment intersection (2D)
    bool CheckIntersection(const LineSegment<T, 2>& segment) const
    {
        return CheckParallelTriangleSegmentIntersection(segment);
    }

    // Triangle with triangle intersection (3D)
    bool CheckIntersection(const Triangle<T, 3>& other) const
    {
        if (degeneration_type       != TriangleDegenerationType::TRIANGLE || 
            other.degeneration_type != TriangleDegenerationType::TRIANGLE)
        {
            return CheckDegenerateTrianglesIntersection(other);
        }

        const Plane first_plane  (this->a, this->b, this->c);
        const Plane second_plane (other.a, other.b, other.c);

        const TrianglePlaneDistances first_triangle_distances = FindDistanceFromTriangleVerticesToPlane(second_plane);
        const TrianglePlaneIntersection first_relative_to_second = CheckPlaneTriangleIntersection(first_triangle_distances);

        const TrianglePlaneDistances second_triangle_distances = other.FindDistanceFromTriangleVerticesToPlane(first_plane);
        const TrianglePlaneIntersection second_relative_to_first = CheckPlaneTriangleIntersection(second_triangle_distances);
        
        // One of the triangles doesn't intersect another’s plane
        if (first_relative_to_second == TrianglePlaneIntersection::NO_INTERSECTION || 
            second_relative_to_first == TrianglePlaneIntersection::NO_INTERSECTION)
        {
            return false;
        }

        // Triangles are coplanar
        if (first_relative_to_second == TrianglePlaneIntersection::COPLANAR)
        {
            return CheckCoplanarTrianglesIntersection(other);
        }

        // Common case

        const Vector<T, Dim> planes_intersection_line (first_plane.normal.CrossProduct(second_plane.normal));

        LineSegment<T, 1> first_intersection  =       FindTriangleLineIntersectionSegment(planes_intersection_line, first_triangle_distances);
        LineSegment<T, 1> second_intersection = other.FindTriangleLineIntersectionSegment(planes_intersection_line, second_triangle_distances);

        // if segments intersect, return true, otherwise return false
        return first_intersection.CheckIntersection(second_intersection);
    }

    // Triangle with triangle intersection (2D)
    bool CheckIntersection(const Triangle<T, 2>& other) const
    {
        if (degeneration_type       != TriangleDegenerationType::TRIANGLE || 
            other.degeneration_type != TriangleDegenerationType::TRIANGLE)
        {
            return CheckDegenerateTrianglesIntersection(other);
        }
        
        return CheckCoplanarTrianglesIntersection(other);
    }

  private:
    bool CheckParallelTriangleSegmentIntersection(const LineSegment<T, Dim>& segment) const
    {
        LineSegment ab{a, b};
        LineSegment bc{b, c}; 
        LineSegment ca{c, a};

        // check if segment inside triangle
        if (ContainsPoint(segment.begin) || ContainsPoint(segment.end)) 
            return true;
        
        // check if segment intersect triangle edges
        if (ab.CheckIntersection(segment) || bc.CheckIntersection(segment) || ca.CheckIntersection(segment))
            return true;

        return false;
    }

    TrianglePlaneDistances<T> FindDistanceFromTriangleVerticesToPlane(const Plane<T>& plane) const
    {
        const T d_a = plane.DistanceToPoint(a);
        const T d_b = plane.DistanceToPoint(b);
        const T d_c = plane.DistanceToPoint(c);

        return {d_a, d_b, d_c};
    }

    static TrianglePlaneIntersection CheckPlaneTriangleIntersection(const TrianglePlaneDistances<T>& distances)
    {
        bool coplanar = FloatingPointE(distances.d_a, T{0}) && FloatingPointE(distances.d_b, T{0}) && FloatingPointE(distances.d_c, T{0});
        if (coplanar) 
        {
            return TrianglePlaneIntersection::COPLANAR;
        }

        bool same_side = (FloatingPointG(distances.d_a, T{0}) && FloatingPointG(distances.d_b, T{0}) && FloatingPointG(distances.d_c, T{0})) ||
                         (FloatingPointL(distances.d_a, T{0}) && FloatingPointL(distances.d_b, T{0}) && FloatingPointL(distances.d_c, T{0}));
        if (same_side) 
        {
            return TrianglePlaneIntersection::NO_INTERSECTION; 
        }

        return TrianglePlaneIntersection::INTERSECTING;
    }

    bool CheckCoplanarTrianglesIntersection(const Triangle<T, Dim>& other) const
    {
        const LineSegment<T, Dim> first_edge_ab (a, b);
        const LineSegment<T, Dim> first_edge_bc (b, c);
        const LineSegment<T, Dim> first_edge_ca (c, a);

        const bool is_first_triangle_edges_intersect_second_triangle = 
            other.CheckParallelTriangleSegmentIntersection(first_edge_ab) ||
            other.CheckParallelTriangleSegmentIntersection(first_edge_bc) ||
            other.CheckParallelTriangleSegmentIntersection(first_edge_ca);

        if (is_first_triangle_edges_intersect_second_triangle)
        {
            return true;
        }

        const bool is_second_triangle_contains_in_first_triangle = 
            ContainsPoint(other.a) &&
            ContainsPoint(other.b) &&
            ContainsPoint(other.c);

        if (is_second_triangle_contains_in_first_triangle)
        {
            return true;
        }

        return false;
    }

    LineSegment<T, Dim> FindDegenerateTriangleSegmentWithMaxLength() const
    {
        assert (degeneration_type == TriangleDegenerationType::SEGMENT);

        LineSegment<T, Dim> ab{a, b};
        LineSegment<T, Dim> ac{a, c};
        LineSegment<T, Dim> bc{b, c};

        LineSegment<T, Dim> segment = ab;
        if (ac.LengthSquared() > segment.LengthSquared()) {
            segment = ac;
        }
        if (bc.LengthSquared() > segment.LengthSquared()) {
            segment = bc;
        }

        return segment;
    }

    bool CheckDegenerateTrianglesIntersection (const Triangle<T, Dim>& other) const
    {
        switch (degeneration_type)
        {
            case TriangleDegenerationType::POINT:
            {
                switch (other.degeneration_type)
                {
                    case TriangleDegenerationType::POINT:
                        return a == other.a;
                        
                    case TriangleDegenerationType::SEGMENT:
                    {
                        LineSegment<T, Dim> segment = other.FindDegenerateTriangleSegmentWithMaxLength();

                        return segment.ContainsPoint(a);
                    }
                        
                    case TriangleDegenerationType::TRIANGLE:
                        return other.ContainsPoint(a);
                }
                break;
            }
                
            case TriangleDegenerationType::SEGMENT:
            {
                LineSegment<T, Dim> first_segment = FindDegenerateTriangleSegmentWithMaxLength();
                
                switch (other.degeneration_type)
                {
                    case TriangleDegenerationType::POINT:
                        return first_segment.ContainsPoint(other.a);
                        
                    case TriangleDegenerationType::SEGMENT:
                    {     
                        LineSegment<T, Dim> second_segment = other.FindDegenerateTriangleSegmentWithMaxLength();

                        return first_segment.CheckIntersection(second_segment);
                    }
                        
                    case TriangleDegenerationType::TRIANGLE:
                    {
                        return other.CheckIntersection(first_segment);
                    }
                }
                break;
            }
                
            case TriangleDegenerationType::TRIANGLE:
            {
                switch (other.degeneration_type)
                {
                    case TriangleDegenerationType::POINT:
                        return ContainsPoint(other.a);
                        
                    case TriangleDegenerationType::SEGMENT:
                    {
                        LineSegment<T, Dim> segment = other.FindDegenerateTriangleSegmentWithMaxLength();

                        return CheckIntersection(segment);
                    }

                    case TriangleDegenerationType::TRIANGLE:
                        assert ("Unknown case of triangles degeneration"); 
                        return false;
                }
                break;
            }

            default:
                assert ("Unknown case of triangles degeneration"); 
                return false;
        }

        return false;
    }

    // it’s a total ****, but it works :)
    LineSegment<T, 1> FindTriangleLineIntersectionSegment(const Vector<T, 3>& line, const TrianglePlaneDistances<T>& distances) const 
    {
        // Project vertices to the line: projection = Dot (Direction, (Vertice - Line_offset))
        // optimization: don't use line offset point
        const T a_projection = line.DotProduct(a);
        const T b_projection = line.DotProduct(b);
        const T c_projection = line.DotProduct(c);

        T first_point_projection = 0;
        T second_point_projection = 0;
        T mid_point_projection = 0;

        TrianglePlaneDistances<T> new_distances {};

        // Are two points lies on the triangle plane?
        if (FloatingPointE(distances.d_a, T{0}) && FloatingPointE(distances.d_b, T{0}))
        {
            return {std::min(a_projection, b_projection), std::max(a_projection, b_projection)};
        }
        if (FloatingPointE(distances.d_b, T{0}) && FloatingPointE(distances.d_c, T{0}))
        {
            return {std::min(b_projection, c_projection), std::max(b_projection, c_projection)};
        }
        if (FloatingPointE(distances.d_a, T{0}) && FloatingPointE(distances.d_c, T{0}))
        {
            return {std::min(a_projection, c_projection), std::max(a_projection, c_projection)};
        }

        // Is one point lies on the triangle plane and other points doesn't lies on the same side?
        if (FloatingPointE(distances.d_a, T{0}) && (FloatingPointG(distances.d_b, T{0}) != FloatingPointG(distances.d_c, T{0})) ||
            FloatingPointE(distances.d_b, T{0}) && (FloatingPointG(distances.d_a, T{0}) != FloatingPointG(distances.d_c, T{0})))
        {
            first_point_projection = a_projection;
            second_point_projection = b_projection;
            mid_point_projection = c_projection;

            new_distances = {distances.d_a, distances.d_b, distances.d_c};
        }
        else if (FloatingPointE(distances.d_c, T{0}) && (FloatingPointG(distances.d_a, T{0}) != FloatingPointG(distances.d_b, T{0})))
        {
            first_point_projection = a_projection;
            second_point_projection = c_projection;
            mid_point_projection = b_projection;

            new_distances = {distances.d_a, distances.d_c, distances.d_b};
        }
        
        // Two points are on one side of the line, the remaining point is on the other side.
        // To determine this, we use the distances from the vertices of the triangle to the plane of the second triangle
        else if ((FloatingPointG(distances.d_a, T{0}) && FloatingPointG(distances.d_b, T{0})) || 
                 (FloatingPointL(distances.d_a, T{0}) && FloatingPointL(distances.d_b, T{0})))
        {
            first_point_projection = a_projection;
            second_point_projection = b_projection;
            mid_point_projection = c_projection;

            new_distances = {distances.d_a, distances.d_b, distances.d_c};
        }

        else if ((FloatingPointG(distances.d_a, T{0}) && FloatingPointG(distances.d_c, T{0})) ||
                 (FloatingPointL(distances.d_a, T{0}) && FloatingPointL(distances.d_c, T{0})))
        {
            first_point_projection = a_projection;
            second_point_projection = c_projection;
            mid_point_projection = b_projection;

            new_distances = {distances.d_a, distances.d_c, distances.d_b};
        }

        else if ((FloatingPointG(distances.d_b, T{0}) && FloatingPointG(distances.d_c, T{0})) ||
                 (FloatingPointL(distances.d_b, T{0}) && FloatingPointL(distances.d_c, T{0})))
        {
            first_point_projection = b_projection;
            second_point_projection = c_projection;
            mid_point_projection = a_projection;

            new_distances = {distances.d_b, distances.d_c, distances.d_a};
        }

        else
        {
            assert ("Unknown case of triangles intersection");
        }

        // division by zero
        assert(!FloatingPointE(new_distances.d_a, new_distances.d_c));
        assert(!FloatingPointE(new_distances.d_b, new_distances.d_c));

        T intersection_begin = first_point_projection  + ((mid_point_projection - first_point_projection)  * new_distances.d_a) / (new_distances.d_a - new_distances.d_c);
        T intersection_end   = second_point_projection + ((mid_point_projection - second_point_projection) * new_distances.d_b) / (new_distances.d_b - new_distances.d_c);

        return {std::min(intersection_begin, intersection_end), std::max(intersection_begin, intersection_end)};
    }
};

} // namespace Triangles
