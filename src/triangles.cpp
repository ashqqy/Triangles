#include "triangles.hpp"

namespace Triangles
{

double Vector::DotProduct(const Vector& first, const Vector& second)
{
    return first.GetX() * second.GetX() + first.GetY() * second.GetY() + first.GetZ() * second.GetZ();
}

Vector Vector::CrossProduct(const Vector& first, const Vector& second)
{
    double c_x = first.GetY() * second.GetZ() - first.GetZ() * second.GetY();
    double c_y = first.GetZ() * second.GetX() - first.GetX() * second.GetZ();
    double c_z = first.GetX() * second.GetY() - first.GetY() * second.GetX();

    return Vector(c_x, c_y, c_z);
}

// returns true if triangles intersect, false otherwise
bool CheckTrianglesIntersection(const Triangle& first_triangle, const Triangle& second_triangle)
{
    Plane first_plane  (first_triangle);
    Plane second_plane (second_triangle);

    TrianglePlaneIntersection first_relative_to_second = CheckPlaneTriangleIntersection(second_plane, first_triangle);
    TrianglePlaneIntersection second_relative_to_first = CheckPlaneTriangleIntersection(first_plane, second_triangle);
    
    // One of the triangles doesn't intersect another’s plane
    if (first_relative_to_second == TrianglePlaneIntersection::NO_INTERSECTION || 
        second_relative_to_first == TrianglePlaneIntersection::NO_INTERSECTION)
    {
        return false;
    }

    // Triangles are coplanar
    if (first_relative_to_second == TrianglePlaneIntersection::COPLANAR)
    {
        return CheckCoplanarTrianglesIntersection(first_triangle, second_triangle);
    }

    // Common case
    // ...
}

TrianglePlaneIntersection CheckPlaneTriangleIntersection(const Plane& plane, const Triangle& triangle)
{
    const double d_a = FindDistanceFromPointToPlane(triangle.GetA(), plane);
    const double d_b = FindDistanceFromPointToPlane(triangle.GetB(), plane);
    const double d_c = FindDistanceFromPointToPlane(triangle.GetC(), plane);

    const bool coplanar = (d_a == 0) && (d_b == 0) && (d_c == 0);
    if (coplanar) { return TrianglePlaneIntersection::COPLANAR; }

    const bool same_side = ((d_a > 0) == (d_b > 0)) && ((d_b > 0) == (d_c > 0));
    if (same_side) { return TrianglePlaneIntersection::NO_INTERSECTION; }

    return TrianglePlaneIntersection::INTERSECTING;
}

bool CheckCoplanarTrianglesIntersection (const Triangle& first_triangle, const Triangle& second_triangle)
{
    // ...
}

double FindDistanceFromPointToPlane(const Point& point, const Plane& plane)
{
    Vector point_radius_vector (point);

    return Vector::DotProduct (plane.GetNormal(), point_radius_vector) + plane.GetOffset();
}

} // namespace Triangles
