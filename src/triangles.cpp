#include <algorithm>
#include <cassert>
#include <utility>
#include <iostream>

#include "triangles.hpp"

namespace Triangles
{

// ------------------------------------- Main algorithm function -------------------------------------

// returns true if triangles intersect, false otherwise
bool CheckTrianglesIntersection(const Triangle& first_triangle, const Triangle& second_triangle)
{
    const Plane first_plane  (first_triangle);
    const Plane second_plane (second_triangle);

    const TrianglePlaneDistances first_triangle_distances = FindDistanceFromTriangleVerticesToPlane(first_triangle, second_plane);
    const TrianglePlaneIntersection first_relative_to_second = CheckPlaneTriangleIntersection(first_triangle_distances);

    const TrianglePlaneDistances second_triangle_distances = FindDistanceFromTriangleVerticesToPlane(second_triangle, first_plane);
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
        return CheckCoplanarTrianglesIntersection(first_triangle, second_triangle);
    }

    // Vertice of first triangle lies inside second
    if (first_triangle.ContainsPoint(second_triangle.GetA()) || 
        first_triangle.ContainsPoint(second_triangle.GetB()) ||
        first_triangle.ContainsPoint(second_triangle.GetC()))
    {
        return true;
    }

    // Vertice of second triangle lies inside first
    if (second_triangle.ContainsPoint(first_triangle.GetA()) || 
        second_triangle.ContainsPoint(first_triangle.GetB()) ||
        second_triangle.ContainsPoint(first_triangle.GetC()))
    {
        return true;
    }

    // Common case

    // optimization: don't compute offset point
    const Line planes_intersection_line (Vector::CrossProduct(first_plane.GetNormal(), second_plane.GetNormal()));

    LineSegment<double> first_intersection  = FindTriangleLineIntersectionSegment(first_triangle, planes_intersection_line, first_triangle_distances);
    LineSegment<double> second_intersection = FindTriangleLineIntersectionSegment(second_triangle, planes_intersection_line, second_triangle_distances);

    // if segments intersect, return true, otherwise return false
    return CheckSegmentsIntersection (first_intersection, second_intersection);
}

// ------------------------------------- Main support functions --------------------------------------

TrianglePlaneIntersection CheckPlaneTriangleIntersection(const TrianglePlaneDistances& distances)
{
    const bool coplanar = DoubleEqual(distances.d_a, 0) && DoubleEqual(distances.d_b, 0) && DoubleEqual(distances.d_c, 0);
    if (coplanar) { return TrianglePlaneIntersection::COPLANAR; }

    bool same_side = (DoubleGreater(distances.d_a, 0) && DoubleGreater(distances.d_b, 0) && DoubleGreater(distances.d_b, 0)) ||
                     (DoubleLess(distances.d_a, 0)    && DoubleLess(distances.d_b, 0)    && DoubleLess(distances.d_c, 0));
    if (same_side) 
    {
        return TrianglePlaneIntersection::NO_INTERSECTION; 
    }

    return TrianglePlaneIntersection::INTERSECTING;
}

bool CheckCoplanarTrianglesIntersection(const Triangle& first_triangle, const Triangle& second_triangle)
{
    const LineSegment first_edge_ab (first_triangle.GetA(), first_triangle.GetB());
    const LineSegment first_edge_bc (first_triangle.GetB(), first_triangle.GetC());
    const LineSegment first_edge_ca (first_triangle.GetC(), first_triangle.GetA());

    const bool is_first_triangle_edges_intersect_second_triangle = 
        CheckTriangleSegmentIntersection (second_triangle, first_edge_ab) &&
        CheckTriangleSegmentIntersection (second_triangle, first_edge_bc) &&
        CheckTriangleSegmentIntersection (second_triangle, first_edge_ca);

    if (is_first_triangle_edges_intersect_second_triangle)
    {
        return true;
    }

    const bool is_first_triangle_contains_in_second_triangle = 
        second_triangle.ContainsPoint(first_triangle.GetA());
        second_triangle.ContainsPoint(first_triangle.GetB());
        second_triangle.ContainsPoint(first_triangle.GetC());

    if (is_first_triangle_contains_in_second_triangle)
    {
        return true;
    }

    const bool is_second_triangle_contains_in_first_triangle = 
        first_triangle.ContainsPoint(second_triangle.GetA());
        first_triangle.ContainsPoint(second_triangle.GetB());
        first_triangle.ContainsPoint(second_triangle.GetC());

    if (is_second_triangle_contains_in_first_triangle)
    {
        return true;
    }

    return false;
}

LineSegment<double> FindTriangleLineIntersectionSegment(const Triangle& triangle, const Line& line, const TrianglePlaneDistances& distances)
{
    // Project vertices to the line: projection = Dot (Direction, (Vertice - Line_offset))
    // optimization: don't use line offset point
    const double a_projection = Vector::DotProduct (line.GetDirection(), Vector(triangle.GetA()));
    const double b_projection = Vector::DotProduct (line.GetDirection(), Vector(triangle.GetB()));
    const double c_projection = Vector::DotProduct (line.GetDirection(), Vector(triangle.GetC()));

    double first_point_projection = 0;
    double second_point_projection = 0;
    double mid_point_projection = 0;

    TrianglePlaneDistances new_distances;

    // TODO: points lies on the edge 
    
    // Two points are on one side of the line, the remaining point is on the other side.
    // To determine this, we use the distances from the vertices of the triangle to the plane of the second triangle
    if ((distances.d_a > 0) == (distances.d_b > 0))
    {
        first_point_projection = a_projection;
        second_point_projection = b_projection;
        mid_point_projection = c_projection;

        new_distances = {distances.d_a, distances.d_b, distances.d_c};
    }

    else if ((distances.d_a > 0) == (distances.d_c > 0))
    {
        first_point_projection = a_projection;
        second_point_projection = c_projection;
        mid_point_projection = b_projection;

        new_distances = {distances.d_a, distances.d_c, distances.d_b};
    }

    else 
    {
        first_point_projection = b_projection;
        second_point_projection = c_projection;
        mid_point_projection = a_projection;

        new_distances = {distances.d_b, distances.d_c, distances.d_a};
    }

    // division by zero
    assert(!DoubleEqual(new_distances.d_a, new_distances.d_c));
    assert(!DoubleEqual(new_distances.d_b, new_distances.d_c));

    double intersection_begin = first_point_projection  + ((mid_point_projection - first_point_projection)  * new_distances.d_a) / (new_distances.d_a - new_distances.d_c);
    double intersection_end   = second_point_projection + ((mid_point_projection - second_point_projection) * new_distances.d_b) / (new_distances.d_b - new_distances.d_c);

    return LineSegment (std::min(intersection_begin, intersection_end), std::max(intersection_begin, intersection_end));
}

// ------------------------------------- Line segment functions --------------------------------------

// 1D segments
bool CheckSegmentsIntersection(const LineSegment<double>& segment1, const LineSegment<double>& segment2)
{
    return DoubleLessOrEqual(segment1.GetBegin(), segment2.GetEnd()) &&
           DoubleLessOrEqual(segment2.GetBegin(), segment1.GetEnd());
}

// 3D segments
bool CheckSegmentsIntersection(const LineSegment<Point>& segment1, const LineSegment<Point>& segment2)
{
    Vector vec1(segment1);
    Vector vec2(segment2);

    Vector segments_cross = Vector::CrossProduct(vec1, vec2);

    // Check if segments are parallel
    if (segments_cross.IsZero())
    {
        return CheckParallelSegmentsIntersection(segment1, segment2);
    }

    Vector vec_between_starts(segment1.GetBegin(), segment2.GetBegin());
    
    // Find the volume of the parallelepiped formed by these segments
    // volume = scalar triple product = dot (vec_between_starts, cross (vec1, vec2)) = dot (vec_between_starts, segments_cross)
    double volume = Vector::DotProduct (vec_between_starts, segments_cross);

    // if (volume != 0) => segments are skew => segments don't intersect
    if (!DoubleEqual(volume, 0))
    {
        return false;
    }

    // Segments are co-planar and not parallel

    // Find t1 and t2: 
    // begin1 + t1 * vec1 = begin2 + t2 * vec2

    Vector cross1 = Vector::CrossProduct(vec_between_starts, vec1);
    Vector cross2 = Vector::CrossProduct(vec_between_starts, vec2);

    double t1 = Vector::DotProduct(cross2, segments_cross) / 
                Vector::DotProduct(segments_cross, segments_cross);

    double t2 = Vector::DotProduct(cross1, segments_cross) / 
                Vector::DotProduct(segments_cross, segments_cross);

    return (DoubleLessOrEqual(0, t1) && DoubleLessOrEqual(t1, 1) && 
            DoubleLessOrEqual(0, t2) && DoubleLessOrEqual(t2, 1));
}

bool CheckParallelSegmentsIntersection(const LineSegment<Point>& segment1, const LineSegment<Point>& segment2)
{
    Vector vec1(segment1);
    Vector vec_between_starts(segment1.GetBegin(), segment2.GetBegin());

    // Check if segments aren't on the same line
    if (!Vector::CrossProduct(vec1, vec_between_starts).IsZero())
    {
        return false;
    }

    // Make a projection on each axis and check 1D case
    LineSegment proj_x1(segment1.GetBegin().GetX(), segment1.GetEnd().GetX());
    LineSegment proj_x2(segment2.GetBegin().GetX(), segment2.GetEnd().GetX());
    
    LineSegment proj_y1(segment1.GetBegin().GetY(), segment1.GetEnd().GetY());
    LineSegment proj_y2(segment2.GetBegin().GetY(), segment2.GetEnd().GetY());
    
    LineSegment proj_z1(segment1.GetBegin().GetZ(), segment1.GetEnd().GetZ());
    LineSegment proj_z2(segment2.GetBegin().GetZ(), segment2.GetEnd().GetZ());
    
    return CheckSegmentsIntersection(proj_x1, proj_x2) &&
           CheckSegmentsIntersection(proj_y1, proj_y2) &&
           CheckSegmentsIntersection(proj_z1, proj_z2);
}

bool CheckTriangleSegmentIntersection(const Triangle& triangle, const LineSegment<Point>& segment)
{
    LineSegment ab = LineSegment(triangle.GetA(), triangle.GetB());
    LineSegment bc = LineSegment(triangle.GetB(), triangle.GetC());
    LineSegment ca = LineSegment(triangle.GetC(), triangle.GetA());

    if (CheckSegmentsIntersection (ab, segment)) return true;
    if (CheckSegmentsIntersection (bc, segment)) return true;
    if (CheckSegmentsIntersection (ca, segment)) return true;

    return false;
}

// ----------------------------------- Distance to plane functions -----------------------------------

double FindDistanceFromPointToPlane(const Point& point, const Plane& plane)
{
    return Vector::DotProduct(plane.GetNormal(), Vector (point)) + plane.GetOffset();
}

TrianglePlaneDistances FindDistanceFromTriangleVerticesToPlane(const Triangle& triangle, const Plane& plane)
{
    const double d_a = FindDistanceFromPointToPlane(triangle.GetA(), plane);
    const double d_b = FindDistanceFromPointToPlane(triangle.GetB(), plane);
    const double d_c = FindDistanceFromPointToPlane(triangle.GetC(), plane);

    TrianglePlaneDistances distances (d_a, d_b, d_c);

    return distances;
}

} // namespace Triangles
