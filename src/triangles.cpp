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
    if (first_triangle.DegenerationType()  != TriangleDegenerationType::TRIANGLE || 
        second_triangle.DegenerationType() != TriangleDegenerationType::TRIANGLE)
    {
        return CheckDegenerateTrianglesIntersection(first_triangle, second_triangle);
    }

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

    // Common case

    // optimization: don't compute offset point
    const Line planes_intersection_line (Vector::CrossProduct(first_plane.GetNormal(), second_plane.GetNormal()));

    LineSegment<double> first_intersection  = FindTriangleLineIntersectionSegment(first_triangle, planes_intersection_line, first_triangle_distances);
    LineSegment<double> second_intersection = FindTriangleLineIntersectionSegment(second_triangle, planes_intersection_line, second_triangle_distances);

    // if segments intersect, return true, otherwise return false
    return CheckSegmentsIntersection (first_intersection, second_intersection);
}

// ------------------------------------- Main support functions --------------------------------------

bool CheckDegenerateTrianglesIntersection (const Triangle& first_triangle, const Triangle& second_triangle)
{
    switch (first_triangle.DegenerationType())
    {
        case TriangleDegenerationType::POINT:
        {
            Point first_point = first_triangle.GetA();
            
            switch (second_triangle.DegenerationType())
            {
                case TriangleDegenerationType::POINT:
                    return first_point == second_triangle.GetA();
                    
                case TriangleDegenerationType::SEGMENT:
                {
                    Point segment_start = second_triangle.GetA();
                    Point segment_end   = (segment_start != second_triangle.GetB()) ? second_triangle.GetB() : second_triangle.GetC();
                    LineSegment second_segment(segment_start, segment_end);

                    return CheckSegmentPointIntersection(second_segment, first_point);
                }
                    
                case TriangleDegenerationType::TRIANGLE:
                    return second_triangle.ContainsPoint(first_point);
            }
            break;
        }
            
        case TriangleDegenerationType::SEGMENT:
        {
            Point first_segment_start = first_triangle.GetA();
            Point first_segment_end   = (first_segment_start != first_triangle.GetB()) ? first_triangle.GetB() : first_triangle.GetC();
            LineSegment first_segment(first_segment_start, first_segment_end);
            
            switch (second_triangle.DegenerationType())
            {
                case TriangleDegenerationType::POINT:
                    return CheckSegmentPointIntersection(first_segment, second_triangle.GetA());
                    
                case TriangleDegenerationType::SEGMENT:
                {                    
                    Point segment_start = second_triangle.GetA();
                    Point segment_end   = (segment_start != second_triangle.GetB()) ? second_triangle.GetB() : second_triangle.GetC();
                    LineSegment second_segment(segment_start, segment_end);

                    return CheckSegmentsIntersection(first_segment, second_segment);
                }
                    
                case TriangleDegenerationType::TRIANGLE:
                    return CheckTriangleSegmentIntersection(second_triangle, first_segment);
            }
            break;
        }
            
        case TriangleDegenerationType::TRIANGLE:
        {
            switch (second_triangle.DegenerationType())
            {
                case TriangleDegenerationType::POINT:
                    return first_triangle.ContainsPoint(second_triangle.GetA());
                    
                case TriangleDegenerationType::SEGMENT:
                {
                    Point segment_start = second_triangle.GetA();
                    Point segment_end   = (segment_start != second_triangle.GetB()) ? second_triangle.GetB() : second_triangle.GetC();
                    LineSegment second_segment(segment_start, segment_end);

                    return CheckTriangleSegmentIntersection(first_triangle, second_segment);
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

TrianglePlaneIntersection CheckPlaneTriangleIntersection(const TrianglePlaneDistances& distances)
{
    const bool coplanar = DoubleEqual(distances.d_a, 0) && DoubleEqual(distances.d_b, 0) && DoubleEqual(distances.d_c, 0);
    if (coplanar) { return TrianglePlaneIntersection::COPLANAR; }

    bool same_side = (DoubleGreater(distances.d_a, 0) && DoubleGreater(distances.d_b, 0) && DoubleGreater(distances.d_c, 0)) ||
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

// it’s a total ****, but it works :)
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

    // Are two points lies on the triangle plane?
    if (DoubleEqual(distances.d_a, 0) && DoubleEqual(distances.d_b, 0))
    {
        return LineSegment(std::min(a_projection, b_projection), std::max(a_projection, b_projection));
    }
    if (DoubleEqual(distances.d_b, 0) && DoubleEqual(distances.d_c, 0))
    {
        return LineSegment(std::min(b_projection, c_projection), std::max(b_projection, c_projection));
    }
    if (DoubleEqual(distances.d_a, 0) && DoubleEqual(distances.d_c, 0))
    {
        return LineSegment(std::min(a_projection, c_projection), std::max(a_projection, c_projection));
    }

    // Is one point lies on the triangle plane and other points doesn't lies on the same side?
    if (DoubleEqual(distances.d_a, 0) && (DoubleGreater(distances.d_b, 0) != DoubleGreater(distances.d_c, 0)) ||
        DoubleEqual(distances.d_b, 0) && (DoubleGreater(distances.d_a, 0) != DoubleGreater(distances.d_c, 0)))
    {
        first_point_projection = a_projection;
        second_point_projection = b_projection;
        mid_point_projection = c_projection;

        new_distances = {distances.d_a, distances.d_b, distances.d_c};
    }
    else if (DoubleEqual(distances.d_c, 0) && (DoubleGreater(distances.d_a, 0) != DoubleGreater(distances.d_b, 0)))
    {
        first_point_projection = a_projection;
        second_point_projection = c_projection;
        mid_point_projection = b_projection;

        new_distances = {distances.d_a, distances.d_c, distances.d_b};
    }
    
    // Two points are on one side of the line, the remaining point is on the other side.
    // To determine this, we use the distances from the vertices of the triangle to the plane of the second triangle
    else if ((DoubleGreater(distances.d_a, 0) && DoubleGreater(distances.d_b, 0)) || 
             (DoubleLess   (distances.d_a, 0) && DoubleLess   (distances.d_b, 0)))
    {
        first_point_projection = a_projection;
        second_point_projection = b_projection;
        mid_point_projection = c_projection;

        new_distances = {distances.d_a, distances.d_b, distances.d_c};
    }

    else if ((DoubleGreater(distances.d_a, 0) && DoubleGreater(distances.d_c, 0)) ||
             (DoubleLess   (distances.d_a, 0) && DoubleLess   (distances.d_c, 0)))
    {
        first_point_projection = a_projection;
        second_point_projection = c_projection;
        mid_point_projection = b_projection;

        new_distances = {distances.d_a, distances.d_c, distances.d_b};
    }

    else if ((DoubleGreater(distances.d_b, 0) && DoubleGreater(distances.d_c, 0)) ||
             (DoubleLess   (distances.d_b, 0) && DoubleLess   (distances.d_c, 0)))
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

    Plane plane(triangle);
    Vector normal = plane.GetNormal();
    Vector direction(segment);

    double dot = Vector::DotProduct(normal, direction);

    // Is segment parallel to plane
    if (DoubleEqual(dot, 0))
    {
        if (CheckSegmentsIntersection(ab, segment)) return true;
        if (CheckSegmentsIntersection(bc, segment)) return true;
        if (CheckSegmentsIntersection(ca, segment)) return true;

        return false;
    }

    // not parallel
    double t = -(Vector::DotProduct(normal, Vector(segment.GetBegin())) + plane.GetOffset()) / dot;
    
    if (DoubleLessOrEqual(0, t) && DoubleLessOrEqual(t, 1))
    {
        Point intersection_point = segment.GetBegin() + Point(direction.GetX() * t, direction.GetY() * t, direction.GetZ() * t);
        return triangle.ContainsPoint(intersection_point);
    }

    return false;
}

bool CheckSegmentPointIntersection (const LineSegment<Point>& segment, const Point& point)
{
    Vector direction(segment);
    Vector connection(point, segment.GetBegin());

    if (!Vector::CrossProduct(direction, connection).IsZero())
    {
        return false;
    }

    return DoubleGreaterOrEqual(Vector::DotProduct(connection, direction), 0) && 
           DoubleLessOrEqual   (Vector::DotProduct(connection, direction), Vector::DotProduct(direction, direction));
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
