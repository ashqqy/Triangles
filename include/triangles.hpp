#pragma once

#include <cmath>
#include <type_traits> // for std::is_same

namespace Triangles
{

// ---------------------------------------- Double comparsion ----------------------------------------

inline bool DoubleLessOrEqual(double left, double right, double epsilon = 1e-7)
{
    return left - right < epsilon;
}

inline bool DoubleLess(double left, double right, double epsilon = 1e-7)
{
    return left - right < -epsilon;
}

inline bool DoubleGreaterOrEqual(double left, double right, double epsilon = 1e-7)
{
    return left - right > -epsilon;
}

inline bool DoubleGreater(double left, double right, double epsilon = 1e-7)
{
    return left - right > epsilon;
}

inline bool DoubleEqual(double a, double b, double epsilon = 1e-7)
{
    return std::abs(a - b) < epsilon;
}

// ---------------------------------------------- Point ----------------------------------------------

struct Point
{
  public:
    Point(double x, double y, double z) : x_(x), y_(y), z_(z) {}

    Point operator+(const Point& point) const
    {
        return Point (x_ + point.x_, y_ + point.y_, z_ + point.z_);
    }

    Point operator-(const Point& point) const
    {
        return Point (x_ - point.x_, y_ - point.y_, z_ - point.z_);
    }

    bool operator==(const Point& point) const
    {
        return DoubleEqual(x_, point.GetX()) && DoubleEqual(y_, point.GetY()) && DoubleEqual(z_, point.GetZ());
    }

    double GetX() const { return x_; }
    double GetY() const { return y_; }
    double GetZ() const { return z_; }

    bool IsZero() const { return DoubleEqual(x_, 0) && DoubleEqual(y_, 0) && DoubleEqual(z_, 0); }

  private:
    const double x_;
    const double y_;
    const double z_;
};

// ------------------------------------------ Line segment -------------------------------------------

template<typename T>
struct LineSegment
{
    static_assert(std::is_same<T, double>::value || std::is_same<T, Point>::value,
                  "T must be double or Point");

  public:
    LineSegment(T begin, T end) :
        begin_(begin), end_(end) {} 
    
    T GetBegin() const { return begin_; }
    T GetEnd()   const { return end_; }

  private:
    const T begin_;
    const T end_;
};

// --------------------------------------------- Vector ----------------------------------------------

struct Vector
{
  public:
    Vector(double x, double y, double z) : vector_end_point_ (Point (x, y, z)) {}
    Vector(const Point& vector_end_point) : vector_end_point_ (vector_end_point) {}
    Vector(const Point& vector_start_point, const Point& vector_end_point) : vector_end_point_ (vector_end_point - vector_start_point) {}
    Vector(const LineSegment<Point>& segment) : vector_end_point_ (segment.GetEnd() - segment.GetBegin()) {}

    double GetX() const { return vector_end_point_.GetX(); }
    double GetY() const { return vector_end_point_.GetY(); }
    double GetZ() const { return vector_end_point_.GetZ(); }

    bool IsZero() const { return vector_end_point_.IsZero(); }

    static double DotProduct   (const Vector& first, const Vector& second)
    {
        return first.GetX() * second.GetX() + first.GetY() * second.GetY() + first.GetZ() * second.GetZ();
    }

    static Vector CrossProduct (const Vector& first, const Vector& second)
    {
        double c_x = first.GetY() * second.GetZ() - first.GetZ() * second.GetY();
        double c_y = first.GetZ() * second.GetX() - first.GetX() * second.GetZ();
        double c_z = first.GetX() * second.GetY() - first.GetY() * second.GetX();

        return Vector(c_x, c_y, c_z);
    }

  private:
    const Point vector_end_point_;
};

// -------------------------------------------- Triangle ---------------------------------------------

enum class TriangleDegenerationType
{
    POINT,
    SEGMENT,
    TRIANGLE
};

struct Triangle
{
  public:
    Triangle (const Point& a, const Point& b, const Point& c) : a_(a), b_(b), c_(c)
    {
        if (a_ == b_ && b_ == c_)
        {
            degeneration_type_ = TriangleDegenerationType::POINT;
        }

        if (a_ == b_ || b_ == c_ || a_ == c_)
        {
            degeneration_type_ = TriangleDegenerationType::SEGMENT;
        }

        degeneration_type_ = TriangleDegenerationType::TRIANGLE;
    }

    Point GetA() const { return a_; }
    Point GetB() const { return b_; }
    Point GetC() const { return c_; }

    bool ContainsPoint(const Point& point) const;

    TriangleDegenerationType DegenerationType() const { return degeneration_type_; }

  private:
    Point a_;
    Point b_;
    Point c_;

    TriangleDegenerationType degeneration_type_;
};

// ---------------------------------------------- Plane ----------------------------------------------

struct Plane
{
  public:
    Plane (const Point& a, const Point& b, const Point& c) : 
        normal_ (Vector::CrossProduct(Vector(a, b), Vector(a, c))),
        offset_ (-Vector::DotProduct(normal_, a)) {}

    Plane (const Triangle& triangle) : 
        normal_ (Vector::CrossProduct(Vector(triangle.GetA(), triangle.GetB()), Vector(triangle.GetA(), triangle.GetC()))),
        offset_ (-Vector::DotProduct(normal_, triangle.GetA())) {}

    bool ContainsPoint(const Point& point) const
    {
        return DoubleEqual(Vector::DotProduct (normal_, point) + offset_, 0);
    }

    Vector GetNormal() const { return normal_; }
    double GetOffset() const { return offset_; }

  private:
    // pi: (normal_, X) + offset_ = 0, where X belong pi
    Vector normal_;
    double offset_;
};

inline bool Triangle::ContainsPoint (const Point& point) const
{
    Vector ab = Vector(a_, b_);
    Vector bc = Vector(b_, c_);
    Vector ca = Vector(c_, a_);

    Vector ap = Vector(a_, point);
    Vector bp = Vector(b_, point);
    Vector cp = Vector(c_, point);

    Plane triangle_plane(*this);

    // Point doesn't lie in triangle plane
    if (!triangle_plane.ContainsPoint(point))
    {
        return false;
    }

    Vector triangle_normal = triangle_plane.GetNormal();

    // Check from which side of the edge is a point 
    double ab_side = Vector::DotProduct(Vector::CrossProduct(ab, ap), triangle_normal); 
    double bc_side = Vector::DotProduct(Vector::CrossProduct(bc, bp), triangle_normal); 
    double ca_side = Vector::DotProduct(Vector::CrossProduct(ca, cp), triangle_normal); 

    if (DoubleLessOrEqual(0, ab_side) && DoubleLessOrEqual(0, bc_side) && DoubleLessOrEqual(0, ca_side))
    {
        return true;
    }

    if (DoubleLessOrEqual(ab_side, 0) && DoubleLessOrEqual(bc_side, 0) && DoubleLessOrEqual(ca_side, 0))
    {
        return true;
    }

    return false;
}

// ----------------------------------------------- Line ----------------------------------------------

struct Line
{
  public:
    Line (const Vector& direction, const Vector& offset) :
        direction_ (direction), offset_ (offset) {} 
    
    Line (const Vector& direction) :
        direction_ (direction), offset_ (Vector (0, 0, 0)) {} 

    Vector GetDirection() const { return direction_; }
    Vector GetOffset()    const { return offset_; }

  private:
    // L = offset_ + t * direction_;
    const Vector direction_;
    const Vector offset_;
};

// ---------------------------------------------------------------------------------------------------
// -------------------------------------- Function declarations --------------------------------------
// ---------------------------------------------------------------------------------------------------

// ------------------------------------- Main algorithm function -------------------------------------

bool CheckTrianglesIntersection(const Triangle& first_triangle, const Triangle& second_triangle);

// ------------------------------------- Main support functions --------------------------------------

enum class TrianglePlaneIntersection
{
    COPLANAR,
    NO_INTERSECTION,
    INTERSECTING
};

struct TrianglePlaneDistances
{
    TrianglePlaneDistances() = default;

    TrianglePlaneDistances(const double& a, const double& b, const double& c) 
        : d_a(a), d_b(b), d_c(c) {}

    double d_a;
    double d_b;
    double d_c;
};

bool CheckDegenerateTrianglesIntersection (const Triangle& first_triangle, const Triangle& second_triangle);
TrianglePlaneIntersection CheckPlaneTriangleIntersection(const TrianglePlaneDistances& distances);
bool CheckCoplanarTrianglesIntersection(const Triangle& first_triangle, const Triangle& second_triangle);
LineSegment<double> FindTriangleLineIntersectionSegment(const Triangle& triangle, const Line& line, const TrianglePlaneDistances& distances);

// ------------------------------------- Line segment functions --------------------------------------

bool CheckSegmentsIntersection        (const LineSegment<double>& segment1, const LineSegment<double>& segment2);
bool CheckSegmentsIntersection        (const LineSegment<Point>& segment1,  const LineSegment<Point>& segment2);
bool CheckParallelSegmentsIntersection(const LineSegment<Point>& segment1,  const LineSegment<Point>& segment2);

bool CheckTriangleSegmentIntersection (const Triangle& triangle, const LineSegment<Point>& segment);
bool CheckSegmentPointIntersection    (const LineSegment<Point>& segment, const Point& point);

// ----------------------------------- Distance to plane functions -----------------------------------

double FindDistanceFromPointToPlane(const Point& point, const Plane& plane);
TrianglePlaneDistances FindDistanceFromTriangleVerticesToPlane(const Triangle& triangle, const Plane& plane);

} // namespace Triangles
