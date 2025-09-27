#pragma once

#include <cmath>
#include <type_traits> // for std::is_same

namespace Triangles
{

inline bool DoubleLessOrEqual(double left, double right, double epsilon = 1e-10)
{
    return left < right || std::abs(left - right) < epsilon;
}

inline bool DoubleEqual(double a, double b, double epsilon = 1e-10)
{
    return std::abs(a - b) < epsilon;
}

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

    double GetX() const { return x_; }
    double GetY() const { return y_; }
    double GetZ() const { return z_; }

    bool IsZero() const { return DoubleEqual(x_, 0) && DoubleEqual(y_, 0) && DoubleEqual(z_, 0); }

  private:
    const double x_;
    const double y_;
    const double z_;
};

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

    static double DotProduct   (const Vector& first, const Vector& second);
    static Vector CrossProduct (const Vector& first, const Vector& second);

  private:
    const Point vector_end_point_;
};

struct Triangle
{
  public:
    Triangle (const Point& a, const Point& b, const Point& c) : a_(a), b_(b), c_(c) {}

    Point GetA() const { return a_; }
    Point GetB() const { return b_; }
    Point GetC() const { return c_; }

    bool ContainsPoint(const Point& point) const
    {
        Vector ab = Vector(a_, b_);
        Vector bc = Vector(b_, c_);
        Vector ca = Vector(c_, a_);

        Vector ap = Vector(a_, point);
        Vector bp = Vector(b_, point);
        Vector cp = Vector(c_, point);

        Vector triangle_normal = Vector::CrossProduct(ab, bc);
        
        // Point doesn't lie in triangle plane
        if (!DoubleEqual(Vector::DotProduct(triangle_normal, ap), 0))
        {
            return false;
        }

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

  private:
    const Point a_;
    const Point b_;
    const Point c_;
};

struct Plane
{
  public:
    Plane (const Point& a, const Point& b, const Point& c) : 
        normal_ (Vector::CrossProduct(Vector(a, b), Vector(a, c))),
        offset_ (-Vector::DotProduct(normal_, a)) {}

    Plane (const Triangle& triangle) : 
        normal_ (Vector::CrossProduct(Vector(triangle.GetA(), triangle.GetB()), Vector(triangle.GetA(), triangle.GetC()))),
        offset_ (-Vector::DotProduct(normal_, triangle.GetA())) {}

    Vector GetNormal() const { return normal_; }
    double GetOffset() const { return offset_; }

  private:
    // pi: (normal_, X) + offset_ = 0, where X belong pi
    Vector normal_;
    double offset_;
};

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

bool CheckTrianglesIntersection(const Triangle& first_triangle, const Triangle& second_triangle);

} // namespace Triangles
