#pragma once

namespace Triangles
{

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

  private:
    const double x_;
    const double y_;
    const double z_;
};

struct Vector
{
  public:
    Vector(double x, double y, double z) : vector_end_point_ (Point (x, y, z)) {}
    Vector(const Point& vector_end_point) : vector_end_point_ (vector_end_point) {}
    Vector(const Point& vector_start_point, const Point& vector_end_point) : vector_end_point_ (vector_end_point - vector_start_point) {}

    double GetX() const { return vector_end_point_.GetX (); }
    double GetY() const { return vector_end_point_.GetY (); }
    double GetZ() const { return vector_end_point_.GetZ (); }

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

  private:
    const Point a_;
    const Point b_;
    const Point c_;
};

struct Plane
{
  public:
    Plane (const Point& a, const Point& b, const Point& c) : 
        normal_ (Vector::CrossProduct (Vector (a, b), Vector (a, c))),
        offset_ (-Vector::DotProduct (normal_, a)) {}

    Plane (const Triangle& triangle) : 
        normal_ (Vector::CrossProduct (Vector (triangle.GetA(), triangle.GetB()), Vector (triangle.GetA(), triangle.GetC()))),
        offset_ (-Vector::DotProduct (normal_, triangle.GetA())) {}

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

struct LineSegment
{
  public:
    LineSegment (double begin, double end) :
        begin_ (begin), end_ (end) {} 
    
    double GetBegin() const { return begin_; }
    double GetEnd()   const { return end_; }

  private:
    const double begin_;
    const double end_;
};

enum class TrianglePlaneIntersection
{
    COPLANAR,
    NO_INTERSECTION,
    INTERSECTING
};

bool CheckTrianglesIntersection(const Triangle& first_triangle, const Triangle& second_triangle);

} // namespace Triangles
