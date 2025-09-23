#pragma once

namespace Triangles
{
struct Point
{
  public:
    Point (double x, double y, double z) : x_(x), y_(y), z_(z) {}

    Point operator+ (const Point& point) const
    {
        return Point (x_ + point.x_, y_ + point.y_, z_ + point.z_);
    }

    Point operator- (const Point& point) const
    {
        return Point (x_ - point.x_, y_ - point.y_, z_ - point.z_);
    }

    double GetX () const { return x_; }
    double GetY () const { return y_; }
    double GetZ () const { return z_; }

  private:
    const double x_;
    const double y_;
    const double z_;
};

struct Vector
{
  public:
    Vector (double x, double y, double z) : vector_end_point_ (Point (x, y, z)) {}
    Vector (const Point vector_end_point) : vector_end_point_ (vector_end_point) {}
    Vector (const Point& vector_start_point, const Point& vector_end_point) : vector_end_point_ (vector_end_point - vector_start_point) {}

    double GetX () const { return vector_end_point_.GetX (); }
    double GetY () const { return vector_end_point_.GetY (); }
    double GetZ () const { return vector_end_point_.GetZ (); }

    static double DotProduct    (const Vector& first, const Vector& second);
    static Vector CrossProduct (const Vector& first, const Vector& second);

  private:
    const Point vector_end_point_;
};

struct Triangle
{
  public:
    Triangle (const Point& A, const Point& B, const Point& C) : A_(A), B_(B), C_(C) {}

  private:
    const Point A_;
    const Point B_;
    const Point C_;
};

// bool IntersectionCheck (const Triangle& first_triangle, const Triangle& second_triangle);
}
