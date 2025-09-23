#pragma once

namespace Triangles
{
struct Point
{
  public:
    Point (float x, float y, float z) : x_(x), y_(y), z_(z) {}

    Point operator+ (const Point& point) const
    {
        return Point (x_ + point.x_, y_ + point.y_, z_ + point.z_);
    }

    Point operator- (const Point& point) const
    {
        return Point (x_ - point.x_, y_ - point.y_, z_ - point.z_);
    }

    float GetX () const { return x_; }
    float GetY () const { return y_; }
    float GetZ () const { return z_; }

  private:
    const float x_;
    const float y_;
    const float z_;
};

struct Vector
{
  public:
    Vector (const Point vector_end_point) : vector_end_point_ (vector_end_point) {}
    Vector (const Point& vector_start_point, const Point& vector_end_point) : vector_end_point_ (vector_end_point - vector_start_point) {}

    float GetX () const { return vector_end_point_.GetX (); }
    float GetY () const { return vector_end_point_.GetY (); }
    float GetZ () const { return vector_end_point_.GetZ (); }

    static float DotProduct    (const Vector& first, const Vector& second);
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

bool IntersectionCheck (const Triangle& first_triangle, const Triangle& second_triangle);
}
