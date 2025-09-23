#pragma once

namespace Triangles
{
struct Point
{
    Point (float x, float y, float z) : x_(x), y_(y), z_(z) {}

  private:
    const float x_;
    const float y_;
    const float z_;
};

struct Triangle
{
    Triangle (const Point& A, const Point& B, const Point& C) : A_(A), B_(B), C_(C) {}


  private:
    const Point A_;
    const Point B_;
    const Point C_;
};

bool IntersectionCheck (const Triangle& first_triangle, const Triangle& second_triangle);
}
