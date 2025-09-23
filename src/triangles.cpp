#include "triangles.hpp"

namespace Triangles
{
double Vector::DotProduct (const Vector& first, const Vector& second)
{
    return first.GetX() * second.GetX() + first.GetY() * second.GetY() + first.GetZ() * second.GetZ();
}

Vector Vector::CrossProduct (const Vector& first, const Vector& second)
{
    double c_x = first.GetY () * second.GetZ () - first.GetZ () * second.GetY ();
    double c_y = first.GetZ () * second.GetX () - first.GetX () * second.GetZ ();
    double c_z = first.GetX () * second.GetY () - first.GetY () * second.GetX ();

    return Vector (c_x, c_y, c_z);
}
}
