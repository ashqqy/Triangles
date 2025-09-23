#include "triangles.hpp"

namespace Triangles
{
inline float Vector::DotProduct (const Vector& first, const Vector& second)
{
    return first.GetX() * second.GetX() + first.GetY() * second.GetY() + first.GetZ() * second.GetZ();
}

inline Vector Vector::CrossProduct (const Vector& first, const Vector& second)
{
    float c_x = first.GetY () * second.GetZ () - first.GetZ () * second.GetY ();
    float c_y = first.GetZ () * second.GetX () - first.GetX () * second.GetZ ();
    float c_z = first.GetX () * second.GetY () - first.GetY () * second.GetX ();

    return Vector (Point (c_x, c_y, c_z));
}
}
