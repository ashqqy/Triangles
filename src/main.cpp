#include <cassert>
#include <iostream>
#include <vector>

struct Point
{
    Point (int p_x, int p_y, int p_z) : x(p_x), y(p_y), z(p_z)
    {
    }

  private:
    const int x;
    const int y;
    const int z;
};

struct Triangle
{
    Triangle (const Point& t_A, const Point& t_B, const Point& t_C) : A(t_A), B(t_B), C(t_C)
    {
    }

  private:
    const Point A;
    const Point B;
    const Point C;
};

int main ()
{
    size_t triangles_count = 0;
    std::cin >> triangles_count;
    assert (0 < triangles_count && triangles_count < 1000000);

    std::vector<Triangle> triangles;
    triangles.reserve (triangles_count);

    for (size_t i = 0; i < triangles_count; ++i)
    {
        int x1, y1, z1, x2, y2, z2, x3, y3, z3;
        std::cin >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3;

        Point A (x1, y1, z1);
        Point B (x2, y2, z2);
        Point C (x3, y3, z3);
        
        triangles.emplace_back (A, B, C);
    }

    std::cout << "Прочитано треугольников: " << triangles.size() << std::endl;
}
