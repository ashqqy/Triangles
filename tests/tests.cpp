#include <gtest/gtest.h>

#include "triangles.hpp"

int main(int argc, char **argv)
{
    testing::InitGoogleTest (&argc, argv);

    return RUN_ALL_TESTS ();
}

// -----------------------------------------------------------------------------
// ---------------------------- Vectors dot product ----------------------------
// -----------------------------------------------------------------------------

TEST (vector_dot_product, int_small)
{
    Triangles::Vector first {5, 6, 7};
    Triangles::Vector second {1, 3, 2};
    double answer = Triangles::Vector::DotProduct (first, second);
    EXPECT_DOUBLE_EQ (answer, 37);
}

TEST (vector_dot_product, int_big)
{
    Triangles::Vector first {435, 345, 71};
    Triangles::Vector second {1546, 356, 289};
    double answer = Triangles::Vector::DotProduct (first, second);
    EXPECT_DOUBLE_EQ (answer, 815849);
}

TEST (vector_dot_product, int_neg)
{
    Triangles::Vector first {-4, -435, 33};
    Triangles::Vector second {324, 66, -800};
    double answer = Triangles::Vector::DotProduct (first, second);
    EXPECT_DOUBLE_EQ (answer, -56406);
}

TEST (vector_dot_product, small)
{
    Triangles::Vector first {5.5, 6.6, 3.0};
    Triangles::Vector second {-2.0, 3.9, 8};
    double answer = Triangles::Vector::DotProduct (first, second);
    EXPECT_DOUBLE_EQ (answer, 38.74);
}

TEST (vector_dot_product, large)
{
    Triangles::Vector first {2e6, -1e6, -3e6};
    Triangles::Vector second {-4e6, 5e6, 6e6};
    double answer = Triangles::Vector::DotProduct (first, second);
    EXPECT_DOUBLE_EQ (answer, -3.1e13);
}

TEST (vector_dot_product, ortho)
{
    Triangles::Vector first {1.0, 1.0, 1.0};
    Triangles::Vector second {-1.0, 1.0, 0.0};
    double answer = Triangles::Vector::DotProduct (first, second);
    EXPECT_DOUBLE_EQ (answer, 0.0);
}

TEST (vector_dot_product, parallel)
{
    Triangles::Vector first {1.0, 1.0, 1.0};
    Triangles::Vector second {3.0, 3.0, 3.0};
    double answer = Triangles::Vector::DotProduct (first, second);
    EXPECT_DOUBLE_EQ (answer, 9.0);
}

// -----------------------------------------------------------------------------
// --------------------------- Vectors cross product ---------------------------
// -----------------------------------------------------------------------------

TEST (vector_cross_product, int_small)
{
    Triangles::Vector first {5, 6, 7};
    Triangles::Vector second {1, 3, 2};
    Triangles::Vector answer = Triangles::Vector::CrossProduct (first, second);
    EXPECT_DOUBLE_EQ (answer.GetX (), -9);
    EXPECT_DOUBLE_EQ (answer.GetY (), -3);
    EXPECT_DOUBLE_EQ (answer.GetZ (), 9);
}

TEST (vector_cross_product, int_big)
{
    Triangles::Vector first {435, 345, 71};
    Triangles::Vector second {1546, 356, 289};
    Triangles::Vector answer = Triangles::Vector::CrossProduct (first, second);
    EXPECT_DOUBLE_EQ (answer.GetX (), 74429);
    EXPECT_DOUBLE_EQ (answer.GetY (), -15949);
    EXPECT_DOUBLE_EQ (answer.GetZ (), -378510);
}

TEST (vector_cross_product, int_neg)
{
    Triangles::Vector first {-4, -435, 33};
    Triangles::Vector second {324, 66, -800};
    Triangles::Vector answer = Triangles::Vector::CrossProduct (first, second);
    EXPECT_DOUBLE_EQ (answer.GetX (), 345822);
    EXPECT_DOUBLE_EQ (answer.GetY (), 7492);
    EXPECT_DOUBLE_EQ (answer.GetZ (), 140676);
}

TEST (vector_cross_product, small)
{
    Triangles::Vector first {5.5, 6.6, 3.0};
    Triangles::Vector second {-2.0, 3.9, 8};
    Triangles::Vector answer = Triangles::Vector::CrossProduct (first, second);
    EXPECT_DOUBLE_EQ (answer.GetX (), 41.1);
    EXPECT_DOUBLE_EQ (answer.GetY (), -50);
    EXPECT_DOUBLE_EQ (answer.GetZ (), 34.65);
}

TEST (vector_cross_product, large)
{
    Triangles::Vector first {2e6, -1e6, -3e6};
    Triangles::Vector second {-4e6, 5e6, 6e6};
    Triangles::Vector answer = Triangles::Vector::CrossProduct (first, second);
    EXPECT_DOUBLE_EQ (answer.GetX (), 9000000000000);
    EXPECT_DOUBLE_EQ (answer.GetY (), 0);
    EXPECT_DOUBLE_EQ (answer.GetZ (), 6000000000000);
}

TEST (vector_cross_product, ortho)
{
    Triangles::Vector first {1.0, 1.0, 1.0};
    Triangles::Vector second {-1.0, 1.0, 0.0};
    Triangles::Vector answer = Triangles::Vector::CrossProduct (first, second);
    EXPECT_DOUBLE_EQ (answer.GetX (), -1);
    EXPECT_DOUBLE_EQ (answer.GetY (), -1);
    EXPECT_DOUBLE_EQ (answer.GetZ (), 2);
}

TEST (vector_cross_product, parallel)
{
    Triangles::Vector first {1.0, 1.0, 1.0};
    Triangles::Vector second {3.0, 3.0, 3.0};
    Triangles::Vector answer = Triangles::Vector::CrossProduct (first, second);
    EXPECT_DOUBLE_EQ (answer.GetX (), 0);
    EXPECT_DOUBLE_EQ (answer.GetY (), 0);
    EXPECT_DOUBLE_EQ (answer.GetZ (), 0);
}

// -----------------------------------------------------------------------------
// -------------------------- Triangle.ContainsPoint ---------------------------
// -----------------------------------------------------------------------------

TEST (triangle_contains_point, contains_easy)
{
    Triangles::Triangle triangle(Triangles::Point (1, 0, 1), Triangles::Point(0, 0, 1), Triangles::Point(0, 1, 1));
    Triangles::Point point(0.25, 0.25, 1);

    GTEST_EXPECT_TRUE (triangle.ContainsPoint(point));
}

TEST (triangle_contains_point, not_contains_easy)
{
    Triangles::Triangle triangle(Triangles::Point (1, 0, 1), Triangles::Point(0, 0, 1), Triangles::Point(0, 1, 1));
    Triangles::Point point(1, 1, 1);

    GTEST_EXPECT_FALSE (triangle.ContainsPoint(point));
}

TEST (triangle_contains_point, lies_on_vertice)
{
    Triangles::Triangle triangle(Triangles::Point (1, 0, 1), Triangles::Point(0, 0, 1), Triangles::Point(0, 1, 1));
    Triangles::Point point(1, 0, 1);

    GTEST_EXPECT_TRUE (triangle.ContainsPoint(point));
}

TEST (triangle_contains_point, lies_on_edge)
{
    Triangles::Triangle triangle(Triangles::Point (1, 0, 1), Triangles::Point(0, 0, 1), Triangles::Point(0, 1, 1));
    Triangles::Point point(0.5, 0.5, 1);

    GTEST_EXPECT_TRUE (triangle.ContainsPoint(point));
}

TEST (triangle_contains_point, contains_hard)
{
    Triangles::Triangle triangle(Triangles::Point (4.56, -8.97, 1), Triangles::Point(4.5, -5, -3.3), Triangles::Point(3.38, -4.96, 5.7));
    Triangles::Point point(3.993390801589, -6.000985985925, 1.999181904135);

    GTEST_EXPECT_TRUE (triangle.ContainsPoint(point));
}

TEST (triangle_contains_point, not_contains_hard)
{
    Triangles::Triangle triangle(Triangles::Point (4.56, -8.97, 1), Triangles::Point(4.5, -5, -3.3), Triangles::Point(3.38, -4.96, 5.7));
    Triangles::Point point(0.72, -9.53, -2.23);

    GTEST_EXPECT_FALSE (triangle.ContainsPoint(point));
}

// -----------------------------------------------------------------------------
// --------------------------- Whole algorithm tests ---------------------------
// -----------------------------------------------------------------------------

TEST (triangles_intersection, not_intersect)
{
    Triangles::Triangle first  {Triangles::Point (1, 5, 1), Triangles::Point(-1, 3, 0), Triangles::Point(0.8, 1, 0)};
    Triangles::Triangle second {Triangles::Point (1.3, 2.3, 0), Triangles::Point(-2, 5, 0), Triangles::Point(-0.8, 6, 0)};
    bool answer = Triangles::CheckTrianglesIntersection (first, second);
    GTEST_EXPECT_FALSE (answer);
}

TEST (triangles_intersection, intersect)
{
    Triangles::Triangle first  {Triangles::Point (1, 5, 0), Triangles::Point(-1, 3, -0.3), Triangles::Point(0.8, 1, 0)};
    Triangles::Triangle second {Triangles::Point (1.3, 2.3, 0), Triangles::Point(-2, 5, 0), Triangles::Point(-0.8, 6, 0)};
    bool answer = Triangles::CheckTrianglesIntersection (first, second);
    GTEST_EXPECT_TRUE (answer);
}

TEST (triangles_intersection, same_edge_coplanar)
{
    Triangles::Triangle first  {Triangles::Point (0, 0, 0), Triangles::Point(0, 1, 0), Triangles::Point(1, 0, 0)};
    Triangles::Triangle second {Triangles::Point (0, 1, 0), Triangles::Point(1, 0, 0), Triangles::Point(1, 1, 0)};
    bool answer = Triangles::CheckTrianglesIntersection (first, second);
    GTEST_EXPECT_TRUE (answer);
}


TEST (triangles_intersection, same_edge)
{
    Triangles::Triangle first  {Triangles::Point (0, 0, 0), Triangles::Point(0, 1, 0), Triangles::Point(1, 0, 0)};
    Triangles::Triangle second {Triangles::Point (0, 1, 0), Triangles::Point(1, 0, 0), Triangles::Point(1, 1, 1)};
    bool answer = Triangles::CheckTrianglesIntersection (first, second);
    GTEST_EXPECT_TRUE (answer);
}
