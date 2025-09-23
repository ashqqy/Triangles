#include <gtest/gtest.h>

#include "triangles.hpp"

int main(int argc, char **argv)
{
  testing::InitGoogleTest (&argc, argv);

  return RUN_ALL_TESTS ();
}

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
