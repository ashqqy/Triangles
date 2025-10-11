#include <gtest/gtest.h>

#include "triangles.hpp"

int main(int argc, char **argv)
{
    testing::InitGoogleTest (&argc, argv);

    return RUN_ALL_TESTS ();
}

// -----------------------------------------------------------------------------
// ------------------------- Floating point comparsion -------------------------
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// ---------------------------- Vectors dot product ----------------------------
// -----------------------------------------------------------------------------

using namespace Triangles;

TEST (vector_dot_product, basic)
{
    Vector<double, 3> first_3d {5, 6, 7};
    Vector<double, 3> second_3d {1, 3, 2};
    double answer_3d = first_3d.DotProduct(second_3d);
    EXPECT_DOUBLE_EQ (answer_3d, 5 * 1 + 6 * 3 + 7 * 2);

    Vector<float, 3> first_3f {5, 6, 7};
    Vector<float, 3> second_3f {1, 3, 2};
    float answer_3f = first_3f.DotProduct(second_3f);
    EXPECT_FLOAT_EQ (answer_3f, 5 * 1 + 6 * 3 + 7 * 2);

    Vector<double, 2> first_2d {5, 6};
    Vector<double, 2> second_2d {1, 3};
    double answer_2d = first_2d.DotProduct(second_2d);
    EXPECT_DOUBLE_EQ (answer_2d, 5 * 1 + 6 * 3);

    Vector<float, 2> first_2f {5, 6};
    Vector<float, 2> second_2f {1, 3};
    float answer_2f = first_2f.DotProduct(second_2f);
    EXPECT_FLOAT_EQ (answer_2f, 5 * 1 + 6 * 3);
}

TEST (vector_dot_product, basic2)
{
    Vector<double, 3> first_3d {5.5, 6.6, 3.0};
    Vector<double, 3> second_3d {-2.0, 3.9, 8};
    double answer_3d = first_3d.DotProduct(second_3d);
    EXPECT_DOUBLE_EQ (answer_3d, 38.74);

    Vector<float, 3> first_3f {5.5, 6.6, 3.0};
    Vector<float, 3> second_3f {-2.0, 3.9, 8};
    float answer_3f = first_3f.DotProduct(second_3f);
    EXPECT_FLOAT_EQ (answer_3f, 5.5 * (-2.0) + 6.6 * 3.9 + 3.0 * 8);

    Vector<double, 2> first_2d {5.5, 6.6};
    Vector<double, 2> second_2d {-2.0, 3.9};
    double answer_2d = first_2d.DotProduct(second_2d);
    EXPECT_DOUBLE_EQ (answer_2d, 5.5 * (-2.0) + 6.6 * 3.9);

    Vector<float, 2> first_2f {5.5, 6.6};
    Vector<float, 2> second_2f {-2.0, 3.9};
    float answer_2f = first_2f.DotProduct(second_2f);
    EXPECT_FLOAT_EQ (answer_2f, 5.5 * (-2.0) + 6.6 * 3.9);
}

TEST (vector_dot_product, large)
{
    Vector<double, 3> first_3d {2e6, -1e6, -3e6};
    Vector<double, 3> second_3d {-4e6, 5e6, 6e6};
    double answer_3d = first_3d.DotProduct(second_3d);
    EXPECT_DOUBLE_EQ (answer_3d, -3.1e13);

    Vector<float, 3> first_3f {2e6, -1e6, -3e6};
    Vector<float, 3> second_3f {-4e6, 5e6, 6e6};
    float answer_3f = first_3f.DotProduct(second_3f);
    EXPECT_FLOAT_EQ (answer_3f, -3.1e13);

    Vector<double, 2> first_2d {2e6, -1e6};
    Vector<double, 2> second_2d {-4e6, 5e6};
    double answer_2d = first_2d.DotProduct(second_2d);
    EXPECT_DOUBLE_EQ (answer_2d, -1.3e13);

    Vector<float, 2> first_2f {2e6, -1e6};
    Vector<float, 2> second_2f {-4e6, 5e6};
    float answer_2f = first_2f.DotProduct(second_2f);
    EXPECT_FLOAT_EQ (answer_2f, -1.3e13);
}

TEST (vector_dot_product, ortho)
{
    Vector<double, 3> first_3d {1.0, 1.0, 1.0};
    Vector<double, 3> second_3d {-1.0, 1.0, 0.0};
    double answer_3d = first_3d.DotProduct(second_3d);
    EXPECT_DOUBLE_EQ (answer_3d, 0.0);

    Vector<float, 3> first_3f {1.0, 1.0, 1.0};
    Vector<float, 3> second_3f {-1.0, 1.0, 0.0};
    float answer_3f = first_3f.DotProduct(second_3f);
    EXPECT_FLOAT_EQ (answer_3f, 0.0f);

    Vector<double, 2> first_2d {1.0, 1.0};
    Vector<double, 2> second_2d {-1.0, 1.0};
    double answer_2d = first_2d.DotProduct(second_2d);
    EXPECT_DOUBLE_EQ (answer_2d, 0.0);

    Vector<float, 2> first_2f {1.0, 1.0};
    Vector<float, 2> second_2f {-1.0, 1.0};
    float answer_2f = first_2f.DotProduct(second_2f);
    EXPECT_FLOAT_EQ (answer_2f, 0.0f);
}

TEST (vector_dot_product, parallel)
{
    Vector<double, 3> first_3d {1.0, 1.0, 1.0};
    Vector<double, 3> second_3d {3.0, 3.0, 3.0};
    double answer_3d = first_3d.DotProduct(second_3d);
    EXPECT_DOUBLE_EQ (answer_3d, 9.0);

    Vector<float, 3> first_3f {1.0, 1.0, 1.0};
    Vector<float, 3> second_3f {3.0, 3.0, 3.0};
    float answer_3f = first_3f.DotProduct(second_3f);
    EXPECT_FLOAT_EQ (answer_3f, 9.0f);

    Vector<double, 2> first_2d {1.0, 1.0};
    Vector<double, 2> second_2d {3.0, 3.0};
    double answer_2d = first_2d.DotProduct(second_2d);
    EXPECT_DOUBLE_EQ (answer_2d, 6.0);

    Vector<float, 2> first_2f {1.0, 1.0};
    Vector<float, 2> second_2f {3.0, 3.0};
    float answer_2f = first_2f.DotProduct(second_2f);
    EXPECT_FLOAT_EQ (answer_2f, 6.0f);
}

TEST (vector_dot_product, zero_vector)
{
    Vector<double, 3> first_3d {0.0, 0.0, 0.0};
    Vector<double, 3> second_3d {1.0, 2.0, 3.0};
    double answer_3d = first_3d.DotProduct(second_3d);
    EXPECT_DOUBLE_EQ (answer_3d, 0.0);

    Vector<float, 3> first_3f {0.0f, 0.0f, 0.0f};
    Vector<float, 3> second_3f {1.0f, 2.0f, 3.0f};
    float answer_3f = first_3f.DotProduct(second_3f);
    EXPECT_FLOAT_EQ (answer_3f, 0.0f);

    Vector<double, 2> first_2d {0.0, 0.0};
    Vector<double, 2> second_2d {1.0, 2.0};
    double answer_2d = first_2d.DotProduct(second_2d);
    EXPECT_DOUBLE_EQ (answer_2d, 0.0);

    Vector<float, 2> first_2f {0.0f, 0.0f};
    Vector<float, 2> second_2f {1.0f, 2.0f};
    float answer_2f = first_2f.DotProduct(second_2f);
    EXPECT_FLOAT_EQ (answer_2f, 0.0f);
}

TEST (vector_dot_product, same)
{
    Vector<double, 3> first_3d {1.0, 2.0, 3.0};
    Vector<double, 3> second_3d {1.0, 2.0, 3.0};
    double answer_3d = first_3d.DotProduct(second_3d);
    EXPECT_DOUBLE_EQ (answer_3d, 14);

    Vector<float, 3> first_3f {1.0f, 2.0f, 3.0f};
    Vector<float, 3> second_3f {1.0f, 2.0f, 3.0f};
    float answer_3f = first_3f.DotProduct(second_3f);
    EXPECT_FLOAT_EQ (answer_3f, 14);

    Vector<double, 2> first_2d {1.0, 2.0};
    Vector<double, 2> second_2d {1.0, 2.0};
    double answer_2d = first_2d.DotProduct(second_2d);
    EXPECT_DOUBLE_EQ (answer_2d, 5);

    Vector<float, 2> first_2f {1.0f, 2.0f};
    Vector<float, 2> second_2f {1.0f, 2.0f};
    float answer_2f = first_2f.DotProduct(second_2f);
    EXPECT_FLOAT_EQ (answer_2f, 5);
}

// -----------------------------------------------------------------------------
// --------------------------- Vectors cross product ---------------------------
// -----------------------------------------------------------------------------

TEST (vector_cross_product, basic)
{
    Vector<double, 3> first_3d {5.5, 6.6, 3.0};
    Vector<double, 3> second_3d {-2.0, 3.9, 8};
    Vector<double, 3> answer_3d = first_3d.CrossProduct(second_3d);
    EXPECT_DOUBLE_EQ (answer_3d[0], 41.1);
    EXPECT_DOUBLE_EQ (answer_3d[1], -50);
    EXPECT_DOUBLE_EQ (answer_3d[2], 34.65);

    Vector<float, 3> first_3f {5.5, 6.6, 3.0};
    Vector<float, 3> second_3f {-2.0, 3.9, 8};
    Vector<float, 3> answer_3f = first_3f.CrossProduct(second_3f);
    EXPECT_FLOAT_EQ (answer_3f[0], 41.1f);
    EXPECT_FLOAT_EQ (answer_3f[1], -50.0f);
    EXPECT_FLOAT_EQ (answer_3f[2], 34.65f);
}

TEST (vector_cross_product, basic2)
{
    Vector<double, 3> first_3d {-4, -435, 33};
    Vector<double, 3> second_3d {324, 66, -800};
    Vector<double, 3> answer_3d = first_3d.CrossProduct(second_3d);
    EXPECT_DOUBLE_EQ (answer_3d[0], 345822);
    EXPECT_DOUBLE_EQ (answer_3d[1], 7492);
    EXPECT_DOUBLE_EQ (answer_3d[2], 140676);

    Vector<float, 3> first_3f {-4, -435, 33};
    Vector<float, 3> second_3f {324, 66, -800};
    Vector<float, 3> answer_3f = first_3f.CrossProduct(second_3f);
    EXPECT_FLOAT_EQ (answer_3f[0], 345822.0f);
    EXPECT_FLOAT_EQ (answer_3f[1], 7492.0f);
    EXPECT_FLOAT_EQ (answer_3f[2], 140676.0f);
}

TEST (vector_cross_product, large)
{
    Vector<double, 3> first_3d {2e6, -1e6, -3e6};
    Vector<double, 3> second_3d {-4e6, 5e6, 6e6};
    Vector<double, 3> answer_3d = first_3d.CrossProduct(second_3d);
    EXPECT_DOUBLE_EQ (answer_3d[0], 9000000000000);
    EXPECT_DOUBLE_EQ (answer_3d[1], 0);
    EXPECT_DOUBLE_EQ (answer_3d[2], 6000000000000);

    Vector<float, 3> first_3f {2e6, -1e6, -3e6};
    Vector<float, 3> second_3f {-4e6, 5e6, 6e6};
    Vector<float, 3> answer_3f = first_3f.CrossProduct(second_3f);
    EXPECT_FLOAT_EQ (answer_3f[0], 9000000000000.0f);
    EXPECT_FLOAT_EQ (answer_3f[1], 0.0f);
    EXPECT_FLOAT_EQ (answer_3f[2], 6000000000000.0f);
}

TEST (vector_cross_product, ortho)
{
    Vector<double, 3> first_3d {1.0, 1.0, 1.0};
    Vector<double, 3> second_3d {-1.0, 1.0, 0.0};
    Vector<double, 3> answer_3d = first_3d.CrossProduct(second_3d);
    EXPECT_DOUBLE_EQ (answer_3d[0], -1);
    EXPECT_DOUBLE_EQ (answer_3d[1], -1);
    EXPECT_DOUBLE_EQ (answer_3d[2], 2);

    Vector<float, 3> first_3f {1.0, 1.0, 1.0};
    Vector<float, 3> second_3f {-1.0, 1.0, 0.0};
    Vector<float, 3> answer_3f = first_3f.CrossProduct(second_3f);
    EXPECT_FLOAT_EQ (answer_3f[0], -1.0f);
    EXPECT_FLOAT_EQ (answer_3f[1], -1.0f);
    EXPECT_FLOAT_EQ (answer_3f[2], 2.0f);
}

TEST (vector_cross_product, parallel)
{
    Vector<double, 3> first_3d {1.0, 1.0, 1.0};
    Vector<double, 3> second_3d {3.0, 3.0, 3.0};
    Vector<double, 3> answer_3d = first_3d.CrossProduct(second_3d);
    EXPECT_DOUBLE_EQ (answer_3d[0], 0);
    EXPECT_DOUBLE_EQ (answer_3d[1], 0);
    EXPECT_DOUBLE_EQ (answer_3d[2], 0);

    Vector<float, 3> first_3f {1.0, 1.0, 1.0};
    Vector<float, 3> second_3f {3.0, 3.0, 3.0};
    Vector<float, 3> answer_3f = first_3f.CrossProduct(second_3f);
    EXPECT_FLOAT_EQ (answer_3f[0], 0.0f);
    EXPECT_FLOAT_EQ (answer_3f[1], 0.0f);
    EXPECT_FLOAT_EQ (answer_3f[2], 0.0f);
}

TEST (vector_cross_product, zero_vector)
{
    Vector<double, 3> first_3d {0.0, 0.0, 0.0};
    Vector<double, 3> second_3d {1.0, 2.0, 3.0};
    Vector<double, 3> answer_3d = first_3d.CrossProduct(second_3d);
    EXPECT_DOUBLE_EQ (answer_3d[0], 0.0);
    EXPECT_DOUBLE_EQ (answer_3d[1], 0.0);
    EXPECT_DOUBLE_EQ (answer_3d[2], 0.0);

    Vector<float, 3> first_3f {1.0f, 2.0f, 3.0f};
    Vector<float, 3> second_3f {0.0f, 0.0f, 0.0f};
    Vector<float, 3> answer_3f = first_3f.CrossProduct(second_3f);
    EXPECT_FLOAT_EQ (answer_3f[0], 0.0f);
    EXPECT_FLOAT_EQ (answer_3f[1], 0.0f);
    EXPECT_FLOAT_EQ (answer_3f[2], 0.0f);
}

// -----------------------------------------------------------------------------
// ------------------------- LineSegment.ContainsPoint -------------------------
// -----------------------------------------------------------------------------

TEST (segment_contains_point, 1d_basic)
{
    LineSegment<double, 1> segment_d(-1.0, 1.0);
    double point_inside_d = 0.0;
    double point_outside_d = -2.0;

    GTEST_EXPECT_TRUE (segment_d.ContainsPoint(point_inside_d));
    GTEST_EXPECT_FALSE (segment_d.ContainsPoint(point_outside_d));

    LineSegment<float, 1> segment_f(-1.0f, 1.0f);
    float point_inside_f = 0.0f;
    float point_outside_f = -2.0f;

    GTEST_EXPECT_TRUE (segment_f.ContainsPoint(point_inside_f));
    GTEST_EXPECT_FALSE (segment_f.ContainsPoint(point_outside_f));
}

TEST (segment_contains_point, 1d_borderline)
{
    LineSegment<double, 1> segment_d(-2.0, -1.0);
    double point_left_d = -2.0;
    double point_right_d = -1.0;

    GTEST_EXPECT_TRUE (segment_d.ContainsPoint(point_left_d));
    GTEST_EXPECT_TRUE (segment_d.ContainsPoint(point_right_d));

    LineSegment<float, 1> segment_f(-2.0f, -1.0f);
    float point_left_f = -2.0f;
    float point_right_f = -1.0f;

    GTEST_EXPECT_TRUE (segment_f.ContainsPoint(point_left_f));
    GTEST_EXPECT_TRUE (segment_f.ContainsPoint(point_right_f));
}

TEST (segment_contains_point, 2d_basic)
{
    LineSegment<double, 2> segment_d({0.0, 0.0}, {1.0, 1.0});
    Vector<double, 2> point_inside_d = {0.5, 0.5};
    Vector<double, 2> point_outside_d = {2.0, 5.0};

    GTEST_EXPECT_TRUE (segment_d.ContainsPoint(point_inside_d));
    GTEST_EXPECT_FALSE (segment_d.ContainsPoint(point_outside_d));

    LineSegment<float, 2> segment_f({0.0f, 0.0f}, {1.0f, 1.0f});
    Vector<float, 2> point_inside_f = {0.5f, 0.5f};
    Vector<float, 2> point_outside_f = {2.0f, 5.0f};

    GTEST_EXPECT_TRUE (segment_f.ContainsPoint(point_inside_f));
    GTEST_EXPECT_FALSE (segment_f.ContainsPoint(point_outside_f));
}

TEST (segment_contains_point, 2d_borderline)
{
    LineSegment<double, 2> segment_d({0.0, 0.0}, {1.0, 1.0});
    Vector<double, 2> point_start_d = {0.0, 0.0};
    Vector<double, 2> point_end_d = {1.0, 1.0};

    GTEST_EXPECT_TRUE (segment_d.ContainsPoint(point_start_d));
    GTEST_EXPECT_TRUE (segment_d.ContainsPoint(point_end_d));

    LineSegment<float, 2> segment_f({0.0f, 0.0f}, {1.0f, 1.0f});
    Vector<float, 2> point_start_f = {0.0f, 0.0f};
    Vector<float, 2> point_end_f = {1.0f, 1.0f};

    GTEST_EXPECT_TRUE (segment_f.ContainsPoint(point_start_f));
    GTEST_EXPECT_TRUE (segment_f.ContainsPoint(point_end_f));
}

TEST (segment_contains_point, 3d_basic)
{
    LineSegment<double, 3> segment_d({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0});
    Vector<double, 3> point_inside_d = {0.5, 0.5, 0.5};
    Vector<double, 3> point_outside_d = {2.0, 2.0, 5.0};

    GTEST_EXPECT_TRUE (segment_d.ContainsPoint(point_inside_d));
    GTEST_EXPECT_FALSE (segment_d.ContainsPoint(point_outside_d));

    LineSegment<float, 3> segment_f({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    Vector<float, 3> point_inside_f = {0.5f, 0.5f, 0.5f};
    Vector<float, 3> point_outside_f = {2.0f, 2.0f, 2.0f};

    GTEST_EXPECT_TRUE (segment_f.ContainsPoint(point_inside_f));
    GTEST_EXPECT_FALSE (segment_f.ContainsPoint(point_outside_f));
}

TEST (segment_contains_point, 3d_borderline)
{
    LineSegment<double, 3> segment_d({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0});
    Vector<double, 3> point_start_d = {0.0, 0.0, 0.0};
    Vector<double, 3> point_end_d = {1.0, 1.0, 1.0};

    GTEST_EXPECT_TRUE (segment_d.ContainsPoint(point_start_d));
    GTEST_EXPECT_TRUE (segment_d.ContainsPoint(point_end_d));

    LineSegment<float, 3> segment_f({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    Vector<float, 3> point_start_f = {0.0f, 0.0f, 0.0f};
    Vector<float, 3> point_end_f = {1.0f, 1.0f, 1.0f};

    GTEST_EXPECT_TRUE (segment_f.ContainsPoint(point_start_f));
    GTEST_EXPECT_TRUE (segment_f.ContainsPoint(point_end_f));
}

// -----------------------------------------------------------------------------
// ------------ LineSegment.CheckIntersection (with other segment) -------------
// -----------------------------------------------------------------------------

TEST (segment_segment_check_intersection, 1d_basic)
{
    LineSegment<double, 1> first_d(-1.0, 1.0);
    LineSegment<double, 1> second_intersect_d(0.0, 2.0);
    LineSegment<double, 1> second_not_intersect_d(1.1, 2.0);

    GTEST_EXPECT_TRUE  (first_d.CheckIntersection(second_intersect_d));
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_not_intersect_d));

    LineSegment<float, 1> first_f(-1.0f, 1.0f);
    LineSegment<float, 1> second_intersect_f(0.0f, 2.0f);
    LineSegment<float, 1> second_not_intersect_f(1.1f, 2.0f);

    GTEST_EXPECT_TRUE  (first_f.CheckIntersection(second_intersect_f));
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_not_intersect_f));
}

TEST (segment_segment_check_intersection, 1d_borderline)
{
    LineSegment<double, 1> first_d(-1.0, 1.0);
    LineSegment<double, 1> second_touch_left_d(-1.0, -0.5);
    LineSegment<double, 1> second_touch_right_d(1.0, 2.0);

    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_touch_left_d));
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_touch_right_d));

    LineSegment<float, 1> first_f(-1.0f, 1.0f);
    LineSegment<float, 1> second_touch_left_f(-1.0f, -0.5f);
    LineSegment<float, 1> second_touch_right_f(1.0f, 2.0f);

    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_touch_left_f));
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_touch_right_f));
}

TEST (segment_segment_check_intersection, 2d_basic)
{
    LineSegment<double, 2> first_d({0.0, 0.0}, {1.0, 1.0});
    LineSegment<double, 2> second_intersect_d({0.0, 1.0}, {1.0, 0.0});
    LineSegment<double, 2> second_not_intersect_d({2.0, 2.0}, {3.0, 3.0});

    GTEST_EXPECT_TRUE  (first_d.CheckIntersection(second_intersect_d));
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_not_intersect_d));

    LineSegment<float, 2> first_f({0.0f, 0.0f}, {1.0f, 1.0f});
    LineSegment<float, 2> second_intersect_f({0.0f, 1.0f}, {1.0f, 0.0f});
    LineSegment<float, 2> second_not_intersect_f({2.0f, 2.0f}, {3.0f, 3.0f});

    GTEST_EXPECT_TRUE  (first_f.CheckIntersection(second_intersect_f));
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_not_intersect_f));
}

TEST (segment_segment_check_intersection, 2d_borderline)
{
    LineSegment<double, 2> first_d({0.0, 0.0}, {1.0, 1.0});
    LineSegment<double, 2> second_touch_start_d({0.0, 0.0}, {0.0, 1.0});
    LineSegment<double, 2> second_touch_end_d({1.0, 1.0}, {2.0, 1.0});

    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_touch_start_d));
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_touch_end_d));

    LineSegment<float, 2> first_f({0.0f, 0.0f}, {1.0f, 1.0f});
    LineSegment<float, 2> second_touch_start_f({0.0f, 0.0f}, {0.0f, 1.0f});
    LineSegment<float, 2> second_touch_end_f({1.0f, 1.0f}, {2.0f, 1.0f});

    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_touch_start_f));
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_touch_end_f));
}

TEST (segment_segment_check_intersection, 2d_parallel)
{
    LineSegment<double, 2> first_d({0.0, 0.0}, {1.0, 1.0});
    LineSegment<double, 2> second_parallel_intersect_d({0.0, 1.0}, {1.0, 2.0});
    LineSegment<double, 2> second_parallel_not_intersect_d({2.0, 0.0}, {3.0, 1.0});

    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_parallel_intersect_d));
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_parallel_not_intersect_d));

    LineSegment<float, 2> first_f({0.0f, 0.0f}, {1.0f, 1.0f});
    LineSegment<float, 2> second_parallel_intersect_f({0.0f, 1.0f}, {1.0f, 2.0f});
    LineSegment<float, 2> second_parallel_not_intersect_f({2.0f, 0.0f}, {3.0f, 1.0f});

    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_parallel_intersect_f));
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_parallel_not_intersect_f));
}

TEST (segment_segment_check_intersection, 3d_basic)
{
    LineSegment<double, 3> first_d({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0});
    LineSegment<double, 3> second_intersect_d({0.0, 0.0, 1.0}, {1.0, 1.0, 0.0});
    LineSegment<double, 3> second_not_intersect_d({2.0, 2.0, 2.0}, {3.0, 3.0, 3.0});

    GTEST_EXPECT_TRUE  (first_d.CheckIntersection(second_intersect_d));
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_not_intersect_d));

    LineSegment<float, 3> first_f({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    LineSegment<float, 3> second_intersect_f({0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 0.0f});
    LineSegment<float, 3> second_not_intersect_f({2.0f, 2.0f, 2.0f}, {3.0f, 3.0f, 3.0f});

    GTEST_EXPECT_TRUE  (first_f.CheckIntersection(second_intersect_f));
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_not_intersect_f));
}

TEST (segment_segment_check_intersection, 3d_borderline)
{
    LineSegment<double, 3> first_d({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0});
    LineSegment<double, 3> second_touch_start_d({0.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    LineSegment<double, 3> second_touch_end_d({1.0, 1.0, 1.0}, {2.0, 1.0, 1.0});

    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_touch_start_d));
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_touch_end_d));

    LineSegment<float, 3> first_f({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    LineSegment<float, 3> second_touch_start_f({0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    LineSegment<float, 3> second_touch_end_f({1.0f, 1.0f, 1.0f}, {2.0f, 1.0f, 1.0f});

    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_touch_start_f));
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_touch_end_f));
}

TEST (segment_segment_check_intersection, 3d_parallel)
{
    LineSegment<double, 3> first_d({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0});
    LineSegment<double, 3> second_parallel_intersect_d({0.5, 0.5, 0.5}, {1.5, 1.5, 1.5});
    LineSegment<double, 3> second_parallel_not_intersect_d({2.0, 2.0, 2.0}, {3.0, 3.0, 3.0});

    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_parallel_intersect_d));
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_parallel_not_intersect_d));

    LineSegment<float, 3> first_f({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    LineSegment<float, 3> second_parallel_intersect_f({0.5f, 0.5f, 0.5f}, {1.5f, 1.5f, 1.5f});
    LineSegment<float, 3> second_parallel_not_intersect_f({2.0f, 2.0f, 2.0f}, {3.0f, 3.0f, 3.0f});

    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_parallel_intersect_f));
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_parallel_not_intersect_f));
}

TEST (segment_segment_check_intersection, 3d_skew)
{
    LineSegment<double, 3> first_d({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    LineSegment<double, 3> second_skew_d({0.5, -1.0, 1.0}, {0.5, 1.0, 1.0});

    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_skew_d));

    LineSegment<float, 3> first_f({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f});
    LineSegment<float, 3> second_skew_f({0.5f, -1.0f, 1.0f}, {0.5f, 1.0f, 1.0f});

    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_skew_f));
}

// -----------------------------------------------------------------------------
// ---------------------------- Plane.ContainsPoint ----------------------------
// -----------------------------------------------------------------------------

TEST (plane_contains_point, different_planes)
{
    Plane<double> plane_xz_d({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    Vector<double, 3> point_on_xz_d{0.5, 0.0, 0.5};
    Vector<double, 3> point_off_xz_d{0.5, 1.0, 0.5};

    GTEST_EXPECT_TRUE  (plane_xz_d.ContainsPoint(point_on_xz_d));
    GTEST_EXPECT_FALSE (plane_xz_d.ContainsPoint(point_off_xz_d));

    Plane<float> plane_xz_f({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f});
    Vector<float, 3> point_on_xz_f{0.5f, 0.0f, 0.5f};
    Vector<float, 3> point_off_xz_f{0.5f, 1.0f, 0.5f};

    GTEST_EXPECT_TRUE  (plane_xz_f.ContainsPoint(point_on_xz_f));
    GTEST_EXPECT_FALSE (plane_xz_f.ContainsPoint(point_off_xz_f));
}

TEST (plane_contains_point, tilted_plane)
{
    Plane<double> plane_tilted_d({1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0});
    Vector<double, 3> point_on_tilted_d{0.5, 0.3, 0.2};
    Vector<double, 3> point_off_tilted_d{0.5, 0.5, 0.5};

    GTEST_EXPECT_TRUE  (plane_tilted_d.ContainsPoint(point_on_tilted_d));

    GTEST_EXPECT_FALSE (plane_tilted_d.ContainsPoint(point_off_tilted_d));

    Plane<float> plane_tilted_f({1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f});
    Vector<float, 3> point_on_tilted_f{0.5f, 0.3f, 0.2f};
    Vector<float, 3> point_off_tilted_f{0.5f, 0.5f, 0.5f};

    GTEST_EXPECT_TRUE  (plane_tilted_f.ContainsPoint(point_on_tilted_f));
    GTEST_EXPECT_FALSE (plane_tilted_f.ContainsPoint(point_off_tilted_f));
}

TEST (plane_contains_point, border_cases)
{
    Plane<double> plane_d({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    Vector<double, 3> point_origin_d{0.0, 0.0, 0.0};
    Vector<double, 3> point_on_edge1_d{1.0, 0.0, 0.0};
    Vector<double, 3> point_on_edge2_d{0.0, 1.0, 0.0};

    GTEST_EXPECT_TRUE (plane_d.ContainsPoint(point_origin_d));
    GTEST_EXPECT_TRUE (plane_d.ContainsPoint(point_on_edge1_d));
    GTEST_EXPECT_TRUE (plane_d.ContainsPoint(point_on_edge2_d));

    Plane<float> plane_f({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    Vector<float, 3> point_origin_f{0.0f, 0.0f, 0.0f};
    Vector<float, 3> point_on_edge1_f{1.0f, 0.0f, 0.0f};
    Vector<float, 3> point_on_edge2_f{0.0f, 1.0f, 0.0f};

    GTEST_EXPECT_TRUE (plane_f.ContainsPoint(point_origin_f));
    GTEST_EXPECT_TRUE (plane_f.ContainsPoint(point_on_edge1_f));
    GTEST_EXPECT_TRUE (plane_f.ContainsPoint(point_on_edge2_f));
}

TEST (plane_contains_point, point_in_plane_but_outside_triangle)
{
    Plane<double> plane_d({0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 1.0});
    
    Vector<double, 3> point_in_plane1_d{2.0, 2.0, 0.0};
    Vector<double, 3> point_in_plane2_d{0.0, 2.0, 2.0};
    Vector<double, 3> point_in_plane3_d{-1.0, -1.0, 0.0};
    
    GTEST_EXPECT_TRUE(plane_d.ContainsPoint(point_in_plane1_d));
    GTEST_EXPECT_TRUE(plane_d.ContainsPoint(point_in_plane2_d));
    GTEST_EXPECT_TRUE(plane_d.ContainsPoint(point_in_plane3_d));

    Plane<float> plane_f({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 1.0f});
    
    Vector<float, 3> point_in_plane1_f{2.0f, 2.0f, 0.0f};
    Vector<float, 3> point_in_plane2_f{0.0f, 2.0f, 2.0f};
    Vector<float, 3> point_in_plane3_f{-1.0f, -1.0f, 0.0f};
    
    GTEST_EXPECT_TRUE(plane_f.ContainsPoint(point_in_plane1_f));
    GTEST_EXPECT_TRUE(plane_f.ContainsPoint(point_in_plane2_f));
    GTEST_EXPECT_TRUE(plane_f.ContainsPoint(point_in_plane3_f));
}

// -----------------------------------------------------------------------------
// --------------------------- Plane.DistanceToPoint ---------------------------
// -----------------------------------------------------------------------------

TEST (plane_distance_to_point, basic_cases)
{
    Plane<double> plane_xy({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    
    Vector<double, 3> point_in_plane{0.5, 0.5, 0.0};
    Vector<double, 3> point_above{0.5, 0.5, 2.0};
    Vector<double, 3> point_below{0.5, 0.5, -3.0};
    
    EXPECT_DOUBLE_EQ(0.0, plane_xy.DistanceToPoint(point_in_plane));
    EXPECT_DOUBLE_EQ(2.0, plane_xy.DistanceToPoint(point_above));
    EXPECT_DOUBLE_EQ(-3.0, plane_xy.DistanceToPoint(point_below));

    Plane<float> plane_xy_f({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    
    Vector<float, 3> point_in_plane_f{0.5f, 0.5f, 0.0f};
    Vector<float, 3> point_above_f{0.5f, 0.5f, 2.0f};
    Vector<float, 3> point_below_f{0.5f, 0.5f, -3.0f};
    
    EXPECT_FLOAT_EQ(0.0f, plane_xy_f.DistanceToPoint(point_in_plane_f));
    EXPECT_FLOAT_EQ(2.0f, plane_xy_f.DistanceToPoint(point_above_f));
    EXPECT_FLOAT_EQ(-3.0f, plane_xy_f.DistanceToPoint(point_below_f));
}

TEST (plane_distance_to_point, tilted_plane)
{
    Plane<double> plane_tilted({1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0});
    
    Vector<double, 3> point_in_plane{0.3, 0.3, 0.4};
    Vector<double, 3> point_above{1.0, 1.0, 1.0};
    Vector<double, 3> point_below{0.0, 0.0, 0.0};
    
    EXPECT_NEAR(0.0, plane_tilted.DistanceToPoint(point_in_plane), 1e-10);
    EXPECT_NEAR(2.0 / std::sqrt(3), plane_tilted.DistanceToPoint(point_above), 1e-10);
    EXPECT_NEAR(-1.0 / std::sqrt(3), plane_tilted.DistanceToPoint(point_below), 1e-10);

    Plane<float> plane_tilted_f({1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f});
    
    Vector<float, 3> point_in_plane_f{0.3f, 0.3f, 0.4f};
    Vector<float, 3> point_above_f{1.0f, 1.0f, 1.0f};
    Vector<float, 3> point_below_f{0.0f, 0.0f, 0.0f};
    
    EXPECT_NEAR(0.0f, plane_tilted_f.DistanceToPoint(point_in_plane_f), 1e-6f);
    EXPECT_NEAR(2.0f / std::sqrt(3.0f), plane_tilted_f.DistanceToPoint(point_above_f), 1e-6f);
    EXPECT_NEAR(-1.0f / std::sqrt(3.0f), plane_tilted_f.DistanceToPoint(point_below_f), 1e-6f);
}

TEST (plane_distance_to_point, negative_offset)
{
    Plane<double> plane_neg({0.0, 0.0, -2.0}, {1.0, 0.0, -2.0}, {0.0, 1.0, -2.0});
    
    Vector<double, 3> point_at_plane{1.0, 2.0, -2.0};
    Vector<double, 3> point_above{0.0, 0.0, 0.0};
    Vector<double, 3> point_below{0.0, 0.0, -5.0};
    
    EXPECT_DOUBLE_EQ(0.0, plane_neg.DistanceToPoint(point_at_plane));
    EXPECT_DOUBLE_EQ(2.0, plane_neg.DistanceToPoint(point_above));
    EXPECT_DOUBLE_EQ(-3.0, plane_neg.DistanceToPoint(point_below));

    Plane<float> plane_neg_f({0.0f, 0.0f, -2.0f}, {1.0f, 0.0f, -2.0f}, {0.0f, 1.0f, -2.0f});
    
    Vector<float, 3> point_at_plane_f{1.0f, 2.0f, -2.0f};
    Vector<float, 3> point_above_f{0.0f, 0.0f, 0.0f};
    Vector<float, 3> point_below_f{0.0f, 0.0f, -5.0f};
    
    EXPECT_FLOAT_EQ(0.0f, plane_neg_f.DistanceToPoint(point_at_plane_f));
    EXPECT_FLOAT_EQ(2.0f, plane_neg_f.DistanceToPoint(point_above_f));
    EXPECT_FLOAT_EQ(-3.0f, plane_neg_f.DistanceToPoint(point_below_f));
}

TEST (plane_distance_to_point, border_cases)
{
    Plane<double> plane_origin({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    
    Vector<double, 3> origin{0.0, 0.0, 0.0};
    Vector<double, 3> far_away{0.0, 0.0, 100.0};
    
    EXPECT_DOUBLE_EQ(0.0, plane_origin.DistanceToPoint(origin));
    EXPECT_DOUBLE_EQ(100.0, plane_origin.DistanceToPoint(far_away));

    Plane<float> plane_origin_f({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    
    Vector<float, 3> origin_f{0.0f, 0.0f, 0.0f};
    Vector<float, 3> far_away_f{0.0f, 0.0f, 100.0f};
    
    EXPECT_FLOAT_EQ(0.0f, plane_origin_f.DistanceToPoint(origin_f));
    EXPECT_FLOAT_EQ(100.0f, plane_origin_f.DistanceToPoint(far_away_f));
}

TEST (plane_distance_to_point, point_in_plane_but_outside_triangle)
{
    Plane<double> plane_xy({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    
    Vector<double, 3> point_outside1{2.0, 0.0, 0.0};
    Vector<double, 3> point_outside2{0.0, 2.0, 0.0};
    Vector<double, 3> point_outside3{-1.0, 0.0, 0.0};
    
    EXPECT_DOUBLE_EQ(0.0, plane_xy.DistanceToPoint(point_outside1));
    EXPECT_DOUBLE_EQ(0.0, plane_xy.DistanceToPoint(point_outside2));
    EXPECT_DOUBLE_EQ(0.0, plane_xy.DistanceToPoint(point_outside3));

    Plane<float> plane_xy_f({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    
    Vector<float, 3> point_outside1_f{2.0f, 0.0f, 0.0f};
    Vector<float, 3> point_outside2_f{0.0f, 2.0f, 0.0f};
    Vector<float, 3> point_outside3_f{-1.0f, 0.0f, 0.0f};
    
    EXPECT_FLOAT_EQ(0.0f, plane_xy_f.DistanceToPoint(point_outside1_f));
    EXPECT_FLOAT_EQ(0.0f, plane_xy_f.DistanceToPoint(point_outside2_f));
    EXPECT_FLOAT_EQ(0.0f, plane_xy_f.DistanceToPoint(point_outside3_f));
}

// -----------------------------------------------------------------------------
// -------------------------- Triangle.ContainsPoint ---------------------------
// -----------------------------------------------------------------------------

TEST (triangle_contains_point, 2d_basic)
{
    Triangle<double, 2> triangle_d({0, 0}, {1, 0}, {0, 1});
    Vector<double, 2> point_inside_d{0.25, 0.25};
    Vector<double, 2> point_outside_d{0.75, 0.75};
    Vector<double, 2> point_on_edge_d{0.5, 0.0};
    Vector<double, 2> point_on_vertex_d{0.0, 1.0};

    GTEST_EXPECT_TRUE(triangle_d.ContainsPoint(point_inside_d));
    GTEST_EXPECT_FALSE(triangle_d.ContainsPoint(point_outside_d));
    GTEST_EXPECT_TRUE(triangle_d.ContainsPoint(point_on_edge_d));
    GTEST_EXPECT_TRUE(triangle_d.ContainsPoint(point_on_vertex_d));

    Triangle<float, 2> triangle_f({0, 0}, {1, 0}, {0, 1});
    Vector<float, 2> point_inside_f{0.25, 0.25};
    Vector<float, 2> point_outside_f{0.75, 0.75};
    Vector<float, 2> point_on_edge_f{0.5, 0.0};
    Vector<float, 2> point_on_vertex_f{0.0, 1.0};

    GTEST_EXPECT_TRUE(triangle_f.ContainsPoint(point_inside_f));
    GTEST_EXPECT_FALSE(triangle_f.ContainsPoint(point_outside_f));
    GTEST_EXPECT_TRUE(triangle_f.ContainsPoint(point_on_edge_f));
    GTEST_EXPECT_TRUE(triangle_f.ContainsPoint(point_on_vertex_f));
}

TEST (triangle_contains_point, 2d_border_cases)
{
    Triangle<double, 2> triangle_d({0, 0}, {1, 0}, {0, 1});
    Vector<double, 2> point_on_border1{0.3, 0.7};
    Vector<double, 2> point_on_border2{0.7, 0.3};
    Vector<double, 2> point_close_outside{0.51, 0.51};
    Vector<double, 2> point_close_inside{0.49, 0.49};

    GTEST_EXPECT_TRUE(triangle_d.ContainsPoint(point_on_border1));
    GTEST_EXPECT_TRUE(triangle_d.ContainsPoint(point_on_border2));
    GTEST_EXPECT_FALSE(triangle_d.ContainsPoint(point_close_outside));
    GTEST_EXPECT_TRUE(triangle_d.ContainsPoint(point_close_inside));

    Triangle<float, 2> triangle_f({0, 0}, {1, 0}, {0, 1});
    Vector<float, 2> point_on_border1_f{0.3, 0.7};
    Vector<float, 2> point_on_border2_f{0.7, 0.3};
    Vector<float, 2> point_close_outside_f{0.51, 0.51};
    Vector<float, 2> point_close_inside_f{0.49, 0.49};

    GTEST_EXPECT_TRUE(triangle_f.ContainsPoint(point_on_border1_f));
    GTEST_EXPECT_TRUE(triangle_f.ContainsPoint(point_on_border2_f));
    GTEST_EXPECT_FALSE(triangle_f.ContainsPoint(point_close_outside_f));
    GTEST_EXPECT_TRUE(triangle_f.ContainsPoint(point_close_inside_f));
}

TEST (triangle_contains_point, 3d_basic)
{
    Triangle<double, 3> triangle_d({1, 0, 1}, {0, 0, 1}, {0, 1, 1});
    Vector<double, 3> point_inside_d{0.25, 0.25, 1};
    Vector<double, 3> point_outside_d{0.3, 0.3, 3};
    Vector<double, 3> point_on_edge_d{0.5, 0.0, 1};
    Vector<double, 3> point_on_vertex_d{0, 1, 1};

    GTEST_EXPECT_TRUE(triangle_d.ContainsPoint(point_inside_d));
    GTEST_EXPECT_FALSE(triangle_d.ContainsPoint(point_outside_d));
    GTEST_EXPECT_TRUE(triangle_d.ContainsPoint(point_on_edge_d));
    GTEST_EXPECT_TRUE(triangle_d.ContainsPoint(point_on_vertex_d));

    Triangle<float, 3> triangle_f({1, 0, 1}, {0, 0, 1}, {0, 1, 1});
    Vector<float, 3> point_inside_f{0.25, 0.25, 1};
    Vector<float, 3> point_outside_f{0.3, 0.3, 3};
    Vector<float, 3> point_on_edge_f{0.5, 0.0, 1};
    Vector<float, 3> point_on_vertex_f{0, 1, 1};

    GTEST_EXPECT_TRUE(triangle_f.ContainsPoint(point_inside_f));
    GTEST_EXPECT_FALSE(triangle_f.ContainsPoint(point_outside_f));
    GTEST_EXPECT_TRUE(triangle_f.ContainsPoint(point_on_edge_f));
    GTEST_EXPECT_TRUE(triangle_f.ContainsPoint(point_on_vertex_f));
}

TEST (triangle_contains_point, 3d_tilted_plane)
{
    Triangle<double, 3> triangle_d({1, 0, 0}, {0, 1, 0}, {0, 0, 1});
    Vector<double, 3> point_inside_d{0.3, 0.3, 0.4};
    Vector<double, 3> point_outside_d{0.5, 0.5, 0.5};
    Vector<double, 3> point_on_edge_d{0.5, 0.5, 0.0};
    Vector<double, 3> point_on_vertex_d{1, 0, 0};

    GTEST_EXPECT_TRUE(triangle_d.ContainsPoint(point_inside_d));
    GTEST_EXPECT_FALSE(triangle_d.ContainsPoint(point_outside_d));
    GTEST_EXPECT_TRUE(triangle_d.ContainsPoint(point_on_edge_d));
    GTEST_EXPECT_TRUE(triangle_d.ContainsPoint(point_on_vertex_d));

    Triangle<float, 3> triangle_f({1, 0, 0}, {0, 1, 0}, {0, 0, 1});
    Vector<float, 3> point_inside_f{0.3, 0.3, 0.4};
    Vector<float, 3> point_outside_f{0.5, 0.5, 0.5};
    Vector<float, 3> point_on_edge_f{0.5, 0.5, 0.0};
    Vector<float, 3> point_on_vertex_f{1, 0, 0};

    GTEST_EXPECT_TRUE(triangle_f.ContainsPoint(point_inside_f));
    GTEST_EXPECT_FALSE(triangle_f.ContainsPoint(point_outside_f));
    GTEST_EXPECT_TRUE(triangle_f.ContainsPoint(point_on_edge_f));
    GTEST_EXPECT_TRUE(triangle_f.ContainsPoint(point_on_vertex_f));
}

TEST (triangle_contains_point, 3d_border_cases)
{
    Triangle<double, 3> triangle_d({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Vector<double, 3> point_origin{0, 0, 0};
    Vector<double, 3> point_on_xy_plane{0.2, 0.2, 0};
    Vector<double, 3> point_above_plane{0.2, 0.2, 0.1};
    Vector<double, 3> point_on_extended_edge{2, 0, 0};

    GTEST_EXPECT_TRUE(triangle_d.ContainsPoint(point_origin));
    GTEST_EXPECT_TRUE(triangle_d.ContainsPoint(point_on_xy_plane));
    GTEST_EXPECT_FALSE(triangle_d.ContainsPoint(point_above_plane));
    GTEST_EXPECT_FALSE(triangle_d.ContainsPoint(point_on_extended_edge));

    Triangle<float, 3> triangle_f({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Vector<float, 3> point_origin_f{0, 0, 0};
    Vector<float, 3> point_on_xy_plane_f{0.2, 0.2, 0};
    Vector<float, 3> point_above_plane_f{0.2, 0.2, 0.1};
    Vector<float, 3> point_on_extended_edge_f{2, 0, 0};

    GTEST_EXPECT_TRUE(triangle_f.ContainsPoint(point_origin_f));
    GTEST_EXPECT_TRUE(triangle_f.ContainsPoint(point_on_xy_plane_f));
    GTEST_EXPECT_FALSE(triangle_f.ContainsPoint(point_above_plane_f));
    GTEST_EXPECT_FALSE(triangle_f.ContainsPoint(point_on_extended_edge_f));
}

// -----------------------------------------------------------------------------
// ----------------- Triangle.CheckIntersection (with segment) -----------------
// -----------------------------------------------------------------------------

TEST (triangle_segment_check_intersection, 2d_basic)
{
    Triangle<double, 2> triangle_d({0, 0}, {1, 0}, {0, 1});
    
    LineSegment<double, 2> segment_inside_d({0.1, 0.1}, {0.2, 0.2});
    LineSegment<double, 2> segment_outside_d({0.8, 0.8}, {0.9, 0.9});
    LineSegment<double, 2> segment_crossing_d({-0.5, 0.25}, {0.5, 0.25});
    LineSegment<double, 2> segment_touching_vertex_d({-0.5, 1.0}, {0.0, 1.0});
    LineSegment<double, 2> segment_on_edge_d({0.2, 0.0}, {0.8, 0.0});

    GTEST_EXPECT_TRUE(triangle_d.CheckIntersection(segment_inside_d));
    GTEST_EXPECT_FALSE(triangle_d.CheckIntersection(segment_outside_d));
    GTEST_EXPECT_TRUE(triangle_d.CheckIntersection(segment_crossing_d));
    GTEST_EXPECT_TRUE(triangle_d.CheckIntersection(segment_touching_vertex_d));
    GTEST_EXPECT_TRUE(triangle_d.CheckIntersection(segment_on_edge_d));

    Triangle<float, 2> triangle_f({0, 0}, {1, 0}, {0, 1});
    
    LineSegment<float, 2> segment_inside_f({0.1f, 0.1f}, {0.2f, 0.2f});
    LineSegment<float, 2> segment_outside_f({0.8f, 0.8f}, {0.9f, 0.9f});
    LineSegment<float, 2> segment_crossing_f({-0.5f, 0.25f}, {0.5f, 0.25f});
    LineSegment<float, 2> segment_touching_vertex_f({-0.5f, 1.0f}, {0.0f, 1.0f});
    LineSegment<float, 2> segment_on_edge_f({0.2f, 0.0f}, {0.8f, 0.0f});

    GTEST_EXPECT_TRUE(triangle_f.CheckIntersection(segment_inside_f));
    GTEST_EXPECT_FALSE(triangle_f.CheckIntersection(segment_outside_f));
    GTEST_EXPECT_TRUE(triangle_f.CheckIntersection(segment_crossing_f));
    GTEST_EXPECT_TRUE(triangle_f.CheckIntersection(segment_touching_vertex_f));
    GTEST_EXPECT_TRUE(triangle_f.CheckIntersection(segment_on_edge_f));
}

TEST (triangle_segment_check_intersection, 2d_border_cases)
{
    Triangle<double, 2> triangle_d({0, 0}, {1, 0}, {0, 1});
    
    LineSegment<double, 2> segment_two_edges_d({-0.5, 0.5}, {0.5, -0.5});
    LineSegment<double, 2> segment_tangent_d({-0.5, 0.0}, {-0.5, 1.0});
    LineSegment<double, 2> segment_coincident_edge_d({0.0, 0.0}, {1.0, 0.0});
    LineSegment<double, 2> segment_short_border_d({0.499, 0.501}, {0.501, 0.499});

    GTEST_EXPECT_TRUE(triangle_d.CheckIntersection(segment_two_edges_d));
    GTEST_EXPECT_FALSE(triangle_d.CheckIntersection(segment_tangent_d));
    GTEST_EXPECT_TRUE(triangle_d.CheckIntersection(segment_coincident_edge_d));
    GTEST_EXPECT_TRUE(triangle_d.CheckIntersection(segment_short_border_d));
}

TEST (triangle_segment_check_intersection, 3d_basic)
{
    Triangle<double, 3> triangle_d({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    
    LineSegment<double, 3> segment_in_plane_d({0.1, 0.1, 0}, {0.2, 0.2, 0});
    LineSegment<double, 3> segment_above_d({0.1, 0.1, 1}, {0.2, 0.2, 1});
    LineSegment<double, 3> segment_crossing_d({0.25, 0.25, -1}, {0.25, 0.25, 1});
    LineSegment<double, 3> segment_on_edge_d({0.2, 0.0, 0}, {0.8, 0.0, 0});

    GTEST_EXPECT_TRUE(triangle_d.CheckIntersection(segment_in_plane_d));
    GTEST_EXPECT_FALSE(triangle_d.CheckIntersection(segment_above_d));
    GTEST_EXPECT_TRUE(triangle_d.CheckIntersection(segment_crossing_d));
    GTEST_EXPECT_TRUE(triangle_d.CheckIntersection(segment_on_edge_d));

    Triangle<float, 3> triangle_f({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    
    LineSegment<float, 3> segment_in_plane_f({0.1f, 0.1f, 0}, {0.2f, 0.2f, 0});
    LineSegment<float, 3> segment_above_f({0.1f, 0.1f, 1}, {0.2f, 0.2f, 1});
    LineSegment<float, 3> segment_crossing_f({0.25f, 0.25f, -1}, {0.25f, 0.25f, 1});
    LineSegment<float, 3> segment_on_edge_f({0.2f, 0.0f, 0}, {0.8f, 0.0f, 0});

    GTEST_EXPECT_TRUE(triangle_f.CheckIntersection(segment_in_plane_f));
    GTEST_EXPECT_FALSE(triangle_f.CheckIntersection(segment_above_f));
    GTEST_EXPECT_TRUE(triangle_f.CheckIntersection(segment_crossing_f));
    GTEST_EXPECT_TRUE(triangle_f.CheckIntersection(segment_on_edge_f));
}

TEST (triangle_segment_check_intersection, 3d_tilted_plane)
{
    Triangle<double, 3> triangle_d({1, 0, 0}, {0, 1, 0}, {0, 0, 1});
    
    LineSegment<double, 3> segment_in_plane_d({0.3, 0.3, 0.4}, {0.4, 0.4, 0.2});
    LineSegment<double, 3> segment_crossing_d({1, 1, 1}, {0, 0, 0});
    LineSegment<double, 3> segment_parallel_d({2, 0, 0}, {2, 1, 0});

    GTEST_EXPECT_TRUE(triangle_d.CheckIntersection(segment_in_plane_d));
    GTEST_EXPECT_TRUE(triangle_d.CheckIntersection(segment_crossing_d));
    GTEST_EXPECT_FALSE(triangle_d.CheckIntersection(segment_parallel_d));
}

// // -----------------------------------------------------------------------------
// // --------------------------- Whole algorithm tests ---------------------------
// // -----------------------------------------------------------------------------

TEST (triangles_intersection, basic_no_intersection_parallel_planes)
{
    Triangle<double, 3> first_d({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Triangle<double, 3> second_d({0, 0, 1}, {1, 0, 1}, {0, 1, 1});
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Triangle<float, 3> second_f({0, 0, 1}, {1, 0, 1}, {0, 1, 1});
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, basic_intersection_crossing_planes)
{
    Triangle<double, 3> first_d({0, 0, 0}, {2, 0, 0}, {0, 2, 0});
    Triangle<double, 3> second_d({1, 1, -1}, {1, 1, 1}, {1, -1, 0});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f({0, 0, 0}, {2, 0, 0}, {0, 2, 0});
    Triangle<float, 3> second_f({1, 1, -1}, {1, 1, 1}, {1, -1, 0});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, edge_touching_3d)
{
    Triangle<double, 3> first_d({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Triangle<double, 3> second_d({0, 0, 0}, {0, 0, 1}, {0, 1, 1});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Triangle<float, 3> second_f({0, 0, 0}, {0, 0, 1}, {0, 1, 1});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, vertex_touching_3d)
{
    Triangle<double, 3> first_d({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Triangle<double, 3> second_d({0, 0, 0}, {-1, 0, 1}, {0, -1, 1});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Triangle<float, 3> second_f({0, 0, 0}, {-1, 0, 1}, {0, -1, 1});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, coplanar_no_overlap)
{
    Triangle<double, 3> first_d({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Triangle<double, 3> second_d({2, 0, 0}, {3, 0, 0}, {2, 1, 0});
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Triangle<float, 3> second_f({2, 0, 0}, {3, 0, 0}, {2, 1, 0});
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, coplanar_edge_overlap)
{
    Triangle<double, 3> first_d({0, 0, 0}, {2, 0, 0}, {0, 2, 0});
    Triangle<double, 3> second_d({1, 0, 0}, {3, 0, 0}, {1, 2, 0});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f({0, 0, 0}, {2, 0, 0}, {0, 2, 0});
    Triangle<float, 3> second_f({1, 0, 0}, {3, 0, 0}, {1, 2, 0});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, coplanar_vertex_inside)
{
    Triangle<double, 3> first_d({0, 0, 0}, {2, 0, 0}, {0, 2, 0});
    Triangle<double, 3> second_d({0.5, 0.5, 0}, {1.5, 0.5, 0}, {0.5, 1.5, 0});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f({0, 0, 0}, {2, 0, 0}, {0, 2, 0});
    Triangle<float, 3> second_f({0.5, 0.5, 0}, {1.5, 0.5, 0}, {0.5, 1.5, 0});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, skew_lines_no_intersection)
{
    Triangle<double, 3> first_d({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Triangle<double, 3> second_d({0.5, 0.5, 1}, {1.5, 0.5, 1}, {0.5, 1.5, 1});
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Triangle<float, 3> second_f({0.5, 0.5, 1}, {1.5, 0.5, 1}, {0.5, 1.5, 1});
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, precision_edge_case_near_miss)
{
    Triangle<double, 3> first_d({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Triangle<double, 3> second_d({1.0001, 0, 0}, {2, 0, 0}, {1.0001, 1, 0});
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Triangle<float, 3> second_f({1.0001f, 0, 0}, {2, 0, 0}, {1.0001f, 1, 0});
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, precision_edge_case_near_hit)
{
    Triangle<double, 3> first_d({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Triangle<double, 3> second_d({0.9999, 0, 0}, {2, 0, 0}, {0.9999, 1, 0});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Triangle<float, 3> second_f({0.9999f, 0, 0}, {2, 0, 0}, {0.9999f, 1, 0});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, large_coordinates)
{
    Triangle<double, 3> first_d({0, 0, 0}, {1e6, 0, 0}, {0, 1e6, 0});
    Triangle<double, 3> second_d({0, 0, 1e6}, {1e6, 0, 1e6}, {0, 1e6, 1e6});
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f({0, 0, 0}, {1e3f, 0, 0}, {0, 1e3f, 0});
    Triangle<float, 3> second_f({0, 0, 1e3f}, {1e3f, 0, 1e3f}, {0, 1e3f, 1e3f});
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, small_triangles)
{
    Triangle<double, 3> first_d({0, 0, 0}, {1e-6, 0, 0}, {0, 1e-6, 0});
    Triangle<double, 3> second_d({0, 0, 1e-6}, {1e-6, 0, 1e-6}, {0, 1e-6, 1e-6});
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f({0, 0, 0}, {1e-3f, 0, 0}, {0, 1e-3f, 0});
    Triangle<float, 3> second_f({0, 0, 1e-3f}, {1e-3f, 0, 1e-3f}, {0, 1e-3f, 1e-3f});
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, perpendicular_intersection)
{
    Triangle<double, 3> first_d({0, 0, 0}, {2, 0, 0}, {0, 2, 0});
    Triangle<double, 3> second_d({1, 1, -1}, {1, 1, 1}, {2, 2, 0});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f({0, 0, 0}, {2, 0, 0}, {0, 2, 0});
    Triangle<float, 3> second_f({1, 1, -1}, {1, 1, 1}, {2, 2, 0});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, degenerate_point_triangle_false)
{
    Triangle<double, 3> first_d  ({1, 1, 1}, {1, 1, 1}, {1, 1, 1});
    Triangle<double, 3> second_d ({0, 0, 0}, {0, 1, 0}, {1, 0, 0});
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f  ({1, 1, 1}, {1, 1, 1}, {1, 1, 1});
    Triangle<float, 3> second_f ({0, 0, 0}, {0, 1, 0}, {1, 0, 0});
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, degenerate_point_triangle_true)
{
    Triangle<double, 3> first_d  ({0.35, 0.35, 0}, {0.35, 0.35, 0}, {0.35, 0.35, 0});
    Triangle<double, 3> second_d ({0, 0, 0}, {0, 1, 0}, {1, 0, 0});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f  ({0.35, 0.35, 0}, {0.35, 0.35, 0}, {0.35, 0.35, 0});
    Triangle<float, 3> second_f ({0, 0, 0}, {0, 1, 0}, {1, 0, 0});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, degenerate_segment_triangle_false)
{
    Triangle<double, 3> first_d  ({-0.35, 0.2, 3}, {-0.35, 0.2, 3}, {100, 100, 100});
    Triangle<double, 3> second_d ({0, 0, 0}, {0, -1, 0}, {-1, 0, 0});
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f  ({-0.35, 0.2, 3}, {-0.35, 0.2, 3}, {100, 100, 100});
    Triangle<float, 3> second_f ({0, 0, 0}, {0, -1, 0}, {-1, 0, 0});
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, degenerate_segment_triangle_true)
{
    Triangle<double, 3> first_d  ({-0.35, 0.2, 3}, {-0.35, 0.2, 3}, {-0.2, -0.2, -1});
    Triangle<double, 3> second_d ({0, 0, 0}, {0, -1, 0}, {-1, 0, 0});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f  ({-0.35, 0.2, 3}, {-0.35, 0.2, 3}, {-0.2, -0.2, -1});
    Triangle<float, 3> second_f ({0, 0, 0}, {0, -1, 0}, {-1, 0, 0});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, degenerate_segment_segment_true)
{
    Triangle<double, 3> first_d({1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {1.0, 0.0, 0.0});
    Triangle<double, 3> second_d({0.0, 0.0, 1.0}, {1.0, -1.0, -1.0}, {0.0, 0.0, 1.0});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f({1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f});
    Triangle<float, 3> second_f({0.0f, 0.0f, 1.0f}, {1.0f, -1.0f, -1.0f}, {0.0f, 0.0f, 1.0f});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection, degenerate_segment_segment_false)
{
    Triangle<double, 3> first_d  ({0.8, 0, 0}, {0, -1.4, 0}, {0.8, 0, 0});
    Triangle<double, 3> second_d ({0, 0, 0.8}, {0, 0, 0.8}, {-1, 0, 0});
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_d));

    Triangle<float, 3> first_f  ({0.8, 0, 0}, {0, -1.4, 0}, {0.8, 0, 0});
    Triangle<float, 3> second_f ({0, 0, 0.8}, {0, 0, 0.8}, {-1, 0, 0});
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, basic_no_intersection)
{
    Triangle<double, 2> first_d({0, 0}, {1, 0}, {0, 1});
    Triangle<double, 2> second_d({2, 0}, {3, 0}, {2, 1});
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({0, 0}, {1, 0}, {0, 1});
    Triangle<float, 2> second_f({2, 0}, {3, 0}, {2, 1});
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, basic_intersection)
{
    Triangle<double, 2> first_d({0, 0}, {2, 0}, {0, 2});
    Triangle<double, 2> second_d({1, 1}, {3, 1}, {1, 3});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({0, 0}, {2, 0}, {0, 2});
    Triangle<float, 2> second_f({1, 1}, {3, 1}, {1, 3});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, complete_overlap)
{
    Triangle<double, 2> first_d({0, 0}, {1, 0}, {0, 1});
    Triangle<double, 2> second_d({0, 0}, {1, 0}, {0, 1});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({0, 0}, {1, 0}, {0, 1});
    Triangle<float, 2> second_f({0, 0}, {1, 0}, {0, 1});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, vertex_touching)
{
    Triangle<double, 2> first_d({0, 0}, {1, 0}, {0, 1});
    Triangle<double, 2> second_d({0, 0}, {-1, 0}, {0, -1});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({0, 0}, {1, 0}, {0, 1});
    Triangle<float, 2> second_f({0, 0}, {-1, 0}, {0, -1});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, edge_touching)
{
    Triangle<double, 2> first_d({0, 0}, {1, 0}, {0, 1});
    Triangle<double, 2> second_d({0, 0}, {0, 1}, {-1, 0});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({0, 0}, {1, 0}, {0, 1});
    Triangle<float, 2> second_f({0, 0}, {0, 1}, {-1, 0});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, one_triangle_inside_other)
{
    Triangle<double, 2> first_d({0, 0}, {3, 0}, {0, 3});
    Triangle<double, 2> second_d({0.5, 0.5}, {1.5, 0.5}, {0.5, 1.5});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({0, 0}, {3, 0}, {0, 3});
    Triangle<float, 2> second_f({0.5, 0.5}, {1.5, 0.5}, {0.5, 1.5});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, shared_edge_only)
{
    Triangle<double, 2> first_d({0, 0}, {1, 0}, {0, 1});
    Triangle<double, 2> second_d({0, 0}, {1, 0}, {1, 1});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({0, 0}, {1, 0}, {0, 1});
    Triangle<float, 2> second_f({0, 0}, {1, 0}, {1, 1});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, vertex_inside_other_triangle)
{
    Triangle<double, 2> first_d({0, 0}, {2, 0}, {0, 2});
    Triangle<double, 2> second_d({0.5, 0.5}, {3, 0.5}, {0.5, 3});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({0, 0}, {2, 0}, {0, 2});
    Triangle<float, 2> second_f({0.5, 0.5}, {3, 0.5}, {0.5, 3});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, precision_near_miss)
{
    Triangle<double, 2> first_d({0, 0}, {1, 0}, {0, 1});
    Triangle<double, 2> second_d({1.0001, 0}, {2, 0}, {1.0001, 1});
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({0, 0}, {1, 0}, {0, 1});
    Triangle<float, 2> second_f({1.0001f, 0}, {2, 0}, {1.0001f, 1});
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, precision_near_hit)
{
    Triangle<double, 2> first_d({0, 0}, {1, 0}, {0, 1});
    Triangle<double, 2> second_d({0.9999, 0}, {2, 0}, {0.9999, 1});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({0, 0}, {1, 0}, {0, 1});
    Triangle<float, 2> second_f({0.9999f, 0}, {2, 0}, {0.9999f, 1});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, degenerate_triangle_point)
{
    Triangle<double, 2> first_d({0.5, 0.5}, {0.5, 0.5}, {0.5, 0.5});
    Triangle<double, 2> second_d({0, 0}, {1, 0}, {0, 1});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({0.5f, 0.5f}, {0.5f, 0.5f}, {0.5f, 0.5f});
    Triangle<float, 2> second_f({0, 0}, {1, 0}, {0, 1});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, degenerate_triangle_point_outside)
{
    Triangle<double, 2> first_d({2, 2}, {2, 2}, {2, 2});
    Triangle<double, 2> second_d({0, 0}, {1, 0}, {0, 1});
    GTEST_EXPECT_FALSE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({2, 2}, {2, 2}, {2, 2});
    Triangle<float, 2> second_f({0, 0}, {1, 0}, {0, 1});
    GTEST_EXPECT_FALSE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, degenerate_triangle_line)
{
    Triangle<double, 2> first_d({0, 0}, {1, 0}, {0.5, 0});
    Triangle<double, 2> second_d({0.5, -1}, {0.5, 1}, {1, 0});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({0, 0}, {1, 0}, {0.5, 0});
    Triangle<float, 2> second_f({0.5f, -1}, {0.5f, 1}, {1, 0});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, axis_aligned_triangles)
{
    Triangle<double, 2> first_d({0, 0}, {2, 0}, {0, 2});
    Triangle<double, 2> second_d({1, 0}, {3, 0}, {1, 2});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({0, 0}, {2, 0}, {0, 2});
    Triangle<float, 2> second_f({1, 0}, {3, 0}, {1, 2});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, rotated_triangles_intersect)
{
    Triangle<double, 2> first_d({0, 0}, {1, 1}, {-1, 1});
    Triangle<double, 2> second_d({0, 0.5}, {1, 1.5}, {-1, 1.5});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({0, 0}, {1, 1}, {-1, 1});
    Triangle<float, 2> second_f({0, 0.5f}, {1, 1.5f}, {-1, 1.5f});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}

TEST (triangles_intersection_2d, small_triangles)
{
    Triangle<double, 2> first_d({0, 0}, {1e-6, 0}, {0, 1e-6});
    Triangle<double, 2> second_d({0.5e-6, 0.5e-6}, {1.5e-6, 0.5e-6}, {0.5e-6, 1.5e-6});
    GTEST_EXPECT_TRUE (first_d.CheckIntersection(second_d));

    Triangle<float, 2> first_f({0, 0}, {1e-3f, 0}, {0, 1e-3f});
    Triangle<float, 2> second_f({0.5e-3f, 0.5e-3f}, {1.5e-3f, 0.5e-3f}, {0.5e-3f, 1.5e-3f});
    GTEST_EXPECT_TRUE (first_f.CheckIntersection(second_f));
}
