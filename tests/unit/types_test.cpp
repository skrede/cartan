#include <liepp/types.h>

#include <liepp/detail/epsilon.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("types: vector3 alias compiles and works", "[types]")
{
    liepp::vector3<double> v = liepp::vector3<double>::Zero();
    REQUIRE(v.norm() == 0.0);

    v = liepp::vector3<double>::UnitX();
    REQUIRE(v.norm() == Catch::Approx(1.0));
}

TEST_CASE("types: matrix4 alias compiles", "[types]")
{
    liepp::matrix4<double> m = liepp::matrix4<double>::Identity();
    REQUIRE(m(0, 0) == 1.0);
    REQUIRE(m(0, 1) == 0.0);
}

TEST_CASE("types: quaternion alias compiles", "[types]")
{
    liepp::quaternion<double> q = liepp::quaternion<double>::Identity();
    REQUIRE(q.w() == Catch::Approx(1.0));
    REQUIRE(q.vec().norm() == Catch::Approx(0.0));
}

TEST_CASE("types: float scalar variant compiles", "[types]")
{
    liepp::vector3<float> v = liepp::vector3<float>::UnitZ();
    REQUIRE(v.norm() == Catch::Approx(1.0f));
}

TEST_CASE("detail: epsilon_v is positive and small", "[detail]")
{
    REQUIRE(liepp::detail::epsilon_v<double> > 0.0);
    REQUIRE(liepp::detail::epsilon_v<double> < 1e-10);
    REQUIRE(liepp::detail::epsilon_v<float> > 0.0f);
    REQUIRE(liepp::detail::epsilon_v<float> < 1e-4f);
}

TEST_CASE("detail: sqrt_epsilon_v is between epsilon and 1", "[detail]")
{
    REQUIRE(liepp::detail::sqrt_epsilon_v<double> > liepp::detail::epsilon_v<double>);
    REQUIRE(liepp::detail::sqrt_epsilon_v<double> < 1.0);
}
