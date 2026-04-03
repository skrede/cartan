#include <cartan/types.h>

#include <cartan/detail/epsilon.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("types: vector3 alias compiles and works", "[types]")
{
    cartan::vector3<double> v = cartan::vector3<double>::Zero();
    REQUIRE(v.norm() == 0.0);

    v = cartan::vector3<double>::UnitX();
    REQUIRE(v.norm() == Catch::Approx(1.0));
}

TEST_CASE("types: matrix4 alias compiles", "[types]")
{
    cartan::matrix4<double> m = cartan::matrix4<double>::Identity();
    REQUIRE(m(0, 0) == 1.0);
    REQUIRE(m(0, 1) == 0.0);
}

TEST_CASE("types: quaternion alias compiles", "[types]")
{
    cartan::quaternion<double> q = cartan::quaternion<double>::Identity();
    REQUIRE(q.w() == Catch::Approx(1.0));
    REQUIRE(q.vec().norm() == Catch::Approx(0.0));
}

TEST_CASE("types: float scalar variant compiles", "[types]")
{
    cartan::vector3<float> v = cartan::vector3<float>::UnitZ();
    REQUIRE(v.norm() == Catch::Approx(1.0f));
}

TEST_CASE("detail: epsilon_v is positive and small", "[detail]")
{
    REQUIRE(cartan::detail::epsilon_v<double> > 0.0);
    REQUIRE(cartan::detail::epsilon_v<double> < 1e-10);
    REQUIRE(cartan::detail::epsilon_v<float> > 0.0f);
    REQUIRE(cartan::detail::epsilon_v<float> < 1e-4f);
}

TEST_CASE("detail: sqrt_epsilon_v is between epsilon and 1", "[detail]")
{
    REQUIRE(cartan::detail::sqrt_epsilon_v<double> > cartan::detail::epsilon_v<double>);
    REQUIRE(cartan::detail::sqrt_epsilon_v<double> < 1.0);
}
