#include <cartan/lie/se2.h>
#include <cartan/lie/se3.h>
#include <cartan/lie/so2.h>
#include <cartan/lie/so3.h>
#include <cartan/lie/hat_vee.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <rapidcheck/catch.h>

#include <cmath>
#include <numbers>
#include <exception>
#include <rapidcheck.h>

using Catch::Approx;

// ============================================================================
// Helper: matrix distance (uses Eigen .eval() to handle expression templates)
// ============================================================================

template <typename Derived1, typename Derived2>
double mat_dist(const Eigen::MatrixBase<Derived1>& a, const Eigen::MatrixBase<Derived2>& b)
{
    return (a - b).norm();
}

// ============================================================================
// RapidCheck generators
// ============================================================================

namespace rc
{

template <>
struct Arbitrary<cartan::so2<double>>
{
    static Gen<cartan::so2<double>> arbitrary()
    {
        return gen::exec([]
        {
            double theta = static_cast<double>(*gen::inRange(-3141, 3142)) / 1000.0;
            return cartan::so2<double>::exp(theta);
        });
    }
};

template <>
struct Arbitrary<cartan::se2<double>>
{
    static Gen<cartan::se2<double>> arbitrary()
    {
        return gen::exec([]
        {
            cartan::vector3<double> v;
            v(0) = static_cast<double>(*gen::inRange(-3000, 3001)) / 1000.0;
            v(1) = static_cast<double>(*gen::inRange(-5000, 5001)) / 1000.0;
            v(2) = static_cast<double>(*gen::inRange(-5000, 5001)) / 1000.0;
            return cartan::se2<double>::exp(v);
        });
    }
};

template <>
struct Arbitrary<cartan::so3<double>>
{
    static Gen<cartan::so3<double>> arbitrary()
    {
        return gen::exec([]
        {
            double x = static_cast<double>(*gen::inRange(-3000, 3001)) / 1000.0;
            double y = static_cast<double>(*gen::inRange(-3000, 3001)) / 1000.0;
            double z = static_cast<double>(*gen::inRange(-3000, 3001)) / 1000.0;
            cartan::vector3<double> phi;
            phi << x, y, z;
            double norm = phi.norm();
            if (norm > 3.13)
            {
                phi *= 3.13 / norm;
            }
            return cartan::so3<double>::exp(phi);
        });
    }
};

template <>
struct Arbitrary<cartan::se3<double>>
{
    static Gen<cartan::se3<double>> arbitrary()
    {
        return gen::exec([]
        {
            cartan::vector6<double> v;
            v(0) = static_cast<double>(*gen::inRange(-3000, 3001)) / 1000.0;
            v(1) = static_cast<double>(*gen::inRange(-3000, 3001)) / 1000.0;
            v(2) = static_cast<double>(*gen::inRange(-3000, 3001)) / 1000.0;
            v(3) = static_cast<double>(*gen::inRange(-5000, 5001)) / 1000.0;
            v(4) = static_cast<double>(*gen::inRange(-5000, 5001)) / 1000.0;
            v(5) = static_cast<double>(*gen::inRange(-5000, 5001)) / 1000.0;
            double norm = v.head<3>().norm();
            if (norm > 3.13)
            {
                v.head<3>() *= 3.13 / norm;
            }
            return cartan::se3<double>::exp(v);
        });
    }
};

} // namespace rc

// ============================================================================
// SO(2) Group Axioms
// ============================================================================

TEST_CASE("so2: group axioms (property-based)", "[so2][property]")
{
    SECTION("closure")
    {
        rc::prop("so2 closure: (a*b) is valid SO(2)",
            [](const cartan::so2<double>& a, const cartan::so2<double>& b)
            {
                auto c = a * b;
                auto R = c.matrix();
                auto RtR = R.transpose() * R;
                RC_ASSERT(mat_dist(RtR, cartan::matrix2<double>::Identity()) < 1e-10);
                RC_ASSERT(std::abs(R.determinant() - 1.0) < 1e-10);
            });
    }

    SECTION("associativity")
    {
        rc::prop("so2 associativity: (a*b)*c == a*(b*c)",
            [](const cartan::so2<double>& a,
               const cartan::so2<double>& b,
               const cartan::so2<double>& c)
            {
                auto lhs = (a * b) * c;
                auto rhs = a * (b * c);
                RC_ASSERT(mat_dist(lhs.matrix(), rhs.matrix()) < 1e-10);
            });
    }

    SECTION("identity")
    {
        rc::prop("so2 identity: a*I == a and I*a == a",
            [](const cartan::so2<double>& a)
            {
                auto id = cartan::so2<double>::identity();
                RC_ASSERT(mat_dist((a * id).matrix(), a.matrix()) < 1e-12);
                RC_ASSERT(mat_dist((id * a).matrix(), a.matrix()) < 1e-12);
            });
    }

    SECTION("inverse")
    {
        rc::prop("so2 inverse: a*a^{-1} == I and a^{-1}*a == I",
            [](const cartan::so2<double>& a)
            {
                auto I = cartan::matrix2<double>::Identity();
                RC_ASSERT(mat_dist((a * a.inverse()).matrix(), I) < 1e-12);
                RC_ASSERT(mat_dist((a.inverse() * a).matrix(), I) < 1e-12);
            });
    }
}

// ============================================================================
// SE(2) Group Axioms
// ============================================================================

TEST_CASE("se2: group axioms (property-based)", "[se2][property]")
{
    SECTION("closure")
    {
        rc::prop("se2 closure: (a*b) is valid SE(2)",
            [](const cartan::se2<double>& a, const cartan::se2<double>& b)
            {
                auto c = a * b;
                auto R = c.rotation().matrix();
                auto RtR = R.transpose() * R;
                RC_ASSERT(mat_dist(RtR, cartan::matrix2<double>::Identity()) < 1e-10);
            });
    }

    SECTION("associativity")
    {
        rc::prop("se2 associativity",
            [](const cartan::se2<double>& a,
               const cartan::se2<double>& b,
               const cartan::se2<double>& c)
            {
                auto lhs = (a * b) * c;
                auto rhs = a * (b * c);
                RC_ASSERT(mat_dist(lhs.matrix(), rhs.matrix()) < 1e-8);
            });
    }

    SECTION("identity")
    {
        rc::prop("se2 identity",
            [](const cartan::se2<double>& a)
            {
                auto id = cartan::se2<double>::identity();
                RC_ASSERT(mat_dist((a * id).matrix(), a.matrix()) < 1e-12);
                RC_ASSERT(mat_dist((id * a).matrix(), a.matrix()) < 1e-12);
            });
    }

    SECTION("inverse")
    {
        rc::prop("se2 inverse",
            [](const cartan::se2<double>& a)
            {
                auto I3 = Eigen::Matrix<double, 3, 3>::Identity();
                RC_ASSERT(mat_dist((a * a.inverse()).matrix(), I3) < 1e-10);
                RC_ASSERT(mat_dist((a.inverse() * a).matrix(), I3) < 1e-10);
            });
    }
}

// ============================================================================
// SO(3) Group Axioms
// ============================================================================

TEST_CASE("so3: group axioms (property-based)", "[so3][property]")
{
    SECTION("closure")
    {
        rc::prop("so3 closure: (a*b) is valid SO(3)",
            [](const cartan::so3<double>& a, const cartan::so3<double>& b)
            {
                auto c = a * b;
                auto R = c.matrix();
                auto RtR = R.transpose() * R;
                RC_ASSERT(mat_dist(RtR, cartan::matrix3<double>::Identity()) < 1e-10);
                RC_ASSERT(std::abs(R.determinant() - 1.0) < 1e-10);
            });
    }

    SECTION("associativity")
    {
        rc::prop("so3 associativity: (a*b)*c == a*(b*c)",
            [](const cartan::so3<double>& a,
               const cartan::so3<double>& b,
               const cartan::so3<double>& c)
            {
                auto lhs = (a * b) * c;
                auto rhs = a * (b * c);
                RC_ASSERT(mat_dist(lhs.matrix(), rhs.matrix()) < 1e-10);
            });
    }

    SECTION("identity")
    {
        rc::prop("so3 identity: a*I == a and I*a == a",
            [](const cartan::so3<double>& a)
            {
                auto id = cartan::so3<double>::identity();
                RC_ASSERT(mat_dist((a * id).matrix(), a.matrix()) < 1e-12);
                RC_ASSERT(mat_dist((id * a).matrix(), a.matrix()) < 1e-12);
            });
    }

    SECTION("inverse")
    {
        rc::prop("so3 inverse: a*a^{-1} == I and a^{-1}*a == I",
            [](const cartan::so3<double>& a)
            {
                auto I = cartan::matrix3<double>::Identity();
                RC_ASSERT(mat_dist((a * a.inverse()).matrix(), I) < 1e-12);
                RC_ASSERT(mat_dist((a.inverse() * a).matrix(), I) < 1e-12);
            });
    }
}

// ============================================================================
// SE(3) Group Axioms
// ============================================================================

TEST_CASE("se3: group axioms (property-based)", "[se3][property]")
{
    SECTION("closure")
    {
        rc::prop("se3 closure: (a*b) is valid SE(3)",
            [](const cartan::se3<double>& a, const cartan::se3<double>& b)
            {
                auto c = a * b;
                auto R = c.rotation().matrix();
                auto RtR = R.transpose() * R;
                RC_ASSERT(mat_dist(RtR, cartan::matrix3<double>::Identity()) < 1e-10);
                RC_ASSERT(std::abs(R.determinant() - 1.0) < 1e-10);
            });
    }

    SECTION("associativity")
    {
        rc::prop("se3 associativity",
            [](const cartan::se3<double>& a,
               const cartan::se3<double>& b,
               const cartan::se3<double>& c)
            {
                auto lhs = (a * b) * c;
                auto rhs = a * (b * c);
                RC_ASSERT(mat_dist(lhs.matrix(), rhs.matrix()) < 1e-8);
            });
    }

    SECTION("identity")
    {
        rc::prop("se3 identity",
            [](const cartan::se3<double>& a)
            {
                auto id = cartan::se3<double>::identity();
                RC_ASSERT(mat_dist((a * id).matrix(), a.matrix()) < 1e-12);
                RC_ASSERT(mat_dist((id * a).matrix(), a.matrix()) < 1e-12);
            });
    }

    SECTION("inverse")
    {
        rc::prop("se3 inverse",
            [](const cartan::se3<double>& a)
            {
                auto I = cartan::matrix4<double>::Identity();
                RC_ASSERT(mat_dist((a * a.inverse()).matrix(), I) < 1e-10);
                RC_ASSERT(mat_dist((a.inverse() * a).matrix(), I) < 1e-10);
            });
    }
}

// ============================================================================
// Exp/Log Roundtrip (all groups)
// ============================================================================

TEST_CASE("so2: exp/log roundtrip (property-based)", "[so2][property]")
{
    rc::prop("so2 exp(log(X)).matrix() ~= X.matrix()",
        [](const cartan::so2<double>& x)
        {
            auto angle = x.log();
            auto back = cartan::so2<double>::exp(angle);
            RC_ASSERT(mat_dist(back.matrix(), x.matrix()) < 1e-12);
        });
}

TEST_CASE("se2: exp/log roundtrip (property-based)", "[se2][property]")
{
    rc::prop("se2 exp(log(X)).matrix() ~= X.matrix()",
        [](const cartan::se2<double>& x)
        {
            auto v = x.log();
            auto back = cartan::se2<double>::exp(v);
            RC_ASSERT(mat_dist(back.matrix(), x.matrix()) < 1e-8);
        });
}

TEST_CASE("so3: exp/log roundtrip (property-based)", "[so3][property]")
{
    rc::prop("so3 exp(log(X)).matrix() ~= X.matrix()",
        [](const cartan::so3<double>& x)
        {
            auto phi = x.log();
            auto back = cartan::so3<double>::exp(phi);
            RC_ASSERT(mat_dist(back.matrix(), x.matrix()) < 1e-10);
        });
}

TEST_CASE("se3: exp/log roundtrip (property-based)", "[se3][property]")
{
    rc::prop("se3 exp(log(X)).matrix() ~= X.matrix()",
        [](const cartan::se3<double>& x)
        {
            auto v = x.log();
            auto back = cartan::se3<double>::exp(v);
            RC_ASSERT(mat_dist(back.matrix(), x.matrix()) < 1e-8);
        });
}

// ============================================================================
// Adjoint identity: SO(3)
// ============================================================================

TEST_CASE("so3: adjoint identity (property-based)", "[so3][property]")
{
    rc::prop("so3 adjoint: R.adjoint() * omega == vee(R * hat(omega) * R^T)",
        []
        {
            auto R = *rc::gen::arbitrary<cartan::so3<double>>();
            cartan::vector3<double> omega;
            omega << static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0,
                     static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0,
                     static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0;

            cartan::vector3<double> lhs = R.adjoint() * omega;
            cartan::matrix3<double> rhs_mat =
                R.matrix() * cartan::hat(omega) * R.matrix().transpose();
            cartan::vector3<double> rhs = cartan::vee(rhs_mat);
            RC_ASSERT((lhs - rhs).norm() < 1e-10);
        });
}

// ============================================================================
// Adjoint identity: SE(3)
// ============================================================================

TEST_CASE("se3: adjoint identity (property-based)", "[se3][property]")
{
    rc::prop("se3 adjoint: Ad_T * V == vee(T * hat(V) * T^{-1})",
        []
        {
            auto T = *rc::gen::arbitrary<cartan::se3<double>>();
            cartan::vector6<double> V;
            V << static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0,
                 static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0,
                 static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0,
                 static_cast<double>(*rc::gen::inRange(-5000, 5001)) / 1000.0,
                 static_cast<double>(*rc::gen::inRange(-5000, 5001)) / 1000.0,
                 static_cast<double>(*rc::gen::inRange(-5000, 5001)) / 1000.0;

            cartan::vector6<double> lhs = T.adjoint() * V;
            cartan::matrix4<double> V_hat = cartan::hat(V);
            cartan::matrix4<double> result =
                T.matrix() * V_hat * T.inverse().matrix();
            cartan::vector6<double> rhs = cartan::vee(result);
            RC_ASSERT((lhs - rhs).norm() < 1e-8);
        });
}

// ============================================================================
// SO(3) singularity tests
// ============================================================================

TEST_CASE("so3: exp/log roundtrip near theta~0 (property-based)", "[so3][property]")
{
    rc::prop("so3 near-zero: exp(phi).log() is finite",
        []
        {
            double x = static_cast<double>(*rc::gen::inRange(-100, 101)) * 1e-10;
            double y = static_cast<double>(*rc::gen::inRange(-100, 101)) * 1e-10;
            double z = static_cast<double>(*rc::gen::inRange(-100, 101)) * 1e-10;
            cartan::vector3<double> phi;
            phi << x, y, z;
            auto r = cartan::so3<double>::exp(phi);
            auto result = r.log();
            RC_ASSERT(std::isfinite(result(0)));
            RC_ASSERT(std::isfinite(result(1)));
            RC_ASSERT(std::isfinite(result(2)));
        });
}

TEST_CASE("so3: exp/log roundtrip near theta~pi (property-based)", "[so3][property]")
{
    rc::prop("so3 near-pi: exp(phi).matrix() roundtrips correctly",
        []
        {
            // Generate phi with |phi| near pi
            double x = static_cast<double>(*rc::gen::inRange(-1000, 1001)) / 1000.0;
            double y = static_cast<double>(*rc::gen::inRange(-1000, 1001)) / 1000.0;
            double z = static_cast<double>(*rc::gen::inRange(-1000, 1001)) / 1000.0;
            cartan::vector3<double> dir;
            dir << x, y, z;
            double n = dir.norm();
            if (n < 1e-6)
            {
                dir << 1.0, 0.0, 0.0;
                n = 1.0;
            }
            // Scale to near pi
            double theta = std::numbers::pi - static_cast<double>(*rc::gen::inRange(1, 1000)) * 1e-6;
            cartan::vector3<double> phi = (theta / n) * dir;

            auto r = cartan::so3<double>::exp(phi);
            auto result = r.log();
            RC_ASSERT(std::isfinite(result(0)));
            RC_ASSERT(std::isfinite(result(1)));
            RC_ASSERT(std::isfinite(result(2)));
            // Roundtrip via matrix comparison
            auto back = cartan::so3<double>::exp(result);
            RC_ASSERT(mat_dist(back.matrix(), r.matrix()) < 1e-8);
        });
}

// ============================================================================
// SO(3) Jacobian identities
// ============================================================================

TEST_CASE("so3: Jacobian identity J_l * J_l_inv == I (property-based)", "[so3][property]")
{
    rc::prop("so3 J_l * J_l_inv == I",
        []
        {
            double x = static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0;
            double y = static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0;
            double z = static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0;
            cartan::vector3<double> phi;
            phi << x, y, z;
            // Clamp away from pi
            double norm = phi.norm();
            if (norm > 3.13)
            {
                phi *= 3.13 / norm;
            }

            auto Jl = cartan::so3<double>::left_jacobian(phi);
            auto Jl_inv = cartan::so3<double>::left_jacobian_inv(phi);
            auto I = cartan::matrix3<double>::Identity();
            RC_ASSERT(mat_dist(Jl * Jl_inv, I) < 1e-8);
        });
}

TEST_CASE("so3: J_r(phi) == J_l(-phi) (property-based)", "[so3][property]")
{
    rc::prop("so3 J_r(phi) == J_l(-phi)",
        []
        {
            double x = static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0;
            double y = static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0;
            double z = static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0;
            cartan::vector3<double> phi;
            phi << x, y, z;
            double norm = phi.norm();
            if (norm > 3.13)
            {
                phi *= 3.13 / norm;
            }

            auto Jr = cartan::so3<double>::right_jacobian(phi);
            auto Jl_neg = cartan::so3<double>::left_jacobian(-phi);
            RC_ASSERT(mat_dist(Jr, Jl_neg) < 1e-12);
        });
}
