#include <liepp/lie/se2.h>
#include <liepp/lie/se3.h>
#include <liepp/lie/so2.h>
#include <liepp/lie/so3.h>
#include <liepp/lie/hat_vee.h>

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
struct Arbitrary<liepp::so2<double>>
{
    static Gen<liepp::so2<double>> arbitrary()
    {
        return gen::exec([]
        {
            double theta = static_cast<double>(*gen::inRange(-3141, 3142)) / 1000.0;
            return liepp::so2<double>::exp(theta);
        });
    }
};

template <>
struct Arbitrary<liepp::se2<double>>
{
    static Gen<liepp::se2<double>> arbitrary()
    {
        return gen::exec([]
        {
            liepp::vector3<double> v;
            v(0) = static_cast<double>(*gen::inRange(-3000, 3001)) / 1000.0;
            v(1) = static_cast<double>(*gen::inRange(-5000, 5001)) / 1000.0;
            v(2) = static_cast<double>(*gen::inRange(-5000, 5001)) / 1000.0;
            return liepp::se2<double>::exp(v);
        });
    }
};

template <>
struct Arbitrary<liepp::so3<double>>
{
    static Gen<liepp::so3<double>> arbitrary()
    {
        return gen::exec([]
        {
            double x = static_cast<double>(*gen::inRange(-3000, 3001)) / 1000.0;
            double y = static_cast<double>(*gen::inRange(-3000, 3001)) / 1000.0;
            double z = static_cast<double>(*gen::inRange(-3000, 3001)) / 1000.0;
            liepp::vector3<double> phi;
            phi << x, y, z;
            double norm = phi.norm();
            if (norm > 3.13)
            {
                phi *= 3.13 / norm;
            }
            return liepp::so3<double>::exp(phi);
        });
    }
};

template <>
struct Arbitrary<liepp::se3<double>>
{
    static Gen<liepp::se3<double>> arbitrary()
    {
        return gen::exec([]
        {
            liepp::vector6<double> v;
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
            return liepp::se3<double>::exp(v);
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
            [](const liepp::so2<double>& a, const liepp::so2<double>& b)
            {
                auto c = a * b;
                auto R = c.matrix();
                auto RtR = R.transpose() * R;
                RC_ASSERT(mat_dist(RtR, liepp::matrix2<double>::Identity()) < 1e-10);
                RC_ASSERT(std::abs(R.determinant() - 1.0) < 1e-10);
            });
    }

    SECTION("associativity")
    {
        rc::prop("so2 associativity: (a*b)*c == a*(b*c)",
            [](const liepp::so2<double>& a,
               const liepp::so2<double>& b,
               const liepp::so2<double>& c)
            {
                auto lhs = (a * b) * c;
                auto rhs = a * (b * c);
                RC_ASSERT(mat_dist(lhs.matrix(), rhs.matrix()) < 1e-10);
            });
    }

    SECTION("identity")
    {
        rc::prop("so2 identity: a*I == a and I*a == a",
            [](const liepp::so2<double>& a)
            {
                auto id = liepp::so2<double>::identity();
                RC_ASSERT(mat_dist((a * id).matrix(), a.matrix()) < 1e-12);
                RC_ASSERT(mat_dist((id * a).matrix(), a.matrix()) < 1e-12);
            });
    }

    SECTION("inverse")
    {
        rc::prop("so2 inverse: a*a^{-1} == I and a^{-1}*a == I",
            [](const liepp::so2<double>& a)
            {
                auto I = liepp::matrix2<double>::Identity();
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
            [](const liepp::se2<double>& a, const liepp::se2<double>& b)
            {
                auto c = a * b;
                auto R = c.rotation().matrix();
                auto RtR = R.transpose() * R;
                RC_ASSERT(mat_dist(RtR, liepp::matrix2<double>::Identity()) < 1e-10);
            });
    }

    SECTION("associativity")
    {
        rc::prop("se2 associativity",
            [](const liepp::se2<double>& a,
               const liepp::se2<double>& b,
               const liepp::se2<double>& c)
            {
                auto lhs = (a * b) * c;
                auto rhs = a * (b * c);
                RC_ASSERT(mat_dist(lhs.matrix(), rhs.matrix()) < 1e-8);
            });
    }

    SECTION("identity")
    {
        rc::prop("se2 identity",
            [](const liepp::se2<double>& a)
            {
                auto id = liepp::se2<double>::identity();
                RC_ASSERT(mat_dist((a * id).matrix(), a.matrix()) < 1e-12);
                RC_ASSERT(mat_dist((id * a).matrix(), a.matrix()) < 1e-12);
            });
    }

    SECTION("inverse")
    {
        rc::prop("se2 inverse",
            [](const liepp::se2<double>& a)
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
            [](const liepp::so3<double>& a, const liepp::so3<double>& b)
            {
                auto c = a * b;
                auto R = c.matrix();
                auto RtR = R.transpose() * R;
                RC_ASSERT(mat_dist(RtR, liepp::matrix3<double>::Identity()) < 1e-10);
                RC_ASSERT(std::abs(R.determinant() - 1.0) < 1e-10);
            });
    }

    SECTION("associativity")
    {
        rc::prop("so3 associativity: (a*b)*c == a*(b*c)",
            [](const liepp::so3<double>& a,
               const liepp::so3<double>& b,
               const liepp::so3<double>& c)
            {
                auto lhs = (a * b) * c;
                auto rhs = a * (b * c);
                RC_ASSERT(mat_dist(lhs.matrix(), rhs.matrix()) < 1e-10);
            });
    }

    SECTION("identity")
    {
        rc::prop("so3 identity: a*I == a and I*a == a",
            [](const liepp::so3<double>& a)
            {
                auto id = liepp::so3<double>::identity();
                RC_ASSERT(mat_dist((a * id).matrix(), a.matrix()) < 1e-12);
                RC_ASSERT(mat_dist((id * a).matrix(), a.matrix()) < 1e-12);
            });
    }

    SECTION("inverse")
    {
        rc::prop("so3 inverse: a*a^{-1} == I and a^{-1}*a == I",
            [](const liepp::so3<double>& a)
            {
                auto I = liepp::matrix3<double>::Identity();
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
            [](const liepp::se3<double>& a, const liepp::se3<double>& b)
            {
                auto c = a * b;
                auto R = c.rotation().matrix();
                auto RtR = R.transpose() * R;
                RC_ASSERT(mat_dist(RtR, liepp::matrix3<double>::Identity()) < 1e-10);
                RC_ASSERT(std::abs(R.determinant() - 1.0) < 1e-10);
            });
    }

    SECTION("associativity")
    {
        rc::prop("se3 associativity",
            [](const liepp::se3<double>& a,
               const liepp::se3<double>& b,
               const liepp::se3<double>& c)
            {
                auto lhs = (a * b) * c;
                auto rhs = a * (b * c);
                RC_ASSERT(mat_dist(lhs.matrix(), rhs.matrix()) < 1e-8);
            });
    }

    SECTION("identity")
    {
        rc::prop("se3 identity",
            [](const liepp::se3<double>& a)
            {
                auto id = liepp::se3<double>::identity();
                RC_ASSERT(mat_dist((a * id).matrix(), a.matrix()) < 1e-12);
                RC_ASSERT(mat_dist((id * a).matrix(), a.matrix()) < 1e-12);
            });
    }

    SECTION("inverse")
    {
        rc::prop("se3 inverse",
            [](const liepp::se3<double>& a)
            {
                auto I = liepp::matrix4<double>::Identity();
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
        [](const liepp::so2<double>& x)
        {
            auto angle = x.log();
            auto back = liepp::so2<double>::exp(angle);
            RC_ASSERT(mat_dist(back.matrix(), x.matrix()) < 1e-12);
        });
}

TEST_CASE("se2: exp/log roundtrip (property-based)", "[se2][property]")
{
    rc::prop("se2 exp(log(X)).matrix() ~= X.matrix()",
        [](const liepp::se2<double>& x)
        {
            auto v = x.log();
            auto back = liepp::se2<double>::exp(v);
            RC_ASSERT(mat_dist(back.matrix(), x.matrix()) < 1e-8);
        });
}

TEST_CASE("so3: exp/log roundtrip (property-based)", "[so3][property]")
{
    rc::prop("so3 exp(log(X)).matrix() ~= X.matrix()",
        [](const liepp::so3<double>& x)
        {
            auto phi = x.log();
            auto back = liepp::so3<double>::exp(phi);
            RC_ASSERT(mat_dist(back.matrix(), x.matrix()) < 1e-10);
        });
}

TEST_CASE("se3: exp/log roundtrip (property-based)", "[se3][property]")
{
    rc::prop("se3 exp(log(X)).matrix() ~= X.matrix()",
        [](const liepp::se3<double>& x)
        {
            auto v = x.log();
            auto back = liepp::se3<double>::exp(v);
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
            auto R = *rc::gen::arbitrary<liepp::so3<double>>();
            liepp::vector3<double> omega;
            omega << static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0,
                     static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0,
                     static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0;

            liepp::vector3<double> lhs = R.adjoint() * omega;
            liepp::matrix3<double> rhs_mat =
                R.matrix() * liepp::hat(omega) * R.matrix().transpose();
            liepp::vector3<double> rhs = liepp::vee(rhs_mat);
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
            auto T = *rc::gen::arbitrary<liepp::se3<double>>();
            liepp::vector6<double> V;
            V << static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0,
                 static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0,
                 static_cast<double>(*rc::gen::inRange(-3000, 3001)) / 1000.0,
                 static_cast<double>(*rc::gen::inRange(-5000, 5001)) / 1000.0,
                 static_cast<double>(*rc::gen::inRange(-5000, 5001)) / 1000.0,
                 static_cast<double>(*rc::gen::inRange(-5000, 5001)) / 1000.0;

            liepp::vector6<double> lhs = T.adjoint() * V;
            liepp::matrix4<double> V_hat = liepp::hat(V);
            liepp::matrix4<double> result =
                T.matrix() * V_hat * T.inverse().matrix();
            liepp::vector6<double> rhs = liepp::vee(result);
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
            liepp::vector3<double> phi;
            phi << x, y, z;
            auto r = liepp::so3<double>::exp(phi);
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
            liepp::vector3<double> dir;
            dir << x, y, z;
            double n = dir.norm();
            if (n < 1e-6)
            {
                dir << 1.0, 0.0, 0.0;
                n = 1.0;
            }
            // Scale to near pi
            double theta = std::numbers::pi - static_cast<double>(*rc::gen::inRange(1, 1000)) * 1e-6;
            liepp::vector3<double> phi = (theta / n) * dir;

            auto r = liepp::so3<double>::exp(phi);
            auto result = r.log();
            RC_ASSERT(std::isfinite(result(0)));
            RC_ASSERT(std::isfinite(result(1)));
            RC_ASSERT(std::isfinite(result(2)));
            // Roundtrip via matrix comparison
            auto back = liepp::so3<double>::exp(result);
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
            liepp::vector3<double> phi;
            phi << x, y, z;
            // Clamp away from pi
            double norm = phi.norm();
            if (norm > 3.13)
            {
                phi *= 3.13 / norm;
            }

            auto Jl = liepp::so3<double>::left_jacobian(phi);
            auto Jl_inv = liepp::so3<double>::left_jacobian_inv(phi);
            auto I = liepp::matrix3<double>::Identity();
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
            liepp::vector3<double> phi;
            phi << x, y, z;
            double norm = phi.norm();
            if (norm > 3.13)
            {
                phi *= 3.13 / norm;
            }

            auto Jr = liepp::so3<double>::right_jacobian(phi);
            auto Jl_neg = liepp::so3<double>::left_jacobian(-phi);
            RC_ASSERT(mat_dist(Jr, Jl_neg) < 1e-12);
        });
}
