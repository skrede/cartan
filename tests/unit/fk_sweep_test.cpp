#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include "../test_utils.h"

#include <liepp/types.h>
#include <liepp/lie/se3.h>
#include <liepp/lie/so3.h>
#include <liepp/chain/kinematic_chain.h>
#include <liepp/kinematics/forward_kinematics.h>

#include <numbers>

// ============================================================================
// FK sweep: DOF 1-7 x {double, float} x {fixed, dynamic}
//
// For each DOF level:
//   A. Zero config returns home pose
//   B. Non-zero config produces non-home pose
//   C. Fixed-vs-dynamic comparison
// ============================================================================

TEMPLATE_TEST_CASE("FK sweep: DOF 1-7", "[fk][sweep]", double, float)
{
    using Scalar = TestType;
    const Scalar tol = Scalar(100) * liepp::test::test_eps<Scalar>;

    // ------------------------------------------------------------------
    // DOF 1
    // ------------------------------------------------------------------
    SECTION("DOF 1")
    {
        auto chain = liepp::test::make_1r_chain<Scalar>();
        auto M = chain.home();

        SECTION("zero config returns home")
        {
            Eigen::Vector<Scalar, 1> q = Eigen::Vector<Scalar, 1>::Zero();
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar err = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(err < tol);
        }

        SECTION("non-zero config differs from home")
        {
            Eigen::Vector<Scalar, 1> q;
            q << Scalar(0.1);
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar diff = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(diff > Scalar(0.01));
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 1> q_fixed;
            q_fixed << Scalar(0.3);
            auto fk_fixed = liepp::forward_kinematics(chain, q_fixed);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(1);
            q_dyn << Scalar(0.3);
            auto fk_dyn = liepp::forward_kinematics(dyn_chain, q_dyn);

            Scalar err = (fk_fixed.end_effector.inverse() * fk_dyn.end_effector).log().norm();
            REQUIRE(err < tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 2
    // ------------------------------------------------------------------
    SECTION("DOF 2")
    {
        auto chain = liepp::test::make_2r_planar_chain<Scalar>();
        auto M = chain.home();

        SECTION("zero config returns home")
        {
            Eigen::Vector<Scalar, 2> q = Eigen::Vector<Scalar, 2>::Zero();
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar err = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(err < tol);
        }

        SECTION("non-zero config differs from home")
        {
            Eigen::Vector<Scalar, 2> q;
            q << Scalar(0.1), Scalar(0.1);
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar diff = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(diff > Scalar(0.01));
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 2> q_fixed;
            q_fixed << Scalar(0.3), Scalar(-0.2);
            auto fk_fixed = liepp::forward_kinematics(chain, q_fixed);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(2);
            q_dyn << Scalar(0.3), Scalar(-0.2);
            auto fk_dyn = liepp::forward_kinematics(dyn_chain, q_dyn);

            Scalar err = (fk_fixed.end_effector.inverse() * fk_dyn.end_effector).log().norm();
            REQUIRE(err < tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 3
    // ------------------------------------------------------------------
    SECTION("DOF 3")
    {
        auto chain = liepp::test::make_3r_planar_chain<Scalar>();
        auto M = chain.home();

        SECTION("zero config returns home")
        {
            Eigen::Vector<Scalar, 3> q = Eigen::Vector<Scalar, 3>::Zero();
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar err = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(err < tol);
        }

        SECTION("non-zero config differs from home")
        {
            Eigen::Vector<Scalar, 3> q;
            q << Scalar(0.1), Scalar(0.1), Scalar(0.1);
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar diff = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(diff > Scalar(0.01));
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 3> q_fixed;
            q_fixed << Scalar(0.3), Scalar(-0.2), Scalar(0.5);
            auto fk_fixed = liepp::forward_kinematics(chain, q_fixed);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(3);
            q_dyn << Scalar(0.3), Scalar(-0.2), Scalar(0.5);
            auto fk_dyn = liepp::forward_kinematics(dyn_chain, q_dyn);

            Scalar err = (fk_fixed.end_effector.inverse() * fk_dyn.end_effector).log().norm();
            REQUIRE(err < tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 4
    // ------------------------------------------------------------------
    SECTION("DOF 4")
    {
        auto chain = liepp::test::make_4r_spatial_chain<Scalar>();
        auto M = chain.home();

        SECTION("zero config returns home")
        {
            Eigen::Vector<Scalar, 4> q = Eigen::Vector<Scalar, 4>::Zero();
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar err = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(err < tol);
        }

        SECTION("non-zero config differs from home")
        {
            Eigen::Vector<Scalar, 4> q;
            q << Scalar(0.1), Scalar(0.1), Scalar(0.1), Scalar(0.1);
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar diff = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(diff > Scalar(0.01));
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 4> q_fixed;
            q_fixed << Scalar(0.3), Scalar(-0.2), Scalar(0.5), Scalar(-0.1);
            auto fk_fixed = liepp::forward_kinematics(chain, q_fixed);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(4);
            q_dyn << Scalar(0.3), Scalar(-0.2), Scalar(0.5), Scalar(-0.1);
            auto fk_dyn = liepp::forward_kinematics(dyn_chain, q_dyn);

            Scalar err = (fk_fixed.end_effector.inverse() * fk_dyn.end_effector).log().norm();
            REQUIRE(err < tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 5
    // ------------------------------------------------------------------
    SECTION("DOF 5")
    {
        auto chain = liepp::test::make_puma560_5dof_chain<Scalar>();
        auto M = chain.home();

        SECTION("zero config returns home")
        {
            Eigen::Vector<Scalar, 5> q = Eigen::Vector<Scalar, 5>::Zero();
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar err = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(err < tol);
        }

        SECTION("non-zero config differs from home")
        {
            Eigen::Vector<Scalar, 5> q;
            q << Scalar(0.1), Scalar(0.1), Scalar(0.1), Scalar(0.1), Scalar(0.1);
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar diff = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(diff > Scalar(0.01));
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 5> q_fixed;
            q_fixed << Scalar(0.3), Scalar(-0.2), Scalar(0.5), Scalar(-0.1), Scalar(0.4);
            auto fk_fixed = liepp::forward_kinematics(chain, q_fixed);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(5);
            q_dyn << Scalar(0.3), Scalar(-0.2), Scalar(0.5), Scalar(-0.1), Scalar(0.4);
            auto fk_dyn = liepp::forward_kinematics(dyn_chain, q_dyn);

            Scalar err = (fk_fixed.end_effector.inverse() * fk_dyn.end_effector).log().norm();
            REQUIRE(err < tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 6 (UR3e)
    // ------------------------------------------------------------------
    SECTION("DOF 6 (UR3e)")
    {
        auto chain = liepp::test::make_ur3e_chain<Scalar>();
        auto M = chain.home();

        SECTION("zero config returns home")
        {
            Eigen::Vector<Scalar, 6> q = Eigen::Vector<Scalar, 6>::Zero();
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar err = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(err < tol);
        }

        SECTION("non-zero config differs from home")
        {
            Eigen::Vector<Scalar, 6> q;
            q << Scalar(0.1), Scalar(0.1), Scalar(0.1),
                 Scalar(0.1), Scalar(0.1), Scalar(0.1);
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar diff = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(diff > Scalar(0.01));
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 6> q_fixed;
            q_fixed << Scalar(0.3), Scalar(-0.2), Scalar(0.5),
                       Scalar(-0.1), Scalar(0.4), Scalar(-0.3);
            auto fk_fixed = liepp::forward_kinematics(chain, q_fixed);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(6);
            q_dyn << Scalar(0.3), Scalar(-0.2), Scalar(0.5),
                     Scalar(-0.1), Scalar(0.4), Scalar(-0.3);
            auto fk_dyn = liepp::forward_kinematics(dyn_chain, q_dyn);

            Scalar err = (fk_fixed.end_effector.inverse() * fk_dyn.end_effector).log().norm();
            REQUIRE(err < tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 6 (KR6 SIXX)
    // ------------------------------------------------------------------
    SECTION("DOF 6 (KR6 SIXX)")
    {
        auto chain = liepp::test::make_kr6_sixx_chain<Scalar>();
        auto M = chain.home();

        SECTION("zero config returns home")
        {
            Eigen::Vector<Scalar, 6> q = Eigen::Vector<Scalar, 6>::Zero();
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar err = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(err < tol);
        }

        SECTION("non-zero config differs from home")
        {
            Eigen::Vector<Scalar, 6> q;
            q << Scalar(0.1), Scalar(0.1), Scalar(0.1),
                 Scalar(0.1), Scalar(0.1), Scalar(0.1);
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar diff = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(diff > Scalar(0.01));
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 6> q_fixed;
            q_fixed << Scalar(0.3), Scalar(-0.2), Scalar(0.5),
                       Scalar(-0.1), Scalar(0.4), Scalar(-0.3);
            auto fk_fixed = liepp::forward_kinematics(chain, q_fixed);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(6);
            q_dyn << Scalar(0.3), Scalar(-0.2), Scalar(0.5),
                     Scalar(-0.1), Scalar(0.4), Scalar(-0.3);
            auto fk_dyn = liepp::forward_kinematics(dyn_chain, q_dyn);

            Scalar err = (fk_fixed.end_effector.inverse() * fk_dyn.end_effector).log().norm();
            REQUIRE(err < tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 7
    // ------------------------------------------------------------------
    SECTION("DOF 7")
    {
        auto chain = liepp::test::make_lbr_iiwa_chain<Scalar>();
        auto M = chain.home();

        SECTION("zero config returns home")
        {
            Eigen::Vector<Scalar, 7> q = Eigen::Vector<Scalar, 7>::Zero();
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar err = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(err < tol);
        }

        SECTION("non-zero config differs from home")
        {
            Eigen::Vector<Scalar, 7> q;
            q << Scalar(0.1), Scalar(0.1), Scalar(0.1), Scalar(0.1),
                 Scalar(0.1), Scalar(0.1), Scalar(0.1);
            auto fk = liepp::forward_kinematics(chain, q);
            Scalar diff = (fk.end_effector.inverse() * M).log().norm();
            REQUIRE(diff > Scalar(0.01));
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 7> q_fixed;
            q_fixed << Scalar(0.3), Scalar(-0.2), Scalar(0.5), Scalar(-0.1),
                       Scalar(0.4), Scalar(-0.3), Scalar(0.2);
            auto fk_fixed = liepp::forward_kinematics(chain, q_fixed);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(7);
            q_dyn << Scalar(0.3), Scalar(-0.2), Scalar(0.5), Scalar(-0.1),
                     Scalar(0.4), Scalar(-0.3), Scalar(0.2);
            auto fk_dyn = liepp::forward_kinematics(dyn_chain, q_dyn);

            Scalar err = (fk_fixed.end_effector.inverse() * fk_dyn.end_effector).log().norm();
            REQUIRE(err < tol);
        }
    }
}
