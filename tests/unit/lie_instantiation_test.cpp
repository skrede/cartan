// The four Lie group types at both checking policies and both scalar types:
// sixteen forms, each driven through an identity, an exponential, a composition,
// an inverse, a logarithm and an approximate comparison.
//
// The calls are the instantiation mechanism rather than a behavior check.
// Instantiating a class template instantiates its member declarations and not
// their bodies, so a slice that only named the sixteen forms would compile every
// one of them and cover almost none. The two policies are both driven because
// what separates them is the checking path a construction takes.
//
// The assertions are deliberately weak. Round-trip accuracy and the strict
// policy's rejection of malformed input are the subject of the group's own test
// files; what is asserted here is that each operation returns something readable
// -- a finite tangent, a comparison that answers.

#include <cartan/types.h>

#include <cartan/lie/se2.h>
#include <cartan/lie/se3.h>
#include <cartan/lie/so2.h>
#include <cartan/lie/so3.h>
#include <cartan/lie/policy.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <type_traits>

namespace spp = cartan;

namespace
{

/// The tangent space differs per group, so the drive carries one per group
/// rather than forcing a single spelling on all four.
template <typename Group>
struct tangent;

template <typename Scalar, typename Policy>
struct tangent<spp::so2<Scalar, Policy>>
{
    using scalar_type = Scalar;

    static Scalar sample() { return Scalar(0.3); }
};

template <typename Scalar, typename Policy>
struct tangent<spp::se2<Scalar, Policy>>
{
    using scalar_type = Scalar;

    static spp::vector3<Scalar> sample()
    {
        return spp::vector3<Scalar>(Scalar(0.1), Scalar(0.2), Scalar(0.3));
    }
};

template <typename Scalar, typename Policy>
struct tangent<spp::so3<Scalar, Policy>>
{
    using scalar_type = Scalar;

    static spp::vector3<Scalar> sample()
    {
        return spp::vector3<Scalar>(Scalar(0.2), Scalar(-0.1), Scalar(0.3));
    }
};

template <typename Scalar, typename Policy>
struct tangent<spp::se3<Scalar, Policy>>
{
    using scalar_type = Scalar;

    static spp::vector6<Scalar> sample()
    {
        spp::vector6<Scalar> v;
        v << Scalar(0.2), Scalar(-0.1), Scalar(0.3),
            Scalar(0.4), Scalar(0.5), Scalar(-0.2);
        return v;
    }
};

template <typename Scalar>
    requires std::is_floating_point_v<Scalar>
bool finite(Scalar value)
{
    return std::isfinite(value);
}

template <typename Derived>
bool finite(const Eigen::MatrixBase<Derived>& value)
{
    return value.allFinite();
}

/// The comparison is exercised reflexively and at a deliberately loose bound: an
/// element is compared against itself, which any implementation must accept, so
/// the check reports whether the comparison answers and never how accurately.
template <typename Group>
void drive_group()
{
    using scalar_type = typename tangent<Group>::scalar_type;

    const Group identity = Group::identity();
    const Group moved = Group::exp(tangent<Group>::sample());
    const Group composed = moved * identity;
    const Group inverted = moved.inverse();

    CHECK(finite(moved.log()));
    CHECK(finite(composed.log()));
    CHECK(finite(inverted.log()));
    CHECK(moved.isApprox(moved, scalar_type(1e-3)));
    CHECK(identity.isApprox(identity, scalar_type(1e-3)));
}

template <typename Scalar, typename Policy>
void drive_groups()
{
    drive_group<spp::so2<Scalar, Policy>>();
    drive_group<spp::se2<Scalar, Policy>>();
    drive_group<spp::so3<Scalar, Policy>>();
    drive_group<spp::se3<Scalar, Policy>>();
}

template <typename Scalar>
void drive_policies()
{
    drive_groups<Scalar, spp::strict_policy>();
    drive_groups<Scalar, spp::fast_policy>();
}

}

TEST_CASE("every Lie group form is driven through its operations at double",
    "[lie][instantiation]")
{
    drive_policies<double>();
}

TEST_CASE("every Lie group form is driven through its operations at float",
    "[lie][instantiation]")
{
    drive_policies<float>();
}
