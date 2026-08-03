#include <cartan/serial/fk/singularity_analysis.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>

namespace spp = cartan;

namespace
{

using vec2 = Eigen::Vector2d;

constexpr double k_eps = std::numeric_limits<double>::epsilon();

}

// A structurally rank-deficient Jacobian has an exactly zero smallest singular
// value in exact arithmetic, but the decomposition of it lands on a clean zero
// on one instruction set and on rounding-level noise on another. Both are the
// same configuration and both must be called singular, which is what the floor
// buys. Written against a synthetic spectrum rather than a decomposition on
// purpose: routing through JacobiSVD would test whichever of the two outcomes
// this machine happens to produce, which is the platform-specific answer that
// made the original assertion fail on AppleClang and MSVC while passing here.
TEST_CASE("a smallest singular value below the resolution floor reads as singular",
    "[ik][diagnostics][resolution_floor]")
{
    const vec2 noise(1.0, k_eps * 0.5);

    auto kappa = spp::condition_number(noise);
    REQUIRE(kappa);
    CHECK(std::isinf(*kappa));

    auto index = spp::isotropy(noise);
    REQUIRE(index);
    CHECK(*index == 0.0);
}

// The floor must not swallow a spectrum that is merely ill-conditioned: a
// condition number of 1e8 is a measurement and has to survive as one.
TEST_CASE("a smallest singular value above the resolution floor keeps its ratio",
    "[ik][diagnostics][resolution_floor]")
{
    const vec2 conditioned(1.0, 1e-8);

    auto kappa = spp::condition_number(conditioned);
    REQUIRE(kappa);
    CHECK_FALSE(std::isinf(*kappa));
    CHECK(*kappa == 1e8);

    auto index = spp::isotropy(conditioned);
    REQUIRE(index);
    CHECK(*index == 1e-8);
}

// The two measures are documented as inverses of one another, so they cannot
// disagree about which side of the floor a spectrum falls on -- a caller
// branching on one and reporting the other would otherwise contradict itself
// exactly at a singularity.
TEST_CASE("the condition number and the isotropy index classify the same spectra",
    "[ik][diagnostics][resolution_floor]")
{
    for (double smallest : {k_eps * 0.1, k_eps * 0.5, k_eps * 10.0, 1e-6, 0.5, 1.0})
    {
        const vec2 sigma(1.0, smallest);

        auto kappa = spp::condition_number(sigma);
        auto index = spp::isotropy(sigma);
        REQUIRE(kappa);
        REQUIRE(index);

        INFO("smallest singular value " << smallest);
        CHECK(std::isinf(*kappa) == (*index == 0.0));
    }
}

// The floor is relative to the largest singular value, so the verdict follows
// the spectrum's shape and not its units. An absolute threshold would call a
// well-conditioned Jacobian in millimetres singular and a genuinely singular one
// in kilometres healthy.
TEST_CASE("the singular verdict is unchanged by scaling the whole spectrum",
    "[ik][diagnostics][resolution_floor]")
{
    for (double scale : {1e-9, 1e-3, 1.0, 1e3, 1e9})
    {
        const vec2 singular = vec2(1.0, k_eps * 0.5) * scale;
        const vec2 conditioned = vec2(1.0, 1e-8) * scale;

        INFO("scale " << scale);

        auto singular_kappa = spp::condition_number(singular);
        REQUIRE(singular_kappa);
        CHECK(std::isinf(*singular_kappa));

        auto conditioned_kappa = spp::condition_number(conditioned);
        REQUIRE(conditioned_kappa);
        CHECK_FALSE(std::isinf(*conditioned_kappa));
    }
}
