#include "../fixtures/chain_factories.h"

#include <cartan/serial/ik/solver/projected_lm.h>

#include <cartan/types.h>
#include <cartan/lie/se3.h>
#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <catch2/catch_test_macros.hpp>

#include <bit>
#include <ios>
#include <vector>
#include <random>
#include <string>
#include <cstdint>
#include <fstream>
#include <sstream>

// ============================================================================
// Bitwise golden gate for projected_lm.
//
// This test captures the exact solver outputs (final joint vector, error norm,
// iteration count, terminal status) for a fixed panel of canonical robot arms
// and FK-walked targets seeded at 42, for both float and double. The values
// are recorded bit-exactly (via std::bit_cast to the integer of matching width)
// so that any perturbation of the numeric result -- down to the last mantissa
// bit -- is caught. There is deliberately no tolerance: the golden is an exact
// record, not an approximate one.
//
// The golden record is generated once against a reference build (the hidden
// "[.capture]" case writes projected_lm_golden_data.h next to this source), and
// the always-run comparison case then re-derives the same outputs and asserts
// bitwise equality against the committed record. A storage-only refactor of the
// solver's internal temporaries must reproduce this record unchanged.
// ============================================================================

namespace
{

// The canonical panel is intentionally scalar-agnostic: each chain is exercised
// for both float and double, and every (chain, target, scalar) case contributes
// its flattened output to the golden blob in a fixed, reproducible order.
constexpr int panel_targets = 8;
constexpr unsigned int panel_seed = 42;

struct golden_blob
{
    std::vector<double> double_scalars;
    std::vector<int> double_ints;
    std::vector<float> float_scalars;
    std::vector<int> float_ints;
};

template <typename Scalar, int N>
void run_chain(
    const cartan::kinematic_chain<Scalar, N>& chain,
    std::vector<Scalar>& scalars_out,
    std::vector<int>& ints_out)
{
    std::mt19937 rng(panel_seed);

    cartan::convergence_criteria<Scalar> criteria;
    criteria.max_iterations_per_attempt = 200;
    criteria.max_total_work_units = 100000;

    const int n = chain.num_joints();

    for (int t = 0; t < panel_targets; ++t)
    {
        auto target = cartan::fixtures::random_reachable_target(chain, rng);

        cartan::projected_lm<cartan::kinematic_chain<Scalar, N>> stepper;
        typename cartan::joint_state<Scalar, N>::position_type q0 =
            cartan::joint_state<Scalar, N>::position_type::Zero();

        stepper.setup(chain, target, q0, criteria);

        cartan::ik_status status = cartan::ik_status::running;
        for (int s = 0; s < 6000 && status == cartan::ik_status::running; ++s)
        {
            status = stepper.step(chain, 1).status;
        }

        const auto& sol = stepper.solution();
        for (int i = 0; i < n; ++i)
        {
            scalars_out.push_back(sol(i));
        }
        scalars_out.push_back(stepper.error_norm());
        ints_out.push_back(stepper.iterations());
        ints_out.push_back(static_cast<int>(stepper.status()));
    }
}

golden_blob build_blob()
{
    golden_blob blob;

    run_chain(cartan::fixtures::make_ur3e_chain<double>(), blob.double_scalars, blob.double_ints);
    run_chain(cartan::fixtures::make_lbr_med14_chain<double>(), blob.double_scalars, blob.double_ints);
    run_chain(cartan::fixtures::make_kr6_sixx_chain<double>(), blob.double_scalars, blob.double_ints);
    run_chain(cartan::fixtures::make_panda_chain<double>(), blob.double_scalars, blob.double_ints);
    run_chain(cartan::fixtures::make_3r_planar_chain<double>(), blob.double_scalars, blob.double_ints);

    run_chain(cartan::fixtures::make_ur3e_chain<float>(), blob.float_scalars, blob.float_ints);
    run_chain(cartan::fixtures::make_lbr_med14_chain<float>(), blob.float_scalars, blob.float_ints);
    run_chain(cartan::fixtures::make_kr6_sixx_chain<float>(), blob.float_scalars, blob.float_ints);
    run_chain(cartan::fixtures::make_panda_chain<float>(), blob.float_scalars, blob.float_ints);
    run_chain(cartan::fixtures::make_3r_planar_chain<float>(), blob.float_scalars, blob.float_ints);

    return blob;
}

}

#if __has_include("projected_lm_golden_data.h")
#include "projected_lm_golden_data.h"
#define CARTAN_PLM_HAVE_GOLDEN 1
#else
#define CARTAN_PLM_HAVE_GOLDEN 0
#endif

#if CARTAN_PLM_HAVE_GOLDEN

TEST_CASE("projected_lm outputs are bitwise identical to the golden record", "[ik][projected_lm][golden]")
{
    const golden_blob blob = build_blob();

    // Floating-point results are not bit-portable across compilers, so the
    // bitwise golden gate is only valid on the build that captured the record.
    // The solver still runs everywhere via build_blob above; the exact
    // comparison runs on the reference build (Linux, GCC) and is skipped
    // elsewhere.
#if !(defined(__linux__) && defined(__GNUC__) && !defined(__clang__))
    SKIP("bitwise golden gate runs on the reference build only (Linux/GCC)");
#endif

    REQUIRE(blob.double_scalars.size() == cartan_plm_golden::double_scalar_count);
    REQUIRE(blob.double_ints.size() == cartan_plm_golden::double_int_count);
    REQUIRE(blob.float_scalars.size() == cartan_plm_golden::float_scalar_count);
    REQUIRE(blob.float_ints.size() == cartan_plm_golden::float_int_count);

    for (std::size_t i = 0; i < blob.double_scalars.size(); ++i)
    {
        const auto bits = std::bit_cast<std::uint64_t>(blob.double_scalars[i]);
        REQUIRE(bits == cartan_plm_golden::double_scalar_bits[i]);
    }
    for (std::size_t i = 0; i < blob.double_ints.size(); ++i)
    {
        REQUIRE(blob.double_ints[i] == cartan_plm_golden::double_ints[i]);
    }
    for (std::size_t i = 0; i < blob.float_scalars.size(); ++i)
    {
        const auto bits = std::bit_cast<std::uint32_t>(blob.float_scalars[i]);
        REQUIRE(bits == cartan_plm_golden::float_scalar_bits[i]);
    }
    for (std::size_t i = 0; i < blob.float_ints.size(); ++i)
    {
        REQUIRE(blob.float_ints[i] == cartan_plm_golden::float_ints[i]);
    }
}

#endif

// Hidden generator: writes the committed bitwise golden record next to this
// source file. Run explicitly with the "[.capture]" tag against a reference
// build, then rebuild so the always-run comparison case links the record.
TEST_CASE("capture projected_lm bitwise golden record", "[.capture]")
{
#ifndef CARTAN_PLM_GOLDEN_DIR
    FAIL("CARTAN_PLM_GOLDEN_DIR not defined; cannot write golden record");
#else
    const golden_blob blob = build_blob();

    std::ostringstream out;
    out << std::hex << std::showbase;

    auto emit_u64 = [&](const std::vector<double>& v)
    {
        for (std::size_t i = 0; i < v.size(); ++i)
        {
            if (i != 0 && i % 4 == 0) { out << "\n    "; }
            out << std::bit_cast<std::uint64_t>(v[i]) << "ull, ";
        }
    };
    auto emit_u32 = [&](const std::vector<float>& v)
    {
        for (std::size_t i = 0; i < v.size(); ++i)
        {
            if (i != 0 && i % 6 == 0) { out << "\n    "; }
            out << std::bit_cast<std::uint32_t>(v[i]) << "u, ";
        }
    };
    auto emit_int = [&](const std::vector<int>& v)
    {
        out << std::dec;
        for (std::size_t i = 0; i < v.size(); ++i)
        {
            if (i != 0 && i % 12 == 0) { out << "\n    "; }
            out << v[i] << ", ";
        }
        out << std::hex;
    };

    std::ostringstream body;
    body << "#ifndef HPP_GUARD_CARTAN_TESTS_UNIT_PROJECTED_LM_GOLDEN_DATA_H\n";
    body << "#define HPP_GUARD_CARTAN_TESTS_UNIT_PROJECTED_LM_GOLDEN_DATA_H\n\n";
    body << "// Auto-generated bitwise golden record for the projected_lm solver.\n";
    body << "// Regenerate with the \"[.capture]\" test case. Do not edit by hand.\n\n";
    body << "#include <cstddef>\n#include <cstdint>\n\n";
    body << "namespace cartan_plm_golden\n{\n\n";

    body << "inline constexpr std::uint64_t double_scalar_bits[] = {\n    ";
    out.str("");
    emit_u64(blob.double_scalars);
    body << out.str() << "\n};\n\n";
    out.str("");

    body << "inline constexpr int double_ints[] = {\n    ";
    emit_int(blob.double_ints);
    body << out.str() << "\n};\n\n";
    out.str("");

    body << "inline constexpr std::uint32_t float_scalar_bits[] = {\n    ";
    emit_u32(blob.float_scalars);
    body << out.str() << "\n};\n\n";
    out.str("");

    body << "inline constexpr int float_ints[] = {\n    ";
    emit_int(blob.float_ints);
    body << out.str() << "\n};\n\n";

    body << std::dec;
    body << "inline constexpr std::size_t double_scalar_count = "
         << blob.double_scalars.size() << ";\n";
    body << "inline constexpr std::size_t double_int_count = "
         << blob.double_ints.size() << ";\n";
    body << "inline constexpr std::size_t float_scalar_count = "
         << blob.float_scalars.size() << ";\n";
    body << "inline constexpr std::size_t float_int_count = "
         << blob.float_ints.size() << ";\n\n";
    body << "}\n\n";
    body << "#endif\n";

    const std::string path = std::string(CARTAN_PLM_GOLDEN_DIR) + "/projected_lm_golden_data.h";
    std::ofstream file(path);
    REQUIRE(file.good());
    file << body.str();
    file.close();
    REQUIRE(file.good());
#endif
}
