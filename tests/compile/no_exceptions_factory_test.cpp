// Compile-only proof that the validated Lie factories can be instantiated and
// called without C++ exceptions. Built twice: once with -fno-exceptions
// -fno-rtti (the embedded posture) and once with the default flags. If the
// exceptions-off build ever fails to compile, an ungated throw has crept back
// into cartan::expected::value() on a path these factories reach.

#include "cartan/expected.h"

#include "cartan/frames/rotation.h"
#include "cartan/frames/transform.h"

#include "cartan/lie/se2.h"
#include "cartan/lie/se3.h"

namespace
{

// Unconstrained frame tags for the frame-checked factories.
struct world
{
};

struct base
{
};

// Accumulate results into a volatile sink so the optimizer must keep every
// factory call: this makes each one a genuine ODR use rather than dead code.
volatile float g_sink = 0.0F;

}

int main()
{
    using cartan::matrix3;
    using cartan::matrix4;

    float acc = 0.0F;

    // se3<float>::from_matrix — a 4x4 homogeneous identity is a valid element.
    const matrix4<float> t4 = matrix4<float>::Identity();
    auto se3_result = cartan::se3<float>::from_matrix(t4);
    if (se3_result.has_value())
    {
        acc += (*se3_result).translation().sum();
    }

    // se2<float>::from_matrix — a 3x3 homogeneous identity is a valid element.
    const Eigen::Matrix<float, 3, 3> t3 = Eigen::Matrix<float, 3, 3>::Identity();
    auto se2_result = cartan::se2<float>::from_matrix(t3);
    if (se2_result.has_value())
    {
        acc += (*se2_result).translation().sum();
    }

    // rotation<world, base, float>::from_matrix — an identity SO(3) block.
    const matrix3<float> r3 = matrix3<float>::Identity();
    auto rot_result = cartan::rotation<world, base, float>::from_matrix(r3);
    if (rot_result.has_value())
    {
        // value() on a value-holding expected: exercises the accessor whose
        // throw site is gated, taken only when has_value() is already true.
        acc += rot_result.value().matrix().trace();
    }

    // transform<world, base, float>::from_matrix — the 4x4 identity again.
    auto tf_result = cartan::transform<world, base, float>::from_matrix(t4);
    if (tf_result.has_value())
    {
        acc += (*tf_result).matrix().trace();
    }

    g_sink = acc;
    return 0;
}
