/// Cartan ESP-IDF compile-only smoke. Instantiates a representative slice of
/// the public surface — Lie groups, a small kinematic chain, FK, and a closed-
/// form analytical subproblem — using `float` scalars so the templates are
/// instantiated against the embedded-typical floating-point type. There is no
/// printout, no flash assertion, no hardware interaction; the smoke passes if
/// `idf.py build` reaches the link step.

#include "cartan/lie/se3.h"
#include "cartan/lie/so3.h"
#include "cartan/serial/chain/kinematic_chain.h"
#include "cartan/serial/chain/screw_axis.h"
#include "cartan/serial/fk/forward_kinematics.h"
#include "cartan/analytical/paden_kahan.h"

#include "esp_log.h"

namespace
{

constexpr const char* TAG = "cartan_smoke";

[[nodiscard]] cartan::kinematic_chain<float, 3> build_planar_3r()
{
    using vec3f = cartan::vector3<float>;
    using screw = cartan::screw_axis<float>;

    auto s1 = screw::revolute(vec3f(0.f, 0.f, 1.f), vec3f(0.f, 0.f, 0.f));
    auto s2 = screw::revolute(vec3f(0.f, 0.f, 1.f), vec3f(1.f, 0.f, 0.f));
    auto s3 = screw::revolute(vec3f(0.f, 0.f, 1.f), vec3f(2.f, 0.f, 0.f));

    using rot = cartan::so3<float>;
    using pose = cartan::se3<float>;
    pose home(rot::identity(), vec3f(3.f, 0.f, 0.f));

    using limits = cartan::joint_limits<float>;
    return cartan::kinematic_chain<float, 3>{
        home,
        {s1, s2, s3},
        {limits{-3.14f, 3.14f}, limits{-3.14f, 3.14f}, limits{-3.14f, 3.14f}}};
}

} // namespace

extern "C" void app_main()
{
    auto chain = build_planar_3r();

    Eigen::Matrix<float, 3, 1> q;
    q << 0.1f, 0.2f, 0.3f;

    auto fk = cartan::forward_kinematics(chain, q);
    const float x = fk.end_effector.translation().x();

    // Side-effect that the optimizer cannot prove dead, ensuring the FK call
    // survives link-time DCE on size-optimized embedded builds.
    ESP_LOGI(TAG, "cartan FK end-effector x = %.4f", static_cast<double>(x));

    // Paden-Kahan subproblem 1: invoke a closed-form analytical free function
    // so cartan-analytical's template instantiations are exercised too.
    cartan::vector3<float> w(0.f, 0.f, 1.f);
    cartan::vector3<float> p(1.f, 0.f, 0.f);
    cartan::vector3<float> q_target(0.f, 1.f, 0.f);
    auto pk1 = cartan::paden_kahan_1<float>(w, p, q_target);
    if (pk1.has_value())
    {
        ESP_LOGI(TAG, "paden_kahan_1 theta = %.4f",
            static_cast<double>(pk1.value()));
    }
}
