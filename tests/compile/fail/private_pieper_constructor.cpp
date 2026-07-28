// The Pieper constructor is private so that every chain reaching a solver has
// passed the shoulder-intersection and wrist-sphericity gates; naming it
// directly must not compile, or the gates are advisory.
#include <cartan/analytical.h>
#include <cartan/serial_chain.h>

#include <array>

int main()
{
    using scalar = double;
    using vec3 = cartan::vector3<scalar>;
    using chain_type = cartan::static_chain<scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_z>;

    const vec3 wrist(0.7, 0.0, 0.5);
    const vec3 ee(0.8, 0.0, 0.5);
    const std::array<cartan::screw_axis<scalar>, 6> axes = {
        cartan::screw_axis<scalar>::revolute(vec3(0, 0, 1), vec3(0, 0, 0)),
        cartan::screw_axis<scalar>::revolute(vec3(0, 1, 0), vec3(0, 0, 0.5)),
        cartan::screw_axis<scalar>::revolute(vec3(0, 1, 0), vec3(0.4, 0, 0.5)),
        cartan::screw_axis<scalar>::revolute(vec3(0, 0, 1), wrist),
        cartan::screw_axis<scalar>::revolute(vec3(0, 1, 0), wrist),
        cartan::screw_axis<scalar>::revolute(vec3(0, 0, 1), wrist)};

    const auto limit = *cartan::joint_limits<scalar>::make(-3.15, 3.15);
    const std::array<cartan::joint_limits<scalar>, 6> limits = {
        limit, limit, limit, limit, limit, limit};

    const auto home = cartan::se3<scalar>(cartan::so3<scalar>::identity(), ee);
    const auto chain = *chain_type::make(home, axes, limits);

    cartan::pieper_6r_solver<chain_type> solver(
        chain, cartan::default_verification_tolerance_v<scalar>, wrist);
    return solver.solve(home).has_value() ? 0 : 1;
}
