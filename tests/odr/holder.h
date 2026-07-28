#ifndef HPP_GUARD_CARTAN_TESTS_ODR_HOLDER_H
#define HPP_GUARD_CARTAN_TESTS_ODR_HOLDER_H

// A build option selects what gets compiled; only a component target's own
// propagated definition may reach a translation unit. One arriving here means a
// public name can vary by build option again, which is the state this fixture
// exists to forbid, so refuse to compile rather than report agreement.
#ifdef CARTAN_BUILD_ARGMIN
#error "a backend build option reached a translation unit; only CARTAN_HAS_<BACKEND> may"
#endif

// Mirrors the registered backend list. A backend added there and forgotten here
// leaves the two units below unable to tell their configurations apart, and
// they refuse to compile rather than compare vacuously.
#ifdef CARTAN_HAS_ARGMIN
#define CARTAN_ODR_BACKEND_LINKED 1
#endif

#include <cartan/serial/ik/ik.h>

#include <cstddef>

using chain6 = cartan::kinematic_chain<double, 6>;

/// An alias template has no linkage, so a cross-unit disagreement about
/// `cartan::lm` can only surface through an entity whose own name is fixed.
/// Hence the public alias rather than an implementation, and hence no anonymous
/// namespace: both units must name this one type, not two distinct ones.
struct holder
{
    cartan::lm<chain6> solver;
};

std::size_t holder_size_native();
std::size_t holder_size_backend();
void touch(holder&);

#endif
