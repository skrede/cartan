#ifndef HPP_GUARD_CARTAN_TESTS_ODR_HOLDER_H
#define HPP_GUARD_CARTAN_TESTS_ODR_HOLDER_H

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
