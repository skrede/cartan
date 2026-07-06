#ifndef HPP_GUARD_CARTAN_TESTS_SUPPORT_ALLOC_COUNTER_H
#define HPP_GUARD_CARTAN_TESTS_SUPPORT_ALLOC_COUNTER_H

#include <new>
#include <atomic>
#include <cstddef>
#include <cstdlib>

// Translation-unit-local heap-allocation counter for the no-hot-path-alloc gate.
// Including this header in a test TU replaces the global operator new/delete (and
// the array, sized, and nothrow variants) so every heap allocation bumps an
// atomic counter. A test snapshots alloc_count(), runs a steady-state loop, and
// asserts the delta is zero, proving the loop allocated nothing on the paths that
// route through operator new.
//
// This counter is deliberately paired with Eigen's runtime no-malloc trap in the
// same TU: Eigen's aligned allocator reaches std::malloc directly and bypasses
// operator new, so the counter alone would false-pass on an Eigen allocation,
// while the Eigen trap alone is elided unless eigen_assert is redefined to throw.
// The two mechanisms together cover both allocation routes.
//
// Because operator new/delete are replaceable at most once per program, this
// header must be included in exactly one TU of any executable that links it.

namespace cartan::testing
{

inline std::atomic<std::size_t>& alloc_counter_storage() noexcept
{
    static std::atomic<std::size_t> counter{0};
    return counter;
}

inline std::size_t alloc_count() noexcept
{
    return alloc_counter_storage().load(std::memory_order_relaxed);
}

inline void reset_alloc_count() noexcept
{
    alloc_counter_storage().store(0, std::memory_order_relaxed);
}

}

inline void* cartan_counting_alloc(std::size_t size)
{
    cartan::testing::alloc_counter_storage().fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(size == 0 ? 1 : size))
    {
        return p;
    }
    throw std::bad_alloc{};
}

void* operator new(std::size_t size)
{
    return cartan_counting_alloc(size);
}
void* operator new[](std::size_t size)
{
    return cartan_counting_alloc(size);
}
void* operator new(std::size_t size, const std::nothrow_t&) noexcept
{
    cartan::testing::alloc_counter_storage().fetch_add(1, std::memory_order_relaxed);
    return std::malloc(size == 0 ? 1 : size);
}
void* operator new[](std::size_t size, const std::nothrow_t&) noexcept
{
    cartan::testing::alloc_counter_storage().fetch_add(1, std::memory_order_relaxed);
    return std::malloc(size == 0 ? 1 : size);
}

void operator delete(void* p) noexcept
{
    std::free(p);
}
void operator delete[](void* p) noexcept
{
    std::free(p);
}
void operator delete(void* p, std::size_t) noexcept
{
    std::free(p);
}
void operator delete[](void* p, std::size_t) noexcept
{
    std::free(p);
}
void operator delete(void* p, const std::nothrow_t&) noexcept
{
    std::free(p);
}
void operator delete[](void* p, const std::nothrow_t&) noexcept
{
    std::free(p);
}

#endif
