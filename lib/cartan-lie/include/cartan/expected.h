#ifndef HPP_GUARD_CARTAN_EXPECTED_H
#define HPP_GUARD_CARTAN_EXPECTED_H

/// cartan::expected — a 1:1 API mirror of std::expected from C++23, provided
/// in cartan so the library compiles under C++20 toolchains (notably ESP-IDF /
/// Arduino-ESP32 / Teensy / RP2040 whose libstdc++ vintages don't yet ship
/// <expected>, and manylinux2014's GCC 10 baseline used by the Python wheels).
///
/// Modelled after the C++23 specification (P0323R12 / [expected] in N4950).
/// Method signatures, value-category overloads, exception semantics, and
/// constructor selection rules track the standard closely enough that callers
/// can migrate to std::expected by flipping a typedef once the C++23 stdlib
/// becomes universal across cartan's deployment targets.
///
/// References:
///   - P0323R12 std::expected (Sutton, Halpern, 2022)
///   - Sy Brand, "tl::expected — A single-header reference implementation"
///   - libstdc++ <expected> in GCC 12+

#include <exception>
#include <initializer_list>
#include <memory>
#include <type_traits>
#include <utility>
#include <version>

#if defined(__cpp_lib_expected) && __cpp_lib_expected >= 202211L
#  include <expected>
#  define CARTAN_HAS_STD_EXPECTED 1
#else
#  define CARTAN_HAS_STD_EXPECTED 0
#endif

namespace cartan
{

template <typename E>
class unexpected;

template <typename T, typename E>
class expected;

/// Disambiguation tag for constructing expected in the error state.
struct unexpect_t
{
    explicit unexpect_t() = default;
};

inline constexpr unexpect_t unexpect{};

/// Thrown by expected::value() when the expected holds an error.
/// Specialized for void to provide the polymorphic base class.
template <typename E>
class bad_expected_access;

template <>
class bad_expected_access<void> : public std::exception
{
protected:
    bad_expected_access() noexcept = default;
    bad_expected_access(const bad_expected_access&) = default;
    bad_expected_access(bad_expected_access&&) = default;
    bad_expected_access& operator=(const bad_expected_access&) = default;
    bad_expected_access& operator=(bad_expected_access&&) = default;
    ~bad_expected_access() override = default;

public:
    const char* what() const noexcept override
    {
        return "cartan::bad_expected_access";
    }
};

template <typename E>
class bad_expected_access : public bad_expected_access<void>
{
public:
    explicit bad_expected_access(E e)
        : m_error(std::move(e))
    {
    }

    [[nodiscard]] E& error() & noexcept { return m_error; }
    [[nodiscard]] const E& error() const& noexcept { return m_error; }
    [[nodiscard]] E&& error() && noexcept { return std::move(m_error); }
    [[nodiscard]] const E&& error() const&& noexcept { return std::move(m_error); }

private:
    E m_error;
};

namespace detail
{

/// Customization point for the exceptions-disabled fault path. When the
/// translation unit is compiled without C++ exceptions (`-fno-exceptions`,
/// the default on ESP-IDF and most bare-metal toolchains), value() on an
/// errored expected cannot throw; it instead calls the hook below. The
/// default aborts deterministically via a trap instruction. Firmware that
/// wants its own fail-stop (log, reset, blink an LED) defines
/// CARTAN_ON_BAD_EXPECTED_ACCESS() to its own handler at compile time,
/// e.g. -DCARTAN_ON_BAD_EXPECTED_ACCESS=my_fault_handler.
#ifndef CARTAN_ON_BAD_EXPECTED_ACCESS
#  define CARTAN_ON_BAD_EXPECTED_ACCESS() __builtin_trap()
#endif

[[noreturn]] inline void on_bad_expected_access() noexcept
{
    CARTAN_ON_BAD_EXPECTED_ACCESS();
    __builtin_unreachable();
}

}

/// unexpected<E> wraps an error value for tag-dispatching expected
/// construction. Mirrors std::unexpected.
template <typename E>
class unexpected
{
public:
    template <typename Err = E>
        requires (!std::is_same_v<std::remove_cvref_t<Err>, unexpected>
                  && !std::is_same_v<std::remove_cvref_t<Err>, std::in_place_t>
                  && std::is_constructible_v<E, Err>)
    constexpr explicit unexpected(Err&& e)
        : m_error(std::forward<Err>(e))
    {
    }

    template <typename... Args>
        requires std::is_constructible_v<E, Args...>
    constexpr explicit unexpected(std::in_place_t, Args&&... args)
        : m_error(std::forward<Args>(args)...)
    {
    }

    template <typename U, typename... Args>
        requires std::is_constructible_v<E, std::initializer_list<U>&, Args...>
    constexpr explicit unexpected(std::in_place_t,
                                  std::initializer_list<U> il,
                                  Args&&... args)
        : m_error(il, std::forward<Args>(args)...)
    {
    }

    constexpr unexpected(const unexpected&) = default;
    constexpr unexpected(unexpected&&) = default;
    constexpr unexpected& operator=(const unexpected&) = default;
    constexpr unexpected& operator=(unexpected&&) = default;

    [[nodiscard]] constexpr E& error() & noexcept { return m_error; }
    [[nodiscard]] constexpr const E& error() const& noexcept { return m_error; }
    [[nodiscard]] constexpr E&& error() && noexcept { return std::move(m_error); }
    [[nodiscard]] constexpr const E&& error() const&& noexcept { return std::move(m_error); }

    constexpr void swap(unexpected& other) noexcept(std::is_nothrow_swappable_v<E>)
    {
        using std::swap;
        swap(m_error, other.m_error);
    }

    template <typename E2>
    friend constexpr bool operator==(const unexpected& lhs, const unexpected<E2>& rhs)
    {
        return lhs.error() == rhs.error();
    }

#if CARTAN_HAS_STD_EXPECTED
    /// Convert to std::unexpected<G>. Explicit: cartan and std unexpected
    /// have distinct type identity even when G == E; the conversion
    /// makes that identity change visible at the call site.
    template <typename G = E>
        requires std::is_constructible_v<G, const E&>
    constexpr explicit operator std::unexpected<G>() const&
    {
        return std::unexpected<G>(m_error);
    }

    template <typename G = E>
        requires std::is_constructible_v<G, E>
    constexpr explicit operator std::unexpected<G>() &&
    {
        return std::unexpected<G>(std::move(m_error));
    }

    /// Construct from std::unexpected<G>.
    template <typename G>
        requires std::is_constructible_v<E, const G&>
    constexpr explicit unexpected(const std::unexpected<G>& other)
        : m_error(other.error())
    {
    }

    template <typename G>
        requires std::is_constructible_v<E, G>
    constexpr explicit unexpected(std::unexpected<G>&& other)
        : m_error(std::move(other).error())
    {
    }
#endif

private:
    E m_error;
};

// Deduction guide so unexpected{e} infers E.
template <typename E>
unexpected(E) -> unexpected<E>;

namespace detail
{

template <typename T>
inline constexpr bool is_expected_v = false;

template <typename T, typename E>
inline constexpr bool is_expected_v<expected<T, E>> = true;

template <typename T>
inline constexpr bool is_unexpected_v = false;

template <typename E>
inline constexpr bool is_unexpected_v<unexpected<E>> = true;

} // namespace detail

/// expected<T, E> — sum type holding either a value of T or an error of E.
/// 1:1 API mirror of std::expected from C++23.
template <typename T, typename E>
class expected
{
    static_assert(!std::is_reference_v<T>,
        "cartan::expected requires a non-reference value type T");
    static_assert(!std::is_void_v<E>,
        "cartan::expected requires a non-void error type E (use cartan::expected<T, MyError>)");
    static_assert(!std::is_reference_v<E>,
        "cartan::expected requires a non-reference error type E");

public:
    using value_type = T;
    using error_type = E;
    using unexpected_type = unexpected<E>;

    template <typename U>
    using rebind = expected<U, error_type>;

    // ----- constructors -----

    constexpr expected() noexcept(std::is_nothrow_default_constructible_v<T>)
        requires std::is_default_constructible_v<T>
        : m_has_value(true)
    {
        std::construct_at(std::addressof(m_value));
    }

    constexpr expected(const expected& other)
        : m_has_value(other.m_has_value)
    {
        if (m_has_value)
            std::construct_at(std::addressof(m_value), other.m_value);
        else
            std::construct_at(std::addressof(m_error), other.m_error);
    }

    constexpr expected(expected&& other)
        noexcept(std::is_nothrow_move_constructible_v<T>
                 && std::is_nothrow_move_constructible_v<E>)
        : m_has_value(other.m_has_value)
    {
        if (m_has_value)
            std::construct_at(std::addressof(m_value), std::move(other.m_value));
        else
            std::construct_at(std::addressof(m_error), std::move(other.m_error));
    }

    template <typename U, typename G>
        requires std::is_constructible_v<T, const U&>
                 && std::is_constructible_v<E, const G&>
                 && (!std::is_constructible_v<T, expected<U, G>&>)
                 && (!std::is_constructible_v<T, expected<U, G>>)
                 && (!std::is_constructible_v<T, const expected<U, G>&>)
                 && (!std::is_constructible_v<T, const expected<U, G>>)
    constexpr explicit(!std::is_convertible_v<const U&, T>
                       || !std::is_convertible_v<const G&, E>)
    expected(const expected<U, G>& other)
        : m_has_value(other.has_value())
    {
        if (m_has_value)
            std::construct_at(std::addressof(m_value), *other);
        else
            std::construct_at(std::addressof(m_error), other.error());
    }

    template <typename U, typename G>
        requires std::is_constructible_v<T, U>
                 && std::is_constructible_v<E, G>
                 && (!std::is_constructible_v<T, expected<U, G>&>)
                 && (!std::is_constructible_v<T, expected<U, G>>)
                 && (!std::is_constructible_v<T, const expected<U, G>&>)
                 && (!std::is_constructible_v<T, const expected<U, G>>)
    constexpr explicit(!std::is_convertible_v<U, T>
                       || !std::is_convertible_v<G, E>)
    expected(expected<U, G>&& other)
        : m_has_value(other.has_value())
    {
        if (m_has_value)
            std::construct_at(std::addressof(m_value), std::move(*other));
        else
            std::construct_at(std::addressof(m_error), std::move(other.error()));
    }

    template <typename U = T>
        requires (!std::is_same_v<std::remove_cvref_t<U>, expected>)
                 && (!std::is_same_v<std::remove_cvref_t<U>, std::in_place_t>)
                 && (!std::is_same_v<std::remove_cvref_t<U>, unexpect_t>)
                 && (!detail::is_unexpected_v<std::remove_cvref_t<U>>)
                 && std::is_constructible_v<T, U>
    constexpr explicit(!std::is_convertible_v<U, T>)
    expected(U&& v)
        : m_has_value(true)
    {
        std::construct_at(std::addressof(m_value), std::forward<U>(v));
    }

    template <typename G>
        requires std::is_constructible_v<E, const G&>
    constexpr explicit(!std::is_convertible_v<const G&, E>)
    expected(const unexpected<G>& u)
        : m_has_value(false)
    {
        std::construct_at(std::addressof(m_error), u.error());
    }

    template <typename G>
        requires std::is_constructible_v<E, G>
    constexpr explicit(!std::is_convertible_v<G, E>)
    expected(unexpected<G>&& u)
        : m_has_value(false)
    {
        std::construct_at(std::addressof(m_error), std::move(u).error());
    }

    template <typename... Args>
        requires std::is_constructible_v<T, Args...>
    constexpr explicit expected(std::in_place_t, Args&&... args)
        : m_has_value(true)
    {
        std::construct_at(std::addressof(m_value), std::forward<Args>(args)...);
    }

    template <typename U, typename... Args>
        requires std::is_constructible_v<T, std::initializer_list<U>&, Args...>
    constexpr explicit expected(std::in_place_t,
                                std::initializer_list<U> il,
                                Args&&... args)
        : m_has_value(true)
    {
        std::construct_at(std::addressof(m_value), il, std::forward<Args>(args)...);
    }

    template <typename... Args>
        requires std::is_constructible_v<E, Args...>
    constexpr explicit expected(unexpect_t, Args&&... args)
        : m_has_value(false)
    {
        std::construct_at(std::addressof(m_error), std::forward<Args>(args)...);
    }

    template <typename U, typename... Args>
        requires std::is_constructible_v<E, std::initializer_list<U>&, Args...>
    constexpr explicit expected(unexpect_t,
                                std::initializer_list<U> il,
                                Args&&... args)
        : m_has_value(false)
    {
        std::construct_at(std::addressof(m_error), il, std::forward<Args>(args)...);
    }

    constexpr ~expected()
    {
        if (m_has_value)
        {
            if constexpr (!std::is_trivially_destructible_v<T>)
                std::destroy_at(std::addressof(m_value));
        }
        else
        {
            if constexpr (!std::is_trivially_destructible_v<E>)
                std::destroy_at(std::addressof(m_error));
        }
    }

    // ----- assignment -----

    constexpr expected& operator=(const expected& other)
    {
        if (m_has_value && other.m_has_value)
            m_value = other.m_value;
        else if (m_has_value)
            reinit_as_error(other.m_error);
        else if (other.m_has_value)
            reinit_as_value(other.m_value);
        else
            m_error = other.m_error;
        return *this;
    }

    constexpr expected& operator=(expected&& other)
        noexcept(std::is_nothrow_move_assignable_v<T>
                 && std::is_nothrow_move_constructible_v<T>
                 && std::is_nothrow_move_assignable_v<E>
                 && std::is_nothrow_move_constructible_v<E>)
    {
        if (m_has_value && other.m_has_value)
            m_value = std::move(other.m_value);
        else if (m_has_value)
            reinit_as_error(std::move(other.m_error));
        else if (other.m_has_value)
            reinit_as_value(std::move(other.m_value));
        else
            m_error = std::move(other.m_error);
        return *this;
    }

    template <typename U = T>
        requires (!std::is_same_v<std::remove_cvref_t<U>, expected>)
                 && (!detail::is_unexpected_v<std::remove_cvref_t<U>>)
                 && std::is_constructible_v<T, U>
                 && std::is_assignable_v<T&, U>
    constexpr expected& operator=(U&& v)
    {
        if (m_has_value)
            m_value = std::forward<U>(v);
        else
            reinit_as_value(std::forward<U>(v));
        return *this;
    }

    template <typename G>
        requires std::is_constructible_v<E, const G&>
                 && std::is_assignable_v<E&, const G&>
    constexpr expected& operator=(const unexpected<G>& u)
    {
        if (m_has_value)
            reinit_as_error(u.error());
        else
            m_error = u.error();
        return *this;
    }

    template <typename G>
        requires std::is_constructible_v<E, G>
                 && std::is_assignable_v<E&, G>
    constexpr expected& operator=(unexpected<G>&& u)
    {
        if (m_has_value)
            reinit_as_error(std::move(u).error());
        else
            m_error = std::move(u).error();
        return *this;
    }

    template <typename... Args>
        requires std::is_nothrow_constructible_v<T, Args...>
    constexpr T& emplace(Args&&... args) noexcept
    {
        if (m_has_value)
        {
            if constexpr (!std::is_trivially_destructible_v<T>)
                std::destroy_at(std::addressof(m_value));
        }
        else
        {
            if constexpr (!std::is_trivially_destructible_v<E>)
                std::destroy_at(std::addressof(m_error));
            m_has_value = true;
        }
        std::construct_at(std::addressof(m_value), std::forward<Args>(args)...);
        return m_value;
    }

    // ----- swap -----

    constexpr void swap(expected& other)
        noexcept(std::is_nothrow_move_constructible_v<T>
                 && std::is_nothrow_swappable_v<T>
                 && std::is_nothrow_move_constructible_v<E>
                 && std::is_nothrow_swappable_v<E>)
    {
        using std::swap;
        if (m_has_value && other.m_has_value)
        {
            swap(m_value, other.m_value);
        }
        else if (!m_has_value && !other.m_has_value)
        {
            swap(m_error, other.m_error);
        }
        else if (m_has_value)
        {
            E temp = std::move(other.m_error);
            other.reinit_as_value(std::move(m_value));
            reinit_as_error(std::move(temp));
        }
        else
        {
            other.swap(*this);
        }
    }

    // ----- observers -----

    [[nodiscard]] constexpr const T* operator->() const noexcept
    {
        return std::addressof(m_value);
    }

    [[nodiscard]] constexpr T* operator->() noexcept
    {
        return std::addressof(m_value);
    }

    [[nodiscard]] constexpr const T& operator*() const& noexcept { return m_value; }
    [[nodiscard]] constexpr T& operator*() & noexcept { return m_value; }
    [[nodiscard]] constexpr const T&& operator*() const&& noexcept { return std::move(m_value); }
    [[nodiscard]] constexpr T&& operator*() && noexcept { return std::move(m_value); }

    [[nodiscard]] constexpr bool has_value() const noexcept { return m_has_value; }
    [[nodiscard]] constexpr explicit operator bool() const noexcept { return m_has_value; }

    constexpr T& value() &
    {
        if (!m_has_value)
        {
#if defined(__cpp_exceptions)
            throw bad_expected_access<E>(m_error);
#else
            detail::on_bad_expected_access();
#endif
        }
        return m_value;
    }

    constexpr const T& value() const&
    {
        if (!m_has_value)
        {
#if defined(__cpp_exceptions)
            throw bad_expected_access<E>(m_error);
#else
            detail::on_bad_expected_access();
#endif
        }
        return m_value;
    }

    constexpr T&& value() &&
    {
        if (!m_has_value)
        {
#if defined(__cpp_exceptions)
            throw bad_expected_access<E>(std::move(m_error));
#else
            detail::on_bad_expected_access();
#endif
        }
        return std::move(m_value);
    }

    constexpr const T&& value() const&&
    {
        if (!m_has_value)
        {
#if defined(__cpp_exceptions)
            throw bad_expected_access<E>(std::move(m_error));
#else
            detail::on_bad_expected_access();
#endif
        }
        return std::move(m_value);
    }

    [[nodiscard]] constexpr E& error() & noexcept { return m_error; }
    [[nodiscard]] constexpr const E& error() const& noexcept { return m_error; }
    [[nodiscard]] constexpr E&& error() && noexcept { return std::move(m_error); }
    [[nodiscard]] constexpr const E&& error() const&& noexcept { return std::move(m_error); }

    template <typename U>
    [[nodiscard]] constexpr T value_or(U&& default_value) const&
    {
        return m_has_value ? m_value : static_cast<T>(std::forward<U>(default_value));
    }

    template <typename U>
    [[nodiscard]] constexpr T value_or(U&& default_value) &&
    {
        return m_has_value ? std::move(m_value) : static_cast<T>(std::forward<U>(default_value));
    }

    template <typename G = E>
    [[nodiscard]] constexpr E error_or(G&& default_error) const&
    {
        return m_has_value ? static_cast<E>(std::forward<G>(default_error)) : m_error;
    }

    template <typename G = E>
    [[nodiscard]] constexpr E error_or(G&& default_error) &&
    {
        return m_has_value ? static_cast<E>(std::forward<G>(default_error)) : std::move(m_error);
    }

    // ----- monadic operations -----

    template <typename F>
    constexpr auto and_then(F&& f) &
    {
        using R = std::remove_cvref_t<std::invoke_result_t<F, T&>>;
        static_assert(detail::is_expected_v<R>,
            "F must return a cartan::expected");
        if (m_has_value) return std::forward<F>(f)(m_value);
        return R(unexpect, m_error);
    }

    template <typename F>
    constexpr auto and_then(F&& f) const&
    {
        using R = std::remove_cvref_t<std::invoke_result_t<F, const T&>>;
        static_assert(detail::is_expected_v<R>,
            "F must return a cartan::expected");
        if (m_has_value) return std::forward<F>(f)(m_value);
        return R(unexpect, m_error);
    }

    template <typename F>
    constexpr auto and_then(F&& f) &&
    {
        using R = std::remove_cvref_t<std::invoke_result_t<F, T&&>>;
        static_assert(detail::is_expected_v<R>,
            "F must return a cartan::expected");
        if (m_has_value) return std::forward<F>(f)(std::move(m_value));
        return R(unexpect, std::move(m_error));
    }

    template <typename F>
    constexpr auto or_else(F&& f) &
    {
        using R = std::remove_cvref_t<std::invoke_result_t<F, E&>>;
        static_assert(detail::is_expected_v<R>,
            "F must return a cartan::expected");
        if (m_has_value) return R(std::in_place, m_value);
        return std::forward<F>(f)(m_error);
    }

    template <typename F>
    constexpr auto or_else(F&& f) const&
    {
        using R = std::remove_cvref_t<std::invoke_result_t<F, const E&>>;
        static_assert(detail::is_expected_v<R>,
            "F must return a cartan::expected");
        if (m_has_value) return R(std::in_place, m_value);
        return std::forward<F>(f)(m_error);
    }

    template <typename F>
    constexpr auto or_else(F&& f) &&
    {
        using R = std::remove_cvref_t<std::invoke_result_t<F, E&&>>;
        static_assert(detail::is_expected_v<R>,
            "F must return a cartan::expected");
        if (m_has_value) return R(std::in_place, std::move(m_value));
        return std::forward<F>(f)(std::move(m_error));
    }

    template <typename F>
    constexpr auto transform(F&& f) &
    {
        using U = std::remove_cv_t<std::invoke_result_t<F, T&>>;
        if (m_has_value) return expected<U, E>(std::in_place, std::forward<F>(f)(m_value));
        return expected<U, E>(unexpect, m_error);
    }

    template <typename F>
    constexpr auto transform(F&& f) const&
    {
        using U = std::remove_cv_t<std::invoke_result_t<F, const T&>>;
        if (m_has_value) return expected<U, E>(std::in_place, std::forward<F>(f)(m_value));
        return expected<U, E>(unexpect, m_error);
    }

    template <typename F>
    constexpr auto transform(F&& f) &&
    {
        using U = std::remove_cv_t<std::invoke_result_t<F, T&&>>;
        if (m_has_value)
            return expected<U, E>(std::in_place, std::forward<F>(f)(std::move(m_value)));
        return expected<U, E>(unexpect, std::move(m_error));
    }

    template <typename F>
    constexpr auto transform_error(F&& f) &
    {
        using G = std::remove_cv_t<std::invoke_result_t<F, E&>>;
        if (m_has_value) return expected<T, G>(std::in_place, m_value);
        return expected<T, G>(unexpect, std::forward<F>(f)(m_error));
    }

    template <typename F>
    constexpr auto transform_error(F&& f) const&
    {
        using G = std::remove_cv_t<std::invoke_result_t<F, const E&>>;
        if (m_has_value) return expected<T, G>(std::in_place, m_value);
        return expected<T, G>(unexpect, std::forward<F>(f)(m_error));
    }

    template <typename F>
    constexpr auto transform_error(F&& f) &&
    {
        using G = std::remove_cv_t<std::invoke_result_t<F, E&&>>;
        if (m_has_value) return expected<T, G>(std::in_place, std::move(m_value));
        return expected<T, G>(unexpect, std::forward<F>(f)(std::move(m_error)));
    }

    // ----- comparison -----

    template <typename T2, typename E2>
    friend constexpr bool operator==(const expected& lhs, const expected<T2, E2>& rhs)
    {
        if (lhs.has_value() != rhs.has_value()) return false;
        if (lhs.has_value()) return *lhs == *rhs;
        return lhs.error() == rhs.error();
    }

    template <typename T2>
        requires (!detail::is_expected_v<T2> && !detail::is_unexpected_v<T2>)
    friend constexpr bool operator==(const expected& lhs, const T2& v)
    {
        return lhs.has_value() && *lhs == v;
    }

    template <typename E2>
    friend constexpr bool operator==(const expected& lhs, const unexpected<E2>& u)
    {
        return !lhs.has_value() && lhs.error() == u.error();
    }

#if CARTAN_HAS_STD_EXPECTED
    /// Convert to std::expected<U, G>. Explicit because cartan::expected and
    /// std::expected have distinct type identity even when (U, G) == (T, E);
    /// the cast site documents the type-boundary crossing.
    template <typename U = T, typename G = E>
        requires std::is_constructible_v<U, const T&>
                 && std::is_constructible_v<G, const E&>
    constexpr explicit operator std::expected<U, G>() const&
    {
        if (m_has_value)
            return std::expected<U, G>(std::in_place, m_value);
        return std::expected<U, G>(std::unexpect, m_error);
    }

    template <typename U = T, typename G = E>
        requires std::is_constructible_v<U, T>
                 && std::is_constructible_v<G, E>
    constexpr explicit operator std::expected<U, G>() &&
    {
        if (m_has_value)
            return std::expected<U, G>(std::in_place, std::move(m_value));
        return std::expected<U, G>(std::unexpect, std::move(m_error));
    }

    /// Construct from std::expected<U, G>.
    template <typename U, typename G>
        requires std::is_constructible_v<T, const U&>
                 && std::is_constructible_v<E, const G&>
    constexpr explicit expected(const std::expected<U, G>& other)
        : m_has_value(other.has_value())
    {
        if (m_has_value)
            std::construct_at(std::addressof(m_value), *other);
        else
            std::construct_at(std::addressof(m_error), other.error());
    }

    template <typename U, typename G>
        requires std::is_constructible_v<T, U>
                 && std::is_constructible_v<E, G>
    constexpr explicit expected(std::expected<U, G>&& other)
        : m_has_value(other.has_value())
    {
        if (m_has_value)
            std::construct_at(std::addressof(m_value), std::move(*other));
        else
            std::construct_at(std::addressof(m_error), std::move(other.error()));
    }
#endif

private:
    bool m_has_value;
    union
    {
        T m_value;
        E m_error;
    };

    template <typename Arg>
    constexpr void reinit_as_value(Arg&& arg)
    {
        if constexpr (!std::is_trivially_destructible_v<E>)
            std::destroy_at(std::addressof(m_error));
        std::construct_at(std::addressof(m_value), std::forward<Arg>(arg));
        m_has_value = true;
    }

    template <typename Arg>
    constexpr void reinit_as_error(Arg&& arg)
    {
        if constexpr (!std::is_trivially_destructible_v<T>)
            std::destroy_at(std::addressof(m_value));
        std::construct_at(std::addressof(m_error), std::forward<Arg>(arg));
        m_has_value = false;
    }
};

/// expected<void, E> — void specialization. has_value()/operator bool() and
/// error() are the only meaningful observers; value() returns void.
template <typename E>
class expected<void, E>
{
    static_assert(!std::is_void_v<E>,
        "cartan::expected requires a non-void error type E");
    static_assert(!std::is_reference_v<E>,
        "cartan::expected requires a non-reference error type E");

public:
    using value_type = void;
    using error_type = E;
    using unexpected_type = unexpected<E>;

    constexpr expected() noexcept
        : m_has_value(true)
    {
    }

    constexpr expected(const expected& other)
        : m_has_value(other.m_has_value)
    {
        if (!m_has_value)
            std::construct_at(std::addressof(m_error), other.m_error);
    }

    constexpr expected(expected&& other) noexcept(std::is_nothrow_move_constructible_v<E>)
        : m_has_value(other.m_has_value)
    {
        if (!m_has_value)
            std::construct_at(std::addressof(m_error), std::move(other.m_error));
    }

    template <typename G>
        requires std::is_constructible_v<E, const G&>
    constexpr explicit(!std::is_convertible_v<const G&, E>)
    expected(const unexpected<G>& u)
        : m_has_value(false)
    {
        std::construct_at(std::addressof(m_error), u.error());
    }

    template <typename G>
        requires std::is_constructible_v<E, G>
    constexpr explicit(!std::is_convertible_v<G, E>)
    expected(unexpected<G>&& u)
        : m_has_value(false)
    {
        std::construct_at(std::addressof(m_error), std::move(u).error());
    }

    constexpr explicit expected(std::in_place_t) noexcept
        : m_has_value(true)
    {
    }

    template <typename... Args>
        requires std::is_constructible_v<E, Args...>
    constexpr explicit expected(unexpect_t, Args&&... args)
        : m_has_value(false)
    {
        std::construct_at(std::addressof(m_error), std::forward<Args>(args)...);
    }

    constexpr ~expected()
    {
        if (!m_has_value)
        {
            if constexpr (!std::is_trivially_destructible_v<E>)
                std::destroy_at(std::addressof(m_error));
        }
    }

    constexpr expected& operator=(const expected& other)
    {
        if (m_has_value && !other.m_has_value)
            std::construct_at(std::addressof(m_error), other.m_error);
        else if (!m_has_value && other.m_has_value)
        {
            if constexpr (!std::is_trivially_destructible_v<E>)
                std::destroy_at(std::addressof(m_error));
        }
        else if (!m_has_value)
            m_error = other.m_error;
        m_has_value = other.m_has_value;
        return *this;
    }

    constexpr expected& operator=(expected&& other)
        noexcept(std::is_nothrow_move_assignable_v<E>
                 && std::is_nothrow_move_constructible_v<E>)
    {
        if (m_has_value && !other.m_has_value)
            std::construct_at(std::addressof(m_error), std::move(other.m_error));
        else if (!m_has_value && other.m_has_value)
        {
            if constexpr (!std::is_trivially_destructible_v<E>)
                std::destroy_at(std::addressof(m_error));
        }
        else if (!m_has_value)
            m_error = std::move(other.m_error);
        m_has_value = other.m_has_value;
        return *this;
    }

    constexpr void emplace() noexcept
    {
        if (!m_has_value)
        {
            if constexpr (!std::is_trivially_destructible_v<E>)
                std::destroy_at(std::addressof(m_error));
            m_has_value = true;
        }
    }

    constexpr void swap(expected& other) noexcept(std::is_nothrow_move_constructible_v<E>
                                                  && std::is_nothrow_swappable_v<E>)
    {
        using std::swap;
        if (m_has_value && other.m_has_value) return;
        if (!m_has_value && !other.m_has_value)
        {
            swap(m_error, other.m_error);
            return;
        }
        if (m_has_value)
        {
            std::construct_at(std::addressof(m_error), std::move(other.m_error));
            if constexpr (!std::is_trivially_destructible_v<E>)
                std::destroy_at(std::addressof(other.m_error));
        }
        else
        {
            other.swap(*this);
            return;
        }
        m_has_value = false;
        other.m_has_value = true;
    }

    [[nodiscard]] constexpr bool has_value() const noexcept { return m_has_value; }
    [[nodiscard]] constexpr explicit operator bool() const noexcept { return m_has_value; }

    constexpr void operator*() const noexcept {}

    constexpr void value() const&
    {
        if (!m_has_value)
        {
#if defined(__cpp_exceptions)
            throw bad_expected_access<E>(m_error);
#else
            detail::on_bad_expected_access();
#endif
        }
    }

    constexpr void value() &&
    {
        if (!m_has_value)
        {
#if defined(__cpp_exceptions)
            throw bad_expected_access<E>(std::move(m_error));
#else
            detail::on_bad_expected_access();
#endif
        }
    }

    [[nodiscard]] constexpr E& error() & noexcept { return m_error; }
    [[nodiscard]] constexpr const E& error() const& noexcept { return m_error; }
    [[nodiscard]] constexpr E&& error() && noexcept { return std::move(m_error); }
    [[nodiscard]] constexpr const E&& error() const&& noexcept { return std::move(m_error); }

    template <typename G = E>
    [[nodiscard]] constexpr E error_or(G&& default_error) const&
    {
        return m_has_value ? static_cast<E>(std::forward<G>(default_error)) : m_error;
    }

    template <typename G = E>
    [[nodiscard]] constexpr E error_or(G&& default_error) &&
    {
        return m_has_value ? static_cast<E>(std::forward<G>(default_error)) : std::move(m_error);
    }

    template <typename F>
    constexpr auto and_then(F&& f) const&
    {
        using R = std::remove_cvref_t<std::invoke_result_t<F>>;
        static_assert(detail::is_expected_v<R>,
            "F must return a cartan::expected");
        if (m_has_value) return std::forward<F>(f)();
        return R(unexpect, m_error);
    }

    template <typename F>
    constexpr auto and_then(F&& f) &&
    {
        using R = std::remove_cvref_t<std::invoke_result_t<F>>;
        static_assert(detail::is_expected_v<R>,
            "F must return a cartan::expected");
        if (m_has_value) return std::forward<F>(f)();
        return R(unexpect, std::move(m_error));
    }

    template <typename F>
    constexpr auto or_else(F&& f) const&
    {
        using R = std::remove_cvref_t<std::invoke_result_t<F, const E&>>;
        static_assert(detail::is_expected_v<R>,
            "F must return a cartan::expected");
        if (m_has_value) return R();
        return std::forward<F>(f)(m_error);
    }

    template <typename F>
    constexpr auto or_else(F&& f) &&
    {
        using R = std::remove_cvref_t<std::invoke_result_t<F, E&&>>;
        static_assert(detail::is_expected_v<R>,
            "F must return a cartan::expected");
        if (m_has_value) return R();
        return std::forward<F>(f)(std::move(m_error));
    }

    template <typename F>
    constexpr auto transform(F&& f) const&
    {
        using U = std::remove_cv_t<std::invoke_result_t<F>>;
        if (m_has_value)
        {
            if constexpr (std::is_void_v<U>)
            {
                std::forward<F>(f)();
                return expected<void, E>();
            }
            else
            {
                return expected<U, E>(std::in_place, std::forward<F>(f)());
            }
        }
        if constexpr (std::is_void_v<U>) return expected<void, E>(unexpect, m_error);
        else return expected<U, E>(unexpect, m_error);
    }

    template <typename F>
    constexpr auto transform_error(F&& f) const&
    {
        using G = std::remove_cv_t<std::invoke_result_t<F, const E&>>;
        if (m_has_value) return expected<void, G>();
        return expected<void, G>(unexpect, std::forward<F>(f)(m_error));
    }

    template <typename E2>
    friend constexpr bool operator==(const expected& lhs, const expected<void, E2>& rhs)
    {
        if (lhs.has_value() != rhs.has_value()) return false;
        return lhs.has_value() || lhs.error() == rhs.error();
    }

    template <typename E2>
    friend constexpr bool operator==(const expected& lhs, const unexpected<E2>& u)
    {
        return !lhs.has_value() && lhs.error() == u.error();
    }

#if CARTAN_HAS_STD_EXPECTED
    /// Convert to std::expected<void, G>. Explicit cast site documents the
    /// type-boundary crossing — cartan::expected<void, E> and
    /// std::expected<void, G> are distinct types even when G == E.
    template <typename G = E>
        requires std::is_constructible_v<G, const E&>
    constexpr explicit operator std::expected<void, G>() const&
    {
        if (m_has_value) return std::expected<void, G>();
        return std::expected<void, G>(std::unexpect, m_error);
    }

    template <typename G = E>
        requires std::is_constructible_v<G, E>
    constexpr explicit operator std::expected<void, G>() &&
    {
        if (m_has_value) return std::expected<void, G>();
        return std::expected<void, G>(std::unexpect, std::move(m_error));
    }

    /// Construct from std::expected<void, G>.
    template <typename G>
        requires std::is_constructible_v<E, const G&>
    constexpr explicit expected(const std::expected<void, G>& other)
        : m_has_value(other.has_value())
    {
        if (!m_has_value)
            std::construct_at(std::addressof(m_error), other.error());
    }

    template <typename G>
        requires std::is_constructible_v<E, G>
    constexpr explicit expected(std::expected<void, G>&& other)
        : m_has_value(other.has_value())
    {
        if (!m_has_value)
            std::construct_at(std::addressof(m_error), std::move(other.error()));
    }
#endif

private:
    bool m_has_value;
    union
    {
        char m_dummy;
        E m_error;
    };
};

}

#endif
