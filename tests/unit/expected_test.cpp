#include <cartan/expected.h>

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <string>
#include <utility>

namespace
{

struct counted_dtor
{
    int* counter;
    explicit counted_dtor(int* c) : counter(c) {}
    counted_dtor(const counted_dtor& o) : counter(o.counter) {}
    counted_dtor(counted_dtor&& o) noexcept : counter(o.counter) { o.counter = nullptr; }
    counted_dtor& operator=(const counted_dtor& o) { counter = o.counter; return *this; }
    counted_dtor& operator=(counted_dtor&& o) noexcept { counter = o.counter; o.counter = nullptr; return *this; }
    ~counted_dtor() { if (counter) ++*counter; }
};

}

TEST_CASE("expected: default constructs a value", "[expected]")
{
    cartan::expected<int, std::string> e;
    REQUIRE(e.has_value());
    REQUIRE(static_cast<bool>(e));
    REQUIRE(*e == 0);
}

TEST_CASE("expected: constructs from a value", "[expected]")
{
    cartan::expected<int, std::string> e(42);
    REQUIRE(e.has_value());
    REQUIRE(*e == 42);
    REQUIRE(e.value() == 42);
}

TEST_CASE("expected: constructs from unexpected", "[expected]")
{
    cartan::expected<int, std::string> e(cartan::unexpected<std::string>("bad"));
    REQUIRE_FALSE(e.has_value());
    REQUIRE_FALSE(static_cast<bool>(e));
    REQUIRE(e.error() == "bad");
}

TEST_CASE("expected: unexpect_t tag constructs in error state", "[expected]")
{
    cartan::expected<int, std::string> e(cartan::unexpect, "fail");
    REQUIRE_FALSE(e.has_value());
    REQUIRE(e.error() == "fail");
}

TEST_CASE("expected: in_place_t tag forwards constructor args", "[expected]")
{
    cartan::expected<std::string, int> e(std::in_place, 5, 'x');
    REQUIRE(e.has_value());
    REQUIRE(*e == "xxxxx");
}

TEST_CASE("expected: value() throws bad_expected_access on error", "[expected]")
{
    cartan::expected<int, std::string> e(cartan::unexpect, "boom");
    bool caught = false;
    try
    {
        (void)e.value();
    }
    catch (const cartan::bad_expected_access<std::string>& ex)
    {
        caught = true;
        REQUIRE(ex.error() == "boom");
    }
    REQUIRE(caught);
}

TEST_CASE("expected: value_or returns default on error", "[expected]")
{
    cartan::expected<int, std::string> ok(42);
    cartan::expected<int, std::string> err(cartan::unexpect, "x");
    REQUIRE(ok.value_or(-1) == 42);
    REQUIRE(err.value_or(-1) == -1);
}

TEST_CASE("expected: error_or returns default on success", "[expected]")
{
    cartan::expected<int, std::string> ok(42);
    cartan::expected<int, std::string> err(cartan::unexpect, "boom");
    REQUIRE(ok.error_or("no-error") == "no-error");
    REQUIRE(err.error_or("no-error") == "boom");
}

TEST_CASE("expected: copy and move preserve state", "[expected]")
{
    cartan::expected<int, std::string> ok(42);
    auto copy = ok;
    REQUIRE(copy.has_value());
    REQUIRE(*copy == 42);

    cartan::expected<int, std::string> err(cartan::unexpect, "boom");
    auto moved = std::move(err);
    REQUIRE_FALSE(moved.has_value());
    REQUIRE(moved.error() == "boom");
}

TEST_CASE("expected: assignment switches between value and error", "[expected]")
{
    cartan::expected<int, std::string> e(42);
    e = cartan::unexpected<std::string>("fail");
    REQUIRE_FALSE(e.has_value());
    REQUIRE(e.error() == "fail");

    e = 99;
    REQUIRE(e.has_value());
    REQUIRE(*e == 99);
}

TEST_CASE("expected: destructors fire on the active alternative", "[expected]")
{
    int counter = 0;
    {
        cartan::expected<counted_dtor, int> e(std::in_place, &counter);
        REQUIRE(e.has_value());
    }
    REQUIRE(counter == 1);

    int err_counter = 0;
    {
        cartan::expected<int, counted_dtor> e(cartan::unexpect, &err_counter);
        REQUIRE_FALSE(e.has_value());
    }
    REQUIRE(err_counter == 1);
}

TEST_CASE("expected: and_then chains value transformations", "[expected]")
{
    auto doubled = [](int x) -> cartan::expected<int, std::string> {
        return x * 2;
    };
    cartan::expected<int, std::string> e(21);
    auto r = e.and_then(doubled);
    REQUIRE(r.has_value());
    REQUIRE(*r == 42);

    cartan::expected<int, std::string> err(cartan::unexpect, "boom");
    auto r2 = err.and_then(doubled);
    REQUIRE_FALSE(r2.has_value());
    REQUIRE(r2.error() == "boom");
}

TEST_CASE("expected: transform applies function to value, propagates error", "[expected]")
{
    cartan::expected<int, std::string> e(7);
    auto r = e.transform([](int x) { return x + 1; });
    REQUIRE(r.has_value());
    REQUIRE(*r == 8);

    cartan::expected<int, std::string> err(cartan::unexpect, "x");
    auto r2 = err.transform([](int x) { return x + 1; });
    REQUIRE_FALSE(r2.has_value());
    REQUIRE(r2.error() == "x");
}

TEST_CASE("expected: transform_error maps the error type", "[expected]")
{
    cartan::expected<int, int> err(cartan::unexpect, 42);
    auto r = err.transform_error([](int e) { return std::to_string(e); });
    REQUIRE_FALSE(r.has_value());
    REQUIRE(r.error() == "42");
}

TEST_CASE("expected: or_else recovers from error", "[expected]")
{
    auto recover = [](const std::string&) -> cartan::expected<int, std::string> {
        return 99;
    };
    cartan::expected<int, std::string> err(cartan::unexpect, "boom");
    auto r = err.or_else(recover);
    REQUIRE(r.has_value());
    REQUIRE(*r == 99);
}

TEST_CASE("expected<void, E>: default constructs in success state", "[expected]")
{
    cartan::expected<void, std::string> e;
    REQUIRE(e.has_value());
    REQUIRE(static_cast<bool>(e));
}

TEST_CASE("expected<void, E>: unexpect_t holds error", "[expected]")
{
    cartan::expected<void, std::string> e(cartan::unexpect, "fail");
    REQUIRE_FALSE(e.has_value());
    REQUIRE(e.error() == "fail");
}

TEST_CASE("expected<void, E>: value() throws on error", "[expected]")
{
    cartan::expected<void, std::string> e(cartan::unexpect, "boom");
    bool caught = false;
    try { e.value(); }
    catch (const cartan::bad_expected_access<std::string>&) { caught = true; }
    REQUIRE(caught);
}

TEST_CASE("expected: works with move-only types", "[expected]")
{
    cartan::expected<std::unique_ptr<int>, std::string> e(std::make_unique<int>(7));
    REQUIRE(e.has_value());
    REQUIRE(**e == 7);

    cartan::expected<std::unique_ptr<int>, std::string> e2 = std::move(e);
    REQUIRE(e2.has_value());
    REQUIRE(**e2 == 7);
}

TEST_CASE("expected: equality compares value and error states", "[expected]")
{
    cartan::expected<int, std::string> a(42);
    cartan::expected<int, std::string> b(42);
    cartan::expected<int, std::string> c(43);
    cartan::expected<int, std::string> err(cartan::unexpect, "boom");

    REQUIRE(a == b);
    REQUIRE_FALSE(a == c);
    REQUIRE_FALSE(a == err);
    REQUIRE(a == 42);
    REQUIRE(err == cartan::unexpected<std::string>("boom"));
}

TEST_CASE("expected: emplace replaces current alternative with new value", "[expected]")
{
    cartan::expected<int, std::string> err(cartan::unexpect, "x");
    REQUIRE_FALSE(err.has_value());
    err.emplace(42);
    REQUIRE(err.has_value());
    REQUIRE(*err == 42);
}
