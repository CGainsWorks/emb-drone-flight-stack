// Minimal self-contained unit-test framework.
//
// Deliberately dependency-free: the host test suite must build and run on a
// clean machine with nothing but a C++ compiler. No network fetch, no submodule.
#pragma once

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace testing {

struct TestCase {
    const char* name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}
inline int& failure_count() {
    static int f = 0;
    return f;
}
inline int& check_count() {
    static int c = 0;
    return c;
}

struct Registrar {
    Registrar(const char* n, std::function<void()> f) { registry().push_back({n, f}); }
};

inline void report_fail(const char* file, int line, const std::string& msg) {
    ++failure_count();
    std::printf("    [FAIL] %s:%d\n           %s\n", file, line, msg.c_str());
}

inline int run_all() {
    int failed_tests = 0;
    std::printf("running %d test(s)\n", static_cast<int>(registry().size()));
    for (auto& tc : registry()) {
        const int before = failure_count();
        tc.fn();
        if (failure_count() > before) {
            std::printf("  FAIL  %s\n", tc.name);
            ++failed_tests;
        } else {
            std::printf("  ok    %s\n", tc.name);
        }
    }
    std::printf("\n%d assertion(s), %d failure(s) in %d test(s)\n", check_count(),
                failure_count(), static_cast<int>(registry().size()));
    return failed_tests == 0 ? 0 : 1;
}

}  // namespace testing

#define TEST(name)                                                       \
    static void name();                                                  \
    static ::testing::Registrar test_registrar_##name(#name, name);      \
    static void name()

#define CHECK(cond)                                                             \
    do {                                                                        \
        ++::testing::check_count();                                             \
        if (!(cond)) {                                                          \
            ::testing::report_fail(__FILE__, __LINE__, "CHECK(" #cond ") failed"); \
        }                                                                       \
    } while (0)

#define CHECK_EQ(a, b)                                                          \
    do {                                                                        \
        ++::testing::check_count();                                             \
        auto va_ = (a);                                                         \
        auto vb_ = (b);                                                         \
        if (!(va_ == vb_)) {                                                    \
            ::testing::report_fail(__FILE__, __LINE__,                          \
                                   "CHECK_EQ(" #a ", " #b ") failed: " +        \
                                       std::to_string(va_) + " != " +           \
                                       std::to_string(vb_));                    \
        }                                                                       \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                   \
    do {                                                                        \
        ++::testing::check_count();                                             \
        const double va_ = static_cast<double>(a);                              \
        const double vb_ = static_cast<double>(b);                              \
        if (!(std::fabs(va_ - vb_) <= static_cast<double>(tol))) {              \
            ::testing::report_fail(__FILE__, __LINE__,                          \
                                   "CHECK_NEAR(" #a ", " #b ") failed: " +      \
                                       std::to_string(va_) + " vs " +           \
                                       std::to_string(vb_) + " (tol " +         \
                                       std::to_string(static_cast<double>(tol)) + ")"); \
        }                                                                       \
    } while (0)

#define TEST_MAIN()                     \
    int main() { return ::testing::run_all(); }
