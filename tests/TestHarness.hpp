// =============================================================================
//  tests/TestHarness.hpp
//
//  A tiny, dependency-free unit-test harness. Kept deliberately minimal so the
//  headless foundation builds with nothing but a C++17 compiler. Each test file
//  defines TESTS via TEST_CASE() and links against this header's main().
// =============================================================================
#pragma once

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace fsimtest {

struct TestCase {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}

inline int& failureCount() { static int f = 0; return f; }

struct Registrar {
    Registrar(const std::string& name, std::function<void()> fn) {
        registry().push_back({name, std::move(fn)});
    }
};

inline void reportFailure(const char* file, int line, const std::string& msg) {
    ++failureCount();
    std::printf("    [FAIL] %s:%d  %s\n", file, line, msg.c_str());
}

inline bool approxEqual(double a, double b, double tol) {
    return std::fabs(a - b) <= tol;
}

} // namespace fsimtest

// ---- Macros -----------------------------------------------------------------
#define TEST_CASE(name)                                                        \
    static void name();                                                        \
    static ::fsimtest::Registrar registrar_##name(#name, name);                \
    static void name()

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) ::fsimtest::reportFailure(__FILE__, __LINE__,             \
                                               "CHECK failed: " #cond);        \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                  \
    do {                                                                       \
        const double _va = (a), _vb = (b), _vt = (tol);                        \
        if (!::fsimtest::approxEqual(_va, _vb, _vt)) {                         \
            ::fsimtest::reportFailure(__FILE__, __LINE__,                      \
                std::string("CHECK_NEAR failed: " #a " (") +                   \
                std::to_string(_va) + ") vs " #b " (" +                        \
                std::to_string(_vb) + "), tol " + std::to_string(_vt));        \
        }                                                                      \
    } while (0)

// Single shared main() for every test executable.
#define FSIM_TEST_MAIN()                                                       \
    int main() {                                                              \
        int run = 0;                                                          \
        for (auto& t : ::fsimtest::registry()) {                             \
            std::printf("  RUN  %s\n", t.name.c_str());                       \
            const int before = ::fsimtest::failureCount();                    \
            t.fn();                                                           \
            ++run;                                                            \
            if (::fsimtest::failureCount() == before)                         \
                std::printf("    [ ok ]\n");                                  \
        }                                                                     \
        std::printf("\n%d test(s) run, %d failure(s)\n",                      \
                    run, ::fsimtest::failureCount());                         \
        return ::fsimtest::failureCount() == 0 ? 0 : 1;                       \
    }
