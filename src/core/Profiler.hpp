// =============================================================================
//  core/Profiler.hpp
//
//  Lightweight, dependency-free performance instrumentation: named timing
//  accumulators (count / total / min / max / average) and an RAII ScopedTimer
//  that records a labelled span on destruction. Used to profile the physics
//  step, terrain build, and render passes when tuning. The accumulation logic
//  is pure (durations can be supplied directly) so it is unit-tested.
// =============================================================================
#pragma once

#include <chrono>
#include <limits>
#include <map>
#include <string>

namespace fsim {

class Profiler {
public:
    struct Stat {
        long long count{0};
        double total{0.0};  // seconds
        double minVal{std::numeric_limits<double>::infinity()};
        double maxVal{0.0};
        double average() const { return count > 0 ? total / double(count) : 0.0; }
    };

    void add(const std::string& name, double seconds) {
        Stat& s = stats_[name];
        ++s.count;
        s.total += seconds;
        if (seconds < s.minVal) s.minVal = seconds;
        if (seconds > s.maxVal) s.maxVal = seconds;
    }

    const Stat& get(const std::string& name) const {
        static const Stat kEmpty;
        auto it = stats_.find(name);
        return it == stats_.end() ? kEmpty : it->second;
    }

    const std::map<std::string, Stat>& stats() const { return stats_; }
    void reset() { stats_.clear(); }

private:
    std::map<std::string, Stat> stats_;
};

// Records the lifetime of the scope into a Profiler under 'name'.
class ScopedTimer {
public:
    ScopedTimer(Profiler& profiler, std::string name)
        : profiler_(profiler), name_(std::move(name)),
          start_(std::chrono::high_resolution_clock::now()) {}

    ~ScopedTimer() {
        const auto end = std::chrono::high_resolution_clock::now();
        profiler_.add(name_, std::chrono::duration<double>(end - start_).count());
    }

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
    Profiler& profiler_;
    std::string name_;
    std::chrono::high_resolution_clock::time_point start_;
};

} // namespace fsim
