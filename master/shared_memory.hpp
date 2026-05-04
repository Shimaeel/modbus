#pragma once
/**
 * @file shared_memory.hpp
 * @brief Layer 3 — process-local point database (a.k.a. shared memory / tag DB).
 *
 * @details
 * The "shared memory" the manager described: function snippets read/write the
 * relay and drop the result here, keyed by a string tag. The state machine
 * (or any consumer — HMI, logger, API) reads from here without touching the
 * wire.
 *
 *   producer side:  readContactInputs(master, sm)  -> sm.setBools("io.input", ...)
 *   consumer side:  auto pt = sm.get("io.input");  -> pt.boolVec, pt.timestamp, pt.quality
 *
 * Plain C++ only — no asio, no Modbus types beyond what's needed. A single
 * `std::mutex` guards the underlying map; concurrent producers/consumers are
 * safe at single-key granularity (each `set`/`get` is atomic).
 *
 * Quality flags follow the convention every SCADA system uses:
 *   GOOD     — value reflects a successful read.
 *   STALE    — last read failed, previous value is still here for display.
 *   BAD      — read failed, value is meaningless.
 *   UNKNOWN  — never read.
 */

#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace Modbus {

/** @brief Point-quality enum (SCADA convention). */
enum class Quality : uint8_t {
    UNKNOWN = 0,
    GOOD,
    STALE,
    BAD,
};

inline const char* qualityStr(Quality q)
{
    switch (q) {
    case Quality::GOOD:    return "GOOD";
    case Quality::STALE:   return "STALE";
    case Quality::BAD:     return "BAD";
    default:               return "UNKNOWN";
    }
}

/**
 * @brief Stored point — value + timestamp + quality.
 *
 * The `value` variant covers the four shapes Modbus produces:
 *   - bool                         (single coil / contact)
 *   - uint16_t                     (single register)
 *   - std::vector<bool>            (block of coils)
 *   - std::vector<uint16_t>        (block of registers)
 *   - std::string                  (decoded ASCII string from registers)
 */
struct Point {
    using Value = std::variant<
        std::monostate,
        bool,
        uint16_t,
        std::vector<bool>,
        std::vector<uint16_t>,
        std::string
    >;

    Value                                  value;
    std::chrono::steady_clock::time_point  timestamp;
    Quality                                quality{Quality::UNKNOWN};
    std::string                            error;   ///< populated when quality == BAD/STALE
};

/**
 * @class SharedMemory
 * @brief Thread-safe key→Point store.
 *
 * Mutex granularity is coarse (one lock for the whole map). That's fine for
 * Phase A — we have a handful of tags and one polling thread. Refine to a
 * sharded mutex or a lock-free reader pattern only if profiling demands.
 */
class SharedMemory {
public:
    /** @brief Store/replace value with quality=GOOD. */
    template <typename T>
    void set(const std::string& key, T v)
    {
        std::lock_guard<std::mutex> g(mtx_);
        auto& p     = map_[key];
        p.value     = std::move(v);
        p.timestamp = std::chrono::steady_clock::now();
        p.quality   = Quality::GOOD;
        p.error.clear();
    }

    /** @brief Mark an existing point BAD with an error message. Keeps last value as STALE if present. */
    void markBad(const std::string& key, std::string err)
    {
        std::lock_guard<std::mutex> g(mtx_);
        auto& p   = map_[key];
        // If we previously had a good value, demote to STALE so UI can still display it.
        p.quality = std::holds_alternative<std::monostate>(p.value)
                        ? Quality::BAD
                        : Quality::STALE;
        p.error   = std::move(err);
        p.timestamp = std::chrono::steady_clock::now();
    }

    /** @brief Fetch a copy of a point, or nullopt if the key is absent. */
    std::optional<Point> get(const std::string& key) const
    {
        std::lock_guard<std::mutex> g(mtx_);
        auto it = map_.find(key);
        if (it == map_.end()) return std::nullopt;
        return it->second;
    }

    /** @brief Snapshot of all keys (useful for UI dumps / CSV export). */
    std::map<std::string, Point> snapshot() const
    {
        std::lock_guard<std::mutex> g(mtx_);
        return map_;
    }

    /** @brief Drop everything (e.g., on disconnect). */
    void clear()
    {
        std::lock_guard<std::mutex> g(mtx_);
        map_.clear();
    }

private:
    mutable std::mutex                      mtx_;
    std::map<std::string, Point>            map_;
};

} // namespace Modbus
