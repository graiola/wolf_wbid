/* SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>

namespace wolf_wbid {

class ReferenceFreshness {
 public:
  static std::int64_t now() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  std::uint64_t generation() const { return generation_.load(std::memory_order_acquire); }

  // A callback captures its generation before preparing the buffered command.
  // Publishing an old callback after reset cannot make that command ready.
  // Receipt writers for one task run on its single-threaded ROS executor.
  void received(std::uint64_t generation, std::int64_t receipt = now()) {
    receipt_.store(receipt, std::memory_order_release);
    received_generation_.store(generation, std::memory_order_release);
  }
  void invalidate() { generation_.fetch_add(1, std::memory_order_acq_rel); }

  bool fresh(std::int64_t current, std::int64_t max_age) const {
    return freshUsingClock(max_age, [current] { return current; });
  }

  bool fresh(std::int64_t max_age) const {
    return freshUsingClock(max_age, [] { return now(); });
  }

 private:
  template <class Clock>
  bool freshUsingClock(std::int64_t max_age, Clock&& clock) const {
    const auto generation = generation_.load(std::memory_order_acquire);
    const auto received_generation = received_generation_.load(std::memory_order_acquire);
    const auto receipt = receipt_.load(std::memory_order_acquire);
    // Read time after the receipt: a concurrent callback must not appear to
    // come from the future just because the caller sampled time first.
    const auto current = clock();
    return generation == received_generation && generation == generation_.load(std::memory_order_acquire) &&
           receipt > 0 && max_age > 0 && current >= receipt && current - receipt <= max_age;
  }

  std::atomic<std::int64_t> receipt_{0};
  std::atomic<std::uint64_t> generation_{1};
  std::atomic<std::uint64_t> received_generation_{0};
};

}  // namespace wolf_wbid
