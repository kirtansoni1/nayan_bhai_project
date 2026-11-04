#pragma once

#include <stdint.h>

// Counts cycles in a row whose sensor window saw a detection.
// The window opens after task 9 and closes at cycle end. Detections outside it are ignored.
// Not thread safe, the owner guards it with its own lock.
class SensorCounter {
 public:
  explicit SensorCounter(uint32_t limit) : limit_(limit) {}

  /**
   * Fresh start: count 0, window closed.
   */
  void reset() {
    count_ = 0;
    open_ = false;
    hit_ = false;
  }

  /**
   * Opens the window for the current cycle.
   * @param active_now sensor is already active, counted as this cycle's detection
   */
  void open_window(bool active_now) {
    open_ = true;
    hit_ = false;
    if (active_now) {
      detect();
    }
  }

  /**
   * Detection edge. Only the first one inside an open window counts.
   * @return true if it was counted
   */
  bool detect() {
    if (!open_ || hit_) {
      return false;
    }
    hit_ = true;
    ++count_;
    return true;
  }

  /**
   * Closes the window at cycle end. A cycle without a detection resets the count.
   * @return true when the limit was reached and the machine must stop
   */
  bool close_window() {
    if (!open_) {
      return false;
    }
    open_ = false;
    if (!hit_) {
      count_ = 0;
      return false;
    }
    hit_ = false;
    if (count_ >= limit_) {
      count_ = 0;
      return true;
    }
    return false;
  }

  bool window_open() const { return open_; }
  bool hit() const { return hit_; }
  uint32_t count() const { return count_; }
  uint32_t limit() const { return limit_; }
  bool limit_reached() const { return count_ >= limit_; }

 private:
  uint32_t limit_;
  uint32_t count_ = 0;
  bool open_ = false;
  bool hit_ = false;
};
