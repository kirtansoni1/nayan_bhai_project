#include <unity.h>

#include "sensor_counter.h"

namespace {

// Runs one cycle: window open, optional detections, cycle end.
// Returns true when the machine must stop.
bool cycle(SensorCounter &c, bool active_at_open, int edges) {
  c.open_window(active_at_open);
  for (int i = 0; i < edges; ++i) {
    c.detect();
  }
  return c.close_window();
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_two_in_a_row_stops() {
  SensorCounter c(2);
  TEST_ASSERT_FALSE(cycle(c, false, 1));
  TEST_ASSERT_EQUAL_UINT32(1, c.count());
  TEST_ASSERT_TRUE(cycle(c, false, 1));
  TEST_ASSERT_EQUAL_UINT32(0, c.count());
}

void test_miss_resets_count() {
  SensorCounter c(2);
  TEST_ASSERT_FALSE(cycle(c, false, 0));
  TEST_ASSERT_FALSE(cycle(c, false, 1));
  TEST_ASSERT_FALSE(cycle(c, false, 0));
  TEST_ASSERT_EQUAL_UINT32(0, c.count());
  TEST_ASSERT_FALSE(cycle(c, false, 1));
  TEST_ASSERT_TRUE(cycle(c, false, 1));
}

void test_many_hits_in_one_window_count_once() {
  SensorCounter c(2);
  TEST_ASSERT_FALSE(cycle(c, false, 5));
  TEST_ASSERT_EQUAL_UINT32(1, c.count());
}

void test_detect_outside_window_ignored() {
  SensorCounter c(2);
  TEST_ASSERT_FALSE(c.detect());
  TEST_ASSERT_FALSE(cycle(c, false, 1));
  TEST_ASSERT_FALSE(c.detect());  // between cycles
  TEST_ASSERT_EQUAL_UINT32(1, c.count());
}

void test_active_at_open_counts_once() {
  SensorCounter c(2);
  c.open_window(true);
  TEST_ASSERT_TRUE(c.hit());
  TEST_ASSERT_FALSE(c.detect());  // late edge of the same object
  TEST_ASSERT_FALSE(c.close_window());
  TEST_ASSERT_EQUAL_UINT32(1, c.count());
  TEST_ASSERT_TRUE(cycle(c, true, 0));
}

void test_limit_reached_flag_before_cycle_end() {
  SensorCounter c(2);
  cycle(c, false, 1);
  c.open_window(false);
  TEST_ASSERT_FALSE(c.limit_reached());
  c.detect();
  TEST_ASSERT_TRUE(c.limit_reached());
  TEST_ASSERT_TRUE(c.close_window());
  TEST_ASSERT_FALSE(c.limit_reached());
}

void test_limit_one_stops_first_hit() {
  SensorCounter c(1);
  TEST_ASSERT_FALSE(cycle(c, false, 0));
  TEST_ASSERT_TRUE(cycle(c, false, 1));
}

void test_reset_clears_count_and_window() {
  SensorCounter c(3);
  cycle(c, false, 1);
  c.open_window(false);
  c.detect();
  c.reset();
  TEST_ASSERT_EQUAL_UINT32(0, c.count());
  TEST_ASSERT_FALSE(c.window_open());
  TEST_ASSERT_FALSE(c.close_window());  // no open window, no change
}

void test_close_without_open_keeps_count() {
  SensorCounter c(3);
  cycle(c, false, 1);
  TEST_ASSERT_FALSE(c.close_window());  // cycle aborted before task 9
  TEST_ASSERT_EQUAL_UINT32(1, c.count());
}

void test_long_run_no_overflow() {
  SensorCounter c(2);
  for (int i = 0; i < 100000; ++i) {
    const bool stop = cycle(c, false, (i % 3 == 0) ? 0 : 1);
    TEST_ASSERT_TRUE(c.count() < 2);
    if (stop) {
      TEST_ASSERT_EQUAL_UINT32(0, c.count());
    }
  }
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_two_in_a_row_stops);
  RUN_TEST(test_miss_resets_count);
  RUN_TEST(test_many_hits_in_one_window_count_once);
  RUN_TEST(test_detect_outside_window_ignored);
  RUN_TEST(test_active_at_open_counts_once);
  RUN_TEST(test_limit_reached_flag_before_cycle_end);
  RUN_TEST(test_limit_one_stops_first_hit);
  RUN_TEST(test_reset_clears_count_and_window);
  RUN_TEST(test_close_without_open_keeps_count);
  RUN_TEST(test_long_run_no_overflow);
  return UNITY_END();
}
