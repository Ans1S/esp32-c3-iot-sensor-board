#include <assert.h>
#include <stdio.h>
#include "motion_feedback.h"
int main() {
  lil::MotionFeedbackTracker tracker;
  for (int i=0;i<104*60;++i) tracker.observe(1 + .005F*sinf(i));
  assert(tracker.value().steps == 0 && tracker.value().activeSeconds == 0);
  for (int i=0;i<104*30;++i) tracker.observe(1 + .35F*sinf(2*3.14159265F*2*i/104));
  assert(tracker.value().steps >= 57 && tracker.value().steps <= 61);
  assert(tracker.value().activeSeconds >= 28 && tracker.value().activeSeconds <= 30);
  const auto previous=tracker.value(); tracker.restartFilter();
  for (int i=0;i<104*5;++i) tracker.observe(1);
  assert(tracker.value().steps == previous.steps && tracker.value().activeSeconds == previous.activeSeconds);
  tracker = lil::MotionFeedbackTracker{};
  for (int i=0;i<104*5;++i) tracker.observe(i==104?2:1);
  assert(tracker.value().steps == 0);
  puts("Motion feedback: stationary noise, 120 steps/min, activity, recovery and isolated impact passed");
}
