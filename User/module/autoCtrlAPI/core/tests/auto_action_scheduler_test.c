#include "module/autoCtrlAPI/core/auto_action_scheduler.h"

#include <assert.h>
#include <stdio.h>

enum {
  RES_CHASSIS = (1u << 0),
  RES_POLE = (1u << 1),
  RES_ARM = (1u << 2),
  RES_STORE = (1u << 3),
  RES_ROD = (1u << 4),
  RES_VALVE = (1u << 5),
};

int main(void) {
  AutoActionScheduler_t scheduler;
  AutoActionScheduler_Init(&scheduler);

  AutoActionJob_t *fused = 0;
  AutoActionSchedulerReject_t reject = AUTO_ACTION_SCHED_REJECT_NONE;
  assert(AutoActionScheduler_Submit(
      &scheduler, 10u, 27u, AUTO_ACTION_EXECUTOR_ORE,
      RES_CHASSIS | RES_POLE | RES_ARM | RES_STORE | RES_VALVE, 0x07u, 0u,
      100u, &fused, &reject));
  assert(fused != 0);

  AutoActionJob_t *duplicate = 0;
  assert(AutoActionScheduler_Submit(
      &scheduler, 10u, 27u, AUTO_ACTION_EXECUTOR_ORE,
      RES_CHASSIS | RES_POLE | RES_ARM | RES_STORE | RES_VALVE, 0x07u, 0u,
      101u, &duplicate, &reject));
  assert(duplicate == fused);

  AutoActionJob_t *step = 0;
  assert(AutoActionScheduler_Submit(
      &scheduler, 11u, 23u, AUTO_ACTION_EXECUTOR_STEP,
      RES_CHASSIS | RES_POLE, 0x04u, 0u, 102u, &step, &reject));

  AutoActionJob_t *start = AutoActionScheduler_NextStartable(
      &scheduler, 0u, 0u, 103u);
  assert(start == fused);
  AutoActionScheduler_MarkStarted(&scheduler, fused, 103u);

  start = AutoActionScheduler_NextStartable(&scheduler, 0u, 0u, 104u);
  assert(start == 0);
  assert(step->state == AUTO_ACTION_JOB_WAIT_RESOURCE);
  assert(step->owned_resource_mask == 0u);
  assert(step->waiting_resource_mask == (RES_CHASSIS | RES_POLE));

  assert(AutoActionScheduler_SetOwnedResources(
      &scheduler, fused, RES_ARM | RES_STORE | RES_VALVE, 0u));
  start = AutoActionScheduler_NextStartable(&scheduler, 0u, 0u, 105u);
  assert(start == step);
  AutoActionScheduler_MarkStarted(&scheduler, step, 105u);
  assert(step->owned_resource_mask == (RES_CHASSIS | RES_POLE));
  assert(fused->state == AUTO_ACTION_JOB_RUNNING);

  AutoActionJob_t *rod = 0;
  assert(AutoActionScheduler_Submit(
      &scheduler, 12u, 12u, AUTO_ACTION_EXECUTOR_ROD,
      RES_ROD | RES_STORE | RES_VALVE, 0x01u, 0u, 106u, &rod, &reject));
  start = AutoActionScheduler_NextStartable(&scheduler, 0u, 0u, 107u);
  assert(start == 0);
  assert(rod->owned_resource_mask == 0u);
  assert((rod->waiting_resource_mask & (RES_STORE | RES_VALVE)) != 0u);

  /* A reporting policy may turn a terminal executor failure into public
   * success.  The scheduler must not leak stale failure bits or expose an
   * internally inconsistent SUCCEEDED record. */
  AutoActionScheduler_SetProgress(&scheduler, fused, 0u, 0x04u, 0x03u,
                                  0x0022u, 9u);
  AutoActionScheduler_Finish(&scheduler, fused, AUTO_ACTION_JOB_SUCCEEDED, 0u,
                             108u);
  assert(fused->completed_segment_mask == fused->required_segment_mask);
  assert(fused->failed_segment_mask == 0u);
  assert(fused->failure_mask == 0u);
  start = AutoActionScheduler_NextStartable(&scheduler, 0u, 0u, 109u);
  assert(start == rod);
  AutoActionScheduler_MarkStarted(&scheduler, rod, 109u);
  assert((AutoActionScheduler_GetOwnedResourceMask(&scheduler) & RES_STORE) !=
         0u);

  AutoActionScheduler_Finish(&scheduler, step, AUTO_ACTION_JOB_SUCCEEDED, 0u,
                             110u);
  AutoActionScheduler_Finish(&scheduler, rod, AUTO_ACTION_JOB_CANCELLED, 0u,
                             111u);
  assert(AutoActionScheduler_GetOwnedResourceMask(&scheduler) == 0u);
  assert(AutoActionScheduler_Acknowledge(&scheduler, fused));
  assert(AutoActionScheduler_Acknowledge(&scheduler, step));
  assert(AutoActionScheduler_Acknowledge(&scheduler, rod));

  AutoActionJob_t *store = 0;
  AutoActionJob_t *step2 = 0;
  assert(AutoActionScheduler_Submit(
      &scheduler, 20u, 2u, AUTO_ACTION_EXECUTOR_ORE,
      RES_ARM | RES_STORE | RES_VALVE, 0x02u, 0u, 200u, &store, &reject));
  assert(AutoActionScheduler_Submit(
      &scheduler, 21u, 23u, AUTO_ACTION_EXECUTOR_STEP,
      RES_CHASSIS | RES_POLE, 0x04u, 0u, 201u, &step2, &reject));

  start = AutoActionScheduler_NextStartable(
      &scheduler, 0u, (uint8_t)(1u << AUTO_ACTION_EXECUTOR_ORE), 202u);
  assert(start == step2);
  assert(store->state == AUTO_ACTION_JOB_WAIT_RESOURCE);
  assert(store->blocked_reason == AUTO_ACTION_BLOCK_EXECUTOR);
  AutoActionScheduler_MarkStarted(&scheduler, step2, 202u);
  AutoActionScheduler_Finish(&scheduler, step2, AUTO_ACTION_JOB_SUCCEEDED, 0u,
                             203u);

  start = AutoActionScheduler_NextStartable(
      &scheduler, RES_STORE, 0u, 204u);
  assert(start == 0);
  assert(store->owned_resource_mask == 0u);
  assert(store->waiting_resource_mask == RES_STORE);

  start = AutoActionScheduler_NextStartable(&scheduler, 0u, 0u, 205u);
  assert(start == store);
  AutoActionScheduler_MarkStarted(&scheduler, store, 205u);
  AutoActionScheduler_Finish(&scheduler, store, AUTO_ACTION_JOB_SUCCEEDED, 0u,
                             206u);
  assert(AutoActionScheduler_Acknowledge(&scheduler, step2));
  assert(AutoActionScheduler_Acknowledge(&scheduler, store));

  puts("auto_action_scheduler_test: PASS");
  return 0;
}
