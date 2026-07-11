#include "module/autoCtrlAPI/core/auto_action_scheduler.h"

#include <string.h>

static uint16_t AutoActionScheduler_NextJobId(
    AutoActionScheduler_t *scheduler) {
  uint16_t result = scheduler->next_job_id++;
  if (result == 0u) {
    result = scheduler->next_job_id++;
  }
  if (scheduler->next_job_id == 0u) {
    scheduler->next_job_id = 1u;
  }
  return result;
}

static uint8_t AutoActionScheduler_Conflicts(
    const AutoActionScheduler_t *scheduler, uint16_t job_id,
    uint8_t requested, uint8_t externally_owned_resources) {
  uint8_t conflicts = (uint8_t)(requested & externally_owned_resources);
  for (uint8_t bit = 0u; bit < AUTO_ACTION_RESOURCE_COUNT; ++bit) {
    const uint8_t mask = (uint8_t)(1u << bit);
    if ((requested & mask) != 0u && scheduler->resource_owner[bit] != 0u &&
        scheduler->resource_owner[bit] != job_id) {
      conflicts |= mask;
    }
  }
  return conflicts;
}

static void AutoActionScheduler_ReleaseAll(AutoActionScheduler_t *scheduler,
                                           AutoActionJob_t *job) {
  if (scheduler == 0 || job == 0) {
    return;
  }
  for (uint8_t bit = 0u; bit < AUTO_ACTION_RESOURCE_COUNT; ++bit) {
    if (scheduler->resource_owner[bit] == job->job_id) {
      scheduler->resource_owner[bit] = 0u;
    }
  }
  job->owned_resource_mask = 0u;
  job->waiting_resource_mask = 0u;
}

static void AutoActionScheduler_Touch(AutoActionScheduler_t *scheduler,
                                      AutoActionJob_t *job) {
  scheduler->generation++;
  if (scheduler->generation == 0u) {
    scheduler->generation = 1u;
  }
  job->generation = scheduler->generation;
}

void AutoActionScheduler_Init(AutoActionScheduler_t *scheduler) {
  if (scheduler == 0) {
    return;
  }
  memset(scheduler, 0, sizeof(*scheduler));
  scheduler->next_job_id = 1u;
  scheduler->generation = 1u;
  scheduler->next_submit_order = 1u;
}

bool AutoActionScheduler_IsTerminal(AutoActionJobState_t state) {
  return state == AUTO_ACTION_JOB_SUCCEEDED ||
         state == AUTO_ACTION_JOB_FAILED ||
         state == AUTO_ACTION_JOB_CANCELLED ||
         state == AUTO_ACTION_JOB_REJECTED;
}

AutoActionJob_t *AutoActionScheduler_FindByJobId(
    AutoActionScheduler_t *scheduler, uint16_t job_id) {
  if (scheduler == 0 || job_id == 0u) {
    return 0;
  }
  for (uint8_t i = 0u; i < AUTO_ACTION_SCHEDULER_CAPACITY; ++i) {
    AutoActionJob_t *job = &scheduler->jobs[i];
    if (job->allocated && job->job_id == job_id) {
      return job;
    }
  }
  return 0;
}

AutoActionJob_t *AutoActionScheduler_FindByRequestId(
    AutoActionScheduler_t *scheduler, uint16_t request_id) {
  if (scheduler == 0 || request_id == 0u) {
    return 0;
  }
  for (uint8_t i = 0u; i < AUTO_ACTION_SCHEDULER_CAPACITY; ++i) {
    AutoActionJob_t *job = &scheduler->jobs[i];
    if (job->allocated && job->request_id == request_id) {
      return job;
    }
  }
  return 0;
}

bool AutoActionScheduler_Submit(AutoActionScheduler_t *scheduler,
                                uint16_t request_id, uint8_t action,
                                AutoActionExecutor_t executor,
                                uint8_t required_resources,
                                uint8_t required_segments, uint8_t flags,
                                uint32_t now_ms, AutoActionJob_t **job_out,
                                AutoActionSchedulerReject_t *reject_reason) {
  if (job_out != 0) {
    *job_out = 0;
  }
  if (reject_reason != 0) {
    *reject_reason = AUTO_ACTION_SCHED_REJECT_NONE;
  }
  if (scheduler == 0 || request_id == 0u || action == 0u ||
      executor == AUTO_ACTION_EXECUTOR_NONE) {
    if (reject_reason != 0) {
      *reject_reason = AUTO_ACTION_SCHED_REJECT_INVALID_REQUEST;
    }
    return false;
  }

  AutoActionJob_t *existing =
      AutoActionScheduler_FindByRequestId(scheduler, request_id);
  if (existing != 0) {
    if (existing->action != action || existing->executor != (uint8_t)executor) {
      if (reject_reason != 0) {
        *reject_reason = AUTO_ACTION_SCHED_REJECT_REQUEST_CONFLICT;
      }
      return false;
    }
    if (job_out != 0) {
      *job_out = existing;
    }
    return true;
  }

  AutoActionJob_t *slot = 0;
  for (uint8_t i = 0u; i < AUTO_ACTION_SCHEDULER_CAPACITY; ++i) {
    if (!scheduler->jobs[i].allocated) {
      slot = &scheduler->jobs[i];
      break;
    }
  }
  if (slot == 0) {
    if (reject_reason != 0) {
      *reject_reason = AUTO_ACTION_SCHED_REJECT_QUEUE_FULL;
    }
    return false;
  }

  memset(slot, 0, sizeof(*slot));
  slot->allocated = true;
  slot->request_id = request_id;
  slot->job_id = AutoActionScheduler_NextJobId(scheduler);
  slot->action = action;
  slot->executor = (uint8_t)executor;
  slot->state = AUTO_ACTION_JOB_QUEUED;
  slot->flags = flags;
  slot->required_resource_mask = required_resources;
  slot->required_segment_mask = required_segments;
  slot->submit_order = scheduler->next_submit_order++;
  slot->submit_time_ms = now_ms;
  AutoActionScheduler_Touch(scheduler, slot);
  if (job_out != 0) {
    *job_out = slot;
  }
  return true;
}

AutoActionJob_t *AutoActionScheduler_NextStartable(
    AutoActionScheduler_t *scheduler, uint8_t externally_owned_resources,
    uint8_t unavailable_executor_mask, uint32_t now_ms) {
  (void)now_ms;
  if (scheduler == 0) {
    return 0;
  }

  AutoActionJob_t *best = 0;
  for (uint8_t i = 0u; i < AUTO_ACTION_SCHEDULER_CAPACITY; ++i) {
    AutoActionJob_t *job = &scheduler->jobs[i];
    if (!job->allocated ||
        (job->state != AUTO_ACTION_JOB_QUEUED &&
         job->state != AUTO_ACTION_JOB_WAIT_RESOURCE)) {
      continue;
    }
    const uint8_t old_state = job->state;
    const uint8_t old_waiting = job->waiting_resource_mask;
    const uint8_t old_blocked = job->blocked_reason;
    if ((unavailable_executor_mask & (uint8_t)(1u << job->executor)) != 0u) {
      job->waiting_resource_mask = 0u;
      job->blocked_reason = AUTO_ACTION_BLOCK_EXECUTOR;
      job->state = AUTO_ACTION_JOB_WAIT_RESOURCE;
      if (old_state != job->state || old_waiting != job->waiting_resource_mask ||
          old_blocked != job->blocked_reason) {
        AutoActionScheduler_Touch(scheduler, job);
      }
      continue;
    }
    const uint8_t conflicts = AutoActionScheduler_Conflicts(
        scheduler, job->job_id, job->required_resource_mask,
        externally_owned_resources);
    job->waiting_resource_mask = conflicts;
    job->blocked_reason = (conflicts != 0u) ? AUTO_ACTION_BLOCK_RESOURCE
                                            : AUTO_ACTION_BLOCK_NONE;
    job->state = (conflicts != 0u) ? AUTO_ACTION_JOB_WAIT_RESOURCE
                                   : AUTO_ACTION_JOB_QUEUED;
    if (old_state != job->state || old_waiting != job->waiting_resource_mask ||
        old_blocked != job->blocked_reason) {
      AutoActionScheduler_Touch(scheduler, job);
    }
    if (conflicts == 0u &&
        (best == 0 || job->submit_order < best->submit_order)) {
      best = job;
    }
  }
  if (best == 0) {
    return 0;
  }

  if (!AutoActionScheduler_SetOwnedResources(
          scheduler, best, best->required_resource_mask,
          externally_owned_resources)) {
    return 0;
  }
  best->state = AUTO_ACTION_JOB_STARTING;
  best->blocked_reason = AUTO_ACTION_BLOCK_NONE;
  AutoActionScheduler_Touch(scheduler, best);
  return best;
}

void AutoActionScheduler_MarkStarted(AutoActionScheduler_t *scheduler,
                                     AutoActionJob_t *job,
                                     uint32_t now_ms) {
  if (scheduler == 0 || job == 0 || job->state != AUTO_ACTION_JOB_STARTING) {
    return;
  }
  job->state = AUTO_ACTION_JOB_RUNNING;
  job->executor_started = true;
  job->start_time_ms = now_ms;
  AutoActionScheduler_Touch(scheduler, job);
}

bool AutoActionScheduler_SetOwnedResources(AutoActionScheduler_t *scheduler,
                                           AutoActionJob_t *job,
                                           uint8_t desired_resources,
                                           uint8_t externally_owned_resources) {
  if (scheduler == 0 || job == 0 || !job->allocated) {
    return false;
  }
  const uint8_t additions =
      (uint8_t)(desired_resources & ~job->owned_resource_mask);
  const uint8_t conflicts = AutoActionScheduler_Conflicts(
      scheduler, job->job_id, additions, externally_owned_resources);
  if (conflicts != 0u) {
    job->waiting_resource_mask = conflicts;
    job->blocked_reason = AUTO_ACTION_BLOCK_RESOURCE;
    return false;
  }

  for (uint8_t bit = 0u; bit < AUTO_ACTION_RESOURCE_COUNT; ++bit) {
    const uint8_t mask = (uint8_t)(1u << bit);
    if ((desired_resources & mask) != 0u) {
      scheduler->resource_owner[bit] = job->job_id;
    } else if (scheduler->resource_owner[bit] == job->job_id) {
      scheduler->resource_owner[bit] = 0u;
    }
  }
  job->owned_resource_mask = desired_resources;
  job->waiting_resource_mask = 0u;
  job->blocked_reason = AUTO_ACTION_BLOCK_NONE;
  AutoActionScheduler_Touch(scheduler, job);
  return true;
}

void AutoActionScheduler_SetProgress(AutoActionScheduler_t *scheduler,
                                     AutoActionJob_t *job,
                                     uint8_t running_segments,
                                     uint8_t completed_segments,
                                     uint8_t failed_segments,
                                     uint16_t failure_mask,
                                     uint8_t active_node) {
  if (scheduler == 0 || job == 0 || !job->allocated) {
    return;
  }
  job->running_segment_mask = running_segments;
  job->completed_segment_mask |= completed_segments;
  job->failed_segment_mask |= failed_segments;
  job->failed_segment_mask &= (uint8_t)~job->completed_segment_mask;
  job->failure_mask |= failure_mask;
  job->active_node = active_node;
  AutoActionScheduler_Touch(scheduler, job);
}

bool AutoActionScheduler_RequestCancel(AutoActionScheduler_t *scheduler,
                                       AutoActionJob_t *job) {
  if (scheduler == 0 || job == 0 || !job->allocated ||
      AutoActionScheduler_IsTerminal((AutoActionJobState_t)job->state)) {
    return false;
  }
  job->state = AUTO_ACTION_JOB_CANCEL_REQUESTED;
  job->blocked_reason = AUTO_ACTION_BLOCK_NONE;
  AutoActionScheduler_Touch(scheduler, job);
  return true;
}

void AutoActionScheduler_SetWaitGate(AutoActionScheduler_t *scheduler,
                                     AutoActionJob_t *job,
                                     bool waiting) {
  if (scheduler == 0 || job == 0 || !job->allocated) {
    return;
  }
  job->state = waiting ? AUTO_ACTION_JOB_WAIT_GATE : AUTO_ACTION_JOB_RUNNING;
  job->blocked_reason = waiting ? AUTO_ACTION_BLOCK_EXTERNAL_GATE
                                : AUTO_ACTION_BLOCK_NONE;
  AutoActionScheduler_Touch(scheduler, job);
}

void AutoActionScheduler_Finish(AutoActionScheduler_t *scheduler,
                                AutoActionJob_t *job,
                                AutoActionJobState_t terminal_state,
                                uint16_t failure_mask, uint32_t now_ms) {
  if (scheduler == 0 || job == 0 || !job->allocated ||
      !AutoActionScheduler_IsTerminal(terminal_state)) {
    return;
  }
  AutoActionScheduler_ReleaseAll(scheduler, job);
  job->state = (uint8_t)terminal_state;
  job->running_segment_mask = 0u;
  job->failure_mask |= failure_mask;
  job->finish_time_ms = now_ms;
  AutoActionScheduler_Touch(scheduler, job);
}

bool AutoActionScheduler_Acknowledge(AutoActionScheduler_t *scheduler,
                                     AutoActionJob_t *job) {
  if (scheduler == 0 || job == 0 || !job->allocated ||
      !AutoActionScheduler_IsTerminal((AutoActionJobState_t)job->state)) {
    return false;
  }
  AutoActionScheduler_ReleaseAll(scheduler, job);
  memset(job, 0, sizeof(*job));
  scheduler->generation++;
  if (scheduler->generation == 0u) {
    scheduler->generation = 1u;
  }
  return true;
}

AutoActionJob_t *AutoActionScheduler_NextReport(
    AutoActionScheduler_t *scheduler) {
  if (scheduler == 0) {
    return 0;
  }
  for (uint8_t offset = 0u; offset < AUTO_ACTION_SCHEDULER_CAPACITY; ++offset) {
    const uint8_t index = (uint8_t)(
        (scheduler->report_cursor + offset) % AUTO_ACTION_SCHEDULER_CAPACITY);
    if (scheduler->jobs[index].allocated) {
      scheduler->report_cursor =
          (uint8_t)((index + 1u) % AUTO_ACTION_SCHEDULER_CAPACITY);
      return &scheduler->jobs[index];
    }
  }
  return 0;
}

uint8_t AutoActionScheduler_GetOwnedResourceMask(
    const AutoActionScheduler_t *scheduler) {
  if (scheduler == 0) {
    return 0u;
  }
  uint8_t result = 0u;
  for (uint8_t bit = 0u; bit < AUTO_ACTION_RESOURCE_COUNT; ++bit) {
    if (scheduler->resource_owner[bit] != 0u) {
      result |= (uint8_t)(1u << bit);
    }
  }
  return result;
}
