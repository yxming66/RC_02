#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AUTO_ACTION_SCHEDULER_CAPACITY (4u)
#define AUTO_ACTION_RESOURCE_COUNT (6u)

typedef enum {
  AUTO_ACTION_JOB_FREE = 0,
  AUTO_ACTION_JOB_QUEUED,
  AUTO_ACTION_JOB_WAIT_RESOURCE,
  AUTO_ACTION_JOB_STARTING,
  AUTO_ACTION_JOB_RUNNING,
  AUTO_ACTION_JOB_WAIT_GATE,
  AUTO_ACTION_JOB_CANCEL_REQUESTED,
  AUTO_ACTION_JOB_SUCCEEDED,
  AUTO_ACTION_JOB_FAILED,
  AUTO_ACTION_JOB_CANCELLED,
  AUTO_ACTION_JOB_REJECTED,
} AutoActionJobState_t;

typedef enum {
  AUTO_ACTION_BLOCK_NONE = 0,
  AUTO_ACTION_BLOCK_RESOURCE,
  AUTO_ACTION_BLOCK_EXECUTOR,
  AUTO_ACTION_BLOCK_DEPENDENCY,
  AUTO_ACTION_BLOCK_MOTION_CONSTRAINT,
  AUTO_ACTION_BLOCK_SENSOR,
  AUTO_ACTION_BLOCK_EXTERNAL_GATE,
} AutoActionBlockedReason_t;

typedef enum {
  AUTO_ACTION_EXECUTOR_NONE = 0,
  AUTO_ACTION_EXECUTOR_ORE,
  AUTO_ACTION_EXECUTOR_STEP,
  AUTO_ACTION_EXECUTOR_ROD,
  AUTO_ACTION_EXECUTOR_SICK,
} AutoActionExecutor_t;

typedef enum {
  AUTO_ACTION_SCHED_REJECT_NONE = 0,
  AUTO_ACTION_SCHED_REJECT_INVALID_REQUEST,
  AUTO_ACTION_SCHED_REJECT_INVALID_ACTION,
  AUTO_ACTION_SCHED_REJECT_QUEUE_FULL,
  AUTO_ACTION_SCHED_REJECT_REQUEST_CONFLICT,
  AUTO_ACTION_SCHED_REJECT_JOB_NOT_FOUND,
  AUTO_ACTION_SCHED_REJECT_INVALID_STATE,
  AUTO_ACTION_SCHED_REJECT_START_FAILED,
} AutoActionSchedulerReject_t;

typedef struct {
  bool allocated;
  uint16_t request_id;
  uint16_t job_id;
  uint8_t action;
  uint8_t executor;
  uint8_t state;
  uint8_t flags;
  uint8_t required_resource_mask;
  uint8_t owned_resource_mask;
  uint8_t waiting_resource_mask;
  uint8_t blocked_reason;
  uint8_t required_segment_mask;
  uint8_t running_segment_mask;
  uint8_t completed_segment_mask;
  uint8_t failed_segment_mask;
  uint8_t active_node;
  uint8_t reject_reason;
  uint16_t failure_mask;
  uint16_t generation;
  uint32_t submit_order;
  uint32_t submit_time_ms;
  uint32_t start_time_ms;
  uint32_t finish_time_ms;
} AutoActionJob_t;

typedef struct {
  AutoActionJob_t jobs[AUTO_ACTION_SCHEDULER_CAPACITY];
  uint16_t resource_owner[AUTO_ACTION_RESOURCE_COUNT];
  uint16_t next_job_id;
  uint16_t generation;
  uint32_t next_submit_order;
  uint8_t report_cursor;
} AutoActionScheduler_t;

void AutoActionScheduler_Init(AutoActionScheduler_t *scheduler);
bool AutoActionScheduler_Submit(AutoActionScheduler_t *scheduler,
                                uint16_t request_id, uint8_t action,
                                AutoActionExecutor_t executor,
                                uint8_t required_resources,
                                uint8_t required_segments, uint8_t flags,
                                uint32_t now_ms, AutoActionJob_t **job,
                                AutoActionSchedulerReject_t *reject_reason);
AutoActionJob_t *AutoActionScheduler_FindByJobId(
    AutoActionScheduler_t *scheduler, uint16_t job_id);
AutoActionJob_t *AutoActionScheduler_FindByRequestId(
    AutoActionScheduler_t *scheduler, uint16_t request_id);
AutoActionJob_t *AutoActionScheduler_NextStartable(
    AutoActionScheduler_t *scheduler, uint8_t externally_owned_resources,
    uint32_t now_ms);
void AutoActionScheduler_MarkStarted(AutoActionScheduler_t *scheduler,
                                     AutoActionJob_t *job,
                                     uint32_t now_ms);
bool AutoActionScheduler_SetOwnedResources(AutoActionScheduler_t *scheduler,
                                           AutoActionJob_t *job,
                                           uint8_t desired_resources,
                                           uint8_t externally_owned_resources);
void AutoActionScheduler_SetProgress(AutoActionScheduler_t *scheduler,
                                     AutoActionJob_t *job,
                                     uint8_t running_segments,
                                     uint8_t completed_segments,
                                     uint8_t failed_segments,
                                     uint16_t failure_mask,
                                     uint8_t active_node);
bool AutoActionScheduler_RequestCancel(AutoActionScheduler_t *scheduler,
                                       AutoActionJob_t *job);
void AutoActionScheduler_SetWaitGate(AutoActionScheduler_t *scheduler,
                                     AutoActionJob_t *job,
                                     bool waiting);
void AutoActionScheduler_Finish(AutoActionScheduler_t *scheduler,
                                AutoActionJob_t *job,
                                AutoActionJobState_t terminal_state,
                                uint16_t failure_mask, uint32_t now_ms);
bool AutoActionScheduler_Acknowledge(AutoActionScheduler_t *scheduler,
                                     AutoActionJob_t *job);
AutoActionJob_t *AutoActionScheduler_NextReport(
    AutoActionScheduler_t *scheduler);
uint8_t AutoActionScheduler_GetOwnedResourceMask(
    const AutoActionScheduler_t *scheduler);
bool AutoActionScheduler_IsTerminal(AutoActionJobState_t state);

#ifdef __cplusplus
}
#endif
