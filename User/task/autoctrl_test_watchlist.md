# AutoCtrl 测试实时变量对照清单

适用范围：当前 autoctrltest 分支，Ascend200 为“仅切步、不下发动作命令”版本。

## 1. 建议监视变量分组

### A 触发输入组（确认触发手势被识别）
- dr16.header.online
- dr16.data.sw_l
- dr16.data.sw_r
- last_sw_r
- auto_ctrl_inited

通过判据：
- 上 200 触发手势为 sw_l=DR16_SW_MID 且 sw_r 从 MID 切到 UP。

### B AutoCtrl 状态组（确认状态机是否推进）
- auto_ctrl.state
- auto_ctrl.result
- auto_ctrl.fault
- auto_ctrl.template_id
- auto_ctrl.current_zone
- auto_ctrl.target_zone
- auto_ctrl.template_ctx.step_index
- auto_ctrl.template_ctx.step_enter_time_ms
- auto_ctrl.state_enter_time_ms

通过判据：
- 触发后应依次出现 PREALIGN -> RUN_TEMPLATE。
- 完成时应到 SUCCESS；异常时到 FAIL 并带 fault。

### C 姿态与传感反馈组（确认切步条件是否满足）
- auto_ctrl.feedback.yaw_auto_rad
- auto_ctrl.target_yaw_rad
- auto_ctrl.yaw_error_rad
- auto_ctrl.yaw_tolerance_rad
- auto_ctrl.feedback.sick_front_cm
- auto_ctrl.feedback.sick_left_cm
- auto_ctrl.feedback.sick_right_cm
- auto_ctrl.feedback.sick_rear_cm
- auto_ctrl.feedback.head_front_photo_triggered
- auto_ctrl.feedback.head_rear_photo_triggered

通过判据：
- Step0 依赖 head_front_photo_triggered、yaw 对齐和稳定时间。
- Step2/Step3 依赖 head_rear_photo_triggered。

### D 输出命令组（确认“只切步不动作”）
- auto_ctrl.chassis_cmd.mode
- auto_ctrl.chassis_cmd.ctrl_vec.vx
- auto_ctrl.chassis_cmd.ctrl_vec.vy
- auto_ctrl.chassis_cmd.ctrl_vec.wz
- auto_ctrl.pole_cmd.mode
- auto_ctrl.pole_cmd.auto_target_enable[0]
- auto_ctrl.pole_cmd.auto_target_enable[1]
- auto_ctrl.pole_cmd.auto_target_lift[0]
- auto_ctrl.pole_cmd.auto_target_lift[1]

通过判据：
- PREALIGN 阶段允许有底盘指令。
- RUN_TEMPLATE 阶段（step0~step5）应维持 RELAX/零输出，不应出现动作命令。

### E 计时与阈值组（确认是否超时）
- now_ms（或 osKernelGetTickCount）
- step_elapsed = now_ms - auto_ctrl.template_ctx.step_enter_time_ms
- state_elapsed = now_ms - auto_ctrl.state_enter_time_ms

对照参数：
- pole_extend_settle_ms = 900
- head_front_photo_timeout_ms = 1800
- front_retract_settle_ms = 800
- head_rear_photo_timeout_ms = 2200
- rear_retract_move_ms = 700
- prealign_timeout_ms = 2000
- template_timeout_ms = 10000

## 2. 场景-变量-预期对照

### 场景 1：触发上 200
操作：sw_l 置 MID，sw_r 从 MID 拨到 UP。

重点看：
- dr16.data.sw_l / dr16.data.sw_r / last_sw_r
- auto_ctrl.state / auto_ctrl.template_id / auto_ctrl.result

预期：
- AutoCtrl_StartTransition 成功后，state 进入 PREALIGN，template_id 为 ASCEND_200。

### 场景 2：PREALIGN 阶段
重点看：
- auto_ctrl.state（应为 PREALIGN）
- auto_ctrl.yaw_error_rad 与 auto_ctrl.yaw_tolerance_rad
- auto_ctrl.chassis_cmd.ctrl_vec.vx、wz
- state_elapsed

预期：
- yaw_error 逐步逼近容差。
- 在容差内切到 RUN_TEMPLATE。
- 超过 2000ms 且未对齐，fault=PREALIGN_TIMEOUT。

### 场景 3：RUN_TEMPLATE Step0
重点看：
- auto_ctrl.template_ctx.step_index（应为 0）
- auto_ctrl.feedback.head_front_photo_triggered
- abs(auto_ctrl.yaw_error_rad) <= auto_ctrl.yaw_tolerance_rad
- step_elapsed 与 pole_extend_settle_ms
- auto_ctrl.fault

预期：
- 前光电触发、yaw 对齐且稳定时间满足后切到 step1。
- 超过 head_front_photo_timeout_ms 触发 SENSOR_INVALID。

### 场景 4：RUN_TEMPLATE Step1
重点看：
- step_index（应为 1）
- step_elapsed 与 front_retract_settle_ms

预期：
- 到时后自动切到 step2。

### 场景 5：RUN_TEMPLATE Step2
重点看：
- step_index（应为 2）
- auto_ctrl.feedback.head_rear_photo_triggered
- step_elapsed 与 head_rear_photo_timeout_ms

预期：
- head_rear_photo_triggered=true 时切到 step3。
- 超时触发 SENSOR_INVALID。

### 场景 6：RUN_TEMPLATE Step3
重点看：
- step_index（应为 3）
- auto_ctrl.feedback.head_rear_photo_triggered
- step_elapsed 与 head_rear_photo_timeout_ms

预期：
- head_rear_photo_triggered=true 时切到 step4。
- 超时触发 SENSOR_INVALID。

### 场景 7：RUN_TEMPLATE Step4
重点看：
- step_index（应为 4）
- step_elapsed 与 rear_retract_move_ms

预期：
- 到时后切 step5。

### 场景 8：完成
重点看：
- step_index（到 5）
- auto_ctrl.state / auto_ctrl.result / auto_ctrl.fault

预期：
- state=SUCCESS，result=SUCCESS，fault=NONE。

## 3. 快速故障定位

- 进不了 RUN_TEMPLATE：优先检查 yaw_error、yaw_tolerance、prealign 超时。
- 卡在 Step0：优先检查 head_front_photo_triggered 是否变化、yaw 是否达标、step_elapsed 是否已超过 900ms。
- Step2/3 卡住：优先检查 head_rear_photo_triggered 是否变化。
- 进入 FAIL：先看 auto_ctrl.fault，再回看对应 timeout 条件。
- RUN_TEMPLATE 仍有明显动作命令：检查是否存在其它模块覆盖 chassis/pole 命令。
