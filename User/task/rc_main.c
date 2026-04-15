/*
    rc_main Task
    
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "bsp/uart.h"
#include "device/dr16.h"
#include "module/chassis.h"
#include "module/armpos.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
DR16_t dr16;
static Chassis_CMD_t chassis_cmd;
static DR16_SwitchPos_t last_sw_l = DR16_SW_ERR;  /* 记录左拨杆上一次状态 */
static DR16_SwitchPos_t last_sw_r = DR16_SW_ERR;  /* 记录右拨杆上一次状态 */
extern bool reset; 

/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_rc_main(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  
  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / RC_MAIN_FREQ;

  osDelay(RC_MAIN_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */
  DR16_Init(&dr16);
  DR16_StartDmaRecv(&dr16);  // 立即启动第一次接收
  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
        // 启动下一次DMA接收
    DR16_StartDmaRecv(&dr16);
    // 等待DMA接收完成
    if (DR16_WaitDmaCplt(100)) {
      DR16_ParseData(&dr16);
    } else {
      DR16_Offline(&dr16);
    }
      ArmPos_CMD_t armpos_cmd;
      
      
      
    if (!dr16.header.online || dr16.data.sw_l == DR16_SW_UP) {
      chassis_cmd.mode = CHASSIS_MODE_RELAX;
      chassis_cmd.ctrl_vec.vx = 0.0f;
      chassis_cmd.ctrl_vec.vy = 0.0f;
      chassis_cmd.ctrl_vec.wz = 0.0f;

      armpos_cmd.mode = ARMPOS_MODE_RELAX;
      armpos_cmd.dm_delta = 0.0f;
      armpos_cmd.lz_delta = 0.0f;
      armpos_cmd.rm_delta = 0.0f;
      
    } else if (dr16.data.sw_l == DR16_SW_MID) {
      chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
      chassis_cmd.ctrl_vec.vx = dr16.data.ch_r_x;
      chassis_cmd.ctrl_vec.vy = dr16.data.ch_r_y;
      chassis_cmd.ctrl_vec.wz = dr16.data.ch_l_x;
          
      armpos_cmd.mode = ARMPOS_MODE_ACTIVE;
      armpos_cmd.dm_delta = 0.0f;
      armpos_cmd.lz_delta = 0.0f;
      armpos_cmd.rm_delta = 0.0f;

    } else if (dr16.data.sw_l == DR16_SW_DOWN) {
      chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
      chassis_cmd.ctrl_vec.vx = 0;
      chassis_cmd.ctrl_vec.vy = 0;
      chassis_cmd.ctrl_vec.wz = 0;

      armpos_cmd.mode = ARMPOS_MODE_ACTIVE;
      armpos_cmd.dm_delta = dr16.data.ch_l_y;
      armpos_cmd.lz_delta = dr16.data.ch_r_y;
      armpos_cmd.rm_delta = dr16.data.ch_l_x;
    } else {
      chassis_cmd.mode = CHASSIS_MODE_RELAX;
      chassis_cmd.ctrl_vec.vx = 0.0f;
      chassis_cmd.ctrl_vec.vy = 0.0f;
      chassis_cmd.ctrl_vec.wz = 0.0f;
    }

    osMessageQueueReset(task_runtime.msgq.chassis.cmd);
    osMessageQueuePut(task_runtime.msgq.chassis.cmd, &chassis_cmd, 0, 0);
    osMessageQueueReset(task_runtime.msgq.armpos.cmd);
    osMessageQueuePut(task_runtime.msgq.armpos.cmd, &armpos_cmd, 0, 0);

        /* 检测左拨杆切换到UP位置时触发软件复位 */
    if (dr16.header.online) {
      /* 拨杆从非UP状态切换到UP状态，且复位功能已使能，触发系统复位 */
      if (
         dr16.data.sw_l == DR16_SW_UP &&
          dr16.data.sw_r == DR16_SW_DOWN &&
          last_sw_r != DR16_SW_DOWN && last_sw_r != DR16_SW_ERR) {
          reset=!reset; 
      }
      last_sw_l = dr16.data.sw_l;  /* 更新拨杆状态 */
      last_sw_r = dr16.data.sw_r;  /* 更新右拨杆状态 */
    }
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}