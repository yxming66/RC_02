/*
    atti_esti Task
    
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "bsp/uart.h"
#include "bsp/fdcan.h"
#include "bsp/time.h"
#include "bsp/mm.h"
#include "bsp/pwm.h"
#include "bsp/gpio.h"
#include "component/ahrs.h"
#include "component/crc16.h"
#include "component/pid.h"
#include "module/chassis.h"
#include "device/buzzer.h"
#include "device/bmi088.h"
#include "module/config.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
#define ATTI_EULR_CAN_ID  (0x100u)
#define ATTI_EULR_SCALE   (10000.0f)  /* 0.0001 rad per LSB */
#define ATTI_EULR_CAN_DLC (6u)
#define ATTI_PC_FRAME_HEADER_0 (0xA5u)
#define ATTI_PC_FRAME_HEADER_1 (0x5Au)
#define ATTI_PC_FRAME_TYPE_XYZ (0x01u)
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
typedef struct {
  uint8_t header[2];
  uint8_t type;
  float x;
  float y;
  float z;
  uint16_t crc;
} Atti_PcFrame_t;

BMI088_t bmi088;

AHRS_t chassis_ahrs;
AHRS_Magn_t magn;
AHRS_Eulr_t eulr_chassis;
KPID_t imu_temp_ctrl_pid;

Chassis_IMU_t chassis_to_send;

BMI088_Cali_t cali_bmi088= {
  .gyro_offset = {-0.00147764047f,-0.00273479894f,0.00154074503f},
};

static const KPID_Params_t imu_temp_ctrl_pid_param = {
    .k = 0.1f,
    .p = 0.17f,
    .i = 0.02f,
    .d = 0.02f,
    .i_limit = 1.0f,
    .out_limit = 1.0f,
};

/* 校准相关变量 */
typedef enum {
  CALIB_IDLE,     // 空闲状态
  CALIB_WAITING,  // 按键触发，等待开始
  CALIB_RUNNING,  // 正在校准
  CALIB_DONE      // 校准完成
} CalibState_t;

static CalibState_t calib_state = CALIB_IDLE;
static uint16_t calib_count = 0;
static uint32_t calib_trigger_tick = 0;
static uint32_t calib_beep_tick = 0;
static uint32_t calib_done_start_tick = 0;
static uint8_t calib_done_beep_count = 0;
static uint32_t calib_done_beep_tick = 0;
static bool calib_beep_on = false;
static bool calib_buzzer_busy = false;
static volatile bool calib_request_pending = false;
static struct {
  float x_sum;
  float y_sum; 
  float z_sum;
} gyro_sum= {0.0f,0.0f,0.0f};
static BUZZER_t calib_buzzer;

bool g_buzzer_calib_active = false;

static void start_gyro_calibration(void) {
  calib_request_pending = true;
}
#define CALIB_SAMPLES 5000  // 校准采样数量
#define CALIB_DELAY_MS 2000u
#define CALIB_BEEP_FREQ 800.63f
#define CALIB_BEEP_DUTY 0.50f
#define CALIB_BEEP_ON_MS 120u
#define CALIB_BEEP_OFF_MS 120u
#define CALIB_DONE_BEEP_FREQ 1200.0f
#define CALIB_DONE_BEEP_DUTY 0.50f
#define CALIB_DONE_GAP_MS 300u
#define CALIB_DONE_BEEP_ON_MS 180u
#define CALIB_DONE_BEEP_OFF_MS 160u

static void reset_gyro_calibration_accumulator(void) {
  calib_count = 0;
  memset(&gyro_sum, 0, sizeof(gyro_sum));
}

static void buzzer_output(bool on, float freq, float duty) {
  if (on) {
    BUZZER_Set(&calib_buzzer, freq, duty);
    BUZZER_Start(&calib_buzzer);
  } else {
    BUZZER_Stop(&calib_buzzer);
  }
}

static void request_gyro_calibration(uint32_t now_tick) {
  if (calib_state == CALIB_IDLE || calib_state == CALIB_DONE) {
    calib_state = CALIB_WAITING;
    calib_trigger_tick = now_tick;
    calib_beep_tick = now_tick;
    calib_done_beep_tick = now_tick;
    calib_done_beep_count = 0;
    calib_beep_on = false;
    calib_buzzer_busy = true;
    g_buzzer_calib_active = true;
    BUZZER_Stop(&calib_buzzer);
  }
}

static void process_calibration_buzzer(uint32_t now_tick) {
  if (calib_state == CALIB_RUNNING) {
    uint32_t interval = calib_beep_on ? CALIB_BEEP_ON_MS : CALIB_BEEP_OFF_MS;
    if ((now_tick - calib_beep_tick) >= interval) {
      calib_beep_on = !calib_beep_on;
      calib_beep_tick = now_tick;
      buzzer_output(calib_beep_on, CALIB_BEEP_FREQ, CALIB_BEEP_DUTY);
    }
    return;
  }

  if (calib_state == CALIB_DONE) {
    if ((now_tick - calib_done_start_tick) < CALIB_DONE_GAP_MS) {
      buzzer_output(false, CALIB_DONE_BEEP_FREQ, CALIB_DONE_BEEP_DUTY);
      return;
    }

    if (calib_done_beep_count >= 3u) {
      buzzer_output(false, CALIB_DONE_BEEP_FREQ, CALIB_DONE_BEEP_DUTY);
      calib_beep_on = false;
      calib_state = CALIB_IDLE;
      calib_buzzer_busy = false;
      g_buzzer_calib_active = false;
      return;
    }

    uint32_t interval = calib_beep_on ? CALIB_DONE_BEEP_ON_MS : CALIB_DONE_BEEP_OFF_MS;
    if ((now_tick - calib_done_beep_tick) >= interval) {
      calib_beep_on = !calib_beep_on;
      calib_done_beep_tick = now_tick;
      buzzer_output(calib_beep_on, CALIB_DONE_BEEP_FREQ, CALIB_DONE_BEEP_DUTY);
      if (calib_beep_on) {
        calib_done_beep_count++;
      }
    }
    return;
  }

  if (calib_beep_on) {
    calib_beep_on = false;
    buzzer_output(false, CALIB_BEEP_FREQ, CALIB_BEEP_DUTY);
  }
}

/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
static void Atti_UpdateEulrFromCan(void);
static void Atti_SendXYZToPc(float x, float y, float z);
/* Exported functions ------------------------------------------------------- */
void Task_atti_esti(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  
  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / ATTI_ESTI_FREQ;

  osDelay(ATTI_ESTI_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */
  bool fdcan_ready = false;
  const int8_t fdcan_init_ret = BSP_FDCAN_Init();
  fdcan_ready = (fdcan_init_ret == BSP_OK || fdcan_init_ret == BSP_ERR_INITED);
  if (fdcan_ready) {
    (void)BSP_FDCAN_RegisterId(BSP_FDCAN_2, ATTI_EULR_CAN_ID, 3);
  }

  BMI088_Init(&bmi088,&cali_bmi088);
  AHRS_Init(&chassis_ahrs, &magn, BMI088_GetUpdateFreq(&bmi088));
  PID_Init(&imu_temp_ctrl_pid, KPID_MODE_NO_D,
           1.0f / BMI088_GetUpdateFreq(&bmi088), &imu_temp_ctrl_pid_param);
  BSP_PWM_Start(BSP_PWM_IMU_HEAT);
  BUZZER_Init(&calib_buzzer, BSP_PWM_BUZZER);
  BUZZER_Stop(&calib_buzzer);

  /* 注册按钮回调函数并启用中断 */
  BSP_GPIO_RegisterCallback(BSP_GPIO_USER_KEY, start_gyro_calibration);
  BSP_GPIO_EnableIRQ(BSP_GPIO_USER_KEY);
  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
    uint32_t now_tick = BSP_TIME_Get_ms();

    if (calib_request_pending) {
      calib_request_pending = false;
      request_gyro_calibration(now_tick);
    }

    if (calib_state == CALIB_WAITING && (now_tick - calib_trigger_tick) >= CALIB_DELAY_MS) {
      reset_gyro_calibration_accumulator();
      calib_state = CALIB_RUNNING;
      calib_beep_tick = now_tick;
      calib_beep_on = true;
      buzzer_output(true, CALIB_BEEP_FREQ, CALIB_BEEP_DUTY);
    }

    process_calibration_buzzer(now_tick);

    BMI088_WaitNew();
    BMI088_AcclStartDmaRecv();
    BMI088_AcclWaitDmaCplt();

    BMI088_GyroStartDmaRecv();
    BMI088_GyroWaitDmaCplt();

    /* 锁住RTOS内核防止数据解析过程中断，造成错误 */
    osKernelLock();
    /* 接收完所有数据后，把数据从原始字节加工成方便计算的数据 */
    BMI088_ParseAccl(&bmi088);
    BMI088_ParseGyro(&bmi088);
    // IST8310_Parse(&ist8310);

    /* 陀螺仪校准处理 */
    if (calib_state == CALIB_RUNNING) {
      /* 累加陀螺仪数据用于计算零偏 */
      gyro_sum.x_sum += bmi088.gyro.x;
      gyro_sum.y_sum += bmi088.gyro.y;
      gyro_sum.z_sum += bmi088.gyro.z;
      calib_count++;

      /* 达到采样数量后计算平均值并更新零偏 */
      if (calib_count >= CALIB_SAMPLES) {
        /* 计算平均值作为零偏 */
        cali_bmi088.gyro_offset.x = gyro_sum.x_sum / CALIB_SAMPLES;
        cali_bmi088.gyro_offset.y = gyro_sum.y_sum / CALIB_SAMPLES;
        cali_bmi088.gyro_offset.z = gyro_sum.z_sum / CALIB_SAMPLES;
        
        /* 校准完成，进入完成提示状态 */
        calib_state = CALIB_DONE;
        calib_done_beep_count = 0;
        calib_done_start_tick = now_tick;
        calib_done_beep_tick = now_tick;
        calib_beep_on = false;
        buzzer_output(false, CALIB_DONE_BEEP_FREQ, CALIB_DONE_BEEP_DUTY);
        
        /* 重新初始化BMI088以应用新的校准数据 */
        BMI088_Init(&bmi088, &cali_bmi088);
        AHRS_Init(&chassis_ahrs, &magn, BMI088_GetUpdateFreq(&bmi088));
      }
    }

    /* 只有在非校准状态下才进行正常的姿态解析 */
    if (calib_state != CALIB_RUNNING) {
      /* 根据设备接收到的数据进行姿态解析 */
      AHRS_Update(&chassis_ahrs, &bmi088.accl, &bmi088.gyro, &magn);

      /* 根据解析出来的四元数计算欧拉角 */
      AHRS_GetEulr(&eulr_chassis, &chassis_ahrs);
    }
    
    osKernelUnlock();



    /* 创建修改后的数据副本用于发送到消息队列 */


    chassis_to_send.eulr = eulr_chassis;
    chassis_to_send.gyro = bmi088.gyro;
    osMessageQueueReset(task_runtime.msgq.chassis.imu);
    osMessageQueuePut(task_runtime.msgq.chassis.imu, &chassis_to_send, 0, 0);
    Atti_SendXYZToPc(eulr_chassis.roll, eulr_chassis.pitch, eulr_chassis.yaw);
    BSP_PWM_SetComp(BSP_PWM_IMU_HEAT, PID_Calc(&imu_temp_ctrl_pid, 40.0f, bmi088.temp, 0.0f, 0.0f));
    
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}

static void Atti_SendXYZToPc(float x, float y, float z) {
  static Atti_PcFrame_t tx_frame;

  tx_frame.header[0] = ATTI_PC_FRAME_HEADER_0;
  tx_frame.header[1] = ATTI_PC_FRAME_HEADER_1;
  tx_frame.type = ATTI_PC_FRAME_TYPE_XYZ;
  tx_frame.x = x;
  tx_frame.y = y;
  tx_frame.z = z;
  tx_frame.crc = CRC16_Calc((const uint8_t *)&tx_frame,
                            sizeof(Atti_PcFrame_t) - sizeof(tx_frame.crc),
                            CRC16_INIT);

  (void)BSP_UART_Transmit(BSP_UART_PC, (uint8_t *)&tx_frame, sizeof(tx_frame), false);
}
