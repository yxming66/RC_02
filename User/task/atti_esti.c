/*
    atti_esti Task
    
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "bsp/fdcan.h"
#include "bsp/mm.h"
#include "bsp/gpio.h"
#include "bsp/pwm.h"
#include "component/ahrs.h"
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
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
BMI088_t bmi088;

AHRS_t chassis_ahrs;
AHRS_Eulr_t eulr_chassis;
KPID_t imu_temp_ctrl_pid;

Chassis_IMU_t chassis_to_send;

BMI088_Cali_t cali_bmi088= {
  .gyro_offset = {-0.00151803065f,0.00283994782f,0.000795091735f},
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
  CALIB_RUNNING,  // 正在校准
  CALIB_DONE      // 校准完成
} CalibState_t;

static CalibState_t calib_state = CALIB_IDLE;
static uint16_t calib_count = 0;
static volatile bool calib_request_pending = false;
static bool calib_delay_pending = false;
static uint32_t calib_delay_deadline = 0u;
static uint16_t calib_buzzer_count = 0;
static uint8_t calib_done_beep_count = 0;
static struct {
  float x_sum;
  float y_sum; 
  float z_sum;
} gyro_sum= {0.0f,0.0f,0.0f};

bool g_buzzer_calib_active = false;

/* 陀螺仪Z轴动态零偏补偿 */
#define GYRO_Z_CALI_THRESHOLD_SPEED  100.0f   /* 电机速度阈值(rpm)，低于此值认为静止 */
#define GYRO_Z_CALI_SAMPLE_COUNT     500     /* 静止采样数量(约1.25s @ 400Hz) */
#define GYRO_Z_CALI_ALPHA            0.02f   /* 零偏更新平滑系数 */

typedef struct {
    float z_sum;
    uint16_t sample_count;
    float z_offset;
    float z_offset_filtered;
} GyroZCalib_t;

static GyroZCalib_t gyro_z_calib = {
    .z_sum = 0.0f,
    .sample_count = 0,
    .z_offset = 0.0f,
    .z_offset_filtered = 0.0f,
};

static bool is_chassis_stable(void) {
    extern float chassis_motor_speed_rpm[4];
    for (int i = 0; i < 4; i++) {
        if (fabsf(chassis_motor_speed_rpm[i]) > GYRO_Z_CALI_THRESHOLD_SPEED) {
            return false;
        }
    }
    return true;
}

static void update_gyro_z_offset(void) {
    if (is_chassis_stable()) {
        gyro_z_calib.z_sum += bmi088.gyro.z;
        gyro_z_calib.sample_count++;

        if (gyro_z_calib.sample_count >= GYRO_Z_CALI_SAMPLE_COUNT) {
            gyro_z_calib.z_offset = gyro_z_calib.z_sum / gyro_z_calib.sample_count;
            gyro_z_calib.z_sum = 0.0f;
            gyro_z_calib.sample_count = 0;
        }
    } else {
        gyro_z_calib.z_sum = 0.0f;
        gyro_z_calib.sample_count = 0;
    }

    /* 低通滤波平滑零偏变化 */
    gyro_z_calib.z_offset_filtered +=
        GYRO_Z_CALI_ALPHA * (gyro_z_calib.z_offset - gyro_z_calib.z_offset_filtered);
}

static void start_gyro_calibration(void) {
  calib_request_pending = true;
}
#define CALIB_SAMPLES 5000  // 校准采样数量
#define CALIB_BEEP_FREQ         1200.0f
#define CALIB_BEEP_DUTY         0.50f
#define CALIB_START_DELAY_MS    3000u
#define CALIB_WAIT_PERIOD       200u
#define CALIB_WAIT_ON           40u
#define CALIB_RUNNING_PERIOD    120u
#define CALIB_RUNNING_ON        60u
#define CALIB_DONE_BEEP_PERIOD  1000u
#define CALIB_DONE_BEEP_ON      120u
#define CALIB_DONE_BEEP_COUNT   3u

static void reset_gyro_calibration_accumulator(void) {
  calib_count = 0;
  memset(&gyro_sum, 0, sizeof(gyro_sum));
}

static void reset_calibration_buzzer_state(void) {
  calib_buzzer_count = 0;
  calib_done_beep_count = 0;
  BUZZER_Stop(&buzzer);
}

static void start_calibration_buzzer_now(void) {
  BUZZER_Set(&buzzer, CALIB_BEEP_FREQ, CALIB_BEEP_DUTY);
  BUZZER_Start(&buzzer);
}

static void request_gyro_calibration(void) {
  if (calib_state == CALIB_IDLE && !calib_delay_pending) {
    reset_calibration_buzzer_state();
    g_buzzer_calib_active = true;
    calib_delay_pending = true;
    calib_delay_deadline = osKernelGetTickCount() + CALIB_START_DELAY_MS;
    start_calibration_buzzer_now();
  }
}

static void process_calibration_buzzer(void) {
  if (calib_delay_pending) {
    calib_buzzer_count++;
    uint16_t phase = calib_buzzer_count % CALIB_WAIT_PERIOD;

    if (phase == 0u) {
      BUZZER_Set(&buzzer, CALIB_BEEP_FREQ, CALIB_BEEP_DUTY);
      BUZZER_Start(&buzzer);
    } else if (phase == CALIB_WAIT_ON) {
      BUZZER_Stop(&buzzer);
    }
    return;
  }

  if (calib_state == CALIB_RUNNING) {
    calib_buzzer_count++;
    uint16_t phase = calib_buzzer_count % CALIB_RUNNING_PERIOD;

    if (phase == 0u) {
      BUZZER_Set(&buzzer, CALIB_BEEP_FREQ, CALIB_BEEP_DUTY);
      BUZZER_Start(&buzzer);
    } else if (phase == CALIB_RUNNING_ON) {
      BUZZER_Stop(&buzzer);
    }
    return;
  }

  if (calib_state == CALIB_DONE) {
    if (calib_done_beep_count >= CALIB_DONE_BEEP_COUNT) {
      BUZZER_Stop(&buzzer);
      g_buzzer_calib_active = false;
      calib_state = CALIB_IDLE;
      calib_buzzer_count = 0;
      return;
    }

    calib_buzzer_count++;
    uint16_t phase = calib_buzzer_count % CALIB_DONE_BEEP_PERIOD;

    if (phase == 0u) {
      BUZZER_Set(&buzzer, CALIB_BEEP_FREQ, CALIB_BEEP_DUTY);
      BUZZER_Start(&buzzer);
      calib_done_beep_count++;
    } else if (phase == CALIB_DONE_BEEP_ON) {
      BUZZER_Stop(&buzzer);
    }
    return;
  }

  BUZZER_Stop(&buzzer);
  calib_buzzer_count = 0;
}

/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_atti_esti(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  
  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / ATTI_ESTI_FREQ;

  osDelay(ATTI_ESTI_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */


  if (BMI088_Init(&bmi088, &cali_bmi088) != DEVICE_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }
  if (AHRS_Init(&chassis_ahrs, NULL, BMI088_GetUpdateFreq(&bmi088)) !=
      DEVICE_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }
  if (PID_Init(&imu_temp_ctrl_pid, KPID_MODE_NO_D,
               1.0f / BMI088_GetUpdateFreq(&bmi088),
               &imu_temp_ctrl_pid_param) != DEVICE_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }
  BSP_PWM_Start(BSP_PWM_IMU_HEAT);

  /* 注册按钮回调函数并启用中断 */
  BSP_GPIO_RegisterCallback(BSP_GPIO_USER_KEY, start_gyro_calibration);
  BSP_GPIO_EnableIRQ(BSP_GPIO_USER_KEY);
  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
    if (calib_request_pending) {
      calib_request_pending = false;
      request_gyro_calibration();
    }

    if (calib_delay_pending) {
      uint32_t now = osKernelGetTickCount();
      if ((int32_t)(now - calib_delay_deadline) >= 0) {
        calib_delay_pending = false;
        reset_gyro_calibration_accumulator();
        reset_calibration_buzzer_state();
        calib_state = CALIB_RUNNING;
        g_buzzer_calib_active = true;
        start_calibration_buzzer_now();
      }
    }

    if (g_buzzer_calib_active) {
      process_calibration_buzzer();
    }

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
        calib_buzzer_count = 0;
        calib_done_beep_count = 0;
        start_calibration_buzzer_now();
        
        /* 只需重置姿态解算即可，BMI088内部已持有cali_bmi088指针 */
        AHRS_Init(&chassis_ahrs, NULL, BMI088_GetUpdateFreq(&bmi088));
      }
    }

    /* 只有在非校准状态下才进行正常的姿态解析 */
    if (calib_state != CALIB_RUNNING) {
      /* 动态更新陀螺仪Z轴零偏 */
      update_gyro_z_offset();

      /* 补偿Z轴零偏后再进行姿态解算 */
      float gyro_z_corrected = bmi088.gyro.z - gyro_z_calib.z_offset_filtered;
      AHRS_Gyro_t gyro_corrected = bmi088.gyro;
      gyro_corrected.z = gyro_z_corrected;

      /* 根据设备接收到的数据进行姿态解析 */
      AHRS_Update(&chassis_ahrs, &bmi088.accl, &gyro_corrected, NULL);

      /* 根据解析出来的四元数计算欧拉角 */
      AHRS_GetEulr(&eulr_chassis, &chassis_ahrs);
    }
    
    osKernelUnlock();



    /* 创建修改后的数据副本用于发送到消息队列 */


    chassis_to_send.eulr = eulr_chassis;
    chassis_to_send.gyro = bmi088.gyro;
    osMessageQueueReset(task_runtime.msgq.chassis.imu);
    osMessageQueuePut(task_runtime.msgq.chassis.imu, &chassis_to_send, 0, 0);
    // Atti_SendXYZToPc(eulr_chassis.roll, eulr_chassis.pitch, eulr_chassis.yaw);
    BSP_PWM_SetComp(BSP_PWM_IMU_HEAT, PID_Calc(&imu_temp_ctrl_pid, 40.0f, bmi088.temp, 0.0f, 0.0f));
    
    /* USER CODE END */
    task_runtime.stack_water_mark.atti_esti = uxTaskGetStackHighWaterMark(NULL);
    task_runtime.heartbeat.atti_esti++;
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}
