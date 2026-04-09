/*
µ×ÅÌÄ£×é
*/

#include "cmsis_os2.h"
#include <stdlib.h>
#include "bsp/mm.h"
#include "bsp/can.h"
#include "component/ahrs.h"
#include "device/motor_rm.h"
#include "device/motor.h"
#include "module/chassis.h"
#include "config.h"
#include "math.h"
/**
 * @brief µ×ÅÌÐ¡ÍÓÂÝÄ£Ê½Ïà¹Ø²ÎÊý
 */
#define CHASSIS_ROTOR_WZ_MIN 0.6f       //Ð¡ÍÓÂÝ×îÐ¡ËÙ¶È
#define CHASSIS_ROTOR_WZ_MAX 0.8f				//Ð¡ÍÓÂÝ×î´óËÙ¶È
#define M_7OVER72PI (M_2PI * 7.0f / 72.0f)   //½Ç¶ÈÆ«ÒÆÁ¿£¨ÓÃÔÚ¸úËæÔÆÌ¨35¡ã£©
#define CHASSIS_ROTOR_OMEGA 0.001f        //½ÇËÙ¶È±ä»¯ÆµÂÊ



/**
 * @brief ÉèÖÃµ×ÅÌÄ£Ê½
 * @param c µ×ÅÌ½á¹¹ÌåÖ¸Õë
 * @param mode Ä¿±ê¿ØÖÆÄ£Ê½
 * @param now µ±Ç°Ê±¼ä´Á(ms)
 * @return CHASSIS_OK:³É¹¦   CHASSIS_ERR_NULL:¿Õ
 */
static int8_t Chassis_SetMode(Chassis_t *c, Chassis_Mode_t mode, uint32_t now) {
    if (!c) 
			return CHASSIS_ERR_NULL;
    if (mode == c->mode)
			return CHASSIS_OK;
//Ëæ»úÖÖ×Ó£¬Ð¡ÍÓÂÝÄ£Ê½Ëæ»úÉèÖÃÐý×ª·½Ïò
    if (mode == CHASSIS_MODE_ROTOR && c->mode != CHASSIS_MODE_ROTOR) {
        srand(now);
        c->wz_multi = (rand() % 2) ? -1 : 1;
    }
//ÖØÖÃPIDºÍÂË²¨
    for (uint8_t i = 0; i < c->num_wheel; i++) {
        PID_Reset(&c->pid.motor[i]);
        LowPassFilter2p_Reset(&c->filter.in[i], 0.0f);
        LowPassFilter2p_Reset(&c->filter.out[i], 0.0f);
    }

    c->mode = mode;
    return CHASSIS_OK;
}

/**
 * @brief Ð¡ÍÓÂÝÄ£Ê½¶¯Ì¬½ÇËÙ¶È
 * @param min ×îÐ¡ËÙ¶È
 * @param max ×î´óËÙ¶È
 * @param now µ±Ç°Ê±¼ä´Á(ms)
 * @return ½ÇËÙ¶ÈÖµ
 */
static float Chassis_CalcWz(const float min, const float max, uint32_t now) {
    float wz_vary = fabsf(0.2f * sinf(CHASSIS_ROTOR_OMEGA * (float)now)) + min;
    return (wz_vary > max) ? max : wz_vary;
}

/**
 * @brief µ×ÅÌÄ£Ê½³õÊ¼»¯
 * @param c µ×ÅÌ½á¹¹ÌåÖ¸Õë
 * @param param  µ×ÅÌ²ÎÊý½á¹¹ÌåÖ¸Õë
 * @param mech_zero »úÐµÁãµãÅ·À­½Ç
 * @param target_freq ¿ØÖÆÆµÂÊ(Hz)
 * @return CHASSIS_OK:³É¹¦ CHASSIS_ERR_NULL:¿Õ CHASSIS_ERR_TYPE:²»Ö§³ÖµÄÄ£Ê½
 */
int8_t Chassis_Init(Chassis_t *c, const Chassis_Params_t *param,
                    float target_freq) {
    if (!c) return CHASSIS_ERR_NULL;
											
		//³õÊ¼»¯CANÍ¨ÐÅ									
		BSP_CAN_Init();
    c->param = param;
    c->mode = CHASSIS_MODE_RELAX;
//¸ù¾Ýµ×ÅÌ²»Í¬ÉèÖÃÄ£Ê½ÂÖ×ÓÓë»ìºÏÆ÷
    Mixer_Mode_t mixer_mode;
    switch (param->type) {
        case CHASSIS_TYPE_MECANUM://ÂóÂÖ
					c->num_wheel = 4; 
				mixer_mode = MIXER_MECANUM;
				break;
        case CHASSIS_TYPE_PARLFIX4:
					c->num_wheel = 4; 
				mixer_mode = MIXER_PARLFIX4;
				break;
        case CHASSIS_TYPE_PARLFIX2:
					c->num_wheel = 2; 
				mixer_mode = MIXER_PARLFIX2; 
				break;
        case CHASSIS_TYPE_OMNI_CROSS:
					c->num_wheel = 4;
				mixer_mode = MIXER_OMNICROSS;
				break;
        case CHASSIS_TYPE_OMNI_PLUS: //È«ÏòÂÖ£¨ÀÏ²½±øÀàÐÍ£©
					c->num_wheel = 4;
				mixer_mode = MIXER_OMNIPLUS;
				break;
        case CHASSIS_TYPE_SINGLE:
					c->num_wheel = 1; 
				mixer_mode = MIXER_SINGLE;
				break;
        default: 
					return CHASSIS_ERR_TYPE;
    }
		//³õÊ¼»¯Ê±¼ä´Á
				c->last_wakeup = 0;
				c->dt = 0.0f;
		//³õÊ¼»¯PIDºÍÂË²¨
    for (uint8_t i = 0; i < c->num_wheel; i++) {
        PID_Init(&c->pid.motor[i], KPID_MODE_NO_D, target_freq, &param->pid.motor_pid_param);
        LowPassFilter2p_Init(&c->filter.in[i], target_freq, param->low_pass_cutoff_freq.in);
        LowPassFilter2p_Init(&c->filter.out[i], target_freq, param->low_pass_cutoff_freq.out);
			//ÇåÁãµç»ú·´À¡
				c->feedback.motor[i].rotor_speed = 0;
        c->feedback.motor[i].torque_current = 0;
        c->feedback.motor[i].rotor_abs_angle = 0;
        c->feedback.motor[i].temp = 0;
    }
     //³õÊ¼»¯PIDºÍ»ìºÏÆ÷
    PID_Init(&c->pid.follow, KPID_MODE_CALC_D, target_freq, &param->pid.follow_pid_param);
    Mixer_Init(&c->mixer, mixer_mode);
     //ÇåÁãÔË¶¯ÏòÁ¿ºÍÊä³ö
    c->move_vec.vx = c->move_vec.vy = c->move_vec.wz = 0.0f;
		for (uint8_t i = 0; i < c->num_wheel; i++) { 
				c->out.motor[i] = 0.0f;
		}
		//×¢²á´ó½®µç»ú
		for (int i = 0; i < c->num_wheel; i++) {
			MOTOR_RM_Register(&(c->param->motor_param[i]));
			 
		}
    return CHASSIS_OK;
}

/**
 * @brief ¸üÐÂµç»ú·´À¡(IMU+µç»ú×´Ì¬)
 * @param c µ×ÅÌ½á¹¹ÌåÖ¸Õë
 * @param feedback µ×ÅÌ·´À¡Ö¸Õë½á¹¹Ìå
 * @return CHASSIS_OK:³É¹¦ CHASSIS_ERR_NULL:¿Õ
 */
int8_t Chassis_UpdateFeedback(Chassis_t *c) {


		//¸üÐÂËùÓÐµç»ú·´À¡
    for (uint8_t i = 0; i < c->num_wheel; i++) {
			MOTOR_RM_Update(&(c->param->motor_param[i])); 
			MOTOR_RM_t *rm_motor = MOTOR_RM_GetMotor(&(c->param->motor_param[i]));
			c->motors[i] = rm_motor;
			MOTOR_RM_t *rm = c->motors[i];
         if (rm_motor != NULL) {
            c->feedback.motor[i] = rm_motor->feedback;
             }else 
					{ 
					return CHASSIS_ERR_NULL; 
					} 
		}
    return CHASSIS_OK;
}

/**
 * @brief µ×ÅÌµç»ú¿ØÖÆ
 * @param c µ×ÅÌ½á¹¹ÌåÖ¸Õë
 * @param c_cmd ¿ØÖÆÃüÁî
 * @param now µ±Ç°Ê±¼ä´Á(ms)
 * @return CHASSIS_OK:³É¹¦ CHASSIS_ERR_NULL:¿Õ
 */
int8_t Chassis_Control(Chassis_t *c, const Chassis_CMD_t *c_cmd, uint32_t now) {
    if (!c || !c_cmd) return CHASSIS_ERR_NULL;
		//¼ÆËã¿ØÖÆÖÜÆÚ
    c->dt = (float)(now - c->last_wakeup) / 1000.0f; 
    c->last_wakeup = now;
		if (!isfinite(c->dt) || c->dt <= 0.0f) {
				c->dt = 0.001f;            
		}
		if (c->dt < 0.0005f) c->dt = 0.0005f;   
		if (c->dt > 0.050f)  c->dt = 0.050f;    
    //ÉèÖÃÄ£Ê½
    Chassis_SetMode(c, c_cmd->mode, now);
    //²»Í¬Ä£Ê½ÏÂ¶ÔÓ¦½âËã£¨ÔË¶¯ÏòÁ¿£©
    switch (c->mode) {
        case CHASSIS_MODE_BREAK:
            c->move_vec.vx = c->move_vec.vy = 0.0f;
            break;
        case CHASSIS_MODE_INDEPENDENT:
            c->move_vec.vx = c_cmd->ctrl_vec.vx;
            c->move_vec.vy = c_cmd->ctrl_vec.vy;
            break;
        default: {      //Ò£¿ØÆ÷×ø±ê->»úÌå×ø±êÏµ
            float beta = c->feedback.encoder_gimbalYawMotor - c->mech_zero;
            float cosb = cosf(beta);
            float sinb = sinf(beta);
            c->move_vec.vx = cosb * c_cmd->ctrl_vec.vx - sinb * c_cmd->ctrl_vec.vy;
            c->move_vec.vy = sinb * c_cmd->ctrl_vec.vx + cosb * c_cmd->ctrl_vec.vy;
            break;
        }
    }
		//¸ù¾ÝÄ£Ê½¼ÆËãµ×ÅÌ½ÇËÙ¶È
    switch (c->mode) {
        case CHASSIS_MODE_RELAX:
        case CHASSIS_MODE_BREAK:
        case CHASSIS_MODE_INDEPENDENT:
						c->move_vec.wz = c_cmd->ctrl_vec.wz;
            break;
        case CHASSIS_MODE_OPEN:
						c->move_vec.wz = c_cmd->ctrl_vec.wz;
						break;
				//ÔÆÌ¨¸úËæ
        case CHASSIS_MODE_FOLLOW_GIMBAL:
            c->move_vec.wz = PID_Calc(&c->pid.follow, c->mech_zero, c->feedback.encoder_gimbalYawMotor, 0.0f, c->dt);
            break;
				//ÔÆÌ¨¸úËæ£¨Æ«ÒÆ£©
        case CHASSIS_MODE_FOLLOW_GIMBAL_35:
            c->move_vec.wz = PID_Calc(&c->pid.follow,c->mech_zero +M_7OVER72PI, c->feedback.encoder_gimbalYawMotor, 0.0f, c->dt);
            break;
				//Ð¡ÍÓÂÝ
        case CHASSIS_MODE_ROTOR:
            c->move_vec.wz = c->wz_multi * Chassis_CalcWz(CHASSIS_ROTOR_WZ_MIN,CHASSIS_ROTOR_WZ_MAX, now);
            break;
    }
        //ÔË¶¯Ñ§Äæ½âËã£¬ÔË¶¯ÏòÁ¿·Ö½âÎªµç»ú×ªËÙ
    Mixer_Apply(&c->mixer, &c->move_vec, c->setpoint.motor_rpm, c->num_wheel, 500.0f);


    for (uint8_t i = 0; i < c->num_wheel; i++) {
				float rf = c->setpoint.motor_rpm[i];///Ä¿±ê×ªËÙ
        float fb = LowPassFilter2p_Apply(&c->filter.in[i], (float)c->feedback.motor[i].rotor_speed);
				float out_current;
        switch (c->mode) {
            case CHASSIS_MODE_BREAK:
            case CHASSIS_MODE_FOLLOW_GIMBAL:
            case CHASSIS_MODE_FOLLOW_GIMBAL_35:
            case CHASSIS_MODE_ROTOR:
            case CHASSIS_MODE_INDEPENDENT:
                out_current = PID_Calc(&c->pid.motor[i], c->setpoint.motor_rpm[i], fb, 0.0f, c->dt);
                break;
            case CHASSIS_MODE_OPEN:
                out_current = c->setpoint.motor_rpm[i] / 7000.0f;
                break;
            case CHASSIS_MODE_RELAX:
                out_current = 0.0f;
                break;
        }

       //µÍÍ¨ÂË²¨ºÍÏÞ·ù
         c->out.motor[i] = LowPassFilter2p_Apply(&c->filter.out[i], out_current);
				 Clip(&c->out.motor[i], -c->param->limit.max_current, c->param->limit.max_current);
    }

		
    return CHASSIS_OK;
}

/**
 * @brief µç»úÊä³ö
 * @param c µ×ÅÌ½á¹¹ÌåÖ¸Õë
 */
void Chassis_Output(Chassis_t *c) {
    if (!c) 
			return;

    for (uint8_t i = 0; i < c->num_wheel; i++) {
        MOTOR_RM_t *rm = c->motors[i];
        if (!rm) continue;
        MOTOR_RM_SetOutput(&rm->param, c->out.motor[i]);
    }

    //µ÷ÓÃctrl
    // for (uint8_t i = 0; i < c->num_wheel; i++) {
    //     MOTOR_RM_t *rm = c->motors[0];
    //     if (rm) {
    //         MOTOR_RM_Ctrl(&rm->param);
    //     }
    // }

    MOTOR_RM_t *rm = c->motors[0];
    if (rm) {
        MOTOR_RM_Ctrl(&rm->param);
    }
}

/**
 * @brief ÖØÖÃµ×ÅÌÊä³ö
 * @param c  µ×ÅÌ½á¹¹ÌåÖ¸Õë
 */
void Chassis_ResetOutput(Chassis_t *c) {
    if (!c) return;
    for (uint8_t i = 0; i < c->num_wheel; i++) {
        MOTOR_RM_t *m = c->motors[i];
        if (m) {
            MOTOR_RM_Relax(&(m->param));
        }
    }
}

/**
 * @brief 导出底盘数据
 *
 * @param chassis 底盘数据结构体
 * @param ui UI数据结构体
 */
void Chassis_DumpUI(const Chassis_t *c, Chassis_RefereeUI_t *ui) {
  ui->mode = c->mode;
  ui->angle = c->feedback.encoder_gimbalYawMotor - c->mech_zero;
}
