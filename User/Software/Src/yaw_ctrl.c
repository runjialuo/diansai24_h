#include "yaw_ctrl.h"
#include "attitude.h"
#include "Software_Config.h"

/* delta = Kp·e + ΣKi·e − Kd·gz
   D 项用陀螺而不是误差差分：de/dt = −gz，噪声小、无延迟 */

static float m_target_deg=0.0f;
static float m_integ_pct=0.0f;
static float m_delta_pct=0.0f;   /* 最近一次输出，OLED 显示用 */

void YawCtrl_Init(void){
    m_target_deg=0.0f;
    m_integ_pct=0.0f;
}

void YawCtrl_Reset(){
    m_integ_pct=0.0f;
    m_target_deg=0.0f;
    Attitude_ResetYaw();
}

void YawCtrl_SetTargetDeg(float deg){
    m_target_deg=deg;
}

float YawCtrl_Updata10ms(void){
    float e_deg;
    float delta_pct;    /*默认顺时针为正方向*/

    e_deg=m_target_deg-Attitude_GetYaw();

    if((e_deg>-YAW_DEADBAND_DEG)&&(e_deg<YAW_DEADBAND_DEG))  e_deg=0.0f;

    m_integ_pct+=YAW_KI*e_deg;
    if(m_integ_pct>YAW_IMAX)  m_integ_pct=YAW_IMAX;
    if(m_integ_pct<-YAW_IMAX) m_integ_pct=-YAW_IMAX;

    /*核心pid公式*/
    delta_pct=(YAW_KP*e_deg)+m_integ_pct-(YAW_KD*Attitude_GetGz());

    if(delta_pct>YAW_OUTMAX)    delta_pct=YAW_OUTMAX;
    if(delta_pct<-YAW_OUTMAX)   delta_pct=-YAW_OUTMAX;

    m_delta_pct = delta_pct;      /* 存一份，供 OLED 显示 */

    return delta_pct;
}

float YawCtrl_GetTargetDeg(void){
    return m_target_deg;
}

float YawCtrl_GetDelta(void){
    return m_delta_pct;
}


