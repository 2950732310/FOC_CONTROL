#ifndef FOC_H
#define FOC_H
#include "main.h"


// 基础配置
#define BATVEL 				12.0f    										// 供电电压 V
#define INVBATVEL 		(1.0f / BATVEL) 		  			// 供电电压的倒数
#define MAX_V_LIMIT		0.57737 * BATVEL	    			// 驱动电压最大限幅 1/根3 倍的母线电压
#define TS  					1.0f       				          // PWM 周期看作“单位 1”
#define POLE_PAIRS 		11													// 电机极对数

#define TIMER1_CLK_MHz 170                                                   // 定时器时钟频率
#define PWM_FREQUENCY 20000                                                  // PWM频率20KHz
#define PWM_MEASURE_PERIOD (float)(1.0f / (float)PWM_FREQUENCY)              // PWM周期
#define CURRENT_MEASURE_HZ PWM_FREQUENCY                                     // 电流频率
#define CURRENT_MEASURE_PERIOD (float)(1.0f / (float)CURRENT_MEASURE_HZ)     // 电流周期

/* 常用数学计算宏定义 */
#define M_PI   (3.14159265358979323846f)	    	// 圆周率
#define M_2PI (6.28318530717958647692f)         // 2倍圆周率
#define _SQRT3 (1.7320508075688772935f)         // 3的平方根
#define ONE_BY_SQRT3 (0.57735026919f)           // 1/3的平方根
#define TWO_BY_SQRT3 (1.15470053838f)           // 2/3的平方根
#define _SQRT3_2 (0.86602540378443864f)         // (根3)/2


// 扇区枚举
typedef enum
{
    SECTOR_1 = 1,
    SECTOR_2,
    SECTOR_3,
    SECTOR_4,
    SECTOR_5,
    SECTOR_6
} svpwm_sector_t;

// flash 数据结构体
typedef struct
{
	float ia_zero;          // A相电流零点
	float ib_zero;          // B相电流零点
  float id_kp;         // d轴电流环比例系数
  float id_ki;         // d轴电流环积分系数
  float iq_kp;         // q轴电流环比例系数
  float iq_ki;         // q轴电流环积分系数
  float vel_kp;        // 速度环比例系数
  float vel_ki;        // 速度环积分系数
}flash_data_t;
extern flash_data_t flash_data;

// FOC 数据结构体
typedef struct
{   
    /* 以下部分需要存储到flash中 */
    flash_data_t flash_data;

    /* 以下部分不需要存储到flash中 */
    float vbus;             // 母线电压
    float inv_vbus;         // 母线电压的倒数

    // float theta;            // 电角度
		// float theta_offset;			// 偏移电角度
    float sin_val;          // 电角度的正弦值
    float cos_val;          // 电角度的余弦值

    float i_a;              // a相电流
    float i_b;              // b相电流
    float i_c;              // c相电流

    float v_a;              // a相电压
    float v_b;              // b相电压
    float v_c;              // c相电压

    float i_alpha;          // α轴电流
    float i_beta;           // β轴电流

    float v_alpha;          // α轴电压
    float v_beta;           // β轴电压
		
		float Ualpha_norm;			// α轴电压/母线电压
		float Ubeta_norm;				// β轴电压/母线电压
		
    float i_d;              // d轴电流
    float i_q;              // q轴电流
    float i_d_ref;          // d轴电流参考值
    float i_q_ref;          // q轴电流参考值

    float vel_ref;          // 速度参考值 (rad/s)
    float vel_fb;           // 速度反馈值 (rad/s)

    float v_d;              // d轴电压
    float v_q;              // q轴电压
		
		float vq_set;						// d轴电压设置
		float vd_set;						// q轴电压设置
		
    float dtc_a;            // A 相 PWM 占空比
    float dtc_b;            // B 相 PWM 占空比
    float dtc_c;            // C 相 PWM 占空比 

}FOC_DATA;


void Sin_Cos_Value(FOC_DATA *foc , float elec_angle);
float Normalize_Angle(float angle);
float Limit(float x, float low, float high);

void Clarke(FOC_DATA *foc);
void Inv_Clarke(FOC_DATA *foc);
void Park(FOC_DATA *foc);
void Inv_Park(FOC_DATA *foc);

void Svpwm_Sector(FOC_DATA *foc);
int Svpwm(FOC_DATA *foc);

#endif
// FOC_H
