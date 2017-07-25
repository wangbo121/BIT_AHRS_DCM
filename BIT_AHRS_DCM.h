/*
 * BIT_AHRS_DCM.h
 *
 *  Created on: 2017-7-24
 *      Author: wangbo
 */

#ifndef BIT_AHRS_DCM_H_
#define BIT_AHRS_DCM_H_

#ifndef __int8_t_defined
# define __int8_t_defined
/* signed. */
typedef signed char		int8_t;
typedef short int		int16_t;
typedef int			int32_t;
/* Unsigned.  */
typedef unsigned char		uint8_t;
typedef unsigned short int	uint16_t;
typedef unsigned int		uint32_t;
#endif

struct T_AHRS_INPUT
{
	float ina_acc0;
	float ina_acc1;		//x?ò￡?μ￥???a?×/??^2
	float ina_acc2;		//y?ò


	float ina_gyro0;
	float ina_gyro1;	//x?á￡?μ￥???a?è/??
	float ina_gyro2;	//y?á


	unsigned char gps_valid;

	float gps_latti;	//GPS?3?è￡?μ￥???a?è
	float gps_longi;	//GPS?-?è
	float gps_high;		//GPS???è￡?μ￥???a?×
	float gps_velN;		//GPS±±?ò?ù?è￡??×/??
	float gps_velE;		//GPS???ò?ù?è
	float gps_velD;		//GPS???ò?ù?è
	float gps_speed;//ground speed [km/h]
	float gps_course_angle_rad;
	unsigned char gps_satelnum;		//?àD?êy
	float gps_pdop;		//PDOP?μ
};

struct T_AHRS_OUTPUT
{
	float roll_rad;
	float pitch_rad;
	float yaw_rad;

	float roll_deg;
	float pitch_deg;
	float yaw_deg;
};

#define _G_Dt          0.02    //20ms处理一次
#define _kp_roll_pitch 0.05967
#define _ki_roll_pitch 0.00001278
#define _kp_yaw        0.8
#define _ki_yaw        0.00004
#define SPEEDFILT      3/3.6   //3km/h

class BIT_AHRS_DCM{
public:
	//刘 姿态更新
	void update_attitude(void);

private:
	/*
	 * 从inertial sensors惯性传感器或者说是惯性测量单元
	 * 获取角速度和加速度
	 */
	//刘 获取陀螺的数据
	void get_gyro(float gyro_x, float gyro_y, float gyro_z);
	//刘 获取加速度计的数据
	void get_accel(float accel_x, float accel_y, float accel_z);
	//获取gps的course angle和地速ground speed
	void get_gps(float gps_course_deg, float gps_speed);
	//刘 向心加速度补偿
	void accel_adjust();

	//刘 更新姿态矩阵
	void matrix_update();
	//刘 向量的正交化
	void normalize();
	//刘 向量的归一化
	void renorm(float *a,float *out_vector);
	//刘 陀螺补偿计算
	void drift_correction();
	//刘 计算姿态角
	void euler_angles();

	//这个限制不作内部函数
	float constrain(float m,float a,float b);

	void set(float &v) { roll = v; }

	T_AHRS_INPUT input;
	T_AHRS_OUTPUT output;

	unsigned char renorm_has_problem=false;

	float roll,yaw,pitch;
	float _gyro_vector[3];
	float _accel_vector[3];
	float _omega[3],_omega_integ_corr[3],_omega_P[3],_omega_I[3];
	float _dcm_matrix[3][3]={{1,0,0},{0,1,0},{0,0,1}}; //姿态矩阵

	int gyro_sat_count;
	int renorm_sqrt_count;
	int renorm_blowup_count;
	unsigned char _centripetal=1;

	float course_angle_rad;
	//float PlaneYaw1;
};

#endif /* BIT_AHRS_DCM_H_ */
