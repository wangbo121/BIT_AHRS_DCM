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
	float ina_acc1;		//x?ò￡?μ￥???a?×/??^2
	float ina_acc2;		//y?ò
	float ina_acc3;		//z?ò
	float ina_gyro1;	//x?á￡?μ￥???a?è/??
	float ina_gyro2;	//y?á
	float ina_gyro3;	//z?á
	float gps_latti;	//GPS?3?è￡?μ￥???a?è
	float gps_longi;	//GPS?-?è
	float gps_high;		//GPS???è￡?μ￥???a?×
	float gps_velN;		//GPS±±?ò?ù?è￡??×/??
	float gps_velE;		//GPS???ò?ù?è
	float gps_velD;		//GPS???ò?ù?è
	float gps_angle;	//GPSo??ò??￡?μ￥???a?è
	unsigned char gps_satelnum;		//?àD?êy
	float gps_pdop;		//PDOP?μ

	unsigned char gps_valid;

	float true_roll;		//μ￥???a?è
	float true_pitch;
	float true_yaw;
	float true_latti;		//INSoíGPS×?o?oóμ??3?è￡?μ￥???a?è
	float true_longi;		//INSoíGPS×?o?oóμ??-?è￡?μ￥???a?è

	int   waittime;			//μè′y′??D?÷??è??è?¨μ?ê±??￡???è?3000￡?μ￥???a??
	int   starttime;		//????μ?ê?á2ê±??￡???è?1200￡?μ￥???a??

	int algtime;			// 对准时间,单位为采样周期,不能小于1 ,至少有一针GPS有效的数据,

	float true_velN;
	float true_velE;
	float true_velD;
	float true_high;

};

struct T_AHRS_OUTPUT
{
	float true_roll;
	float true_pitch;
	float true_yaw;
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
	void get_gyro();
	//刘 获取加速度计的数据
	void get_accel();
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

	char problem=0;  // 调试用的

	float _gyro_vector[3];
	float _accel_vector[3];
	float _omega[3],_omega_integ_corr[3],_omega_P[3],_omega_I[3];

	float _dcm_matrix[3][3]={{1,0,0},{0,1,0},{0,0,1}}; //姿态矩阵

	int   gyro_sat_count,renorm_sqrt_count,renorm_blowup_count;
	float _centripetal=1,_health;


	float roll,yaw,pitch;

	float vels1=3.6;//添加的
	float course_angle_rad;
	float PlaneYaw1;
};

#endif /* BIT_AHRS_DCM_H_ */
