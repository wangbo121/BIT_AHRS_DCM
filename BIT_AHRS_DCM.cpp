/*
 * BIT_AHRS_DCM.cpp
 *
 *  Created on: 2017-7-24
 *      Author: wangbo
 */

#include <iostream>
#include <cmath>

#include "BIT_AHRS_DCM.h"
#include "Matrix.h"

using namespace std;

void BIT_AHRS_DCM::get_gyro(float gyro_x, float gyro_y, float gyro_z)
{
	input.ina_gyro0 = gyro_x;
	input.ina_gyro1 = gyro_y;
	input.ina_gyro2 = gyro_z;

	_gyro_vector[0] = - (M_PI/180)*input.ina_gyro0;
	_gyro_vector[1] = - (M_PI/180)*input.ina_gyro1;
	_gyro_vector[2] = - (M_PI/180)*input.ina_gyro2;
}

void BIT_AHRS_DCM::get_accel(float accel_x, float accel_y, float accel_z)
{
	input.ina_acc0 = accel_x;
	input.ina_acc1 = accel_y;
	input.ina_acc2 = accel_z;

	_accel_vector[0] = - input.ina_acc0;
	_accel_vector[1] = - input.ina_acc1;
	_accel_vector[2] = - input.ina_acc2;
}

void BIT_AHRS_DCM::get_gps(float gps_course_rad, float gps_speed)
{
	input.gps_course_angle_rad=gps_course_rad;
	input.gps_speed=gps_speed;
}

/*
 * 向心加速度补偿
 *
 */
void BIT_AHRS_DCM::accel_adjust()
{
	float veloc[3],temp[3];
	veloc[0] = input.gps_speed/3.6;//公里每小时--->米每秒

	temp[0]=0;
	temp[1]=_omega_integ_corr[2]*veloc[0];
	temp[2]=-1.0*_omega_integ_corr[1]*veloc[0];

	vector_sub(_accel_vector,temp,_accel_vector);
}

/*
 * 更新姿态矩阵
 */
void BIT_AHRS_DCM::matrix_update()
{
	float tmp=0.0;
	float update_matrix[3][3];
	float temp_matrix[3][3];

	vector_assign(_omega_integ_corr,_gyro_vector);

	if(fabs(_gyro_vector[0])>=300*(M_PI/180)||fabs(_gyro_vector[1])>=300*(M_PI/180)||fabs(_gyro_vector[2])>=300*(M_PI/180))
	{
		gyro_sat_count++;
	}

	vector_add(_gyro_vector,_omega_I,_omega_integ_corr);
	vector_add(_omega_integ_corr,_omega_P,_omega);

	if(_centripetal)
	{
		accel_adjust();
	}

  #if 1
	  tmp = _G_Dt*_omega[0];
	  update_matrix[1][2]  = -tmp;
	  update_matrix[2][1]  = tmp;

	  tmp = _G_Dt*_omega[1];
	  update_matrix[2][0]  = -tmp;
	  update_matrix[0][2]  = tmp;

	  tmp = _G_Dt*_omega[2];
	  update_matrix[1][0]  = tmp;
	  update_matrix[0][1]  = -tmp;

	  update_matrix[0][0] = 0;
	  update_matrix[1][1] = 0;
	  update_matrix[2][2] = 0;
  #else		//不经过漂移补偿时就直接计算方向余弦阵了
	  update_matrix[0][0] = 0;
	  update_matrix[0][1] = -_G_Dt*_gyro_vector[2];
	  update_matrix[0][2] =  _G_Dt*_gyro_vector[1];
	  update_matrix[1][0] =  _G_Dt*_gyro_vector[2];
	  update_matrix[1][1] = 0;
	  update_matrix[1][2] = -_G_Dt*_gyro_vector[0];
	  update_matrix[2][0] = -_G_Dt*_gyro_vector[1];
	  update_matrix[2][1] =  _G_Dt*_gyro_vector[0];
	  update_matrix[2][2] = 0;
  #endif
	  matrix_dot(_dcm_matrix,update_matrix,temp_matrix);
	  matrix_add(_dcm_matrix,temp_matrix,_dcm_matrix);
}

/*
 * 向量的正交化
 */
void BIT_AHRS_DCM::normalize()
{
	float error=0;
	float temporary0[3],temporary1[3],temporary2[3],temp0[3],temp1[3];

	error=vector_dot(&_dcm_matrix[0][0],&_dcm_matrix[1][0]);

	vector_assign(&temporary0[0],&_dcm_matrix[1][0]);
	vector_assign(&temporary1[0],&_dcm_matrix[0][0]);

	number_mul_vector(0.5*error,&temporary0[0],&temp0[0]);
	vector_sub(&_dcm_matrix[0][0],&temp0[0],&temporary0[0]);
	number_mul_vector(0.5*error,&temporary1[0],&temp1[0]);
	vector_sub(&_dcm_matrix[1][0],&temp1[0],&temporary1[0]);

	vector_cross(temporary0,temporary1,temporary2);

	renorm(temporary0,&_dcm_matrix[0][0]);
	renorm(temporary1,&_dcm_matrix[1][0]);
	renorm(temporary2,&_dcm_matrix[2][0]);

	if(renorm_has_problem)
	{
	  _dcm_matrix[0][0]=1.0;
	  _dcm_matrix[0][1]=0.0;
	  _dcm_matrix[0][2]=0.0;
	  _dcm_matrix[1][0]=0.0;
	  _dcm_matrix[1][1]=1.0;
	  _dcm_matrix[1][2]=0.0;
	  _dcm_matrix[2][0]=0.0;
	  _dcm_matrix[2][1]=0.0;
	  _dcm_matrix[2][2]=1.0;
	}
}

/*
 * 向量的归一化
 */
void BIT_AHRS_DCM::renorm(float *a,float *out_vector)
{
	float renorm_val;

	renorm_val=vector_dot(a,a);
	if(renorm_val<1.5625&&renorm_val>0.64){ renorm_val=0.5*(3-renorm_val); }
	else if(renorm_val<100.0&&renorm_val>=1.5625)
	{
		renorm_val=1.0/sqrt(renorm_val);
		renorm_sqrt_count++;
	}
	else
	{
		renorm_has_problem=true;
		renorm_blowup_count++;
	}

	number_mul_vector(renorm_val,a,out_vector);
}

/*
 * 陀螺仪补偿
 */
void BIT_AHRS_DCM::drift_correction()  //刘
{
	float error_course,_course_over_ground_x,_course_over_ground_y;
	float accel_magnitude;
	float accel_weight;
	float integrator_magnitude;
	float _omega_I_temp;

	float temporary_I[3],temp_P[3],temp_I[3];
	float _error_roll_pitch[3],_error_yaw[3];

	/* 加速度计对俯仰滚转的补偿 */
	accel_magnitude=vector_length(_accel_vector)/9.80665;

	accel_weight=constrain(1-3*fabs(1-accel_magnitude),0,1);

	constrain((0.02*(accel_weight-0.5)),0,1);

	vector_cross(&_dcm_matrix[2][0],_accel_vector,_error_roll_pitch);

	_error_roll_pitch[0]=constrain(_error_roll_pitch[0],-1.17f,1.17f);
	_error_roll_pitch[1]=constrain(_error_roll_pitch[1],-1.17f,1.17f);
	_error_roll_pitch[2]=constrain(_error_roll_pitch[2],-1.17f,1.17f);

	number_mul_vector(_kp_roll_pitch*accel_weight,_error_roll_pitch,_omega_P);
	number_mul_vector(_ki_roll_pitch*accel_weight,_error_roll_pitch,temporary_I);
	vector_add(_omega_I,temporary_I,_omega_I);

	/* GPS对偏航角的补偿 */
	if(input.gps_speed/3.6>=SPEEDFILT) //速度大于3km/h时，认为gps速度有效，进行补偿
	{
		_course_over_ground_x=cos(input.gps_course_angle_rad);
		_course_over_ground_y=sin(input.gps_course_angle_rad);

		error_course=(_dcm_matrix[0][0]*_course_over_ground_y)-(_dcm_matrix[1][0]*_course_over_ground_x);
	}
	else
	{
		error_course=0;
	}

	number_mul_vector(error_course,&_dcm_matrix[2][0],_error_yaw);

	number_mul_vector(_kp_yaw,_error_yaw,temp_P);
	vector_add(temp_P,_omega_P,_omega_P);
	number_mul_vector(_ki_yaw,_error_yaw,temp_I);
	vector_add(temp_I,_omega_I,_omega_I);

	integrator_magnitude=vector_length(_omega_I);	//对积分量进行限制
	if(integrator_magnitude>300*(M_PI/180))
	{
		_omega_I_temp=0.5* (300*M_PI/180) / integrator_magnitude;
		number_mul_vector(_omega_I_temp,_omega_I,_omega_I);
	}
}

/*
 * 计算姿态角
 */
void BIT_AHRS_DCM::euler_angles()
{
	pitch =-asin(_dcm_matrix[2][0]);
	roll  =atan2(_dcm_matrix[2][1],_dcm_matrix[2][2]);
	yaw   =atan2(_dcm_matrix[1][0],_dcm_matrix[0][0]);

	output.roll_deg =roll*(180/M_PI);
	output.pitch_deg=pitch*(180/M_PI);
	output.yaw_deg  =yaw*(180/M_PI);
}

float BIT_AHRS_DCM::constrain(float m,float a,float b)
{
	if(m<=a)        m=a;
	else if(m>=b)   m=b;

	return m;
}

void BIT_AHRS_DCM::update_attitude(void)
{
#if 1
	get_gyro(1,2,3);
	get_accel(4,5,6);
	get_gps(2,5);

	matrix_update();
	normalize();
	drift_correction();
	euler_angles();

	cout<<"roll deg=:"<<output.roll_deg<<endl;
	cout<<"pitch deg=:"<<output.pitch_deg<<endl;
	cout<<"yaw deg=:"<<output.yaw_deg<<endl;

#else
	std::cout<<"wangbo"<<std::endl;
#endif
}
