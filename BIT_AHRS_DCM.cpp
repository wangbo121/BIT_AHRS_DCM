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

void BIT_AHRS_DCM::get_gyro()
{
	_gyro_vector[0]=-(M_PI/180)*input.ina_gyro2;
	_gyro_vector[1]=-(M_PI/180)*input.ina_gyro1;
	_gyro_vector[2]=-(M_PI/180)*input.ina_gyro3;

}
void BIT_AHRS_DCM::get_accel()
{
	_accel_vector[0]=-input.ina_acc2;
	_accel_vector[1]=-input.ina_acc1;
	_accel_vector[2]=-input.ina_acc3;

}
/*
 * 向心加速度补偿
 *
 */
void BIT_AHRS_DCM::accel_adjust()
{
	float veloc[3],temp[3];
	veloc[0] = vels1/3.6;//公里每小时--->米每秒

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
	{gyro_sat_count++;}  //刘 到多少饱和，饱和后怎么处理？

	vector_add(_gyro_vector,_omega_I,_omega_integ_corr);
	vector_add(_omega_integ_corr,_omega_P,_omega);

	if(_centripetal){accel_adjust();} //这块的条件什么时候满足？？

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
  #else    //刘 不经过漂移补偿时就直接计算方向余弦阵了
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

	if(problem)
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
		renorm_sqrt_count++;    //这个什么时候用？
	}
	else
	{
		problem=1;
		renorm_blowup_count++; //这个什么时候用？
	}

	number_mul_vector(renorm_val,a,out_vector);

}

/*
 * 陀螺补偿计算
 */
void BIT_AHRS_DCM::drift_correction()  //刘
{
	float error_course,_course_over_ground_x,_course_over_ground_y;
	float accel_magnitude;
	float accel_weight;
	float integrator_magnitude;
	float _omega_I_temp;


	static char in_motion=0;
	float rot_mat[3][3];
	float temporary_I[3],temp_P[3],temp_I[3];
	float _error_roll_pitch[3],_error_yaw[3];


	/********加计对俯仰滚转的补偿**********/
	accel_magnitude=vector_length(_accel_vector)/9.80665;

	accel_weight=constrain(1-3*fabs(1-accel_magnitude),0,1);

	_health=constrain(_health+(0.02*(accel_weight-0.5)),0,1);  //刘 这个什么时候用，判断加速度计的量是否正常？？

	vector_cross(&_dcm_matrix[2][0],_accel_vector,_error_roll_pitch);

	_error_roll_pitch[0]=constrain(_error_roll_pitch[0],-1.17f,1.17f);
	_error_roll_pitch[1]=constrain(_error_roll_pitch[1],-1.17f,1.17f);
	_error_roll_pitch[2]=constrain(_error_roll_pitch[2],-1.17f,1.17f);

	number_mul_vector(_kp_roll_pitch*accel_weight,_error_roll_pitch,_omega_P);
	number_mul_vector(_ki_roll_pitch*accel_weight,_error_roll_pitch,temporary_I);
	vector_add(_omega_I,temporary_I,_omega_I);


	/********GPS对偏航角的补偿********/


	if(vels1/3.6>=SPEEDFILT) //刘 速度大于3km/h时才进行补偿
	//if(0)
	{
		_course_over_ground_x=cos(PlaneYaw1);
		_course_over_ground_y=sin(PlaneYaw1);
		//if(in_motion)
		//{
			error_course=(_dcm_matrix[0][0]*_course_over_ground_y)-(_dcm_matrix[1][0]*_course_over_ground_x);
		//}
		/*
		else  //刘 刚开始运动或者复位方向余弦阵的时候，下面的算可以快速跟踪GPS的偏航角
		{
			float cos_psi_err,sin_psi_err,yaw;

			yaw=atan2(_dcm_matrix[1][0],_dcm_matrix[0][0]);
			cos_psi_err=cos(PlaneYaw1-yaw);
			sin_psi_err=sin(PlaneYaw1-yaw);

			rot_mat[0][0]=cos_psi_err;
			rot_mat[0][1]=-sin_psi_err;
			rot_mat[1][0]=sin_psi_err;
			rot_mat[1][1]=cos_psi_err;
			rot_mat[0][2]=0;
			rot_mat[1][2]=0;
			rot_mat[2][0]=0;
			rot_mat[2][1]=0;
			rot_mat[2][2]=1.0;

			matrix_dot(rot_mat,_dcm_matrix,_dcm_matrix);
			in_motion=1;
			error_course=0;

		}
		*/
	}
	else
	{
		error_course=0;
		in_motion=0;
	}

	number_mul_vector(error_course,&_dcm_matrix[2][0],_error_yaw);

	number_mul_vector(_kp_yaw,_error_yaw,temp_P);
	vector_add(temp_P,_omega_P,_omega_P);
	number_mul_vector(_ki_yaw,_error_yaw,temp_I);
	vector_add(temp_I,_omega_I,_omega_I);

	integrator_magnitude=vector_length(_omega_I);  //刘 对积分量进行限制
	if(integrator_magnitude>300*(M_PI/180))
	{
		_omega_I_temp=0.5* (300*M_PI/180) / integrator_magnitude;
		number_mul_vector(_omega_I_temp,_omega_I,_omega_I);
	}
}

/*
 * 计算姿态角
 */
void BIT_AHRS_DCM::euler_angles() //刘
{
#if(OUTPUTMODE==2)
	roll  =atan2(_accel_vector[1],-_accel_vector[2]);
	pitch =asin((_accel_vector[0])/(double)9.81);
	yaw   =0;
#else
	pitch =-asin(_dcm_matrix[2][0]);
	roll  =atan2(_dcm_matrix[2][1],_dcm_matrix[2][2]);
	yaw   =atan2(_dcm_matrix[1][0],_dcm_matrix[0][0]);
#endif

	output.true_roll =roll*(180/M_PI);          //刘 输出的是 度
	output.true_pitch=pitch*(180/M_PI);
	output.true_yaw  =yaw*(180/M_PI);


}

void BIT_AHRS_DCM::update_attitude(void) //刘 姿态更新
{
#if 1
    //PlaneYaw1=0; //刘 用于静态模式的测试
    //vels1=0;

	get_gyro();
	get_accel();

	matrix_update();

	normalize();

	drift_correction();

	euler_angles();
#else
	std::cout<<"wangbo"<<std::endl;
#endif
}

float BIT_AHRS_DCM::constrain(float m,float a,float b)
{
	if(m<=a)        m=a;
	else if(m>=b)   m=b;

	return m;
}


