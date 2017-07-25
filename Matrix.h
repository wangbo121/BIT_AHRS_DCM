#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <stdio.h>
#include <math.h>

/* operating for vector */
//test for equality
char eauality_check(float *vector1,float *vector2)
{
	if((*vector1)==(*vector2)&&(*(vector1+1))==(*(vector2+1))&&(*(vector1+2))==(*(vector2+2))) return 1;
	else return 0;
}

//test for inequality
char ineauality_check(float *vector1,float *vector2)
{
	if((*vector1)!=(*vector2)||(*(vector1+1))!=(*(vector2+1))||(*(vector1+2))!=(*(vector2+2))) return 1;
	else return 0;
}

//vector_assign
void vector_assign(float *vector1,float *vector2)
{
	int i;
	for(i=0;i<3;i++)
	{
		*(vector1+i)=*(vector2+i);
	}
}

//anti_vector
void anti_vector(float *vector1,float *vector2,float *out_vector)
{
	int i;
	for(i=0;i<3;i++)
	{
		*(out_vector+i)=-*(vector1+i);
	}
} 

//add
void vector_add(float *vector1,float *vector2,float *out_vector)
{
	int i;
	for(i=0;i<3;i++)
	{
		*(out_vector+i)=*(vector1+i)+*(vector2+i);
	}

} 

//substract
void vector_sub(float *vector1,float *vector2,float *out_vector)
{
	int i;
	for(i=0;i<3;i++)
	{
		*(out_vector+i)=*(vector1+i)-*(vector2+i);
	}
} 

//dot product
float vector_dot(float *vector1,float *vector2)
{
	float dot_value;
  dot_value=(*vector1)*(*vector2)+(*(vector1+1))*(*(vector2+1))+(*(vector1+2))*(*(vector2+2));
	return dot_value;
}

//cross product
void vector_cross(float *vector1,float *vector2,float *out_vector)
{
	*out_vector    =(*(vector1+1))*(*(vector2+2))-(*(vector1+2))*(*(vector2+1));
	*(out_vector+1)=(*(vector1+2))*(*vector2)    -(*vector1)*(*(vector2+2));
	*(out_vector+2)= *(vector1)   *(*(vector2+1))-(*(vector1+1))*(*vector2);
}

//number multiplication
void number_mul_vector(float m,float *vector1,float *out_vector)
{
	int i;
	for(i=0;i<3;i++)
	{
		*(out_vector+i)=m*(*(vector1+i));
	}
}

//length of the vector
double vector_length(float *vector1)
{
	return sqrt((*vector1)*(*vector1)+(*(vector1+1))*(*(vector1+1))+(*(vector1+2))*(*(vector1+2)));
}

//normalize of the vector
void normalize(float *vector1,float *out_vector)
{
	int i;
	for(i=0;i<3;i++)
	{
		*(out_vector+i)=(float)((*(vector1+i))/vector_length(&vector1[0]));
	}
}

/* operating for matrix */
//matrix add
void matrix_add(float matrix1[3][3],float matrix2[3][3],float out_matrix[3][3])
{
	int i,j;
	for(i=0;i<3;i++)
	{
		 for(j=0;j<3;j++)
	   {
		    out_matrix[i][j]=matrix1[i][j]+matrix2[i][j];
	   }
	}  
}

//matrix dot
void matrix_dot(float matrix1[3][3],float matrix2[3][3],float out_matrix[3][3])
{
	out_matrix[0][0] = matrix1[0][0]*matrix2[0][0]+matrix1[0][1]*matrix2[1][0]+matrix1[0][2]*matrix2[2][0];
	out_matrix[0][1] = matrix1[0][0]*matrix2[0][1]+matrix1[0][1]*matrix2[1][1]+matrix1[0][2]*matrix2[2][1];
  out_matrix[0][2] = matrix1[0][0]*matrix2[0][2]+matrix1[0][1]*matrix2[1][2]+matrix1[0][2]*matrix2[2][2];
  
  out_matrix[1][0] = matrix1[1][0]*matrix2[0][0]+matrix1[1][1]*matrix2[1][0]+matrix1[1][2]*matrix2[2][0];
  out_matrix[1][1] = matrix1[1][0]*matrix2[0][1]+matrix1[1][1]*matrix2[1][1]+matrix1[1][2]*matrix2[2][1];
  out_matrix[1][2] = matrix1[1][0]*matrix2[0][2]+matrix1[1][1]*matrix2[1][2]+matrix1[1][2]*matrix2[2][2];
  
  out_matrix[2][0] = matrix1[2][0]*matrix2[0][0]+matrix1[2][1]*matrix2[1][0]+matrix1[2][2]*matrix2[2][0];
  out_matrix[2][1] = matrix1[2][0]*matrix2[0][1]+matrix1[2][1]*matrix2[1][1]+matrix1[2][2]*matrix2[2][1];
  out_matrix[2][2] = matrix1[2][0]*matrix2[0][2]+matrix1[2][1]*matrix2[1][2]+matrix1[2][2]*matrix2[2][2];
}

#endif 
