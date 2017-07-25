#include <stdio.h>
#include <math.h>
#include <string.h>

#include "BIT_AHRS_DCM.h"

int main()
{
	BIT_AHRS_DCM ahrs;

	printf("hello BIT_AHRS_DCM\n");
	ahrs.update_attitude();


	return 0;
}












