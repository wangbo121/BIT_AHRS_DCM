#include <iostream>

#include "BIT_AHRS_DCM.h"

int main()
{
	BIT_AHRS_DCM ahrs;

	std::cout<<"hello BIT_AHRS_DCM"<<std::endl;
	ahrs.update_attitude();

	return 0;
}












