/*
 * EntryCPP.cpp
 *
 *  Created on: Nov 21, 2021
 *      Author: Mateusz
 */
#include "EntryCPP.hpp"
#include "RobotSpecificDefines.hpp"
#include "VescUart.h"
#ifdef __cplusplus
extern "C" {
#endif

VescUart Vesc();

void main_cpp(void)
{

	while(1)
	{

	}
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
}

#ifdef __cplusplus
}
#endif
