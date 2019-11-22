/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
class OpenOneMotor {
public:
	bool Invert = false;
	bool CurrentLimit = true;
	double RampTime = 0;
	int CurrentLimitAmp = 15;
	WPI_TalonSRX * Open (int DeviceNumber);
	OpenOneMotor();
};
