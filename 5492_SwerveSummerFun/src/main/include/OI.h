/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/XboxController.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/WPILib.h>
class OI {
 public:
  OI(); 
	double ReturnDriverStrafeXAxis();
	double ReturnDriverStrafeYAxis();
	double ReturnDriverTurnXAxis();
	bool ReturnDriverRightBumper();
	bool ReturnDriverLeftBumper();
	bool ReturnDriverAButton();
	bool ReturnDriverRightTrigger();
	frc::JoystickButton* manButtonX;
	frc::JoystickButton* manButtonY;
	frc::JoystickButton* manButtonB;
	frc::JoystickButton* manButtonA;
	frc::JoystickButton* manSelect;
	frc::JoystickButton* manStart;
	double ReturnManualLeftYAxis();
	double ReturnManualRightYAxis();
	bool ReturnDriverYButton();
	void OIInit();
	void SetCamMode(int mode);
	void SetStreamMode();
	double DeadBand(double joystick);

	bool ReturnDriverXButton();
	double visionData[3] = {0.0,0.0,0.0};
	
	bool* ReturnLightSensors();
	bool lightData[2] = {false,false}; 
	bool ReturnManualXButton();
	bool ReturnCargoClawLimitSwitch();
	
	bool ReturnDriverBButton();
	void SwapLedMode(int mode);
	int streamMode = 2;
	double targetOffsetAngle_Horizontal;
 	 double targetOffsetAngle_Vertical;
 	 double targetArea;
  	double targetSkew;
	nt::NetworkTableEntry ledmode;
	std::shared_ptr<NetworkTable> table;
	double* ReturnVisionX();
	bool ReturnManualLeftBump();
	bool ReturnManualLeftTrigger();
	bool ReturnManualAButton();
	bool ReturnManualRightBumper();
	bool ReturnManualYButton();
	bool ReturnManualBButton();
	std::vector<double> xCorners;
	std::vector<double> yCorners;
 private:
  frc::XboxController * _driverStick;
	frc::XboxController * _manualStick;
	frc::DigitalInput* LightSensorZero;
	frc::DigitalInput* LightSensorOne;
};
