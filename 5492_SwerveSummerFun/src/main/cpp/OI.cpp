/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "RobotMap.h"
#include <vector>

	
	 
OI::OI() {
 	 _driverStick = 0;
	_manualStick = 0;

	
	table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  // Process operator interface input here.
}
void OI::OIInit(){
  	if (_driverStick == 0)
	{
		_driverStick = new frc::XboxController(0);
	}
	if (_manualStick == 0)
	{
		_manualStick = new frc::XboxController(1);
	}
/*	LightSensorZero = new frc::DigitalInput(0);
	LightSensorOne = new frc::DigitalInput(1);*/
	manButtonX = new frc::JoystickButton(_manualStick, 3);
	manButtonY = new frc::JoystickButton(_manualStick, 4);
	manButtonB = new frc::JoystickButton(_manualStick, 2);
	//manButtonA = new frc::JoystickButton(_manualStick, 1);
	/*double habLevel[2] = {armHab, liftHab};
	double lowLevel[2] = {armLow, liftLow}; // this doesn't exist
	double midLevel[2] = {armMid, liftMid};
	double reverseLevel[2] = {-armReverse, liftHab};
	manButtonX->WhileHeld(new CleanAndJerk(habLevel));
	manButtonY->WhileHeld(new CleanAndJerk(midLevel));
	manButtonB->WhileHeld(new CleanAndJerk(reverseLevel));*/
	//manSelect->WhileHeld(new FirmlyGrasp(-manHarpoon));
	//manStart->WhileHeld(new FirmlyGrasp(manHarpoon));
	

}	
bool OI::ReturnDriverAButton() {
	return _driverStick->GetAButtonPressed();
}
bool OI::ReturnDriverLeftBumper() {
	return _driverStick->GetBumper(frc::GenericHID::kLeftHand);
}
bool OI::ReturnDriverRightBumper() {
	return _driverStick->GetBumper(frc::GenericHID::kRightHand);
}
bool OI::ReturnDriverRightTrigger() {
	return (_driverStick->GetTriggerAxis(frc::GenericHID::kLeftHand) > 0.5);
}
bool OI::ReturnDriverBButton() {
	return _driverStick->GetBButtonPressed();
}
bool OI::ReturnDriverYButton(){
	return _driverStick->GetYButtonPressed();
}
double OI::ReturnDriverXAxis(){
	return DeadBand(_driverStick->GetX(frc::GenericHID::kRightHand));

}
bool OI::ReturnManualRightBumper(){
	return _manualStick->GetBumper(frc::GenericHID::kRightHand);
}
double OI::ReturnDriverYAxis(){
	return DeadBand(_driverStick->GetY(frc::GenericHID::kLeftHand));
}

double OI::DeadBand(double joystick) {
	if (-0.1295 < joystick && joystick < 0.1295) {
		joystick = 0;
	}
		return joystick;
}

double OI::ReturnManualLeftYAxis(){
	return DeadBand(_manualStick->GetY(frc::GenericHID::kLeftHand));
}

bool OI::ReturnManualLeftTrigger(){
	return (DeadBand(_manualStick->GetTriggerAxis(frc::GenericHID::kLeftHand)) > 0.5);
}

bool OI::ReturnManualLeftBump(){
	return _manualStick->GetBumper(frc::GenericHID::kLeftHand);
}

bool OI::ReturnDriverXButton() {
	return _driverStick->GetXButton();
}
double OI::ReturnManualRightYAxis(){
	return DeadBand(_manualStick->GetY(frc::GenericHID::kRightHand));
}


/*void OI::SwapLedMode(int mode)	{
	//1 is off, 3 is on
	table->PutNumber("ledMode",mode);
}
void OI::SetStreamMode(){
	// 0 standard sidebyside, 1 picture in picture main, 2 pip secondary 
	if (_driverStick->GetStartButtonPressed()){
		streamMode = (streamMode == 1)?2:1;
	}
	table->PutNumber("stream", streamMode);
}
void OI::SetCamMode(int mode) {
	// 0 vision process, 1 driver cam
	table->PutNumber("camMode", mode);
}
double* OI::ReturnVisionX(){
	targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0); 
	targetArea = table->GetNumber("ta",0.0);
	visionData[0] = targetOffsetAngle_Horizontal;
	visionData[1] = targetArea;

	xCorners = table->GetNumberArray("tcornx", {1.0,0.0,0.0,0.0});
	yCorners = table->GetNumberArray("tcorny", {1.0,0.0,0.0,0.0});
	VisionerCornerFinder* CornerFinder = new VisionerCornerFinder();
	visionData[2] = CornerFinder->LostandFound(xCorners, yCorners);
	printf("Diff=%f\n",visionData[2]);
	return visionData;
}
bool* OI::ReturnLightSensors() {
	lightData[0] = LightSensorZero->Get();
	lightData[1] = LightSensorOne->Get();
	return lightData;
}*/

bool OI::ReturnManualAButton() {
	return _manualStick->GetAButtonPressed();
}
bool OI::ReturnManualXButton(){
	if (_manualStick->GetXButtonReleased()){
	frc::Scheduler::GetInstance()->RemoveAll();
	}
	return _manualStick->GetXButton();
}
bool OI::ReturnManualYButton() {
	return _manualStick->GetYButton();	
}
bool OI::ReturnManualBButton() {
	return _manualStick->GetBButton();	
}


