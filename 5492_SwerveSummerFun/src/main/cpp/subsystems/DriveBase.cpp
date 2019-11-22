/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveBase.h"
#include "RobotMap.h"
#include "OI.h"
#include <subsystems/DriveBase.h>
#include <frc/Commands/Scheduler.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/drive/DifferentialDrive.h>
#include "OI.h"
#include <frc/Joystick.h>
#include <frc/RobotDrive.h>
#include <iostream>
#include "Robot.h"
#include <ctre/Phoenix.h>
#include <commands/TankDrive.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
WPI_TalonSRX * FrontLDrive;
WPI_TalonSRX * FrontRDrive;
WPI_TalonSRX * BackLDrive; 
WPI_TalonSRX * BackRDrive;

WPI_TalonSRX * FrontLTurn;
WPI_TalonSRX * FrontRTurn;
WPI_TalonSRX * BackLTurn; 
WPI_TalonSRX * BackRTurn;

DriveBase::DriveBase() : Subsystem("DriveBase") {}

void DriveBase::DriveBaseInit() {
initialized = true;
		FrontLDrive = new WPI_TalonSRX (frontLeftDrive);
		FrontRDrive = new WPI_TalonSRX (frontRightDrive);
		BackLDrive = new WPI_TalonSRX (backLeftDrive);
		BackRDrive = new WPI_TalonSRX (backRightDrive);
		FrontLTurn = new WPI_TalonSRX (frontLeftTurn);
		FrontRTurn = new WPI_TalonSRX (frontRightTurn);
		BackLTurn = new WPI_TalonSRX (backLeftTurn);
		BackRTurn = new WPI_TalonSRX (backRightTurn);
		
		FrontRDrive->ConfigFactoryDefault();
		FrontLDrive->ConfigFactoryDefault();
		BackRDrive->ConfigFactoryDefault();
		BackLDrive->ConfigFactoryDefault();

		FrontRDrive->SetInverted(true);
		FrontLDrive->SetInverted(false);
		BackRDrive->SetInverted(true);
		BackLDrive->SetInverted(false);

		FrontRDrive->ConfigPeakCurrentLimit(50,0);
		FrontLDrive->ConfigPeakCurrentLimit(50,0);
		BackRDrive->ConfigPeakCurrentLimit(50,0);
		BackLDrive->ConfigPeakCurrentLimit(50,0);

		FrontRDrive->ConfigPeakCurrentDuration(1000,0);
		FrontLDrive->ConfigPeakCurrentDuration(1000,0);
		BackRDrive->ConfigPeakCurrentDuration(1000,0);
		BackLDrive->ConfigPeakCurrentDuration(1000,0);

		FrontLDrive->ConfigOpenloopRamp(RampTime, 0);
    	FrontRDrive->ConfigOpenloopRamp(RampTime, 0);
   		BackLDrive->ConfigOpenloopRamp(RampTime, 0);
    	BackRDrive->ConfigOpenloopRamp(RampTime, 0);

		FrontRDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
		FrontLDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);

		FrontRDrive->EnableCurrentLimit(true);
		FrontLDrive->EnableCurrentLimit(true);
		BackRDrive->EnableCurrentLimit(true);
		BackLDrive->EnableCurrentLimit(true);


		//PID BTW
		/*FrontRDrive->Config_kP(0, RightP, 0);
		FrontRDrive->Config_kI(0, RightI, 0);
		FrontRDrive->Config_kD(0, RightD, 0);
		FrontLDrive->Config_kP(0, LeftP, 0);
		FrontLDrive->Config_kI(0, LeftI, 0);
		FrontLDrive->Config_kD(0, LeftD, 0);
		*/
		FrontRDrive->ConfigNominalOutputForward(NominalOutput, 0);
		FrontRDrive->ConfigNominalOutputReverse(-NominalOutput, 0);
		FrontLDrive->ConfigNominalOutputForward(NominalOutput, 0);
		FrontLDrive->ConfigNominalOutputReverse(-NominalOutput, 0);

		FrontRDrive->SetSensorPhase(false);
		FrontLDrive->SetSensorPhase(false);


		FrontRDrive->ConfigPeakOutputForward(MaxOutput, 0);
		FrontRDrive->ConfigPeakOutputReverse(-MaxOutput, 0);
		FrontLDrive->ConfigPeakOutputForward(MaxOutput, 0);
		FrontLDrive->ConfigPeakOutputReverse(-MaxOutput, 0);

		FrontRDrive->ConfigNeutralDeadband(PIDDeadband, 0);
		FrontLDrive->ConfigNeutralDeadband(PIDDeadband, 0);

		FrontRDrive->SetSelectedSensorPosition(0,0,0);
		FrontLDrive->SetSelectedSensorPosition(0,0,0);


		BackLDrive->SetSafetyEnabled(false);
		BackRDrive->SetSafetyEnabled(false);


		BackLDrive->Follow(*FrontLDrive);
		BackRDrive->Follow(*FrontRDrive);
		printf("Done setting up motor \n");

		
}
void DriveBase::RampSwitch(bool rampOn) {
	double ramp = (rampOn)?RampTime:0;
	FrontLDrive->ConfigOpenloopRamp(ramp, 0);
    FrontRDrive->ConfigOpenloopRamp(ramp, 0);
   	BackLDrive->ConfigOpenloopRamp(ramp, 0);
    BackRDrive->ConfigOpenloopRamp(ramp, 0);

}
void DriveBase::InitDefaultCommand() {
  if (!initialized) {
		DriveBase::DriveBaseInit();
	}
	Robot::m_drivebase.SetDefaultCommand(new TankDrive());
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}
void DriveBase::SwerveDrive(double xAxis, double yAxis){
  
}
void DriveBase::reverseDrive (bool bButton) {
if (bButton) {
	driveConstant = driveConstant * -1;
}
}
void DriveBase::slowDrive (bool yButton) {
	if (yButton){
		if (speed == 1)
		{
			speed = .5;
		}
		else
		{
			speed = 1;
		}
		
	}
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
