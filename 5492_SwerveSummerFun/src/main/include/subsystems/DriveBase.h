/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/commands/Subsystem.h>

class DriveBase : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
  bool initialized = false;
  double driveConstant = -1;
  double speed = 1;

 public:
  DriveBase();
  void DriveBaseInit();
  void InitDefaultCommand() override;
  void ArcadeDrive(double xAxis, double yAxis);
  void RampSwitch(bool rampOn);
  void reverseDrive(bool bButton);
  void slowDrive(bool yButton);
};
