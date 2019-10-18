/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//Alvin - Basically set ports and talon numbers for talons and controllers.
//Put them here so its all in one place
#pragma once
//Drive Base
const int frontLeftDrive = 3; //encoder
const int frontRightDrive = 12; //encoder
const int backLeftDrive = 2;
const int backRightDrive = 0;

const int frontLeftTurn = 13; //encoder
const int frontRightTurn = 14; //encoder
const int backLeftTurn = 1;
const int backRightTurn = 15;

const double PIDDeadband = 0.1;
const double RampTime = 0.00;
const double NominalOutput = 1.00;
const double MaxOutput = 0.9
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

// For example to map the left and right motors, you could define the
// following variables to use with your drivetrain subsystem.
// constexpr int kLeftMotor = 1;
// constexpr int kRightMotor = 2;

// If you are using multiple modules, make sure to define both the port
// number and the module. For example you with a rangefinder:
// constexpr int kRangeFinderPort = 1;
// constexpr int kRangeFinderModule = 1;
