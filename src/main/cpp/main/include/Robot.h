/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <AHRS.h>
#include <ctre/phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Drive/DifferentialDrive.h>
#include <frc/IterativeRobot.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <string>
#include <frc/WPILib.h>
#include <main/include/Elevator.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/Servo.h>
#include <main/include/Constant.h>

namespace frc {
class Preferences;
} /* namespace frc */

class Robot: public frc::IterativeRobot {

	VictorSPX frontLeft { 4 }; //40amp
	VictorSPX backLeft { 5 }; //40amp
	VictorSPX frontRight { 8 }; //40amp
	VictorSPX backRight { 9 }; //40amp

	VictorSPX Climber_Bottom { 6 }; //40amp
	VictorSPX Climber_Top { 7 }; //40amp

	XboxController driver { 0 };
	XboxController operater { 1 };
	Joystick operate{1};

	Timer drive_timer;

	Timer PID_Timer;

	AHRS *gyro;

	Elevator E;

public:

	void RobotInit() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void getInput();
	void RaiderDrive(double zRotation, double xSpeed);
	void LimeLight(char Item);
	void setLevel(std::string Object,int Level);
	void PIDControl(double Desired, std::string Object, SpeedController &kSpeedController);

private:
	//Limelight
	std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
	double tx = table->GetNumber("tx",0.0);
	double ty = table->GetNumber("ty",0.0);
	double ta = table->GetNumber("ta",0.0);
	double ts = table->GetNumber("ts",0.0);
	double pipeline = table->GetNumber("getpipe", 0.0);

	//Pneumatics
	DoubleSolenoid *disk = new DoubleSolenoid(0, 1);
	DoubleSolenoid *shift;
	DoubleSolenoid *lift = new DoubleSolenoid(4, 5);

	//Servo
	Servo *climber = new Servo(2);

	//SmartDashboard
	Preferences *prefs;

	//Encoders
	double Circumfrence = 18.9;
	double GearRatio = 1/1;
	double loop = 0;

	//Driver Inputs
	double input_lt, input_rt; //Forward and Backward, Left and Right Trigger
	double left_x; //Turn, Left Joy: X Axis
	bool shiftup, shiftdown; //A and B button
	bool rightBumper, leftBumper; //Left and Right Bumper
	bool TargetTape, TargetBall; //X and Y
	bool NullTarget; //Right Joystick Button
	bool ServoController; //Start Button
	bool ServoDropped = false;

	//Operater Input
	double elevator_up, elevator_down; //Left and Right Trigger
	bool climb; //Button 9: Left Joy
	double intake; //Right Joy Y Axis
	bool elbowup, elbowdown;//X: Arm Up, Y: Arm Down
	bool release_1; //Right Bumper
	bool release_2; //Left Bumper
	double POV; //O: Top Level, 90: Middle Level, 180:Bottom
	bool Zero; //Button 10: Right Joy
	double ClimberWheels; //Left Joy Y Axis
	bool Straight; //B Button
	bool MaxHeight; //A Button

	//Drive Values
	double drive_multiplier;
	double turn_multiplier;
	double speed;

	

	//PID
	double kP;
	double kI;
	double kD;
	double kF;
	double kError;
	double kIntegral;
	double kDerivative;
	double kCurrent;
	double kSpeed;

	double ElbowkP;
	double ElbowkI;
	double ElbowkD;

	double ElevatorkP;
	double ElevatorkI;
	double ElevatorkD;

	double Test;

};
