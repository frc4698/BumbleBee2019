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

	//SmartDashboard
	Preferences *prefs;

	//Driver Inputs
	double Input_Lt, Input_Rt; //Forward and Backward, Left and Right Trigger
	double LeftX; //Turn, Left Joy: X Axis
	bool ShiftUp, ShiftDown; //A and B button
	bool rightBumper, leftBumper; //Left and Right Bumper
	bool TargetTape, TargetBall; //X and Y
	bool NullTarget; //Right Joystick Button
	bool ServoController; //Start Button
	double AutoAline; //Start button
	

	//Operater Input
	double ElevatorUp, ElevatorDown; //Left and Right Trigger
	bool Climb; //Button 9: Left Joy
	double Intake; //Right Joy Y Axis
	bool ElbowUp, ElbowDown;//X: Arm Up, Y: Arm Down
	bool Grab; //Right Bumper
	bool Release; //Left Bumper
	double POV; //O: Top Level, 90: Middle Level, 180:Bottom
	bool Zero; //Button 10: Right Joy
	double ClimberWheels; //Left Joy Y Axis
	bool Straight; //B Button
	bool MaxHeight; //A Button
	bool ClimberMode; //Activated by Start Button

	//Drive Values
	double DriveMultiplier;
	double TurnMultiplier;
	double Speed;
	double ElevatorSpeed;
	bool CurrentPos = false;

	//PID
	double kP; //Drivetrain P value
	double kI; //Drivetrain I value
	double kD; //Drivetrain D value

	double ElbowkP; //Elbow P Value
	double ElbowkI; //Elbow I Value
	double ElbowkD; //Elbow D Value

	double ElevatorkP; //Elevator P Value
	double ElevatorkI; //Elevator I Value
	double ElevatorkD; //Elevator D Value

	double tError;
	double tIntegral;
	double tPrevError;
	double tPCorrection;
	double tICorrection;
	double tDCorrection;
	double tCorrection;

	double fError;
	double fIntegral;
	double fPrevError;
	double fPCorrection;
	double fICorrection;
	double fDCorrection;
	double fCorrection;
};
