/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/AnalogInput.h>

class AutoTargeting {

 public:
  double LineUp();
  double DriveForward();
  double Chase(char Choose);
  double AutoTargetTurn();
  double AutoTargetForward();

  double kF;
  double tP;
  double fP;
  double sP;
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  frc::AnalogInput *Ultrasonic = new frc::AnalogInput {9};
  
 private:

  //Limelight
	double tx = table->GetNumber("tx",0.0);
	double ty = table->GetNumber("ty",0.0);
	double ta = table->GetNumber("ta",0.0);
	double ts = table->GetNumber("ts",0.0);
	double pipeline = table->GetNumber("getpipe", 0.0);

  //PID Values
  double sError;
  double tCorrection;
  double fCorrection;
  double sCorrection;
  double kCorrection;
};
