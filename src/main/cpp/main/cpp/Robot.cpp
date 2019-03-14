#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include <frc/DriverStation.h>
#include <frc/GenericHID.h>
#include <main/include/Robot.h>
#include <frc/Preferences.h>
#include <frc/RobotBase.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/SPI.h>
#include <frc/VictorSP.h>
#include <exception>

void Robot::RobotInit() {
	frontRight.SetInverted(true);
	backRight.SetInverted(true);
	E.ElevatorFront.SetInverted(true);
	prefs = Preferences::GetInstance();
	frc::CameraServer::GetInstance()->StartAutomaticCapture();
	gyro = new AHRS(SPI::Port::kMXP);

	shift = new DoubleSolenoid(6, 7);

}

void Robot::DisabledInit() {
	shift->Set(DoubleSolenoid::Value::kForward);
	ServoDropped = false;
	climber->SetAngle(150); //High
}

void Robot::DisabledPeriodic() {
	getInput();

	drive_multiplier = prefs->GetDouble("DriveMultiplier", .8);
	turn_multiplier = prefs->GetDouble("TurnMultiplier", .8);
	kGyrokP = prefs->GetDouble("GyrokP", .25);
	kGyrokI = prefs->GetDouble("GyrokI", .01);
	ElbowkD = prefs->GetDouble("ElbowkD", 0);
	ElbowkI = prefs->GetDouble("ElbowkI", 0);
	ElbowkP = prefs->GetDouble("ElbowkP", 0);
	ElevatorkP = prefs->GetDouble("ElevatorP", 0);
	ElevatorkI = prefs->GetDouble("ElevatorI", 0);
	ElevatorkD = prefs->GetDouble("ElevatorD", 0);

	SmartDashboard::PutNumber("Drive Multiplier:", drive_multiplier);
	SmartDashboard::PutNumber("Turn Multiplier:", turn_multiplier);
	SmartDashboard::PutNumber("GyrokP:", kGyrokP);
	SmartDashboard::PutNumber("GyrokI:", kGyrokI);
	SmartDashboard::PutNumber("X Button:", POV);
	SmartDashboard::PutNumber("Elbow:", E.ElbowController->GetSelectedSensorPosition());
	SmartDashboard::PutNumber("elbowup", elbowup);
	SmartDashboard::PutNumber("P", ElbowkP);
	SmartDashboard::PutNumber("I", ElbowkI);
	SmartDashboard::PutNumber("D", ElbowkD);
	SmartDashboard::PutNumber("EP", ElevatorkP);
	SmartDashboard::PutNumber("EI", ElevatorkI);
	SmartDashboard::PutNumber("ED", ElevatorkD);
	SmartDashboard::PutNumber("Elevator", E.ElevatorMaster->GetSelectedSensorPosition());

	if(driver.GetStartButton()){
		E.ElbowController->SetSelectedSensorPosition(0);
	}
	if(driver.GetRawButton(7)){
		E.ElevatorMaster->SetSelectedSensorPosition(0);
		Lock = 0;
	}
}

void Robot::AutonomousInit() {
	//Pneumatics
	disk->Set(frc::DoubleSolenoid::Value::kReverse);

	//Elbow PID
	E.ElbowController->SetSelectedSensorPosition(0, kPIDLoopIdx, 30);
	E.ElbowController->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, 30);
	E.ElbowController->SetSensorPhase(true);
	E.ElbowController->Config_kD(kPIDLoopIdx, ElbowkD, 30);
	E.ElbowController->Config_kI(kPIDLoopIdx, ElbowkI, 30);
	E.ElbowController->Config_kP(kPIDLoopIdx, ElbowkP, 30);
	E.ElbowController->Config_IntegralZone(kPIDLoopIdx, 250, 30);
	E.ElbowController->ConfigAllowableClosedloopError(kPIDLoopIdx, 200, 30);

	//Elevator PID
	E.ElevatorBack.Set(ControlMode::Follower, 0);
	E.ElevatorMaster->SetSelectedSensorPosition(0, kPIDLoopIdx, 30);
	E.ElevatorMaster->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, 30);
	E.ElevatorMaster->SetSensorPhase(true);
	E.ElevatorMaster->Config_kD(kPIDLoopIdx, ElevatorkD, 30);
	E.ElevatorMaster->Config_kI(kPIDLoopIdx, ElevatorkI, 30);
	E.ElevatorMaster->Config_kP(kPIDLoopIdx, ElevatorkP, 30);
	E.ElevatorMaster->Config_IntegralZone(kPIDLoopIdx, 250, 30);
	E.ElevatorMaster->ConfigClosedLoopPeakOutput(kPIDLoopIdx, .45, 30);
}

void Robot::AutonomousPeriodic() {
	TeleopPeriodic();
}

void Robot::TeleopInit() {
}

int Level;
std::string Object;

void Robot::TeleopPeriodic() {

	getInput();
SmartDashboard::PutNumber("elbowup", elbowup);
	//Driver
	//Speed Throttle
	if (leftBumper) {
		speed = .5/drive_multiplier;
	}
	else if (rightBumper) {
		speed = 1/drive_multiplier;
	}
	else {
		speed = 1;
	}

	//Shift
	if (shiftup) {
		shift->Set(DoubleSolenoid::Value::kForward);
	}
	if (shiftdown) {
		shift->Set(DoubleSolenoid::Value::kReverse);
	}

	RaiderDrive(-(left_x * turn_multiplier), (-input_rt + input_lt) * drive_multiplier * speed);

	//Operator
	//Release Disk
	if (release_1 || release_2){
		disk->Set(DoubleSolenoid::kForward);
	}
	else{
		disk->Set(DoubleSolenoid::kReverse);
	}
	if(abs(elevator_up-elevator_down) > .1){
		E.ElevatorFront.Set(ControlMode::PercentOutput, -(elevator_up - elevator_down) * .5);
		Lock = E.ElevatorMaster->GetSelectedSensorPosition();
	}
	else{
		E.ElevatorFront.Set(ControlMode::Position, Lock);
	}

	if(ServoController){
		ServoDropped = true;
	}

	if(ServoDropped){
		climber->SetAngle(50); //Low	
	} else {
		climber->SetAngle(150); //High
	}
 	 
	//climber->SetAngle(50); //Low

	if(abs(intake) > .1){
		E.ArmLeft.Set(intake * .65);
		E.ArmRight.Set(-intake * .65);
	}
	else{
		E.ArmLeft.Set(.05);
		E.ArmRight.Set(-.05);
	}

	if (Straight){
		E.ElbowController->Set(ControlMode::Position, -5900);
	}

	else if (elbowup){
		E.ElbowController->Set(ControlMode::Position, -3500);
	}

	else if (elbowdown){
		E.ElbowController->Set(ControlMode::Position, -9150);
	}

	else if(MaxHeight){
		E.ElbowController->Set(ControlMode::Position, 0);
	}

	Climber_Bottom.Set(ControlMode::PercentOutput, ClimberWheels);
	Climber_Top.Set(ControlMode::PercentOutput, ClimberWheels);

	SmartDashboard::PutNumber("Teleop Elevator", E.ElevatorMaster->GetSelectedSensorPosition());
	SmartDashboard::PutNumber("Desired", Lock);

}

void Robot::getInput() {

	//Driver
	input_lt = driver.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand); //Left Trigger
	input_rt = driver.GetTriggerAxis(GenericHID::JoystickHand::kRightHand); //Right Trigger

	left_x = driver.GetX(GenericHID::JoystickHand::kLeftHand); //Left Joystick: X Axis

	leftBumper = driver.GetBumper(GenericHID::JoystickHand::kLeftHand); //Left Bumper
	rightBumper = driver.GetBumper(GenericHID::JoystickHand::kRightHand); //RIght Bumper

	shiftup = driver.GetAButton(); //A Button
	shiftdown = driver.GetBButton(); //B Button

	TargetTape = driver.GetXButton(); //X Button
	TargetBall = driver.GetYButton(); //Y Button
	NullTarget = driver.GetRawButton(10); //Right Joystick Button

	//Operater
	elevator_up = operater.GetTriggerAxis(GenericHID::JoystickHand::kRightHand); //Right Trigger
	elevator_down = operater.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand); //Left Trigger

	elbowup = operater.GetXButton(); //X Button
	elbowdown = operater.GetAButton(); //Y Button
	MaxHeight = operater.GetYButton(); //A Button
	Straight = operater.GetBButton(); //B Button

	release_1 = operater.GetBumper(frc::GenericHID::JoystickHand::kRightHand);
	release_2 = operater.GetBumper(frc::GenericHID::JoystickHand::kLeftHand);
	ServoController = operater.GetStartButton(); //Start Button

	intake = operater.GetY(frc::GenericHID::JoystickHand::kLeftHand); //Left Joystick: Y Axis
	Zero = operater.GetRawButton(9); //Left Joystick Button

	ClimberWheels = operater.GetY(frc::GenericHID::JoystickHand::kRightHand); //Right Joystick: Y Axis
	climb = operater.GetRawButton(10); //Right Joystick Button

	POV = operater.GetPOV(); //D-Pad


}

void Robot::RaiderDrive(double zRotation, double xSpeed){
	//Left
	frontLeft.Set(ControlMode::PercentOutput, xSpeed - zRotation);
	backLeft.Set(ControlMode::PercentOutput, xSpeed - zRotation);

	//Right
	frontRight.Set(ControlMode::PercentOutput, xSpeed + zRotation);
	backRight.Set(ControlMode::PercentOutput, xSpeed + zRotation);
}

void Robot::PIDControl(double Desired, std::string Object, SpeedController &kSpeedController){
	kError = Desired - kCurrent;
	kIntegral += kError;
	kSpeed = kP * kError + kI * kIntegral + kF;
	if(kSpeed < 0){
		kSpeed = kSpeed/3;
	}
	kSpeedController.Set(kSpeed);
	SmartDashboard::PutNumber("Current Speed", -(kSpeed + kF));
	SmartDashboard::PutNumber("Current Error", kError);
	SmartDashboard::PutNumber("Current Integral", kIntegral);
	SmartDashboard::PutNumber("Current", kCurrent);
}


//PID Controller Position = 1/2(gravity(16ft^2)) + Velocity + Initial Position

START_ROBOT_CLASS(Robot)
