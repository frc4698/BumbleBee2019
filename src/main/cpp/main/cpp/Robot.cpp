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
	E.ElevatorBack.SetInverted(true);
	E.ElevatorSlave->Set(ControlMode::Follower, 0);
	//E.ElbowMotor.SetInverted(true);
	prefs = Preferences::GetInstance();
	cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
	camera.SetResolution(160, 120);
	gyro = new AHRS(SPI::Port::kMXP);

	shift = new DoubleSolenoid(6, 7);

	AT.table->PutNumber("pipeline", 0);

}


void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {
	getInput();

	DriveMultiplier = prefs->GetDouble("DriveMultiplier", .8);
	TurnMultiplier = prefs->GetDouble("TurnMultiplier", .8);
	ElbowkD = prefs->GetDouble("ElbowkD", 0);	
	ElbowkI = prefs->GetDouble("ElbowkI", 0);
	ElbowkP = prefs->GetDouble("ElbowkP", 0);
	ElevatorkP = prefs->GetDouble("ElevatorP", 0);
	ElevatorkI = prefs->GetDouble("ElevatorI", 0);
	ElevatorkD = prefs->GetDouble("ElevatorD", 0);
	AT.tP = prefs->GetDouble("tP", 0);
	AT.kF = prefs->GetDouble("kF", 0);

	SmartDashboard::PutNumber("Drive Multiplier:", DriveMultiplier);
	SmartDashboard::PutNumber("Turn Multiplier:", TurnMultiplier);
	SmartDashboard::PutNumber("Elbow:", E.ElbowController->GetSelectedSensorPosition());
	SmartDashboard::PutNumber("Elevator", E.ElevatorMaster->GetSelectedSensorPosition());

	if(driver.GetStartButton()){
		E.ElbowController->SetSelectedSensorPosition(0);
	}
	if(driver.GetRawButton(7)){
		E.ElevatorMaster->SetSelectedSensorPosition(0);
		E.Lock = 0;
	}
}

void Robot::AutonomousInit() {
	//Pneumatics
	disk->Set(frc::DoubleSolenoid::Value::kForward);
	shift->Set(frc::DoubleSolenoid::Value::kReverse);
	lift->Set(frc::DoubleSolenoid::Value::kReverse);

	//Elbow PID
	E.ElbowController->SetSelectedSensorPosition(0, kPIDLoopIdx, 30);
	E.ElbowController->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, 30);
	E.ElbowController->SetSensorPhase(false);
	E.ElbowController->Config_kD(kPIDLoopIdx, ElbowkD, 30);
	E.ElbowController->Config_kI(kPIDLoopIdx, ElbowkI, 30);
	E.ElbowController->Config_kP(kPIDLoopIdx, ElbowkP, 30);
	E.ElbowController->Config_IntegralZone(kPIDLoopIdx, 250, 30);
	E.ElbowController->ConfigAllowableClosedloopError(kPIDLoopIdx, 200, 30);

	//Elevator PID
	E.ElevatorMaster->SetSelectedSensorPosition(0, kPIDLoopIdx, 30);
	E.ElevatorMaster->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, 30);
	E.ElevatorMaster->SetSensorPhase(false);
	E.ElevatorMaster->Config_kD(kPIDLoopIdx, ElevatorkD, 30);
	E.ElevatorMaster->Config_kI(kPIDLoopIdx, ElevatorkI, 30);
	E.ElevatorMaster->Config_kP(kPIDLoopIdx, ElevatorkP, 30);
	E.ElevatorMaster->Config_IntegralZone(kPIDLoopIdx, 250, 30);
	E.ElevatorMaster->ConfigClosedLoopPeakOutput(kPIDLoopIdx, .45, 30);

	//Climber
	CurrentPos = false;

	E.ElbowController->SetSelectedSensorPosition(0);
	E.ElevatorMaster->SetSelectedSensorPosition(0);
	E.Lock = 0;
	E.ElbowController->Set(ControlMode::Position, 0);
}

void Robot::AutonomousPeriodic() {
	TeleopPeriodic();
}

void Robot::TeleopInit() {
}

int Level;
std::string Object;

void Robot::TeleopPeriodic() {
	
	double tx = table->GetNumber("tx",0.0);
	double ty = table->GetNumber("ty",0.0);
	double ta = table->GetNumber("ta",0.0);
	double ts = table->GetNumber("ts",0.0);

	getInput();
	//Driver

	//Speed Throttle
	if (leftBumper) {
		Speed = .5;
	}
	else {
		Speed = 1;
	}

	//Shift
	if (ShiftUp) {
		shift->Set(DoubleSolenoid::Value::kForward);
		TurnMultiplier = .8;
	}
	if (ShiftDown) {
		shift->Set(DoubleSolenoid::Value::kReverse);
		TurnMultiplier = .4;
	}

	//Limelight Target Reflective Strips
	if(AutoAline){
		table->PutNumber("pipeline", 2);
		tCorrection = AT.AutoTargetTurn();
	}

	//Turns off Targeting
	if(NullTarget){
		table->PutNumber("pipeline", 0);
		tCorrection = 0;
		tIntegral = 0;
	}
	/*
	//PID for Left and Right
	tError = tx;
	tPCorrection = tError * kP;
	if (abs(tIntegral) > 100000){
		tIntegral = tIntegral/5;
	}
	tIntegral += tError;
	tICorrection = tIntegral * kI;
	tDCorrection = (tError - tPrevError) * kD;
	tCorrection = tPCorrection + tICorrection + tDCorrection;
	if(table->GetNumber("getpipe", 0.0) == 0 || tx == 0){
		tCorrection = 0;
		tIntegral = 0;
	}
	tPrevError = tError;

	//PID for Forward and Backwards
	fError = ty;
	fPCorrection = fError * kP;
	if (abs(tIntegral) > 100000){
		fIntegral = fIntegral/5;
	}
	fIntegral += fError;
	fICorrection = fIntegral * kI;
	fDCorrection = (fError - fPrevError) * kD;
	fCorrection = fPCorrection + fICorrection + fDCorrection;
	if(table->GetNumber("getpipe", 0.0) == 0 || tx == 0){
		fCorrection = 0;
		fIntegral = 0;
	}
	fPrevError = fError;*/

	//Custom Drive Class (Like Arcade Drive)
	RaiderDrive((-(LeftX * TurnMultiplier) * Speed) + tCorrection, (-Input_Rt + Input_Lt) * DriveMultiplier * Speed);

	//Operator

	//Elevator
	ElevatorSpeed = ElevatorUp - ElevatorDown;

	E.Climber(0, ClimberPully);

	//Move Elevator
	E.MoveElevator(-ElevatorSpeed);
	//Intake
	E.IntakeSpeed(-Intake);

	//Release Disk
	if (Release){
		disk->Set(DoubleSolenoid::kReverse);
	}
	else if (Grab){
		disk->Set(DoubleSolenoid::kForward);
	}

	if (Straight){
		E.ElbowController->Set(ControlMode::Position, 5000);
	}

	else if (ElbowUp){
		E.ElbowController->Set(ControlMode::Position, 3000);
	}

	else if (ElbowDown){
		E.ElbowController->Set(ControlMode::Position, 8800);
	}

	else if(MaxHeight){
		E.ElbowController->Set(ControlMode::Position, 0);
	}

	if(POV == 270){
		E.ElbowController->Set(ControlMode::Position, -6200);
	}
	if(POV == 90){
		E.ElevatorMaster->Set(ControlMode::Position, -11450);
		E.ElbowController->Set(ControlMode::Position, -3000);
	}

	if(Climb){
		if(CurrentPos == false){
			lift->Set(DoubleSolenoid::Value::kForward);
			CurrentPos = true;
		}
		else{
			lift->Set(DoubleSolenoid::Value::kReverse);
			CurrentPos = false;
		}
	}

}

void Robot::getInput() {

	//Driver
	Input_Lt = driver.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand); //Left Trigger
	Input_Rt = driver.GetTriggerAxis(GenericHID::JoystickHand::kRightHand); //Right Trigger

	LeftX = driver.GetX(GenericHID::JoystickHand::kLeftHand); //Left Joystick: X Axis

	leftBumper = driver.GetBumper(GenericHID::JoystickHand::kLeftHand); //Left Bumper
	rightBumper = driver.GetBumper(GenericHID::JoystickHand::kRightHand); //RIght Bumper

	ShiftUp = driver.GetAButton(); //A Button
	ShiftDown = driver.GetBButton(); //B Button

	AutoAline = driver.GetStartButton(); //Start Button

	NullTarget = driver.GetRawButton(10); //Right Joystick Button

	//Operater
	ElevatorUp = operater.GetTriggerAxis(GenericHID::JoystickHand::kRightHand); //Right Trigger
	ElevatorDown = operater.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand); //Left Trigger

	ElbowUp = operater.GetXButton(); //X Button
	ElbowDown = operater.GetAButton(); //Y Button
	MaxHeight = operater.GetYButton(); //A Button
	Straight = operater.GetBButton(); //B Button

	Grab = operater.GetBumper(frc::GenericHID::JoystickHand::kRightHand);
	Release = operater.GetBumper(frc::GenericHID::JoystickHand::kLeftHand);
	ServoController = operater.GetStartButton(); //Start Button

	Intake = operater.GetY(frc::GenericHID::JoystickHand::kLeftHand); //Left Joystick: Y Axis

	ClimberPully = operater.GetY(frc::GenericHID::JoystickHand::kRightHand); //Right Joystick: Y Axis
	Climb = operater.GetRawButtonReleased(10); //Right Joystick Button

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


//PID Controller Position = 1/2(gravity(16ft^2)) + Velocity + Initial Position

START_ROBOT_CLASS(Robot)
