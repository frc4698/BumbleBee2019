#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include <main/include/Elevator.h>
#include <frc/SmartDashboard/SmartDashboard.h>

void Elevator::MoveElevator(double xSpeed){

	if(fabs(xSpeed) > .01){
		if(xSpeed > 0){
			ElevatorMultiplier = .35;
		}
		else{
			ElevatorMultiplier = 1;
		}
		ElevatorFront.Set(ControlMode::PercentOutput, xSpeed * ElevatorMultiplier);
		Lock = ElevatorMaster->GetSelectedSensorPosition();
	}
	else{
		ElevatorFront.Set(ControlMode::Position, Lock);
		if(Lock > -100){
			ElevatorFront.Set(ControlMode::PercentOutput, 0);
		}
	}
	//ElevatorFront.Set(ControlMode::PercentOutput, xSpeed);
	frc::SmartDashboard::PutNumber("Abs SPeed", abs(xSpeed));
	frc::SmartDashboard::PutBoolean("Abs Speed Bool", abs(xSpeed) > .01);
}

void Elevator::IntakeSpeed(double xSpeed){
	if(abs(xSpeed) > .1){
		ArmLeft.Set(xSpeed * .65);
		ArmRight.Set(-xSpeed * .65);
	}
	else{
		ArmLeft.Set(-.175);
		ArmRight.Set(.175);
	}
}

void Elevator::Climber(double xSpeed, double ySpeed){
	ClimberBottom->Set(ControlMode::Follower, 7);
	Climber_Top.Set(ControlMode::PercentOutput, xSpeed);
	PullyMotor.Set(ControlMode::PercentOutput, ySpeed);
}
