#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include <main/include/Elevator.h>
#include <frc/SmartDashboard/SmartDashboard.h>

void Elevator::MoveElevator(double xSpeed){
	if(abs(xSpeed) > .1){
		if(xSpeed > 0){
			ElevatorMultiplier = .65;
		}
		else{
			ElevatorMultiplier = .5;
		}
		if(ElevatorMaster->GetSelectedSensorPosition() > 10000 || ElevatorMaster->GetSelectedSensorPosition() < 1000){
			ElevatorFront.Set(ControlMode::PercentOutput, xSpeed * .75);
			Lock = ElevatorMaster->GetSelectedSensorPosition();
		}
		else{
			ElevatorFront.Set(ControlMode::PercentOutput, xSpeed * ElevatorMultiplier);
			Lock = ElevatorMaster->GetSelectedSensorPosition();
		}
	}
	else{
		ElevatorFront.Set(ControlMode::Position, Lock);
	}
}

void Elevator::IntakeSpeed(double xSpeed){
	if(abs(xSpeed) > .1){
		ArmLeft.Set(xSpeed * .65);
		ArmRight.Set(-xSpeed * .65);
	}
	else{
		ArmLeft.Set(.05);
		ArmRight.Set(-.05);
	}
}

