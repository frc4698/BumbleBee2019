#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include <main/include/Elevator.h>
#include <frc/SmartDashboard/SmartDashboard.h>

void Elevator::MoveElevator(double kPosition){
	ElevatorMaster->Set(ControlMode::Position, kPosition);
}

void Elevator::IntakeSpeed(double xSpeed){
	ArmLeft.Set(xSpeed);
	ArmRight.Set(xSpeed);
}

void Elevator::MoveElbow(double kPosition){
	ElbowController->Set(ControlMode::Position, kPosition);
}

