/*
 * Elevator.h
 *
 *  Created on: Jan 31, 2019
 *      Author: Shane Becker
 */

#ifndef SRC_MAIN_ELEVATOR_H_
#define SRC_MAIN_ELEVATOR_H_

#include <ctre/phoenix.h>
#include <string>
#include <frc/VictorSP.h>


class Elevator {
	/*DigitalInput Elevator_Zero;
	DigitalInput Elevator_Max;
	DigitalInput Elbow_Min;
	DigitalInput Elbow_Max;
	DigitalInput Elbow_Zero;*/

public:

	void MoveElevator(double kPosition);
	void IntakeSpeed(double xSpeed);

	TalonSRX ElevatorFront { 0 }; //40amp
	TalonSRX ElevatorBack { 1 }; //40amp

	TalonSRX ElbowMotor { 3 }; //30amp
	frc::VictorSP ArmLeft { 0 }; //30amp
	frc::VictorSP ArmRight { 1 }; //30amp

	TalonSRX *ElevatorMaster = new TalonSRX(0);
	TalonSRX *ElevatorSlave = new TalonSRX(1);
	TalonSRX *ElbowController = new TalonSRX(3);

	double Lock;

private:

double ElevatorMultiplier;

};

#endif /* SRC_MAIN_ELEVATOR_H_ */
