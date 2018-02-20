/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <math.h>

using namespace std;

class Robot : public frc::IterativeRobot {
public:

	TalonSRX *frontLeftSpeedController, *backLeftSpeedController, *frontRightSpeedController, *backRightSpeedController, *leftIntakeSpeedController, *rightIntakeSpeedController, *centerToteTunnel, *leftToteTunnel, *rightToteTunnel, *climberBackUpSpeedController;
	Joystick *myStick;
	SmartDashboard *myData;
	DoubleSolenoid *gearBoxShifter;
	double unitsToSwitch = calculateUnitsAuto(120);
	double PulseWidth = 0;
	bool solenoidForward;
	bool solenoidBackward;
	DoubleSolenoid *intake1;
	DoubleSolenoid *intake2;
	string gameData;
	PigeonIMU *pigeon;
	double gyroValues[3];
	void RobotInit() {
		myStick = new Joystick(0);
		frontLeftSpeedController = new TalonSRX(0);
		frontRightSpeedController = new TalonSRX(1);
		backLeftSpeedController = new TalonSRX(3);
		backRightSpeedController = new TalonSRX(2);
		leftIntakeSpeedController = new TalonSRX(4);
		rightIntakeSpeedController = new TalonSRX(5);
		centerToteTunnel = new TalonSRX(21);
		rightToteTunnel = new TalonSRX(7);
		leftToteTunnel = new TalonSRX(8);
		//climberBackUpSpeedController = new TalonSRX(21);
		leftIntakeSpeedController->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
		gearBoxShifter = new DoubleSolenoid(0,1);
		pigeon = new PigeonIMU(rightToteTunnel);
		//intake1 = new DoubleSolenoid(3,4);
		//intake2 = new DoubleSolenoid(5,6);
	//	intake1 -> Set(DoubleSolenoid::Value::kReverse);
	//	intake2 -> Set(DoubleSolenoid::Value::kReverse);


	}

	void torqueMode() {
		solenoidForward = myStick->GetRawButton(5);  // LB
		if (solenoidForward) {
			gearBoxShifter->Set(DoubleSolenoid::Value::kForward);
		}
	}


	void mechIntakeOuttake() {

		double mechValue = myStick->GetRawAxis(2);
		leftIntakeSpeedController->Set(ControlMode::PercentOutput, (mechValue));
		rightIntakeSpeedController->Set(ControlMode::PercentOutput, (mechValue));
		centerToteTunnel->Set(ControlMode::PercentOutput, (mechValue));
		leftToteTunnel->Set(ControlMode::PercentOutput, (-mechValue));
		rightToteTunnel->Set(ControlMode::PercentOutput, (mechValue));

		double mechValue1 = myStick->GetRawAxis(3);
				leftIntakeSpeedController->Set(ControlMode::PercentOutput, (-mechValue1));
				rightIntakeSpeedController->Set(ControlMode::PercentOutput, (-mechValue1));
				centerToteTunnel->Set(ControlMode::PercentOutput, (-mechValue1));
				leftToteTunnel->Set(ControlMode::PercentOutput, (mechValue1));
				rightToteTunnel->Set(ControlMode::PercentOutput, (-mechValue1));
	}


	void mechIntake() {

	}

	void speedMode() {
		solenoidBackward = myStick->GetRawButton(6); //RB
		if (solenoidBackward) {
			gearBoxShifter->Set(DoubleSolenoid::Value::kReverse);
		}
	}

	double calculateUnitsAuto(int distance)
	{
		double units = (distance/(6*M_PI))*4096;
		return units;
	}
	void moveRobotTeleop() {
		double leftWheels = myStick->GetRawAxis(1);
		double rightWheels = myStick->GetRawAxis(5);
		if(leftWheels < 0) {
			frontLeftSpeedController->Set(ControlMode::PercentOutput, leftWheels*leftWheels*(-1));
			backLeftSpeedController->Set(ControlMode::PercentOutput, leftWheels*leftWheels*(-1));
		}

		else {
			frontLeftSpeedController->Set(ControlMode::PercentOutput, leftWheels* leftWheels);
			backLeftSpeedController->Set(ControlMode::PercentOutput, leftWheels*leftWheels);
		}

		if(rightWheels < 0) {
			frontRightSpeedController->Set(ControlMode::PercentOutput, (rightWheels)*(rightWheels));
			backRightSpeedController->Set(ControlMode::PercentOutput, (rightWheels)*(rightWheels));
		}

		else {
			frontRightSpeedController->Set(ControlMode::PercentOutput, (-rightWheels)*(rightWheels));
			backRightSpeedController->Set(ControlMode::PercentOutput, (-rightWheels)*(rightWheels));
		}
	}
	void moveAutoForward()
	{
		frontLeftSpeedController->Set(ControlMode::PercentOutput, -0.1);
		backLeftSpeedController->Set(ControlMode::PercentOutput, -0.1);
		frontRightSpeedController->Set(ControlMode::PercentOutput, 0.1);
		backRightSpeedController->Set(ControlMode::PercentOutput, 0.1);
	}
	void colorIsInFrontOfUsAuto() {
		  while (PulseWidth < unitsToSwitch)
			{
				moveAutoForward();
			}
		  shootCubeAuto();

	 }
	void shootCubeAuto() {
		centerToteTunnel->Set(ControlMode::PercentOutput, 0.75);
		rightToteTunnel->Set(ControlMode::PercentOutput, 0.75);
		leftToteTunnel->Set(ControlMode::PercentOutput, 0.75);
	}
	void turnNinetyLeft() //right wheels go forward, left wheels go back
	{
		while(gyroValues[2] > -69) {
		pigeon->GetAccumGyro(gyroValues);
		myData->PutNumber("Gyro z", gyroValues[2]);
		frontLeftSpeedController->Set(ControlMode::PercentOutput, -.3);
		backLeftSpeedController->Set(ControlMode::PercentOutput, -.3);
		frontRightSpeedController->Set(ControlMode::PercentOutput, -.3);
		backRightSpeedController->Set(ControlMode::PercentOutput, -.3);
		}
		pigeon->SetAccumZAngle(0,10);
		frontRightSpeedController->GetSensorCollection().SetQuadraturePosition(0,10);

	}
	void turnNinetyRight() //right wheels go forward, left wheels go back
	{
		while(gyroValues[2] < 90) {
		pigeon->GetAccumGyro(gyroValues);
		myData->PutNumber("Gyro z", gyroValues[2]);
		frontLeftSpeedController->Set(ControlMode::PercentOutput, -1);
		backLeftSpeedController->Set(ControlMode::PercentOutput, -1);
		frontRightSpeedController->Set(ControlMode::PercentOutput, 1);
		backRightSpeedController->Set(ControlMode::PercentOutput, 1);
		}
		pigeon->SetAccumZAngle(0,10);
		frontRightSpeedController->GetSensorCollection().SetQuadraturePosition(0,10);

	}

	void rightIfOtherTeamNoAuto() { //not going straight, starting on the right going to the left
		while (PulseWidth < calculateUnitsAuto(60)) {
			moveAutoForward();
		}
		turnNinetyLeft();
		while (PulseWidth < calculateUnitsAuto(120)) {
			moveAutoForward();
		}
		turnNinetyRight();
		while (PulseWidth < calculateUnitsAuto(60)) {
			moveAutoForward();
		}
		shootCubeAuto();
	}

	void toteTunnelIn() {
		while (myStick -> GetRawButton(3)) {
			leftToteTunnel -> Set(ControlMode::PercentOutput,-0.2);
			rightToteTunnel -> Set(ControlMode::PercentOutput,0.2);
			centerToteTunnel->Set(ControlMode::PercentOutput,0.2);
			leftIntakeSpeedController->Set(ControlMode::PercentOutput, 0.2);
			rightIntakeSpeedController->Set(ControlMode::PercentOutput, 0.2);
		}
	}

	void toteTunnelOut() {
		while (myStick -> GetRawButton(4)) {
			leftToteTunnel -> Set(ControlMode::PercentOutput,0.2);
			rightToteTunnel -> Set(ControlMode::PercentOutput,-0.2);
			centerToteTunnel->Set(ControlMode::PercentOutput,-0.2);
			leftIntakeSpeedController->Set(ControlMode::PercentOutput, -0.2);
			rightIntakeSpeedController->Set(ControlMode::PercentOutput, -0.2);


		}
	}

	void leftIfOtherTeamNoAuto() { //not going straight, starting on the left going to the right
		while (PulseWidth < calculateUnitsAuto(60)) {
			moveAutoForward();
		}
		turnNinetyRight();
		while (PulseWidth < calculateUnitsAuto(120)) {
			moveAutoForward();
		}
		turnNinetyLeft();
		while (PulseWidth < calculateUnitsAuto(60)) {
			moveAutoForward();
		}
		shootCubeAuto();
	}

	 void leftOtherTeamHasAuto() {	 //not going straight, starting on the left and going around the switch to the right
		 while (PulseWidth < calculateUnitsAuto(120))
		 {
			 moveAutoForward();
		 }
		 turnNinetyLeft();
		 while (PulseWidth < calculateUnitsAuto(65))
		 {
			 moveAutoForward();
		 }
		 turnNinetyRight();
		 while (PulseWidth <calculateUnitsAuto(110))
		 {
			 moveAutoForward();
		 }
		 turnNinetyRight();
		 while (PulseWidth<calculateUnitsAuto(264))
		 {
			 moveAutoForward();
		 }
		 turnNinetyLeft();
		 shootCubeAuto();
 	 }



	void rightSideIfOtherTeamAuto() { //not going straight, starting on the right and going around the switch to the left
			 while (PulseWidth < calculateUnitsAuto(120))
			 {
				 moveAutoForward();
			 }
			 turnNinetyRight();
			 while (PulseWidth < calculateUnitsAuto(65))
			 {
				 moveAutoForward();
			 }
			 turnNinetyLeft();
			 while (PulseWidth <calculateUnitsAuto(110))
			 {
				 moveAutoForward();
			 }
			 turnNinetyLeft();
			 while (PulseWidth<calculateUnitsAuto(264))
			 {
				 moveAutoForward();
			 }
			 turnNinetyLeft();
			 shootCubeAuto();
		  }

	void AutonomousInit() override {
		gameData = DriverStation::GetInstance().GetGameSpecificMessage();
		if(gameData.length() > 2)
		{
			if(gameData[0] == 'L') //switch the L to R if we start on the right
			{
				colorIsInFrontOfUsAuto();
			}
			else
			{
				//run leftOtherTeamHasAuto() or leftIfOtherTeamNoAuto()
			}
		//intake1 -> Set(DoubleSolenoid::Value::kForward);
		//intake2 -> Set(DoubleSolenoid::Value::kForward);
		}
	}

	void AutonomousPeriodic() {
		PulseWidth = frontRightSpeedController->GetSensorCollection().GetPulseWidthVelocity();
		myData->PutNumber("Pulse Width Counter", PulseWidth);
	}

	void TeleopInit() {
		pigeon->SetAccumZAngle(0,10);

	}

	void TeleopPeriodic() {
		torqueMode();
		speedMode();
		//mechIntakeOuttake();
		double mechValue = myStick->GetRawAxis(2);
		double mechValue1 = myStick->GetRawAxis(3);

		if (mechValue > 0) {
		leftIntakeSpeedController->Set(ControlMode::PercentOutput, (mechValue*mechValue));
		rightIntakeSpeedController->Set(ControlMode::PercentOutput, (mechValue*mechValue));
		centerToteTunnel->Set(ControlMode::PercentOutput, (-mechValue*mechValue));
		leftToteTunnel->Set(ControlMode::PercentOutput, (-mechValue*mechValue));
		rightToteTunnel->Set(ControlMode::PercentOutput, (mechValue*mechValue));
		}

		else {
			leftIntakeSpeedController->Set(ControlMode::PercentOutput, (0));
			rightIntakeSpeedController->Set(ControlMode::PercentOutput, (0));
			centerToteTunnel->Set(ControlMode::PercentOutput, (0));
			leftToteTunnel->Set(ControlMode::PercentOutput, (0));
			rightToteTunnel->Set(ControlMode::PercentOutput, (0));
		}
		if (mechValue1 > 0) {
		leftIntakeSpeedController->Set(ControlMode::PercentOutput, (-mechValue1*mechValue1));
		rightIntakeSpeedController->Set(ControlMode::PercentOutput, (-mechValue1*mechValue1));
		centerToteTunnel->Set(ControlMode::PercentOutput, (mechValue1*mechValue1));
		leftToteTunnel->Set(ControlMode::PercentOutput, (mechValue1*mechValue1));
		rightToteTunnel->Set(ControlMode::PercentOutput, (-mechValue1*mechValue1));
		mechValue1 = myStick->GetRawAxis(3);
		}

		//toteTunnelIn();
		//toteTunnelOut();
		moveRobotTeleop();
		pigeon->GetAccumGyro(gyroValues);
		myData->PutNumber("Gyro z", gyroValues[2]);
		myData->PutNumber("Gyro x", gyroValues[0]);
		myData->PutNumber("Gyro y", gyroValues[1]);
/*
		bool buttontester = myStick->GetRawButton(1);
		if(buttontester)
		{
			turnNinetyLeft();

		}
		pigeon->SetAccumZAngle(0,10);

*/
	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};
START_ROBOT_CLASS(Robot)
