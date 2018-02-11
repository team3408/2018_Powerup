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

	TalonSRX *frontLeftSpeedController, *backLeftSpeedController, *frontRightSpeedController, *backRightSpeedController, *leftIntakeSpeedController, *rightIntakeSpeedController, *centerToteTunnel, *leftToteTunnel, *rightToteTunnel;
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
	void RobotInit() {
		myStick = new Joystick(0);
		frontLeftSpeedController = new TalonSRX(2);
		frontRightSpeedController = new TalonSRX(1);
		backLeftSpeedController = new TalonSRX(0);
		backRightSpeedController = new TalonSRX(3);
		leftIntakeSpeedController = new TalonSRX(4);
		rightIntakeSpeedController = new TalonSRX(5);
		centerToteTunnel = new TalonSRX(6);
		rightToteTunnel = new TalonSRX(7);
		leftToteTunnel = new TalonSRX(8);
		frontRightSpeedController->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
		gearBoxShifter = new DoubleSolenoid(1,2);
		intake1 = new DoubleSolenoid(3,4);
		intake2 = new DoubleSolenoid(5,6);
		intake1 -> Set(DoubleSolenoid::Value::kReverse);
		intake2 -> Set(DoubleSolenoid::Value::kReverse);
	}

	void torqueMode() {
		solenoidForward = myStick->GetRawButton(5);  // LB
		if (solenoidForward) {
			gearBoxShifter->Set(DoubleSolenoid::Value::kForward);
		}
	}

	void mechIntakeOuttake() {
		double mechValue = myStick->GetRawAxis(3);
		leftIntakeSpeedController->Set(ControlMode::PercentOutput, mechValue);
		rightIntakeSpeedController->Set(ControlMode::PercentOutput, mechValue);
		centerToteTunnel->Set(ControlMode::PercentOutput, mechValue);
		leftToteTunnel->Set(ControlMode::PercentOutput, mechValue);
		rightToteTunnel->Set(ControlMode::PercentOutput, mechValue);
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
		double leftWheels = myStick->GetRawAxis(5);
		double rightWheels = myStick->GetRawAxis(1);
		frontLeftSpeedController->Set(ControlMode::PercentOutput, leftWheels);
		backLeftSpeedController->Set(ControlMode::PercentOutput, leftWheels);
		frontRightSpeedController->Set(ControlMode::PercentOutput, rightWheels);
		backRightSpeedController->Set(ControlMode::PercentOutput, rightWheels);
	}
	void moveAutoForward()
	{
		frontLeftSpeedController->Set(ControlMode::PercentOutput, 0.5);
		backLeftSpeedController->Set(ControlMode::PercentOutput, 0.5);
		frontRightSpeedController->Set(ControlMode::PercentOutput, 0.5);
		backRightSpeedController->Set(ControlMode::PercentOutput, 0.5);
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
		//while gyro != -90
		frontLeftSpeedController->Set(ControlMode::PercentOutput, -1);
		backLeftSpeedController->Set(ControlMode::PercentOutput, -1);
		frontRightSpeedController->Set(ControlMode::PercentOutput, 1);
		backRightSpeedController->Set(ControlMode::PercentOutput, 1);
		//Wait 0.3, Reset Gyro
	}
	void turnNinetyRight() //right wheels go forward, left wheels go back
	{
		//while gyro != 90
		frontLeftSpeedController->Set(ControlMode::PercentOutput, -1);
		backLeftSpeedController->Set(ControlMode::PercentOutput, -1);
		frontRightSpeedController->Set(ControlMode::PercentOutput, 1);
		backRightSpeedController->Set(ControlMode::PercentOutput, 1);
		//Wait 0.3, Reset Gyro
	}

	void rightIfOtherTeamNoAuto() { //not going straight, starting on the right going to the left
		while (PulseWidth < calculateUnitsAuto(60)) {
			moveAutoForward();
		}
		//Turn 90 degrees to the left
		//Reset PulseWidth
		while (PulseWidth < calculateUnitsAuto(120)) {
			moveAutoForward();
		}
		//Turn 90 degrees to the right
		//Reset PulseWidth
		while (PulseWidth < calculateUnitsAuto(60)) {
			moveAutoForward();
		}
		shootCubeAuto();
	}

	void leftIfOtherTeamNoAuto() { //not going straight, starting on the left going to the right
		while (PulseWidth < calculateUnitsAuto(60)) {
			moveAutoForward();
		}
		//Turn 90 degrees to the right
		//Reset PulseWidth
		while (PulseWidth < calculateUnitsAuto(120)) {
			moveAutoForward();
		}
		//Turn 90 degrees to the left
		//Reset PulseWidth
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
		 //Turn 90 left
		 PulseWidth = 0;
		 while (PulseWidth < calculateUnitsAuto(65))
		 {
			 moveAutoForward();
		 }
		 //Turn 90 right
		 PulseWidth=0;
		 while (PulseWidth <calculateUnitsAuto(110))
		 {
			 moveAutoForward();
		 }
		 //Turn 90 right
		 PulseWidth = 0;
		 while (PulseWidth<calculateUnitsAuto(264))
		 {
			 moveAutoForward();
		 }
		 //Turn 90 right
		 shootCubeAuto();
 	 }



	void rightSideIfOtherTeamAuto() { //not going straight, starting on the right and going around the switch to the left
			 while (PulseWidth < calculateUnitsAuto(120))
			 {
				 moveAutoForward();
			 }
			 //Turn 90 right
			 PulseWidth = 0;
			 while (PulseWidth < calculateUnitsAuto(65))
			 {
				 moveAutoForward();
			 }
			 //Turn 90 left
			 PulseWidth=0;
			 while (PulseWidth <calculateUnitsAuto(110))
			 {
				 moveAutoForward();
			 }
			 //Turn 90 left
			 PulseWidth = 0;
			 while (PulseWidth<calculateUnitsAuto(264))
			 {
				 moveAutoForward();
			 }
			 //Turn 90 left
			 shootCubeAuto();
		  }

	void AutonomousInit() override {
		gameData = DriverStation::GetInstance().GetGameSpecificMessage();
		if(gameData.length() > 0)
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

	void TeleopInit() {}

	void TeleopPeriodic() {
		torqueMode();
		speedMode();
		mechIntakeOuttake();
		moveRobotTeleop();
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
