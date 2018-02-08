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
class Robot : public frc::IterativeRobot {
public:

	TalonSRX *frontLeftSpeedController, *backLeftSpeedController, *frontRightSpeedController, *backRightSpeedController;
	Joystick *myStick;
	SmartDashboard *myData;
	double unitsToSwitch = calculateUnitsAuto(140);
	double PulseWidth = 0;



	double calculateUnitsAuto(int distance)
	{
		double units = (distance/(6*M_PI))*4096;
		return units;
	}

	void moveForward()
	{
		frontLeftSpeedController->Set(ControlMode::PercentOutput, 0.5);
		backLeftSpeedController->Set(ControlMode::PercentOutput, 0.5);
		frontRightSpeedController->Set(ControlMode::PercentOutput, 0.5);
		backRightSpeedController->Set(ControlMode::PercentOutput, 0.5);
	}

	void RobotInit() {
		myStick = new Joystick(0);
		frontLeftSpeedController = new TalonSRX(2);
		frontRightSpeedController = new TalonSRX(1);
		backLeftSpeedController = new TalonSRX(0);
		backRightSpeedController = new TalonSRX(3);
		frontRightSpeedController->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
	}

	void AutonomousInit() override {
	}
	void AutonomousPeriodic() {
		PulseWidth = frontRightSpeedController->GetSensorCollection().GetPulseWidthVelocity();
		myData->PutNumber("Pulse Width Counter", PulseWidth);
		while (PulseWidth < unitsToSwitch)
		{
			moveForward();
		}
		//(robot has stopped) shoot cube
	}

	void TeleopInit() {}

	void TeleopPeriodic() {

		double leftWheels = myStick->GetRawAxis(5);
		double rightWheels = myStick->GetRawAxis(1);
		frontLeftSpeedController->Set(ControlMode::PercentOutput, leftWheels);
		backLeftSpeedController->Set(ControlMode::PercentOutput, leftWheels);
		frontRightSpeedController->Set(ControlMode::PercentOutput, rightWheels);
		backRightSpeedController->Set(ControlMode::PercentOutput, rightWheels);

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
