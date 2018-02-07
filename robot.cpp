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
using namespace std;
class Robot : public frc::IterativeRobot {
public:

	TalonSRX *frontLeftSpeedController, *backLeftSpeedController, *frontRightSpeedController, *backRightSpeedController;
	Joystick *myStick;

	void RobotInit() {
		myStick = new Joystick(0);
		frontLeftSpeedController = new TalonSRX(2);
		frontRightSpeedController = new TalonSRX(1);
		backLeftSpeedController = new TalonSRX(0);
		backRightSpeedController = new TalonSRX(3);
		frontLeftSpeedController->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);

	}
	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override {
		int pulseWidth = frontLeftSpeedController->GetSensorCollection().GetPulseWidthVelocity();

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {}

	void TeleopPeriodic() {
		double leftWheels = myStick->GetRawAxis(5);
		double rightWheels = myStick->GetRawAxis(1);
		frontLeftSpeedController->Set(ControlMode::PercentOutput, -leftWheels);
		backLeftSpeedController->Set(ControlMode::PercentOutput, -leftWheels);
		frontRightSpeedController->Set(ControlMode::PercentOutput, rightWheels);
		backRightSpeedController->Set(ControlMode::PercentOutput, rightWheels);
	}

	void TestPeriodic() {

	}

private:

};
START_ROBOT_CLASS(Robot)
