// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "RobotContainer.h"

class Robot: public frc::TimedRobot {
public:
	Robot();
	void RobotPeriodic() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void DisabledExit() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void AutonomousExit() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TeleopExit() override;
	void TestInit() override;
	void TestPeriodic() override;
	void TestExit() override;

private:
	frc2::Command *m_autonomousCommand;

	RobotContainer m_container;
};
