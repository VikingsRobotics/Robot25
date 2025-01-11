// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "Constants.h"

#include "commands/SwerveControllerCommand.h"
#include "commands/SwerveJoystickCommand.h"

#include <frc2/command/RunCommand.h>

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() : joystick {
		Drive::ControllerPorts::kDriverControllerPort } {
	SwerveJoystickCommand joystickCommand { &swerveSubsystem, joystick, true };
	joystick.Button(frc::Joystick::ButtonType::kTopButton).OnTrue(
			frc2::RunCommand(
					[&joystickCommand]() {
						joystickCommand.SetFieldCentric(
								!joystickCommand.IsFieldCentric());
					}).ToPtr());
	swerveSubsystem.SetDefaultCommand(std::move(joystickCommand));

	ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return nullptr;
}
