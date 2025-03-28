#pragma once

#include "Disable.h"

#ifndef NO_ROLLER

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include <frc/DoubleSolenoid.h>

class RollerSubsystem: public frc2::SubsystemBase {
public:
	RollerSubsystem();
	RollerSubsystem(RollerSubsystem &rhs) = delete;
	RollerSubsystem& operator=(RollerSubsystem &rhs) = delete;
	RollerSubsystem(RollerSubsystem &&rhs) = delete;
	RollerSubsystem& operator=(RollerSubsystem &&rhs) = delete;

	void Periodic() override;

	void PutRollerDown();
	void PutRollerUp();

	bool IsRollerDown();
	bool IsRollerUp();

	void SetRollerWheel(double speed);
	double GetRollerWheelSpeed();
private:
	ctre::phoenix::motorcontrol::can::TalonSRX m_rollerWheel;
	frc::DoubleSolenoid m_solenoid;
};

#endif
