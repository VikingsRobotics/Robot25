#pragma once

#include "Disable.h"

#ifndef NO_ROLLER

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <rev/SparkMax.h>

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
	rev::spark::SparkMax m_rollerWheel;
	frc::DoubleSolenoid m_solenoid;
};

#endif
