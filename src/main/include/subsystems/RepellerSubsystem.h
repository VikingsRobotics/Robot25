#pragma once

#include "Disable.h"
#ifndef NO_ELEVATOR

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

/* Dedicated to the smell that grease and OTHER THINGS make */

class RepellerSubsystem: public frc2::SubsystemBase {
public:
	RepellerSubsystem();
	RepellerSubsystem(RepellerSubsystem &rhs) = delete;
	RepellerSubsystem& operator=(RepellerSubsystem &rhs) = delete;
	RepellerSubsystem(RepellerSubsystem &&rhs) = delete;
	RepellerSubsystem& operator=(RepellerSubsystem &&rhs) = delete;

	void Periodic() override;

	void SetRepellerWheel(double speed);
	double GetRepellerWheelSpeed();

private:
	ctre::phoenix::motorcontrol::can::VictorSPX m_wheel;
};

#endif
