#pragma once

#include "Disable.h"
#ifndef NO_ELEVATOR

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

/* Dedicated to the smell that grease and OTHER THINGS make */

class RepellerSubsystem: public frc2::SubsystemBase {
public:
	RepellerSubsystem();
	RepellerSubsystem(RepellerSubsystem &rhs) = delete;
	RepellerSubsystem& operator=(RepellerSubsystem &rhs) = delete;
	RepellerSubsystem(RepellerSubsystem &&rhs) = delete;
	RepellerSubsystem& operator=(RepellerSubsystem &&rhs) = delete;

	frc2::CommandPtr Forward();
	frc2::CommandPtr Backward();
	frc2::CommandPtr Stall();

private:
	ctre::phoenix::motorcontrol::can::TalonSRX m_wheel;
	double m_speed = 0.0;
};

#endif
