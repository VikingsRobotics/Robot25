#pragma once

#include "subsystems/SwerveSubsystem.h"

#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/sysid/SysIdRoutineLog.h>

struct SwerveSysIdRoutine {
	SwerveSysIdRoutine(SwerveSubsystem *subsystem);
	frc2::sysid::SysIdRoutine sysIdRoutineTranslation;
	frc2::sysid::SysIdRoutine sysIdRoutineRotation;
	frc2::sysid::SysIdRoutine sysIdRoutineSteer;
};
