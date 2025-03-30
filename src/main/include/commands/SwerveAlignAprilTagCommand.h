#include "Disable.h"

#ifndef NO_SWERVE_ALIGN_APRILTAG_COMMAND

#include "subsystems/SwerveSubsystem.h"
#include "subsystems/VisionProvider.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Transform2d.h>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <frc/Timer.h>

class SwerveAlignAprilTagCommand: public frc2::CommandHelper<frc2::Command,
		SwerveAlignAprilTagCommand> {
public:
	SwerveAlignAprilTagCommand(SwerveSubsystem *const subsystem,
			VisionProvider *const vision, frc::Transform2d aprilOffset,
			frc::Transform2d tolerance, double translationP,
			double translationI, double translationD, double rotationP,
			double rotationI, double rotationD);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;
private:
	int FindClosestTagId();
private:
	SwerveSubsystem *const m_subsystem;
	VisionProvider *const m_vision;
	frc::PIDController m_xTranslationController;
	frc::PIDController m_yTranslationController;
	frc::PIDController m_rotationController;
	int m_tagId = -1;

	frc::Timer m_lostTag { };
	frc::Timer m_poseReady { };
};

#endif
