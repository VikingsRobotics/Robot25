#include "commands/SwerveAlignAprilTagCommand.h"

#ifndef NO_SWERVE_ALIGN_APRILTAG_COMMAND

#include "Constants.h"
#include "subsystems/VisionProvider.h"

SwerveAlignAprilTagCommand::SwerveAlignAprilTagCommand(
		SwerveSubsystem *const subsystem, VisionProvider *const vision,
		frc::Transform2d aprilOffset, frc::Transform2d tolerance,
		double translationP, double translationI, double translationD,
		double rotationP, double rotationI, double rotationD) : m_subsystem {
		subsystem }, m_vision { vision }, m_xTranslationController {
		translationP, translationI, translationD }, m_yTranslationController {
		translationP, translationI, translationD }, m_rotationController {
		rotationP, rotationI, rotationD } {

	m_xTranslationController.SetSetpoint(aprilOffset.X().value());
	m_xTranslationController.SetTolerance(tolerance.X().value());

	m_yTranslationController.SetSetpoint(aprilOffset.Y().value());
	m_yTranslationController.SetTolerance(tolerance.Y().value());

	m_rotationController.SetSetpoint(aprilOffset.Rotation().Radians().value());
	m_rotationController.SetTolerance(tolerance.Rotation().Radians().value());

	AddRequirements(subsystem);
	SetName("Swerve Vision Alignment Command");
}

void SwerveAlignAprilTagCommand::Initialize() {
	m_lostTag.Reset();
	m_poseReady.Reset();

	m_tagId = FindClosestTagId();
	if (m_tagId <= 0) {
		this->Cancel();
	}
}

void SwerveAlignAprilTagCommand::Execute() {
	if (!m_vision->ForcedProcess() || !m_vision->IsAprilTagInView(m_tagId)) {
		m_subsystem->Drive(frc::ChassisSpeeds { .vx = 0_mps, .vy = 0_mps,
				.omega = 0_rad_per_s });
		return;
	}

	frc::Pose2d target { };
	{
		std::optional < VisionProvider::AprilTagWithConfidence > aprilTag =
				m_vision->GetRelativeAprilTag(m_tagId);
		if (!aprilTag.has_value()
				|| aprilTag.value().confidence
						< Drive::Vision::kRequiredConfidence) {
			m_subsystem->Drive(frc::ChassisSpeeds { .vx = 0_mps, .vy = 0_mps,
					.omega = 0_rad_per_s });
			return;
		}

		target =
				frc::Pose2d {
						aprilTag.value().tag.relativePose.Translation().ToTranslation2d(),
						aprilTag.value().tag.relativePose.Rotation().ToRotation2d() };
	}

	m_lostTag.Reset();
	m_subsystem->Drive(
			frc::ChassisSpeeds { .vx = units::meters_per_second_t {
					m_xTranslationController.Calculate(target.X().value()) },
					.vy = units::meters_per_second_t {
							m_yTranslationController.Calculate(
									target.Y().value()) },
					.omega = units::radians_per_second_t {
							m_rotationController.Calculate(
									target.Rotation().Radians().value()) } });

	if (!m_rotationController.AtSetpoint()
			|| !m_yTranslationController.AtSetpoint()
			|| !m_xTranslationController.AtSetpoint()) {
		m_poseReady.Reset();
	}
}

void SwerveAlignAprilTagCommand::End(bool interrupted) {
	m_subsystem->Drive(frc::ChassisSpeeds { .vx = 0_mps, .vy = 0_mps, .omega =
			0_rad_per_s });
	m_xTranslationController.Reset();
	m_yTranslationController.Reset();
	m_rotationController.Reset();
}

bool SwerveAlignAprilTagCommand::IsFinished() {
	return m_lostTag.HasElapsed(Drive::Align::Time::kTagOutOfViewTime)
			|| m_poseReady.HasElapsed(Drive::Align::Time::kPoseValidation);
}

int SwerveAlignAprilTagCommand::FindClosestTagId() {
	if (!m_vision->ForcedProcess()) {
		return -1;
	}
	std::vector < VisionProvider::AprilTagTransform > aprils =
			m_vision->GetValidRelativeAprilTags(
					Drive::Vision::kRequiredConfidence);
	if (aprils.size() == 0) {
		return -1;
	}
	VisionProvider::AprilTagTransform closest = aprils[0];
	for (const VisionProvider::AprilTagTransform &tag : aprils) {
		if (tag.relativePose.Translation().Norm()
				< closest.relativePose.Translation().Norm()) {
			closest = tag;
		}
	}
	return closest.ID;
}

#endif
