#include "commands/SwerveAlignAprilTagCommand.h"

#ifndef NO_SWERVE_ALIGN_APRILTAG_COMMAND

#include "Constants.h"

SwerveAlignAprilTagCommand::SwerveAlignAprilTagCommand(
		SwerveSubsystem *const subsystem, bool rightReef) : m_subsystem {
		subsystem }, m_rightReef { rightReef }, m_xTranslationController {
		Drive::Align::System::kXTranslationP, 0, 0 }, m_yTranslationController {
		Drive::Align::System::kYTranslationP, 0, 0 }, m_rotationController {
		Drive::Align::System::kRotationP, 0, 0 } {
	AddRequirements(subsystem);
	SetName("Swerve Vision Alignment Command");
}

void SwerveAlignAprilTagCommand::Initialize() {
	m_xTranslationController.SetSetpoint(
			Drive::Align::Location::kXTranslationEndpoint.value());
	m_xTranslationController.SetTolerance(
			Drive::Align::Location::kXTranslationThreshold.value());

	m_yTranslationController.SetSetpoint(
			Drive::Align::Location::kYTranslationEndpoint.value());
	m_yTranslationController.SetTolerance(
			Drive::Align::Location::kYTranslationThreshold.value());

	m_rotationController.SetSetpoint(
			Drive::Align::Location::kRotationEndpoint.value());
	m_rotationController.SetTolerance(
			Drive::Align::Location::kRotationThreshold.value());

	m_lostTag.Reset();
	m_poseReady.Reset();

	m_tagId = FindClosestTagId();
}

void SwerveAlignAprilTagCommand::Execute() {
	if (!m_subsystem->IsAprilTagInView(m_tagId)) {
		m_subsystem->Drive(frc::ChassisSpeeds { .vx = 0_mps, .vy = 0_mps,
				.omega = 0_rad_per_s });
		return;
	}
	m_lostTag.Reset();

	frc::Pose2d target = m_subsystem->GetPose2d();
	target =
			target.RelativeTo(
					m_subsystem->fieldLayout.GetTagPose(m_tagId).value_or(
							target).ToPose2d());

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
}

bool SwerveAlignAprilTagCommand::IsFinished() {
	return m_lostTag.HasElapsed(Drive::Align::Time::kTagOutOfViewTime)
			|| m_poseReady.HasElapsed(Drive::Align::Time::kPoseValidation);
}

int SwerveAlignAprilTagCommand::FindClosestTagId() {
	std::vector < frc::AprilTag > aprils = m_subsystem->GetValidAprilTags();
	if (aprils.size() == 0) {
		return -1;
	}
	frc::AprilTag closest = aprils.at(0);
	frc::Pose2d robotPos = m_subsystem->GetPose2d();
	for (const frc::AprilTag &tag : aprils) {
		if (robotPos.Translation().Distance(
				closest.pose.ToPose2d().Translation())
				< robotPos.Translation().Distance(
						tag.pose.ToPose2d().Translation())) {
			closest = tag;
		}
	}
	return closest.ID;
}

#endif
