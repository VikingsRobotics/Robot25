#include "subsystems/VisionProvider.h"
#ifdef NON
VisionProvider::VisionProvider() :
 m_tagTable{ nt::NetworkTableInstance::GetDefault().GetTable("april-tag") }, 
 m_tag{ m_tagTable->GetRawTopic("tag").Subscribe("april",std::span<uint8_t,4>(defaultPack)) } {
	valueChange = nt::NetworkTableInstance::GetDefault().AddListener(
		m_tag,
		nt::EventFlags::kValueAll,
		[this](const nt::Event& event) {
          // can only get doubles because it's a DoubleSubscriber, but
          // could check value.IsDouble() here too
          std::scoped_lock lock{mutex};
          //yValue = event.GetValueData()->value.GetDouble();
          //yValueUpdated = true;
        }
	);
}

constexpr frc::Pose3d VisionProvider::GetPose3dFromVisionTable(
		NetworkPose netPose) {
	/* Stored in the following format from 
	 https://github.com/wpilibsuite/allwpilib/blob/main/wpilibcExamples/src/main/cpp/examples/AprilTagsVision/cpp/Robot.cpp 
	 
	 {	x : double <-- meter_t, 	y: double <-- meter_t, 		z: double <-- meter_t 
	 x : double <-- rotation_t, 	y : double <-- rotation_t, 	z : double <-- rotation_t }
	 */

	return frc::Pose3d { units::meter_t { netPose.meterTranslationX }, units::meter_t {
			netPose.meterTranslationY }, units::meter_t { netPose.meterTranslationZ },
			frc::Rotation3d { units::radian_t { netPose.radianRotationX },
					units::radian_t { netPose.radianRotationY }, units::radian_t {
							netPose.radianRotationZ } } };
}

std::vector<frc::AprilTag> VisionProvider::GetValidEstimatedAprilTags() {
	
}

std::vector<frc::AprilTag> VisionProvider::GetValidAprilTags() {
	
}

bool VisionProvider::IsAprilTagInView(int id) {
	if (id <= 0 || id > static_cast<int>(Drive::Vision::kNumAprilTags + 1)) {
		return false;
	}

	std::array<AprilTagWithConfidence, Drive::Vision::kNumAprilTags>::const_iterator isFound =
			std::find_if(std::cbegin(m_foundTags), std::cend(m_foundTags),
					[id](const AprilTagWithConfidence &val) {
						return val.tag.ID == id;
					});

	return isFound != std::cend(m_foundTags);
}

#endif
