#include "subsystems/VisionProvider.h"

#ifndef NO_VISION

#include <endian.h>

VisionProvider::VisionProvider() : m_tagTable {
		nt::NetworkTableInstance::GetDefault().GetTable("april-tag") }, m_tag {
		m_tagTable->GetRawTopic("tag").Subscribe("AprilTagWithConfidence",
				std::span<uint8_t, 4>(defaultPack)) } {
	valueChange = nt::NetworkTableInstance::GetDefault().AddListener(m_tag,
			nt::EventFlags::kValueAll, [this](const nt::Event &event) {
				std::scoped_lock lock { mutex };
				ProcessData(event.GetValueEventData()->value.GetRaw(), false);
			}
	);

	m_lastRead.Reset();
}

constexpr frc::Transform3d VisionProvider::DecodeData(double tx, double ty,
		double tz, double rx, double ry, double rz) {
	/* Stored in the following format from 
	 https://github.com/wpilibsuite/allwpilib/blob/main/wpilibcExamples/src/main/cpp/examples/AprilTagsVision/cpp/Robot.cpp 
	 
	 {	x : double <-- meter_t, 	y: double <-- meter_t, 		z: double <-- meter_t 
	 x : double <-- rotation_t, 	y : double <-- rotation_t, 	z : double <-- rotation_t }
	 */

	return frc::Transform3d { frc::Translation3d { units::meter_t { tx },
			units::meter_t { ty }, units::meter_t { tz } }, frc::Rotation3d {
			units::radian_t { rx }, units::radian_t { ry },
			units::radian_t { rz } } };
}

std::vector<VisionProvider::AprilTagTransform> VisionProvider::GetValidRelativeAprilTags(
		float requiredConfidenced) {
	std::scoped_lock lock { mutex };
	std::vector<AprilTagWithConfidence> aprilTags(std::begin (m_foundTags),
			std::end (m_foundTags));
	std::erase_if(aprilTags,
			[requiredConfidenced](const AprilTagWithConfidence &tag) {
				return tag.confidence > requiredConfidenced || tag.tag.ID == 0;
			});

	std::vector < AprilTagTransform > relativePositions;
	std::transform(aprilTags.begin(), aprilTags.end(),
			relativePositions.begin(),
			[](const AprilTagWithConfidence &input) -> AprilTagTransform {
				return input.tag;
			});

	return relativePositions;
}

std::vector<frc::AprilTag> VisionProvider::GetValidAprilTags(
		float requiredConfidenced) {
	std::scoped_lock lock { mutex };
	std::vector<AprilTagWithConfidence> aprilTags(std::begin (m_foundTags),
			std::end (m_foundTags));
	std::erase_if(aprilTags,
			[requiredConfidenced](const AprilTagWithConfidence &tag) {
				return tag.confidence > requiredConfidenced || tag.tag.ID == 0;
			});

	std::vector < frc::AprilTag > aprilTagPositions;
	std::transform(aprilTags.begin(), aprilTags.end(),
			aprilTagPositions.begin(),
			[this](const AprilTagWithConfidence &input) -> frc::AprilTag {
				std::optional < frc::Pose3d > pose = fieldLayout.GetTagPose(
						input.tag.ID);
				return pose.has_value() ? frc::AprilTag { .ID = input.tag.ID,
													.pose = pose.value() } :
											frc::AprilTag { .ID = 0, .pose =
													frc::Pose3d { } };
			});
	std::erase_if(aprilTagPositions, [](const frc::AprilTag &tag) {
		return tag.ID == 0;
	});

	return aprilTagPositions;
}

std::optional<VisionProvider::AprilTagWithConfidence> VisionProvider::GetRelativeAprilTag(
		int ID) {
	if (ID == 0 || ID > static_cast<int>(Drive::Vision::kNumAprilTags + 1)) {
		return std::nullopt;
	}
	std::scoped_lock lock { mutex };
	AprilTagWithConfidence returningTag = m_foundTags[ID - 1];
	return returningTag.tag.ID == ID ?
			std::optional<VisionProvider::AprilTagWithConfidence> { returningTag } :
			std::nullopt;
}

bool VisionProvider::IsAprilTagInView(int id) {
	if (id <= 0 || id > static_cast<int>(Drive::Vision::kNumAprilTags + 1)) {
		return false;
	}
	std::scoped_lock lock { mutex };
	AprilTagWithConfidence tag = m_foundTags[id - 1];
	return tag.tag.ID == id;
}

void VisionProvider::ProcessData(std::span<const uint8_t> data, bool forced) {
	uint8_t numOfAprilTag = data[0];
	uint8_t flags = data[1];
	constexpr uint8_t VALID = 0b10000000;
	constexpr uint8_t TIMES = 0b01111111;
	// unused
	/*uint16_t unused = static_cast<uint16_t>(be16toh(
	 *reinterpret_cast<uint16_t*>(data[2])));*/

	if (!forced
			&& (!(flags & VALID) || numOfAprilTag == 0
					|| numOfAprilTag > Drive::Vision::kNumAprilTags)) {
		if ((flags & TIMES) % Drive::Vision::kProcessDataOnNth != 0) {
			if (!m_lastRead.HasElapsed(Drive::Vision::kProcessDataEvery)) {
				return;
			}
		}
	}

	std::fill(std::begin(m_foundTags), std::end(m_foundTags),
			AprilTagWithConfidence { .confidence = 0, .tag = AprilTagTransform {
					.ID = 0, .relativePose = frc::Transform3d { } } });

	for (size_t index = 0; index < numOfAprilTag; ++index) {
		m_foundTags[static_cast<uint32_t>(be32toh(
				*reinterpret_cast<uint32_t*>(data[8 + 56 * index])))] =
				AprilTagWithConfidence {
						.confidence =
								*reinterpret_cast<float*>(be32toh(
										*reinterpret_cast<uint32_t*>(data[4
												+ 56 * index]))),
						.tag =
								AprilTagTransform {
										.ID =
												*reinterpret_cast<int*>(be32toh(
														*reinterpret_cast<uint32_t*>(data[8
																+ 56 * index]))),
										.relativePose =
												DecodeData(
														*reinterpret_cast<double*>(be64toh(
																*reinterpret_cast<uint64_t*>(data[12
																		+ 56
																				* index]))),
														*reinterpret_cast<double*>(be64toh(
																*reinterpret_cast<uint64_t*>(data[20
																		+ 56
																				* index]))),
														*reinterpret_cast<double*>(be64toh(
																*reinterpret_cast<uint64_t*>(data[28
																		+ 56
																				* index]))),
														*reinterpret_cast<double*>(be64toh(
																*reinterpret_cast<uint64_t*>(data[36
																		+ 56
																				* index]))),
														*reinterpret_cast<double*>(be64toh(
																*reinterpret_cast<uint64_t*>(data[44
																		+ 56
																				* index]))),
														*reinterpret_cast<double*>(be64toh(
																*reinterpret_cast<uint64_t*>(data[52
																		+ 56
																				* index])))) } };
	}
	if (friended_swerveFunction)
		friended_swerveFunction();
	m_lastRead.Reset();
}

void VisionProvider::ForcedProcess() {
	std::scoped_lock lock { mutex };
	ProcessData(m_tag.Get(std::span<uint8_t, 4> { defaultPack }), true);
}

frc::AprilTagFieldLayout VisionProvider::GetLayout() {
	frc::AprilTagFieldLayout layout = frc::AprilTagFieldLayout::LoadField(
			frc::AprilTagField::k2025ReefscapeAndyMark);
	layout.SetOrigin(
			frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
	return layout;
}

#endif
