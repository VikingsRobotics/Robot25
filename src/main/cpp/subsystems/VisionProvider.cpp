#include "subsystems/VisionProvider.h"

#ifndef NO_VISION

#include <wpi/Endian.h>
#include "subsystems/SwerveSubsystem.h"

#ifdef NO_SWERVE
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
#else
VisionProvider::VisionProvider(SwerveSubsystem &odomentryCallback) : m_tagTable {
		nt::NetworkTableInstance::GetDefault().GetTable("april-tag") }, m_tag {
		m_tagTable->GetRawTopic("tag").Subscribe("AprilTagWithConfidence",
				std::span<uint8_t, 4>(defaultPack)) }, m_swerveSubsystem {
		odomentryCallback } {
	valueChange = nt::NetworkTableInstance::GetDefault().AddListener(m_tag,
			nt::EventFlags::kValueAll, [this](const nt::Event &event) {
				std::scoped_lock lock { mutex };
				ProcessData(event.GetValueEventData()->value.GetRaw(), false);
			}
	);

	m_lastRead.Reset();
}
#endif

VisionProvider::~VisionProvider() {
	nt::NetworkTableInstance::GetDefault().RemoveListener(valueChange);
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
				return tag.confidence > requiredConfidenced || tag.tag.ID == 0
						|| !fieldLayout.GetTagPose(tag.tag.ID).has_value();
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
				return tag.confidence > requiredConfidenced || tag.tag.ID == 0
						|| !fieldLayout.GetTagPose(tag.tag.ID).has_value();
			});

	std::vector < frc::AprilTag > aprilTagPositions;
	std::transform(aprilTags.begin(), aprilTags.end(),
			aprilTagPositions.begin(),
			[](const AprilTagWithConfidence &input) -> frc::AprilTag {
				std::optional < frc::Pose3d > pose = fieldLayout.GetTagPose(
						input.tag.ID);
				return frc::AprilTag { .ID = input.tag.ID, .pose =
						pose.value_or(frc::Pose3d { }) };
			});

	return aprilTagPositions;
}

std::optional<VisionProvider::AprilTagWithConfidence> VisionProvider::GetRelativeAprilTag(
		int ID) {
	if (ID == 0 || ID > static_cast<int>(Drive::Vision::kNumAprilTags + 1)) {
		return std::nullopt;
	}
	std::scoped_lock lock { mutex };
	// All tags are one less in the array since arrays are at zero but the tags start at one
	AprilTagWithConfidence returningTag = m_foundTags[ID - 1];
	// tag ID is stored as 0 if not found
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
	using namespace wpi::support::endian;
	uint8_t numOfAprilTag = data[0];
	uint8_t flags = data[1];
	constexpr uint8_t VALID = 0b10000000;
	constexpr uint8_t TIMES = 0b01111111;

	m_lastValid = (flags & VALID);
	// unused (16 bits)

	if (!forced
			&& (!m_lastValid || numOfAprilTag == 0
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
	const uint8_t *at = data.data() + 4;
	// @formatter:off 
	for (size_t index = 0; index < numOfAprilTag; ++index) {
		m_foundTags[static_cast<size_t>(read32be(at + (4 + 56 * index)))] =
			AprilTagWithConfidence { 
				.confidence = std::bit_cast<float>(read32be(at + (0 + 56 * index))),
				.tag = AprilTagTransform { 
					.ID = std::bit_cast<int>(read32be(at + (4 + 56 * index))),
					.relativePose =	DecodeData(
						std::bit_cast<double>(read64be(at + (8 + 56 * index))),
						std::bit_cast<double>(read64be(at + (16 + 56 * index))),
						std::bit_cast<double>(read64be(at + (24 + 56 * index))),
						std::bit_cast<double>(read64be(at + (32 + 56 * index))),
						std::bit_cast<double>(read64be(at + (40 + 56 * index))),
						std::bit_cast<double>(read64be(at + (48 + 56 * index)))) 
				} 
			};
	}
	// @formatter:on
	m_lastRead.Reset();
#ifndef NO_SWERVE
	m_swerveSubsystem.AddBestEstimates(*this,
			GetValidRelativeAprilTags(Drive::Vision::kRequiredConfidence));
#endif
}

bool VisionProvider::ForcedProcess() {
	std::scoped_lock lock { mutex };
	ProcessData(m_tag.Get(std::span<uint8_t, 4> { defaultPack }), true);
	return m_lastValid;
}

frc::AprilTagFieldLayout VisionProvider::GetLayout() {
	frc::AprilTagFieldLayout layout = frc::AprilTagFieldLayout::LoadField(
			frc::AprilTagField::k2025ReefscapeWelded);
	layout.SetOrigin(
			frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
	return layout;
}

#endif
