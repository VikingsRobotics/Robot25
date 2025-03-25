#pragma once

#include <Constants.h>

#include <vector>

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/RawTopic.h>
#include <networktables/BooleanTopic.h>

#include <frc/apriltag/AprilTag.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include <wpi/mutex.h>

#include <atomic>

class VisionProvider {
public:
	VisionProvider();

	std::vector<frc::AprilTag> GetValidEstimatedAprilTags();
	std::vector<frc::AprilTag> GetValidAprilTags();
	bool IsAprilTagInView(int id);

	inline static const frc::AprilTagFieldLayout fieldLayout {
			frc::AprilTagFieldLayout::LoadField(
					frc::AprilTagField::k2025ReefscapeAndyMark) };
private:

	/* Providers for the network tables */
	std::shared_ptr<nt::NetworkTable> m_tagTable;
	/*
	 | Raw Topic (Big Endian) |
	 | Num of April Tag In View : u8 | Num of Pose Estimations : u8 | FUTURE : u16 | 
	 ...
	 | (Num of April Tag)
	 \_/
	 | April Tag Id : i32 | Offset to Pose Estimation : i32 (0 is no estimation) |
	 ...
	 | (Num of Pose Estimations)
	 \_/ 
	 | tx : f64 | ty : f64 | tz : f64 | rx : f64 | ry : f64 | rz : f64 |
	 translation is in meter_t
	 rotation is in radian_t
	 */
	nt::RawSubscriber m_tag;

	/* Current cached values so no data races */
	std::atomic<bool> m_tagsAreReady = false;
	struct AprilTagWithConfidence {
		frc::AprilTag tag;
		float confidence;
	};
	/* DO NOT EDIT THIS VALUE OUTSIDE ProtectNetworkTable */
	std::array<AprilTagWithConfidence, Drive::Vision::kNumAprilTags> m_foundTags;

	struct NetworkPose {
		double meterTranslationX;
		double meterTranslationY;
		double meterTranslationZ;
		double radianRotationX;
		double radianRotationY;
		double radianRotationZ;
	};

	struct NetworkAprilId {
		int32_t tagId;
		NetworkPose *pose;
	};

	struct NetworkPack {
		uint8_t numOfAprilTags;
		uint8_t numOfPoseEst;
		uint16_t unused;
		NetworkAprilId *listOfTags;
		NetworkPose *listOfPoses;
	};

	NetworkPack m_recievedPack {
		.numOfAprilTags = 0,
		.numOfPoseEst = 0,
		.unused = 0,
		.listOfTags = nullptr,
		.listOfPoses = nullptr};
	std::array<NetworkAprilId, Drive::Vision::kNumAprilTags> m_aprilIds { };
	std::array<NetworkPose, Drive::Vision::kNumAprilTags> m_aprilPoses { };

	std::array<uint8_t, 4> defaultPack { 0, 0, 0, 0 };

	wpi::mutex mutex;
	NT_Listener valueChange;
private:
	constexpr frc::Pose3d GetPose3dFromVisionTable(NetworkPose netPose);
};
