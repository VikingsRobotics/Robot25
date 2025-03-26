#pragma once

#include "Disable.h"
#ifndef NO_VISION
#include "Constants.h"

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
#include <frc/Timer.h>

#include <atomic>

class VisionProvider {
private:
	static frc::AprilTagFieldLayout GetLayout();
public:
	VisionProvider();

	struct AprilTagTransform {
		int ID;
		frc::Transform3d relativePose;
	};

	struct AprilTagWithConfidence {
		float confidence;
		AprilTagTransform tag;
	};

	void ForcedProcess();
	std::vector<AprilTagTransform> GetValidRelativeAprilTags(
			float requiredConfidenced);
	std::optional<AprilTagWithConfidence> GetRelativeAprilTag(int ID);
	std::vector<frc::AprilTag> GetValidAprilTags(float requiredConfidenced);
	bool IsAprilTagInView(int id);

	inline static const frc::AprilTagFieldLayout fieldLayout = GetLayout();
private:

	/* Providers for the network tables */
	std::shared_ptr<nt::NetworkTable> m_tagTable;
	/*
	 | Raw Topic (Big Endian) |
	 | Num of April Tag In View : u8 | Flags (VALID | TIMES ) : u8 | FUTURE : u16 | 
	 ...
	 | (Num of April Tag)
	 \/
	 | Confidence  : f32   | April Tag Id : i32  |
	 | tx : f64 | ty : f64 | tz : f64 | rx : f64 | ry : f64 | rz : f64 |
	 translation is in meter_t
	 rotation is in radian_t
	 */
	nt::RawSubscriber m_tag;
	frc::Timer m_lastRead;

	/* DO NOT EDIT THIS VALUE OUTSIDE ProtectNetworkTable */
	std::array<AprilTagWithConfidence, Drive::Vision::kNumAprilTags> m_foundTags;

	std::array<uint8_t, 4> defaultPack { 0, 0, 0, 0 };

	wpi::mutex mutex;
	NT_Listener valueChange;

	std::function<void(void)> friended_swerveFunction { []() -> void {
		return;
	} };
	friend class SwerveSubsystem;
private:
	constexpr frc::Transform3d DecodeData(double tx, double ty, double tz,
			double rx, double ry, double rz);
	void ProcessData(std::span<const uint8_t> data, bool forced);
};

#endif
