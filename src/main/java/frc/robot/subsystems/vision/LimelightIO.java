package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.LimelightHelpers;

@Logged
public class LimelightIO implements VisionIO {
    private static final int LIMELIGHT_IMU_EXTERNAL = 0;
    private static final int LIMELIGHT_IMU_FUSED = 1;
    private static final int LIMELIGHT_IMU_INTERNAL = 2;

    private final String cameraName;

    public LimelightIO(String name, Transform3d transform) {
        this.cameraName = name;

        // Initially the Limelight IMU should be in FUSED mode, it will change when robot is enabled.
        LimelightHelpers.SetIMUMode(cameraName, LIMELIGHT_IMU_FUSED);

        // Tell the limelight were on the robot it is located.
        Rotation3d rotation = transform.getRotation();
        LimelightHelpers.setCameraPose_RobotSpace(name,
                transform.getX(),
                transform.getY(),
                transform.getZ(),
                rotation.getX(),
                rotation.getY(),
                rotation.getZ());
    }

    public void updateInputs(VisionInputs visionInputs, double currentYaw) {
        // Assume pose will not be updated.
        visionInputs.poseUpdated = false;

        // Give the Limelight our current robot yaw as provided by the Pigeon.
        LimelightHelpers.SetRobotOrientation(cameraName, Units.radiansToDegrees(currentYaw), 0, 0, 0, 0, 0);

        // reading the yaw from the limelights internal IMU
        LimelightHelpers.IMUData imuData = LimelightHelpers.getIMUData(cameraName);
        visionInputs.IMUYaw = imuData.Yaw;

        // Get a new pose estimate
        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

        // If no tags were seen then return without doing anything.
        if (estimate.tagCount == 0)
            return;

        visionInputs.estimatedPose = estimate.pose;
        visionInputs.timestampSeconds = estimate.timestampSeconds;

        // Extract list of AprilTag Ids see in this pose estimate.
        int[] targetIds = new int[estimate.rawFiducials.length];
        int index = 0;
        for (LimelightHelpers.RawFiducial tag : estimate.rawFiducials) {
            targetIds[index++] = tag.id;
        }
        visionInputs.targetIds = targetIds;
        visionInputs.poseUpdated = true;
    }

    public void robotStateChanged() {
        // When the robot is disabled then seed the limelight's IMU with data from the Pigeon but once
        // the robot is enabled then switch to the Limelight's internal IMU.
        if (RobotState.isDisabled()) {
            LimelightHelpers.SetIMUMode(cameraName, LIMELIGHT_IMU_FUSED);
        } else {
            LimelightHelpers.SetIMUMode(cameraName, LIMELIGHT_IMU_INTERNAL);
        }
    }
}
