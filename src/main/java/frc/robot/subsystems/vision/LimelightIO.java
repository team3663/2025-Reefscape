package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;

@Logged
public class LimelightIO implements VisionIO {
    private static final int LIMELIGHT_IMU_EXTERNAL = 0;
    private static final int LIMELIGHT_IMU_FUSED = 1;
    private static final int LIMELIGHT_IMU_INTERNAL = 2;

    private final String cameraName;
    private boolean isIgnoredNet;

    public LimelightIO(String name, Transform3d transform, boolean isIgnored) {
        this.cameraName = name;
        this.isIgnoredNet= isIgnored;


        // Initially the Limelight IMU should be in FUSED mode, it will change when robot is enabled.
        LimelightHelpers.SetIMUMode(cameraName, LIMELIGHT_IMU_EXTERNAL);

        // Tell the limelight were on the robot it is located.
        Rotation3d rotation = transform.getRotation();
        LimelightHelpers.setCameraPose_RobotSpace(name,
                transform.getX(),
                -transform.getY(),
                transform.getZ(),
                Units.radiansToDegrees(rotation.getX()),
                Units.radiansToDegrees(rotation.getY()),
                Units.radiansToDegrees(rotation.getZ()));
    }
    @Override
    public boolean isIgnoredIfNotNet(){
        return isIgnoredNet;
    }

    public void updateInputs(VisionInputs visionInputs, double currentYaw) {
        // Assume pose will not be updated.
        visionInputs.poseUpdated = false;

        // Give the Limelight our current robot yaw as provided by the Pigeon.
        double orientationStart = System.currentTimeMillis();
        LimelightHelpers.SetRobotOrientation(cameraName, Units.radiansToDegrees(currentYaw), 0, 0, 0, 0, 0);
        double orientationEnd = System.currentTimeMillis();
        visionInputs.orientationDuration = orientationEnd - orientationStart;

        // reading the yaw from the limelights internal IMU
        double imuStart = System.currentTimeMillis();
        LimelightHelpers.IMUData imuData = LimelightHelpers.getIMUData(cameraName);
        visionInputs.IMUYaw = imuData.Yaw;
        double imuEnd = System.currentTimeMillis();
        visionInputs.imuDataDuration = imuEnd - imuStart;

        // Get a new pose estimate
        double poseStart = System.currentTimeMillis();
        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
        double poseEnd = System.currentTimeMillis();
        visionInputs.poseEstimateDuration = poseEnd - poseStart;


        // processed if valid estimate.
        double filterStart = System.currentTimeMillis();
        if (estimate != null && estimate.tagCount != 0) {
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
        double filterEnd = System.currentTimeMillis();
        visionInputs.filterDuration = filterEnd - filterStart;

    }

    public void robotStateChanged() {
        // When the robot is disabled then seed the limelight's IMU with data from the Pigeon but once
        // the robot is enabled then switch to the Limelight's internal IMU.
//        if (RobotState.isDisabled()) {
//            LimelightHelpers.SetIMUMode(cameraName, LIMELIGHT_IMU_FUSED);
//        } else {
//            LimelightHelpers.SetIMUMode(cameraName, LIMELIGHT_IMU_INTERNAL);
//        }
    }
}
