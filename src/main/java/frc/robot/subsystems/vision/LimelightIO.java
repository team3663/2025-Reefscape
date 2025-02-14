package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

@Logged
public class LimelightIO implements VisionIO {

    private final String cameraName;

    public LimelightIO(String name, Transform3d transform){
        this.cameraName = name;

        // Tell the limelight were on the robot it is located.
        Rotation3d rotation = transform.getRotation();
        LimelightHelpers.setCameraPose_RobotSpace( name,
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

        // When the robot is disabled then seed the limelight's IMU with data from the Pigeon but once
        // the robot is enabled then switch to the Limelight's internal IMU.
        if (RobotState.isDisabled()) {
            // Seed the limelights IMU with yaw values from external IMU (Pigeon)
            LimelightHelpers.SetIMUMode(cameraName, 1);
        }
        else {
            LimelightHelpers.SetIMUMode(cameraName, 2);
        }

        // Give the Limelight our current robot yaw as provided by the IMU.
        LimelightHelpers.SetRobotOrientation(cameraName, Units.radiansToDegrees(currentYaw), 0, 0, 0, 0, 0);

        // Get a new post estimate
        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

        // If no tags were seen then return without doing anything.
        if ( estimate.tagCount == 0)
            return;

        visionInputs.estimatedPose = estimate.pose;
        visionInputs.timestampSeconds = estimate.timestampSeconds;

        // Extract list list of AprilTag Ids see in this pose estimate.
        int[] targetIds = new int[estimate.rawFiducials.length];
        int index = 0;
        for (LimelightHelpers.RawFiducial tag: estimate.rawFiducials ) {
            targetIds[index++] = tag.id;
        }
        visionInputs.targetIds = targetIds;
        visionInputs.poseUpdated = true;
    }
}
