package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
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

        LimelightHelpers.SetRobotOrientation(cameraName, Units.radiansToDegrees(currentYaw), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

        // If no tags were seen then return without doing anything.
        if ( mt2.tagCount == 0)
            return;

        visionInputs.estimatedPose = mt2.pose;
        visionInputs.timestampSeconds = mt2.timestampSeconds;

        // Extract list list of AprilTag Ids see in this pose estimate.
        int[] targetIds = new int[mt2.rawFiducials.length];
        int index = 0;
        for (LimelightHelpers.RawFiducial tag: mt2.rawFiducials ) {
            targetIds[index++] = tag.id;
        }
        visionInputs.targetIds = targetIds;
        visionInputs.poseUpdated = true;
    }
}
