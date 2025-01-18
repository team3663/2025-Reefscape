package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {

    VisionMeasurement currentMeasurement = new VisionMeasurement(Pose2d.kZero, 0, VecBuilder.fill(0,0,0));

    private Vision() {
    }

    @Override
    public void periodic() {

        //  Need the robot's current yaw as an input to vision calculations
        double yawDegrees = 0;

        LimelightHelpers.SetRobotOrientation("limelight", yawDegrees, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        boolean doRejectUpdate = false;
        if (mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }

        if(!doRejectUpdate)
        {
            currentMeasurement.estimatedPose = mt2.pose;
            currentMeasurement.timestamp = mt2.timestampSeconds;
            currentMeasurement.stdDevs = VecBuilder.fill(0,0,0);
        }
    }
}

