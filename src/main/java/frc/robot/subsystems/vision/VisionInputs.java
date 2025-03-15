package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;

@Logged
public class VisionInputs {
    public Pose2d estimatedPose = Pose2d.kZero;
    public double timestampSeconds;
    public int[] targetIds = new int[0];
    public boolean poseUpdated;
    public double IMUYaw;
    public double orientationDuration;
    public double imuDataDuration;
    public double poseEstimateDuration;
    public double filterDuration;
}
