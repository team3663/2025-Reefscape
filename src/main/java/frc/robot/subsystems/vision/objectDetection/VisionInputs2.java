package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;

@Logged
public class VisionInputs2 {
    public Pose2d estimatedPose = Pose2d.kZero;
    public double timestampSeconds;
    public boolean poseUpdated;
    public double IMUYaw;
    public double orientationDuration;
    public double imuDataDuration;
    public double poseEstimateDuration;
    public double filterDuration;
    public double ta;
    public double txnc;
    public double tync;
}