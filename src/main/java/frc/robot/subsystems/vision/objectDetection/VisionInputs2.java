package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Translation2d;

@Logged
public class VisionInputs2 {
    public Translation2d[] poses;
    public int[] ids;
    public boolean[] poseUpdated;
    public double IMUYaw;
    public double orientationDuration;
    public double imuDataDuration;
    public double poseEstimateDuration;
    public double filterDuration;
}