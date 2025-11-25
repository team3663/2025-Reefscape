package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Translation2d;

@Logged
public class VisionInputs2 {
    public Translation2d[] translations = new Translation2d[0];
    public int[] ids = new int[0];
    public double IMUYaw;
    public double orientationDuration;
    public double imuDataDuration;
    public double poseEstimateDuration;
    public double filterDuration;
}