package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionInputs {
    public Pose2d estimatedPose = Pose2d.kZero;
    public double timestampSeconds;
    public int[] targetIds;
    public boolean poseUpdated;
}
