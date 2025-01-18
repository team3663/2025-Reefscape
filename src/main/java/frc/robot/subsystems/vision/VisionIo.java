package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIo {
    default void updateInputs(VisionInputs inputs) {
    }

    class VisionInputs {
        public Pose3d estimatedPose = new Pose3d();
        public double timestampSeconds;
        public int[] targetIds;
        public boolean poseUpdated;
    }
}
