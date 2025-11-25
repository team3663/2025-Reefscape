package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO2 {

    /**
     * @param visionInputs     - VisionInputs object to update
     * @param robotPose - Robot's current position.
     */
    default void updateInputs(VisionInputs2 visionInputs, Pose2d robotPose) {
    }

    default void robotStateChanged() {
    }

    default boolean isIgnoredIfNotNet() {
        return false;
    }
}