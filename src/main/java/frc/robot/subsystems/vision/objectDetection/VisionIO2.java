package frc.robot.subsystems.vision.objectDetection;

public interface VisionIO2 {

    /**
     * @param inputs     - VisionInputs object to update
     * @param currentYaw - Robot's current yaw in radians.
     */
    default void updateInputs(VisionInputs2 inputs, double currentYaw) {
    }

    default void robotStateChanged() {
    }

    default boolean isIgnoredIfNotNet() {
        return false;
    }
}