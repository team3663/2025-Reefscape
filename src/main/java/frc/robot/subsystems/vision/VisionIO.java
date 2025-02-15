package frc.robot.subsystems.vision;

public interface VisionIO {

    /**
     * @param inputs - VisionInputs object to update
     * @param currentYaw - Robot's current yaw in radians.
     */
    default void updateInputs(VisionInputs inputs, double currentYaw) {}

    default void robotStateChanged() {}
}
