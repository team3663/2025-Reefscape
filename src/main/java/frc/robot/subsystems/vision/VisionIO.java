package frc.robot.subsystems.vision;

import frc.robot.RobotMode;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public interface VisionIO {

    /**
     * @param inputs - VisionInputs object to update
     * @param currentYaw - Robot's current yaw in radians.
     */
    default void updateInputs(VisionInputs inputs, double currentYaw) {}
    default void robotStateChanged() {}

    default boolean isIgnoredIfNotNet(){ return false;}


}
