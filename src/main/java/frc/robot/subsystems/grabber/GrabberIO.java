package frc.robot.subsystems.grabber;

public interface GrabberIO {
    default void updateInputs(GrabberInputs inputs) {
    }

    default void stop() {
        setTargetVoltage(0.0);
    }

    default void setTargetVoltage(double voltage) {
    }

    default boolean getBeamBreakState() {
        return true;
    }
}
