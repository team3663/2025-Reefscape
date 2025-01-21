package frc.robot.subsystems.wrist;

public interface WristIO {
    default void updateInputs(WristInputs inputs) {
    }

    default void stop() {
        setTargetVoltage(0.0);
    }

    default void setTargetPosition(double postion) {
    }

    default void setTargetVoltage(double voltage) {
    }

    default void resetPosition() {
    }
}