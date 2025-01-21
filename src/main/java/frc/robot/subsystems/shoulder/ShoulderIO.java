package frc.robot.subsystems.shoulder;

public interface ShoulderIO {
    default void updateInputs(ShoulderInputs inputs) {
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