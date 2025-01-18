package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    default void updateInputs(ElevatorInputs inputs) {
    }

    default void stop() {
        setTargetVoltage(0.0);
    }

    default void setTargetVoltage(double voltage) {
    }

    default void setTargetPosition(double position) {
    }

    default void resetPosition() {
    }
}
