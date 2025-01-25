package frc.robot.subsystems.climber;

public interface ClimberIO {
    default void updateInputs(ClimberInputs inputs) {
    }

    default void setTargetPosition(double position) {
    }

    default void setTargetVoltage(double voltage) {
    }

    default void resetPosition() {
    }

    default void stop() {
        setTargetVoltage(0.0);
    }
}
