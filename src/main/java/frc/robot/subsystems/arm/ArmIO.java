package frc.robot.subsystems.arm;

public interface ArmIO {
    default void updateInputs(ArmInputs inputs) {
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