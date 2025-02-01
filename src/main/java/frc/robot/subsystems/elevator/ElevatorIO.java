package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    default Elevator.Constants getConstants() {
        return new Elevator.Constants(1.0);
    }

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