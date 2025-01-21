package frc.robot.subsystems.arm;

public interface ArmIO {
    default void updateInputs(ArmInputs inputs) {
    }

    default void stopShoulder() {
        setTargetVoltageShoulder(0.0);
    }

    default void setTargetPositionShoulder(double position) {
    }

    default void setTargetVoltageShoulder(double voltage) {
    }

    default void resetPositionShoulder() {
    }

    default void stopWrist() {
        setTargetVoltageWrist(0.0);
    }

    default void setTargetPositionWrist(double position) {
    }

    default void setTargetVoltageWrist(double voltage) {
    }

    default void resetPositionWrist() {
    }
}