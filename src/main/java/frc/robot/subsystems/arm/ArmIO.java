package frc.robot.subsystems.arm;

public interface ArmIO {
    default void updateInputs(ArmInputs inputs) {
    }

    default void stopShoulder() {
        setShoulderTargetVoltage(0.0);
    }

    default void setShoulderTargetPosition(double position) {
    }

    default void setShoulderTargetVoltage(double voltage) {
    }

    default void resetShoulderPosition() {
    }

    default void stopWrist() {
        setWristTargetVoltage(0.0);
    }

    default void setWristTargetPosition(double position) {
    }

    default void setWristTargetVoltage(double voltage) {
    }

    default void resetWristPosition() {
    }
}