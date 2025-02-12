package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;

public interface ArmIO {
    default Arm.Constants getConstants() {
        return new Arm.Constants(
                0.2, Units.degreesToRadians(-135.0), Units.degreesToRadians(180.0),
                0.05, Units.degreesToRadians(-90.0), Units.degreesToRadians(90.0));
    }

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

    default void sysIdShoulder(Voltage voltage){

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

    default void sysIdWrist(Voltage voltage){

    }
}