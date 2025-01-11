package frc.robot.subsystems.prototype;


public interface PrototypeIO {
    default void updateInputs(PrototypeInputs inputs) {
    }

    default void stopMotor1() {
        setTargetVoltageMotor1(0.0);
    }

    default void resetRotationsMotor1() {
    }

    default void setTargetRotationsMotor1(double rotations) {
    }

    default void setTargetVelocityMotor1(double velocity) {
    }

    default void setTargetVoltageMotor1(double voltage) {
    }

    default void stopMotor2() {
        setTargetVoltageMotor2(0.0);
    }

    default void resetRotationsMotor2() {
    }

    default void stopMotors() {
        setTargetVoltageMotor1(0.0);
        setTargetVoltageMotor2(0.0);
    }

    ;

    default void setTargetRotationsMotor2(double rotations) {
    }

    default void setTargetVelocityMotor2(double velocity) {
    }

    default void setTargetVoltageMotor2(double voltage) {
    }
}
