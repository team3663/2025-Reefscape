package frc.robot.subsystems.prototype;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

@Logged
public class Prototype extends SubsystemBase {
    private static final double VELOCITY_THRESHOLD = Units.rotationsPerMinuteToRadiansPerSecond(100.0);
    private static final double ROTATIONS_THRESHOLD = 0.1;
    private static final double VOLTAGE_THRESHOLD = 12.0;

    final PrototypeIO io;
    final PrototypeInputs inputs = new PrototypeInputs();

    double targetVelocityMotor1 = 0.0;
    double targetRotationsMotor1 = 0.0;
    double targetVoltageMotor1 = 0.0;

    double targetVelocityMotor2 = 0.0;
    double targetRotationsMotor2 = 0.0;
    double targetVoltageMotor2 = 0.0;

    public Prototype(PrototypeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public double getVelocityMotor1() {
        return inputs.currentVelocityMotor1;
    }

    public double getRotationsMotor1() {
        return inputs.currentRotationsMotor1;
    }

    public boolean atTargetVelocityMotor1() {
        return Math.abs(inputs.currentVelocityMotor1 - targetVelocityMotor1) < VELOCITY_THRESHOLD;
    }

    public boolean atTargetRotationsMotor1() {
        return Math.abs(inputs.currentRotationsMotor1 - targetRotationsMotor1) < ROTATIONS_THRESHOLD;
    }

    public boolean atTargetVoltageMotor1() {
        return Math.abs(inputs.currentAppliedVoltageMotor1 - targetVoltageMotor1) < VOLTAGE_THRESHOLD;
    }

    public double getVoltageMotor1() {
        return inputs.currentAppliedVoltageMotor1;
    }

    public double getVoltageMotor2() {
        return inputs.currentAppliedVoltageMotor2;
    }

    public double getVelocityMotor2() {
        return inputs.currentVelocityMotor2;
    }

    public double getRotationsMotor2() {
        return inputs.currentRotationsMotor2;
    }

    public boolean atTargetVelocityMotor2() {
        return Math.abs(inputs.currentVelocityMotor2 - targetVelocityMotor2) < VELOCITY_THRESHOLD;
    }

    public boolean atTargetRotationsMotor2() {
        return Math.abs(inputs.currentRotationsMotor2 - targetRotationsMotor1) < ROTATIONS_THRESHOLD;
    }

    public boolean atTargetVoltageMotor2() {
        return Math.abs(inputs.currentAppliedVoltageMotor2 - targetVoltageMotor2) < VOLTAGE_THRESHOLD;
    }

    /**
     * Reset the amount of rotations on the 1st motor to 0
     */
    public Command resetRotationsMotor1() {
        return runOnce(
                io::resetRotationsMotor1
        );
    }

    /**
     * Reset the amount of rotations on the 2nd motor to 0
     */
    public Command resetRotationsMotor2() {
        return runOnce(
                io::resetRotationsMotor2
        );
    }

    /**
     * Stops the 1st motor by running at 0 voltage
     */
    public Command stopMotor1() {
        return runOnce(
                io::stopMotor1
        );
    }

    /**
     * Stops both motors by running at 0 voltage
     */
    public Command stopMotors() {
        return runOnce(
                io::stopMotors
        );
    }

    /**
     * Stops the 2nd motor by running at 0 voltage
     */
    public Command stopMotor2() {
        return runOnce(
                io::stopMotor2
        );
    }

    /**
     * Tells the 1st motor to move a certain amount of rotations and does not end
     */
    public Command followRotationsMotor1(DoubleSupplier rotations) {
        return runEnd(
                () -> {
                    targetRotationsMotor1 = rotations.getAsDouble();
                    io.setTargetRotationsMotor1(targetRotationsMotor1);
                },
                io::stopMotor1
        );
    }

    /**
     * Tells the 2nd motor to move a certain amount of rotations and does not end
     */
    public Command followRotationsMotor2(DoubleSupplier rotations) {
        return runEnd(
                () -> {
                    targetRotationsMotor1 = rotations.getAsDouble();
                    io.setTargetRotationsMotor2(targetRotationsMotor1);
                },
                io::stopMotor2
        );
    }

    /**
     * Tells the 1st motor to move a certain amount of rotations and then ends
     */
    public Command toRotationsMotor1(double radians) {
        return followRotationsMotor1(() -> radians).until(this::atTargetRotationsMotor1);
    }

    /**
     * Tells the 2nd motor to move a certain amount of rotations and then ends
     */
    public Command toRotationsMotor2(double radians) {
        return followRotationsMotor2(() -> radians).until(this::atTargetRotationsMotor2);
    }

    /**
     * Tells the 1st motor to move with velocity and does not end
     */
    public Command followVelocityMotor1(DoubleSupplier velocity) {
        return runEnd(
                () -> {
                    targetVelocityMotor1 = velocity.getAsDouble();
                    io.setTargetVelocityMotor1(targetVelocityMotor1);
                },
                io::stopMotor1
        );
    }

    /**
     * Tells the 2nd motor to move with velocity and does not end
     */
    public Command followVelocityMotor2(DoubleSupplier velocity) {
        return runEnd(
                () -> {
                    targetVelocityMotor2 = velocity.getAsDouble();
                    io.setTargetVelocityMotor2(targetVelocityMotor2);
                },
                io::stopMotor2
        );
    }

    /**
     * Tells the 1st motor to move with a certain velocity and then ends
     */
    public Command withVelocityMotor1(double velocity) {
        return runEnd(
                () -> {
                    targetVelocityMotor1 = velocity;
                    io.setTargetVelocityMotor1(targetVelocityMotor1);
                },
                io::stopMotor1
        );
    }

    /**
     * Tells the 1st motor to move with a certain velocity and then ends
     */
    public Command withVelocityMotor2(double velocity) {
        return runEnd(
                () -> {
                    targetVelocityMotor2 = velocity;
                    io.setTargetVelocityMotor2(targetVelocityMotor2);
                },
                io::stopMotor2
        );
    }


    public Command followVoltage(DoubleSupplier voltageMotor1, DoubleSupplier voltageMotor2) {
        return runEnd(
                () -> {
                    targetVoltageMotor1 = voltageMotor1.getAsDouble();
                    io.setTargetVoltageMotor1(targetVoltageMotor1);
                    targetVoltageMotor2 = voltageMotor2.getAsDouble();
                    io.setTargetVoltageMotor2(targetVoltageMotor2);
                },
                io::stopMotors
        );
    }


}

