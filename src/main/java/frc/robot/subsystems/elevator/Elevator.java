package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

@Logged
public class Elevator extends SubsystemBase {

    private static final double POSITION_THRESHOLD = Units.inchesToMeters(1);

    private final ElevatorIO io;
    private final ElevatorInputs inputs = new ElevatorInputs();
    private final Constants constants;

    private double targetPosition = 0.0;

    public Elevator(ElevatorIO io) {
        this.io = io;
        this.constants = io.getConstants();
    }

    public Constants getConstants() {
        return constants;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public double getVelocity() {
        return inputs.currentVelocityMotor1;
    }

    public double getPosition() {
        return inputs.currentPositionMotor1;
    }

    public boolean atTargetPosition() {
        return Math.abs(inputs.currentPositionMotor1 - targetPosition) < POSITION_THRESHOLD;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public Command stop() {
        return runOnce(() -> {
            targetPosition = 0.0;
            io.stop();
        });
    }

    public Command resetPosition() {
        return runOnce(io::resetPosition);
    }

    public Command goToPosition(double position) {
        return run(() -> {
            io.setTargetPosition(position);
            targetPosition = position;
        }).until(this::atTargetPosition);
    }

    public Command followPosition(DoubleSupplier position) {
        return run(() -> {
            targetPosition = position.getAsDouble();
            io.setTargetPosition(targetPosition);
        });
    }

    public record Constants(
            double minimumPosition,
            double maximumPosition
    ) {}
}