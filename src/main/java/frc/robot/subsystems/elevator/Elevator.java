package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

@Logged
public class Elevator extends SubsystemBase {
    private static final double WAIT_TIME = 0.25; // In seconds
    private static final double POSITION_THRESHOLD = Units.inchesToMeters(1.0);
    private static final double VELOCITY_THRESHOLD = Units.rotationsPerMinuteToRadiansPerSecond(1.0);

    private final ElevatorIO io;
    private final ElevatorInputs inputs = new ElevatorInputs();
    private final Constants constants;

    private boolean zeroed = false;
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
        return runEnd(() -> {
            targetPosition = position.getAsDouble();
            io.setTargetPosition(targetPosition);
        }, io::stop);
    }

    public Command lock() {
        return runOnce(
                () -> io.setLocked(true));
    }


    public Command unlock() {
        return runOnce(
                () -> io.setLocked(false));
    }


    public Command zero() {
        return waitUntil(() -> Math.abs(inputs.currentVelocityMotor1) < VELOCITY_THRESHOLD)
                // Then reset the climber position
                .andThen(() -> {
                    io.resetPosition();
                    zeroed = true;
                })
                // Before we check if we're at the bottom hard stop, wait a little
                .beforeStarting(waitSeconds(WAIT_TIME))
                // Retract while we haven't found the bottom hard stop
                .withDeadline(runEnd(
                        () -> io.setTargetVoltage(-1.0),
                        io::stop))
                // Before we move, unlock the climber
                .beforeStarting(unlock())
                // After we finish, lock the climber
                .andThen(lock());
    }

    public record Constants(
            double maximumPosition
    ) {}
}