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
    public static final double POSITION_THRESHOLD = Units.inchesToMeters(1.0);
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
        return atPosition(targetPosition, POSITION_THRESHOLD);
    }

    public boolean atPosition(double position, double threshold) {
        return Math.abs(inputs.currentPositionMotor1 - position) < threshold;
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

    public Command zero() {
        // Run the Elevator backwards until stopped and then stop
        return runEnd(() -> io.setTargetVoltage(-1.0), io::stop)
                // While doing that wait until the elevator stops (Hit the hard stop)
                // Also stop the previous command when this one stops (It hit the hard stop and reset position)
                .withDeadline(waitUntil(() -> Math.abs(inputs.currentVelocityMotor1) < VELOCITY_THRESHOLD)
                        // Then reset the elevator position and set zeroed to true
                        .andThen(() -> {
                            io.resetPosition();
                            zeroed = true;
                        })
                        // Before we check if we're at the bottom hard stop, wait a little so that it doesn't think we hit it because we haven't started going yet
                        .beforeStarting(waitSeconds(WAIT_TIME)));
    }

    public record Constants(
            double maximumPosition
    ) {}
}