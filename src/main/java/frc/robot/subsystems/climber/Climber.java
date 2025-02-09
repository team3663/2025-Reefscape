package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

@Logged
public class Climber extends SubsystemBase {
    private static final double VELOCITY_THRESHOLD = Units.rotationsPerMinuteToRadiansPerSecond(1);
    private static final double POSITION_THRESHOLD = Units.degreesToRadians(1.0);
    private static final double WAIT_TIME = 0.25;
    private static final double DEPLOY_ANGLE = Units.degreesToRadians(90);
    private static final double CLIMB_ANGLE = Units.degreesToRadians(270);

    private final ClimberIO io;
    private final Constants constants;
    private final ClimberInputs inputs = new ClimberInputs();

    private boolean zeroed = false;
    private double targetPosition = 0.0;
    private double targetVoltage = 0.0;


    public Climber(ClimberIO io) {
        this.io = io;
        this.constants = io.getConstants();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public boolean atTargetPosition() {
        return Math.abs(inputs.currentPosition - targetPosition) < POSITION_THRESHOLD;
    }

    public Command stop() {
        return runOnce(() -> {
                    targetPosition = 0.0;
                    targetVoltage = 0.0;
                    io.stop();
                }
        );
    }

    public Command resetPosition() {
        return runOnce(
                io::resetPosition
        );
    }


    public Command goToPosition(double position) {
        return runEnd(
                () -> {
                    io.setTargetPosition(position);
                    targetPosition = position;
                }, io::stop
        ).until(this::atTargetPosition);
    }

    public Command followPosition(DoubleSupplier position) {
        return runEnd(
                () -> {
                    targetPosition = position.getAsDouble();
                    io.setTargetPosition(targetPosition);
                },
                io::stop
        );
    }

    public Command zero() {
        // Run the Elevator backwards until stopped and then stop
        return runEnd(() -> io.setTargetVoltage(-1.0), io::stop)
                // While doing that wait until the elevator stops (Hit the hard stop)
                // Also stop the previous command when this one stops (It hit the hard stop and reset position)
                .withDeadline(waitUntil(() -> Math.abs(inputs.currentVelocity) < VELOCITY_THRESHOLD)
                        // Then reset the elevator position and set zeroed to true
                        .andThen(() -> {
                            io.resetPosition();
                            zeroed = true;
                        })
                        // Before we check if we're at the bottom hard stop, wait a little so that it doesn't think we hit it because we haven't started going yet
                        .beforeStarting(waitSeconds(WAIT_TIME)));
    }

    public Command deploy() {
        return goToPosition(DEPLOY_ANGLE);
    }

    public Command climb() {
        return goToPosition(CLIMB_ANGLE);
    }

    public record Constants(
            double maximumPosition
    ) {}
}
