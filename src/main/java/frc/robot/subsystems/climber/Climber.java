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
    private static final double POSITION_THRESHOLD = Units.degreesToRadians(1.0);
    private static final double WAIT_TIME = 0.25;
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
        // Wait until the climber stops moving
        return waitUntil(() -> Math.abs(inputs.currentVelocity) < 0.01)
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
                        io::stop));
    }
    public record Constants(
            double maximumPosition
    ) {}
}
