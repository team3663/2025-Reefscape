package frc.robot.subsystems.groundIntake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

@Logged
public class GroundIntake extends SubsystemBase {
    public static final double POSITION_THRESHOLD = Units.degreesToRadians(2.0);
    private final GroundIntakeIO io;
    private final Constants constants;
    private final GroundIntakeInputs inputs = new GroundIntakeInputs();

    private double targetVoltage = 0.0;
    private double targetPivotPosition = 0.0;

    public GroundIntake(GroundIntakeIO io) {
        this.io = io;
        this.constants = io.getConstants();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public double getTargetPivotPosition() {
        return targetPivotPosition;
    }

    public Command stop() {
        return runOnce(() -> {
            targetPivotPosition = 0.0;
            io.stopPivot();
            io.stopIntake();
        });
    }

    public boolean isGamePieceDetected() {
        return inputs.gamePieceDetected;
    }

    public boolean getGamePieceNotDetected() {
        return !inputs.gamePieceDetected;
    }

    // Pivot Motor
    public boolean atTargetPosition() {
        return this.atPivotTargetPosition();
    }

    public boolean atPosition(double position, double threshold) {
        return Math.abs(inputs.pivotCurrentPosition - position) < threshold;
    }

    public boolean atPivotTargetPosition() {
        return atPosition(targetPivotPosition, POSITION_THRESHOLD);
    }

    public Command followPositions(DoubleSupplier pivotPosition) {
        return run(() -> {
            targetPivotPosition = getValidPositionPivot(pivotPosition.getAsDouble());
            io.setTargetPositionPivot(targetPivotPosition);
        });
    }

    public double getValidPositionPivot(double position) {
        return Math.max(constants.minimumPivotAngle, Math.min(constants.maximumPivotAngle, position));
    }

    public double getPivotPosition() {
        return inputs.pivotCurrentPosition;
    }

    // Intake Motor
    private void stopIntakeInternal() {
        targetVoltage = 0.0;
        io.stopIntake();
    }

    private Command withVoltage(double voltage) {
        return runEnd(() -> {
            targetVoltage = voltage;
            io.setTargetVoltageIntake(voltage);
        }, this::stopIntakeInternal);
    }

    public Command eject() {
        return withVoltage(-6.0);
    }

    public Command grabCoral() {
        Debouncer[] debouncerHolder = new Debouncer[1];

        return withVoltage(6.0)
                .withDeadline(
                        Commands.sequence(
                                Commands.runOnce(() -> debouncerHolder[0] = new Debouncer(0.06)),
                                Commands.waitUntil(() -> debouncerHolder[0].calculate(isGamePieceDetected()))
                        ))
                .unless(this::isGamePieceDetected);
    }

    public record Constants(double minimumPivotAngle, double maximumPivotAngle) {
    }
}
