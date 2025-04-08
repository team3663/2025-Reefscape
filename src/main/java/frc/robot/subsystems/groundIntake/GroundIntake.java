package frc.robot.subsystems.groundIntake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

@Logged
public class GroundIntake extends SubsystemBase {
    public static final double POSITION_THRESHOLD = Units.degreesToRadians(2.0);

    private final GroundIntakeIO io;
    private final Constants constants;
    private final GroundIntakeInputs inputs = new GroundIntakeInputs();

    private boolean pivotZeroed = false;
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
        return runOnce(this::stopIntakeInternal);
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
            if (pivotZeroed) {
                targetPivotPosition = getValidPositionPivot(pivotPosition.getAsDouble());
                io.setTargetPositionPivot(targetPivotPosition);
            }
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

    public Command resetPivotPositionToDefault() {
        return Commands.runOnce(() -> {
            io.resetPivotPosition(constants.minimumPivotAngle + Units.degreesToRadians(1.0));
            pivotZeroed = true;
        });
    }

    private Command withVoltage(double voltage) {
        return runEnd(() -> {
            targetVoltage = voltage;
            io.setTargetVoltageIntake(targetVoltage);
        }, this::stopIntakeInternal);
    }

    public Command eject() {
        return withVoltage(-2.0);
    }

    public Command grabCoral() {
        Debouncer[] debouncerHolder = new Debouncer[1];

        return runIntakeAndPivot(6.0, frc.robot.Constants.GroundIntakePositions.INTAKING_ANGLE);
//                .withDeadline(
//                        Commands.sequence(
//                                Commands.runOnce(() -> debouncerHolder[0] = new Debouncer(0.06)),
//                                Commands.waitUntil(() -> debouncerHolder[0].calculate(isGamePieceDetected()))
//                        ))
//                .unless(this::isGamePieceDetected);
    }

    public Command runIntakeAndPivot(double voltage, double position) {
        return runEnd(() -> {
            targetVoltage = voltage;
            io.setTargetVoltageIntake(targetVoltage);
            if (pivotZeroed) {
                targetPivotPosition = getValidPositionPivot(position);
                io.setTargetPositionPivot(targetPivotPosition);
            }
        }, this::stopIntakeInternal);
    }

    public Command zeroPivot() {
        return runEnd(() -> {
            io.setTargetVoltagePivot(-1.5);
            targetPivotPosition = constants.minimumPivotAngle;
        }, io::stopPivot)
                .withDeadline(waitUntil(() -> Math.abs(inputs.pivotCurrentVelocity) < 0.01)
                        .beforeStarting(waitSeconds(0.25))
                        .andThen(() -> {
                            io.resetPivotPosition(constants.minimumPivotAngle());
                            pivotZeroed = true;
                        }));
    }

    public record Constants(double minimumPivotAngle, double maximumPivotAngle) {
    }
}
