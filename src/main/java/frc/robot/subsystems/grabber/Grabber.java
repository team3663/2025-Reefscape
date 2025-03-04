package frc.robot.subsystems.grabber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMode;
import frc.robot.utility.Gamepiece;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@Logged
public class Grabber extends SubsystemBase {

    private final GrabberIO io;
    private final GrabberInputs inputs = new GrabberInputs();

    private Gamepiece gamepiece = Gamepiece.CORAL;
    private double targetVoltage = 0.0;

    public Grabber(GrabberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (hasAlgae() && targetVoltage == 0.0)
        {
            io.setTargetVoltage(-1.0);
        }
    }

    public double getCurrentVelocity() {
        return inputs.currentVelocity;
    }

    public double getCurrentVoltage() {
        return inputs.currentAppliedVoltage;
    }

    public boolean isGamePieceDetected() {
        return inputs.gamePieceDetected;
    }

    public boolean isGamePieceNotDetected() {
        return !inputs.gamePieceDetected;
    }

    public boolean hasAlgae() {
        return isGamePieceDetected() && gamepiece == Gamepiece.ALGAE;
    }

    public boolean hasCoral() {
        return isGamePieceDetected() && gamepiece == Gamepiece.CORAL;
    }

    public Command stop() {
        return runOnce(this::stopInternal);
    }

    // Supports command implementations by making it easier to both reset the target position and stop the motor
    private void stopInternal() {
        targetVoltage = 0.0;
        io.stop();
    }

    private Command withVoltage(double voltage) {
        return runEnd(() -> {
            targetVoltage = voltage;
            io.setTargetVoltage(targetVoltage);
        }, this::stopInternal);
    }

    public Command eject() {
        return Commands.either(
                        withVoltage(4.0), // algae
                        withVoltage(-6.0), // coral
                        this::hasAlgae
                ).withDeadline(
                        Commands.waitUntil(this::isGamePieceNotDetected))
                .andThen(Commands.waitSeconds(0.25)
                );
    }

    public Command grabAlgae() {
        Debouncer[] debouncerHolder = new Debouncer[1];

        return withVoltage(-4.0)
                .withDeadline(
                        Commands.sequence(
                                Commands.runOnce(() -> debouncerHolder[0] = new Debouncer(0.1)),
                                Commands.waitUntil(() -> debouncerHolder[0].calculate(isGamePieceDetected()))
                        ))
                .beforeStarting(() -> gamepiece = Gamepiece.ALGAE)
                .unless(this::isGamePieceDetected);
    }

    public Command placeAlgae() {
        return withVoltage(6.0).withDeadline(
                Commands.waitUntil(this::isGamePieceNotDetected)
                        .andThen(Commands.waitSeconds(0.25))
        );
    }

    public Command removeAlgae() {
        return withVoltage(3.0);
    }

    public Command grabCoral() {
        Debouncer[] debouncerHolder = new Debouncer[1];

        return withVoltage(6.0)
                .withDeadline(
                        Commands.sequence(
                                Commands.runOnce(() -> debouncerHolder[0] = new Debouncer(0.04)),
                                Commands.waitUntil(() -> debouncerHolder[0].calculate(isGamePieceDetected()))
                        ))
                .unless(this::isGamePieceDetected)
                .beforeStarting(() -> gamepiece = Gamepiece.CORAL);
    }

    public Command placeCoral() {
        return withVoltage(6.0);
    }

    public Command getRobotModeCommand(Supplier<RobotMode> robotMode, BooleanSupplier readyToPlace) {
        Debouncer[] debouncerHolder = new Debouncer[1];

        return run(() -> {
            if (!robotMode.get().isPlacingMode())
                if (robotMode.get().getGamepiece() == Gamepiece.ALGAE) {
                    // Grab Algae
                    targetVoltage = -4.0;
                    if (gamepiece != Gamepiece.ALGAE)
                        debouncerHolder[0] = new Debouncer(0.1);
                } else {
                    // Grab Coral
                    targetVoltage = 6.0;
                    if (gamepiece != Gamepiece.ALGAE)
                        debouncerHolder[0] = new Debouncer(0.04);
                }
            else if (readyToPlace.getAsBoolean())
                if (robotMode.get().getGamepiece() == Gamepiece.ALGAE) {
                    // Place Algae
                    targetVoltage = 6.0;
                } else {
                    // Place Coral
                    targetVoltage = 6.0;
                }
            io.setTargetVoltage(targetVoltage);
            gamepiece = robotMode.get().getGamepiece();

        }).until(() -> (debouncerHolder[0].calculate(isGamePieceDetected()) && !robotMode.get().isPlacingMode()))
                .withDeadline(Commands.waitUntil(() -> this.isGamePieceDetected() && gamepiece == Gamepiece.ALGAE).andThen(Commands.waitSeconds(0.25)))
                .unless(() -> inputs.gamePieceDetected && !robotMode.get().isPlacingMode());
    }
}