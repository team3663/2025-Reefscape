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
    public static final double CORAL_DEBOUNCE_TIME = 0.05;
    public static final double CORAL_GRAB_VOLTAGE = 6.0;
    public static final double CORAL_PLACE_VOLTAGE = 6.0;
    public static final double CORAL_PLACE_SLOW_VOLTAGE = 3.0;
    public static final double CORAL_EJECT_VOLTAGE = -6.0;

    public static final double ALGAE_DEBOUNCE_TIME = 0.1;
    public static final double ALGAE_GRAB_VOLTAGE = -9.0;
    public static final double ALGAE_HOLD_VOLTAGE = -3.0;
    public static final double ALGAE_PLACE_VOLTAGE = 6.0;
    public static final double ALGAE_PLACE_DELAY = 0.25;
    public static final double ALGAE_REMOVE_VOLTAGE = 3.0;
    public static final double ALGAE_EJECT_VOLTAGE = 4.0;

    private final GrabberIO io;
    private final GrabberInputs inputs = new GrabberInputs();

    private Debouncer algaeDebouncer = new Debouncer(0.25);

    private Gamepiece gamepiece = Gamepiece.CORAL;
    private double targetVoltage = 0.0;
    private boolean holdingAlgae = false;

    public Grabber(GrabberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        holdingAlgae = algaeDebouncer.calculate(isGamePieceDetected() && gamepiece == Gamepiece.ALGAE);
        if (holdingAlgae && targetVoltage == 0.0)
        {
            io.setTargetVoltage(ALGAE_HOLD_VOLTAGE);
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
        return holdingAlgae;
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
                        withVoltage(ALGAE_EJECT_VOLTAGE), // algae
                        withVoltage(CORAL_EJECT_VOLTAGE), // coral
                        this::hasAlgae
                ).withDeadline(
                        Commands.waitUntil(this::isGamePieceNotDetected))
                .andThen(Commands.waitSeconds(0.25)
                );
    }

    public Command grabAlgae() {
        return withVoltage(ALGAE_GRAB_VOLTAGE)
                .until(this::hasAlgae)
                .beforeStarting(() -> gamepiece = Gamepiece.ALGAE)
                .unless(this::isGamePieceDetected);
    }

    public Command placeAlgae() {
        return withVoltage(ALGAE_PLACE_VOLTAGE).withDeadline(
                Commands.waitUntil(this::isGamePieceNotDetected)
                        .andThen(Commands.waitSeconds(ALGAE_PLACE_DELAY))
        );
    }

    public Command removeAlgae() {
        return withVoltage(ALGAE_REMOVE_VOLTAGE);
    }

    public Command grabCoral() {
        Debouncer[] debouncerHolder = new Debouncer[1];

        return withVoltage(CORAL_GRAB_VOLTAGE)
                .withDeadline(
                        Commands.sequence(
                                Commands.runOnce(() -> debouncerHolder[0] = new Debouncer(CORAL_DEBOUNCE_TIME)),
                                Commands.waitUntil(() -> debouncerHolder[0].calculate(isGamePieceDetected()))
                        ))
                .unless(this::isGamePieceDetected)
                .beforeStarting(() -> gamepiece = Gamepiece.CORAL);
    }

    public Command placeCoral() {
        return withVoltage(CORAL_PLACE_VOLTAGE);
    }

    public Command getRobotModeCommand(Supplier<RobotMode> robotMode, BooleanSupplier readyToPlace) {
        Debouncer[] debouncerHolder = new Debouncer[1];

        return runEnd(() -> {
            if (hasAlgae() && !readyToPlace.getAsBoolean()) {
                targetVoltage = ALGAE_HOLD_VOLTAGE;
            } else if (!robotMode.get().isPlacingMode())
                if (robotMode.get().getGamepiece() == Gamepiece.ALGAE) {
                    // Grab Algae
                    targetVoltage = ALGAE_GRAB_VOLTAGE;
                } else {
                    // Grab Coral
                    targetVoltage = CORAL_GRAB_VOLTAGE;
                    if (gamepiece != Gamepiece.CORAL)
                        debouncerHolder[0] = new Debouncer(CORAL_DEBOUNCE_TIME);
                }
            else if (readyToPlace.getAsBoolean()) {
                if (robotMode.get().getGamepiece() == Gamepiece.ALGAE) {
                    // Place Algae
                    targetVoltage = ALGAE_PLACE_VOLTAGE;
                } else {
                    // Place Coral
                    if (robotMode.get() == RobotMode.CORAL_LEVEL_1)
                        targetVoltage = CORAL_PLACE_SLOW_VOLTAGE;
                    else
                        targetVoltage = CORAL_PLACE_VOLTAGE;
                }
            }
            io.setTargetVoltage(targetVoltage);
            gamepiece = robotMode.get().getGamepiece();
        },this::stopInternal)
                .beforeStarting(runOnce(() -> {
                    gamepiece = robotMode.get().getGamepiece();
                    debouncerHolder[0] = new Debouncer(gamepiece == Gamepiece.CORAL ? CORAL_DEBOUNCE_TIME : ALGAE_DEBOUNCE_TIME);
                }))
                .until(() -> (debouncerHolder[0].calculate(isGamePieceDetected()) && robotMode.get() == RobotMode.CORAL_STATION) ||
                        (hasAlgae() && (robotMode.get() == RobotMode.ALGAE_REMOVE_UPPER || robotMode.get() == RobotMode.ALGAE_REMOVE_LOWER)))
                .withDeadline(
                        Commands.waitUntil(() -> this.isGamePieceNotDetected() && gamepiece == Gamepiece.ALGAE && robotMode.get().isPlacingMode()).andThen(Commands.waitSeconds(ALGAE_PLACE_DELAY)))
                .unless(() -> (inputs.gamePieceDetected && !robotMode.get().isPlacingMode()));
    }

//    public Command placeCoralSlow() {
//        return withVoltage(CORAL_PLACE_SLOW_VOLTAGE);
//    }
}