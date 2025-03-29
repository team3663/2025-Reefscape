package frc.robot.subsystems.grabber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Gamepiece;

@Logged
public class Grabber extends SubsystemBase {

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
            io.setTargetVoltage(-1.5);
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

    public boolean getGamePieceNotDetected() {
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
                        withVoltage(4.0), // algae
                        withVoltage(-6.0), // coral
                        this::hasAlgae
                );
    }

    public Command grabAlgae() {
        return withVoltage(-12.0)
                .until(this::hasAlgae)
                .beforeStarting(() -> gamepiece = Gamepiece.ALGAE)
                .unless(this::isGamePieceDetected);
    }

    public Command placeAlgae()
    {
        return withVoltage(10.0).withDeadline(
                Commands.waitUntil(this::getGamePieceNotDetected)
                        .andThen(Commands.waitSeconds(0.25))
        );
    }

    public Command removeAlgae()
    {
        return withVoltage(3.0);
    }

    public Command grabCoral() {
        Debouncer[] debouncerHolder = new Debouncer[1];

        return withVoltage(6.0)
                .withDeadline(
                        Commands.sequence(
                                Commands.runOnce(() -> debouncerHolder[0] = new Debouncer(0.06)),
                                Commands.waitUntil(() -> debouncerHolder[0].calculate(isGamePieceDetected()))
                        ))
                .unless(this::isGamePieceDetected)
                .beforeStarting(() -> gamepiece = Gamepiece.CORAL);
    }

    public Command placeCoral() {
        return withVoltage(12.0);
    }

    public Command placeCoralL4() {
        return withVoltage(10.0);
    }

    public Command placeCoralSlow() {
        return withVoltage(6.0);
    }
}