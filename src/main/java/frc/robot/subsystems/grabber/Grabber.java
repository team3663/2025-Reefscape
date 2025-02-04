package frc.robot.subsystems.grabber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

@Logged
public class Grabber extends SubsystemBase {

    private final GrabberIO io;
    private final GrabberInputs inputs = new GrabberInputs();

    private double targetVoltage = 0.0;

    public Grabber(GrabberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public double getCurrentVelocity() {
        return inputs.currentVelocity;
    }

    public double getCurrentVoltage() {
        return inputs.currentAppliedVoltage;
    }

    public boolean getGamePieceNotDetected() {
        return !inputs.gamePieceDetected;
    }

    public Command stop() {
        return runOnce(stopRun());
    }

    private Runnable stopRun() {
        return () -> {
            targetVoltage = 0.0;
            io.stop();
        };
    }

    public Command withVoltage(double voltage) {
        return runEnd(() -> {
            targetVoltage = voltage;
            io.setTargetVoltage(targetVoltage);
        }, stopRun());
    }

    public Command followVoltage(DoubleSupplier velocity) {
        return runEnd(() -> {
            targetVoltage = velocity.getAsDouble();
            io.setTargetVoltage(targetVoltage);
        }, stopRun());
    }

    public Command withVoltageUntilDetected(double voltage) {
        return runEnd(() -> {
                    targetVoltage = voltage;
                    io.setTargetVoltage(targetVoltage);
                }, stopRun()
        ).until(() -> inputs.gamePieceDetected);
    }
}