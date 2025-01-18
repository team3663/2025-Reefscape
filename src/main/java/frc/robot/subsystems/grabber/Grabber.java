package frc.robot.subsystems.grabber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

@Logged
public class Grabber extends SubsystemBase {
    private final double VOLTAGE_THRESHOLD = 1.0;

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

    public double getCurrentPosition() {
        return inputs.currentPosition;
    }

    public boolean atTargetVoltage() {
        return Math.abs(inputs.currentAppliedVoltage - targetVoltage) < VOLTAGE_THRESHOLD;
    }

    public Command stop() {
        return runOnce(() -> {
                    targetVoltage = 0.0;
                    io.stop();
                }
        );
    }

    public boolean getBeamBreakState() {
        return io.getBeamBreakState();
    }

    public Command withVoltage(double voltage) {
        return runEnd(() -> {
            targetVoltage = voltage;
            io.setTargetVoltage(targetVoltage);
        }, io::stop);
    }

    public Command followVoltage(DoubleSupplier velocity) {
        return runEnd(() -> {
            targetVoltage = velocity.getAsDouble();
            io.setTargetVoltage(targetVoltage);
        }, io::stop);
    }

    public Command withVoltageUntilBeam(double voltage) {
        return runEnd(() -> {
                    targetVoltage = voltage;
                    io.setTargetVoltage(targetVoltage);
                }, io::stop
        ).until(() -> inputs.beamBreakState);
    }
}