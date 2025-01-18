package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
    public static final double VELOCITY_THRESHOLD = Units.rotationsPerMinuteToRadiansPerSecond(100.0);
    public static final double POSITION_THRESHOLD = Units.degreesToRadians(5);
    private final ArmIO io;
    private ArmInputs inputs;
    private double targetPosition = 0.0;

    public Arm(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public double getVelocity() {
        return inputs.currentVelocity;
    }

    public double getPosition() {
        return inputs.currentPosition;
    }

    public boolean atTargetPosition() {
        return Math.abs(inputs.currentPosition - targetPosition) < POSITION_THRESHOLD;
    }

    public Command stop() {
        return runOnce(
                io::stop
        );
    }

    public Command resetPosition() {
        return runOnce(
                () -> io.resetPosition()
        );
    }

    public Command goToPosition(double position) {
        return run(
                () -> {
                    io.setTargetPosition(position);
                    targetPosition = position;
                }
        ).until(this::atTargetPosition);
    }

    public Command followPosition(DoubleSupplier position) {
        return runEnd(
                () -> {
                    io.setTargetPosition(position.getAsDouble());
                    targetPosition = position.getAsDouble();
                },
                io::stop
        );
    }
}
