package frc.robot.subsystems.wrist;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

@Logged
public class Wrist extends SubsystemBase {
    public static final double POSITION_THRESHOLD = Units.degreesToRadians(5);
    private final WristIO io;
    private WristInputs inputs = new WristInputs();
    private double targetPosition = 0.0;

    public Wrist(WristIO io) {
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
        return runOnce(() -> {
                    targetPosition = 0.0;
                    io.stop();
                }
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
        return runEnd(() -> {
            targetPosition = position.getAsDouble();
            io.setTargetPosition(targetPosition);
        }, io::stop);
    }
}