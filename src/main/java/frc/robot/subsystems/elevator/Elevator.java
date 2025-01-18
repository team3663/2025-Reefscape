package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

@Logged
public class Elevator extends SubsystemBase {

    private static final double POSITION_THRESHOLD = Units.degreesToRadians(5);

    private final ElevatorIO io;
    private final ElevatorInputs inputs = new ElevatorInputs();

    private double targetPosition = 0.0;

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public double getVelocityMotor1() {
        return inputs.currentVelocityMotor1;
    }

    public double getVelocityMotor2() {
        return inputs.currentVelocityMotor2;
    }

    public double getPosition() {
        return inputs.currentPositionMotor1;
    }

    public boolean atTargetPosition() {
        return Math.abs(inputs.currentPositionMotor1 - targetPosition) < POSITION_THRESHOLD;
    }

    public Command stop() {
        return runOnce(io::stop);
    }

    public Command resetPosition() {
        return runOnce(io::resetPosition);
    }

    public Command goToPosition(double position) {
        return run(() -> {
            io.setTargetPosition(position);
            targetPosition = position;
        }).until(this::atTargetPosition);
    }

    public Command followPosition(DoubleSupplier position) {
        return runEnd(() -> {
            io.setTargetPosition(position.getAsDouble());
            targetPosition = position.getAsDouble();
        }, io::stop);
    }
}