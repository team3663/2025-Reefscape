package frc.robot.subsystems.arm;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

@Logged
public class Arm extends SubsystemBase {
    public static final double POSITION_THRESHOLD = Units.degreesToRadians(5);

    private final ArmIO io;
    private final ArmInputs inputs = new ArmInputs();
    private final Constants constants;

    private double targetShoulderPosition = 0.0;
    private double targetWristPosition = 0.0;

    public Arm(ArmIO io) {
        this.io = io;
        constants = io.getConstants();
    }

    public Constants getConstants() {
        return constants;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public double getTargetShoulderPosition() {
        return targetShoulderPosition;
    }

    public double getTargetWristPosition() {
        return targetWristPosition;
    }

    public Command stop() {
        return runOnce(() -> {
                    targetShoulderPosition = 0.0;
                    targetWristPosition = 0.0;
                    io.stopShoulder();
                    io.stopWrist();
                }
        );
    }

    public boolean atTargetPositions() {
        return this.atShoulderTargetPosition() && this.atWristTargetPosition();
    }

    public Command goToPositions(double positionShoulder, double positionWrist) {
        return run(() -> {
            // Shoulder
            targetShoulderPosition = positionShoulder;
            io.setShoulderTargetPosition(positionShoulder);

            // Wrist
            targetWristPosition = positionWrist;
            io.setWristTargetPosition(positionWrist);
        }).until(this::atTargetPositions);
    }

    public Command followPositions(DoubleSupplier positionShoulder, DoubleSupplier positionWrist) {
        return run(() -> {
            // Shoulder
            targetShoulderPosition = positionShoulder.getAsDouble();
            io.setShoulderTargetPosition(targetShoulderPosition);

            // Wrist
            targetWristPosition = positionWrist.getAsDouble();
            io.setWristTargetPosition(targetWristPosition);
        });
    }

    public double getShoulderPosition() {
        return inputs.currentPositionShoulder;
    }

    public boolean atShoulderTargetPosition() {
        return Math.abs(inputs.currentPositionShoulder - targetShoulderPosition) < POSITION_THRESHOLD;
    }

    public double getWristPosition() {
        return inputs.currentPositionWrist;
    }

    public boolean atWristTargetPosition() {
        return Math.abs(inputs.currentPositionWrist - targetWristPosition) < POSITION_THRESHOLD;
    }

    public record Constants(double shoulderLength,
                            double minimumShoulderAngle, double maximumShoulderAngle,
                            double wristLength,
                            double minimumWristAngle, double maximumWristAngle) {
    }
}