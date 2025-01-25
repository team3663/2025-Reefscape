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
    private ArmInputs inputs = new ArmInputs();

    private double targetShoulderPosition = 0.0;
    private double targetWristPosition = 0.0;

    public Arm(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
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

    public Command resetPositions() {
        return runOnce(() -> {
            io.resetShoulderPosition();
            io.resetWristPosition();
        });
    }

    public boolean atTargetPositions() {
        return this.atShoulderTargetPosition() && this.atWristTargetPosition();
    }

    public Command goToPositions(double positionShoulder, double positionWrist) {
        return runEnd(() -> {
                    // Shoulder
                    targetShoulderPosition = positionShoulder;
                    io.setShoulderTargetPosition(positionShoulder);

                    // Wrist
                    targetWristPosition = positionWrist;
                    io.setWristTargetPosition(positionWrist);
                }, this::stop
        ).until(this::atTargetPositions);
    }

    public Command followPositions(DoubleSupplier positionShoulder, DoubleSupplier positionWrist) {
        return runEnd(() -> {
            // Shoulder
            targetShoulderPosition = positionShoulder.getAsDouble();
            io.setShoulderTargetPosition(targetShoulderPosition);

            // Wrist
            targetWristPosition = positionWrist.getAsDouble();
            io.setWristTargetPosition(targetWristPosition);
        }, this::stop);
    }

    public double getShoulderPosition() {
        return inputs.currentPositionShoulder;
    }

    public boolean atShoulderTargetPosition() {
        return Math.abs(inputs.currentPositionShoulder - targetShoulderPosition) < POSITION_THRESHOLD;
    }

    public Command resetShoulderPosition() {
        return runOnce(io::resetShoulderPosition);
    }

    public double getWristPosition() {
        return inputs.currentPositionWrist;
    }

    public boolean atWristTargetPosition() {
        return Math.abs(inputs.currentPositionWrist - targetWristPosition) < POSITION_THRESHOLD;
    }

    public Command resetWristPosition() {
        return runOnce(io::resetWristPosition);
    }
}