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

    public Command stopMotors() {
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
            io.resetPositionShoulder();
            io.resetPositionWrist();
        });
    }

    public boolean atTargetPositions() {
        return this.atTargetPositionShoulder() && this.atTargetPositionWrist();
    }

    public Command goToPositions(double positionShoulder, double positionWrist) {
        return runEnd(() -> {
                    // Shoulder
                    targetShoulderPosition = positionShoulder;
                    io.setTargetPositionShoulder(targetShoulderPosition);

                    // Wrist
                    targetWristPosition = positionWrist;
                    io.setTargetPositionWrist(targetWristPosition);
                }, this::stop
        ).until(this::atTargetPositions);
    }

    public Command followPositions(DoubleSupplier positionShoulder, DoubleSupplier positionWrist) {
        return runEnd(() -> {
                    // Shoulder
                    targetShoulderPosition = positionShoulder.getAsDouble();
                    io.setTargetPositionShoulder(targetShoulderPosition);

                    // Wrist
                    targetWristPosition = positionWrist.getAsDouble();
                    io.setTargetPositionWrist(targetWristPosition);
                }, this::stop);
    }

    public double getPositionShoulder() {
        return inputs.currentPositionShoulder;
    }

    public boolean atTargetPositionShoulder() {
        return Math.abs(inputs.currentPositionShoulder - targetShoulderPosition) < POSITION_THRESHOLD;
    }

    public Command stopShoulder() {
        return runOnce(() -> {
                    targetShoulderPosition = 0.0;
                    io.stopShoulder();
                }
        );
    }

    public Command resetPositionShoulder() {
        return runOnce(io::resetPositionShoulder);
    }

    public Command goToPositionShoulder(double position) {
        return runEnd(() -> {
                    io.setTargetPositionShoulder(position);
                    targetShoulderPosition = position;
                }, io::stopShoulder
        ).until(this::atTargetPositionShoulder);
    }

    public Command followPositionShoulder(DoubleSupplier position) {
        return runEnd(() -> {
            targetShoulderPosition = position.getAsDouble();
            io.setTargetPositionShoulder(targetShoulderPosition);
        }, io::stopShoulder);
    }

    public double getPositionWrist() {
        return inputs.currentPositionWrist;
    }

    public boolean atTargetPositionWrist() {
        return Math.abs(inputs.currentPositionWrist - targetWristPosition) < POSITION_THRESHOLD;
    }

    public Command stop() {
        return runOnce(() -> {
                    targetWristPosition = 0.0;
                    io.stopWrist();
                }
        );
    }

    public Command resetPositionWrist() {
        return runOnce(io::resetPositionWrist);
    }

    public Command goToPositionWrist(double position) {
        return runEnd(() -> {
                    io.setTargetPositionWrist(position);
                    targetWristPosition = position;
                }, io::stopWrist
        ).until(this::atTargetPositionWrist);
    }

    public Command followPositionWrist(DoubleSupplier position) {
        return runEnd(() -> {
            targetWristPosition = position.getAsDouble();
            io.setTargetPositionWrist(targetWristPosition);
        }, io::stopWrist);
    }
}