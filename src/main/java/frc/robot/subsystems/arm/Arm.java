package frc.robot.subsystems.arm;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

@Logged
public class Arm extends SubsystemBase {
    public static final double POSITION_THRESHOLD = Units.degreesToRadians(5);
    private final ArmIO io;
    private final ArmInputs inputs = new ArmInputs();
    private final Arm.Constants constants;
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
    public boolean atPositions(double shoulderPosition, double wristPosition) {
        return this.shoulderAtPosition(shoulderPosition, POSITION_THRESHOLD) && this.wristAtPosition(wristPosition, POSITION_THRESHOLD);
    }

    public Command goToPositions(double shoulderPosition, double wristPosition) {
        return runEnd(() -> {
                    // Shoulder
                    targetShoulderPosition = shoulderPosition;
                    io.setShoulderTargetPosition(shoulderPosition);

                    // Wrist
                    targetWristPosition = wristPosition;
                    io.setWristTargetPosition(wristPosition);
                }, this::stop
        ).until(this::atTargetPositions);
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
        return inputs.currentShoulderPosition;
    }

    public boolean atShoulderTargetPosition() {
        return shoulderAtPosition(targetShoulderPosition, POSITION_THRESHOLD);
    }

    public boolean shoulderAtPosition(double position, double threshold) {
        return Math.abs(inputs.currentShoulderPosition - position) < threshold;
    }

    public double getWristPosition() {
        return inputs.currentWristPosition;
    }

    public boolean atWristTargetPosition() {
        return wristAtPosition(targetWristPosition, POSITION_THRESHOLD);
    }

    public boolean wristAtPosition(double position, double threshold) {
        return Math.abs(inputs.currentWristPosition - position) < threshold;
    }

    public Command resetWristPosition() {
        return runOnce(io::resetWristPosition);
    }


    public Command zeroWrist() {
        return runEnd(() -> io.setWristTargetVoltage(-1.0),
                io::stopWrist)
                .withDeadline(waitUntil(() -> Math.abs(inputs.currentWristVelocity) < 0.01)
                        .beforeStarting(waitSeconds(0.25))
                        .andThen(io::resetWristPosition));
    }

    public record Constants(double shoulderLength, double minimumShoulderAngle, double maximumShoulderAngle,
                            double wristLength, double minimumWristAngle, double maximumWristAngle) {
    }
}