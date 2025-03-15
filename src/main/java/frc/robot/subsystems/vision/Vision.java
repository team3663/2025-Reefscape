package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import java.util.*;
import java.util.function.Consumer;
import java.util.function.Supplier;

@Logged
public class Vision extends SubsystemBase {

    private static final InterpolatingMatrixTreeMap<Double, N3, N1> MEASUREMENT_STD_DEV_DISTANCE_MAP = new InterpolatingMatrixTreeMap<>();

    @NotLogged
    private final VisionIO[] ios;
    @NotLogged
    private final AprilTagFieldLayout fieldLayout;
    @NotLogged
    private final VisionInputs[] visionInputs;

    private final VisionInputs frontInputs;
    private final VisionInputs backInputs;

    // current yaw of robot as provided by the pigeon
    private Rotation2d currentYaw = new Rotation2d();
    @NotLogged
    private final List<VisionMeasurement> acceptedMeasurements = new ArrayList<>();
    private final double[] ioUpdateDurations;
    private final double[] processingDurations;

    static {
        MEASUREMENT_STD_DEV_DISTANCE_MAP.put(0.1, VecBuilder.fill(0.05, 0.05, 0.05));
        MEASUREMENT_STD_DEV_DISTANCE_MAP.put(8.0, VecBuilder.fill(3.0, 3.0, 3.0));
    }

    public Vision(AprilTagFieldLayout fieldLayout, VisionIO... ios) {
        this.ios = ios;
        this.fieldLayout = fieldLayout;
        this.ioUpdateDurations = new double[ios.length];
        this.processingDurations = new double[ios.length];

        visionInputs = new VisionInputs[ios.length];
        for (int i = 0; i < visionInputs.length; i++) {
            visionInputs[i] = new VisionInputs();
        }
        if (visionInputs.length > 0) {
            frontInputs = visionInputs[0];
        } else {
            frontInputs = new VisionInputs();
        }
        if (visionInputs.length > 1) {
            backInputs = visionInputs[1];
        } else {
            backInputs = new VisionInputs();
        }

        // Register the command we use to detect when the robot is enabled/disabled.
        RobotModeTriggers.disabled().onChange(updateRobotState());
    }

    @Override
    public void periodic() {

        for (int i = 0; i < ios.length; i++) {
            double start = System.currentTimeMillis();
            ios[i].updateInputs(visionInputs[i], currentYaw.getRadians());
            double end = System.currentTimeMillis();
            double duration = end - start;
            ioUpdateDurations[i] = duration;
        }

        acceptedMeasurements.clear();
        for (int i = 0; i < visionInputs.length; i++) {
            VisionInputs visionInput = visionInputs[i];
            double start = System.currentTimeMillis();
            Pose2d pose = visionInput.estimatedPose;
            double timestamp = visionInput.timestampSeconds;

            // Skip inputs that haven't updated
            if (!visionInput.poseUpdated) continue;

            // Skip measurements that are not with in the field boundary
            if (pose.getX() < 0.0 || pose.getX() > fieldLayout.getFieldLength() ||
                    pose.getY() < 0.0 || pose.getY() > fieldLayout.getFieldWidth())
                continue;

            // Compute the standard deviation to use based on the distance to the closest tag
            double closestTagDistance = Double.MAX_VALUE;
            for (int targetId : visionInput.targetIds) {
                Optional<Pose3d> v = fieldLayout.getTagPose(targetId);
                if (v.isPresent()) {
                    double distance = v.get().toPose2d().getTranslation().getDistance(pose.getTranslation());
                    if (distance < closestTagDistance) {
                        closestTagDistance = distance;
                    }
                }
            }

            // If for some reason we were unable to calculate the distance to the closest tag, assume we are infinitely far away
            Matrix<N3, N1> stdDevs = MEASUREMENT_STD_DEV_DISTANCE_MAP.get(closestTagDistance);

            acceptedMeasurements.add(new VisionMeasurement(pose, timestamp, stdDevs));
            double end = System.currentTimeMillis();
            double duration = end - start;
            processingDurations[i] = duration;
        }
    }

    /**
     * @return List of updated vision measurements to be passed to drivetrain.
     */
    public List<VisionMeasurement> getVisionMeasurements() {
        return acceptedMeasurements;
    }

    /**
     * @return Command that consumes vision measurements
     */
    public Command consumeVisionMeasurements(Consumer<List<VisionMeasurement>> visionMeasurementConsumer,
                                             Supplier<Rotation2d> yawSupplier) {
        return run(() -> {
            visionMeasurementConsumer.accept(acceptedMeasurements);
            currentYaw = yawSupplier.get();
        });
    }

    /**
     * @return Command that is called to let us detect changes in the RobotState
     */
    public Command updateRobotState() {

        // Let all of our IOs know that there has been a change in the robot state.
        return runOnce(() -> {
            for (VisionIO io : ios) {
                io.robotStateChanged();
            }
        });
    }
}

