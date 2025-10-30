package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

@Logged
public class Vision2 extends SubsystemBase {

    private static final InterpolatingMatrixTreeMap<Double, N3, N1> MEASUREMENT_STD_DEV_DISTANCE_MAP = new InterpolatingMatrixTreeMap<>();

    @NotLogged
    private final VisionIO2[] ios;
    @NotLogged
    private final VisionInputs2[] visionInputs;

    private final VisionInputs2 leftInputs;
    private final VisionInputs2 rightInputs;
    private final VisionInputs2 backInputs;

    // current yaw of robot as provided by the pigeon
    private Rotation2d currentYaw = new Rotation2d();
    @NotLogged
    private final List<VisionMeasurement2> acceptedMeasurements = new ArrayList<>();
    private final double[] ioUpdateDurations;
    private final double[] processingDurations;

    static {
        MEASUREMENT_STD_DEV_DISTANCE_MAP.put(0.1, VecBuilder.fill(0.05, 0.05, 0.05));
        MEASUREMENT_STD_DEV_DISTANCE_MAP.put(8.0, VecBuilder.fill(3.0, 3.0, 3.0));
    }

    public Vision2(VisionIO2... ios) {
        this.ios = ios;

        this.ioUpdateDurations = new double[ios.length];
        this.processingDurations = new double[ios.length];

        visionInputs = new VisionInputs2[ios.length];
        for (int i = 0; i < visionInputs.length; i++) {
            visionInputs[i] = new VisionInputs2();
        }
        if (visionInputs.length > 0) {
            leftInputs = visionInputs[0];
        } else {
            leftInputs = new VisionInputs2();
        }
        if (visionInputs.length > 1) {
            rightInputs = visionInputs[1];
        } else {
            rightInputs = new VisionInputs2();
        }

        if (visionInputs.length > 2) {
            backInputs = visionInputs[2];
        } else {
            backInputs = new VisionInputs2();
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
            VisionInputs2 visionInput = visionInputs[i];

            // Skip inputs that haven't updated
            if (!visionInput.poseUpdated[i]) continue;

            double start = System.currentTimeMillis();
            Translation2d pose = visionInput.poses[i];
            int id = visionInput.ids[i];

            Matrix<N3, N1> stdDev = MEASUREMENT_STD_DEV_DISTANCE_MAP.get(pose.getNorm());

            acceptedMeasurements.add(new VisionMeasurement2(pose, id, stdDev));
            double duration = System.currentTimeMillis() - start;
            processingDurations[i] = duration;
        }
    }

    /**
     * @return List of updated vision measurements to be passed to drivetrain.
     */
    public List<VisionMeasurement2> getVisionMeasurements() {
        return acceptedMeasurements;
    }

    /**
     * @return Command that consumes vision measurements
     */
    public Command updateValues(Supplier<Rotation2d> yawSupplier) {
        return run(() -> currentYaw = yawSupplier.get());
    }

    /**
     * @return Command that is called to let us detect changes in the RobotState
     */
    public Command updateRobotState() {
        // Let all of our IOs know that there has been a change in the robot state.
        return runOnce(() -> {
            for (VisionIO2 io : ios) {
                io.robotStateChanged();
            }
        });
    }
}