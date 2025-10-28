package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.RobotMode;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionInputs;
import frc.robot.subsystems.vision.VisionMeasurement;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

@Logged
public class Vision2 extends SubsystemBase {

    private static final InterpolatingMatrixTreeMap<Double, N3, N1> MEASUREMENT_STD_DEV_DISTANCE_MAP = new InterpolatingMatrixTreeMap<>();

    @NotLogged
    private final VisionIO[] ios;
    @NotLogged
    private final VisionInputs[] visionInputs;

    private final VisionInputs leftInputs;
    private final VisionInputs rightInputs;
    private final VisionInputs backInputs;

    // current yaw of robot as provided by the pigeon
    private Rotation2d currentYaw = new Rotation2d();
    @NotLogged
    private final List<VisionMeasurement> acceptedMeasurements = new ArrayList<>();
    private final double[] ioUpdateDurations;
    private final double[] processingDurations;
    private RobotMode robotMode = RobotMode.CORAL_LEVEL_1;

    static {
        MEASUREMENT_STD_DEV_DISTANCE_MAP.put(0.1, VecBuilder.fill(0.05, 0.05, 0.05));
        MEASUREMENT_STD_DEV_DISTANCE_MAP.put(8.0, VecBuilder.fill(3.0, 3.0, 3.0));
    }

    public Vision2(VisionIO... ios) {
        this.ios = ios;

        this.ioUpdateDurations = new double[ios.length];
        this.processingDurations = new double[ios.length];

        visionInputs = new VisionInputs[ios.length];
        for (int i = 0; i < visionInputs.length; i++) {
            visionInputs[i] = new VisionInputs();
        }
        if (visionInputs.length > 0) {
            leftInputs = visionInputs[0];
        } else {
            leftInputs = new VisionInputs();
        }
        if (visionInputs.length > 1) {
            rightInputs = visionInputs[1];
        } else {
            rightInputs = new VisionInputs();
        }

        if (visionInputs.length > 2) {
            backInputs = visionInputs[2];
        } else {
            backInputs = new VisionInputs();
        }

        // Register the command we use to detect when the robot is enabled/disabled.
        RobotModeTriggers.disabled().onChange(updateRobotState());
    }

    @Override
    public void periodic() {
        acceptedMeasurements.clear();

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
                                             Supplier<Rotation2d> yawSupplier, Supplier<RobotMode> robotmode) {
        return run(() -> {
            visionMeasurementConsumer.accept(acceptedMeasurements);
            currentYaw = yawSupplier.get();
            robotMode = robotmode.get();
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