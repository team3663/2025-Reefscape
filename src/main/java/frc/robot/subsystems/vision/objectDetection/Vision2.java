package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.function.Supplier;

@Logged
public class Vision2 extends SubsystemBase {

    private static final InterpolatingMatrixTreeMap<Double, N3, N1> MEASUREMENT_STD_DEV_DISTANCE_MAP = new InterpolatingMatrixTreeMap<>();

    @NotLogged
    private final VisionIO2 io;
    @NotLogged
    private final VisionInputs2 visionInput;

    private int pipelineIndex = 1;

    // Current pose of the robot as provided by RobotContainer
    private Pose2d robotPose = new Pose2d();
    @NotLogged
    private final ArrayList<VisionMeasurement2> acceptedMeasurements = new ArrayList<>();
    private long measurmentsLastUpdated = 0;

    private double ioUpdateDuration;
    private double[] processingDurations;

    static {
        MEASUREMENT_STD_DEV_DISTANCE_MAP.put(0.1, VecBuilder.fill(0.05, 0.05, 0.05));
        MEASUREMENT_STD_DEV_DISTANCE_MAP.put(8.0, VecBuilder.fill(3.0, 3.0, 3.0));
    }

    public Vision2(VisionIO2 io) {
        this.io = io;

        visionInput = new VisionInputs2();

        // Register the command we use to detect when the robot is enabled/disabled.
        RobotModeTriggers.disabled().onChange(updateRobotState());

        setPipelineIndex(1);
    }

    @Override
    public void periodic() {
        double start = System.currentTimeMillis();
        io.updateInputs(visionInput, robotPose);
        double end = System.currentTimeMillis();
        double duration = end - start;
        ioUpdateDuration = duration;

        if (visionInput.ids.length > 0 || System.currentTimeMillis() - measurmentsLastUpdated >= Constants.ObjectDetection.MEASUREMENT_TIMEOUT)
            acceptedMeasurements.clear();
        // Loop through each of the detected pieces
        double[] durations = new double[visionInput.ids.length];
        for (int i = 0; i < visionInput.ids.length; i++) {
            start = System.currentTimeMillis();
            Translation2d translation = visionInput.translations[i];
            int id = visionInput.ids[i];

            Matrix<N3, N1> stdDev = MEASUREMENT_STD_DEV_DISTANCE_MAP.get(translation.getNorm());

            acceptedMeasurements.add(new VisionMeasurement2(translation, id, stdDev));
            durations[i] = System.currentTimeMillis() - start;
            measurmentsLastUpdated = System.currentTimeMillis();
        }
        processingDurations = durations;
    }

    /**
     * @return ArrayList of updated vision measurements to be passed to drivetrain.
     */
    public ArrayList<VisionMeasurement2> getVisionMeasurements() {
        return acceptedMeasurements;
    }

    public Pose2d[] getGamePieces() {
        ArrayList<VisionMeasurement2> pieceMeasurements = getVisionMeasurements();
        Pose2d[] poses = new Pose2d[pieceMeasurements.size()];
        for (int i = 0; i < pieceMeasurements.size(); i++)
            poses[i] = new Pose2d(pieceMeasurements.get(i).estimatedTranslation, Rotation2d.kZero);
        return poses;
    }

    public Pose2d getClosestGamePiece() {
        ArrayList<VisionMeasurement2> pieceMeasurements = getVisionMeasurements();
        if (pieceMeasurements.isEmpty()) return null;
        Translation2d pieceTranslation = pieceMeasurements.get(0).estimatedTranslation;
        return new Pose2d(pieceTranslation, Rotation2d.kZero);
    }

    public Pose2d getTargetForClosestGamePiece() {
        ArrayList<VisionMeasurement2> pieceMeasurements = getVisionMeasurements();
        if (pieceMeasurements.isEmpty()) return null;
        Translation2d pieceTranslation = pieceMeasurements.get(0).estimatedTranslation;
        Rotation2d rotation = pieceTranslation.minus(robotPose.getTranslation()).getAngle();
        Translation2d offset = new Translation2d(Constants.ObjectDetection.PICKUP_DISTANCE, rotation);
        return new Pose2d(pieceTranslation.minus(offset), rotation);
    }

    /**
     * Sets the pipeline index
     *
     * @param index The index of the new pipeline
     */
    public void setPipelineIndex(int index) {
        io.setPipelineIndex(index);
        pipelineIndex = index;
    }

    public int getPipelineIndex() {
        return pipelineIndex;
    }

    /**
     * @return Command that consumes vision measurements
     */
    public Command updateValues(Supplier<Pose2d> robotPose) {
        return runOnce(() -> this.robotPose = robotPose.get());
    }

    /**
     * @return Command that is called to let us detect changes in the RobotState
     */
    public Command updateRobotState() {
        // Let all of our IOs know that there has been a change in the robot state.
        return runOnce(io::robotStateChanged);
    }
}