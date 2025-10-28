package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

@Logged
public class VisionMeasurement2 {
    public Pose2d estimatedPose;
    public double timestamp;
    @NotLogged
    public Matrix<N3, N1> stdDevs;

    public VisionMeasurement2(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        this.estimatedPose = pose;
        this.timestamp = timestamp;
        this.stdDevs = stdDevs;
    }
}