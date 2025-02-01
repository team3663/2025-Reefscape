package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

@Logged
public class VisionMeasurement {
    public Pose2d estimatedPose;
    public double timestamp;
    public Matrix<N3, N1> stdDevs;

    public VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        this.estimatedPose = pose;
        this.timestamp = timestamp;
        this.stdDevs = stdDevs;
    }
}
