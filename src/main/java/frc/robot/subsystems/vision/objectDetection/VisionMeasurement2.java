package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

@Logged
public class VisionMeasurement2 {
    public Translation2d estimatedTranslation;
    public int id;
    @NotLogged
    public Matrix<N3, N1> stdDev;

    public VisionMeasurement2(Translation2d pose, int id, Matrix<N3, N1> stdDev) {
        this.estimatedTranslation = pose;
        this.id = id;
        this.stdDev = stdDev;
    }
}