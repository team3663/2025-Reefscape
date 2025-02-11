package frc.robot.subsystems.drivetrain;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;

public interface DrivetrainIO {
    default void updateInputs(DrivetrainInputs inputs) {
    }

    /**
     * Drives field oriented with the ability to specify and X, Y, and Angular Velocity
     *
     * @param xVelocity       The target X (forward) velocity in meters per second.
     * @param yVelocity       The target Y (towards the left side of the robot) velocity in meters per second.
     * @param angularVelocity The target angular (counter-clockwise positive) velocity in radians per second.
     */
    default void driveFieldOriented(double xVelocity, double yVelocity, double angularVelocity) {
    }

    default void driveFieldOriented(SwerveSample sample) {
    }

    default void resetOdometry(Pose2d newPose) {
    }

    default void resetFieldOriented() {
    }

    default void followTrajectory(SwerveSample Sample) {
    }

    default void driveSysIdTranslation(Voltage voltage) {

    }


    default Drivetrain.Constants getConstants() {
        return new Drivetrain.Constants(
                5.0,
                Units.rotationsPerMinuteToRadiansPerSecond(60.0)
        );
    }

    default void addVisionMeasurement(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {
    }

    /**
     * Stops the drivetrain
     */
    default void stop() {
    }
}

