package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.util.Units;

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

    default void resetFieldOriented() {

    }

    default Drivetrain.Constants getConstants() {
        return new Drivetrain.Constants(
                5.0,
                Units.rotationsPerMinuteToRadiansPerSecond(60.0)
        );
    }

    /**
     * Stops the drivetrain
     */
    default void stop() {
    }
}
