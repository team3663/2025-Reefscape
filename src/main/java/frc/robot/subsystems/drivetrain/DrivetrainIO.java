package frc.robot.subsystems.drivetrain;

public interface DrivetrainIO {
    default void updateInputs(DrivetrainInputs inputs) {
    }

    default void driveFieldOriented(double xVelocity, double yVelocity, double angularVelocity) {
    }

    default void stop() {
    }
}
