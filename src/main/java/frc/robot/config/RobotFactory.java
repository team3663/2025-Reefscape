package frc.robot.config;

import frc.robot.subsystems.drivetrain.DrivetrainIO;

public interface RobotFactory {
    default DrivetrainIO createDrivetrainIo() {
        return new DrivetrainIO() {
        };
    }
}
