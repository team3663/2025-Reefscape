package frc.robot.config;

import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.grabber.GrabberIO;

public interface RobotFactory {
    default DrivetrainIO createDrivetrainIo() {
        return new DrivetrainIO() {
        };
    }

    default ArmIO createArmIo() {
        return new ArmIO() {
        };
    }

    default ElevatorIO createElevatorIo() {
        return new ElevatorIO() {
        };
    }

    default GrabberIO createGrabberIo() {
        return new GrabberIO() {
        };
    }

    default ClimberIO createClimberIo() {
        return new ClimberIO() {
        };
    }
}
