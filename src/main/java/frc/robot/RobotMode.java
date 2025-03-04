package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.utility.Gamepiece;

public enum RobotMode {
    // TODO: get the actual values of these variables!!
    CORAL_LEVEL_1(Units.inchesToMeters(0), Units.degreesToRadians(34.36), Units.degreesToRadians(40.34), Gamepiece.CORAL),
    CORAL_LEVEL_2(Units.inchesToMeters(3.6066), Units.degreesToRadians(51.5), Units.degreesToRadians(26.80), Gamepiece.CORAL),
    CORAL_LEVEL_3(Units.inchesToMeters(20.1066), Units.degreesToRadians(51.5), Units.degreesToRadians(26.80), Gamepiece.CORAL),
    CORAL_LEVEL_4(Units.inchesToMeters(47.94498), Units.degreesToRadians(65.9277), Units.degreesToRadians(3.61), Gamepiece.CORAL),
    ALGAE_PROCESSOR(0, Units.degreesToRadians(90), Units.degreesToRadians(70.0), Gamepiece.ALGAE),
    ALGAE_NET(1.55, Units.degreesToRadians(90), Units.degreesToRadians(0), Gamepiece.ALGAE),
    ALGAE_REMOVE_LOWER(Units.inchesToMeters(7.52), Units.degreesToRadians(34.28), Units.degreesToRadians(59.24), Gamepiece.ALGAE),
    ALGAE_REMOVE_UPPER(Units.inchesToMeters(24.02), Units.degreesToRadians(34.28), Units.degreesToRadians(59.24), Gamepiece.ALGAE),
    CORAL_STATION(Constants.ArmPositions.CORAL_STATION_ELEVATOR_HEIGHT, Constants.ArmPositions.CORAL_STATION_SHOULDER_ANGLE, Constants.ArmPositions.CORAL_STATION_WRIST_ANGLE, Gamepiece.CORAL);

    private final double elevatorHeight;
    private final double shoulderAngle;
    private final double wristAngle;
    private final Gamepiece gamepiece;

    RobotMode(double elevatorHeight, double shoulderAngle, double wristAngle, Gamepiece gamepiece) {
        this.elevatorHeight = elevatorHeight;
        this.shoulderAngle = shoulderAngle;
        this.wristAngle = wristAngle;
        this.gamepiece = gamepiece;
    }

    public double getElevatorHeight() {
        return elevatorHeight;
    }

    public double getWristAngle() {
        return wristAngle;
    }

    public double getShoulderAngle() {
        return shoulderAngle;
    }

    public Gamepiece getGamepiece() {
        return gamepiece;
    }
}