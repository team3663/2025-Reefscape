package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmPositions;
import frc.robot.utility.Gamepiece;

public enum RobotMode {
    // TODO: get the actual values of these variables!!
    CORAL_LEVEL_1(Units.inchesToMeters(0.53), Units.degreesToRadians(28.48), Units.degreesToRadians(76.29), Gamepiece.CORAL, true),
    CORAL_LEVEL_2(Units.inchesToMeters(4.458), Units.degreesToRadians(44.121), Units.degreesToRadians(36.211), Gamepiece.CORAL, true),
    CORAL_LEVEL_3(Units.inchesToMeters(22.037), Units.degreesToRadians(40.166), Units.degreesToRadians(31.377), Gamepiece.CORAL, true),
    CORAL_LEVEL_4(Units.inchesToMeters(51.522), Units.degreesToRadians(47.109), Units.degreesToRadians(-9.580), Gamepiece.CORAL, true),
    ALGAE_PROCESSOR(0, Units.degreesToRadians(7.0), Units.degreesToRadians(70.0), Gamepiece.ALGAE, true),
    ALGAE_NET(Units.inchesToMeters(60.5), Units.degreesToRadians(90.0), Units.degreesToRadians(40.0), Gamepiece.ALGAE, true),
    // ALGAE_REMOVE_LOWER positions for when we are one coral length away from the reef to be implemented later:
    // ALGAE_REMOVE_LOWER(0.48, Units.degreesToRadians(12.13), Units.degreesToRadians(51.33), Gamepiece.ALGAE, false),
    ALGAE_REMOVE_LOWER(Units.inchesToMeters(9.52), Units.degreesToRadians(31.28), Units.degreesToRadians(62.24), Gamepiece.ALGAE, false),
    ALGAE_REMOVE_UPPER(Units.inchesToMeters(26.02), Units.degreesToRadians(31.28), Units.degreesToRadians(62.24), Gamepiece.ALGAE, false),
    CORAL_STATION(ArmPositions.CORAL_STATION_ELEVATOR_HEIGHT, ArmPositions.CORAL_STATION_SHOULDER_ANGLE, ArmPositions.CORAL_STATION_WRIST_ANGLE, Gamepiece.CORAL, false),
    ALGAE_PICKUP_GROUND(Units.inchesToMeters(0.0), Units.degreesToRadians(0.0), Units.degreesToRadians(30.0), Gamepiece.ALGAE, false);

    private final double elevatorHeight;
    private final double shoulderAngle;
    private final double wristAngle;
    private final Gamepiece gamepiece;
    private final boolean placingMode;

    RobotMode(double elevatorHeight, double shoulderAngle, double wristAngle, Gamepiece gamepiece, boolean placingMode) {
        this.elevatorHeight = elevatorHeight;
        this.shoulderAngle = shoulderAngle;
        this.wristAngle = wristAngle;
        this.gamepiece = gamepiece;
        this.placingMode = placingMode;
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

    public boolean isPlacingMode() {
        return placingMode;
    }
}