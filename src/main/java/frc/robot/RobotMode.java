package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.utility.Gamepiece;
import frc.robot.Constants.ArmPositions;

public enum RobotMode {
    // TODO: get the actual values of these variables!!
    CORAL_LEVEL_1(Units.inchesToMeters(0), Units.degreesToRadians(34.36), Units.degreesToRadians(40.34), Gamepiece.CORAL, true),
    CORAL_LEVEL_2(Units.inchesToMeters(3.6066), Units.degreesToRadians(51.5), Units.degreesToRadians(26.80), Gamepiece.CORAL, true),
    CORAL_LEVEL_3(Units.inchesToMeters(20.1066), Units.degreesToRadians(51.5), Units.degreesToRadians(26.80), Gamepiece.CORAL, true),
    CORAL_LEVEL_4(Units.inchesToMeters(47.94498), Units.degreesToRadians(65.9277), Units.degreesToRadians(3.61), Gamepiece.CORAL, true),
    ALGAE_PROCESSOR(0, Units.degreesToRadians(90), Units.degreesToRadians(70.0), Gamepiece.ALGAE, true),
    ALGAE_NET(Units.inchesToMeters(60.5), Units.degreesToRadians(90), Units.degreesToRadians(0), Gamepiece.ALGAE, true),
    ALGAE_REMOVE_LOWER(Units.inchesToMeters(7.52), Units.degreesToRadians(34.28), Units.degreesToRadians(59.24), Gamepiece.ALGAE, false),
    ALGAE_REMOVE_UPPER(Units.inchesToMeters(24.02), Units.degreesToRadians(34.28), Units.degreesToRadians(59.24), Gamepiece.ALGAE, false),
    CORAL_STATION(ArmPositions.CORAL_STATION_ELEVATOR_HEIGHT, ArmPositions.CORAL_STATION_SHOULDER_ANGLE, ArmPositions.CORAL_STATION_WRIST_ANGLE, Gamepiece.CORAL, false);

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