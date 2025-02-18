package frc.robot;

import edu.wpi.first.math.util.Units;

public enum RobotMode {
    // TODO: get the actual values of these variables!!
    CORAL_LEVEL_1(0.1, Units.degreesToRadians(0), Units.degreesToRadians(-70), false),
    CORAL_LEVEL_2(0.2, Units.degreesToRadians(20), Units.degreesToRadians(-52.5), false),
    CORAL_LEVEL_3(0.3, Units.degreesToRadians(40), Units.degreesToRadians(-35), false),
    CORAL_LEVEL_4(0.4, Units.degreesToRadians(60), Units.degreesToRadians(-17.5), false),
    ALGAE_PROCESSOR(0.5, Units.degreesToRadians(120), Units.degreesToRadians(17.5), true),
    ALGAE_NET(0.6, Units.degreesToRadians(140), Units.degreesToRadians(35), true),
    ALGAE_REMOVE_LOWER(0.7, Units.degreesToRadians(160), Units.degreesToRadians(52.5), true),
    ALGAE_REMOVE_UPPER(1.55, Units.degreesToRadians(180), Units.degreesToRadians(70), true),
    CORAL_STATION(Constants.ArmPositions.CORAL_STATION_ELEVATOR_HEIGHT, Constants.ArmPositions.CORAL_STATION_SHOULDER_ANGLE, Constants.ArmPositions.CORAL_STATION_WRIST_ANGLE, false);


    private final double elevatorHeight;
    private final double shoulderAngle;
    private final double wristAngle;
    private final boolean isAlgaeMode;

    RobotMode(double elevatorHeight, double shoulderAngle, double wristAngle, boolean isAlgaeMode) {
        this.elevatorHeight = elevatorHeight;
        this.shoulderAngle = shoulderAngle;
        this.wristAngle = wristAngle;
        this.isAlgaeMode = isAlgaeMode;
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

    public boolean isAlgaeMode() {
        return isAlgaeMode;
    }
}