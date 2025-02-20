package frc.robot;

public enum RobotMode {
    // TODO: get the actual values of these variables!!
    CORAL_LEVEL_1(0.1, 0.1, 0.1, false),
    CORAL_LEVEL_2(0.2, 0.2, 0.2, false),
    CORAL_LEVEL_3(0.3, 0.3, 0.3, false),
    CORAL_LEVEL_4(0.4, 0.4, 0.4, false),
    ALGAE_PROCESSOR(0.5, 0.5, 0.5, true),
    ALGAE_NET(0.6, 0.6, 0.6, true),
    ALGAE_REMOVE_LOWER(0.7, 0.7, 0.7, true),
    ALGAE_REMOVE_UPPER(0.8, 0.8, 0.8, true),
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