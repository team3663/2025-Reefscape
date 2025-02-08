package frc.robot;

public enum RobotMode {
    CORAL_LEVEL_1(0.1, 0.1, 0.1),
    CORAL_LEVEL_2(0.2, 0.2, 0.2),
    CORAL_LEVEL_3(0.3, 0.3, 0.3),
    CORAL_LEVEL_4(0.4, 0.4, 0.4),
    ALGAE_PROCESSOR(0.5, 0.5, 0.5),
    ALGAE_NET(0.6, 0.6, 0.6),
    ALGAE_REMOVE_LOWER(0.7, 0.7, 0.7),
    ALGAE_REMOVE_UPPER(0.8, 0.8, 0.8);

    private final double elevatorHeight;
    private final double shoulderAngle;
    private final double wristAngle;

    RobotMode(double elevatorHeight, double shoulderAngle, double wristAngle) {
        this.elevatorHeight = elevatorHeight;
        this.shoulderAngle = shoulderAngle;
        this.wristAngle = wristAngle;
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
}