package frc.robot;

import edu.wpi.first.math.util.Units;

public enum RobotMode {
    // TODO: get the actual values of these variables!!
    CORAL_LEVEL_1(Units.inchesToMeters(0), Units.degreesToRadians(34.36), Units.degreesToRadians(40.34), false, false),
    CORAL_LEVEL_2(Units.inchesToMeters(4.1066), Units.degreesToRadians(51.5), Units.degreesToRadians(26.80), false, false),
    CORAL_LEVEL_3(Units.inchesToMeters(20.1066), Units.degreesToRadians(51.5), Units.degreesToRadians(26.80), false, false),
    CORAL_LEVEL_4(Units.inchesToMeters(58.858), Units.degreesToRadians(39.199), Units.degreesToRadians(-32.871), false, false),
    ALGAE_PROCESSOR(0, Units.degreesToRadians(90), Units.degreesToRadians(0), true, false),
    ALGAE_NET(0, Units.degreesToRadians(90), Units.degreesToRadians(0), true, false),
    ALGAE_REMOVE_LOWER(0, Units.degreesToRadians(90), Units.degreesToRadians(0), true, true),
    ALGAE_REMOVE_UPPER(Units.inchesToMeters(0), Units.degreesToRadians(90), Units.degreesToRadians(0), true, true),
    CORAL_STATION(Constants.ArmPositions.CORAL_STATION_ELEVATOR_HEIGHT, Constants.ArmPositions.CORAL_STATION_SHOULDER_ANGLE, Constants.ArmPositions.CORAL_STATION_WRIST_ANGLE, false, false);


    private final double elevatorHeight;
    private final double shoulderAngle;
    private final double wristAngle;
    private final boolean isAlgaeMode;
    private final boolean runGrabberReverse;

    RobotMode(double elevatorHeight, double shoulderAngle, double wristAngle, boolean isAlgaeMode, boolean runGrabberReverse) {
        this.elevatorHeight = elevatorHeight;
        this.shoulderAngle = shoulderAngle;
        this.wristAngle = wristAngle;
        this.isAlgaeMode = isAlgaeMode;
        this.runGrabberReverse = runGrabberReverse;
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

    public boolean isRunGrabberReverse() {
        return runGrabberReverse;
    }
}