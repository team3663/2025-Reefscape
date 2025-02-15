package frc.robot;

import java.util.EnumMap;
import java.util.Map;

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
    CORAL_STATION(Constants.ArmPositions.CORAL_STATION_ELEVATOR_HEIGHT, Constants.ArmPositions.CORAL_STATION_SHOULDER_ANGLE,
            Constants.ArmPositions.CORAL_STATION_WRIST_ANGLE, false);

    public final static int REVERSE_POSITION_ELEVATOR_HEIGHT_ID = 0;
    public final static int REVERSE_POSITION_SHOULDER_ANGLE_ID = 1;
    public final static int REVERSE_POSITION_WRIST_ANGLE_ID = 2;

    public static final Map<RobotMode, double[]> reverseCoralPositionsMap = new EnumMap<>(RobotMode.class);
    private final double elevatorHeight;
    private final double shoulderAngle;
    private final double wristAngle;
    private final boolean isAlgaeMode;

    RobotMode(double elevatorHeight, double shoulderAngle, double wristAngle, boolean isAlgaeMode) {
        this.elevatorHeight = elevatorHeight;
        this.shoulderAngle = shoulderAngle;
        this.wristAngle = wristAngle;
        this.isAlgaeMode = isAlgaeMode;
        initializeReverseCoralPositions();
    }

    private static void initializeReverseCoralPositions() {
        // TODO: get the actual values of these variables!!
        reverseCoralPositionsMap.put(CORAL_LEVEL_1, new double[] {0.1, -0.1, -0.1});
        reverseCoralPositionsMap.put(CORAL_LEVEL_2, new double[] {0.2, -0.2, -0.2});
        reverseCoralPositionsMap.put(CORAL_LEVEL_3, new double[] {0.3, -0.3, -0.3});
        reverseCoralPositionsMap.put(CORAL_LEVEL_4, new double[] {0.4, -0.4, -0.4});
        reverseCoralPositionsMap.put(CORAL_STATION, new double[] {Constants.ArmPositions.CORAL_STATION_REVERSE_ELEVATOR_HEIGHT,
                Constants.ArmPositions.CORAL_STATION_REVERSE_SHOULDER_ANGLE, Constants.ArmPositions.CORAL_STATION_REVERSE_WRIST_ANGLE});
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