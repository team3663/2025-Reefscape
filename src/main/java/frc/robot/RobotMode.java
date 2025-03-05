package frc.robot;

import edu.wpi.first.math.util.Units;

import java.util.EnumMap;
import java.util.Map;

public enum RobotMode {
    // TODO: get the actual values of these variables!!
    CORAL_LEVEL_1(Units.inchesToMeters(0), Units.degreesToRadians(34.36), Units.degreesToRadians(40.34), false, false),
    CORAL_LEVEL_2(Units.inchesToMeters(3.6066), Units.degreesToRadians(51.5), Units.degreesToRadians(26.80), false, false),
    CORAL_LEVEL_3(Units.inchesToMeters(20.1066), Units.degreesToRadians(51.5), Units.degreesToRadians(26.80), false, false),
    CORAL_LEVEL_4(Units.inchesToMeters(47.94498), Units.degreesToRadians(65.9277), Units.degreesToRadians(3.61), false, false),
    ALGAE_PROCESSOR(0, Units.degreesToRadians(90), Units.degreesToRadians(0), true, false),
    ALGAE_NET(1.55, Units.degreesToRadians(90), Units.degreesToRadians(0), true, false),
    ALGAE_REMOVE_LOWER(0, Units.degreesToRadians(90), Units.degreesToRadians(0), true, true),
    ALGAE_REMOVE_UPPER(Units.inchesToMeters(0), Units.degreesToRadians(90), Units.degreesToRadians(0), true, true),
    CORAL_STATION(Constants.ArmPositions.CORAL_STATION_ELEVATOR_HEIGHT, Constants.ArmPositions.CORAL_STATION_SHOULDER_ANGLE, Constants.ArmPositions.CORAL_STATION_WRIST_ANGLE, false, false);

    public final static int REVERSE_POSITION_ELEVATOR_HEIGHT_ID = 0;
    public final static int REVERSE_POSITION_SHOULDER_ANGLE_ID = 1;
    public final static int REVERSE_POSITION_WRIST_ANGLE_ID = 2;

    public static final Map<RobotMode, double[]> reverseCoralPositionsMap = new EnumMap<>(RobotMode.class);
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

    public boolean isRunGrabberReverse() {
        return runGrabberReverse;
    }
}