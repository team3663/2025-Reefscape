package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final double MK4_2PLUS_REDUCTION = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double MK4_3PLUS_REDUCTION = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);

    public static final double MK4N_STEER_REDUCTION = 18.75;
    public static final double MK4N_STEER_INERTIA = 0.00001;
    public static final double MK4N_STEER_FRICTION_VOLTAGE = 0.25;
    public static final Slot0Configs MK4N_STEER_PID_CONSTANTS = new Slot0Configs()
            .withKP(100.0);
    public static final double MK4N_WHEEL_RADIUS = Units.inchesToMeters(4.0);

    public static final double MK4I_STEER_REDUCTION = (150.0 / 7.0);
    public static final double MK4I_STEER_INERTIA = 0.00001;
    public static final double MK4I_STEER_FRICTION_VOLTAGE = 0.25;
    public static final double MK4_WHEEL_RADIUS = Units.inchesToMeters(1.93);
    public static final Slot0Configs MK4I_STEER_PID_CONSTANTS = new Slot0Configs()
            .withKP(50);


    public static class ArmPositions {
        // TODO: get the actual values of these variables!!
        // Coral Station
        public static final double CORAL_STATION_ELEVATOR_HEIGHT = 0.0;
        public static final double CORAL_STATION_SHOULDER_ANGLE = 0.0;
        public static final double CORAL_STATION_WRIST_ANGLE = 0.0;

        // Shoulder Safe variables
        public static final double SHOULDER_SAFE_ANGLE = Units.degreesToRadians(90);
        public static final double SHOULDER_SAFE_THRESHOLD = Units.degreesToRadians(30);
    }

    // Vision camera constants
    // Roll, pitch & yaw values are in radians
    // X, Y & Z translation values are in meters
    // TODO Get real values from CAD/Mech.
    public static final String LEFT_CAMERA_NAME = "limelight-l";
    public static final double LEFT_CAMERA_ROLL = 0.0;
    public static final double LEFT_CAMERA_PITCH = 0.0;
    public static final double LEFT_CAMERA_YAW = 0.0;
    public static final double LEFT_CAMERA_X = 0.0;
    public static final double LEFT_CAMERA_Y = 0.0;
    public static final double LEFT_CAMERA_Z = 0.0;

    public static final String RIGHT_CAMERA_NAME = "limelight-r";
    public static final double RIGHT_CAMERA_ROLL = 0.0;
    public static final double RIGHT_CAMERA_PITCH = 0.0;
    public static final double RIGHT_CAMERA_YAW = 0.0;
    public static final double RIGHT_CAMERA_X = 0.0;
    public static final double RIGHT_CAMERA_Y = 0.0;
    public static final double RIGHT_CAMERA_Z = 0.0;
}