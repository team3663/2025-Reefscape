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
        // TODO: get the actual values of all of these variables!!
        // Coral Scoring
        // Level 1
        public static final double CORAL_LEVEL_1_ELEVATOR_HEIGHT = 0.0;
        public static final double CORAL_LEVEL_1_SHOULDER_ANGLE = 0.0;
        public static final double CORAL_LEVEL_1_WRIST_ANGLE = 0.0;
        // Level 2
        public static final double CORAL_LEVEL_2_ELEVATOR_HEIGHT = 0.0;
        public static final double CORAL_LEVEL_2_SHOULDER_ANGLE = 0.0;
        public static final double CORAL_LEVEL_2_WRIST_ANGLE = 0.0;
        // Level 3
        public static final double CORAL_LEVEL_3_ELEVATOR_HEIGHT = 0.0;
        public static final double CORAL_LEVEL_3_SHOULDER_ANGLE = 0.0;
        public static final double CORAL_LEVEL_3_WRIST_ANGLE = 0.0;
        // Level 4
        public static final double CORAL_LEVEL_4_ELEVATOR_HEIGHT = 0.0;
        public static final double CORAL_LEVEL_4_SHOULDER_ANGLE = 0.0;
        public static final double CORAL_LEVEL_4_WRIST_ANGLE = 0.0;

        // Algae Scoring
        // Net
        public static final double ALGAE_NET_ELEVATOR_HEIGHT = 0.0;
        public static final double ALGAE_NET_SHOULDER_ANGLE = 0.0;
        public static final double ALGAE_NET_WRIST_ANGLE = 0.0;
        // Processor
        public static final double ALGAE_PROCESSOR_ELEVATOR_HEIGHT = 0.0;
        public static final double ALGAE_PROCESSOR_SHOULDER_ANGLE = 0.0;
        public static final double ALGAE_PROCESSOR_WRIST_ANGLE = 0.0;

        // Removing Algae
        // Upper
        public static final double REMOVE_ALGAE_UPPER_ELEVATOR_HEIGHT = 0.0;
        public static final double REMOVE_ALGAE_UPPER_SHOULDER_ANGLE = 0.0;
        public static final double REMOVE_ALGAE_UPPER_WRIST_ANGLE = 0.0;
        // Lower
        public static final double REMOVE_ALGAE_LOWER_ELEVATOR_HEIGHT = 0.0;
        public static final double REMOVE_ALGAE_LOWER_SHOULDER_ANGLE = 0.0;
        public static final double REMOVE_ALGAE_LOWER_WRIST_ANGLE = 0.0;

        // Coral Station
        public static final double CORAL_STATION_ELEVATOR_HEIGHT = 0.0;
        public static final double CORAL_STATION_SHOULDER_ANGLE = 0.0;
        public static final double CORAL_STATION_WRIST_ANGLE = 0.0;
    }
}