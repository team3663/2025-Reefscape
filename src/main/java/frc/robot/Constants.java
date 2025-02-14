package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.List;

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
    public static final double WHEEL_COF = 1.2;
    public static final Slot0Configs MK4I_STEER_PID_CONSTANTS = new Slot0Configs()
            .withKP(50);

    public static final Pose2d BLUE_BRANCH_A = new Pose2d(3.2095999717712402, 4.149653434753418, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_BRANCH_B = new Pose2d(3.2095999717712402, 3.861128807067871, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_BRANCH_C = new Pose2d(3.7248220443725586, 2.9955556392669678, Rotation2d.fromRadians(-2.1471722386632273));
    public static final Pose2d BLUE_BRANCH_D = new Pose2d(4.0133466720581055, 2.8306846618652344, Rotation2d.fromRadians(-2.1471722386632273));
    public static final Pose2d BLUE_BRANCH_E = new Pose2d(5.023182392120361, 2.8306844234466553, Rotation2d.fromRadians(-1.030375862463906));
    public static final Pose2d BLUE_BRANCH_F = new Pose2d(5.291097640991211, 2.9955556392669678, Rotation2d.fromRadians(-1.030375862463906));
    public static final Pose2d BLUE_BRANCH_G = new Pose2d(5.7663140296936035, 3.8684024810791016, Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_BRANCH_H = new Pose2d(5.7663140296936035, 4.183597087860107, Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_BRANCH_I = new Pose2d(5.257153511047363, 5.032198429107666, Rotation2d.fromRadians(1.063697673973628));
    public static final Pose2d BLUE_BRANCH_J = new Pose2d(4.9662041664123535, 5.214041709899902, Rotation2d.fromRadians(1.063697673973628));
    public static final Pose2d BLUE_BRANCH_K = new Pose2d(4.020619869232178, 5.238287448883057, Rotation2d.fromRadians(2.1785979220562233));
    public static final Pose2d BLUE_BRANCH_L = new Pose2d(3.71754789352417, 5.044321537017822, Rotation2d.fromRadians((2.1785979220562233)));

    public static final Pose2d WALL_SIDE_CORAL_STATION = new Pose2d(1.6524484157562256, 7.5020751953125, Rotation2d.fromRadians(2.2011979827768244));
    public static final Pose2d DRIVER_SIDE_CORAL_STATION = new Pose2d(0.5996723175048828, 6.717278480529785, Rotation2d.fromRadians(2.2011979827768244));
    public static final Pose2d PROCESSOR_SIDE_WALL_SIDE_CORAL_STATION = new Pose2d(1.6639333963394165, 0.5604525804519653, Rotation2d.fromRadians(-2.2404352392387104));
    public static final Pose2d PROCESSOR_SIDE_DRIVER_SIDE_CORAL_STATION = new Pose2d(0.6226420998573303, 1.276340365409851, Rotation2d.fromRadians(-2.2404352392387104));

    static final List<Pose2d> blueBranchPoses = List.of(BLUE_BRANCH_A, BLUE_BRANCH_B,
            BLUE_BRANCH_C, BLUE_BRANCH_D, BLUE_BRANCH_E, BLUE_BRANCH_F,
            BLUE_BRANCH_G, BLUE_BRANCH_H, BLUE_BRANCH_I, BLUE_BRANCH_J,
            BLUE_BRANCH_K, BLUE_BRANCH_L);

    static final List<Pose2d> coralStationPoses = List.of(WALL_SIDE_CORAL_STATION, DRIVER_SIDE_CORAL_STATION,
            PROCESSOR_SIDE_DRIVER_SIDE_CORAL_STATION, PROCESSOR_SIDE_WALL_SIDE_CORAL_STATION);

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
}