package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

    public static final double DEBOUNCE_TIME = 0.5;

    public static final double REEF_POSE_Y_OFFSET = Units.inchesToMeters(158.5);
    public static final double REEF_BLUE_POSE_X_OFFSET = Units.inchesToMeters(176.75);
    public static final double REEF_RED_POSE_X_OFFSET = Units.inchesToMeters(514.125);
    public static final double X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF = Units.inchesToMeters(31.625);
    public static final double Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF = Units.inchesToMeters(6.5);
    public static final Transform2d ROBOT_REEF_OFFSET = new Transform2d(Units.inchesToMeters(21.125), 0, Rotation2d.fromDegrees(180));
    public static final Transform2d ROBOT_CORAL_STATION_OFFSET = new Transform2d(Units.inchesToMeters(5), 0, Rotation2d.fromDegrees(180));

    public static final Pose2d BLUE_BRANCH_A = getBranchPose(3, false, REEF_BLUE_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d BLUE_BRANCH_B = getBranchPose(3, true, REEF_BLUE_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d BLUE_BRANCH_C = getBranchPose(4, false, REEF_BLUE_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d BLUE_BRANCH_D = getBranchPose(4, true, REEF_BLUE_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d BLUE_BRANCH_E = getBranchPose(5, false, REEF_BLUE_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d BLUE_BRANCH_F = getBranchPose(5, true, REEF_BLUE_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d BLUE_BRANCH_G = getBranchPose(0, false, REEF_BLUE_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d BLUE_BRANCH_H = getBranchPose(0, true, REEF_BLUE_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d BLUE_BRANCH_I = getBranchPose(1, false, REEF_BLUE_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d BLUE_BRANCH_J = getBranchPose(1, true, REEF_BLUE_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d BLUE_BRANCH_K = getBranchPose(2, false, REEF_BLUE_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d BLUE_BRANCH_L = getBranchPose(2, true, REEF_BLUE_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);

    public static final Pose2d RED_BRANCH_A = getBranchPose(0, true, REEF_RED_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d RED_BRANCH_B = getBranchPose(0, false, REEF_RED_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d RED_BRANCH_C = getBranchPose(1, true, REEF_RED_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d RED_BRANCH_D = getBranchPose(1, false, REEF_RED_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d RED_BRANCH_E = getBranchPose(2, true, REEF_RED_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d RED_BRANCH_F = getBranchPose(2, false, REEF_RED_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d RED_BRANCH_G = getBranchPose(3, true, REEF_RED_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d RED_BRANCH_H = getBranchPose(3, false, REEF_RED_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d RED_BRANCH_I = getBranchPose(4, true, REEF_RED_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d RED_BRANCH_J = getBranchPose(4, false, REEF_RED_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d RED_BRANCH_K = getBranchPose(5, true, REEF_RED_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);
    public static final Pose2d RED_BRANCH_L = getBranchPose(5, false, REEF_RED_POSE_X_OFFSET, REEF_POSE_Y_OFFSET);

    public static final Pose2d BLUE_LEFT_FAR_SIDE_CORAL_STATION = new Pose2d(1.6382254362106323, 7.540848731994629, Rotation2d.fromDegrees(306));
    public static final Pose2d BLUE_LEFT_NEAR_SIDE_CORAL_STATION = new Pose2d(0.5229208469390869, 6.728616237640381, Rotation2d.fromDegrees(306));
    public static final Pose2d BLUE_RIGHT_NEAR_SIDE_CORAL_STATION = new Pose2d(0.5559024810791016, 1.349269151687622, Rotation2d.fromDegrees(54));
    public static final Pose2d BLUE_RIGHT_FAR_SIDE_CORAL_STATION = new Pose2d(1.5863468647003174, 0.621896505355835, Rotation2d.fromDegrees(54));

    public static final Pose2d RED_LEFT_FAR_SIDE_CORAL_STATION = new Pose2d((17.548225 - 1.6382254362106323), 0.621896505355835, Rotation2d.fromDegrees(126));
    public static final Pose2d RED_LEFT_NEAR_SIDE_CORAL_STATION = new Pose2d((17.548225 - 0.5229208469390869), 1.349269151687622, Rotation2d.fromDegrees(126));
    public static final Pose2d RED_RIGHT_NEAR_SIDE_CORAL_STATION = new Pose2d((17.548225 - 0.5559024810791016), 6.728616237640381, Rotation2d.fromDegrees(234));
    public static final Pose2d RED_RIGHT_FAR_SIDE_CORAL_STATION = new Pose2d((17.548225 - 1.5863468647003174), 7.540848731994629, Rotation2d.fromDegrees(234));

    public static final List<Pose2d> BLUE_BRANCH_POSES = List.of(BLUE_BRANCH_A, BLUE_BRANCH_B,
            BLUE_BRANCH_C, BLUE_BRANCH_D, BLUE_BRANCH_E, BLUE_BRANCH_F,
            BLUE_BRANCH_G, BLUE_BRANCH_H, BLUE_BRANCH_I, BLUE_BRANCH_J,
            BLUE_BRANCH_K, BLUE_BRANCH_L);

    public static final List<Pose2d> RED_BRANCH_POSES = List.of(RED_BRANCH_A, RED_BRANCH_B,
            RED_BRANCH_C, RED_BRANCH_D, RED_BRANCH_E, RED_BRANCH_F,
            RED_BRANCH_G, RED_BRANCH_H, RED_BRANCH_I, RED_BRANCH_J,
            RED_BRANCH_K, RED_BRANCH_L);

    public static final List<Pose2d> BLUE_CORAL_STATION_POSES = List.of(BLUE_LEFT_NEAR_SIDE_CORAL_STATION, BLUE_LEFT_FAR_SIDE_CORAL_STATION,
            BLUE_RIGHT_NEAR_SIDE_CORAL_STATION, BLUE_RIGHT_FAR_SIDE_CORAL_STATION);

    public static final List<Pose2d> RED_CORAL_STATION_POSES = List.of(RED_LEFT_NEAR_SIDE_CORAL_STATION, RED_LEFT_FAR_SIDE_CORAL_STATION,
            RED_RIGHT_NEAR_SIDE_CORAL_STATION, RED_RIGHT_FAR_SIDE_CORAL_STATION);

    private static Pose2d getBranchPose(int number, boolean adding, double xOffset, double yOffset){
        if (adding) {
            return new Pose2d(Math.hypot(X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF, Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF) *
                    Math.cos(((Math.PI / 3) * number) + Math.asin(Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF / X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF)) + xOffset,
                    Math.hypot(X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF, Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF) * Math.sin(((Math.PI / 3) * number) +
                            Math.asin(Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF / X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF)) + yOffset,
                    Rotation2d.fromRadians((Math.PI / 3) * number));
        }
        else {
            return new Pose2d(Math.hypot(X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF, Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF) *
                    Math.cos(((Math.PI / 3) * number) - Math.asin(Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF / X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF)) + xOffset,
                    Math.hypot(X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF, Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF) * Math.sin((Math.PI / 3) * number -
                            Math.asin(Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF / X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF)) + yOffset,
                    Rotation2d.fromRadians((Math.PI / 3) * number));
        }
    }

    public static class ArmPositions {
        // TODO: get the actual values of these variables!!
        // Coral Station
        public static final double CORAL_STATION_ELEVATOR_HEIGHT = 0.0;
        public static final double CORAL_STATION_SHOULDER_ANGLE = 0.0;
        public static final double CORAL_STATION_WRIST_ANGLE = 0.0;

        // Shoulder Safe variables
        public static final double SHOULDER_SAFE_ANGLE = Units.degreesToRadians(90);
        public static final double SHOULDER_SAFE_THRESHOLD = Units.degreesToRadians(30);

        // Shoulder max angle when we have an algae
        public static final double SHOULDER_ALGAE_MAX_ANGLE = Units.degreesToRadians(70);

        // Default positions and angles
        public static final double ELEVATOR_DEFAULT_POSITION = 0;
        public static final double SHOULDER_DEFAULT_ANGLE = Units.degreesToRadians(90);
        public static final double WRIST_DEFAULT_ANGLE = 0;
    }

    // Arm buffers
    public static final double SHOULDER_BUFFER = Units.inchesToMeters(0.0);
    public static final double WRIST_BUFFER = Units.inchesToMeters(0.0);


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