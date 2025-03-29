package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

import java.util.List;

public class Constants {
    public static final boolean SUPERSTRUCTURE_COAST = false;

    public static final double MK4_2PLUS_REDUCTION = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double MK4_3PLUS_REDUCTION = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);

    public static final double MK4N_STEER_REDUCTION = 18.75;
    public static final double MK4N_STEER_INERTIA = 0.00001;
    public static final double MK4N_STEER_FRICTION_VOLTAGE = 0.25;
    public static final Slot0Configs MK4N_STEER_PID_CONSTANTS = new Slot0Configs()
            .withKP(100.0);

    public static final double MK4I_STEER_REDUCTION = (150.0 / 7.0);
    public static final double MK4I_STEER_INERTIA = 0.00001;
    public static final double MK4I_STEER_FRICTION_VOLTAGE = 0.25;
    public static final double MK4_WHEEL_RADIUS = Units.inchesToMeters(1.93);
    public static final double WHEEL_COF = 1.2;
    public static final Slot0Configs MK4I_STEER_PID_CONSTANTS = new Slot0Configs()
            .withKP(50);

    public static final double DEBOUNCE_TIME = 0.5;
    public static final boolean IS_ANDYMARK = false;
    public static final AprilTagFieldLayout FIELD =
            AprilTagFieldLayout.loadField(IS_ANDYMARK ? AprilTagFields.k2025ReefscapeAndyMark : AprilTagFields.k2025ReefscapeWelded);

    private static final Transform2d LEFT_BRANCH_ARM_OFFSET = new Transform2d(Units.inchesToMeters(18.0), -Units.inchesToMeters(6.875), Rotation2d.fromDegrees(180));
    private static final Transform2d RIGHT_BRANCH_ARM_OFFSET = new Transform2d(Units.inchesToMeters(18.0), Units.inchesToMeters(7.125), Rotation2d.fromDegrees(180));

    private static final Transform2d CENTER_OFFSET = new Transform2d(Units.inchesToMeters(16.5), 0.0, Rotation2d.fromDegrees(180));

    public static final Transform2d LEFT_FAR_CORAL_STATION_OFFSET = new Transform2d(Units.inchesToMeters(15.0), Units.inchesToMeters(14.5), Rotation2d.kZero);
    public static final Transform2d LEFT_NEAR_CORAL_STATION_OFFSET = new Transform2d(Units.inchesToMeters(15.0), -Units.inchesToMeters(19.0), Rotation2d.kZero);

    public static final Transform2d RIGHT_FAR_CORAL_STATION_OFFSET = new Transform2d(Units.inchesToMeters(15.0), -Units.inchesToMeters(17.5), Rotation2d.kZero);
    public static final Transform2d RIGHT_NEAR_CORAL_STATION_OFFSET = new Transform2d(Units.inchesToMeters(15.0), Units.inchesToMeters(16.5), Rotation2d.kZero);

    public static final Pose2d BLUE_LEFT_FAR_SIDE_CORAL_STATION = FIELD.getTagPose(13).get().toPose2d().plus(LEFT_FAR_CORAL_STATION_OFFSET);
    public static final Pose2d BLUE_LEFT_NEAR_SIDE_CORAL_STATION = FIELD.getTagPose(13).get().toPose2d().plus(LEFT_NEAR_CORAL_STATION_OFFSET);
    public static final Pose2d BLUE_RIGHT_NEAR_SIDE_CORAL_STATION = FIELD.getTagPose(12).get().toPose2d().plus(RIGHT_NEAR_CORAL_STATION_OFFSET);
    public static final Pose2d BLUE_RIGHT_FAR_SIDE_CORAL_STATION = FIELD.getTagPose(12).get().toPose2d().plus(RIGHT_FAR_CORAL_STATION_OFFSET);

    //    public static final Pose2d RED_LEFT_FAR_SIDE_CORAL_STATION = new Pose2d(Units.inchesToMeters(628.78430181), 0.7049461603164673, Rotation2d.fromDegrees(126));
    public static final Pose2d RED_LEFT_FAR_SIDE_CORAL_STATION = FIELD.getTagPose(1).get().toPose2d().plus(LEFT_FAR_CORAL_STATION_OFFSET);
    public static final Pose2d RED_LEFT_NEAR_SIDE_CORAL_STATION = FIELD.getTagPose(1).get().toPose2d().plus(LEFT_NEAR_CORAL_STATION_OFFSET);
    public static final Pose2d RED_RIGHT_NEAR_SIDE_CORAL_STATION = FIELD.getTagPose(2).get().toPose2d().plus(RIGHT_NEAR_CORAL_STATION_OFFSET);
    public static final Pose2d RED_RIGHT_FAR_SIDE_CORAL_STATION = FIELD.getTagPose(2).get().toPose2d().plus(RIGHT_FAR_CORAL_STATION_OFFSET);


    public static final Pose2d RED_BRANCH_A1 = FIELD.getTagPose(7).get().toPose2d().plus(LEFT_BRANCH_ARM_OFFSET);
    public static final Pose2d RED_BRANCH_A2 = FIELD.getTagPose(7).get().toPose2d().plus(RIGHT_BRANCH_ARM_OFFSET);
    public static final Pose2d RED_BRANCH_B1 = FIELD.getTagPose(8).get().toPose2d().plus(LEFT_BRANCH_ARM_OFFSET);
    public static final Pose2d RED_BRANCH_B2 = FIELD.getTagPose(8).get().toPose2d().plus(RIGHT_BRANCH_ARM_OFFSET);
    public static final Pose2d RED_BRANCH_C1 = FIELD.getTagPose(9).get().toPose2d().plus(LEFT_BRANCH_ARM_OFFSET);
    public static final Pose2d RED_BRANCH_C2 = FIELD.getTagPose(9).get().toPose2d().plus(RIGHT_BRANCH_ARM_OFFSET);
    public static final Pose2d RED_BRANCH_D1 = FIELD.getTagPose(10).get().toPose2d().plus(LEFT_BRANCH_ARM_OFFSET);
    public static final Pose2d RED_BRANCH_D2 = FIELD.getTagPose(10).get().toPose2d().plus(RIGHT_BRANCH_ARM_OFFSET);
    public static final Pose2d RED_BRANCH_E1 = FIELD.getTagPose(11).get().toPose2d().plus(LEFT_BRANCH_ARM_OFFSET);
    public static final Pose2d RED_BRANCH_E2 = FIELD.getTagPose(11).get().toPose2d().plus(RIGHT_BRANCH_ARM_OFFSET);
    public static final Pose2d RED_BRANCH_F1 = FIELD.getTagPose(6).get().toPose2d().plus(LEFT_BRANCH_ARM_OFFSET);
    public static final Pose2d RED_BRANCH_F2 = FIELD.getTagPose(6).get().toPose2d().plus(RIGHT_BRANCH_ARM_OFFSET);

    public static final Pose2d RED_CENTER_A = FIELD.getTagPose(7).get().toPose2d().plus(CENTER_OFFSET);
    public static final Pose2d RED_CENTER_B = FIELD.getTagPose(8).get().toPose2d().plus(CENTER_OFFSET);
    public static final Pose2d RED_CENTER_C = FIELD.getTagPose(9).get().toPose2d().plus(CENTER_OFFSET);
    public static final Pose2d RED_CENTER_D = FIELD.getTagPose(10).get().toPose2d().plus(CENTER_OFFSET);
    public static final Pose2d RED_CENTER_E = FIELD.getTagPose(11).get().toPose2d().plus(CENTER_OFFSET);
    public static final Pose2d RED_CENTER_F = FIELD.getTagPose(6).get().toPose2d().plus(CENTER_OFFSET);


    public static final Pose2d BLUE_BRANCH_A1 = FIELD.getTagPose(18).get().toPose2d().plus(LEFT_BRANCH_ARM_OFFSET);
    public static final Pose2d BLUE_BRANCH_A2 = FIELD.getTagPose(18).get().toPose2d().plus(RIGHT_BRANCH_ARM_OFFSET);
    public static final Pose2d BLUE_BRANCH_B1 = FIELD.getTagPose(17).get().toPose2d().plus(LEFT_BRANCH_ARM_OFFSET);
    public static final Pose2d BLUE_BRANCH_B2 = FIELD.getTagPose(17).get().toPose2d().plus(RIGHT_BRANCH_ARM_OFFSET);
    public static final Pose2d BLUE_BRANCH_C1 = FIELD.getTagPose(22).get().toPose2d().plus(LEFT_BRANCH_ARM_OFFSET);
    public static final Pose2d BLUE_BRANCH_C2 = FIELD.getTagPose(22).get().toPose2d().plus(RIGHT_BRANCH_ARM_OFFSET);
    public static final Pose2d BLUE_BRANCH_D1 = FIELD.getTagPose(21).get().toPose2d().plus(LEFT_BRANCH_ARM_OFFSET);
    public static final Pose2d BLUE_BRANCH_D2 = FIELD.getTagPose(21).get().toPose2d().plus(RIGHT_BRANCH_ARM_OFFSET);
    public static final Pose2d BLUE_BRANCH_E1 = FIELD.getTagPose(20).get().toPose2d().plus(LEFT_BRANCH_ARM_OFFSET);
    public static final Pose2d BLUE_BRANCH_E2 = FIELD.getTagPose(20).get().toPose2d().plus(RIGHT_BRANCH_ARM_OFFSET);
    public static final Pose2d BLUE_BRANCH_F1 = FIELD.getTagPose(19).get().toPose2d().plus(LEFT_BRANCH_ARM_OFFSET);
    public static final Pose2d BLUE_BRANCH_F2 = FIELD.getTagPose(19).get().toPose2d().plus(RIGHT_BRANCH_ARM_OFFSET);

    public static final Pose2d BLUE_CENTER_A = FIELD.getTagPose(18).get().toPose2d().plus(CENTER_OFFSET);
    public static final Pose2d BLUE_CENTER_B = FIELD.getTagPose(17).get().toPose2d().plus(CENTER_OFFSET);
    public static final Pose2d BLUE_CENTER_C = FIELD.getTagPose(22).get().toPose2d().plus(CENTER_OFFSET);
    public static final Pose2d BLUE_CENTER_D = FIELD.getTagPose(21).get().toPose2d().plus(CENTER_OFFSET);
    public static final Pose2d BLUE_CENTER_E = FIELD.getTagPose(20).get().toPose2d().plus(CENTER_OFFSET);
    public static final Pose2d BLUE_CENTER_F = FIELD.getTagPose(19).get().toPose2d().plus(CENTER_OFFSET);


    public static final List<Pose2d> RED_BRANCH_POSES = List.of(RED_BRANCH_A1, RED_BRANCH_A2, RED_BRANCH_B1, RED_BRANCH_B2,
            RED_BRANCH_C1, RED_BRANCH_C2, RED_BRANCH_D1, RED_BRANCH_D2, RED_BRANCH_E1, RED_BRANCH_E2, RED_BRANCH_F1, RED_BRANCH_F2
    );

    public static final List<Pose2d> BLUE_BRANCH_POSES = List.of(BLUE_BRANCH_A1, BLUE_BRANCH_A2, BLUE_BRANCH_B1, BLUE_BRANCH_B2,
            BLUE_BRANCH_C1, BLUE_BRANCH_C2, BLUE_BRANCH_D1, BLUE_BRANCH_D2, BLUE_BRANCH_E1, BLUE_BRANCH_E2, BLUE_BRANCH_F1, BLUE_BRANCH_F2
    );

    public static final List<Pose2d> RED_CENTER_POSES = List.of(RED_CENTER_A, RED_CENTER_B, RED_CENTER_C,
            RED_CENTER_D, RED_CENTER_E, RED_CENTER_F
    );

    public static final List<Pose2d> BLUE_CENTER_POSES = List.of(BLUE_CENTER_A, BLUE_CENTER_B, BLUE_CENTER_C,
            BLUE_CENTER_D, BLUE_CENTER_E, BLUE_CENTER_F
    );


    public static final List<Pose2d> BLUE_CORAL_STATION_POSES = List.of(BLUE_LEFT_NEAR_SIDE_CORAL_STATION, BLUE_LEFT_FAR_SIDE_CORAL_STATION,
            BLUE_RIGHT_NEAR_SIDE_CORAL_STATION, BLUE_RIGHT_FAR_SIDE_CORAL_STATION);

    public static final List<Pose2d> RED_CORAL_STATION_POSES = List.of(RED_LEFT_NEAR_SIDE_CORAL_STATION, RED_LEFT_FAR_SIDE_CORAL_STATION,
            RED_RIGHT_NEAR_SIDE_CORAL_STATION, RED_RIGHT_FAR_SIDE_CORAL_STATION);

    public static final double NET_DISTANCE_FROM_CENTER_LINE = 0.9670559999999986;
    public static final double BLUE_NET_LINE_X = (FIELD.getFieldLength() / 2.0) - NET_DISTANCE_FROM_CENTER_LINE;
    public static final Rotation2d BLUE_NET_ROTATION = Rotation2d.fromDegrees(0.0);
    public static final double[] BLUE_NET_X_RANGE = {4.65, 7.6};
    public static final double RED_NET_LINE_X = (FIELD.getFieldLength() / 2.0) + NET_DISTANCE_FROM_CENTER_LINE;
    public static final Rotation2d RED_NET_ROTATION = Rotation2d.fromDegrees(180.0);
    public static final double[] RED_NET_X_RANGE = {0.5, 3.43};

    public static final double NET_MAX_SPEED = Units.inchesToMeters(10.0);

    public static class ArmPositions {
        // Coral Station
        public static final double CORAL_STATION_ELEVATOR_HEIGHT = Units.inchesToMeters(7.811);
        public static final double CORAL_STATION_SHOULDER_ANGLE = Units.degreesToRadians(137.0);
        public static final double CORAL_STATION_WRIST_ANGLE = Units.degreesToRadians(-70.0);

        // Angles and heights for when we have an algae
        public static final double SHOULDER_ALGAE_MAX_ANGLE = Units.degreesToRadians(66.70);
        public static final double WRIST_ALGAE_MAX_ANGLE = Units.degreesToRadians(32.96);
        // Elevator height where, when we have an algae, it is high enough that we don't need to limit the shoulder or wrist angle
        public static final double ELEVATOR_ALGAE_SAFE_HEIGHT = Units.feetToMeters(1.0);

        // Shoulder Safe variables
        public static final double SHOULDER_REEF_ANGLE_CORAL = Units.degreesToRadians(15.0);

        // Default heights and angles
        public static final double ELEVATOR_DEFAULT_POSITION = 0.0;
        public static final double SHOULDER_DEFAULT_ANGLE = Units.degreesToRadians(90.0);
        public static final double WRIST_DEFAULT_ANGLE = 0.0;

        // Max Positions when moving
        public static final double ELEVATOR_MAX_MOVING_HEIGHT = Units.feetToMeters(2.0);
        public static final double SHOULDER_MAX_MOVING_OFFSET = Units.degreesToRadians(10.0);
        public static final double WRIST_MOVING_OFFSET = Units.degreesToRadians(10.0);

    }

    // Arm buffers
    public static final double SHOULDER_BUFFER = Units.inchesToMeters(0.0);
    public static final double WRIST_BUFFER = Units.inchesToMeters(0.0);

    // Vision camera constants
    // Roll, pitch & yaw values are in radians
    // X, Y & Z translation values are in meters
    public static final String FRONT_RIGHT_CAMERA_NAME = "limelight-right";
    public static final double FRONT_RIGHT_CAMERA_ROLL = Units.degreesToRadians(0.0);
    public static final double FRONT_RIGHT_CAMERA_PITCH = Units.degreesToRadians(20.0);
    public static final double FRONT_RIGHT_CAMERA_YAW = Units.degreesToRadians(0.0);
    public static final double FRONT_RIGHT_CAMERA_X = Units.inchesToMeters(27.0 / 2 - 5.125);
    public static final double FRONT_RIGHT_CAMERA_Y = -Units.inchesToMeters(27.0 / 2.0 - 5.5);
    public static final double FRONT_RIGHT_CAMERA_Z = Units.inchesToMeters(8.50);

    public static final String FRONT_LEFT_CAMERA_NAME = "limelight-left";
    public static final double FRONT_LEFT_CAMERA_ROLL = Units.degreesToRadians(0.0);
    public static final double FRONT_LEFT_CAMERA_PITCH = Units.degreesToRadians(20.0);
    public static final double FRONT_LEFT_CAMERA_YAW = Units.degreesToRadians(0.0);
    public static final double FRONT_LEFT_CAMERA_X = Units.inchesToMeters(27.0 / 2 - 5.0);
    public static final double FRONT_LEFT_CAMERA_Y = Units.inchesToMeters(27.0 / 2.0 - 5.3);
    public static final double FRONT_LEFT_CAMERA_Z = Units.inchesToMeters(8.8125);

    public static final String BACK_CAMERA_NAME = "limelight-back";
    public static final double BACK_CAMERA_ROLL = Units.degreesToRadians(0.0);
    public static final double BACK_CAMERA_PITCH = Units.degreesToRadians(20.0);
    public static final double BACK_CAMERA_YAW = Units.degreesToRadians(180.0);
    public static final double BACK_CAMERA_X = -Units.inchesToMeters(27.0 / 2.0 - 5.125);
    public static final double BACK_CAMERA_Y = -Units.inchesToMeters(27.0 / 2.0 - 5.5625);
    public static final double BACK_CAMERA_Z = Units.inchesToMeters(8.375);
}