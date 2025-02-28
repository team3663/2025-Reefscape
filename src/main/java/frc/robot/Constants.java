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

    public static final double MK4I_STEER_REDUCTION = (150.0 / 7.0);
    public static final double MK4I_STEER_INERTIA = 0.00001;
    public static final double MK4I_STEER_FRICTION_VOLTAGE = 0.25;
    public static final double MK4_WHEEL_RADIUS = Units.inchesToMeters(1.93);
    public static final double WHEEL_COF = 1.2;
    public static final Slot0Configs MK4I_STEER_PID_CONSTANTS = new Slot0Configs()
            .withKP(50);

    public static final double DEBOUNCE_TIME = 0.5;
    public static final boolean IS_ANDYMARK= false;
    public static final double WELDED_REEF_POSE_Y_OFFSET = Units.inchesToMeters(158.5);
    public static final double ANDYMARK_REEF_POSE_Y_OFFSET = Units.inchesToMeters(158.3);
    public static final double BLUE_REEF_POSE_X_OFFSET = Units.inchesToMeters(176.75);
    public static final double RED_REEF_POSE_X_OFFSET = Units.inchesToMeters(514.125);

    public static final double X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF = Units.inchesToMeters(31.625);
    public static final double Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF = Units.inchesToMeters(6.5);
    public static final Transform2d ROBOT_REEF_OFFSET = new Transform2d(Units.inchesToMeters(18.675), 0, Rotation2d.fromDegrees(180));
    public static final Transform2d ROBOT_CORAL_STATION_OFFSET = new Transform2d(Units.inchesToMeters(2), 0, Rotation2d.fromDegrees(0));


    public static final Pose2d BLUE_LEFT_FAR_SIDE_CORAL_STATION = new Pose2d(1.5771037340164185, Units.inchesToMeters(289.246214), Rotation2d.fromDegrees(306));
    public static final Pose2d BLUE_LEFT_NEAR_SIDE_CORAL_STATION = new Pose2d(0.5229208469390869, 6.728616237640381, Rotation2d.fromDegrees(306));
    public static final Pose2d BLUE_RIGHT_NEAR_SIDE_CORAL_STATION = new Pose2d(0.5559024810791016, 1.349269151687622, Rotation2d.fromDegrees(54));
    public static final Pose2d BLUE_RIGHT_FAR_SIDE_CORAL_STATION = new Pose2d(1.5771037340164185, 0.7049461603164673, Rotation2d.fromDegrees(54));

    public static final Pose2d RED_LEFT_FAR_SIDE_CORAL_STATION = new Pose2d(Units.inchesToMeters(628.78430181), 0.7049461603164673, Rotation2d.fromDegrees(126));
    public static final Pose2d RED_LEFT_NEAR_SIDE_CORAL_STATION = new Pose2d((17.548225 - 0.5229208469390869), 1.349269151687622, Rotation2d.fromDegrees(126));
    public static final Pose2d RED_RIGHT_NEAR_SIDE_CORAL_STATION = new Pose2d((17.548225 - 0.5559024810791016), 6.728616237640381, Rotation2d.fromDegrees(234));
    public static final Pose2d RED_RIGHT_FAR_SIDE_CORAL_STATION = new Pose2d(Units.inchesToMeters(628.78430181), Units.inchesToMeters(289.246214), Rotation2d.fromDegrees(234));


    public static BranchPositions RED_WELDED_BRANCH_POSITIONS = new BranchPositions(
            RED_REEF_POSE_X_OFFSET, WELDED_REEF_POSE_Y_OFFSET
    );
    public static BranchPositions BLUE_WELDED_BRANCH_POSITIONS = new BranchPositions(
            BLUE_REEF_POSE_X_OFFSET, WELDED_REEF_POSE_Y_OFFSET
    );
    public static BranchPositions RED_ANDYMARK_BRANCH_POSITIONS = new BranchPositions(
            RED_REEF_POSE_X_OFFSET, ANDYMARK_REEF_POSE_Y_OFFSET
    );
    public static BranchPositions BLUE_ANDYMARK_BRANCH_POSITIONS = new BranchPositions(
            BLUE_REEF_POSE_X_OFFSET, ANDYMARK_REEF_POSE_Y_OFFSET
    );

    public static class BranchPositions {
        public final Pose2d BRANCH_A;
        public final Pose2d BRANCH_B;
        public final Pose2d BRANCH_C;
        public final Pose2d BRANCH_D;
        public final Pose2d BRANCH_E;
        public final Pose2d BRANCH_F;
        public final Pose2d BRANCH_G;
        public final Pose2d BRANCH_H;
        public final Pose2d BRANCH_I;
        public final Pose2d BRANCH_J;
        public final Pose2d BRANCH_K;
        public final Pose2d BRANCH_L;

        public BranchPositions(double X_OFFSET, double Y_OFFSET) {
            BRANCH_A = getBranchPose(3, false, X_OFFSET, Y_OFFSET);
            BRANCH_B = getBranchPose(3, true, X_OFFSET, Y_OFFSET);
            BRANCH_C = getBranchPose(4, false, X_OFFSET, Y_OFFSET);
            BRANCH_D = getBranchPose(4, true, X_OFFSET, Y_OFFSET);
            BRANCH_E = getBranchPose(5, false, X_OFFSET, Y_OFFSET);
            BRANCH_F = getBranchPose(5, true, X_OFFSET, Y_OFFSET);
            BRANCH_G = getBranchPose(0, false, X_OFFSET, Y_OFFSET);
            BRANCH_H = getBranchPose(0, true, X_OFFSET, Y_OFFSET);
            BRANCH_I = getBranchPose(1, false, X_OFFSET, Y_OFFSET);
            BRANCH_J = getBranchPose(1, true, X_OFFSET, Y_OFFSET);
            BRANCH_K = getBranchPose(2, false, X_OFFSET, Y_OFFSET);
            BRANCH_L = getBranchPose(2, true, X_OFFSET, Y_OFFSET);

        }


    }

    public static final List<Pose2d> RED_WELDED_BRANCH_POSES = List.of(RED_WELDED_BRANCH_POSITIONS.BRANCH_A, RED_WELDED_BRANCH_POSITIONS.BRANCH_B,
            RED_WELDED_BRANCH_POSITIONS.BRANCH_C,RED_WELDED_BRANCH_POSITIONS.BRANCH_D,
            RED_WELDED_BRANCH_POSITIONS.BRANCH_E,RED_WELDED_BRANCH_POSITIONS.BRANCH_F,
            RED_WELDED_BRANCH_POSITIONS.BRANCH_G, RED_WELDED_BRANCH_POSITIONS.BRANCH_H,
            RED_WELDED_BRANCH_POSITIONS.BRANCH_I,RED_WELDED_BRANCH_POSITIONS.BRANCH_J,
            RED_WELDED_BRANCH_POSITIONS.BRANCH_K,RED_WELDED_BRANCH_POSITIONS.BRANCH_L
    );
    public static final List<Pose2d> BLUE_WELDED_BRANCH_POSES = List.of(BLUE_WELDED_BRANCH_POSITIONS.BRANCH_A, BLUE_WELDED_BRANCH_POSITIONS.BRANCH_B,
            BLUE_WELDED_BRANCH_POSITIONS.BRANCH_C,BLUE_WELDED_BRANCH_POSITIONS.BRANCH_D,
            BLUE_WELDED_BRANCH_POSITIONS.BRANCH_E,BLUE_WELDED_BRANCH_POSITIONS.BRANCH_F,
            BLUE_WELDED_BRANCH_POSITIONS.BRANCH_G, BLUE_WELDED_BRANCH_POSITIONS.BRANCH_H,
            BLUE_WELDED_BRANCH_POSITIONS.BRANCH_I,BLUE_WELDED_BRANCH_POSITIONS.BRANCH_J,
            BLUE_WELDED_BRANCH_POSITIONS.BRANCH_K,BLUE_WELDED_BRANCH_POSITIONS.BRANCH_L
    );



    public static final List<Pose2d> RED_ANDYMARK_BRANCH_POSES = List.of(RED_ANDYMARK_BRANCH_POSITIONS.BRANCH_A, RED_ANDYMARK_BRANCH_POSITIONS.BRANCH_B,
            RED_ANDYMARK_BRANCH_POSITIONS.BRANCH_C,RED_ANDYMARK_BRANCH_POSITIONS.BRANCH_D,
            RED_ANDYMARK_BRANCH_POSITIONS.BRANCH_E,RED_ANDYMARK_BRANCH_POSITIONS.BRANCH_F,
            RED_ANDYMARK_BRANCH_POSITIONS.BRANCH_G, RED_ANDYMARK_BRANCH_POSITIONS.BRANCH_H,
            RED_ANDYMARK_BRANCH_POSITIONS.BRANCH_I,RED_ANDYMARK_BRANCH_POSITIONS.BRANCH_J,
            RED_ANDYMARK_BRANCH_POSITIONS.BRANCH_K,RED_ANDYMARK_BRANCH_POSITIONS.BRANCH_L
    );
    public static final List<Pose2d> BLUE_ANDYMARK_BRANCH_POSES = List.of(BLUE_ANDYMARK_BRANCH_POSITIONS.BRANCH_A, BLUE_ANDYMARK_BRANCH_POSITIONS.BRANCH_B,
            BLUE_ANDYMARK_BRANCH_POSITIONS.BRANCH_C,BLUE_ANDYMARK_BRANCH_POSITIONS.BRANCH_D,
            BLUE_ANDYMARK_BRANCH_POSITIONS.BRANCH_E,BLUE_ANDYMARK_BRANCH_POSITIONS.BRANCH_F,
            BLUE_ANDYMARK_BRANCH_POSITIONS.BRANCH_G, BLUE_ANDYMARK_BRANCH_POSITIONS.BRANCH_H,
            BLUE_ANDYMARK_BRANCH_POSITIONS.BRANCH_I,BLUE_ANDYMARK_BRANCH_POSITIONS.BRANCH_J,
            BLUE_ANDYMARK_BRANCH_POSITIONS.BRANCH_K,BLUE_ANDYMARK_BRANCH_POSITIONS.BRANCH_L
    );


    public static final List<Pose2d> BLUE_CORAL_STATION_POSES = List.of(BLUE_LEFT_NEAR_SIDE_CORAL_STATION, BLUE_LEFT_FAR_SIDE_CORAL_STATION,
            BLUE_RIGHT_NEAR_SIDE_CORAL_STATION, BLUE_RIGHT_FAR_SIDE_CORAL_STATION);

    public static final List<Pose2d> RED_CORAL_STATION_POSES = List.of(RED_LEFT_NEAR_SIDE_CORAL_STATION, RED_LEFT_FAR_SIDE_CORAL_STATION,
            RED_RIGHT_NEAR_SIDE_CORAL_STATION, RED_RIGHT_FAR_SIDE_CORAL_STATION);

    private static Pose2d getBranchPose(int number, boolean adding, double xOffset, double yOffset) {
        if (adding) {
            return new Pose2d(Math.hypot(X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF, Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF) *
                    Math.cos(((Math.PI / 3) * number) + Math.asin(Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF / X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF)) + xOffset,
                    Math.hypot(X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF, Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF) * Math.sin(((Math.PI / 3) * number) +
                            Math.asin(Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF / X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF)) + yOffset,
                    Rotation2d.fromRadians((Math.PI / 3) * number));
        } else {
            return new Pose2d(Math.hypot(X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF, Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF) *
                    Math.cos(((Math.PI / 3) * number) - Math.asin(Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF / X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF)) + xOffset,
                    Math.hypot(X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF, Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF) * Math.sin((Math.PI / 3) * number -
                            Math.asin(Y_BRANCH_DISTANCE_FROM_CENTER_OF_REEF / X_BRANCH_DISTANCE_FROM_CENTER_OF_REEF)) + yOffset,
                    Rotation2d.fromRadians((Math.PI / 3) * number));
        }
    }

    public static class ArmPositions {
        // Coral Station
        public static final double CORAL_STATION_ELEVATOR_HEIGHT = Units.inchesToMeters(6.811);
        public static final double CORAL_STATION_SHOULDER_ANGLE = Units.degreesToRadians(137.0);
        public static final double CORAL_STATION_WRIST_ANGLE = Units.degreesToRadians(-70.0);

        // Shoulder Safe variables
        public static final double SHOULDER_SAFE_ANGLE = Units.degreesToRadians(90.0);
        public static final double SHOULDER_SAFE_THRESHOLD = Units.degreesToRadians(25.0);
        public static final double SHOULDER_SAFE_BUFFER = Units.degreesToRadians(3.0);

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
    public static final String FRONT_CAMERA_NAME = "limelight-front";
    public static final double FRONT_CAMERA_ROLL = Units.degreesToRadians(0.0);
    public static final double FRONT_CAMERA_PITCH = Units.degreesToRadians(25.0);
    public static final double FRONT_CAMERA_YAW = Units.degreesToRadians(0.0);
    public static final double FRONT_CAMERA_X = Units.inchesToMeters(27.0 / 2 - 5.375);
    public static final double FRONT_CAMERA_Y = -Units.inchesToMeters(27.0 / 2.0 - 5.5);
    public static final double FRONT_CAMERA_Z = Units.inchesToMeters(8.625);

    public static final String BACK_CAMERA_NAME = "limelight-back";
    public static final double BACK_CAMERA_ROLL = Units.degreesToRadians(0.0);
    public static final double BACK_CAMERA_PITCH = Units.degreesToRadians(25.0);
    public static final double BACK_CAMERA_YAW = Units.degreesToRadians(180.0);
    public static final double BACK_CAMERA_X = -Units.inchesToMeters(27.0 / 2.0 - 5.375);
    public static final double BACK_CAMERA_Y = -Units.inchesToMeters(27.0 / 2.0 - 5.5);
    public static final double BACK_CAMERA_Z = Units.inchesToMeters(8.625);
}